#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "init_i2c.h"
#include "esp_vfs_dev.h"
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <stdbool.h>

// queues
QueueHandle_t temperature_queue;
QueueHandle_t edge_queue;

// isr control priority
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// task handlers
TaskHandle_t temp_task_handle;

// tasks
void temperature_task(void* args);
void alarm_task(void* args);
void lcd_task(void* args);
void sd_card_task(void* args);

// gpio
#define BTN_A GPIO_NUM_10
#define BTN_B GPIO_NUM_9
#define TEMP_PIN GPIO_NUM_47

#define IN_BIT_MASK ((1UL << BTN_A) | (1UL << BTN_B) | (1ULL << TEMP_PIN))

volatile int64_t last_press_a = 0;
volatile int64_t last_press_b = 0;

#define DEBOUNCE_US 150000

// leds
#define LED_1 GPIO_NUM_18
#define LED_2 GPIO_NUM_17
#define LED_3 GPIO_NUM_16
#define LED_4 GPIO_NUM_6
#define LED_5 GPIO_NUM_11

#define OUT_BIT_MASK ((1UL << LED_1) | (1UL << LED_2) | (1UL << LED_3) | (1UL << LED_4))

// ledc
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 512
#define LEDC_DUTY_CICLE 512

// i2c
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 40
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000

// adc
#define ADC_VCC_REF          3.3
#define ADC_RESOLUTION       4095
//// ntc
#define NTC_RESISTENCE_FIXED 10000
#define NTC_BETA_CONSTANT    3950
#define NTC_CHANNEL          ADC1_CHANNEL_6

// sd_card
#define SD_MNT_POINT "/files"
#define MISO GPIO_NUM_12
#define MOSI GPIO_NUM_13
#define SCLK GPIO_NUM_14
#define CS   GPIO_NUM_15

// app defines
#define LOW  0
#define HIGH 1

const int default_edge = 25; // defaults to 24
int edge_temp = default_edge;
volatile bool update_display = false;

// prototypes
//// leds
void update_led(int diff);

//// gpio
void IRAM_ATTR button_a_ISR(void* arg);
void IRAM_ATTR button_b_ISR(void* arg);

//// adc
void set_pwm_duty(uint32_t duty);

//// sd_card
void write_file(char* path, char* data);
void read_file(char* path);

//// setups
void gpio_setup();
void ledc_setup();
void i2c_setup();
void adc_setup();
void sd_card_setup();

//// core
void print_temperature(lcd_i2c_handle_t* lcd, int current_temp, int edge_temp);
int calculate_temperature();
void play_alarm();
void stop_alarm();
void clear_display(lcd_i2c_handle_t * lcd);
void fmt_ntc_temperature(char* temp, int size, int current_temp);
void fmt_edge_temperature(char* temp, int size, int edge_temp);

lcd_i2c_handle_t lcd = {
  .address = 0x27,
  .num = I2C_NUM_1,
  .backlight = 1,
  .size = DISPLAY_16X02
};

void app_main() {
  gpio_setup();
  ledc_setup();
  i2c_setup();
  adc_setup();
  sd_card_setup();

  lcd_i2c_init(&lcd);

  temperature_queue = xQueueCreate(1, sizeof(int));
  edge_queue = xQueueCreate(1, sizeof(int));

  xTaskCreate(
    temperature_task,
    "temperature_task",
    2048,
    NULL,
    5,
    &temp_task_handle
  );

  xTaskCreate(
    alarm_task,
    "alarm_task",
    2048,
    NULL,
    4,
    NULL
  );

  xTaskCreate(
    lcd_task,
    "lcd_task",
    2048,
    NULL,
    4,
    NULL
  );

  xTaskCreate(
    sd_card_task,
    "sd_card_task",
    4096,
    NULL,
    2,
    NULL
  );

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

// tasks impl
void temperature_task(void* args) {
  int temperature = 0;

  while (true) {
    temperature = calculate_temperature();

    xQueueOverwrite(temperature_queue, &temperature);
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void alarm_task(void* args) {
  int difference = 0;
  int temperature = 0;
  int edge = default_edge;

  while (true) {
    if (xQueuePeek(temperature_queue, &temperature, portMAX_DELAY) | xQueuePeekFromISR(edge_queue, &edge)) {
      difference = edge - temperature;
      if (difference < 0) {
        play_alarm();
      } else {
        stop_alarm();
      }

      update_led(difference);
      vTaskDelay(pdMS_TO_TICKS(15));
    }
  }
}


void lcd_task(void* args) {
  int temperature = 0;
  int edge = default_edge;

  while (true) {
    if (xQueuePeek(temperature_queue, &temperature, portMAX_DELAY) | xQueuePeekFromISR(edge_queue, &edge)) {
      print_temperature(&lcd, temperature, edge);
      vTaskDelay(pdMS_TO_TICKS(15));
    }
  }
}

void sd_card_task(void* args) {
  char* path = "/files/readings.txt";
  int temperature = 0;
  int buf_size = 16;
  char data[buf_size];

  while (true) {
    if (xQueuePeek(temperature_queue, &temperature, portMAX_DELAY)) {
      fmt_ntc_temperature(data, buf_size, temperature);
      write_file(path, data);
      read_file(path);
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void play_alarm() {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_CICLE); 
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void stop_alarm() {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void update_led(int diff) {
  if (diff <= 0) {
    gpio_set_level(LED_1, HIGH);
    gpio_set_level(LED_2, HIGH);
    gpio_set_level(LED_3, HIGH);
    gpio_set_level(LED_4, HIGH);

    vTaskDelay(pdMS_TO_TICKS(250));

    gpio_set_level(LED_1, LOW);
    gpio_set_level(LED_2, LOW);
    gpio_set_level(LED_3, LOW);
    gpio_set_level(LED_4, LOW);

    vTaskDelay(pdMS_TO_TICKS(250));
  } else {
    gpio_set_level(LED_1, (diff <= 20 ? HIGH : LOW));
    gpio_set_level(LED_2, (diff <= 15 ? HIGH : LOW));
    gpio_set_level(LED_3, (diff <= 10 ? HIGH : LOW));
    gpio_set_level(LED_4, (diff <= 2 ? HIGH : LOW));
  }
}

void IRAM_ATTR button_a_ISR(void* arg) {
  int64_t now = esp_timer_get_time();
  if (now - last_press_a > DEBOUNCE_US) {
    last_press_a = now;
    edge_temp += 5;
    xQueueOverwriteFromISR(edge_queue, &edge_temp, &xHigherPriorityTaskWoken);
  }
}

void IRAM_ATTR button_b_ISR(void* arg) {
  int64_t now = esp_timer_get_time();
  if (now - last_press_b > DEBOUNCE_US) {
    last_press_b = now;
    edge_temp -= 5;
    xQueueOverwriteFromISR(edge_queue, &edge_temp, &xHigherPriorityTaskWoken);
  }
}

void set_pwm_duty(uint32_t duty) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void print_temperature(lcd_i2c_handle_t* lcd, int current_temp, int edge_temp) {
  int buf_size = 16;
  char line_1[buf_size];
  char line_2[buf_size];

  fmt_ntc_temperature(line_1, buf_size, current_temp);
  fmt_edge_temperature(line_2, buf_size, edge_temp);

  lcd_i2c_cursor_set(lcd, 0, 0);
  lcd_i2c_print(lcd, line_1);
  lcd_i2c_cursor_set(lcd, 0, 1);
  lcd_i2c_print(lcd, line_2);
}

void clear_display(lcd_i2c_handle_t * lcd) {
  lcd_i2c_write(lcd, 0, CLEAR_DISPLAY);
}

int calculate_temperature() {
  int raw_ntc = adc1_get_raw(NTC_CHANNEL);

  if (raw_ntc <= 0 || raw_ntc >= ADC_RESOLUTION) {
    printf("Invalid ADC value: %d\n", raw_ntc);
    return -1000;
  }

  float ntc_tension = (raw_ntc * ADC_VCC_REF) / ADC_RESOLUTION;
  float ntc_resistence = (ntc_tension * NTC_RESISTENCE_FIXED) / (ADC_VCC_REF - ntc_tension);

  float temp_kelvin = 1.0 / (log(ntc_resistence / 10000.0) / NTC_BETA_CONSTANT + 1.0 / 298.15);
  float temp_celsius = temp_kelvin - 273.15;

  return (int)temp_celsius;
}

void fmt_ntc_temperature(char* temp, int size, int current_temp) {
  snprintf(temp, size, current_temp > 0 ? "NTC:  %d " : "NTC: %d ", current_temp);
}

void fmt_edge_temperature(char* temp, int size, int edge_temp) {
  snprintf(temp, size, "Edge: %d ", edge_temp);
}

void write_file(char* path, char* data) {
  char* EOL = "\n";

  FILE* file = fopen(path, "w+a");
  if (file == NULL) {
    printf("Falha ao criar o arquivo");
    return;
  }

  fprintf(file, data);
  fprintf(file, EOL);
  fclose(file);
}

void read_file(char* path) {
  FILE* file = fopen(path, "r");
  if (file == NULL) {
    printf("Falha o ler o arquivo");
    return;
  }

  char content[512];
  fgets(content, sizeof(content), file);
  fclose(file);

  printf("%s", content);
}

// setup
void gpio_setup() {
  gpio_config_t config_out = {
    .pin_bit_mask = OUT_BIT_MASK,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&config_out);

  gpio_config_t config_in = {
    .pin_bit_mask = IN_BIT_MASK,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE
  };
  gpio_config(&config_in);

  gpio_install_isr_service(1);
  gpio_isr_handler_add(BTN_A, button_a_ISR, NULL);
  gpio_isr_handler_add(BTN_B, button_b_ISR, NULL);
}

void ledc_setup() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .timer_num = LEDC_TIMER,
    .duty_resolution = LEDC_DUTY_RES,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = LED_5,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);
}

void i2c_setup() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void adc_setup() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(NTC_CHANNEL, ADC_ATTEN_DB_11);
}

void sd_card_setup() {
  esp_vfs_fat_sdmmc_mount_config_t mnt_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_card_t *sd_card;

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();

  spi_bus_config_t spi_io = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000
  };

  esp_err_t init_status = spi_bus_initialize(host.slot, &spi_io, SDSPI_DEFAULT_DMA);

  if (init_status != ESP_OK) {
    printf("Falha ao inicializar o spi");
    return;
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = CS;
  slot_config.host_id = host.slot;

  init_status = esp_vfs_fat_sdspi_mount(SD_MNT_POINT, &host, &slot_config, &mnt_config, &sd_card);

  if (init_status != ESP_OK) {
    if (init_status == ESP_FAIL) {
      printf("Falha ao montar o sistema de arquivos");
    } else {
      printf("Falha ao inicializar o sd card");
      return;
    }
  }

  sdmmc_card_print_info(stdout, sd_card);
}
