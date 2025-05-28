#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "init_i2c.h"
#include "esp_vfs_dev.h"

#define BTN_A GPIO_NUM_10
#define BTN_B GPIO_NUM_14

#define LED_1 GPIO_NUM_18
#define LED_2 GPIO_NUM_17
#define LED_3 GPIO_NUM_16
#define LED_4 GPIO_NUM_15
#define LED_5 GPIO_NUM_12

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 5000

#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000

int counter = 0;
int count_factor = 1;

#define IN_BIT_MASK ((1UL << BTN_A) | (1UL << BTN_B))
#define OUT_BIT_MASK ((1UL << LED_1) | (1UL << LED_2) | (1UL << LED_3) | (1UL << LED_4))

volatile int64_t last_press_a = 0;
volatile int64_t last_press_b = 0;
volatile bool update_display = false;

#define DEBOUNCE_US 150000

#define LOW  0
#define HIGH 1

// prototypes
void update_led(int num);
void update_all(int num);
void IRAM_ATTR button_a_ISR(void* arg);
void IRAM_ATTR button_b_ISR(void* arg);
void set_pwm_duty(uint32_t duty);
void print_num(lcd_i2c_handle_t* lcd, int num);

lcd_i2c_handle_t lcd = {
  .address = 0x27,
  .num = I2C_NUM_1,
  .backlight = 1,
  .size = DISPLAY_16X02
};

void app_main() {
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

  lcd_i2c_init(&lcd);
  print_num(&lcd, 0);

  while (1) {
    if (update_display) {
      print_num(&lcd, counter);
      printf("Contador: %d\n", counter);
      update_display = false;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void update_led(int num) {
  gpio_set_level(LED_1, ((counter >> 0) & 0x01) ? HIGH : LOW);
  gpio_set_level(LED_2, ((counter >> 1) & 0x01) ? HIGH : LOW);
  gpio_set_level(LED_3, ((counter >> 2) & 0x01) ? HIGH : LOW);
  gpio_set_level(LED_4, ((counter >> 3) & 0x01) ? HIGH : LOW);
}

void update_all(int num) {
  update_led(num);
  set_pwm_duty((num * 1023) / 15);
  update_display = true;
}

void IRAM_ATTR button_a_ISR(void* arg) {
  int64_t now = esp_timer_get_time();
  if (now - last_press_a > DEBOUNCE_US) {
    last_press_a = now;
    counter += count_factor;
    if (counter > 15) counter = 0;
    update_all(counter);
  }
}

void IRAM_ATTR button_b_ISR(void* arg) {
  int64_t now = esp_timer_get_time();
  if (now - last_press_b > DEBOUNCE_US) {
    last_press_b = now;
    counter -= count_factor;
    if (counter <= 0) counter = 0;
    update_all(counter);
  }
}

void set_pwm_duty(uint32_t duty) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void print_num(lcd_i2c_handle_t* lcd, int num) {
  lcd_i2c_cursor_set(lcd, 0, 0);
  lcd_i2c_print(lcd, "Hex: 0x%X", num);

  lcd_i2c_cursor_set(lcd, 0, 1);
  if (num < 10)
    lcd_i2c_print(lcd, "Decimal: 0%d", num);
  else
    lcd_i2c_print(lcd, "Decimal: %d", num);
}
