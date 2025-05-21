#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#define BTN_A GPIO_NUM_10
#define BTN_B GPIO_NUM_14

#define LED_1 GPIO_NUM_18
#define LED_2 GPIO_NUM_17
#define LED_3 GPIO_NUM_16
#define LED_4 GPIO_NUM_15

int counter = 0;
int count_factor = 1;

#define IN_BIT_MASK ((1UL << BTN_A) | (1UL << BTN_B))
#define OUT_BIT_MASK ((1UL << LED_1) | (1UL << LED_2) | (1UL << LED_3) | (1UL << LED_4))

volatile int64_t last_press_a = 0;
volatile int64_t last_press_b = 0;

#define DEBOUNCE_US 150000

#define LOW  0
#define HIGH 1

void IRAM_ATTR button_a_ISR(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_a > DEBOUNCE_US) {
        last_press_a = now;
        counter += count_factor;
        if (counter > 15) counter -= 16;

        gpio_set_level(LED_1, ((counter >> 0) & 0x01) ? HIGH : LOW);
        gpio_set_level(LED_2, ((counter >> 1) & 0x01) ? HIGH : LOW);
        gpio_set_level(LED_3, ((counter >> 2) & 0x01) ? HIGH : LOW);
        gpio_set_level(LED_4, ((counter >> 3) & 0x01) ? HIGH : LOW);
    }
}

void IRAM_ATTR button_b_ISR(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_b > DEBOUNCE_US) {
        last_press_b = now;
        count_factor = (count_factor == 1) ? 2 : 1;
    }
}


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
    .intr_type = GPIO_INTR_LOW_LEVEL
  };
  gpio_config(&config_in);

  gpio_install_isr_service(1);
  gpio_isr_handler_add(BTN_A, button_a_ISR, NULL);
  gpio_isr_handler_add(BTN_B, button_b_ISR, NULL);


  while (1) vTaskDelay(1000);
}
