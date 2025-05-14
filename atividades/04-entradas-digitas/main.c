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

#define LOW 0
#define HIGH 1

#define DEBOUNCE_US 50000

void app_main() {
  gpio_config_t config_out = {
    .pin_bit_mask = OUT_BIT_MASK,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&config_out);

  gpio_reset_pin(BTN_A);
  gpio_reset_pin(BTN_B);

  gpio_set_direction(BTN_A, GPIO_MODE_INPUT);
  gpio_set_direction(BTN_B, GPIO_MODE_INPUT);


  int64_t bounce_time_a = esp_timer_get_time(); 
  int64_t bounce_time_b = esp_timer_get_time();

  while(1) {
    int64_t now = esp_timer_get_time();

    // if (counter == 15) counter = 0;
    if (!gpio_get_level(BTN_A) && now - bounce_time_a > DEBOUNCE_US) {
      bounce_time_a = now;
      counter += count_factor;
      if (counter > 15) {
        counter -= 16;
      }
    }
    if (!gpio_get_level(BTN_B) && now - bounce_time_b > DEBOUNCE_US) {
      bounce_time_b = now;
      count_factor = count_factor == 1 ? 2 : 1; 
    }

    gpio_set_level(LED_1, ((counter >> 0) & 0b01) ? HIGH : LOW);
    gpio_set_level(LED_2, ((counter >> 1) & 0b01) ? HIGH : LOW);
    gpio_set_level(LED_3, ((counter >> 2) & 0b01) ? HIGH : LOW);
    gpio_set_level(LED_4, ((counter >> 3) & 0b01) ? HIGH : LOW);
  }

} 
