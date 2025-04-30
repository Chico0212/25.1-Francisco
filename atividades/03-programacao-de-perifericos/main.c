#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#define LED_1_IO 15
#define LED_2_IO 18
#define IO_BIT_MASK ((1ULL << LED_1_IO) | (1ULL << LED_2_IO))

void app_main() {
  gpio_config_t config = {
    .pin_bit_mask = IO_BIT_MASK,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  gpio_config(&config);

  int led_1_state = 0;
  int led_2_state = 0;

  int cnt = 0;
  while (true) {
    led_1_state = ~led_1_state;
    gpio_set_level(LED_1_IO, led_1_state);
    cnt++;
    if (cnt == 5){
      led_2_state = ~led_2_state;
      gpio_set_level(LED_2_IO, led_2_state);
      cnt = 0;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
