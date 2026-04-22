#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BTN 13
#define LED 12
#define DEBOUNCE 200000
#define TIME_ON 10000000 //10s

int64_t last_change_time = 0;
int last_btn_val = 1;
int led_state = 0;
int64_t led_on_time = 0;

void app_main() {

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(BTN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BTN, GPIO_PULLUP_ONLY);

  while (1) {

    int btn_val = gpio_get_level(BTN);
    int64_t time = esp_timer_get_time();

    if (btn_val == 0 && last_btn_val == 1 && (time - last_change_time > DEBOUNCE)) {

      led_state = !led_state; //tem q ser toggle
      gpio_set_level(LED, led_state);
      last_change_time = time;
      if (led_state == 1) {
        led_on_time = time;
      }
    }
    if (led_state == 1 && (time - led_on_time > TIME_ON)) {
      led_state = 0;
      gpio_set_level(LED, 0);
    }

    last_btn_val = btn_val;
  }
}
