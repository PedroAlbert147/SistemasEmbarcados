#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BTN 13
#define LED 12
#define DEBOUNCE 200000       //200ms
#define TIME_ON 10000000      //10s

int64_t last_signal_change = 0;  
int current_stable_state = 1;    
int last_stable_state = 1;       // ultimo debounce state
int led_state = 0;
int64_t led_on_time = 0;

void app_main() {

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(BTN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BTN, GPIO_PULLUP_ONLY);

  while (1) {

    int raw_btn = gpio_get_level(BTN);
    int64_t time = esp_timer_get_time();

    
    static int last_raw_btn = 1;
    if (raw_btn != last_raw_btn) {
      last_signal_change = time;
      last_raw_btn = raw_btn;
    }

    
    if ((time - last_signal_change) >= DEBOUNCE) {
      current_stable_state = raw_btn;
    }

    
    if (current_stable_state == 0 && last_stable_state == 1) {
      
      led_state = !led_state; //eh toggle
      gpio_set_level(LED, led_state);
      if (led_state == 1) {
        led_on_time = time;
      }
    }

    last_stable_state = current_stable_state;

    //contagem regressiva
    if (led_state == 1 && (time - led_on_time > TIME_ON)) {
      led_state = 0;
      gpio_set_level(LED, 0);
    }
  }
}