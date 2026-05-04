#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BTN       13
#define LED       12
#define DEBOUNCE  200000
#define TEMPO_ON  10000000
#define HOLD_TIME 2000000

static QueueHandle_t fila;
static int led_ligado = 0;
static int64_t timer_led = 0;
static int64_t press_time = 0;

static void IRAM_ATTR isr_btn(void *arg) {
    int64_t t = esp_timer_get_time();
    xQueueSendFromISR(fila, &t, NULL);
}

void app_main(void) 
{
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BTN, GPIO_INTR_ANYEDGE);

    fila = xQueueCreate(10, sizeof(int64_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN, isr_btn, NULL);

    int64_t tempo;
    int btn_nivel, btn_anterior = 1;

    while (1) {
        // Timeout curto sempre, pra verificar o timer do LED
        if (xQueueReceive(fila, &tempo, pdMS_TO_TICKS(50)) == pdTRUE) {
            btn_nivel = gpio_get_level(BTN);

            if (btn_nivel == 0 && btn_anterior == 1) {
                press_time = tempo;
            }

            if (btn_nivel == 1 && btn_anterior == 0) {
                int64_t duracao = tempo - press_time;

                if (duracao < DEBOUNCE) {
                    btn_anterior = btn_nivel;
                    // Não usa continue, deixa cair pra verificação do timer
                }
                else if (duracao >= HOLD_TIME) {
                    led_ligado = 0;
                    gpio_set_level(LED, 0);
                }
                else {
                    led_ligado = 1;
                    timer_led = tempo;  // <- ATUALIZA SEMPRE que há clique válido
                    gpio_set_level(LED, 1);
                }
            }
            btn_anterior = btn_nivel;
        }

        // Verificação do timer — roda a cada loop, SEMPRE
        if (led_ligado && (esp_timer_get_time() - timer_led >= TEMPO_ON)) {
            led_ligado = 0;
            gpio_set_level(LED, 0);
        }
    }
}