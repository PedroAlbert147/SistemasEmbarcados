#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/adc_types.h"

static const char *debug = "MAZE"; //para os prints

//entradas e saidas:-----------------------------
#define LED_PIN 4
#define JOYSTICK_H_CHA ADC_CHANNEL_0 //gpio1 hor
#define JOYSTICK_V_CHA ADC_CHANNEL_1 //gpio2 ver
#define SERVO_X_PIN 18 //eixo horizontal
#define SERVO_Y_PIN 19 //eixo vertical
#define SERVO_MIN_PULSE 1000 //valor mínimo para reconhecimento do servo
#define SERVO_MAX_PULSE 2000 //valor máximo para servo
#define SERVO_MAX_ANGLE 180
#define PWM_FREQ 50 //o servo pode operar entre 40Hz a 400Hz
#define ADC_MAX 4095 //teto do ADC
//---------------------------------------------
static adc_oneshot_unit_handle_t adc_handle;
//valores iniciais:
static volatile int joystick_x = 2048;
static volatile int joystick_y = 2048;
static volatile int servo_x_angle = 90;
static volatile int servo_y_angle = 90;
//-----------------

//inicializando ADC
static void adc_init(void){
  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  adc_oneshot_new_unit(&init_cfg, &adc_handle);

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };

  adc_oneshot_config_channel(adc_handle, JOYSTICK_H_CHA, &chan_cfg);
  adc_oneshot_config_channel(adc_handle, JOYSTICK_V_CHA, &chan_cfg);
}

//leitura ADC
static int adc_read(adc_channel_t chan){
  int raw;
  adc_oneshot_read(adc_handle, chan, &raw);
  return raw;
}




//PWM
static void pwm_init(void){
  ledc_timer_config_t timer = {//timer pwm
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    //.duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);
  ledc_channel_config_t ch_x = {
    .gpio_num = SERVO_X_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ch_x);

  ledc_channel_config_t ch_y = {
    .gpio_num = SERVO_Y_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ch_y);
}

//angulo para PWM
static uint32_t angle_to_duty(int angle){
  int pulse = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)/SERVO_MAX_ANGLE);
  return (pulse*4096)/20000; //200000 eh periodo para 50Hz
}

//angulo servo X
static void set_servo_x(int angle){
  if (angle < 0){ 
    angle = 0;
  }else if (angle > 180){ 
    angle = 180;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty(angle));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

//angulo servo y
static void set_servo_y(int angle){
  if (angle < 0){ 
    angle = 0;
  }else if (angle > 180){ 
    angle = 180;
  }
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, angle_to_duty(angle));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

//task 1: le joystick
static void task_joystick(void *arg){
  while (1){
    joystick_x = adc_read(JOYSTICK_H_CHA);
    joystick_y = adc_read(JOYSTICK_V_CHA);

    //coloca valor raw do ADC para angulo
    servo_x_angle = (joystick_x * 180) / ADC_MAX;
    servo_y_angle = (joystick_y * 180) / ADC_MAX;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//task 2: controla servos
static void task_servos(void *arg){
  while (1){
    set_servo_x(servo_x_angle);
    set_servo_y(servo_y_angle);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
//task 3: monitor serial
static void task_monitor(void *arg){
  while (1){
    uint32_t duty_x = angle_to_duty(servo_x_angle);
    uint32_t duty_y = angle_to_duty(servo_y_angle);
        
    ESP_LOGI(debug,"Vertical: raw=%d angle=%d° duty=%lu Horizontal: raw=%d angle=%d° duty=%lu\n",joystick_x, servo_x_angle, duty_x,joystick_y, servo_y_angle, duty_y);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task 3:
//static void task_monitor(void *arg)
//{
//    while (1) {
//        ESP_LOGI(TAG, "Joy X:%d Y:%d | Servo X:%d° Y:%d°",
//                 joystick_x, joystick_y,
//                 servo_x_angle, servo_y_angle);
//        vTaskDelay(pdMS_TO_TICKS(500));
//    }
//}

void app_main(void){

  adc_init();
  pwm_init();
  set_servo_x(90);
  set_servo_y(90);
  vTaskDelay(pdMS_TO_TICKS(5000));
  ESP_LOGI(debug, "ok");
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 1);

  xTaskCreate(task_joystick, "leitura_joystick", 2048, NULL, 2, NULL);
  xTaskCreate(task_servos, "control_servos", 2048, NULL, 3, NULL);
  xTaskCreate(task_monitor, "monitor", 2048, NULL, 1, NULL);
}
