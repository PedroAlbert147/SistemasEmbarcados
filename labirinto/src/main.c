#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "hal/adc_types.h"
#include "driver/i2c.h"
#include "math.h"

//entradas e saidas:-----------------------------
//#define LED_PIN 4
#define JOYSTICK_H_CHA ADC_CHANNEL_0 //gpio36 (VN) hor
#define JOYSTICK_V_CHA ADC_CHANNEL_3 //gpio39 (VP) ver
#define SERVO_X_PIN 18 //eixo horizontal
#define SERVO_Y_PIN 19 //eixo vertical
#define SERVO_MIN_PULSE 1000 //valor mínimo para reconhecimento do servo
#define SERVO_MAX_PULSE 2000 //valor máximo para servo
#define SERVO_MAX_ANGLE 180
#define PWM_FREQ 5000 //o servo pode operar entre 40Hz a 400Hz
#define ADC_MAX 4095 //teto do ADC
//---------------------------------------------
//MPU6050
#define I2C_MASTER_SCL_IO 22 //SCL
#define I2C_MASTER_SDA_IO 21 //SDA
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
//---------------------------------------------

// Variáveis globais MPU6050
static volatile float pitch = 0.0f;
static volatile float roll = 0.0f;

static adc_oneshot_unit_handle_t adc_handle;

//valores iniciais:
static volatile int joystick_x = 2048;
static volatile int joystick_y = 2048;
static volatile int servo_x_angle = 90;
static volatile int servo_y_angle = 90;

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
  ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
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

//angulo para duty do LED (0-4095 brilho total)
static uint32_t angle_to_duty(int angle){
  // Mapeia ângulo (0-180) para duty (0-4095)
  return (angle * 4095) / 180;
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

//i2c e mpu6050
static void i2c_mpu6050_init(void)
{
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

    uint8_t data[2] = {MPU6050_PWR_MGMT1, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, pdMS_TO_TICKS(1000));
}

//leituras mpu6050
static void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t raw_data[6];

    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, raw_data, 6, pdMS_TO_TICKS(1000));

    *ax = (raw_data[0] << 8) | raw_data[1];
    *ay = (raw_data[2] << 8) | raw_data[3];
    *az = (raw_data[4] << 8) | raw_data[5];
}

//task 1: leitura do joystick
static void task_joystick(void *arg){
  while (1){
    joystick_x = adc_read(JOYSTICK_H_CHA);
    joystick_y = adc_read(JOYSTICK_V_CHA);

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
    // Format: "DATA:pitch,roll"
    printf("DATA:%.2f,%.2f\n", pitch, roll);
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 samples/second
  }
}


//static void task_monitor(void *arg){
//  while (1){
//    printf("--- MAZE DEBUG ---\n");
//    printf("Joystick X:%d Y:%d\n", joystick_x, joystick_y);
//    printf("Servo X:%d° Y:%d°\n", servo_x_angle, servo_y_angle);
//    printf("Pitch: %.1f° | Roll: %.1f°\n", pitch, roll);
//    printf("------------------\n");
//    vTaskDelay(pdMS_TO_TICKS(500));
//  }
//}

//task 4: mpu6050
static void task_mpu6050(void *arg)
{
    int16_t ax, ay, az;

    while (1) {
        mpu6050_read_accel(&ax, &ay, &az);

        // Convert raw to g-force (assuming ±2g default range, 16384 LSB/g)
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;

        // Correct pitch and roll from accelerometer
        pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;
        roll  = atan2f(ay_g, az_g) * 180.0f / M_PI;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//task 4: mpu6050
//static void task_mpu6050(void *arg)
//{
//    int16_t ax, ay, az;
//
//    while (1) {
//        mpu6050_read_accel(&ax, &ay, &az);
//
//        printf("MPU_RAW: ax=%d, ay=%d, az=%d\n", ax, ay, az);
//
//        pitch = atan2f(ay, az) * 180.0f / M_PI;
//        roll  = atan2f(ax, az) * 180.0f / M_PI;
//
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//}

void app_main(void){

  adc_init();
  pwm_init();
  set_servo_x(90);
  set_servo_y(90);
  i2c_mpu6050_init();

  printf("Sistema pronto!\n");

  vTaskDelay(pdMS_TO_TICKS(5000));

  //gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  //gpio_set_level(LED_PIN, 1);

  xTaskCreate(task_joystick, "leitura_joystick", 2048, NULL, 2, NULL);
  xTaskCreate(task_servos, "control_servos", 2048, NULL, 3, NULL);
  xTaskCreate(task_monitor, "monitor", 4096, NULL, 1, NULL);
  xTaskCreate(task_mpu6050, "mpu6050", 4096, NULL, 4, NULL);
}