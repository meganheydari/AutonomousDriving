/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#define SERVO_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

uint32_t distanceBack = 0;
uint32_t distanceFront = 0;

uint32_t currentAngle = 0;
uint32_t targetAngle = 0;
TaskHandle_t turnHandle = NULL;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static const adc_unit_t unit2 = ADC_UNIT_2;
static const adc_channel_t channel_f = ADC_CHANNEL_7;
static const adc_channel_t channel_b = ADC_CHANNEL_6;

#define DEFAULT_VREF    1100
uint32_t ultra_distance; //cm

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 32);    //Set GPIO 18 as PWM0A, to which servo motor is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 21);    //Set GPIO 14 as PWM0B, to which steering motor is connected
}

void init(void)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1600); // NEUTRAL signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2400); // HIGH signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 900);  // LOW signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1600); // NEUTRAL signal in microseconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);

}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void ultrasonic_f()
{
  //Configure ADC
  adc2_config_channel_atten((adc2_channel_t)channel_f, atten);

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit2, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
  //print_char_val_type(val_type);

  while (1) {
      int adc_reading = 0;
      int adc_temp = 0;
      //Multisampling
      for (int i = 0; i < 50; i++) {
          adc2_get_raw(channel_f, ADC_WIDTH_BIT_10,&adc_reading);
          adc_temp+=adc_reading;
      }
      adc_temp /= 50;
      uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_temp, adc_chars);
      //printf("Ultrasonic Distance: %.3fm\n", voltage / 6.4 * 2.54 / 100); //AN yields 6.4mV/in. for 3.3V
      distanceFront = (uint32_t)voltage / 6.4 * 2.54;
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void ultrasonic_b()
{
    //Configure ADC
    adc2_config_channel_atten((adc2_channel_t)channel_b, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit2, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
    //print_char_val_type(val_type);

    while (1) {
        int adc_reading = 0;
        int adc_temp = 0;
        //Multisampling
        for (int i = 0; i < 50; i++) {
            adc2_get_raw(channel_b, ADC_WIDTH_BIT_10,&adc_reading);
            adc_temp+=adc_reading;
        }
        adc_temp /= 50;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_temp, adc_chars);
        //printf("Ultrasonic Distance: %.3fm\n", voltage / 6.4 * 2.54 / 100); //AN yields 6.4mV/in. for 3.3V
        distanceBack = (uint32_t)voltage / 6.4 * 2.54;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ultrasonic()
{
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
    //print_char_val_type(val_type);

    while (1) {
        int adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 15; i++) {
            adc_reading+=adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= 15;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Ultrasonic Distance: %.3fm\n", voltage / 6.4 * 2.54 / 100); //AN yields 6.4mV/in. for 3.3V
        ultra_distance = (uint32_t)voltage / 6.4 * 2.54;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// task that performs the wheel turning incrementally to avoid harware strain.
void perfromTurn()
{
    uint32_t angle, count;

    while (targetAngle != currentAngle) {
        if (targetAngle > currentAngle){
            for (count = currentAngle; count <= targetAngle; count++) {
                angle = servo_per_degree_init(count);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);

            }
            currentAngle = targetAngle;
        }
        else {
            for (count = currentAngle; count > targetAngle; count--) {
                angle = servo_per_degree_init(count);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);

            }
            currentAngle = targetAngle;
        }
    }
    turnHandle = NULL;
    vTaskDelete(turnHandle);
}

// handles the creating and destroying of tasks that turn the wheels.
void turnHandler(int target)
{
    targetAngle = target;
    printf("Start turn thead. angle changing to: %d\n", targetAngle);

    if(turnHandle != NULL)
    {
        vTaskDelete(turnHandle);
        turnHandle = NULL;
    }

    xTaskCreate(perfromTurn, "perfromTurn", 4096, NULL, 5, &turnHandle);
    configASSERT(turnHandle);
}

void app_main(void)
{
    printf("Testing servo motor.......\n");
    init();
    xTaskCreate(ultrasonic, "ultrasonic", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(ultrasonic_f, "ultrasonic_f", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasonic_b, "ultrasonic_b", 4096, NULL, 5, NULL);


    uint32_t localAngle = 0;
    uint32_t integral = 0;
    uint32_t prev_distance = distanceFront;

    while(1) {

        if(ultra_distance < 50) {  //stop
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        }

        integral = integral + (distanceFront - prev_distance) /10 * 3;

        printf("Dfront: %d, Dback: %d, Ultra: %d\n", distanceFront, distanceBack, ultra_distance);
        printf("prev: %d, curr: %d\n", prev_distance, distanceFront);
        printf("Integral: %d\n", integral);

        prev_distance = distanceFront;
        localAngle = integral + (distanceFront - distanceBack)/10*30 + 45;

        if (localAngle < 0) {
            localAngle = 0;
        }
        if (localAngle > 90) {
            localAngle = 90;
        }

        printf("angle: %f\n", localAngle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(localAngle));

        vTaskDelay(100 / portTICK_RATE_MS);

    }

}
