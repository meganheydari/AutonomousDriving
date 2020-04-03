/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   1          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     // A2
static const adc_channel_t channel2 = ADC_CHANNEL_3;     // A3
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float distance_global = 0.00;
float distance_global2 = 0.00; 

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void get_voltage(void)
{

        //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel, atten);
        adc1_config_channel_atten(channel2, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        uint32_t adc_reading2 = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
                adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                int raw2; 
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_10, &raw);
                adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_10, &raw2);
                adc_reading += raw;
                adc_reading2 += raw2;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars);

           // Calculate the distance using linear regression 

        // convert from mV to V
        float voltagetemp = voltage / 1000.00;
        float voltagetemp2 = voltage2 / 1000.00;


        float distance = 0.00;
        float distance2 = 0.00;

        if (voltagetemp > .8){
                distance = 59.50 + (-40.00/1.40) * voltagetemp; 
        }
        else if (voltagetemp < .8 && voltagetemp > 0.40){
            distance = 80.00 + (-40.00/ 1.10) *voltagetemp; 
        }
        else {
            //printf("Distance must been between 20 cm and 150 cm\n");
        }


        if (voltagetemp2 > .8){
                distance2 = 59.50 + (-40.00/1.40) * voltagetemp2; 
        }
        else if (voltagetemp2 < .8 && voltagetemp2 > 0.40){
            distance2 = 80.00 + (-40.00/ 1.10) *voltagetemp2; 
        }
        else {
            //printf("Distance must been between 20 cm and 150 cm\n");
        }


        printf("Raw: %d \t Distance: %.3f cm\t Voltage: %f\n\n", adc_reading, distance, voltagetemp);
        printf("Raw 2: %d \t Distance 2: %.3f cm\t Voltage 2: %f\n\n", adc_reading2, distance2, voltagetemp2);

        distance_global = distance; 
        distance_global2 = distance2; 

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    get_voltage();
}

