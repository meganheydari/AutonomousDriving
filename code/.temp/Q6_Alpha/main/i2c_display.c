#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h> 
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "ascii.h"

#include "esp_types.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


bool countUp = true;
char buf[100];
int currHours;
int currSeconds;
int currMinutes;

int splitTime; 


// === define for the 14 segment
TaskHandle_t TaskHandle_alpha_down;


// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk (SCL)
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data (SCA)
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value


// servo 
//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 600 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

// === end def


static void i2c_example_master_init(){
    // Debug
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}


int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void alpha_down() {
    i2c_example_master_init();
    // Debug
    int ret;
    ret = alpha_oscillator();
    int temp = 0; 

    // Write to characters to buffer
    uint16_t displaybuffer[8];

    int count =  1; 
    // Continually writes the same command
    while (1) {

    // printf("#1: %d\n", splitTime);
    displaybuffer[3] = (numbertable[(int)(splitTime % 10)]); 
    // splitTime /=10; 

    // printf("#2: %d\n", splitTime);
    displaybuffer[2] = (numbertable[(int)(splitTime/10 % 10)]);
    // splitTime /=10; 

    // printf("#3: %d\n", splitTime);
    displaybuffer[1] = (numbertable[(int)(splitTime/100 % 10)]);
    // splitTime /=10; 

    // printf("#4: %d\n", splitTime);
    displaybuffer[0] = (numbertable[(int)floor((splitTime/1000 % 10))]);



    // displaybuffer[0] = (currHours > 9 ? numbertable[(int)floor(currHours / 10)] : numbertable[0]); //HH
    // displaybuffer[1] = (currHours > 9 ? numbertable[currHours % 10] : numbertable[currHours]); //HH
    // displaybuffer[2] = (currMinutes > 9 ? numbertable[(int)floor(currMinutes / 10)] : numbertable[0]); //MM
    // displaybuffer[3] = (currMinutes > 9 ? numbertable[currMinutes % 10] : numbertable[currMinutes]); //MM


      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

      if(ret == ESP_OK) {
        //printf(" wrote -- %s\n\n", buf);
      }
    }
}




void app_main(void)
{
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);


    // struct timeval now;
    // int rc;
    // time_t seconds;
    // int userTime = echoInp();
    // now.tv_sec=userTime;
    // now.tv_usec=0;

    //14 segment display
    // alpha_down();
    xTaskCreate(alpha_down,"alpha_down", 4096, NULL, 5, NULL);

    vTaskDelay(100 / portTICK_RATE_MS);

    while(1) {

        printf("Split time in seconds: %d \n",splitTime);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
