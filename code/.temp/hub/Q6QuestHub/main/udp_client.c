#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

//include uart communication
#include "uart.h"

// i2c alphanumeric display of an integer variable splitTime
#include "alpha.h"

int splitTime = 8008;


void app_main(void)
{


    //initialize connection to webserver 
    UDPHandler();
    
    // Task for 14 segment display
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    xTaskCreate(alpha_down,"alpha_down", 4096, NULL, 5, NULL);
    
    while(1) {
    	UartHandler();
    	vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

}

