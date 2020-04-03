#ifndef SERIAL_H
#define SERIAL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <stdbool.h> 

// uart define
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#define ECHO_TEST_TXD  26
#define ECHO_TEST_RXD  16 //CHANGE
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)
uint8_t sendData[100];
uint8_t hubData[] = ", \"hub_ID\" : \"01\"}";
// end uart define

//define serial
static const int RX_BUF_SIZE = 1024;
TaskHandle_t  SerialHandle = NULL;
bool sendSerial = false;
int fob_ID = 0;
int hub_ID = 0;
int code   = 0;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
//end define serial

//uart tasks
static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);


    uint8_t *data1 = (const char *) "{\"fob_ID\" : \"01\", \"code\" : \"1234\"}";
    int readLen = strlen((const char *) data1);
    uint8_t *data = (uint8_t *) malloc(readLen);

    // Configure a temporary buffer for the incoming data
    uart_write_bytes(UART_NUM_1, (const char *) data1, readLen);


    while (1) {
        //flush data
        uart_flush(UART_NUM_1);
        memset(data, 0, readLen*sizeof(uint8_t));

        uart_write_bytes(UART_NUM_1, (const char *) data1, readLen);
        int i = 0;
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, readLen,0);
        if (len > 0) {
            if (data[0] == 123 && data[len - 1] == 125) {
                for (i = 0; i < len; i++) {
                    sendData[i] = data[i];
                }
                //printf("\nsendData: %s\n", sendData);
                sendSerial = true;
            }
        }
    }
}
// end uart tasks

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
}

char * append(char * string1, char * string2)
{
    char * result = NULL;
    asprintf(&result, "%s%s", string1, string2);
    return result;
}

static void tx_task(void *arg)
{
    int i = 0;
    while (1) {
        if (sendSerial) {
            for (i = 0; i < sizeof(hubData); i ++) {
                sendData[33 + i] = hubData[i];
            }
            printf("%s\n", sendData);
            sendSerial = false;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    SerialHandle = NULL;
    vTaskDelete(SerialHandle);
}

void initSerial() {
    printf("init serial\n");
    init();
}

void serialHandler() {
    printf("Running serial handler.\n");
    if(SerialHandle != NULL)
    {
        vTaskDelete(SerialHandle);
        SerialHandle = NULL;
    }

    xTaskCreate(tx_task, "tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, &SerialHandle);
    configASSERT(SerialHandle);
}

#endif
