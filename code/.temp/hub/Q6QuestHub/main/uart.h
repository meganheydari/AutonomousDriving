#ifndef UART_H
#define UART_H

// uart define
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "protocol_examples_common.h"
#include <stdbool.h>

#define ECHO_TEST_TXD  26
#define ECHO_TEST_RXD  16 //CHANGE
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

// end uart define
char recievedValues[5] = "";
char temprecievedValues[5] = "";


u16_t CONFIGPORT = 8080;
const char* IP = "192.168.1.108";
bool sendData = false;

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT CONFIGPORT

static const char *TAG = "example";
//static const char *payload = "mock data from hub";
static const char *initPayload = "HUB_INIT";
TaskHandle_t  UDPHandle = NULL;
TaskHandle_t  UartHandle = NULL;


/* Configure parameters of an UART driver,
 * communication pins and install the driver */
uart_config_t uart_config = {
    .baud_rate = 2400,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

// main logic for udp connection to the webserver
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

        #ifdef CONFIG_EXAMPLE_IPV4
                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        #else // IPV6
                struct sockaddr_in6 dest_addr;
                inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
                dest_addr.sin6_family = AF_INET6;
                dest_addr.sin6_port = htons(PORT);
                addr_family = AF_INET6;
                ip_protocol = IPPROTO_IPV6;
                inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        #endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        int err = sendto(sock, initPayload, strlen(initPayload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        ESP_LOGI(TAG, "Init message sent");
    	
            
        char *payload = recievedValues;
        while (1) {

            if (sendData) {

            	if (recievedValues[1] != temprecievedValues[1]) {

                int SendErr = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (SendErr < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                } ESP_LOGI(TAG, "sent");

		        temprecievedValues[0] = recievedValues[0];
		        temprecievedValues[1] = recievedValues[1];
		        temprecievedValues[2] = recievedValues[2];
		        temprecievedValues[3] = recievedValues[3];
		    }


                sendData = false;
            }
            
            //100 ms delay
            vTaskDelay(100 / portTICK_PERIOD_MS);

        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    UDPHandle = NULL;
    vTaskDelete(UDPHandle);
}

// handles creation and deletion of udp connection to webserver
void UDPHandler()
{
	//initialize
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    printf("running udp handler\n");
    if(UDPHandle != NULL)
    {
        vTaskDelete(UDPHandle);
        UDPHandle = NULL;
    }
    xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, &UDPHandle);
    configASSERT(UDPHandle);
}


static void echo_task(void *arg) {

	uart_config_t uart_config = {
    .baud_rate = 1200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};


	uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
    uint8_t *data1 = (const char *) "AAAAAAAA";
    int readLen = 8;
    uint8_t *data = (uint8_t *) malloc(readLen);

    // Configure a temporary buffer for the incoming data
    uart_write_bytes(UART_NUM_1, (const char *) data1, readLen);
    printf("Init\n");

    int len;
    int i;
    while (1) {
        // Read data from the UART
        len = uart_read_bytes(UART_NUM_1, data, readLen, 0);
        if (len > 0) {
        	ESP_LOGI(TAG,"Data recieved: %X %X %X %X %X %X %X %X\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            for (i = 0; i < readLen - 4; i++) {
                if (data[i] == 0x1B) {
                    recievedValues[0] = data[i];
                    recievedValues[1] = data[i+1];
                    recievedValues[2] = data[i+2];
                    recievedValues[3] = data[i+3];

                    ESP_LOGI(TAG,"Recieved Data: %x %x %x %x", recievedValues[0], recievedValues[1], recievedValues[2], recievedValues[3]);
                    sendData = true;
                    UartHandle = NULL;

                    uart_driver_delete(UART_NUM_1);
        			memset(data, 0, readLen*sizeof(uint8_t));
                    vTaskDelete(UartHandle);
                }

            }
        }vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

int count = 0;
// handles creation and deletion of udp connection to webserver
void UartHandler()
{
    //printf("running uart handler\n");
    if(UartHandle != NULL)
    {
    	count += 1;
    	printf("%d\n", count);

    	if (count > 2) {
    		vTaskDelete(UartHandle);
        	UartHandle = NULL;
        	uart_driver_delete(UART_NUM_1);
    	}
    	else {
    		return;
    	}
    }

    count = 0;
    xTaskCreate(echo_task, "uart_echo_task", 4096, NULL, 5, &UartHandle);
    configASSERT(UartHandle);
}

#endif

