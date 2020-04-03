#ifndef UDP_H
#define UDP_H


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
#include "uart.h"


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

                int SendErr = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (SendErr < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                } ESP_LOGI(TAG, "sent");
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

#endif