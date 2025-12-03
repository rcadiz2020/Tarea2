#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "wifi_tcp.h"
#include "wifi_udp.h" 
#include "sdkconfig.h"

// Inclusión condicional de los headers del sensor
#if defined(CONFIG_SENSOR_BMI270)
    #include "bmi270.h"
#elif defined(CONFIG_SENSOR_BME688)
    #include "bme688.h"
#endif

static const char *TAG = "APP_MAIN";
#define SEND_BUFFER_SIZE 256 
#define MAX_CONNECTION_RETRIES 10

static void sensor_net_task(void *arg)
{
    char send_buf[SEND_BUFFER_SIZE];
    
    // Definimos la variable 'sensor_data' según el sensor seleccionado
    #if defined(CONFIG_SENSOR_BMI270)
        bmi270_data_t sensor_data;
    #elif defined(CONFIG_SENSOR_BME688)
        bme688_data_t sensor_data;
    #endif

    int sock = -1;
    int connection_retries = 0;

    while (connection_retries < MAX_CONNECTION_RETRIES) {
        
        // --- SELECCIÓN DE PROTOCOLO ---
        #if defined(CONFIG_PROTOCOL_TCP)
            ESP_LOGI(TAG, "Modo TCP Seleccionado en Menuconfig");
            sock = wifi_tcp_connect();
        #elif defined(CONFIG_PROTOCOL_UDP)
            ESP_LOGI(TAG, "Modo UDP Seleccionado en Menuconfig");
            sock = wifi_udp_create_socket();
        #else
            ESP_LOGE(TAG, "Ningún protocolo definido! Revisa Kconfig.");
            vTaskDelete(NULL);
        #endif

        if (sock < 0) {
            connection_retries++;
            ESP_LOGE(TAG, "Fallo socket (%d/%d). Esperando 5s...", connection_retries, MAX_CONNECTION_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue; 
        }

        ESP_LOGI(TAG, "Socket listo. Iniciando bucle de envío.");
        connection_retries = 0;

        // Bucle de envío continuo
        while (1) {
            esp_err_t ret = ESP_FAIL;
            int len = 0;

            // --- LECTURA Y FORMATO JSON SEGÚN SENSOR ---
            #if defined(CONFIG_SENSOR_BMI270)
                ret = bmi270_read_data(&sensor_data);
                if (ret == ESP_OK) {
                    len = snprintf(send_buf, SEND_BUFFER_SIZE,
                        "{\"type\":\"bmi270\", \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, \"gx\":%.2f, \"gy\":%.2f, \"gz\":%.2f}\n",
                        sensor_data.ax, sensor_data.ay, sensor_data.az,
                        sensor_data.gx, sensor_data.gy, sensor_data.gz);
                }
            #elif defined(CONFIG_SENSOR_BME688)
                ret = bme688_read_data(&sensor_data);
                if (ret == ESP_OK) {
                    // Enviamos Temp, Presion, Humedad y Gas
                    len = snprintf(send_buf, SEND_BUFFER_SIZE,
                        "{\"type\":\"bme688\", \"temp\":%.2f, \"press\":%.2f, \"hum\":%.2f, \"gas\":%.2f}\n",
                        sensor_data.temperature, sensor_data.pressure, sensor_data.humidity, sensor_data.gas_resistance);
                }
            #endif

            if (ret == ESP_OK) {

                ESP_LOGI(TAG, "Enviando: %s", send_buf);

                int send_result = -1;
                
                // --- ENVÍO SEGÚN PROTOCOLO ---
                #if defined(CONFIG_PROTOCOL_TCP)
                    send_result = send(sock, send_buf, len, 0);
                #elif defined(CONFIG_PROTOCOL_UDP)
                    send_result = wifi_udp_send(sock, send_buf, len);
                #endif

                if (send_result < 0) {
                    ESP_LOGE(TAG, "Error enviando datos. Reiniciando socket.");
                    break; 
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                
            } else {
                ESP_LOGE(TAG, "Error leyendo sensor o no disponible.");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } 

        // Limpieza
        #if defined(CONFIG_PROTOCOL_TCP)
            wifi_tcp_disconnect(sock);
        #elif defined(CONFIG_PROTOCOL_UDP)
            wifi_udp_close(sock);
        #endif
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    } 

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando aplicación...");

    wifi_tcp_init_sta();

    // Inicialización condicional del hardware del sensor
    #if defined(CONFIG_SENSOR_BMI270)
        ESP_LOGI(TAG, "Iniciando BMI270...");
        ESP_ERROR_CHECK(bmi270_init());
    #elif defined(CONFIG_SENSOR_BME688)
        ESP_LOGI(TAG, "Iniciando BME688...");
        ESP_ERROR_CHECK(bme688_init());
    #endif

    xTaskCreate(sensor_net_task, "sensor_net_task", 4096, NULL, 5, NULL);
}