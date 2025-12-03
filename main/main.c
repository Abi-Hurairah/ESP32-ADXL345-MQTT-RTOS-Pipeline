/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "station.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adxl.h"
#include "mqtt.h"

static const char *TAG = "MAIN_APP";

extern QueueHandle_t xQueue_Sensordata = NULL;

/*
void read_sensor_task(void *pvParameters)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;
    uint8_t data_buffer[6];
    int16_t x_axis = 0;
    int16_t y_axis = 0;
    int16_t z_axis = 0;
    esp_err_t err;
    BaseType_t xReturn;

    xReturn = xQueueSend(xQueue_Sensordata, (void *)&x_axis, 10);
    
    while(1)
    {
        // 1. Attempt to read the data and capture the return status
        err = adxl345_register_read(dev_handle, ADXL345_DATAX0_REG_ADDR, data_buffer, 6);
        
        // 2. Check if the read was successful (err == ESP_OK)
        if (err == ESP_OK) 
        {
        x_axis = (data_buffer[1] << 8) | data_buffer[0];
        y_axis = (data_buffer[3] << 8) | data_buffer[2];
        z_axis = (data_buffer[5] << 8) | data_buffer[4];

        ESP_LOGI(TAG, "X=%d, Y=%d, Z=%d", x_axis, y_axis, z_axis);

        if (xReturn == pdTRUE)
        {
             printf("%d: Item Send SUCCESS\n", x_axis);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
*/


void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xQueue_Sensordata = xQueueCreate(1, 10);

    if (xQueue_Sensordata == NULL) 
    {
        ESP_LOGI(TAG, "Create FALSE\n");
    }
    else 
    {
        ESP_LOGI(TAG, "Create SUCCESS\n");
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // To Do: Inbetween code that checks for WiFi Connectivity. Perhaps the code below can be used somehow.
    // xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);

    mqtt_app_start();

    uint8_t data[1];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Read the adxl345 WHO_AM_I register, on power up the register should have the value 0x53
    ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DEVID_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    ESP_LOGI(TAG, "Enabling measurement mode");
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_POWER_CTL_REG_ADDR, 1 << ADXL345_MEASURE_BIT));

    ESP_LOGI(TAG, "Reading 6 bytes of axis data... Now with task!");
    xTaskCreate(read_sensor_task, "read_sensor_task", 4096, dev_handle, 6, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 3, NULL);
}
