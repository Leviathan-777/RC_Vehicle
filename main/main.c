/**
 * @file main.c
 * @brief Application entry point.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "vehicle_control.h"
#include "ble_comm.h"

static const char *TAG = "APP_MAIN";

/**
 * @brief Application entry; initialize platform services and motor driver.
 */
void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");

    vehicle_control_init();
    ble_init();

    /* Main loop: idle */
    while (1) 
    {
        ble_send_notification("HEARTBEAT", 9);
        vTaskDelay(pdMS_TO_TICKS(1000)); /* 1 second */
    }
}