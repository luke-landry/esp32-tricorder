
#include <stdio.h>
#include <stdint.h>

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"

#include "ble_task.h"
#include "display.h"
#include "sensor_handler.h"
#include "sensor_task.h"

// Logging tags
#define TAG_INIT "Initialization"

// Delay between periodic sensor readings
#define SENSOR_TASK_PERIOD_MS 1000

bool app_init_sensor(void){
    ESP_LOGI(TAG_INIT, "Initializing sensors...");

    int status = sensor_init();
    if(status != 0){
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x (%s)", __func__, status, esp_err_to_name(status));
        return false;
    }

    return true;
}

bool app_init_sensor_task(void){
    ESP_LOGI(TAG_INIT, "Starting sensor task...");

    if(!sensor_task_start(SENSOR_TASK_PERIOD_MS)){
        ESP_LOGE(TAG_INIT, "Could not start sensor task");
        return false;
    }

    return true;
}

bool app_init_nvs(void){
    ESP_LOGI(TAG_INIT, "Initializing NVS...");

    esp_err_t status = nvs_flash_init();

    // Erase and reinitialize if NVS has new version or is full
    if(status == ESP_ERR_NVS_NEW_VERSION_FOUND || status == ESP_ERR_NVS_NO_FREE_PAGES){
        ESP_LOGW(TAG_INIT, "NVS outdated or full, erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }

    if(status != ESP_OK){
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x (%s)", __func__, status, esp_err_to_name(status));
        return false;
    }

    return true;
}

bool app_init_ble(void){
    ESP_LOGI(TAG_INIT, "Initializing BLE...");

    if(!app_ble_start_task()){
        ESP_LOGE(TAG_INIT, "BLE initialization failed");
        return false;
    }

    return true;
}

bool app_init_display(void){
    ESP_LOGI(TAG_INIT, "Initializing display...");
    return app_display_init();
}

void app_main(void){

    // Initializing display
    if(!app_init_display()){
        ESP_LOGE(TAG_INIT, "Display initialization failed, aborting...");
        return;
    }

    // Initializing sensor
    app_display_set_message_2("Initializing sensor...");
    if(!app_init_sensor()){
        ESP_LOGE(TAG_INIT, "Sensor initialization failed, aborting...");
        app_display_set_message_2("Sensor initialization failed");
        return;
    }

    // Starting periodic sensor task
    app_display_set_message_2("Starting sensor task...");
    if(!app_init_sensor_task()){
        ESP_LOGE(TAG_INIT, "Sensor task failed to start, aborting...");
        app_display_set_message_2("Sensor task failed to start");
        return;
    }

    // Initializing non-volatile storage library to store BLE configuration data
    app_display_set_message_2("Initializing NVS...");
    if(!app_init_nvs()){
        ESP_LOGE(TAG_INIT, "NVS initialization failed, aborting...");
        app_display_set_message_2("NVS initialization failed");
        return;
    }

    // Initializing BLE
    app_display_set_message_2("Initializing BLE...");
    if(!app_init_ble()){
        ESP_LOGE(TAG_INIT, "BLE initialization failed, aborting...");
        app_display_set_message_2("BLE initialization failed");
        return;
    }

    app_display_set_message_2("Ready");
}
