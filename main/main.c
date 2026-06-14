
#include <stdio.h>
#include <stdint.h>

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"

#include "ble_task.h"
#include "sensor_handler.h"

// Logging tags
#define TAG_INIT "Initialization"

bool app_init_sensor(void){
    ESP_LOGI(TAG_INIT, "Initializing sensors...");

    int status = sensor_init();
    if(status != 0){
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x (%s)", __func__, status, esp_err_to_name(status));
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

void app_main(void){

    // Initializing sensor
    if(!app_init_sensor()){
        ESP_LOGE(TAG_INIT, "Sensor initialization failed, aborting...");
        return;
    }

    // Initializing non-volatile storage library to store BLE configuration data
    if(!app_init_nvs()){
        ESP_LOGE(TAG_INIT, "NVS initialization failed, aborting...");
        return;
    }

    // Initializing BLE
    if(!app_init_ble()){
        ESP_LOGE(TAG_INIT, "BLE initialization failed, aborting...");
        return;
    }
}
