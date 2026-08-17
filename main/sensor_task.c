#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "bme280.h"
#include "sensor_handler.h"
#include "display.h"

#include "sensor_task.h"

#define TAG_SENSOR_TASK "SensorTask"

static sensor_reading_t cache_g;
static SemaphoreHandle_t cache_mutex_g;
static uint32_t period_ms_g;

static void sensor_task_update_cache(const struct bme280_data *measurement_data){
    xSemaphoreTake(cache_mutex_g, portMAX_DELAY);
    cache_g.temperature = measurement_data->temperature;
    cache_g.pressure = measurement_data->pressure;
    cache_g.humidity = measurement_data->humidity;
    cache_g.valid = true;
    xSemaphoreGive(cache_mutex_g);
}

static void sensor_task_invalidate_cache(void){
    xSemaphoreTake(cache_mutex_g, portMAX_DELAY);
    cache_g.valid = false;
    xSemaphoreGive(cache_mutex_g);
}

static void sensor_task_main(void *param){

    struct bme280_data measurement_data;

    for(;;){
        int status = sensor_bme280_get_measurements(&measurement_data);
        if(status == 0){
            sensor_task_update_cache(&measurement_data);

            app_display_set_temperature(measurement_data.temperature);
            app_display_set_pressure(measurement_data.pressure);
            app_display_set_humidity(measurement_data.humidity);
        } else {
            ESP_LOGW(TAG_SENSOR_TASK, "Sensor read failed, invalidating cached reading");
            sensor_task_invalidate_cache();
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms_g));
    }
}

bool sensor_task_start(uint32_t period_ms){

    cache_mutex_g = xSemaphoreCreateMutex();
    if(cache_mutex_g == NULL){
        ESP_LOGE(TAG_SENSOR_TASK, "Could not create cache mutex");
        return false;
    }

    period_ms_g = period_ms;

    BaseType_t status = xTaskCreate(sensor_task_main, "sensor_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    if(status != pdPASS){
        ESP_LOGE(TAG_SENSOR_TASK, "Could not create sensor task");
        vSemaphoreDelete(cache_mutex_g);
        cache_mutex_g = NULL;
        return false;
    }

    return true;
}

bool sensor_task_get_cached_reading(sensor_reading_t *out){

    if(cache_mutex_g == NULL){
        return false;
    }

    xSemaphoreTake(cache_mutex_g, portMAX_DELAY);
    memcpy(out, &cache_g, sizeof(cache_g));
    xSemaphoreGive(cache_mutex_g);

    return true;
}
