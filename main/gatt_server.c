
#include "esp_log.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include <math.h>

#include "sensor_handler.h"

#include "gatt_server.h"

// GATT server tag
#define TAG_GATTS "GATTS"

// Environmental sensing service and characteristics UUID values are from Bluetooth standard
// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf?v=1725972233968
static const ble_uuid16_t gattserver_environmental_sensing_svc_uuid = BLE_UUID16_INIT(0x181A);
static const ble_uuid16_t gattserver_pressure_chr_uuid = BLE_UUID16_INIT(0x2A6D);
static const ble_uuid16_t gattserver_temperature_chr_uuid = BLE_UUID16_INIT(0x2A6E);
static const ble_uuid16_t gattserver_humidity_chr_uuid = BLE_UUID16_INIT(0x2A6F);

// Handles for characteristic value attributes (populated by the stack)
static uint16_t gattserver_pressure_chr_val_handle;
static uint16_t gattserver_temperature_chr_val_handle;
static uint16_t gattserver_humidity_chr_val_handle;

// Callback for accessing (reading) pressure characteristic
static int gattserver_access_pressure_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    ESP_LOGI(TAG_GATTS, "Accessing pressure characteristic");

    // This should never occur, as it would indicate that the stack called the incorrect callback for the access request
    if(attr_handle != gattserver_pressure_chr_val_handle){
        ESP_LOGE(TAG_GATTS, "Incorrect attribute handle passed to pressure access callback");
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    int status;

    switch(ctxt->op){
        case BLE_GATT_ACCESS_OP_READ_CHR:
            double raw_pressure_val = sensor_bme280_get_pressure();

            // NaN value indicates error reading from sensor
            if(isnan(raw_pressure_val)) { return BLE_ATT_ERR_UNLIKELY; }

            // BLE standard pressure value in units of Pascals (1 Pa)
            uint32_t pressure_val = (uint32_t)(round(raw_pressure_val));

            ESP_LOGI(TAG_GATTS, "Transmitting pressure value (units of 1 Pa): %lu", pressure_val);
            status = os_mbuf_append(ctxt->om, &pressure_val, sizeof(pressure_val));
            if(status != 0){
                ESP_LOGE(TAG_GATTS, "Unable to append pressure value to the output memory buffer");
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        default:
            // Only reads are allowed for environmental data characteristics
            ESP_LOGW(TAG_GATTS, "Unsupported access operation 0x%x", ctxt->op);
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG_GATTS, "Access operation completed");
    return 0;
}

// Callback for accessing (reading) temperature characteristic
static int gattserver_access_temperature_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    ESP_LOGI(TAG_GATTS, "Accessing temperature characteristic");

    // This should never occur, as it would indicate that the stack called the incorrect callback for the access request
    if(attr_handle != gattserver_temperature_chr_val_handle){
        ESP_LOGE(TAG_GATTS, "Incorrect attribute handle passed to temperature access callback");
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    int status;

    switch(ctxt->op){
        case BLE_GATT_ACCESS_OP_READ_CHR:
            double raw_temperature_val = sensor_bme280_get_temperature();

            // NaN value indicates error reading from sensor
            if(isnan(raw_temperature_val)) { return BLE_ATT_ERR_UNLIKELY; }

            // BLE standard temperature value in units of hundredths of a degree C (0.01°C)
            int16_t temperature_val = (int16_t)(round(raw_temperature_val * 100));

            ESP_LOGI(TAG_GATTS, "Transmitting temperature value (units of 0.01°C): %d", temperature_val);
            status = os_mbuf_append(ctxt->om, &temperature_val, sizeof(temperature_val));
            if(status != 0){
                ESP_LOGE(TAG_GATTS, "Unable to append temperature value to the output memory buffer");
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        default:
            // Only reads are allowed for environmental data characteristics
            ESP_LOGW(TAG_GATTS, "Unsupported access operation 0x%x", ctxt->op);
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG_GATTS, "Access operation completed");
    return 0;
}

// Callback for accessing (reading) humidity characteristic
static int gattserver_access_humidity_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    ESP_LOGI(TAG_GATTS, "Accessing humidity characteristic");

    // This should never occur, as it would indicate that the stack called the incorrect callback for the access request
    if(attr_handle != gattserver_humidity_chr_val_handle){
        ESP_LOGE(TAG_GATTS, "Incorrect attribute handle passed to humidity access callback");
        return BLE_ATT_ERR_INVALID_HANDLE;
    }

    int status;

    switch(ctxt->op){
        case BLE_GATT_ACCESS_OP_READ_CHR:
            double raw_humidity_val = sensor_bme280_get_humidity();

            // NaN value indicates error reading from sensor
            if(isnan(raw_humidity_val)) { return BLE_ATT_ERR_UNLIKELY; }

            // BLE standard humidity value in units of hundredeths of relative humidity percentage (0.01%)
            uint16_t humidity_val = (uint16_t)(round(raw_humidity_val*100));

            ESP_LOGI(TAG_GATTS, "Transmitting humidity value (units of 0.01%%): %d", humidity_val);
            status = os_mbuf_append(ctxt->om, &humidity_val, sizeof(humidity_val));
            if(status != 0){
                ESP_LOGE(TAG_GATTS, "Unable to append humidity value to the output memory buffer");
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        default:
            // Only reads are allowed for environmental data characteristics
            ESP_LOGW(TAG_GATTS, "Unsupported access operation 0x%x", ctxt->op);
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG_GATTS, "Access operation completed");
    return 0;
}

// Definition of environmental sensing service structure
static const struct ble_gatt_svc_def gattserver_services[] = {
    {
        // Environmental sensing service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gattserver_environmental_sensing_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Pressure characteristic
                .uuid = &gattserver_pressure_chr_uuid.u,
                .access_cb = gattserver_access_pressure_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &gattserver_pressure_chr_val_handle,
            }, 
            {
                // Temperature characteristic
                .uuid = &gattserver_temperature_chr_uuid.u,
                .access_cb = gattserver_access_temperature_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &gattserver_temperature_chr_val_handle,
            },
            {
                // Humidity characteristic
                .uuid = &gattserver_humidity_chr_uuid.u,
                .access_cb = gattserver_access_humidity_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &gattserver_humidity_chr_val_handle,
            }, 
            { 0 } // Terminator element for characteristics array
        }
    }, 
    { 0 } // Terminator element of services array
};

int gattserver_init(){

    int status;
    
    // Initialize GAP and GATT services on the server
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Counts the declared services and reserves the necessary resources
    status = ble_gatts_count_cfg(gattserver_services);
    if(status != 0){
        ESP_LOGE(TAG_GATTS, "Could not count service resources during GATT intialization");
        return status;
    }

    // Adds the declared services to the GATT server
    status = ble_gatts_add_svcs(gattserver_services);
    if(status != 0){
        ESP_LOGE(TAG_GATTS, "Could not add services during GATT intialization");
        return status;
    }

    return 0;
}
