
#include <stdio.h>
#include <stdint.h>

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "gatt_server.h"
#include "sensor_handler.h"

// Logging tags
#define TAG_INIT "Initialization"
#define TAG_BLE "BLE"

static uint8_t address_type;

// Callback when the BLE stack resets
void app_ble_on_reset(int reason){
    ESP_LOGW(TAG_BLE, "BLE stack reset: reason = %d", reason);
}

void app_ble_configure_address(){

    int status;

    ESP_LOGI(TAG_BLE, "Configuring Bluetooth address...");
    status = ble_hs_util_ensure_addr(0);
    if(status != 0){
        ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
        return;
    }

    ESP_LOGI(TAG_BLE, "Determining Bluetooth address type...");
    status = ble_hs_id_infer_auto(0, &address_type);
    if(status != 0){
        ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
        return;
    }

    ESP_LOGI(TAG_BLE, "Getting Bluetooth address...");
    uint8_t address[6] = {0};
    status = ble_hs_id_copy_addr(address_type, address, NULL);
    if(status != 0){
        ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
        return;
    } else {
        ESP_LOGI(TAG_BLE, "BLE address: %x:%x:%x:%x:%x:%x", address[5], address[4], address[3], address[2], address[1], address[0]);
    }
}

void app_print_address(uint8_t *address_value, uint8_t address_type, const char *address_name){

    // 18 bytes for XX:XX:XX:XX:XX:XX + null terminator
    char address_str[18];

    const char *address_type_str;

    switch (address_type) {
        case BLE_ADDR_PUBLIC:
            address_type_str = "Public address";
            break;
        case BLE_ADDR_RANDOM:
            address_type_str = "Random address";
            break;
        case BLE_ADDR_PUBLIC_ID:
            address_type_str = "Public identity address";
            break;
        case BLE_ADDR_RANDOM_ID:
            address_type_str = "Random identity address";
            break;
        default:
            address_type_str = "Unknown address type";

    }

    sprintf(address_str, "%02X:%02X:%02X:%02X:%02X:%02X", address_value[5], address_value[4], address_value[3], address_value[2], address_value[1], address_value[0]);

    ESP_LOGI(TAG_BLE, "%s..........%s (type = %d: %s)", address_name, address_str, address_type, address_type_str);
}

void app_print_conn_descriptor(struct ble_gap_conn_desc *conn_descriptor){

    ESP_LOGI(TAG_BLE, "Connection handle..........%d", conn_descriptor->conn_handle);

    app_print_address(conn_descriptor->our_ota_addr.val, conn_descriptor->our_ota_addr.type, "Local OTA address");
    app_print_address(conn_descriptor->our_id_addr.val, conn_descriptor->our_id_addr.type, "Local identity address");
    app_print_address(conn_descriptor->peer_ota_addr.val, conn_descriptor->peer_ota_addr.type, "Peer OTA address");
    app_print_address(conn_descriptor->peer_id_addr.val, conn_descriptor->peer_id_addr.type, "Peer identity address");
    ESP_LOGI(TAG_BLE, "Connection interval..........%d (%.2f ms)", conn_descriptor->conn_itvl, conn_descriptor->conn_itvl*1.25);
    ESP_LOGI(TAG_BLE, "Connection latency..........%d", conn_descriptor->conn_latency);
    ESP_LOGI(TAG_BLE, "Supervision timeout..........%d (%d ms)", conn_descriptor->supervision_timeout, conn_descriptor->supervision_timeout*10);
    ESP_LOGI(TAG_BLE, "Encrypted..........%s", (conn_descriptor->sec_state.encrypted == 1) ? "Yes" : "No"); 
    ESP_LOGI(TAG_BLE, "Bonded..........%s", (conn_descriptor->sec_state.bonded == 1) ? "Yes" : "No");
    ESP_LOGI(TAG_BLE, "Authenticated..........%s", (conn_descriptor->sec_state.authenticated == 1) ? "Yes" : "No");
    ESP_LOGI(TAG_BLE, "Authorized..........%s", (conn_descriptor->sec_state.authorize == 1) ? "Yes" : "No");
    ESP_LOGI(TAG_BLE, "Key size..........%d", conn_descriptor->sec_state.key_size);
}

void app_print_conn_update(const struct ble_gap_upd_params *update_parameters){
    ESP_LOGI(TAG_BLE, "Connection interval minimum..........%d (%.2f ms)", update_parameters->itvl_min, update_parameters->itvl_min * 1.25);
    ESP_LOGI(TAG_BLE, "Connection interval maximum..........%d (%.2f ms)", update_parameters->itvl_max, update_parameters->itvl_max * 1.25);
    ESP_LOGI(TAG_BLE, "Connection latency..........%d", update_parameters->latency);
    ESP_LOGI(TAG_BLE, "Supervision timeout..........%d (%d ms)", update_parameters->supervision_timeout, update_parameters->supervision_timeout * 10);
    ESP_LOGI(TAG_BLE, "Minimum connection event length..........%d (%.3f ms)", update_parameters->min_ce_len, update_parameters->min_ce_len * 0.625);
    ESP_LOGI(TAG_BLE, "Maximum connection event length..........%d (%.3f ms)", update_parameters->max_ce_len, update_parameters->max_ce_len * 0.625);
}

// GAP event callback
int app_ble_on_gap_event(struct ble_gap_event *event, void *arg){
    ESP_LOGI(TAG_BLE, "GAP event type 0x%x", event->type);

    struct ble_gap_conn_desc conn_descriptor;
    int status;
    
    switch(event->type){
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG_BLE, "Connection event (BLE_GAP_EVENT_CONNECT), status = %d", event->connect.status);
            if(event->connect.status == 0){
                ESP_LOGI(TAG_BLE, "Connection successful");

                status = ble_gap_conn_find(event->connect.conn_handle, &conn_descriptor);
                if(status != 0){
                    ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
                    return -1;
                }

                app_print_conn_descriptor(&conn_descriptor);
            } else {
                ESP_LOGW(TAG_BLE, "Connection unsuccessful");
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG_BLE, "Disconnected (BLE_GAP_EVENT_DISCONNECT), reason = %d", event->disconnect.reason);
            break;
        case BLE_GAP_EVENT_CONN_UPDATE:
            ESP_LOGI(TAG_BLE, "Connection update event  (BLE_GAP_EVENT_CONN_UPDATE), status = %d", event->conn_update.status);
            if(event->conn_update.status == 0){
                ESP_LOGI(TAG_BLE, "Connection update successful");
            } else {
                ESP_LOGW(TAG_BLE, "Connection update unsuccessful");
            }
            break;
        case BLE_GAP_EVENT_CONN_UPDATE_REQ:
            ESP_LOGI(TAG_BLE, "Connection update request event  (BLE_GAP_EVENT_CONN_UPDATE_REQ)");
            // Accepting all connection update requests, so no need to modify parameters, only using this case for logging purposes
            app_print_conn_update(event->conn_update_req.peer_params);
            break;
        default:
            ESP_LOGW(TAG_BLE, "Unknown event");
    }

    return 0;
}

void app_ble_start_advertising(){

    // Parameters for the BLE advertisement
    struct ble_gap_adv_params adv_parameters;

    // Fields broadcasted in the BLE advertisement
    struct ble_hs_adv_fields adv_fields;

    const char *device_name;

    int status;

    memset(&adv_fields, 0, sizeof(adv_fields));

    // Set general discoverable mode and Bluetooth Classic unsupported flags
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Include tx power level in advertisement and automatically determine its value
    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Set device name in BLE advertisement packet
    device_name = ble_svc_gap_device_name();

    adv_fields.name_is_complete = 1;
    adv_fields.name = (uint8_t *)device_name;
    adv_fields.name_len = strlen(device_name);

    status = ble_gap_adv_set_fields(&adv_fields);
    if(status != 0){
        ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
        return;
    }

    memset(&adv_parameters, 0 , sizeof(adv_parameters));

    // Undirected connection mode
    adv_parameters.conn_mode = BLE_GAP_CONN_MODE_UND;

    // General discoverable mode
    adv_parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // Start advertisement with app_ble_on_gap_event function as callback for GAP events
    status = ble_gap_adv_start(address_type, NULL, BLE_HS_FOREVER, &adv_parameters, app_ble_on_gap_event, NULL);
    if(status != 0){
        ESP_LOGE(TAG_BLE, "Error in %s: 0x%x", __func__, status);
        return;
    }
}

// Callback when BLE stack synchronizes (is ready to start)
void app_ble_on_sync(){

    ESP_LOGI(TAG_BLE, "NimBLE stack synchronized with controller");

    app_ble_configure_address();

    app_ble_start_advertising();
}

// Task to run NimBLE stack
void app_ble_main_task(void *param){

    ESP_LOGI(TAG_BLE, "NimBLE host task started");

    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void){
    int status;

    // Initializing sensor
    ESP_LOGI(TAG_INIT, "Initializing sensors...");
    status = sensor_init();
    if(status != 0){
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x (%s)", __func__, status, esp_err_to_name(status));
        return;
    }

    // Initializing non-volatile storage library to store BLE configuration data
    // Erase and reinitialize if NVS has new version or is full
    ESP_LOGI(TAG_INIT, "Initializing NVS...");
    status = nvs_flash_init();
    if(status == ESP_ERR_NVS_NEW_VERSION_FOUND || status == ESP_ERR_NVS_NO_FREE_PAGES){
        ESP_LOGW(TAG_INIT, "NVS outdated or full, erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);

    // Initialize NimBLE
    ESP_LOGI(TAG_INIT, "Initializing NimBLE...");
    status = nimble_port_init();
    if(status != 0){
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x (%s)", __func__, status, esp_err_to_name(status));
        return;
    }


    // NimBLE host stack configuration

    // Callback if NimBLE host resets
    ble_hs_cfg.reset_cb = app_ble_on_reset;

    // Callback when host and controller are synchronized 
    // and ready to perform BLE operations
    ble_hs_cfg.sync_cb = app_ble_on_sync;

    // Set the BLE device name
    ESP_LOGI(TAG_INIT, "Setting device name...");
    status = ble_svc_gap_device_name_set("ESP32-Tricorder");
    if(status != 0) { 
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x", __func__,  status); 
        return; 
    }

    ESP_LOGI(TAG_INIT, "Initializing GATT server...");
    status = gattserver_init();
    if(status != 0) { 
        ESP_LOGE(TAG_INIT, "Error in %s: 0x%x", __func__,  status); 
        return; 
    }

    // Start NimBLE task
    nimble_port_freertos_init(app_ble_main_task);
}
