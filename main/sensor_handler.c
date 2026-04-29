
#include <stdbool.h>
#include <string.h>
#include <rom/ets_sys.h>
#include <math.h>

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "bme280.h"

#include "sensor_handler.h"

// Pin definitions for SDA and SCL lines
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

// Tag for logging purposes
#define TAG_BME280 "BME280"

/*
    From BME280 Documentation:
    "The 7-bit device address is 111011x. The 6 MSB bits are fixed. The last bit is changeable by SDO
    value and can be changed during operation. Connecting SDO to GND results in slave address
    1110110 (0x76); connection it to V DDIO results in slave address 1110111 (0x77), which is the same as
    BMP280’s I²C address. The SDO pin cannot be left floating; if left floating, the I²C address will be
    undefined.""

    In the current wiring, SDO is connected to GND so BME280 i2c address is 0x76
*/
#define BME280_ADDRESS 0x76

static uint32_t measurement_time_us_g;
static struct bme280_dev bme280_g;

// Initialize and configure i2c parameters for master device
void sensor_init_i2c_master(i2c_master_bus_handle_t *ptr_bus_handle){

    // configuration for i2c bus by master device
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, ptr_bus_handle));
}

// Initialize and configure i2c parameters for a new device
void sensor_init_i2c_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *ptr_device_handle, uint16_t device_i2c_address){
    

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_i2c_address,
        .scl_speed_hz = 100000,
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, ptr_device_handle));
}

int8_t sensor_bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){

    /*  
        BME280 Documentation
        "To be able to read registers, first the register address must be sent in write mode (slave address
        111011X0). Then either a stop or a repeated start condition must be generated. After this the slave is
        addressed in read mode (RW = ‘1’) at address 111011X1, after which the slave sends out data from
        auto-incremented register addresses until a NOACKM and stop condition occurs."
    */

    //ESP_LOGI(TAG_BME280, "Performing i2c read on register 0x%x (%lu bytes)", reg_addr, len);

    // The passed interface pointer is the esp i2c device handle for the bme280
    i2c_master_dev_handle_t bme280_handle = (i2c_master_dev_handle_t)intf_ptr;

    // Using transmit + receive function because the register address must be written to the bme280 sensor before reading from it
    // reg_addr is the only data that will be written, so its address can act as the write buffer pointer with length 1
    // Setting -1 as timeout to wait forever for operation to complete
    esp_err_t status = i2c_master_transmit_receive(bme280_handle, &reg_addr, 1, reg_data, len, -1);
    if(status != ESP_OK){
        ESP_LOGE(TAG_BME280, "Could not probe bme280 sensor, check I2C connection. ESP error code 0x%x: %s", status, esp_err_to_name(status));
        return status;
    }
    return 0;
}

int8_t sensor_bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){

    /*
        BME280 Documentation
        "Writing is done by sending the slave address in write mode (RW = ‘0’), resulting in slave address
        111011X0 (‘X’ is determined by state of SDO pin. Then the master sends pairs of register addresses
        and register data. The transaction is ended by a stop condition."
    */

    //ESP_LOGI(TAG_BME280, "Performing i2c write on register 0x%x (%lu bytes)", reg_addr, len);

    // The passed interface pointer is the esp i2c device handle for the bme280
    i2c_master_dev_handle_t bme280_handle = (i2c_master_dev_handle_t)intf_ptr;

    // The data to be written is register address (1 byte) + data (len bytes)
    uint8_t write_buffer[1 + len];

    // First byte is register address
    write_buffer[0] = reg_addr;

    // Remaining bytes contain data to be written
    memcpy(&write_buffer[1], reg_data, len);

    esp_err_t status = i2c_master_transmit(bme280_handle, write_buffer, sizeof(write_buffer), -1);
    if(status != ESP_OK){
        ESP_LOGE(TAG_BME280, "Could not probe bme280 sensor, check I2C connection. ESP error code 0x%x: %s", status, esp_err_to_name(status));
        return status;
    }

    return 0;
}

void sensor_bme280_i2c_delay(uint32_t period, void *intf_ptr){
    (void) intf_ptr;

    ESP_LOGI(TAG_BME280, "Delaying for %lu microseconds", period);

    // Period is given in microseconds.
    // Using ROM ets_delay_us to be able to set very short microseconds delays that are not possible with FreeRTOS vTaskDelay
    ets_delay_us(period);
}

void sensor_bme280_print_status(int8_t error){
    switch(error){
        case BME280_OK:
            return;
        case BME280_E_NULL_PTR:
            ESP_LOGE(TAG_BME280, "BME280_E_NULL_PTR");
            return;
        case BME280_E_COMM_FAIL:
            ESP_LOGE(TAG_BME280, "BME280_E_COMM_FAIL");
            return;
        case BME280_E_INVALID_LEN:
            ESP_LOGE(TAG_BME280, "BME280_E_INVALID_LEN");
            return;
        case BME280_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG_BME280, "BME280_E_DEV_NOT_FOUND");
            return;
        case BME280_E_SLEEP_MODE_FAIL:
            ESP_LOGE(TAG_BME280, "BME280_E_SLEEP_MODE_FAIL");
            return;
        case BME280_E_NVM_COPY_FAILED:
            ESP_LOGE(TAG_BME280, "BME280_E_NVM_COPY_FAILED");
            return;
        default:
            ESP_LOGE(TAG_BME280, "Unexpected BME280 error 0x%x", error);
            return;
    }
}

// Function to return measurement data from bme280 containing pressure, temperature, and humidity
int sensor_bme280_get_measurements(struct bme280_data *measurement_data){

    ESP_LOGI(TAG_BME280, "Reading bme280 measurement");

    int8_t bme280_status;
    uint8_t device_status;

    // Reading status of the bme280 device to check if measurement is ready
    ESP_LOGI(TAG_BME280, "Reading bme280 status...");
    bme280_status = bme280_get_regs(BME280_REG_STATUS, &device_status, 1, &bme280_g);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not read bme280 status register");
        return -1;
    }
    if(!(device_status & BME280_STATUS_MEAS_DONE)){
        ESP_LOGE(TAG_BME280, "bme280 not ready to read measurement");
        return -1;
    }

    // Delay to ensure measurement is ready even after status says ready
    ESP_LOGI(TAG_BME280, "Delaying before reading...");
    bme280_g.delay_us(measurement_time_us_g, NULL);


    // Read measurement from bme280
    ESP_LOGI(TAG_BME280, "Reading measurement...");
    bme280_status = bme280_get_sensor_data(BME280_ALL, measurement_data, &bme280_g);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not read measurement from bme280 sensor");
        return -1;
    }


    ESP_LOGI(TAG_BME280, "Temperature: %.2f °C", measurement_data->temperature);
    ESP_LOGI(TAG_BME280, "Pressure: %.2f hPa", measurement_data->pressure / 100.0);
    ESP_LOGI(TAG_BME280, "Humidity: %.2f %%RH", measurement_data->humidity);

    return 0;
}

// Returns pressure in Pascals, or NAN if there was an error reading the value
// Wrapper function for sensor_bme280_get_measurements that returns only the pressure value
double sensor_bme280_get_pressure(){
    int status;
    struct bme280_data measurement_data;
    status = sensor_bme280_get_measurements(&measurement_data);
    if(status != 0){ return NAN; }
    return measurement_data.pressure;
}

// Returns temperature in degrees C, or NAN if there was an error reading the value
// Wrapper function for sensor_bme280_get_measurements that returns only the temperature value
double sensor_bme280_get_temperature(){
    int status;
    struct bme280_data measurement_data;
    status = sensor_bme280_get_measurements(&measurement_data);
    if(status != 0){ return NAN; }
    return measurement_data.temperature;
}

// Returns humidity in % relative humidity, or NAN if there was an error reading the value
// Wrapper function for sensor_bme280_get_measurements that returns only the humidity value
double sensor_bme280_get_humidity(){
    int status;
    struct bme280_data measurement_data;
    status = sensor_bme280_get_measurements(&measurement_data);
    if(status != 0){ return NAN; }
    return measurement_data.humidity;
}

int sensor_init(){

    // i2c bus handle
    i2c_master_bus_handle_t bus_handle;

    // i2c bme280 device handle
    i2c_master_dev_handle_t bme280_handle;

    // initialize i2c bus
    sensor_init_i2c_master(&bus_handle);

    //  BME280 INITIALIZATION

    // add bme280 to i2c bus
    sensor_init_i2c_device(bus_handle, &bme280_handle, BME280_ADDRESS);

    // Probe bme280 i2c device to check if it is connected
    ESP_LOGI(TAG_BME280, "Probing bme280 device...");
    esp_err_t esp_status = i2c_master_probe(bus_handle, BME280_ADDRESS, -1);
    if(esp_status != ESP_OK){
        ESP_LOGE(TAG_BME280, "Could not probe bme280 sensor, check I2C connection. ESP error code 0x%x: %s", esp_status, esp_err_to_name(esp_status));
        return esp_status;
    }
    
    // Create bme280_dev struct containing driver initialization configuration for bme280
    struct bme280_dev bme280 = {
        .intf = BME280_I2C_INTF,
        .read = sensor_bme280_i2c_read,
        .write = sensor_bme280_i2c_write,
        .delay_us = sensor_bme280_i2c_delay,
        .intf_ptr = (void *)bme280_handle
    };

    int8_t bme280_status;

    // Initalize BME280 device
    ESP_LOGI(TAG_BME280, "Initializing bme280 device...");
    bme280_status = bme280_init(&bme280);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not initialize bme280 sensor");
        return bme280_status;
    }

    // Read settings of BME280
    struct bme280_settings settings;
    ESP_LOGI(TAG_BME280, "Reading bme280 settings...");
    bme280_status = bme280_get_sensor_settings(&settings, &bme280);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not read bme280 sensor settings");
        return bme280_status;
    }

    // Configure settings of BME280
    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    // Setting settings of BME280
    ESP_LOGI(TAG_BME280, "Setting bme280 settings...");
    bme280_status = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not set bme280 sensor settings");
        return bme280_status;
    }

    // Setting power mode of BME280
    ESP_LOGI(TAG_BME280, "Setting power mode of bme280...");
    bme280_status = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not set power mode of bme280");
        return bme280_status;
    }

    // Calculating measurement time (period between each measurement)
    uint32_t measurement_time_us;
    ESP_LOGI(TAG_BME280, "Calculating measurement time...");
    bme280_status = bme280_cal_meas_delay(&measurement_time_us, &settings);
    if(bme280_status != 0){
        sensor_bme280_print_status(bme280_status);
        ESP_LOGE(TAG_BME280, "Could not calculate bme280 measurement time");
        return bme280_status;
    }
    ESP_LOGI(TAG_BME280, "Measurement time is %lu microseconds\n", measurement_time_us);

    measurement_time_us_g = measurement_time_us;
    bme280_g = bme280;

    return 0;
}
