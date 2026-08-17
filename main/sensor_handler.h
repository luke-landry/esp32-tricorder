#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "bme280.h"

// Performs a blocking I2C read of pressure, temperature, and humidity
// Returns 0 on success, non-zero on error
int sensor_bme280_get_measurements(struct bme280_data *measurement_data);

// Initializes the sensor on the I2C bus and configures measurement settings
int sensor_init(void);

#endif // SENSOR_HANDLER_H