#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

// Returns pressure in Pascals, or NAN if there was an error reading the value
double sensor_bme280_get_pressure(void);

// Returns temperature in degrees C, or NAN if there was an error reading the value
double sensor_bme280_get_temperature(void);

// Returns humidity in % relative humidity, or NAN if there was an error reading the value
double sensor_bme280_get_humidity(void);

// Initializes the sensor on the I2C bus and configures measurement settings
int sensor_init(void);

#endif // SENSOR_HANDLER_H