#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

// Snapshot of the most recent sensor reading
typedef struct {
    double temperature; // degrees C
    double pressure;    // Pascals
    double humidity;    // % relative humidity
    bool valid;         // false if no successful reading has been taken yet
} sensor_reading_t;

// Starts the periodic sensor polling task where period_ms is the delay between sensor reads.
// Must be called after sensor_init() has succeeded
bool sensor_task_start(uint32_t period_ms);

// Obtain  a thread-safe snapshot of the most recent sensor reading.
// Returns true if a valid reading was obtained, false if the cache is uninitialized or invalid
bool sensor_task_get_cached_reading(sensor_reading_t *out);

#endif // SENSOR_TASK_H
