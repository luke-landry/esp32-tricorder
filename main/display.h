#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

bool app_display_init(void);
bool app_display_set_temperature(double val);
bool app_display_set_humidity(double val);
bool app_display_set_pressure(double val);

// message 1 used for BLE status
bool app_display_set_message_1(const char *msg);

// message 2 used for initialization status
bool app_display_set_message_2(const char *msg);

// message 3 used for error messages
bool app_display_set_message_3(const char *msg);

#endif // DISPLAY_H
