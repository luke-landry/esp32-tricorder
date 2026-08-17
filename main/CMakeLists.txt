idf_component_register(
    SRCS "sensor_task.c" "display.c" "ble_task.c" "gatt_server.c" "main.c" "sensor_handler.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt bme280 esp_lcd esp_lcd_st7735
)
