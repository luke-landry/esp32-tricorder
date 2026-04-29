# ESP32 Bluetooth Environment Data Reader ("Tricorder")

Transmits environment data (pressure, temperature, humidity) over BLE using a BME280 sensor connected to an ESP32 microcontroller via I2C. The NimBLE stack is used instead of the default Bluedroid stack because it supports only BLE (not Bluetooth Classic), making it more memory-efficient and requiring less code to implement.

### Wiring

In this project, the I2C connection uses GPIO pins 21 (SDA) and 22 (SCL) on the ESP32 development board. Also, the SDO pin on the BME280 is grounded, making the I2C address of the BME280 0x76.

#### Pins

![Pin setup](images/TricorderPinSetup.png)

#### Picture

![Wiring setup](images/TricorderWiringSetup.jpg)

### Setup

1. Install and set up the ESP-IDF extension for VSCode
2. Open this project
3. Open an ESP-IDF terminal (can be found in extension menu)
4. Connect an ESP32 board to a USB port on your computer
5. Run `idf.py -p /dev/ttyUSB0 flash monitor` to build, run, and monitor the project on the board. Replace `/dev/ttyUSB0` with the appropriate port, and if you are on Windows, replace it with the name of the COM port the ESP32 is connected to.
6. Use any BLE scanning app on a smartphone (e.g. nRF Connect) to read the pressure, temperature, and humidity characteristics from the Environmental Sensing service on the ESP32 (device is named "ESP32-Tricorder" in BLE scan advertisement).

### Screenshot
Using the nRF Connect app connected to the ESP32-Tricorder to view the environment data.

![Data readings in nRF Connect](images/DataScreenshotnRFConnect.png)

### Architecture

The firmware is split across three source files:

- `sensor_handler.c`: Initializes the I2C bus and BME280 sensor, and exposes functions to read the current pressure, temperature, and humidity values.
- `gatt_server.c`: Registers a standard Bluetooth Environmental Sensing service (UUID 0x181A) with three read-only
characteristics for each measurement. When a BLE client reads a characteristic, the callback reads a live value from the sensor and returns it encoded to the Bluetooth SIG specification.
- `main.c`: Initializes NimBLE, configures the GATT server and GAP advertisement, and handles connection lifecycle events.

The Bosch BME280 driver is included as a component in components/bme280/, providing a hardware-agnostic API that `sensor_handler.c` wires to the ESP-IDF I2C driver.