# Project Description
A modular and robust driver for ADXL345 3-Axis Accelerometer with the native ESP-IDF I2C Master. 

The driver utilizes the ADXL345's auto-increment feature through the internal pointer shift from register 0x32 to register 0x37 in the ADXL345 to read all three axes of 16-bit acceleration data (a total of 6 bytes in a single burst transaction).

# Setup

To start, connecting ESP32's GPIO 21 the sensor's SDA pin and ESP32's GPIO 22 to the sensor's SCL pin. Once you successfully upload the code, you should be able to see "I2C initialized successfully" followed by the I2C address of the ADXL345 sensor. Then, the measurement mode will be enabled in the POWER_CTL register.

The code employs an if (err == ESP_OK) structure within the continuous reading loop to avoid crashes during I2C failures such as NACKs or bus timeouts. Errors encountered will be logged and a recovery will be attempted on the next iteration to improve resilience in real world environments.