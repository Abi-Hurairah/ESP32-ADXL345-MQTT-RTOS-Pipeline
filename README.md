# Project Overview: Robust ADXL345 I2C Driver
A modular and robust driver for ADXL345 3-Axis Accelerometer with the native ESP-IDF I2C Master. The driver utilizes the ADXL345's auto-increment feature through the internal pointer shift from register 0x32 to register 0x37 in the ADXL345 to read all three axes of 16-bit acceleration data in a single I2C burst transaction.

# Core Technical Implementation
The program begins by performing a critical Device ID (WHO_AM_I) verification to confirm the sensor's presence and correct address (0x53) before enabling measurement mode in the POWER_CTL register. The code employs an if (err == ESP_OK) structure within the continuous reading loop to avoid crashes during I2C failures such as NACKs or bus timeouts. Errors encountered will be logged and a recovery will be attempted on the next iteration to improve resilience in real world environments.
