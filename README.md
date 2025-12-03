# Project Description
This project implements a thread-safe IoT pipeline using the ESP32's RTOS to publish real-time 3-axis acceleration data from an ADXL345 over MQTT. 

# Core Features
- Utilizes a custom I2C driver for the ADXL345 to enable 16-bit burst reads for efficiency and robust NACK error handling.
- Implements aysnchronous Wi-Fi and connects to an MQTT broker using the ESP-IDF native stack.
- Creates a FreeRTOS Producer-Consumer pattern with a Queue to ensure thread-safe data transfer between a high-priority Sensor Task and a low-priority MQTT Publisher Task.

# Project Status 
Version 0.9 (active development)
I2C ADXL345, Wi-Fi Station, and MQTT drivers are all completed and individually verified. Successfully merged the components. Next step is preventing the MQTT task to run before the board is connected to Wi-Fi.

# Setup and Prerequisites

- Hardware Required: ESP32 or a compatible board, ADXL345 sensor
- ESP-IDF

Connect ESP32's GPIO 21 (SDA) the sensor's SDA pin and ESP32's GPIO 22 (SCL) to the sensor's SCL pin. Please read the datasheets of your board to identify SDA and SCL pins.

# Instructions
- Clone the repository and update submodules.
- Configure Wi-Fi credentials in station.h and MQTT credentials in mqtt.c.
- Build and flash the firmware using idf.py build flash monitor.
- Configure the MQTT broker and subscriber(s) to subscribe to "test/response". This can be done with Mosquitto (Note: be sure to configure to allow external connections and to set up the ports) 

# Project Roadmap
- Modularized MQTT (done)
- Finalize Producer-Consumer implementation using FreeRTOS Queue (done)
- Implement Wi-Fi status check prior to initializing MQTT task
