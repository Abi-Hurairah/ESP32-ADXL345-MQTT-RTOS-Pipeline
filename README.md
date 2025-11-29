# Project Description
This project implements a thread-safe IoT pipeline using the ESP32's RTOS to publish real-time 3-axis acceleration data from an ADXL345 over MQTT. 

# Core Features
- Utilizes a custom I2C driver for the ADXL345 to enable 16-bit burst reads for efficiency and robust NACK error handling.
- Implements aysnchronous Wi-Fi and connects to an MQTT broker using the ESP-IDF native stack.
- Creates a FreeRTOS Producer-Consumer pattern with a Mutex to ensure thread-safe data transfer between a high-priority Sensor Task and a low-priority MQTT Publisher Task.

# Project Status 
Version 0.7 (active development)
I2C ADXL345, Wi-Fi Sation, and MQTT drivers are all completed and individually verified. Current task is merging these components using FreeRTOS synchronization primitives.

# Setup and Prerequisites

- Hardware Required: ESP32 or a compatible board, ADXL345 sensor
- ESP-IDF

Connect ESP32's GPIO 21 (SDA) the sensor's SDA pin and ESP32's GPIO 22 (SCL) to the sensor's SCL pin. Please read the datasheets of your board to identify SDA and SCL pins.

# Instructions
- Clone the repository and update submodules.
- Configure Wi-Fi credentials in station.h.
- Build and flash the firmware using idf.py build flash monitor.
- Configure the MQTT broker and subscriber(s) to subscribe to "adxl/data/raw".

# Project Roadmap
- MQTT Error Logging
- Modularize MQTT
- Finalize Producer-Consumer implementation using FreeRTOS Mutex
- Implement Dockerized MQTT Broker