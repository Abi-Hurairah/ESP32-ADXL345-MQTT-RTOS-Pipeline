#include "esp_err.h"          // Defines esp_err_t type.
#include "driver/i2c_master.h"// Needed for i2c_master_dev_handle_t
#include "driver/gpio.h"      // Needed for GPIO_NUM_xx

#ifndef ADXL_H
#define ADXL_H

#define I2C_MASTER_SCL_IO           GPIO_NUM_22                 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21                 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          I2C_CLK_SRC_DEFAULT         /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADXL345_SENSOR_ADDR         0x53
#define ADXL345_DEVID_REG_ADDR      0x00
#define ADXL345_POWER_CTL_REG_ADDR  0x2D
#define ADXL345_DATAX0_REG_ADDR     0x32
#define ADXL345_MEASURE_BIT         3

esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

#endif