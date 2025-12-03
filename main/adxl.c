#include "adxl.h" 
#include "system_handles.h"
#include "esp_log.h" 
#include "driver/i2c_master.h"

static const char *TAG = "SENSOR_TASK";

/**
 * @brief Read a sequence of bytes from a adxl345 sensor registers
 */
esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a adxl345 sensor register
 */
esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void read_sensor_task(void *pvParameters)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;
    uint8_t data_buffer[6];
    esp_err_t err;
    BaseType_t xReturn;
    sensor_data_t data_packet = {80};

    while(1)
    {
        // 1. Attempt to read the data and capture the return status
        err = adxl345_register_read(dev_handle, ADXL345_DATAX0_REG_ADDR, data_buffer, 6);

        // 2. Check if the read was successful (err == ESP_OK)
        if (err == ESP_OK) 
        {
        data_packet.x_axis = (data_buffer[1] << 8) | data_buffer[0];
        data_packet.y_axis = (data_buffer[3] << 8) | data_buffer[2];
        data_packet.z_axis = (data_buffer[5] << 8) | data_buffer[4];

        ESP_LOGI(TAG, "X=%d, Y=%d, Z=%d", data_packet.x_axis, data_packet.y_axis, data_packet.z_axis);

        xReturn = xQueueOverwrite(xQueue_Sensordata, (void *)&data_packet);
        if (xReturn == pdTRUE)
        {
            printf("%d: Item Send SUCCESS\n", data_packet.x_axis);
            printf("%d: Item Send SUCCESS\n", data_packet.y_axis);
            printf("%d: Item Send SUCCESS\n", data_packet.z_axis);
        }
        else
        {
            ESP_LOGI(TAG, "Queue Full! Data discarded.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}