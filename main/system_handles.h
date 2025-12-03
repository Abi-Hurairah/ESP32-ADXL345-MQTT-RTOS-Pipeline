#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t xQueue_Sensordata;

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} sensor_data_t;

