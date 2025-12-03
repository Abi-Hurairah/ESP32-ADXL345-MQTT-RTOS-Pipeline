#ifndef MQTT_H
#define MQTT_H

#include "esp_mac.h"
#include "esp_system.h"
#include "esp_event.h"

void log_error_if_nonzero(const char *message, int error_code);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(void);
void mqtt_publish_task(void *pvParameters);

#endif