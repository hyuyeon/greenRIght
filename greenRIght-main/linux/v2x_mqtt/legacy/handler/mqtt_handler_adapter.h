#ifndef TEMP_MQTT_HANDLER_ADAPTER_H
#define TEMP_MQTT_HANDLER_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

typedef struct {
    uint8_t vehicle_id;
    bool initialized;
} MqttHandlerAdapter;

bool mqtt_adapter_init(MqttHandlerAdapter* adapter, const char* host, int port, uint8_t vehicle_id);
void mqtt_adapter_cleanup(MqttHandlerAdapter* adapter);
void mqtt_adapter_publish_self(const VehicleInfo* self_info);
void mqtt_adapter_cleanup_stale(void);
bool mqtt_adapter_get_other(uint8_t vehicle_id, VehicleInfo* out);
bool mqtt_adapter_get_traffic_light(uint8_t tl_id, TrafficLight* out);
int mqtt_adapter_get_others_count(void);

#endif
