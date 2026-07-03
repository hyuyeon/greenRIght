#ifndef MQTT_THREAD_H
#define MQTT_THREAD_H

#include <stdint.h>


typedef struct {
    const char* host;
    int port;
    uint8_t vehicle_id;
} MqttThreadArgs;

void* mqttThread(void* arg);

#endif