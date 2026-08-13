#include "mqtt_handler_adapter.h"

#include <string.h>
#include "mqtt_client.h"

bool mqtt_adapter_init(MqttHandlerAdapter* adapter, const char* host, int port, uint8_t vehicle_id)
{
    if (!adapter) return false;
    memset(adapter, 0, sizeof(*adapter));
    if (mqtt_handler_init(host, port, vehicle_id) != 0) {
        return false;
    }
    adapter->vehicle_id = vehicle_id;
    adapter->initialized = true;
    return true;
}

void mqtt_adapter_cleanup(MqttHandlerAdapter* adapter)
{
    if (!adapter || !adapter->initialized) return;
    mqtt_handler_cleanup();
    adapter->initialized = false;
}

void mqtt_adapter_publish_self(const VehicleInfo* self_info)
{
    mqtt_publish_vehicle_info(self_info);
}

void mqtt_adapter_cleanup_stale(void)
{
    mqtt_cleanup_stale_vehicles();
}

bool mqtt_adapter_get_other(uint8_t vehicle_id, VehicleInfo* out)
{
    return mqtt_get_other_vehicle(vehicle_id, out);
}

bool mqtt_adapter_get_traffic_light(uint8_t tl_id, TrafficLight* out)
{
    return mqtt_get_traffic_light(tl_id, out);
}

int mqtt_adapter_get_others_count(void)
{
    return mqtt_get_others_count();
}
