#ifndef TEMP_MAP_MANAGER_H
#define TEMP_MAP_MANAGER_H

#include <stdbool.h>
#include "IntersectionMap.h"
#include "temp_types.h"

typedef struct {
    IntersectionMap map;
    bool loaded;
} MapManager;

bool map_manager_init(MapManager* manager, const char* xml_path);
bool map_manager_query_vehicle_context(
    const MapManager* manager,
    uint16_t x,
    uint16_t y,
    MapContext* out
);
bool map_manager_lane_allows_right(const MapManager* manager, const char* lanelet_id);
bool map_manager_lane_allows_unprotected_left(const MapManager* manager, const char* lanelet_id);
uint8_t map_manager_parse_lane_num(const char* lanelet_id);
uint8_t map_manager_parse_traffic_light_num(const char* tl_id);

#endif
