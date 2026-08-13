#include "map_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, dst_size, "%s", src);
}

static const MapLanelet* find_lanelet(const MapManager* manager, const char* lanelet_id)
{
    if (!manager || !manager->loaded || !lanelet_id) return NULL;
    for (int i = 0; i < manager->map.lanelet_count; i++) {
        if (strcmp(manager->map.lanelets[i].id, lanelet_id) == 0) {
            return &manager->map.lanelets[i];
        }
    }
    return NULL;
}

static TempDirection parse_direction(const char* maneuver, bool unprotected_left)
{
    if (!maneuver) return TEMP_DIRECTION_UNKNOWN;
    if (strcmp(maneuver, "straight_right") == 0) return TEMP_DIRECTION_RIGHT;
    if (strcmp(maneuver, "straight_left") == 0 && unprotected_left) return TEMP_DIRECTION_UNPROTECTED_LEFT;
    return TEMP_DIRECTION_STRAIGHT;
}

bool map_manager_init(MapManager* manager, const char* xml_path)
{
    if (!manager || !xml_path) return false;
    memset(manager, 0, sizeof(*manager));

    manager->loaded = intersection_map_load_xml(&manager->map, xml_path);
    if (!manager->loaded) {
        fprintf(stderr, "[MapManager] map load failed: %s\n", xml_path);
        return false;
    }

    printf("[MapManager] loaded: lanelets=%d areas=%d tls=%d cz=%d\n",
           manager->map.lanelet_count,
           manager->map.area_count,
           manager->map.traffic_light_count,
           manager->map.conflict_zone_count);
    return true;
}

bool map_manager_query_vehicle_context(
    const MapManager* manager,
    uint16_t x,
    uint16_t y,
    MapContext* out
)
{
    if (!manager || !manager->loaded || !out) return false;
    memset(out, 0, sizeof(*out));

    const char* lane_id = NULL;
    MapQueryResult result = intersection_map_get_lane_id(&manager->map, x, y, &lane_id);
    out->in_intersection_center = (result == MAP_QUERY_INTERSECTION_CENTER);

    if (result != MAP_QUERY_LANE || !lane_id) {
        return out->in_intersection_center;
    }

    const MapLanelet* lanelet = find_lanelet(manager, lane_id);
    if (!lanelet) return false;

    out->found = true;
    safe_copy(out->lanelet_id, sizeof(out->lanelet_id), lanelet->id);
    out->unprotected_left = lanelet->unprotected_left;
    out->direction = parse_direction(lanelet->maneuver, out->unprotected_left);
    safe_copy(out->traffic_light_id, sizeof(out->traffic_light_id), lanelet->traffic_light_id);

    const char* cz_ids[TEMP_MAX_CONFLICT_ZONES];
    size_t cz_count = intersection_map_get_conflict_zone_ids(
        &manager->map,
        lanelet->id,
        cz_ids,
        TEMP_MAX_CONFLICT_ZONES
    );

    out->conflict_zone_count = (uint8_t)cz_count;
    for (size_t i = 0; i < cz_count; i++) {
        safe_copy(out->conflict_zone_ids[i], sizeof(out->conflict_zone_ids[i]), cz_ids[i]);
    }

    return true;
}

bool map_manager_lane_allows_right(const MapManager* manager, const char* lanelet_id)
{
    const MapLanelet* lanelet = find_lanelet(manager, lanelet_id);
    return lanelet && strcmp(lanelet->maneuver, "straight_right") == 0;
}

bool map_manager_lane_allows_unprotected_left(const MapManager* manager, const char* lanelet_id)
{
    const MapLanelet* lanelet = find_lanelet(manager, lanelet_id);
    return lanelet && strcmp(lanelet->maneuver, "straight_left") == 0 && lanelet->unprotected_left;
}

uint8_t map_manager_parse_lane_num(const char* lanelet_id)
{
    if (!lanelet_id || lanelet_id[0] != 'L') return TEMP_INVALID_ID;
    long value = strtol(lanelet_id + 1, NULL, 10);
    if (value < 0 || value > 255) return TEMP_INVALID_ID;
    return (uint8_t)value;
}

uint8_t map_manager_parse_traffic_light_num(const char* tl_id)
{
    if (!tl_id || tl_id[0] == '\0') return TEMP_INVALID_ID;
    if (tl_id[0] == 'T' && tl_id[1] == 'L') {
        long value = strtol(tl_id + 2, NULL, 10);
        if (value >= 0 && value <= 255) return (uint8_t)value;
    }
    return TEMP_INVALID_ID;
}
