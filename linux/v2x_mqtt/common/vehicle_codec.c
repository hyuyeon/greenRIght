#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "vehicle_codec.h"

/* ============================ VehicleInfo ============================ */

/* VehicleInfo 구조체를 cJSON 객체로 변환한다.
 * 반환된 객체의 소유권은 호출자에게 있으며, 사용 후 cJSON_Delete()가 필요하다. */
cJSON *vehicle_info_to_json(const VehicleInfo *v)
{
    if (!v) return NULL;

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    /* MQTT payload에서 필드명을 그대로 볼 수 있도록 구조체 멤버를 JSON key로 매핑한다. */
    cJSON_AddNumberToObject(root, "vehicle_id", v->vehicle_id);
    cJSON_AddNumberToObject(root, "x", v->x);
    cJSON_AddNumberToObject(root, "y", v->y);
    cJSON_AddNumberToObject(root, "speed", v->speed);
    cJSON_AddNumberToObject(root, "heading", v->heading);
    cJSON_AddNumberToObject(root, "lane", v->lane);
    cJSON_AddNumberToObject(root, "direction", v->direction);
    cJSON_AddNumberToObject(root, "cz_x", v->cz_x);
    cJSON_AddNumberToObject(root, "cz_y", v->cz_y);
    cJSON_AddNumberToObject(root, "linked_tl", v->linked_tl);
    cJSON_AddNumberToObject(root, "timestamp", v->timestamp);

    return root;
}

char *vehicle_info_to_json_string(const VehicleInfo *v)
{
    cJSON *root = vehicle_info_to_json(v);
    if (!root) return NULL;
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}

/* 공용 헬퍼: 정수 필드를 안전하게 읽는다 (없으면 false) */
static bool get_uint_field(const cJSON *root, const char *key, unsigned long *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (unsigned long)item->valuedouble;
    return true;
}

bool vehicle_info_from_json(const cJSON *root, VehicleInfo *out)
{
    if (!root || !out) return false;
    unsigned long val;

    if (!get_uint_field(root, "vehicle_id", &val)) return false;
    out->vehicle_id = (uint8_t)val;

    if (!get_uint_field(root, "x", &val)) return false;
    out->x = (uint16_t)val;

    if (!get_uint_field(root, "y", &val)) return false;
    out->y = (uint16_t)val;

    if (!get_uint_field(root, "speed", &val)) return false;
    out->speed = (uint8_t)val;

    if (!get_uint_field(root, "heading", &val)) return false;
    out->heading = (uint16_t)val;

    if (!get_uint_field(root, "lane", &val)) return false;
    out->lane = (uint8_t)val;

    if (!get_uint_field(root, "direction", &val)) return false;
    out->direction = (uint8_t)val;

    if (!get_uint_field(root, "cz_x", &val)) return false;
    out->cz_x = (uint16_t)val;

    if (!get_uint_field(root, "cz_y", &val)) return false;
    out->cz_y = (uint16_t)val;

    if (!get_uint_field(root, "linked_tl", &val)) return false;
    out->linked_tl = (uint8_t)val;

    if (!get_uint_field(root, "timestamp", &val)) return false;
    out->timestamp = (uint16_t)val;

    return true;
}

bool vehicle_info_from_json_string(const char *json_str, VehicleInfo *out)
{
    if (!json_str || !out) return false;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;
    bool ok = vehicle_info_from_json(root, out);
    cJSON_Delete(root);
    return ok;
}

char *vehicle_info_array_to_json_string(const VehicleInfo *arr, int count)
{
    if (!arr && count > 0) return NULL;

    cJSON *jarr = cJSON_CreateArray();
    if (!jarr) return NULL;

    for (int i = 0; i < count; i++) {
        cJSON *item = vehicle_info_to_json(&arr[i]);
        if (!item) {
            cJSON_Delete(jarr);
            return NULL;
        }
        cJSON_AddItemToArray(jarr, item);
    }

    char *str = cJSON_PrintUnformatted(jarr);
    cJSON_Delete(jarr);
    return str;
}

int vehicle_info_array_from_json_string(const char *json_str, VehicleInfo *out, int max_count)
{
    if (!json_str || !out || max_count <= 0) return -1;

    cJSON *jarr = cJSON_Parse(json_str);
    if (!jarr || !cJSON_IsArray(jarr)) {
        if (jarr) cJSON_Delete(jarr);
        return -1;
    }

    int n = 0;
    cJSON *item;
    cJSON_ArrayForEach(item, jarr) {
        if (n >= max_count) break;
        if (vehicle_info_from_json(item, &out[n])) {
            n++;
        }
    }

    cJSON_Delete(jarr);
    return n;
}

/* ============================ TrafficLight ============================ */

cJSON *traffic_light_to_json(const TrafficLight *tl)
{
    if (!tl) return NULL;
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddNumberToObject(root, "type_mask", tl->type_mask);
    cJSON_AddNumberToObject(root, "color", tl->color);
    cJSON_AddNumberToObject(root, "time_left", tl->time_left);

    return root;
}

char *traffic_light_to_json_string(const TrafficLight *tl)
{
    cJSON *root = traffic_light_to_json(tl);
    if (!root) return NULL;
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}

bool traffic_light_from_json(const cJSON *root, TrafficLight *out)
{
    if (!root || !out) return false;
    unsigned long val;

    if (!get_uint_field(root, "type_mask", &val)) return false;
    out->type_mask = (uint8_t)val;

    if (!get_uint_field(root, "color", &val)) return false;
    out->color = (uint8_t)val;

    if (!get_uint_field(root, "time_left", &val)) return false;
    out->time_left = (uint8_t)val;

    return true;
}

bool traffic_light_from_json_string(const char *json_str, TrafficLight *out)
{
    if (!json_str || !out) return false;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;
    bool ok = traffic_light_from_json(root, out);
    cJSON_Delete(root);
    return ok;
}