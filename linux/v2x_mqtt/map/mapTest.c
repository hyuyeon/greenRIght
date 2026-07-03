#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "IntersectionMap.h"

#define INPUT_BUF_SIZE 128

static void trim_newline(char* s)
{
    if (!s) return;

    size_t len = strlen(s);
    if (len > 0 && s[len - 1] == '\n') {
        s[len - 1] = '\0';
    }
}

static int read_line(char* buf, size_t size)
{
    if (!fgets(buf, size, stdin)) {
        return 0;
    }

    trim_newline(buf);
    return 1;
}

static int read_int_prompt(const char* prompt, int* out)
{
    char buf[INPUT_BUF_SIZE];

    printf("%s", prompt);

    if (!read_line(buf, sizeof(buf))) {
        return 0;
    }

    *out = atoi(buf);
    return 1;
}

static int read_double_prompt(const char* prompt, double* out)
{
    char buf[INPUT_BUF_SIZE];

    printf("%s", prompt);

    if (!read_line(buf, sizeof(buf))) {
        return 0;
    }

    *out = atof(buf);
    return 1;
}

static int read_string_prompt(const char* prompt, char* out, size_t out_size)
{
    printf("%s", prompt);

    if (!read_line(out, out_size)) {
        return 0;
    }

    return 1;
}

static void menu_print(void)
{
    printf("\n");
    printf("========== IntersectionMap Test Menu ==========\n");
    printf("1. Query lane_id by (x, y)\n");
    printf("2. Query conflict zone ID list by lane_id\n");
    printf("3. Query traffic light ID by lane_id\n");
    printf("4. Check intersection center by (x, y)\n");
    printf("0. Exit\n");
    printf("==============================================\n");
}

static void test_get_lane_id(const IntersectionMap* map)
{
    double x;
    double y;

    if (!read_double_prompt("Input x: ", &x)) return;
    if (!read_double_prompt("Input y: ", &y)) return;

    const char* lane_id = NULL;

    MapQueryResult result = intersection_map_get_lane_id(
        map,
        x,
        y,
        &lane_id
    );

    printf("\n[RESULT] x=%.2f, y=%.2f\n", x, y);

    if (result == MAP_QUERY_LANE) {
        printf("lane_id = %s\n", lane_id);
    }
    else if (result == MAP_QUERY_INTERSECTION_CENTER) {
        printf("This point is in the intersection center. lane_id = NULL\n");
    }
    else {
        printf("No lane found for this point. lane_id = NULL\n");
    }
}

static void test_get_conflict_zones(const IntersectionMap* map)
{
    char lane_id[MAP_MAX_ID_LEN];

    if (!read_string_prompt("Input lane_id, e.g. L4: ", lane_id, sizeof(lane_id))) {
        return;
    }

    const char* cz_ids[MAP_MAX_CZ_PER_LANE];

    size_t cz_count = intersection_map_get_conflict_zone_ids(
        map,
        lane_id,
        cz_ids,
        MAP_MAX_CZ_PER_LANE
    );

    printf("\n[RESULT] lane_id = %s\n", lane_id);

    if (cz_count == 0) {
        printf("conflict zone = NULL\n");
        return;
    }

    printf("conflict zone count = %zu\n", cz_count);

    for (size_t i = 0; i < cz_count; i++) {
        printf("conflict_zone[%zu] = %s\n", i, cz_ids[i]);
    }
}

static void test_get_traffic_light(const IntersectionMap* map)
{
    char lane_id[MAP_MAX_ID_LEN];

    if (!read_string_prompt("Input lane_id, e.g. L4: ", lane_id, sizeof(lane_id))) {
        return;
    }

    const char* tl_id = intersection_map_get_traffic_light_id(
        map,
        lane_id
    );

    printf("\n[RESULT] lane_id = %s\n", lane_id);

    if (tl_id) {
        printf("traffic_light_id = %s\n", tl_id);
    } else {
        printf("traffic_light_id = NULL\n");
    }
}

static void test_is_intersection_center(const IntersectionMap* map)
{
    double x;
    double y;

    if (!read_double_prompt("Input x: ", &x)) return;
    if (!read_double_prompt("Input y: ", &y)) return;

    bool is_center = intersection_map_is_in_intersection_center(
        map,
        x,
        y
    );

    printf("\n[RESULT] x=%.2f, y=%.2f\n", x, y);

    if (is_center) {
        printf("This point is in the intersection center.\n");
    } else {
        printf("This point is not in the intersection center.\n");
    }
}

int main(int argc, char* argv[])
{
    const char* xml_path = "map.xml";

    if (argc >= 2) {
        xml_path = argv[1];
    }

    IntersectionMap map;

    if (!intersection_map_load_xml(&map, xml_path)) {
        fprintf(stderr, "[ERROR] map load failed: %s\n", xml_path);
        return 1;
    }

    printf("\n[INFO] XML map loaded: %s\n", xml_path);

    while (1) {
        int menu;

        menu_print();

        if (!read_int_prompt("Select menu: ", &menu)) {
            printf("\nInput closed\n");
            break;
        }

        switch (menu) {
        case 1:
            test_get_lane_id(&map);
            break;

        case 2:
            test_get_conflict_zones(&map);
            break;

        case 3:
            test_get_traffic_light(&map);
            break;

        case 4:
            test_is_intersection_center(&map);
            break;

        case 0:
            printf("Exit test\n");
            return 0;

        default:
            printf("Invalid menu. Try again.\n");
            break;
        }
    }

    return 0;
}