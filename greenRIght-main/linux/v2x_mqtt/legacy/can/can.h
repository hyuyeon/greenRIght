#ifndef TEMP_CAN_H
#define TEMP_CAN_H

#include <stdbool.h>
#include "types.h"

int can_handler_init(const char* ifname);
void can_handler_cleanup(void);
bool can_handler_get_ego(EgoVehicle* out);

#endif
