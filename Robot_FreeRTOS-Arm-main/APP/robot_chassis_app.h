#ifndef ROBOT_CHASSIS_APP_H_
#define ROBOT_CHASSIS_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

void robot_chassis_app_init(void);
bool robot_chassis_cmd_drive(chassis_mode_t mode, uint16_t speed_rpm);
void robot_chassis_cmd_stop(void);
void robot_chassis_print_help(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CHASSIS_APP_H_ */

