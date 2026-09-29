#ifndef CHASSIS_H_
#define CHASSIS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_FORWARD,
    CHASSIS_MODE_BACKWARD,
    CHASSIS_MODE_TURN_LEFT,
    CHASSIS_MODE_TURN_RIGHT,
} chassis_mode_t;

void chassis_init(void);
void chassis_set_mode(chassis_mode_t mode, uint16_t speed_rpm);
void chassis_stop(void);
void chassis_periodic_10ms(void);
chassis_mode_t chassis_get_mode(void);
uint16_t chassis_get_speed_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H_ */
