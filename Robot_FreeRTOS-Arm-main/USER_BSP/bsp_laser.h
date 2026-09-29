#ifndef BSP_LASER_H_
#define BSP_LASER_H_

#include <stdbool.h>
#include "hal_data.h"

#ifndef LASER_PIN
#define LASER_PIN BSP_IO_PORT_08_PIN_01
#endif

#ifndef LASER_FIRE_KEY_PIN
#define LASER_FIRE_KEY_PIN BSP_IO_PORT_00_PIN_00
#endif

#ifndef LASER_FIRE_KEY_ACTIVE_LEVEL
#define LASER_FIRE_KEY_ACTIVE_LEVEL BSP_IO_LEVEL_LOW
#endif

void BSP_Laser_Init(void);
void BSP_Laser_On(void);
void BSP_Laser_Off(void);
bool BSP_Laser_FireKey_IsPressed(void);

#endif /* BSP_LASER_H_ */
