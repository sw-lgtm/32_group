#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include "stm32f4xx_hal.h"

void RPLIDAR_Init(void);
void RPLIDAR_StartScan(void);
void RPLIDAR_Stop(void);
void RPLIDAR_Process(void);

#endif /* INC_RPLIDAR_H_ */
