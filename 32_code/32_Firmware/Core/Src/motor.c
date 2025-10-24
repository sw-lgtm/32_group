#include "motor.h"

extern TIM_HandleTypeDef htim3;


void Load(int moto1, int moto2)   //moto left and right
{
	/**
     * moto left contol:
     *   forward: PWM1 = rpm, PWM2 = 0
     *   reverse: PWM1 = 0, PWM2 = rpm(abs)
     *   stop: PWM1 = 0, PWM2 = 0
     */
    if(moto1 < 0) {
        // forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -moto1); // PA6 - PWM1
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);    // PA7 - PWM2
    } 
    else if(moto1 > 0) {
        // reverse
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    // PA6 - PWM1
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, moto1); // PA7 - PWM2
    }
    else {
        // stop
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    
    /**
     * moto right contol:
     *   forward: PWM1 = rpm, PWM2 = 0
     *   reverse: PWM1 = 0, PWM2 = rpm(abs)
     *   stop: PWM1 = 0, PWM2 = 0
     */
    if(moto2 > 0) {
        // forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, moto2); // PB0 - PWM1
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);    // PB1 - PWM2
    } 
    else if(moto2 < 0) {
        // reverse
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    // PB0 - PWM1
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -moto2); // PB1 - PWM2
    }
    else {
        // stop
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}
