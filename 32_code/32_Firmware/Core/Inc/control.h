#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

/* --- 控制状态枚举 --- */
typedef enum {
    CONTROL_STATE_IDLE,         // 停止/空闲
    CONTROL_STATE_OPEN_LOOP,    // 开环控制 (直接给PWM)
    CONTROL_STATE_PID,          // 普通PID闭环 (用于转弯)
    CONTROL_STATE_PID_STRAIGHT  // 航向角闭环的直行状态
} Control_State_t;

/* --- 函数原型声明 --- */
void Control_Init(void);
void Control_Update_10ms(void);
void Control_Set_TargetSpeed(float target_left, float target_right);
void Control_Turn_By_Angle(float angle_deg);
void Control_Set_OpenLoop(int pwm_left, int pwm_right);
void Control_Stop(void);

// Getter functions
float Control_Get_TargetSpeed_L(void);
float Control_Get_TargetSpeed_R(void);
int Control_Get_LastPwm_L(void);
int Control_Get_LastPwm_R(void);

#endif /* CONTROL_H */