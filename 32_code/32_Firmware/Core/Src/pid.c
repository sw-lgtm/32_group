#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Calculate(PID_Controller *pid, float actual_value, float dt)
{
    // 1. 计算误差
    float error = pid->setpoint - actual_value;

    // 2. 计算积分项 (带抗饱和积分 wind-up)
    pid->integral += error * dt;
    // --- 积分限幅 ---
    if (pid->integral > pid->out_max) {
        pid->integral = pid->out_max;
    } else if (pid->integral < pid->out_min) {
        pid->integral = pid->out_min;
    }

    // 3. 计算微分项
    float derivative = (error - pid->prev_error) / dt;

    // 4. 计算总输出
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 5. 更新上一次误差
    pid->prev_error = error;

    // 6. 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}