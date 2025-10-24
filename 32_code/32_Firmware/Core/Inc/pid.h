#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

// PID控制器结构体
typedef struct {
    // 1. 增益参数
    float Kp;
    float Ki;
    float Kd;

    // 2. 目标值
    float setpoint;

    // 3. 内部状态变量
    float integral;
    float prev_error;

    // 4. 输出限制 (非常重要，防止PWM值超出范围)
    float out_min;
    float out_max;

} PID_Controller;

/**
 * @brief 初始化一个PID控制器
 * @param pid: 指向PID控制器结构体的指针
 * @param kp, ki, kd: PID增益
 * @param out_min, out_max: 控制器输出的最小/最大值
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 计算PID控制器的输出
 * @param pid: 指向PID控制器结构体的指针
 * @param actual_value: 当前的测量值 (例如，当前轮速)
 * @param dt: 从上次计算到现在的间隔时间 (单位：秒)
 * @retval float: 计算出的控制量 (例如，PWM占空比)
 */
float PID_Calculate(PID_Controller *pid, float actual_value, float dt);

#endif /* INC_PID_H_ */