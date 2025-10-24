#include "encoder.h"
#include "tim.h"
#include <math.h>

/* --- 编码器和里程计的全局变量定义 --- */
// 速度 (单位: m/s)，带方向符号
float left_speed_signed_mps = 0.0f;
float right_speed_signed_mps = 0.0f;

// 累计位移 (单位: m)，带方向符号
float left_displacement_m = 0.0f;
float right_displacement_m = 0.0f;

// 累计转数 (绝对值)
float left_total_revs = 0.0f;
float right_total_revs = 0.0f;

// 里程计估算的机器人位姿
float robot_x = 0.0f;            // X坐标 (m)
float robot_y = 0.0f;            // Y坐标 (m)
float robot_theta = M_PI / 2.0f; // 航向角 (rad)，初始方向设为Y轴正方向(90度)

// 上次更新的时间戳
static uint32_t last_update_tick = 0;

// 从定时器读取编码器计数值并清零
static int Read_Encoder_Count(TIM_HandleTypeDef *htim)
{
    int temp = (short)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return temp;
}

void Encoder_Init(void)
{
    // 重置所有速度和位移变量
    left_speed_signed_mps = 0.0f;
    right_speed_signed_mps = 0.0f;
    left_displacement_m = 0.0f;
    right_displacement_m = 0.0f;
    left_total_revs = 0.0f;
    right_total_revs = 0.0f;
    
    // 重置里程计位姿
    robot_x = 0.0f;
    robot_y = 0.0f;
    robot_theta = M_PI / 2.0f;

    // 记录初始时间戳
    last_update_tick = HAL_GetTick();
}

void Encoder_Update(void)
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t delta_t_ms = current_tick - last_update_tick;
    last_update_tick = current_tick;
    
    // 如果时间间隔为0，则不进行计算，防止除零错误
    if (delta_t_ms == 0) return;
    
    int left_pulse = Read_Encoder_Count(&htim2);
    int right_pulse = Read_Encoder_Count(&htim4);

    float delta_t_s = delta_t_ms / 1000.0f;

    // --- 计算轮子的线位移和线速度 ---
    float left_delta_disp_m = (((float)left_pulse / ENCODER_PPR) * WHEEL_CIRCUMFERENCE);
    float right_delta_disp_m = (((float)right_pulse / ENCODER_PPR) * WHEEL_CIRCUMFERENCE);
    
    left_displacement_m += left_delta_disp_m;
    right_displacement_m += right_delta_disp_m;
    
    left_speed_signed_mps = left_delta_disp_m / delta_t_s;
    right_speed_signed_mps = right_delta_disp_m / delta_t_s;
    
    left_total_revs += fabsf(((float)left_pulse / ENCODER_PPR));
    right_total_revs += fabsf(((float)right_pulse / ENCODER_PPR));

    // =========================================================
    //                    里程计更新算法
    // =========================================================
    float delta_distance = (left_delta_disp_m + right_delta_disp_m) / 2.0f;
    float delta_theta = (right_delta_disp_m - left_delta_disp_m) / WHEELBASE;

    robot_theta += delta_theta;
    
    if (robot_theta > M_PI) robot_theta -= 2.0f * M_PI;
    if (robot_theta < -M_PI) robot_theta += 2.0f * M_PI;

    robot_x += delta_distance * cosf(robot_theta);
    robot_y += delta_distance * sinf(robot_theta);
}