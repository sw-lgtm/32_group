#include "bluetooth.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "encoder.h"
#include "control.h"

#define COMMAND_TIMEOUT 200
// =========================================================
//                   用于统计转弯次数的全局变量
// =========================================================
int turn_left_count = 0;
int turn_right_count = 0;
int turn_180_count = 0; // 【新增】180度掉头次数计数器
// =========================================================

static uint8_t rx_data_from_bt; 
static uint32_t last_motion_command_tick = 0;

void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_data_from_bt, 1);
}

void Bluetooth_Heartbeat(void)
{
    char msg_buffer[150];

    // 将航向角从弧度转换为度，更直观
    float theta_deg = robot_theta * 180.0f / M_PI;

    // 【修改】更新了心跳包格式，增加了180度转弯的计数显示
    // X,Y: 坐标 (米)
    // Th: Theta/航向角 (度)
    // L/R_C: Left/Right Turn Count (左右转90度次数)
    // U_C: U-Turn Count (掉头180度次数)
    snprintf(msg_buffer, sizeof(msg_buffer), 
            "X:%.2f, Y:%.2f, Th:%.1f, L_C:%d, R_C:%d, U_C:%d\r\n", 
            robot_x,
            robot_y,
            theta_deg,
            turn_left_count,
            turn_right_count,
            turn_180_count); // 【修改】加入了新的计数变量
    
    HAL_UART_Transmit(&huart3, (uint8_t*)msg_buffer, strlen(msg_buffer), 100);
}

void Bluetooth_Process_Timeout(void)
{
    if (last_motion_command_tick != 0 && (HAL_GetTick() - last_motion_command_tick > COMMAND_TIMEOUT))
    {
        Control_Stop();
        last_motion_command_tick = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        bool is_continuous_command = false;

        switch (rx_data_from_bt)
        {
            case 'w':
                Control_Set_TargetSpeed(0.3f, 0.3f);
                is_continuous_command = true;
                break;
            case 's':
                Control_Set_TargetSpeed(-0.3f, -0.3f);
                is_continuous_command = true;
                break;
            case 'a':
                Control_Set_OpenLoop(-45, 45);
                is_continuous_command = true;
                break;
            case 'd':
                Control_Set_OpenLoop(45, -45);
                is_continuous_command = true;
                break;
            
            case 'q': // 左转90度
                Control_Turn_By_Angle(-96.0f);
                turn_left_count++;
                break;
            case 'e': // 右转90度
                Control_Turn_By_Angle(85.0f);
                turn_right_count++;
                break;
            
            // 【新增】处理 'z' 指令的逻辑
            case 'z': // 掉头180度
                Control_Turn_By_Angle(200.0f);
                turn_180_count++;
                break;
            
            case 'p':
                Control_Stop();
                break;
        }
        
        if (is_continuous_command)
        {
            last_motion_command_tick = HAL_GetTick();
        }
        else
        {
            last_motion_command_tick = 0;
        }
        
        HAL_UART_Receive_IT(&huart3, &rx_data_from_bt, 1);
    }
}