/*
 * bluetooth.c
 *
 *  Created on: [Your Date]
 *      Author: [Your Name]
 */

#include "bluetooth.h"
#include "usart.h"
#include <string.h>

// 关键一步：包含电机模块的头文件，这样我们才能调用 Load() 函数
#include "motor.h" 

/* -------------------- Private Variables -------------------- */
static uint8_t rx_data_from_bt; 

/* -------------------- Public Functions (保持不变) -------------------- */
void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_data_from_bt, 1);
}

void Bluetooth_Heartbeat(void)
{
    char *msg = "STM32 Car is ready!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
}


/* -------------------- Callback Functions (核心修改区域) -------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 判断中断是否来自我们用于蓝牙的USART3
  if (huart->Instance == USART3)
  {
    // 根据接收到的数据执行操作
    switch (rx_data_from_bt)
    {
      /* --- 新增：小车运动控制 --- */
      case 'w': // 前进
        Load(50, 50);
        break;
      
      case 's': // 后退
        Load(-50, -50);
        break;

      case 'a': // 左转
        Load(25, 50);
        break;
        
      case 'd': // 右转
        Load(50, 25);
        break;

      case 'p': // 停止 (非常重要!)
        Load(0, 0);
        break;

      /* --- 保留：原来的LED控制 --- */
      case '1':
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        char *ack1 = "LED ON\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)ack1, strlen(ack1), 100);
        break;
        
      case '0':
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        char *ack0 = "LED OFF\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)ack0, strlen(ack0), 100);
        break;
        
      case 't':
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        char *ackt = "LED Toggled\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)ackt, strlen(ackt), 100);
        break;
        
      default:
        // 其他字符不响应，或者可以设置为停止
        // Load(0, 0); 
        break;
    }
    
    // 关键一步：再次启动中断接收，为下一次接收做准备
    HAL_UART_Receive_IT(&huart3, &rx_data_from_bt, 1);
  }
}