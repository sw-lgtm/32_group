/*
 * bluetooth.h
 *
 *  Created on: [Your Date]
 *      Author: [Your Name]
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

// 包含必要的头文件，这里我们只需要 main.h，因为它会引入所有 HAL 库的定义
#include "main.h"

/**
 * @brief 初始化蓝牙模块
 * @note  这个函数应该在 main 函数的初始化部分被调用
 */
void Bluetooth_Init(void);

/**
 * @brief 蓝牙模块的心跳/周期性任务
 * @note  这个函数可以放在 main 函数的 while(1) 循环中执行
 */
void Bluetooth_Heartbeat(void);

#endif /* INC_BLUETOOTH_H_ */