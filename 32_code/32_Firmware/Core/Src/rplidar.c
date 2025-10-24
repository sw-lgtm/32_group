#include "rplidar.h"
#include "usart.h"
#include "bluetooth.h"
#include <string.h>

#define RPLIDAR_CMD_STOP  0x25
#define RPLIDAR_CMD_SCAN  0x20
#define RPLIDAR_FRAME_LEN 5

static uint8_t rplidar_rx_buf[RPLIDAR_FRAME_LEN];

void RPLIDAR_Init(void)
{
}

static void RPLIDAR_SendCmd(uint8_t cmd)
{
  uint8_t header[2] = {0xA5, cmd};
  HAL_UART_Transmit(&huart6, header, 2, 100);
}

void RPLIDAR_StartScan(void)
{
  RPLIDAR_SendCmd(RPLIDAR_CMD_SCAN);
}

void RPLIDAR_Stop(void)
{
  RPLIDAR_SendCmd(RPLIDAR_CMD_STOP);
}

void RPLIDAR_Process(void)
{
  if (HAL_UART_Receive(&huart6, rplidar_rx_buf, RPLIDAR_FRAME_LEN, 100) == HAL_OK)
  {
    if ((rplidar_rx_buf[0] & 0x01) == 0x01)
    {
      uint16_t angle_q6 = ((rplidar_rx_buf[1] >> 1) | (rplidar_rx_buf[2] << 7));
      uint16_t dist_q2 = (rplidar_rx_buf[3] | (rplidar_rx_buf[4] << 8));
      uint8_t bt_data[4];
			char buffer[32];
//      bt_data[0] = dist_q2 & 0xFF;
//      bt_data[1] = (dist_q2 >> 8) & 0xFF;
//      bt_data[2] = angle_q6 & 0xFF;
//      bt_data[3] = (angle_q6 >> 8) & 0xFF;
			float angle = (angle_q6 >> 1) / 64.0f;
      float distance =dist_q2 / 4.0f;
			snprintf(buffer, sizeof(buffer), "A:%.2f,D:%.2f m\r\n", angle, distance/1000.0f);
      HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
//      HAL_UART_Transmit(&huart3, bt_data, 4, 10);
    }
  }
} 