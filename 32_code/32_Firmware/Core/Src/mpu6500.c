#include "mpu6500.h"
#include <math.h>

#define ACCEL_SENS 16384.0f
#define G 9.80665f
#define GYRO_SENS 131.0f

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;
    // 检查WHO_AM_I寄存器
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, 0x75, 1, &check, 1, 100);
    if (check != 0x70 && check != 0x68) return 1; // 0x70/0x68为MPU6500/9250的ID

    // 解除休眠
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, 0x6B, 1, &data, 1, 100);

    // 配置加速度和陀螺仪量程（可选，默认2g/250dps）
    // data = 0x00; // ±2g
    // HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, 0x1C, 1, &data, 1, 100);
    // data = 0x00; // ±250dps
    // HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, 0x1B, 1, &data, 1, 100);

    return 0;
}

void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_Calib *calib) {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    MPU6500_Data data;
    for (int i = 0; i < 100; i++) {
        MPU6500_Read_All(hi2c, &data);
        ax_sum += data.Accel_X;
        ay_sum += data.Accel_Y;
        az_sum += data.Accel_Z;
        gx_sum += data.Gyro_X;
        gy_sum += data.Gyro_Y;
        gz_sum += data.Gyro_Z;
        HAL_Delay(10);
    }
    calib->ax_offset = ax_sum / 100;
    calib->ay_offset = ay_sum / 100;
    calib->az_offset = az_sum / 100;
    calib->gx_offset = gx_sum / 100;
    calib->gy_offset = gy_sum / 100;
    calib->gz_offset = gz_sum / 100;
    calib->velocity_x = 0.0f;
    calib->distance_x = 0.0f;
    calib->dt = 0.01f;
    calib->calibrated = 1;
    calib->accel_x_filt = 0.0f;
    calib->accel_y_filt = 0.0f;
    calib->accel_z_filt = 0.0f;
    calib->gyro_x_filt = 0.0f;
    calib->gyro_y_filt = 0.0f;
    calib->gyro_z_filt = 0.0f;
}

uint8_t MPU6500_Read_All(I2C_HandleTypeDef *hi2c, MPU6500_Data *data) {
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, MPU6500_ACCEL_REG, 1, buf, 14, 100) != HAL_OK)
        return 1;

    data->Accel_X = (int16_t)(buf[0] << 8 | buf[1]);
    data->Accel_Y = (int16_t)(buf[2] << 8 | buf[3]);
    data->Accel_Z = (int16_t)(buf[4] << 8 | buf[5]);
    // buf[6]和buf[7]是温度
    data->Gyro_X  = (int16_t)(buf[8] << 8 | buf[9]);
    data->Gyro_Y  = (int16_t)(buf[10] << 8 | buf[11]);
    data->Gyro_Z  = (int16_t)(buf[12] << 8 | buf[13]);
    return 0;
}

void MPU6500_GetCalibratedData(MPU6500_Data *raw, MPU6500_Calib *calib, MPU6500_Data *out) {
    out->Accel_X = raw->Accel_X - calib->ax_offset;
    out->Accel_Y = raw->Accel_Y - calib->ay_offset;
    out->Accel_Z = raw->Accel_Z - calib->az_offset;
    out->Gyro_X  = raw->Gyro_X - calib->gx_offset;
    out->Gyro_Y  = raw->Gyro_Y - calib->gy_offset;
    out->Gyro_Z  = raw->Gyro_Z - calib->gz_offset;
}

void MPU6500_Convert_Unit(MPU6500_Data *data) {
    data->Accel_X_mps2 = ((float)data->Accel_X / ACCEL_SENS) * G;
    data->Accel_Y_mps2 = ((float)data->Accel_Y / ACCEL_SENS) * G;
    data->Accel_Z_mps2 = ((float)data->Accel_Z / ACCEL_SENS) * G;
    data->Gyro_X_dps = (float)data->Gyro_X / GYRO_SENS;
    data->Gyro_Y_dps = (float)data->Gyro_Y / GYRO_SENS;
    data->Gyro_Z_dps = (float)data->Gyro_Z / GYRO_SENS;
}

void MPU6500_LowPassFilter(MPU6500_Data *data, MPU6500_Calib *calib, float alpha) {
    calib->accel_x_filt = alpha * data->Accel_X_mps2 + (1.0f - alpha) * calib->accel_x_filt;
    calib->accel_y_filt = alpha * data->Accel_Y_mps2 + (1.0f - alpha) * calib->accel_y_filt;
    calib->accel_z_filt = alpha * data->Accel_Z_mps2 + (1.0f - alpha) * calib->accel_z_filt;
    calib->gyro_x_filt = alpha * data->Gyro_X_dps + (1.0f - alpha) * calib->gyro_x_filt;
    calib->gyro_y_filt = alpha * data->Gyro_Y_dps + (1.0f - alpha) * calib->gyro_y_filt;
    calib->gyro_z_filt = alpha * data->Gyro_Z_dps + (1.0f - alpha) * calib->gyro_z_filt;
}

void MPU6500_Update_XMotion(MPU6500_Data *calib_data, MPU6500_Calib *calib) {
    float accel_x = calib->accel_x_filt;
    calib->velocity_x += accel_x * calib->dt;
    calib->distance_x += calib->velocity_x * calib->dt;
} 