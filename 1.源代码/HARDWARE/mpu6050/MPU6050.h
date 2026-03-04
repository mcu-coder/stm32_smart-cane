#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

#define MPU6050_ADDRESS 0xD0  // I2C从机地址

// 寄存器地址（参考MPU6050手册）
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_ACCEL_XOUT_H  0x3B  // 加速度X轴高位

// 函数声明
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                    int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
#endif

