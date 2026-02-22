#include "MPU6050.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"  // 确保包含寄存器定义（如MPU6050_WHO_AM_I等）

#define MPU6050_ADDRESS 0xD0  // 地址与参考代码一致

// 写寄存器（完全参考可工作代码）
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);  // 发送从机地址（写）
    MyI2C_SendByte(RegAddress);       // 发送寄存器地址
    MyI2C_SendByte(Data);             // 发送数据
    MyI2C_Stop();
}

// 读寄存器（完全参考可工作代码）
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);      // 发送从机地址（写）
    MyI2C_SendByte(RegAddress);           // 发送寄存器地址
    
    MyI2C_Start();                        // 重复起始
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);  // 发送从机地址（读）
    Data = MyI2C_ReceiveByte();           // 接收数据
    MyI2C_SendAck(1);                     // 发送非应答
    MyI2C_Stop();
    return Data;
}

// 初始化MPU6050（与参考代码一致）
void MPU6050_Init(void)
{
    MyI2C_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);    // 唤醒芯片，时钟源X轴
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);    // 所有轴不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);    // 采样率分频
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);        // 配置低通滤波器
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);   // 陀螺仪量程±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);  // 加速度计量程±16g
}

// 获取ID（与参考代码一致）
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);  // 读取WHO_AM_I寄存器
}

// 获取数据（与参考代码一致，逐位读取寄存器）
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}
