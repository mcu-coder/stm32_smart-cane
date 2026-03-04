#include "Modules.h"
#include "ultrasonic.h"
#include "MPU6050.h"
#include "delay.h"
#include "usart.h"
#include <stdlib.h>
#include "stdio.h"

// 外部变量声明（保持不变）
extern SensorModules sensorData;
extern SensorThresholdValue Sensorthreshold;
extern DriveModules driveData;

static uint32_t last_sensor_update[6] = {0}; 
// 新增：5次平均的缓冲区（存储最近5次的单次平均）
#define FIVE_TIMES_AVG_N 5  // 固定取5次平均
static int16_t five_avg_buf[FIVE_TIMES_AVG_N] = {0};  // 缓冲区：存5次单次平均
static uint8_t five_avg_idx = 0;                      // 缓冲区索引（循环更新）

// 传感器更新间隔（保持不变）
#define LUX_UPDATE_INTERVAL     100   // 200ms
#define DISTANCE_UPDATE_INTERVAL 150  // 300ms
#define MPU6050_UPDATE_INTERVAL 150

float distance;

// MPU6050初始化（保持不变）
void MPU6050_InitModule(void)
{
    MPU6050_Init();
    sensorData.mpu6050_id = MPU6050_GetID();
    // 初始读取数据（初始化缓冲区用）
    MPU6050_GetData(&sensorData.ax, &sensorData.ay, &sensorData.az,
                   &sensorData.gx, &sensorData.gy, &sensorData.gz);
    // 初始化缓冲区：首次计算单次平均，填入所有缓冲区（避免初始值为0的干扰）
    int32_t sum_accel = (int32_t)sensorData.ax + sensorData.ay + sensorData.az;
    int16_t init_avg = (int16_t)(sum_accel / 3) - 500;
    for(uint8_t i=0; i<FIVE_TIMES_AVG_N; i++){
        five_avg_buf[i] = init_avg;
    }
}

// 核心修改：计算单次平均 + 5次平均
void MPU6050_UpdateData(void)
{
    int32_t sum_accel;          // 单次三轴总和（防溢出）
    int32_t sum_five_avg = 0;   // 5次单次平均的总和（防溢出）
    int16_t five_times_avg;     // 最终5次的平均数

    // 1. 读取原始数据（保持不变）
    MPU6050_GetData(&sensorData.ax, &sensorData.ay, &sensorData.az,
                   &sensorData.gx, &sensorData.gy, &sensorData.gz);

    // 2. 计算单次平均（保留用户原逻辑：(AX+AY+AZ)/3 - 500）
    sum_accel = (int32_t)sensorData.ax + sensorData.ay + sensorData.az;
    sensorData.accel_avg = (int16_t)(sum_accel / 3) - 500;  // 单次平均（已减500）

    // 3. 用缓冲区存储最近5次的单次平均（循环覆盖旧数据）
    five_avg_buf[five_avg_idx] = sensorData.accel_avg;  // 存入当前单次平均
    five_avg_idx = (five_avg_idx + 1) % FIVE_TIMES_AVG_N;  // 索引循环（0→1→2→3→4→0）

    // 4. 计算5次单次平均的总和（int32_t防溢出）
    for(uint8_t i=0; i<FIVE_TIMES_AVG_N; i++){
        sum_five_avg += five_avg_buf[i];
    }

    // 5. 计算5次的平均数（最终结果）
    five_times_avg = (int16_t)(sum_five_avg / FIVE_TIMES_AVG_N);

    // 6. 基于5次平均数做摔倒判断（替换原单次平均判断）
    if (five_times_avg < -300 || five_times_avg > 900) {
        sensorData.mpu_test = 1;  // 超出阈值→摔倒
    } else {
        sensorData.mpu_test = 0;  // 正常
    }

    // 打印验证（新增5次平均数，方便调试）
    printf("单次平均:%d | 5次平均:%d | mpu_test:%d\r\n",
           sensorData.accel_avg, five_times_avg, sensorData.mpu_test);
}

// SensorScan（保持不变）
void SensorScan(void)
{
    static uint32_t last_mpu_update = 0;
    uint32_t current_time = delay_get_tick();
    
    // 每150ms更新MPU6050数据（5次平均总耗时：150ms×5=750ms）
    if (current_time - last_mpu_update > MPU6050_UPDATE_INTERVAL)
    {
        MPU6050_UpdateData();
        last_mpu_update = current_time;
    }

    // 光照和距离读取（保持不变）
    sensorData.lux = LDR_LuxData();
    if(current_time - last_sensor_update[3] > DISTANCE_UPDATE_INTERVAL)
    {
        sensorData.distance = UltrasonicGetLength();
        last_sensor_update[3] = current_time;
    }
}

