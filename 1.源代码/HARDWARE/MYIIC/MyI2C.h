#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

// 函数声明与参考代码一致（无超时返回值，参数匹配）
void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);  // 无返回值，仅发送
uint8_t MyI2C_ReceiveByte(void);    // 无参数，仅接收
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

#endif
