#ifndef _USART2_H_
#define _USART2_H_

#include "stm32f10x.h"
#include <string.h>

// 串口2接收缓存配置
#define USART2_RXBUFF_SIZE  1024  // 接收缓存大小，可根据需求调整

// 全局接收缓存和计数变量（供ESP8266驱动调用）
extern unsigned char Usart2RecBuf[USART2_RXBUFF_SIZE];
extern unsigned short RxCounter2;

// 函数声明
void uart2_Init(u32 bound);                  // 串口2初始化
void uart2_send_byte(unsigned char data);    // 发送单个字节
void uart2_send(unsigned char *buf, u16 len); // 发送指定长度数据

#endif

