#ifndef __USART3_H
#define __USART3_H

#include "sys.h"
#include "GPS.h"  // 包含GPS.h，获取宏定义
#include <stdio.h>
#include <string.h>

// 串口核心宏定义（修改为串口3）
#define USART_REC_LEN   200  	// 最大接收字节数（≥100）
#define EN_USART3_RX    1		// 使能串口3接收

// 串口接收缓冲区声明（供中断使用）
extern char USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;         	// 接收状态标记	
extern char rxdatabufer;
extern u16 point1;

// 串口核心函数声明（修改为uart3_init）
void uart3_init(u32 bound);
void CLR_Buf(void);
void USART3_SendByte(uint8_t data);    // 发送单个字节
void USART3_SendString(uint8_t *str);  // 发送字符串
u8 Hand(char *a);

#endif
