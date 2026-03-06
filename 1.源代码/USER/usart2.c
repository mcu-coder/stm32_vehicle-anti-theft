#include "usart2.h"
#include "delay.h"

// 串口2接收缓存和计数
unsigned char Usart2RecBuf[USART2_RXBUFF_SIZE] = {0};
unsigned short RxCounter2 = 0;

//==========================================================
// 函数名称：uart2_Init
// 函数功能：串口2初始化（带中断接收）
// 入口参数：bound：波特率（如9600、115200）
// 返回参数：无
// 说明：PA2(TX)、PA3(RX)，开启接收中断
//==========================================================
void uart2_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2时钟

    // 2. 配置GPIO引脚
    // PA2(TX) 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA3(RX) 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置USART2参数
    USART_InitStructure.USART_BaudRate = bound;                     // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART2, &USART_InitStructure);

    // 4. 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;          // 串口2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 抢占优先级1（可根据需求调整）
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         // 子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开启接收中断和串口
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 开启接收非空中断
    USART_Cmd(USART2, ENABLE);                     // 使能串口2
}

//==========================================================
// 函数名称：uart2_send_byte
// 函数功能：串口2发送单个字节
// 入口参数：data：要发送的字节
// 返回参数：无
//==========================================================
void uart2_send_byte(unsigned char data)
{
    USART_SendData(USART2, data);
    // 等待发送完成
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

//==========================================================
// 函数名称：uart2_send
// 函数功能：串口2发送指定长度的数据
// 入口参数：buf：数据缓冲区；len：数据长度
// 返回参数：无
//==========================================================
void uart2_send(unsigned char *buf, u16 len)
{
    u16 i;
    for(i = 0; i < len; i++)
    {
        uart2_send_byte(buf[i]);
    }
    // 等待最后一个字节发送完成
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

//==========================================================
// 函数名称：USART2_IRQHandler
// 函数功能：串口2中断服务函数（接收数据）
// 入口参数：无
// 返回参数：无
// 说明：接收数据到缓存，防止缓存溢出
//==========================================================
void USART2_IRQHandler(void)
{
    unsigned char rec_data;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // 接收中断触发
    {
        rec_data = USART_ReceiveData(USART2); // 读取接收数据
        
        // 缓存未溢出时存储数据
        if(RxCounter2 < USART2_RXBUFF_SIZE)
        {
            Usart2RecBuf[RxCounter2++] = rec_data;
        }
        else
        {
            RxCounter2 = 0; // 溢出后重置计数（可根据需求修改处理逻辑）
            memset(Usart2RecBuf, 0, sizeof(Usart2RecBuf));
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 清除中断标志
    }
}


