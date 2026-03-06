// 包含系统核心头文件（如GPIO、NVIC等定义）
#include "sys.h"
// 包含串口驱动头文件（串口3初始化、中断等）
#include "usart3.h"	
// 包含GPS模块头文件（GPS数据结构体、函数声明）
#include "GPS.h" 

// 全局变量：存储串口3接收到的单个字节
char rxdatabufer;
// 全局变量：串口接收缓冲区的指针（索引）
u16 point1 = 0;

////////////////////////////////////////////////////////////////////////////////// 	 
// 如果使用uCOS实时操作系统，包含ucos的头文件
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					// uCOS系统头文件	  
#endif 


 
// 条件编译：若使能了串口3接收功能，则编译以下代码
#if EN_USART3_RX   // EN_USART3_RX在usart.h中定义，非0则使能接收
// 串口3接收缓冲区：最大存储USART_REC_LEN个字节（usart.h中定义）
char USART_RX_BUF[USART_REC_LEN];     // 接收缓冲数组
// 串口接收状态标记（16位）：
// bit15：接收完成标志；bit14：接收到0x0d（回车）；bit13~0：有效字节数
u16 USART_RX_STA=0;       // 接收状态寄存器	  
  
/******************************************************************
 * 函数名：uart3_init
 * 功能：串口3初始化（配置GPIO、USART3、NVIC，开启接收中断）
 ******************************************************************/
void uart3_init(u32 bound)
{
    // 定义GPIO、USART、NVIC初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	// 1. 修改：使能USART3和GPIOB的时钟（串口3挂载在APB1总线上）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
    // 2. 修改：配置USART3_TX（PB10）为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // 选择PB10引脚（串口3TX）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 引脚速度50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出模式
    GPIO_Init(GPIOB, &GPIO_InitStructure); // 初始化PB10
   
    // 3. 修改：配置USART3_RX（PB11）为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // 选择PB11引脚（串口3RX）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;// 浮空输入模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);  // 初始化PB11

   // 4. 修改：配置串口3的NVIC中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; // 选择串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;// 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		// 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// 使能中断通道
	NVIC_Init(&NVIC_InitStructure);	// 初始化NVIC

   // 配置USART3的通信参数
	USART_InitStructure.USART_BaudRate = bound; // 波特率（如9600、115200）
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;// 数据位8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;// 停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;// 无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// 收发模式

    USART_Init(USART3, &USART_InitStructure); // 初始化串口3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);// 开启串口3接收中断（RXNE）
    USART_Cmd(USART3, ENABLE);                    // 使能串口3 

	CLR_Buf();// 清空串口3接收缓冲区
	GPS_ClearBuffer();// 清空GPS模块缓冲区（替代原clrStruct函数）
}

/******************************************************************
 * 函数名：USART3_IRQHandler
 * 功能：串口3中断服务函数（处理GPS数据接收）
 ******************************************************************/
void USART3_IRQHandler(void) {
    u8 Res; // 存储接收的单个字节
    
    // 检查是否是串口3接收中断（RXNE）
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        Res = USART_ReceiveData(USART3); // 读取串口3接收的数据字节
        
        // 若接收到$（NMEA语句起始符），重置缓冲区指针
        if(Res == '$') {
            point1 = 0;
        }
        
        // 若缓冲区未存满，将数据存入缓冲区
        if(point1 < USART_REC_LEN - 1) {
            USART_RX_BUF[point1++] = Res;
        }
        
        // 若接收到\n（NMEA语句结束符），处理数据
        if(Res == '\n') {
            USART_RX_BUF[point1] = '\0';  // 强制添加字符串结束符
            
            // 检查是否是有效NMEA语句（GGA/RMC/GLL）
            char* gga_ptr = strstr((char*)USART_RX_BUF, "GGA");
            char* rmc_ptr = strstr((char*)USART_RX_BUF, "RMC");
            char* gll_ptr = strstr((char*)USART_RX_BUF, "GLL");
            
            // 若是有效语句，复制到GPS缓冲区并标记就绪
            if (gga_ptr || rmc_ptr || gll_ptr) {
                memset(gps_data.buf, 0, GPS_BUF_MAX_LEN); // 清空GPS缓冲区
                // 将NMEA数据复制到GPS缓冲区（防止越界）
                strncpy(gps_data.buf, (char*)USART_RX_BUF, MIN(point1, GPS_BUF_MAX_LEN - 1));
                gps_data.is_data_ready = true; // 标记GPS数据就绪
                
                // 调试打印接收到的语句类型（依然用串口1输出）
                if (gga_ptr) printf("GGA数据就绪\r\n");
                else if (rmc_ptr) printf("RMC数据就绪\r\n");
                else if (gll_ptr) printf("GLL数据就绪\r\n");
            }
            
            point1 = 0; // 重置缓冲区指针
            memset(USART_RX_BUF, 0, USART_REC_LEN); // 清空串口接收缓冲区
        }
    }
}

/******************************************************************
 * 函数名：Hand
 * 功能：串口命令识别函数（检查缓冲区是否包含指定字符串）
 ******************************************************************/
u8 Hand(char *a)                   
{ 
    // 若缓冲区包含字符串a，返回1；否则返回0
    if(strstr(USART_RX_BUF,a)!=NULL)
	    return 1;
	else
		return 0;
}

/******************************************************************
 * 函数名：CLR_Buf
 * 功能：清空串口3接收缓冲区和指针
 ******************************************************************/
void CLR_Buf(void)                          
{
	memset(USART_RX_BUF, 0, USART_REC_LEN);      // 清空接收缓冲区
    point1 = 0;                     // 重置缓冲区指针
}

// 删除原冗余的clrStruct()函数（已被GPS_ClearBuffer()替代）

#endif	// EN_USART3_RX结束


void USART3_SendByte(uint8_t data)
{
    // 等待发送数据寄存器（DR）为空（TXE标志置1表示为空）
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    // 将数据写入发送寄存器，硬件自动发送
    USART_SendData(USART3, data);
    // 可选：等待发送完成
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void USART3_SendString(uint8_t *str)
{
    // 循环发送字符串，直到遇到结束符'\0'
    while (*str != '\0')
    {
        USART3_SendByte(*str);  // 调用单个字节发送函数
        str++;                  // 指针指向-next字符
    }
    // 可选：发送换行符（方便串口助手查看，如"\r\n"）
    USART3_SendByte('\r');
    USART3_SendByte('\n');
}
