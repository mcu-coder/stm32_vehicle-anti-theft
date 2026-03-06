#ifndef VIBRATIVE_H
#define VIBRATIVE_H

#include "stm32f10x.h"  

/************************** 引脚宏定义 **************************/
// 引脚配置，可根据硬件修改
#define VIBRATIVE_GPIO_PORT    GPIOA
#define VIBRATIVE_GPIO_PIN     GPIO_Pin_1
#define VIBRATIVE_GPIO_CLK     RCC_APB2Periph_GPIOA

/************************** 函数声明 **************************/
/**
 * @brief  初始化PA12引脚为输入模式（上拉/下拉可配置）
 * @param  无
 * @retval 无
 */
void Vibrative_Init(void);

/**
 * @brief  检测PA12引脚的电平状态
 * @param  无
 * @retval uint8_t: 1-高电平 / 0-低电平
 */
uint8_t GetData(void); // 声明读取引脚电平函数（1=高电平，0=低电平）

#endif // VIBRATIVE_H

