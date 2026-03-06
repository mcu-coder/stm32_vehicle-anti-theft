#include "vibrative.h"

/**
 * @brief  初始化PA12引脚为输入模式
 * @note   默认配置为上拉输入，可根据需求修改为下拉/浮空
 * @param  无
 * @retval 无
 */
void Vibrative_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(VIBRATIVE_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = VIBRATIVE_GPIO_PIN;        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          
    GPIO_Init(VIBRATIVE_GPIO_PORT, &GPIO_InitStructure);    // 将配置参数写入GPIOA寄存器：完成PA12引脚的初始化  
}

/**
 * @brief  读取PA12引脚当前电平
 * @param  无
 * @retval uint8_t: 1表示高电平，0表示低电平
 */
uint8_t GetData(void)
{
    // 读取GPIOA的Pin12引脚电平
    return GPIO_ReadInputDataBit(VIBRATIVE_GPIO_PORT, VIBRATIVE_GPIO_PIN);
}

