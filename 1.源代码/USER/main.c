#include "stm32f10x.h"
#include "led.h"
#include "beep.h"
#include "usart.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "Modules.h"
#include "adcx.h"
#include "flash.h"
#include "usart2.h"
#include "usart3.h"
#include "timer.h"
#include "TIM2.h"  
#include "GPS.h"
#include "HW.h"
#include "vibrative.h"
#include "ds18b20.h"
 

#define KEY_Long1	11

#define KEY_1	1
#define KEY_2	2
#define KEY_3	3
#define KEY_4	4

#define FLASH_START_ADDR	0x0801f000	//写入的起始地址

 SensorModules sensorData;								//声明传感器数据结构体变量
 SensorThresholdValue Sensorthreshold;		//声明传感器阈值结构体变量
 DriveModules driveData;									//声明驱动器状态结构体变量

uint8_t mode = 1;	//系统模式  1自动  2手动  3设置
u8 dakai;//串口3使用的传递变量
u8 Flag_dakai;//串口3接收标志位
uint8_t is_secondary_menu = 0;  // 0一级菜单，1二级菜单
uint8_t secondary_pos = 1;      // 二级菜单光标位置（1-3对应时/分/秒）
uint8_t secondary_type = 0;   // 二级菜单类型：0=RTC时间，1=定时开启，2=定时关闭

uint8_t send_data[] = "1";
uint8_t send_data1[] = "2";

	extern unsigned char p[16];
	short temperature = 0; 	

//系统静态变量
//static uint8_t count_a = 1;  //自动模式按键数
 uint8_t count_m = 1;  //手动模式按键数
static uint8_t count_s = 1;	 //设置模式按键数
//static uint8_t last_mode = 0;      // 记录上一次的模式
//static uint8_t last_count_s = 0;   // 记录设置模式内上一次的页面

uint8_t auto_page = 1;  
// 静态变量：标记是否首次进入自动模式GPS页面
static uint8_t first_enter_auto_page2 = 0;

extern uint8_t vib_valid_trigger; 

// 新增：ESP8266发送频率控制
static uint32_t last_esp8266_send_time = 0; // 上次发送数据的时间戳
#define ESP8266_SEND_INTERVAL 1000          // 发送间隔：1000ms（1秒1次）

void ESP8266_SendAutoModeData(void);

/**
  * @brief  显示菜单内容
  * @param  无
  * @retval 无
  */
enum 
{
	AUTO_MODE = 1,
	MANUAL_MODE,
	SETTINGS_MODE
	
}MODE_PAGES;

 
	
  
/**
  * @brief  显示菜单1的固定内容
  * @param  无
  * @retval 无
  */
void OLED_autoPage1(void)		//自动模式菜单第一页
{

	//显示“温度：  C”
	OLED_ShowChinese(0,0,0,16,1);	//温
	OLED_ShowChinese(16,0,2,16,1);	//度
	OLED_ShowChar(32,0,':',16,1);
	

	 
	
	OLED_Refresh();
	
}
void OLED_autoPage2(void)   //自动模式菜单第二页
{
	
    OLED_ShowString(45, 0, "G P S", 16, 1); 
	
    // OLED显示"纬度"
	  OLED_ShowChinese(0,16,80,16,1);	
	  OLED_ShowChinese(16,16,82,16,1);	
    OLED_ShowChar(32,16,':',16,1);   
	
    // OLED显示"经度"
	  OLED_ShowChinese(0,32,81,16,1);	
	  OLED_ShowChinese(16,32,82,16,1);	
    OLED_ShowChar(32,32,':',16,1);   
     
}



void SensorDataDisplay1(void)		//传感器数据显示第一页
{
    SensorScan();	//获取传感器数据

     if (sensorData.temp_valid)
	{
    //显示温度数据
		OLED_ShowString(40,0,(u8*)p ,16,1);
	}


  
   
				
	//震动
		if (sensorData.vib ) 
        {
           OLED_ShowChinese(72,32,69,16,1);	//是 
        }
        else
        {					  
           OLED_ShowChinese(72,32,70,16,1);	//否 
        }
}

void SensorDataDisplay2(void)		//传感器数据显示第二页
{
  static uint32_t last_gps_time = 0; // 上次处理GPS数据的时间戳(ms)
    uint32_t current_time = delay_get_tick(); // 获取当前系统时间戳(ms)
    
    // 核心逻辑：首次进入GPS页面时，立即处理GPS数据（跳过500ms限制）
    if (first_enter_auto_page2) {
        if (gps_data.is_data_ready) {
            GPS_ParseNMEA();    // 解析GPS原始数据
        }
        GPS_DisplayAndSend();   // 显示并发送GPS数据
        last_gps_time = current_time; // 更新上次GPS处理时间戳
        first_enter_auto_page2 = 0; // 重置首次进入标志
        return;
    }
     

}

/**
  * @brief  显示手动模式设置界面1
  * @param  无
  * @retval 无
  */
void OLED_manualPage1(void)
{
	//显示“灯光”
	OLED_ShowChinese(16,0,83,16,1);	
	OLED_ShowChinese(32,0,84,16,1);	
	OLED_ShowChinese(48,0,28,16,1);	
	OLED_ShowChar(64,0,':',16,1);
 
	
}

/**
  * @brief  显示手动模式设置参数界面1
  * @param  无
  * @retval 无
  */
void ManualSettingsDisplay1(void)
{
	if(driveData.LED_Flag ==1)
	{
		OLED_ShowChinese(96,0,40,16,1); 	//开
	}
	else
	{
		OLED_ShowChinese(96,0,42,16,1); 	//关
	}
	 
}

/**
  * @brief  显示系统阈值设置界面1
  * @param  无
  * @retval 无
  */
void OLED_settingsPage1(void)
{

	
	//显示“温度阈值”
	OLED_ShowChinese(16,0,0,16,1);	
	OLED_ShowChinese(32,0,2,16,1);	
	OLED_ShowChinese(48,0,26,16,1);	
	OLED_ShowChinese(64,0,27,16,1);	
	OLED_ShowChar(80,0,':',16,1);

 
	

	
}

void OLED_settingsPage2(void)//显示系统阈值设置界面2
{


}
void OLED_settingsPage3(void)//显示系统阈值设置界面3
{

}

void SettingsThresholdDisplay1(void)//实际阈值1
{
	//显示温度阈值数值
	OLED_ShowNum(90,0, Sensorthreshold.tempValue, 2,16,1);
	
	if(driveData.Safe_Flag)
	{
		OLED_ShowChinese(90,16,40,16,1);
	}
	
 

}

void SettingsThresholdDisplay2(void)//实际阈值2
{

}

void SettingsThresholdDisplay3(void)//实际阈值3
{

}

/**
  * @brief  记录自动模式界面下按KEY2的次数
  * @param  无
  * @retval 返回次数
  */
uint8_t SetAuto(void)  
{
    // 检测到KEY2按下时执行页面切换
	  if(KeyNum == KEY_2)  
    {
        KeyNum = 0;         
        // 页面切换逻辑：1→2，2→1
        auto_page = (auto_page == 1) ? 2 : 1;  
        OLED_Clear();        // OLED清屏（避免页面内容重叠）
        delay_ms(5);         // 清屏后短暂延时，避免OLED绘制残留
        
        // 如果切换到GPS页面，标记首次进入
		    if(auto_page == 2) {
            first_enter_auto_page2 = 1;
        }
			
        // 绘制对应页面的固定内容
        if(auto_page == 1)
        {
            OLED_autoPage1();  // 绘制自动模式传感器页
        }
        else
        {
            OLED_autoPage2();  // 绘制自动模式GPS页
        }
    }
    return auto_page;  // 返回当前自动模式页面
}

/**
  * @brief  记录手动模式界面下按KEY2的次数
  * @param  无
  * @retval 返回次数
  */
uint8_t SetManual(void)  
{
	if(KeyNum == KEY_2)
	{
		KeyNum = 0;
		count_m++;
//		if (count_m == 1)
//		{
//			OLED_Clear();
//			
//		}
		if (count_m > 2)  		//一共可以控制的外设数量
		{
			OLED_Clear();
			count_m = 1;
		}
	}
	return count_m;
}

/**
  * @brief  记录阈值界面下按KEY2的次数
  * @param  无
  * @retval 返回次数
  */
uint8_t SetSelection(void)
{

    if(KeyNum == KEY_2 && is_secondary_menu == 0)
    {
        KeyNum = 0;
        count_s++;
        if (count_s > 2)
        {
            count_s = 1;
        }
    }
    return count_s;
}

/**
  * @brief  显示手动模式界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_manualOption(uint8_t num)
{
	switch(num)
	{
		case 1:	
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
		 
			default: break;
	}
}

/**
  * @brief  显示阈值界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_settingsOption(uint8_t num)
{
	static uint8_t prev_num = 1; 

    // 清除上一次光标（仅操作光标位置，不影响数据）
    switch(prev_num)
    {
        case 1: OLED_ShowChar(0, 0, ' ', 16, 1); break; // 系统时间行
        case 2: OLED_ShowChar(0, 16, ' ', 16, 1); break; // 温度阈值行
       
        default: break;
    }
	switch(num)
	{
		case 1:	
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
		case 2:	
			OLED_ShowChar(0, 0,' ',16,1);
			OLED_ShowChar(0,16,'>',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
	 

		default: break;
	}
	 prev_num = num;
    OLED_Refresh(); // 仅刷新光标，数据区域无变化
}

/**
  * @brief  自动模式控制函数
  * @param  无
  * @retval 无
  */
void AutoControl(void)//自动控制
{
	
    uint32_t current_time = delay_get_tick();       // 获取当前系统时间戳（2ms/次）
    #define ALARM_SEND_INTERVAL 3000               // 2秒=2000ms → 2000/2=1000个滴答
    
  
    static uint32_t last_theft_alarm_time = 0;
    static uint32_t last_vib_alarm_time = 0;
	
  if(sensorData.temp>Sensorthreshold.tempValue)
	{
		driveData.BEEP_Flag = 1;
	}
	else 
	{
			driveData.BEEP_Flag = 0;
	}
	
	
    if(driveData.Safe_Flag == 1 && sensorData.hw)
    {
        if(current_time - last_theft_alarm_time >= ALARM_SEND_INTERVAL)
        {
            USART3_SendString("A7:00002");          // 防盗报警语音
            last_theft_alarm_time = current_time;   // 更新防盗报警时间戳
            printf("触发防盗报警，发送A7:00002\n"); // 调试日志（可选）
        }
    }
    else
    {
        last_theft_alarm_time = 0; // 条件不满足时重置，下次可立即触发
    }
    
   
    if(vib_valid_trigger == 1) // 防抖后的有效震动触发
    {
       
        if(current_time - last_vib_alarm_time >= ALARM_SEND_INTERVAL)
        {
            USART3_SendString("A7:00003");          // 震动报警语音
            last_vib_alarm_time = current_time;     // 更新震动报警时间戳
            vib_valid_trigger = 0;                  // 清除触发标志（避免重复判断）
            printf("触发震动报警，发送A7:00003\n"); // 调试日志（可选）
        }
       
      
    }
   
    else
    {
        last_vib_alarm_time = 0;
    }

	
 
}

/**
  * @brief  手动模式控制函数
  * @param  无
  * @retval 无
  */
void ManualControl(uint8_t num)
{
	switch(num)
	{
		case 1:  
            if(KeyNum == KEY_3)
            {
                driveData.LED_Flag = 1;  
                KeyNum = 0;  
                printf("[按键] KEY3按下，LED_Flag置1\n");  
            }
            if(KeyNum == KEY_4)
            {
                driveData.LED_Flag = 0; 
                KeyNum = 0;  
                printf("[按键] KEY4按下，LED_Flag置0\n"); 
            }
            break;

		 

		default: break;
	}
}

/**
  * @brief  控制函数
  * @param  无
  * @retval 无
  */
void Control_Manager(void)
{
    if(driveData.LED_Flag )
    {	
        LED_On(); 
    }
    else 
    {
        LED_Off();
    }
		 
}

/**
  * @brief  阈值设置函数
  * @param  无
  * @retval 无
  */
void ThresholdSettings(uint8_t num)
{
	
	switch (num)
	{
		//温度
		case 1:
			if (KeyNum == KEY_3)
			{
				KeyNum = 0;
				Sensorthreshold.tempValue += 1;
				if (Sensorthreshold.tempValue > 40)
				{
					Sensorthreshold.tempValue = 10;
				}
			}
			 
			break;
			//防盗
			case 2:
			if (KeyNum == KEY_3)
			{
				KeyNum = 0;
				driveData.Safe_Flag = 1;

			}
			else if (KeyNum == KEY_4)
			{
				KeyNum = 0;
				driveData.Safe_Flag = 0;
						
			}
			break;

		
        default: break;
	}   
}
//flash读取
void FLASH_ReadThreshold()
{
	  Sensorthreshold.tempValue = FLASH_R(FLASH_START_ADDR);    // +0
    driveData.Safe_Flag = FLASH_R(FLASH_START_ADDR + 2);     // +2
  // +6
	
}

int main(void)
{ 
    SystemInit();//配置系统时钟为72M	
    delay_init(72);  // 系统时钟72MHz
    ADCX_Init();
    LED_Init();
     
    OLED_Clear();//清屏
    //flash读取
    delay_ms(100);
    FLASH_ReadThreshold();

    
    // 添加的状态管理变量
    static uint8_t last_mode = 0;  // 记录上一次模式
    static uint32_t last_sensor_time = 0; // 传感器扫描时间控制
    static uint32_t last_display_time = 0; // 显示刷新时间控制
    //超过以下值，恢复默认阈值
    if (Sensorthreshold.tempValue > 40 || driveData.Safe_Flag >1 )
       
    {
        FLASH_W(FLASH_START_ADDR, 30, 0);
        FLASH_ReadThreshold();
    }
    
   
    
    while (1)
    {	
      
        // ==================== 获取当前系统时间 ====================
        uint32_t current_time = delay_get_tick(); // 使用系统滴答计数
        
        // ==================== 优化传感器扫描频率 ====================
        if(current_time - last_sensor_time > 100) // 每200ms扫描一次传感器 (100 * 2ms = 200ms)
        {
            SensorScan(); 	//获取传感器数据
            last_sensor_time = current_time;
        }
       
        // ==================== 立即处理按键 ====================
        uint8_t current_key_num = KeyNum; // 保存当前按键值
        
        // 模式切换按键立即处理
        if(current_key_num != 0)
        {
            switch(mode)
            {
                case AUTO_MODE:
                    if(current_key_num == KEY_1)
                    {
                        mode = MANUAL_MODE;
                        count_m = 1;
                        // 切换到手动模式时关闭灯和蜂鸣器
                        driveData.LED_Flag = 0;
                        driveData.BEEP_Flag = 0;
                        KeyNum = 0; // 立即清除按键
                    }
                    else if(current_key_num == KEY_Long1)
                    {
                        mode = SETTINGS_MODE;
                        count_s = 1;
                        KeyNum = 0; // 立即清除按键
                    }
                    break;
                    
               
                    
                case SETTINGS_MODE:
                    // 设置模式内部按键在各自模式中处理
                    break;
            }
        }
        
        // ==================== 模式切换优化 ====================
        if(last_mode != mode)
        {
            OLED_Clear();
            last_mode = mode;
            
            // 立即绘制新模式的固定内容
            switch(mode)
            {
                case AUTO_MODE:
                    OLED_autoPage1();
								   
                    break;
                
                case SETTINGS_MODE:
                    OLED_settingsPage1();
                    break;
            }
            OLED_Refresh(); // 立即刷新显示
        }
        
        // ==================== 模式处理 ====================
        switch(mode)
        {
             case AUTO_MODE: // 自动模式
            {
                // 获取当前自动模式页面（处理KEY2切换）
                uint8_t curr_auto_page = SetAuto();
                if(curr_auto_page == 1)
                {
                    SensorDataDisplay1();	// 显示传感器数据+蓝牙发送
                }
                
                
                AutoControl();     // 执行自动控制逻辑（LED/蜂鸣器）
                Control_Manager(); // 执行硬件设备控制
								
   
                // 新增：定时发送数据到手机（1秒1次）
                if(current_time - last_esp8266_send_time >= ESP8266_SEND_INTERVAL)
                {
                    last_esp8266_send_time = current_time; // 更新发送时间戳
                }
								
                break;
            }    
                
            case MANUAL_MODE:
            {
                static uint8_t manual_page_initialized = 0;
                static uint8_t last_manual_count = 0;
                static uint8_t last_LED_Flag = 0;
                static uint8_t last_BEEP_Flag = 0;
                static uint8_t force_refresh = 0;  // 强制刷新标志
                
                // 模式切换时重新初始化
                if(last_mode != mode)
                {
                    manual_page_initialized = 0;
                    last_manual_count = 0;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                    force_refresh = 1;  // 设置强制刷新标志
                    
                    // 确保光标指向灯光
                    count_m = 1;
                    // 确保设备状态为关
                    driveData.LED_Flag = 0;
                    driveData.BEEP_Flag = 0;
                }
                
                uint8_t current_manual_count = SetManual();
                
                // 检查设备状态是否改变，如果改变则强制刷新显示
                uint8_t need_refresh = 0;
                if(driveData.LED_Flag != last_LED_Flag || driveData.BEEP_Flag != last_BEEP_Flag)
                {
                    need_refresh = 1;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                }
                
                // 确保页面已初始化或光标位置改变或设备状态改变或强制刷新时重新绘制
                if(!manual_page_initialized || current_manual_count != last_manual_count || need_refresh || force_refresh)
                {
                    OLED_manualPage1();          // 固定文字
                    OLED_manualOption(current_manual_count); // 光标
                    ManualSettingsDisplay1();    // 状态
                    manual_page_initialized = 1;
                    last_manual_count = current_manual_count;
                    force_refresh = 0;  // 清除强制刷新标志
                    OLED_Refresh(); // 强制刷新显示
                }
             
                
                Control_Manager();
                break;
            }
                
            case SETTINGS_MODE:
            {
                // 优化设置模式响应速度
                static uint8_t is_threshold_page_inited = 0;
                uint8_t curr_count_s = SetSelection();
                
                // 立即处理设置模式内的按键
                if(current_key_num != 0)
                {
                    if (is_secondary_menu == 1)
                    {
                        // 二级菜单按键立即处理
                        if (current_key_num == KEY_2 || current_key_num == KEY_3 || current_key_num == KEY_4)
                        {
                            // 这里根据你的二级菜单逻辑处理
                            // 处理完后立即刷新
                            OLED_Refresh();
                            KeyNum = 0;
                        }
                        
                    }
                    else
                    {
                        // 一级菜单按键立即处理
                        if (current_key_num == KEY_3 || current_key_num == KEY_4)
                        {
                            ThresholdSettings(curr_count_s);
                            SettingsThresholdDisplay1();
                            OLED_Refresh();
                            KeyNum = 0;
                        }
                        
                    }
                }
                
                // 正常显示逻辑
                if (is_secondary_menu == 1)
                {
                   
                }
               
                break;
            }
        }
        
        // ==================== 限制显示刷新频率 ====================
        if(current_time - last_display_time > 25) // 每50ms刷新一次显示 (25 * 2ms = 50ms)
        {
            // 所有模式都需要刷新显示
            OLED_Refresh();
            last_display_time = current_time;
        }
    }
}
