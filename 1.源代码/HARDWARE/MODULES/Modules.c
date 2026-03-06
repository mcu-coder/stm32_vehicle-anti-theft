#include "vibrative.h"
#include "Modules.h"
#include "HW.h"
#include "ds18b20.h"
#include "stdio.h"
#include "delay.h"
#include "GPS.h"

// 使用 extern 声明在 main.c 中定义的变量
extern SensorModules sensorData;
extern SensorThresholdValue Sensorthreshold;
extern DriveModules driveData;

// 在这里定义 p
unsigned char p[16] = " ";

// 状态变量
static uint8_t temp_read_state = 0;
static uint32_t temp_start_time = 0;
static uint32_t last_sensor_update[6] = {0};

// 传感器更新间隔（单位：系统滴答，2ms/次）
#define TEMP_UPDATE_INTERVAL    400   // 400*2ms = 800ms
#define HR_UPDATE_INTERVAL      50    // 50*2ms = 100ms  

// ==================== 震动传感器防抖配置（修正后） ====================
#define VIB_DEBOUNCE_TIME      10     // 防抖时间：10*2ms = 20ms（稳定20ms判定有效）
#define VIB_TRIGGER_HOLD_TIME  1000   // 触发后保持时间：1000*2ms = 2000ms（2秒）
#define VIB_LOCK_TIME          500    // 触发后额外锁定时间：500*2ms=1000ms（可选，避免重复触发）
#define VIB_POWER_LOCK_TIME    2500   // 5秒=5000ms，2ms/滴答 → 5000/2=2500
// 震动传感器状态机
typedef enum {
    VIB_STATE_IDLE,        // 空闲状态：无触发，可检测
    VIB_STATE_DETECTED,    // 检测到高电平：等待防抖时间确认
    VIB_STATE_TRIGGERED,   // 已确认触发：保持2秒有效状态
    VIB_STATE_LOCKED       // 锁定状态：暂不检测新触发（可选）
} Vib_StateTypeDef;

// 震动传感器静态变量
static Vib_StateTypeDef vib_state = VIB_STATE_IDLE;
static uint32_t vib_detected_start_time = 0;  // 检测到高电平的起始时间
static uint32_t vib_trigger_start_time = 0;   // 触发状态的起始时间（新增）
static uint32_t vib_locked_start_time = 0;    // 锁定状态的起始时间
static uint8_t vib_raw_data = 0;              // 原始电平数据
uint8_t vib_valid_trigger = 0;                // 防抖后的有效触发标志
static uint32_t vib_power_on_time = 0;        // 记录系统上电时间戳
static uint8_t vib_power_lock_init = 0;       // 上电时间初始化标志（确保只记录一次）

void SensorScan(void)
{
    static short temperature = 0;
    uint32_t current_time = delay_get_tick();
    
    // ==================== DS18B20温度读取（保留原有逻辑） ====================
    switch(temp_read_state)
    {
        case 0: // 启动温度转换
            if(current_time - last_sensor_update[0] > TEMP_UPDATE_INTERVAL)
            {
                DS18B20_Start(); // 启动转换
                temp_start_time = current_time;
                temp_read_state = 1;
                last_sensor_update[0] = current_time;
            }
            break;
            
        case 1: // 等待转换完成
            if(current_time - temp_start_time > 400) // 800ms转换时间
            {
                temp_read_state = 2;
            }
            break;
            
        case 2: // 读取温度值
            temperature = DS18B20_Get_Temp();
            if (temperature != 850) { 
                // 温度有效，更新传感器数据
                sensorData.temp = (float)temperature / 10;
                sprintf((char*)p, "%4.1f C", sensorData.temp);
                sensorData.temp_valid = 1; // 标记为有效
            } else {
                // 温度无效（仍为初始85℃），不更新数据，保持标志为0
                sensorData.temp_valid = 0;
            }
            temp_read_state = 0;
            last_sensor_update[0] = current_time;
            break;
    }
    
    // ==================== 人体感应传感器（保留原有逻辑） ====================
    sensorData.hw =  HW_GetData();
    
    // ========== 步骤1：初始化上电时间（仅第一次执行时记录） ==========
    if(!vib_power_lock_init)
    {
        vib_power_on_time = current_time; // 记录上电时的时间戳
        vib_power_lock_init = 1;          // 标记已初始化，避免重复记录
    }
    
    // ========== 步骤2：判断是否在5秒上电锁定期内 ==========
    if(current_time - vib_power_on_time < VIB_POWER_LOCK_TIME)
    {
        // 锁定期内：强制复位震动状态，屏蔽检测
        vib_state = VIB_STATE_IDLE;       // 回到空闲状态
        vib_valid_trigger = 0;            // 复位有效触发标志
        sensorData.vib = 0;               // 传感器数据置0
        return; // 跳过后续震动检测逻辑（也可改为continue，不影响其他传感器）
    }
    
    // ========== 步骤3：5秒后恢复正常震动检测逻辑（原有逻辑不变） ==========
    vib_raw_data = GetData(); // 读取原始电平
    
    switch(vib_state)
    {
        case VIB_STATE_IDLE:
            // 空闲状态：检测到高电平，进入待确认状态
            vib_valid_trigger = 0; // 空闲时置0
            if(vib_raw_data == 1)
            {
                vib_detected_start_time = current_time;
                vib_state = VIB_STATE_DETECTED;
            }
            break;
            
        case VIB_STATE_DETECTED:
            // 待确认状态：检查电平是否稳定持续防抖时间
            if(vib_raw_data == 0)
            {
                // 电平回落，判定为干扰，回到空闲状态
                vib_state = VIB_STATE_IDLE;
            }
            else if(current_time - vib_detected_start_time >= VIB_DEBOUNCE_TIME)
            {
                // 电平稳定20ms，判定为有效碰撞，进入触发保持状态
                vib_valid_trigger = 1;          // 置1：触发有效
                vib_trigger_start_time = current_time; // 记录触发起始时间
                vib_state = VIB_STATE_TRIGGERED;
            }
            break;
            
        case VIB_STATE_TRIGGERED:
            // 触发保持状态：持续2秒有效（vib_valid_trigger=1）
            vib_valid_trigger = 1; // 核心：保持2秒为1
            if(current_time - vib_trigger_start_time >= VIB_TRIGGER_HOLD_TIME)
            {
                // 2秒时间到，进入锁定状态（可选）
                vib_locked_start_time = current_time;
                vib_state = VIB_STATE_LOCKED;
            }
            break;
            
        case VIB_STATE_LOCKED:
            // 锁定状态：额外锁定1秒，避免短时间重复触发
            vib_valid_trigger = 0; // 锁定时置0
            if(current_time - vib_locked_start_time >= VIB_LOCK_TIME)
            {
                vib_state = VIB_STATE_IDLE; // 回到空闲，可重新检测
            }
            break;
            
        default:
            vib_state = VIB_STATE_IDLE; // 异常状态复位
            vib_valid_trigger = 0;
            break;
    }
    
    // 更新sensorData.vib为防抖后的有效状态（同步保持2秒）
    sensorData.vib = vib_valid_trigger;
}


