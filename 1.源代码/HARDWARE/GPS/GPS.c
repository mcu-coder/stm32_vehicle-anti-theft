// 包含GPS模块核心头文件，定义了GPS数据结构体等基础类型
#include "GPS.h"
// 包含串口3驱动头文件（原usart.h已改为串口3）
#include "usart3.h"
#include "usart.h"
// 包含串口2驱动头文件（用于蓝牙发送经纬度）
#include "usart2.h"
// 包含OLED显示驱动头文件（用于GPS数据可视化）
#include "oled.h"   
// 包含延时/滴答定时器头文件（用于时间戳、超时判断）
#include "delay.h"  
// 字符串处理标准库（strstr、strtok等函数）
#include <string.h>
// 标准输入输出库（snprintf、printf等函数）
#include <stdio.h>
// 标准库（atof等数值转换函数）
#include <stdlib.h>

#include "Modules.h"   // 后包含外部变量声明
// 条件编译：若GPS.h中未定义GPS缓冲区最大长度，则定义为128
#ifndef GPS_BUF_MAX_LEN
#define GPS_BUF_MAX_LEN    128    // GPS接收缓冲区的最大字节数，防止内存越界
#endif

// 条件编译：若GPS.h中未定义纬度字符串最大长度，则定义为16
#ifndef LATITUDE_MAX_LEN
#define LATITUDE_MAX_LEN   16     // 存储纬度字符串（度分格式）的最大长度
#endif

// 条件编译：若GPS.h中未定义经度字符串最大长度，则定义为16
#ifndef LONGITUDE_MAX_LEN
#define LONGITUDE_MAX_LEN  16     // 存储经度字符串（度分格式）的最大长度
#endif

// 条件编译：若GPS.h中未定义半球方向字符串长度，则定义为2
#ifndef HEMISPHERE_MAX_LEN
#define HEMISPHERE_MAX_LEN 2      // 存储半球方向（N/S/E/W）的字符串长度（含结束符）
#endif

// 新增宏：30秒无有效GPS数据时，OLED显示"No Signal"
#define GPS_NO_DATA_TIMEOUT 30000 // 超时阈值：30000毫秒（30秒）
// 新增宏：35秒无有效GPS数据时，复位GPS状态（避免与30秒显示逻辑冲突）
#define GPS_STATUS_RESET_TIME 35000 // 复位阈值：35000毫秒（35秒）

// 全局变量：GPS核心数据结构体（需在GPS.h中用extern声明，供其他模块访问）
GPS_Data gps_data = {0}; // 全局GPS数据初始化
float gps_lat_decimal = 0.0f;
float gps_lon_decimal = 0.0f;

// 全局变量：存储第一次捕获的有效GPS纬度（十进制度）
float gps_first_lat_decimal = 0.0f;   
// 全局变量：存储第一次捕获的有效GPS经度（十进制度）
float gps_first_lon_decimal = 0.0f;   
// 全局变量：存储第一次捕获的纬度半球方向（N/S）
char gps_first_ns_hem[HEMISPHERE_MAX_LEN] = {0}; 
// 全局变量：存储第一次捕获的经度半球方向（E/W）
char gps_first_ew_hem[HEMISPHERE_MAX_LEN] = {0}; 
// 全局变量：首次有效数据捕获标记（0=未捕获，1=已捕获）
uint8_t is_first_data_captured = 0;   
// 全局变量：OLED屏幕最后更新时间戳（用于5秒定时更新屏幕）
uint32_t last_screen_update_time = 0; 
// 全局变量：最后一次获取有效GPS数据的时间戳（用于30秒超时判断）
uint32_t gps_last_new_data_time = 0;  

// 外部变量：来自主程序，标识系统当前工作模式（1=自动，2=手动，3=设置）
extern uint8_t mode;               
// 外部变量：来自主程序，标识自动模式下的当前显示页面（1=传感器页，2=GPS页）
extern uint8_t auto_page;          
// 宏定义：自动模式的标识（与主程序保持一致）
#define AUTO_MODE 1               

// 静态变量：最后一次获取有效GPS数据的时间戳（用于35秒复位判断）
static uint32_t gps_last_valid_time = 0;
// 静态变量：GPS信号丢失计数器（统计解析失败次数）
static uint8_t gps_signal_lost_counter = 0;

// 静态函数声明：度分格式（NMEA）转十进制度的工具函数
static float NMEA2Decimal(const char* nmea_str, bool is_longitude);
// 静态函数声明：解析NMEA数据（GGA/RMC/GLL）的核心函数
static uint8_t ParseNMEA_Data(const char* buffer);
// 静态函数声明：校验经纬度和半球信息有效性的工具函数（新增）
static uint8_t GPS_CheckDataValid(float lat, float lon, const char* ns, const char* ew);

/******************************************************************
 * 函数名：GPS_Init
 * 功能：GPS模块初始化（清空缓冲区、重置所有状态标志）
 * 修改点：添加串口3初始化调用，确保GPS数据来源为串口3
 ******************************************************************/
void GPS_Init(void) {
    // 初始化串口3（GPS数据接收串口），波特率匹配GPS模块（通常9600）
    uart3_init(9600); // 修改：调用串口3初始化函数，替代原uart_init
    
    // 清空GPS数据结构体的所有成员（按字节置0）
    memset(&gps_data, 0,  sizeof(GPS_Data));
    // 复位数据就绪标志：表示无新的GPS数据待解析
    gps_data.is_data_ready = false;
    // 复位解析完成标志：表示未完成NMEA数据解析
    gps_data.is_parsed = false;
    // 复位数据有效标志：表示当前GPS数据无效
    gps_data.is_valid = false;
    // 复位十进制度纬度为0
    gps_lat_decimal = 0.0f;
    // 复位十进制度经度为0
    gps_lon_decimal = 0.0f;
    // 复位最后一次有效数据时间戳为0
    gps_last_valid_time = 0;
    // 复位信号丢失计数器为0
    gps_signal_lost_counter = 0;
    
    // 初始化首次数据相关变量
    gps_first_lat_decimal = 0.0f;    // 复位首次纬度为0
    gps_first_lon_decimal = 0.0f;    // 复位首次经度为0
    memset(gps_first_ns_hem, 0, HEMISPHERE_MAX_LEN); // 清空首次纬度半球
    memset(gps_first_ew_hem, 0, HEMISPHERE_MAX_LEN); // 清空首次经度半球
    is_first_data_captured = 0;      // 复位首次捕获标记
    last_screen_update_time = 0;     // 复位屏幕更新时间戳
    gps_last_new_data_time = 0;      // 复位最后一次有效数据时间戳（新增）
    
    // 清空串口3的接收缓冲区（确保初始状态干净）
		
    CLR_Buf();
}

/******************************************************************
 * 函数名：GPS_CheckDataValid
 * 功能：校验经纬度数值和半球信息的有效性（新增）
 ******************************************************************/
static uint8_t GPS_CheckDataValid(float lat, float lon, const char* ns, const char* ew) {
    // 1. 校验经纬度数值范围：纬度[-90,90]，经度[-180,180]
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
        return 0; // 数值范围非法，返回无效
    }
    // 2. 校验经纬度非零：排除GPS模块未定位时的0值
    if (lat == 0.0f && lon == 0.0f) {
        return 0; // 数值为0，返回无效
    }
    // 3. 校验半球信息：纬度必须是N/S，经度必须是E/W
    if ((ns[0] != 'N' && ns[0] != 'S') || (ew[0] != 'E' && ew[0] != 'W')) {
        return 0; // 半球字符非法，返回无效
    }
    return 1; // 所有校验通过，返回有效
}

/******************************************************************
 * 函数名：ParseNMEA_Data
 * 功能：解析NMEA数据（GGA/RMC/GLL），提取经纬度和半球信息
 ******************************************************************/
static uint8_t ParseNMEA_Data(const char* buffer) {
    // 定义临时缓冲区，用于存储待解析的NMEA数据（防止修改原缓冲区）
    char temp_buf[GPS_BUF_MAX_LEN];
    // 将原缓冲区数据复制到临时缓冲区（保留最后1位给结束符）
    strncpy(temp_buf, buffer, GPS_BUF_MAX_LEN - 1); 
    temp_buf[GPS_BUF_MAX_LEN - 1] = '\0';          // 强制添加字符串结束符，避免越界
    
    char* token;                  // 字符串分割后的子串指针
    char* fields[15] = {0};       // 存储NMEA分割后的字段（最多15个）
    uint8_t idx = 0;              // 字段索引计数器
    
    // 以逗号为分隔符，分割第一个字段（如$GPGGA）
    token = strtok(temp_buf, ",");
    // 循环分割剩余字段，直到无数据或字段数达到15
    while (token != NULL && idx < 15) {
        fields[idx++] = token;    // 存储当前字段到数组
        token = strtok(NULL, ",");// 继续分割下一个字段
    }
    
    if (idx < 7) return 0; // 字段数不足7个，解析失败（有效NMEA语句至少7个字段）
    
    // 清空原有经纬度和半球信息（解析失败时不保留旧数据）
    memset(gps_data.latitude, 0, LATITUDE_MAX_LEN);
    memset(gps_data.longitude, 0, LONGITUDE_MAX_LEN);
    memset(gps_data.ns_hemisphere, 0, HEMISPHERE_MAX_LEN);
    memset(gps_data.ew_hemisphere, 0, HEMISPHERE_MAX_LEN);
    gps_data.is_valid = false;    // 先标记数据无效
    
    // 解析GGA语句（优先解析，GGA包含定位质量信息）
    if (strstr(buffer, "GGA")) {
        // fields[6]是定位质量标志：1表示定位有效（0=无效，1=有效）
        if (fields[6] && fields[6][0] == '1') { 
            gps_data.is_valid = true; // 标记数据有效
            // 提取纬度（度分格式）：fields[2]是GGA语句的纬度字段
            if (fields[2]) strncpy(gps_data.latitude, fields[2], LATITUDE_MAX_LEN - 1);
            // 提取纬度半球（N/S）：fields[3]是GGA语句的纬度半球字段
            if (fields[3]) strncpy(gps_data.ns_hemisphere, fields[3], HEMISPHERE_MAX_LEN - 1);
            // 提取经度（度分格式）：fields[4]是GGA语句的经度字段
            if (fields[4]) strncpy(gps_data.longitude, fields[4], LONGITUDE_MAX_LEN - 1);
            // 提取经度半球（E/W）：fields[5]是GGA语句的经度半球字段
            if (fields[5]) strncpy(gps_data.ew_hemisphere, fields[5], HEMISPHERE_MAX_LEN - 1);
        }
    } 
    // 解析RMC语句（备用，RMC包含推荐最小定位信息）
    else if (strstr(buffer, "RMC")) {
        // fields[2]是RMC语句的状态位：A表示有效（V=无效）
        if (fields[2] && fields[2][0] == 'A') { 
            gps_data.is_valid = true; // 标记数据有效
            // 提取纬度：fields[3]是RMC语句的纬度字段
            if (fields[3]) strncpy(gps_data.latitude, fields[3], LATITUDE_MAX_LEN - 1);
            // 提取纬度半球：fields[4]是RMC语句的纬度半球字段
            if (fields[4]) strncpy(gps_data.ns_hemisphere, fields[4], HEMISPHERE_MAX_LEN - 1);
            // 提取经度：fields[5]是RMC语句的经度字段
            if (fields[5]) strncpy(gps_data.longitude, fields[5], LONGITUDE_MAX_LEN - 1);
            // 提取经度半球：fields[6]是RMC语句的经度半球字段
            if (fields[6]) strncpy(gps_data.ew_hemisphere, fields[6], HEMISPHERE_MAX_LEN - 1);
        }
    }
    // 解析GLL语句（备用，GLL是地理定位信息语句）
    else if (strstr(buffer, "GLL")) {
        // fields[6]是GLL语句的状态位：A表示有效（V=无效）
        if (fields[6] && fields[6][0] == 'A') { 
            gps_data.is_valid = true; // 标记数据有效
            // 提取纬度：fields[1]是GLL语句的纬度字段
            if (fields[1]) strncpy(gps_data.latitude, fields[1], LATITUDE_MAX_LEN - 1);
            // 提取纬度半球：fields[2]是GLL语句的纬度半球字段
            if (fields[2]) strncpy(gps_data.ns_hemisphere, fields[2], HEMISPHERE_MAX_LEN - 1);
            // 提取经度：fields[3]是GLL语句的经度字段
            if (fields[3]) strncpy(gps_data.longitude, fields[3], LONGITUDE_MAX_LEN - 1);
            // 提取经度半球：fields[4]是GLL语句的经度半球字段
            if (fields[4]) strncpy(gps_data.ew_hemisphere, fields[4], HEMISPHERE_MAX_LEN - 1);
        }
    }
    
    return gps_data.is_valid; // 返回解析结果（有效=1，无效=0）
}

/******************************************************************
 * 函数名：GPS_ParseNMEA
 * 功能：解析NMEA协议核心语句，提取关键信息并转换为十进制度
 * 修改点：解析失败时不更新全局有效数据和时间戳，保护正确数据
 ******************************************************************/
void GPS_ParseNMEA(void) {
    // 若无新的GPS数据就绪，直接返回
    if (!gps_data.is_data_ready) {
        return;
    }
    
    // 保存原始数据到临时缓冲区，并清空就绪标志
    char temp_buf[GPS_BUF_MAX_LEN];
    strncpy(temp_buf, gps_data.buf, GPS_BUF_MAX_LEN - 1); // 复制数据（留结束符位）
    temp_buf[GPS_BUF_MAX_LEN - 1] = '\0';                 // 强制添加结束符
    gps_data.is_data_ready = false;                       // 标记数据已开始解析
    
    // 检查缓冲区是否包含有效NMEA语句（GGA/RMC/GLL）
    if (!strstr(temp_buf, "GGA") && !strstr(temp_buf, "RMC") && !strstr(temp_buf, "GLL")) {
        gps_signal_lost_counter++;  // 无有效语句，信号丢失计数+1
        gps_data.is_valid = false;  // 标记数据无效
        gps_data.is_parsed = true;  // 标记解析完成
        printf("GPS解析失败：无有效NMEA语句\r\n"); // 调试打印（仍用串口1输出）
        return;
    }
    
    // 调用解析函数，解析NMEA数据
    if (ParseNMEA_Data(temp_buf)) {
        // 将度分格式的经纬度转换为十进制度
        float current_lat = NMEA2Decimal(gps_data.latitude, false); // 纬度转换（false=不是经度）
        float current_lon = NMEA2Decimal(gps_data.longitude, true);  // 经度转换（true=是经度）
        
        // 校验经纬度数值范围是否合法
        if ((current_lat < -90.0f || current_lat > 90.0f) || 
            (current_lon < -180.0f || current_lon > 180.0f)) {
            gps_signal_lost_counter++;  // 范围非法，计数+1
            gps_data.is_valid = false;  // 标记无效
            printf("GPS解析失败：经纬度数值范围非法\r\n");
        } else {
            // 调用数据校验函数，检查经纬度和半球的有效性
            if (GPS_CheckDataValid(current_lat, current_lon, gps_data.ns_hemisphere, gps_data.ew_hemisphere)) {
                // 仅当数据有效时，更新全局经纬度
                gps_lat_decimal = current_lat;
                gps_lon_decimal = current_lon;
                // 更新最后一次有效数据时间戳（滴答定时器）
                gps_last_valid_time = delay_get_tick();
                gps_data.is_valid = true; // 标记数据有效
                gps_signal_lost_counter = 0; // 复位信号丢失计数器
                gps_last_new_data_time = delay_get_tick(); // 记录最新有效数据时间（新增）
                
                // 若未捕获过首次有效数据，则保存首次数据
                if (!is_first_data_captured) {
                    gps_first_lat_decimal = current_lat;  // 保存首次纬度
                    gps_first_lon_decimal = current_lon;  // 保存首次经度
                    // 保存首次纬度半球
                    strncpy(gps_first_ns_hem, gps_data.ns_hemisphere, HEMISPHERE_MAX_LEN - 1);
                    // 保存首次经度半球
                    strncpy(gps_first_ew_hem, gps_data.ew_hemisphere, HEMISPHERE_MAX_LEN - 1);
                    is_first_data_captured = 1; // 标记首次捕获完成
                    // 调试打印首次数据（串口1输出）
                    printf("首次捕获GPS有效数据：纬度%.5f%c, 经度%.5f%c\r\n", 
                           gps_first_lat_decimal, gps_first_ns_hem[0],
                           gps_first_lon_decimal, gps_first_ew_hem[0]);
                    last_screen_update_time = delay_get_tick(); // 首次捕获立即更新屏幕
                }
                // 调试打印最新有效数据（串口1输出）
                printf("GPS解析成功: 纬度%.5f%c, 经度%.5f%c\r\n", 
                       current_lat, gps_data.ns_hemisphere[0],
                       current_lon, gps_data.ew_hemisphere[0]);
            } else {
                gps_signal_lost_counter++;  // 数据校验失败，计数+1
                gps_data.is_valid = false;  // 标记无效
                printf("GPS解析失败：数据有效性校验不通过\r\n");
            }
        }
    } else {
        gps_signal_lost_counter++;  // NMEA解析失败，计数+1
        gps_data.is_valid = false;  // 标记无效
        printf("GPS解析失败：NMEA语句解析失败\r\n");
    }
    
    gps_data.is_parsed = true; // 标记解析完成
}

/******************************************************************
 * 函数名：GPS_DisplayAndSend
 * 功能：GPS显示逻辑重构：解析失败时显示存入的正确数据，不显示错误数据
 ******************************************************************/
void GPS_DisplayAndSend(void) {
    // 仅在自动模式的GPS页面（第2页）执行显示逻辑
    if (mode != AUTO_MODE || auto_page != 2) {
        return;
    }

    char lat_str[12] = {0};    // 存储纬度显示字符串
    char lon_str[12] = {0};    // 存储经度显示字符串
    uint32_t current_time = delay_get_tick(); // 获取当前时间戳

    // 逻辑1：未捕获到首次有效数据，OLED显示"No Signal"
    if (!is_first_data_captured) {
        OLED_ShowString(45, 16, (uint8_t*)"No Signal ", 16, 1); // 经度位置显示无信号
        OLED_ShowString(45, 32, (uint8_t*)"No Signal ", 16, 1); // 纬度位置显示无信号
    }
    // 逻辑2：已捕获首次有效数据（有正确数据可显示）
    else {
        // 子逻辑A：30秒无新有效数据，显示"No Signal"
        if (current_time - gps_last_new_data_time >= GPS_NO_DATA_TIMEOUT) {
            OLED_ShowString(45, 16, (uint8_t*)"No Signal ", 16, 1);
            OLED_ShowString(45, 32, (uint8_t*)"No Signal ", 16, 1);
            printf("GPS30秒无新数据，显示No Signal\r\n");
					 // ========== 核心新增：清空所有保存的经纬度数据 ==========
            // 1. 清空当前经纬度
            gps_lat_decimal = 0.0f;
            gps_lon_decimal = 0.0f;
            // 2. 清空首次捕获的经纬度
            gps_first_lat_decimal = 0.0f;
            gps_first_lon_decimal = 0.0f;
            memset(gps_first_ns_hem, 0, HEMISPHERE_MAX_LEN); // 清空首次纬度半球
            memset(gps_first_ew_hem, 0, HEMISPHERE_MAX_LEN); // 清空首次经度半球
            // 3. 复位首次捕获标记
            is_first_data_captured = 0;
            // 4. 复位GPS有效性标志
            gps_data.is_valid = false;
            // 5. 复位时间戳（避免重复触发清空）
            gps_last_new_data_time = 0;
            gps_last_valid_time = 0;
        
        }
        // 子逻辑B：30秒内有过有效数据（正常显示）
        else {
            // 子逻辑B1：当前解析成功且数据有效，每5秒更新一次屏幕
            if (gps_data.is_valid) {
                // 检查是否达到5秒更新间隔
                if (current_time - last_screen_update_time >= 5000) {
                    // 格式化纬度字符串（保留5位小数+半球）
                    snprintf(lat_str, sizeof(lat_str), "%.5f%c", gps_lat_decimal, gps_data.ns_hemisphere[0]);
                    // 格式化经度字符串（保留5位小数+半球）
                    snprintf(lon_str, sizeof(lon_str), "%.5f%c", gps_lon_decimal, gps_data.ew_hemisphere[0]);
                    OLED_ShowString(45, 16, (uint8_t*)lon_str, 16, 1); // 显示经度
                    OLED_ShowString(45, 32, (uint8_t*)lat_str, 16, 1); // 显示纬度
                    last_screen_update_time = current_time; // 更新屏幕时间戳
                    printf("GPS屏幕更新：最新经纬度\r\n");
                }
            }
            // 子逻辑B2：当前解析失败，显示首次保存的正确数据
            else {
                // 格式化首次纬度数据
                snprintf(lat_str, sizeof(lat_str), "%.5f%c", gps_first_lat_decimal, gps_first_ns_hem[0]);
                // 格式化首次经度数据
                snprintf(lon_str, sizeof(lon_str), "%.5f%c", gps_first_lon_decimal, gps_first_ew_hem[0]);
                OLED_ShowString(45, 16, (uint8_t*)lon_str, 16, 1); // 显示首次经度
                OLED_ShowString(45, 32, (uint8_t*)lat_str, 16, 1); // 显示首次纬度
                printf("GPS解析失败，显示首次保存的正确数据\r\n");
            }
        }
    }

    // 蓝牙发送逻辑：仅当数据有效时，每3秒发送一次经纬度
    static uint32_t last_send_time = 0;
    if (gps_data.is_valid && (current_time - last_send_time > 3000)) {
       // GPS_SendLatLonViaUSART2(); // 调用串口2发送函数
        last_send_time = current_time; // 更新发送时间戳
    }
}

/******************************************************************
 * 函数名：NMEA2Decimal
 * 功能：NMEA度分格式转换为十进制度
 ******************************************************************/
static float NMEA2Decimal(const char* nmea_str, bool is_longitude) {
    // 校验输入字符串是否为空或长度不足
    if (nmea_str == NULL || strlen(nmea_str) < 5) {
        return 0.0f; // 输入无效，返回0
    }

    // 查找小数点的位置（度分格式：如3012.3456表示30度12.3456分）
    char* dot_ptr = strchr(nmea_str, '.');
    if (dot_ptr == NULL) {
        return 0.0f; // 无小数点，返回0
    }

    // 确定度数的位数：经度3位（如120），纬度2位（如30）
    uint8_t deg_digits = is_longitude ? 3 : 2;
    // 校验小数点位置是否合法（度数位数+至少1位分）
    if ((dot_ptr - nmea_str) < deg_digits + 1) {
        return 0.0f; // 格式非法，返回0
    }

    // 提取度数部分并转换为浮点数
    char deg_str[4] = {0};
    strncpy(deg_str, nmea_str, deg_digits); // 复制度数字符
    float deg = atof(deg_str);              // 转换为数值

    // 提取分数部分并转换为浮点数
    char min_str[8] = {0};
    // 复制分数字符（从度数后到小数点后4位）
    strncpy(min_str, nmea_str + deg_digits, dot_ptr - (nmea_str + deg_digits) + 4);
    float min = atof(min_str);              // 转换为数值

    // 十进制度计算公式：度 + 分/60
    return deg + (min / 60.0f);
}

/******************************************************************
 * 函数名：GPS_SendLatLonViaUSART2
 * 功能：通过串口2（蓝牙）发送经纬度数据
 ******************************************************************/
//void GPS_SendLatLonViaUSART2(void) {
//    char bluetooth_buf[64] = {0}; // 蓝牙发送缓冲区
//    // 获取纬度半球（为空则设为空格）
//    char lat_hem = (gps_data.ns_hemisphere[0] != '\0') ? gps_data.ns_hemisphere[0] : ' ';
//    // 获取经度半球（为空则设为空格）
//    char lon_hem = (gps_data.ew_hemisphere[0] != '\0') ? gps_data.ew_hemisphere[0] : ' ';
//    
//    // 格式化经纬度发送字符串
//    snprintf(bluetooth_buf, sizeof(bluetooth_buf), 
//             "纬度:%.5f%c,经度:%.5f%c\r\n",
//             gps_lat_decimal, lat_hem,
//             gps_lon_decimal, lon_hem);
//    USART2_SendString((const char*)bluetooth_buf); // 调用串口2发送函数
//}

/******************************************************************
 * 函数名：GPS_ClearBuffer
 * 功能：彻底清空GPS缓冲区和状态标志
 * 修改点：同步清空串口3的接收缓冲区
 ******************************************************************/
void GPS_ClearBuffer(void) {
    memset(gps_data.buf, 0, sizeof(gps_data.buf));          // 清空GPS接收缓冲区
    memset(gps_data.utc_time, 0, sizeof(gps_data.utc_time));// 清空UTC时间
    memset(gps_data.latitude, 0, sizeof(gps_data.latitude));// 清空纬度字符串
    memset(gps_data.longitude, 0, sizeof(gps_data.longitude));// 清空经度字符串
    memset(gps_data.ns_hemisphere, 0, sizeof(gps_data.ns_hemisphere));// 清空纬度半球
    memset(gps_data.ew_hemisphere, 0, sizeof(gps_data.ew_hemisphere));// 清空经度半球
    gps_data.is_data_ready = false; // 复位数据就绪标志
    gps_data.is_parsed = false;     // 复位解析完成标志
    gps_data.is_valid = false;      // 复位数据有效标志
    gps_signal_lost_counter = 0;    // 复位信号丢失计数器
    
    // 修改：同步清空串口3的接收缓冲区（确保数据链路干净）
    CLR_Buf();
}

/******************************************************************
 * 函数名：GPS_Status_Check
 * 功能：GPS状态检查，长时间无信号时复位状态（调整为35秒）
 ******************************************************************/
void GPS_Status_Check(void) {
    static uint32_t last_check_time = 0; // 上次状态检查时间戳
    uint32_t current_time = delay_get_tick(); // 当前时间戳
    
    // 每秒检查一次状态（避免频繁执行）
    if (current_time - last_check_time < 1000) return;
    last_check_time = current_time; // 更新检查时间戳
    
    // 超过35秒无有效数据，复位GPS模块
    if (gps_last_valid_time > 0 && (current_time - gps_last_valid_time > GPS_STATUS_RESET_TIME)) {
        printf("GPS35秒无信号，复位状态\r\n");
        GPS_Init(); // 调用初始化函数复位（包含串口3重新初始化）
        gps_last_valid_time = 0; // 复位有效时间戳
        gps_signal_lost_counter = 0; // 复位计数器
    }
}

/******************************************************************
 * 函数名：GPS_Debug
 * 功能：GPS调试信息打印（增加解析失败和有效数据来源信息）
 ******************************************************************/
void GPS_Debug(void) {
    static uint32_t last_debug_time = 0; // 上次调试打印时间戳
    uint32_t current_time = delay_get_tick(); // 当前时间戳
    int32_t timeout_remaining = -1; // 30秒超时剩余时间（有符号数避免溢出）
    
    // 每秒打印一次调试信息
    if (current_time - last_debug_time > 1000) {
        last_debug_time = current_time; // 更新调试时间戳
        
        // 计算30秒超时剩余时间
        if (gps_last_new_data_time != 0) {
            timeout_remaining = (int32_t)GPS_NO_DATA_TIMEOUT - (int32_t)(current_time - gps_last_new_data_time);
        } else {
            timeout_remaining = -1; // 无有效数据，剩余时间为-1
        }
        
        // 打印调试信息头
        printf("===== GPS调试信息（串口3接收）=====\r\n"); // 修改：标注串口3
        // 打印GPS状态标志
        printf("数据就绪：%d, 已解析：%d, 有效：%d\r\n",
               gps_data.is_data_ready, gps_data.is_parsed, gps_data.is_valid);
        // 打印首次捕获状态和当前纬度信息
        printf("首次捕获：%d | 纬度原始：%s %s, 转换后：%.5f%c\r\n",
               is_first_data_captured, gps_data.latitude, gps_data.ns_hemisphere, 
               gps_lat_decimal, gps_data.ns_hemisphere[0]);
        // 打印当前经度信息
        printf("经度原始：%s %s, 转换后：%.5f%c\r\n",
               gps_data.longitude, gps_data.ew_hemisphere, 
               gps_lon_decimal, gps_data.ew_hemisphere[0]);
        // 打印首次捕获的经纬度
        printf("首次纬度：%.5f%c, 首次经度：%.5f%c\r\n",
               gps_first_lat_decimal, gps_first_ns_hem[0],
               gps_first_lon_decimal, gps_first_ew_hem[0]);
        // 打印最后有效数据时间和超时剩余时间
        printf("最后有效数据时间：%lu ms | 30秒超时剩余：%ld ms\r\n",
               gps_last_new_data_time, timeout_remaining);
        // 打印信号丢失计数和最后有效时间
        printf("无信号计数：%d, 最后有效时间：%lu ms\r\n",
               gps_signal_lost_counter, gps_last_valid_time);
        // 打印调试信息尾
        printf("========================\r\n");
    }
}

/******************************************************************
 * 函数名：GPS_HandleData
 * 功能：统一处理GPS数据的函数（新增，适配串口3）
 ******************************************************************/
void GPS_HandleData(void) {
    GPS_ParseNMEA();      // 解析串口3接收的NMEA数据
    GPS_DisplayAndSend(); // 处理显示和蓝牙发送
    GPS_Status_Check();   // 检查GPS状态
}

