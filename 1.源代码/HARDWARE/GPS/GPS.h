#ifndef __GPS_H
#define __GPS_H

#include "sys.h"
#include "oled.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>  // 标准整数类型
#include <stdbool.h> // 标准bool类型（替代自定义true/false）
#include <ctype.h>   // 字符判断函数（isdigit）

// 宏定义：基于NMEA协议规范，留足冗余空间
#define GPS_BUF_MAX_LEN    128    // GPS接收缓冲区（NMEA最长约82字节）
#define UTC_TIME_MAX_LEN   10     // UTC时间缓冲区（格式：hhmmss.sss）
#define LATITUDE_MAX_LEN   10     // 纬度缓冲区（格式：ddmm.mmmm）
#define LONGITUDE_MAX_LEN  11     // 经度缓冲区（格式：dddmm.mmmm）
#define HEMISPHERE_MAX_LEN 2      // 半球缓冲区（N/S/E/W + '\0'）
#define MIN(a, b)          ((a) < (b) ? (a) : (b)) // 安全取最小值宏

// GPS数据结构体（规范化命名，增强可读性）
typedef struct {
    char buf[GPS_BUF_MAX_LEN];
    char utc_time[UTC_TIME_MAX_LEN];
    char latitude[LATITUDE_MAX_LEN];
    char longitude[LONGITUDE_MAX_LEN];
    char ns_hemisphere[HEMISPHERE_MAX_LEN];
    char ew_hemisphere[HEMISPHERE_MAX_LEN];
    bool is_data_ready;              // 原始数据是否接收完成（来自串口3）
    bool is_parsed;                  // 数据是否解析完成
    bool is_valid;                   // 定位是否有效
    uint8_t satellite_count;         // 卫星数量（可选，用于质量判断）
} GPS_Data;

// 全局GPS数据对象（供外部模块访问）
extern GPS_Data gps_data; 
extern float gps_lat_decimal;  // 全局纬度（十进制度，保留5位小数）
extern float gps_lon_decimal;  // 全局经度（十进制度，保留5位小数）

// 函数声明（按功能分类，命名规范化）
void GPS_Init(void);                  // GPS模块初始化（包含串口3初始化）
void GPS_ParseNMEA(void);             // 解析串口3接收的NMEA协议（GGA语句）
void GPS_DisplayAndSend(void);        // OLED显示+串口/蓝牙发送
void GPS_ClearBuffer(void);           // 清空GPS缓冲区和串口3缓冲区
static void GPS_ErrorLog(const char* err_info); // 错误日志（静态：内部使用）
static float NMEA2Decimal(const char* nmea_str, bool is_longitude); // 度分→十进制（内部使用）
void GPS_SendLatLonViaUSART2(void);   // 经纬度蓝牙发送（USART2）
void GPS_HandleData(void); // 新增：统一处理GPS数据的函数
void GPS_Status_Check(void);          // GPS状态检查（35秒超时复位）
void GPS_Debug(void);                 // GPS调试信息打印

#endif

