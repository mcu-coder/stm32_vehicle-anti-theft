#ifndef	__MODULES_H_
#define __MODULES_H_

#include "stm32f10x.h"                
#include "adcx.h"
// 关键修正：直接包含GPS.h，获取GPS_Data的完整定义，无需前向声明
#include "GPS.h"

// 现在可以直接声明extern变量（编译器已知道GPS_Data的完整结构）
extern GPS_Data gps_data;
extern float gps_lat_decimal;
extern float gps_lon_decimal;

//uint8_t count_m = 1; 

typedef struct
{
    int32_t hrAvg;   
    int32_t spo2Avg;    
    uint8_t humi;
    float temp;
    uint16_t lux;	
    uint16_t soilHumi;
    uint16_t Smoge;	
    uint16_t AQI;
    uint16_t CO;
    uint16_t hPa;
	uint8_t fire;
	uint8_t hw;
	uint8_t vib;
    uint8_t distance;
	uint8_t temp_valid;   // 温度有效标志：0=无效（初始85℃），1=有效（真实温度）

} SensorModules;

typedef struct
{
	float tempValue;
	uint8_t humiValue;
	//uint8_t tempValue;
	uint16_t luxValue;	
	uint16_t soilHumiValue;
	uint16_t COValue;	
	uint16_t AQIValue;
	uint16_t hPaValue;
	uint16_t SmogeValue;
	uint16_t hrMin ;     // 心率下限
  uint16_t hrMax;   	 // 心率上限
	uint8_t hrAvgValue;
	uint8_t distanceValue;
//    uint16_t spo2Min;    // 血氧下限
//    uint16_t spo2Max;    // 血氧上限
	
}SensorThresholdValue;

typedef struct
{
  //uint8_t	NOW_LED_Flag;
	uint8_t LED_Flag;
	uint8_t BEEP_Flag;
	uint8_t NOW_Curtain_Flag;
	uint8_t Curtain_Flag;	
	uint8_t NOW_Window_Flag;
	uint8_t Window_Flag;	
	uint8_t Fan_Flag;
	uint8_t Humidifier_Flag;
	uint8_t Bump_Flag;
  uint8_t Safe_Flag;
	
}DriveModules;

// 初始化驱动数据（全局变量）
extern SensorModules sensorData;			//声明传感器模块的结构体变量
extern SensorThresholdValue Sensorthreshold;	//声明传感器阈值结构体变量
extern DriveModules driveData;				//声明驱动器状态的结构体变量
void SensorScan(void);

#endif

