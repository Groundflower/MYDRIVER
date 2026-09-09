#ifndef __DHT22_H
#define __DHT22_H

#include "main.h"
#include "gpio.h"

// DHT22设备结构体
typedef struct {
    GPIO_TypeDef* port;    // GPIO端口
    uint16_t pin;          // GPIO引脚
} DHT22_Device;

// 传感器数据结构体
typedef struct {
    float temperature;     // 温度值
    float humidity;        // 湿度值
    uint8_t checksum_error;// 校验错误标志
} DHT22_Data;

// 函数声明
void DHT22_Init(DHT22_Device* device);
uint8_t DHT22_Read(DHT22_Device* device, DHT22_Data* data);
void DHT22_Delay_us(uint16_t us);

#endif
