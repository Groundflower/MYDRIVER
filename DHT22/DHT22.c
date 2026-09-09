#include "dht22.h"

// 微秒级延时函数（使用 DWT 周期计数器，首次调用自动使能）
void DHT22_Delay_us(uint16_t us) {
    static uint8_t dwt_ready = 0;
    if (!dwt_ready) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // 使能 DWT 访问
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 使能周期计数器
        dwt_ready = 1;
    }
    uint32_t ticks = (uint32_t)us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

// 初始化函数：把数据脚配置为开漏输出并置高（总线空闲态）
void DHT22_Init(DHT22_Device* device) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = device->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(device->port, device->pin, GPIO_PIN_SET);
}

// DHT22读取函数
uint8_t DHT22_Read(DHT22_Device* device, DHT22_Data* data) {
    uint8_t buffer[5] = {0};  // DHT22 返回 40 位 = 5 字节
    uint8_t checksum = 0;
    
    // 1. 主机发送开始信号
    HAL_GPIO_WritePin(device->port, device->pin, GPIO_PIN_RESET);
    DHT22_Delay_us(1000);  // 拉低至少1ms
    HAL_GPIO_WritePin(device->port, device->pin, GPIO_PIN_SET);
    DHT22_Delay_us(30);    // 主机拉高20-40μs
    
    // 2. 切换为输入模式等待传感器响应
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = device->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(device->port, &GPIO_InitStruct);
    
    // 3. 等待传感器响应（超时保护）
    uint32_t timeout = 1000;
    while (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_SET) {
        if (timeout-- == 0) return 0;
        DHT22_Delay_us(1);
    }
    
    // 4. 确认传感器响应时序
    while (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_SET);
    
    // 5. 读取40位数据
    for (uint8_t i = 0; i < 40; i++) {
        // 等待低电平结束
        while (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_RESET);
        
        // 延时40μs判断位值
        DHT22_Delay_us(40);
        if (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_SET) {
            buffer[i/8] |= (1 << (7 - (i % 8)));  // 设置对应位
            while (HAL_GPIO_ReadPin(device->port, device->pin) == GPIO_PIN_SET);
        }
    }
    
    // 6. 重新配置为输出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(device->port, device->pin, GPIO_PIN_SET);
    
    // 7. 校验和数据解析：前 4 字节之和的低 8 位应等于第 5 字节
    checksum = (uint8_t)((buffer[0] + buffer[1] + buffer[2] + buffer[3]) & 0xFF);
    if (checksum != buffer[4]) {
        data->checksum_error = 1;
        return 0;
    }

    // 8. 数据转换：湿度 = buffer[0..1]/10，温度 = buffer[2..3]/10（buffer[2] 的 bit7 为符号位）
    data->humidity = (float)(((uint16_t)buffer[0] << 8) | buffer[1]) / 10.0f;
    data->temperature = (float)((((uint16_t)buffer[2] & 0x7F) << 8) | buffer[3]) / 10.0f;
    if (buffer[2] & 0x80) {
        data->temperature = -data->temperature;
    }

    data->checksum_error = 0;
    return 1;
}
