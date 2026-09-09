#include "mpu6050.h"
#include "math.h"

/* 定义M_PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==================== 静态变量 ==================== */
static float accel_sensitivity = 16384.0f;
static float gyro_sensitivity = 131.0f;
static float   gyro_offset[3]     = {0.0f, 0.0f, 0.0f};  /* 三轴偏置估计(°/s) */
static uint8_t  bias_still_cnt[3]  = {0, 0, 0};           /* 各轴连续"静止可学习"帧计数 */
static float    accel_norm_lp      = 1.0f;                /* 加速度模长慢速平均 */
static uint8_t  accel_norm_lp_init = 0;                   /* 慢速平均是否已建立 */

static LPFilter_t accel_filter[3];
static LPFilter_t gyro_filter[3];

/* ==================== DWT延时函数 ==================== */

/**
 * @brief 初始化DWT用于微秒延时
 */
void MPU6050_DWT_Init(void)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 清零计数器 */
    DWT->CYCCNT = 0;
    /* 使能计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 微秒延时函数（使用DWT）
 * @param us: 延时微秒数
 */
void MPU6050_Delay_Us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    
    while((DWT->CYCCNT - start) < ticks);
}

/* ==================== 软件I2C底层实现 ==================== */

#define SCL_H()   HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(MPU6050_GPIO_PORT, MPU6050_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(MPU6050_GPIO_PORT, MPU6050_SDA_PIN)
#define I2C_DELAY() MPU6050_Delay_Us(10)

static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
}

static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
}

static void MPU6050_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_GPIO_PORT, &GPIO_InitStruct);
    
    SCL_H();
    SDA_H();
}

static void MPU6050_I2C_Start(void)
{
    SDA_OUT();
    SDA_H();
    SCL_H();
    I2C_DELAY();
    SDA_L();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
}

static void MPU6050_I2C_Stop(void)
{
    SDA_OUT();
    SDA_L();
    SCL_H();
    I2C_DELAY();
    SDA_H();
    I2C_DELAY();
}

static void MPU6050_I2C_SendByte(uint8_t byte)
{
    SDA_OUT();
    
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }

        I2C_DELAY();
        SCL_H();
        I2C_DELAY();
        SCL_L();
        I2C_DELAY();
        byte <<= 1;
    }
}

static uint8_t MPU6050_I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    
    SDA_IN();
    
    for (uint8_t i = 0; i < 8; i++)
    {
        SCL_H();
        I2C_DELAY();
        data <<= 1;
        if (SDA_READ()) {
            data |= 0x01;
        }
        SCL_L();
        I2C_DELAY();
    }
    
    SDA_OUT();
    if (ack) {
        SDA_L();
    } else {
        SDA_H();
    }
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
    
    return data;
}

static uint8_t MPU6050_I2C_WaitAck(void)
{
    uint8_t ack;
    
    SDA_IN();
    SCL_H();
    I2C_DELAY();
    
    if (SDA_READ()) {
        ack = 1;
    } else {
        ack = 0;
    }
    
    SCL_L();
    I2C_DELAY();
    SDA_OUT();
    
    return ack;
}

/* ==================== I2C读写函数 ==================== */

static void MPU6050_I2C_Write(uint8_t reg, uint8_t data)
{
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDR);
    MPU6050_I2C_WaitAck();
    MPU6050_I2C_SendByte(reg);
    MPU6050_I2C_WaitAck();
    MPU6050_I2C_SendByte(data);
    MPU6050_I2C_WaitAck();
    MPU6050_I2C_Stop();
}

static uint8_t MPU6050_I2C_Read(uint8_t reg)
{
    uint8_t data;

    MPU6050_I2C_Start();

    MPU6050_I2C_SendByte(MPU6050_ADDR);
    if (MPU6050_I2C_WaitAck())
    {
        MPU6050_I2C_Stop();
        return 0xFF;
    }

    MPU6050_I2C_SendByte(reg);
    if (MPU6050_I2C_WaitAck())
    {
        MPU6050_I2C_Stop();
        return 0xFF;
    }

    MPU6050_I2C_Start();

    MPU6050_I2C_SendByte(MPU6050_ADDR | 0x01);
    if (MPU6050_I2C_WaitAck())
    {
        MPU6050_I2C_Stop();
        return 0xFF;
    }

    data = MPU6050_I2C_ReadByte(0);

    MPU6050_I2C_Stop();

    return data;
}

static void MPU6050_I2C_Read_Buffer(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t i;
    
    /* 发送起始信号和设备地址（写模式） */
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDR);
    MPU6050_I2C_WaitAck();
    
    /* 发送寄存器地址 */
    MPU6050_I2C_SendByte(reg);
    MPU6050_I2C_WaitAck();
    
    /* 重新发送起始信号 */
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDR | 0x01);  // 读模式
    MPU6050_I2C_WaitAck();
    
    /* 读取数据 */
    for (i = 0; i < len; i++) {
        if (i == len - 1) {
            buffer[i] = MPU6050_I2C_ReadByte(0);  // 最后一个字节NACK
        } else {
            buffer[i] = MPU6050_I2C_ReadByte(1);  // ACK
        }
    }
    
    MPU6050_I2C_Stop();
}

/* ==================== MPU6050功能函数 ==================== */

/**
 * @brief 初始化MPU6050
 */
uint8_t MPU6050_Init(MPU6050_t *mpu6050)
{
    uint8_t check = 0;
    uint8_t i;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    MPU6050_t temp_data;
    
    /* 初始化DWT延时 */
    MPU6050_DWT_Init();
    
    /* 初始化软件I2C */
    MPU6050_I2C_Init();
    
    /* 检查设备是否连接 */
    check = MPU6050_I2C_Read(MPU6050_REG_WHO_AM_I);
    if (check != 0x68) {
        return 1;
    }
    
    /* 复位设备 */
    MPU6050_I2C_Write(MPU6050_REG_PWR_MGMT_1, 0x80);
    MPU6050_Delay_Us(100000);  /* 延时100ms等待复位完成 */
    
    /* 唤醒设备 */
    MPU6050_I2C_Write(MPU6050_REG_PWR_MGMT_1, 0x0);
    MPU6050_Delay_Us(10000);   /* 延时10ms等待稳定 */
    
    /* 设置采样率: 1kHz */
    MPU6050_Set_Sample_Rate(0x07);
    
    /* 设置数字低通滤波器 */
    MPU6050_Set_DLPF(DLPF_42HZ);
    
    /* 设置量程 */
    MPU6050_Set_Accel_Range(ACCEL_RANGE_2G);
    MPU6050_Set_Gyro_Range(GYRO_RANGE_250DPS);
    
    /* 初始化滤波器 */
    MPU6050_LPFilter_Init(&accel_filter[0], 0.1f);
    MPU6050_LPFilter_Init(&accel_filter[1], 0.1f);
    MPU6050_LPFilter_Init(&accel_filter[2], 0.1f);
    MPU6050_LPFilter_Init(&gyro_filter[0], 0.15f);
    MPU6050_LPFilter_Init(&gyro_filter[1], 0.15f);
    MPU6050_LPFilter_Init(&gyro_filter[2], 0.15f);
    
    /* 清空数据结构体 */
    memset(mpu6050, 0, sizeof(MPU6050_t));
    
    /* 等待设备稳定 */
    MPU6050_Delay_Us(100000);  /* 再延时100ms */
    
    /* 复位自适应零漂状态 */
    MPU6050_AutoBias_Reset();
    
    /* 上电静态粗校准：校准瞬间设备必须保持静止。
     * 三轴各采样若干次求平均，作为偏置初值；运行期再由自适应缓慢修正。 */
    for (i = 0; i < MPU6050_INIT_CALIB_SAMPLES; i++) {
        MPU6050_Read_Gyro(&temp_data);
        gyro_sum[0] += temp_data.Gx;
        gyro_sum[1] += temp_data.Gy;
        gyro_sum[2] += temp_data.Gz;
        MPU6050_Delay_Us(10000);  /* 每次采集间隔10ms */
    }
    
    for (i = 0; i < 3; i++) {
        gyro_offset[i] = gyro_sum[i] / (float)MPU6050_INIT_CALIB_SAMPLES;
    }
    
    /* 回写偏置初值，方便显示 */
    mpu6050->Gyro_X_Offset = gyro_offset[0];
    mpu6050->Gyro_Y_Offset = gyro_offset[1];
    mpu6050->Gyro_Z_Offset = gyro_offset[2];
    
    return 0;
}

/**
 * @brief 读取所有数据
 */
uint8_t MPU6050_Read_All(MPU6050_t *mpu6050)
{
    uint8_t buffer[14];
    
    MPU6050_I2C_Read_Buffer(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
    mpu6050->Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    mpu6050->Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    mpu6050->Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    mpu6050->Temp_RAW = (int16_t)((buffer[6] << 8) | buffer[7]);
    mpu6050->Gyro_X_RAW = (int16_t)((buffer[8] << 8) | buffer[9]);
    mpu6050->Gyro_Y_RAW = (int16_t)((buffer[10] << 8) | buffer[11]);
    mpu6050->Gyro_Z_RAW = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    mpu6050->Ax = (float)mpu6050->Accel_X_RAW / accel_sensitivity;
    mpu6050->Ay = (float)mpu6050->Accel_Y_RAW / accel_sensitivity;
    mpu6050->Az = (float)mpu6050->Accel_Z_RAW / accel_sensitivity;
    mpu6050->Gx = (float)mpu6050->Gyro_X_RAW / gyro_sensitivity;
    mpu6050->Gy = (float)mpu6050->Gyro_Y_RAW / gyro_sensitivity;
    mpu6050->Gz = (float)mpu6050->Gyro_Z_RAW / gyro_sensitivity;
    mpu6050->Temp = (float)mpu6050->Temp_RAW / 340.0f + 36.53f;
    
    MPU6050_Apply_Filter(mpu6050);
    
    return 0;
}

/**
 * @brief 读取加速度计数据
 */
uint8_t MPU6050_Read_Accel(MPU6050_t *mpu6050)
{
    uint8_t buffer[6];
    
    MPU6050_I2C_Read_Buffer(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
    
    mpu6050->Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    mpu6050->Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    mpu6050->Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    mpu6050->Ax = mpu6050->Accel_X_RAW / accel_sensitivity;
    mpu6050->Ay = mpu6050->Accel_Y_RAW / accel_sensitivity;
    mpu6050->Az = mpu6050->Accel_Z_RAW / accel_sensitivity;
    
    return 0;
}

/**
 * @brief 读取陀螺仪数据
 */
uint8_t MPU6050_Read_Gyro(MPU6050_t *mpu6050)
{
    uint8_t buffer[6];
    
    MPU6050_I2C_Read_Buffer(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
    
    mpu6050->Gyro_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    mpu6050->Gyro_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    mpu6050->Gyro_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    mpu6050->Gx = mpu6050->Gyro_X_RAW / gyro_sensitivity;
    mpu6050->Gy = mpu6050->Gyro_Y_RAW / gyro_sensitivity;
    mpu6050->Gz = mpu6050->Gyro_Z_RAW / gyro_sensitivity;
    
    return 0;
}

/**
 * @brief 读取温度数据
 */
uint8_t MPU6050_Read_Temp(MPU6050_t *mpu6050)
{
    uint8_t buffer[2];
    
    MPU6050_I2C_Read_Buffer(MPU6050_REG_TEMP_OUT_H, buffer, 2);
    
    mpu6050->Temp_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    mpu6050->Temp = mpu6050->Temp_RAW / 340.0f + 36.53f;
    
    return 0;
}

/**
 * @brief 设置加速度计量程
 */
void MPU6050_Set_Accel_Range(MPU6050_AccelRange_t range)
{
    MPU6050_I2C_Write(MPU6050_REG_ACCEL_CONFIG, range);
    
    switch (range) {
        case ACCEL_RANGE_2G:  accel_sensitivity = 16384.0f; break;
        case ACCEL_RANGE_4G:  accel_sensitivity = 8192.0f;  break;
        case ACCEL_RANGE_8G:  accel_sensitivity = 4096.0f;  break;
        case ACCEL_RANGE_16G: accel_sensitivity = 2048.0f;  break;
    }
}

/**
 * @brief 设置陀螺仪量程
 */
void MPU6050_Set_Gyro_Range(MPU6050_GyroRange_t range)
{
    MPU6050_I2C_Write(MPU6050_REG_GYRO_CONFIG, range);
    
    switch (range) {
        case GYRO_RANGE_250DPS:  gyro_sensitivity = 131.0f; break;
        case GYRO_RANGE_500DPS:  gyro_sensitivity = 65.5f;  break;
        case GYRO_RANGE_1000DPS: gyro_sensitivity = 32.8f;  break;
        case GYRO_RANGE_2000DPS: gyro_sensitivity = 16.4f;  break;
    }
}

/**
 * @brief 设置数字低通滤波器
 */
void MPU6050_Set_DLPF(MPU6050_DLPF_t dlpf)
{
    MPU6050_I2C_Write(MPU6050_REG_CONFIG, dlpf);
}

/**
 * @brief 设置采样率
 */
void MPU6050_Set_Sample_Rate(uint8_t rate)
{
    MPU6050_I2C_Write(MPU6050_REG_SMPLRT_DIV, rate);
}

/**
 * @brief 初始化低通滤波器
 */
void MPU6050_LPFilter_Init(LPFilter_t *filter, float alpha)
{
    filter->alpha = alpha;
    filter->last_value = 0.0f;
    filter->initialized = 0;
}

/**
 * @brief 低通滤波器更新
 */
float MPU6050_LPFilter_Update(LPFilter_t *filter, float new_value)
{
    float filtered_value;
    
    if (!filter->initialized) {
        filter->last_value = new_value;
        filter->initialized = 1;
        filtered_value = new_value;
    } else {
        filtered_value = filter->alpha * new_value + 
                        (1.0f - filter->alpha) * filter->last_value;
        filter->last_value = filtered_value;
    }
    
    return filtered_value;
}

/**
 * @brief 应用滤波器到所有数据
 */
void MPU6050_Apply_Filter(MPU6050_t *mpu6050)
{
    mpu6050->Ax_Filtered = MPU6050_LPFilter_Update(&accel_filter[0], mpu6050->Ax);
    mpu6050->Ay_Filtered = MPU6050_LPFilter_Update(&accel_filter[1], mpu6050->Ay);
    mpu6050->Az_Filtered = MPU6050_LPFilter_Update(&accel_filter[2], mpu6050->Az);
    
    mpu6050->Gx_Filtered = MPU6050_LPFilter_Update(&gyro_filter[0], mpu6050->Gx);
    mpu6050->Gy_Filtered = MPU6050_LPFilter_Update(&gyro_filter[1], mpu6050->Gy);
    mpu6050->Gz_Filtered = MPU6050_LPFilter_Update(&gyro_filter[2], mpu6050->Gz);
}

/**
 * @brief 复位自适应零漂状态（重新上电校准前调用）
 */
void MPU6050_AutoBias_Reset(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        gyro_offset[i]    = 0.0f;
        bias_still_cnt[i] = 0;
    }
    accel_norm_lp      = 1.0f;
    accel_norm_lp_init = 0;
}

/**
 * @brief 静止检测 + 慢速偏置跟踪（运行期自适应零漂核心）
 * @note  每次姿态解算前调用一次（100Hz 下约每 10ms 一帧）。
 *        判据与参数说明见 mpu6050.h 的"自适应零漂配置"注释。
 */
static void MPU6050_Bias_Track(MPU6050_t *mpu6050)
{
#if MPU6050_AUTO_BIAS_ENABLE
    float gyro[3];
    float norm, residual, step;
    uint8_t i, quiet;

    gyro[0] = mpu6050->Gx_Filtered;
    gyro[1] = mpu6050->Gy_Filtered;
    gyro[2] = mpu6050->Gz_Filtered;

    /* 加速度模长：无线性加速度时约等于 1g */
    norm = sqrtf(mpu6050->Ax_Filtered * mpu6050->Ax_Filtered +
                 mpu6050->Ay_Filtered * mpu6050->Ay_Filtered +
                 mpu6050->Az_Filtered * mpu6050->Az_Filtered);

    if (!accel_norm_lp_init) {
        accel_norm_lp      = norm;
        accel_norm_lp_init = 1;
    } else {
        /* 模长慢速平均（τ≈0.5s），用于感知短时震动/平移 */
        accel_norm_lp += 0.02f * (norm - accel_norm_lp);
    }

    /* 载体"安静"：模长≈1g（无姿态/平移运动）且短时稳定（无震动） */
    quiet = (fabsf(norm - 1.0f) < MPU6050_ACCEL_NORM_TOL) &&
            (fabsf(norm - accel_norm_lp) < MPU6050_ACCEL_JITTER_TOL);

    for (i = 0; i < 3; i++) {
        /* 扣除当前偏置后的角速度：是否真的在转 */
        residual = gyro[i] - gyro_offset[i];

        if (quiet && fabsf(residual) < MPU6050_BIAS_MOTION_DPS) {
            if (bias_still_cnt[i] < MPU6050_BIAS_STILL_FRAMES) {
                bias_still_cnt[i]++;          /* 静止时间还不够，暂不学习 */
            } else {
                /* 长时间没有真实转动 → 残余输出就是漂移，慢速吸进偏置 */
                step = MPU6050_BIAS_LEARN_GAIN * residual;
                if (step >  MPU6050_BIAS_MAX_STEP_DPS) step =  MPU6050_BIAS_MAX_STEP_DPS;
                if (step < -MPU6050_BIAS_MAX_STEP_DPS) step = -MPU6050_BIAS_MAX_STEP_DPS;
                gyro_offset[i] += step;

                if (gyro_offset[i] >  MPU6050_BIAS_LIMIT_DPS) gyro_offset[i] =  MPU6050_BIAS_LIMIT_DPS;
                if (gyro_offset[i] < -MPU6050_BIAS_LIMIT_DPS) gyro_offset[i] = -MPU6050_BIAS_LIMIT_DPS;
            }
        } else {
            /* 有运动/转动或抖动帧：只扣 1 帧计数（迟滞，抗门限边缘抖动） */
            if (bias_still_cnt[i] > 0) {
                bias_still_cnt[i]--;
            }
        }
    }
#endif
}

/**
 * @brief 计算姿态角
 */
void MPU6050_Calculate_Angles(MPU6050_t *mpu6050)
{
    static float roll = 0.0f;
    static float pitch = 0.0f;
    static float yaw = 0.0f;
    float dt = 0.01f;
    float accel_roll, accel_pitch;
    float gx, gy, gz;
    float d_yaw, dead;

    /* 1. 运行期自适应零漂：静止检测 + 慢速偏置跟踪（须在补偿/积分前执行） */
    MPU6050_Bias_Track(mpu6050);

    /* 2. 扣除各轴当前偏置后的角速度(°/s) */
    gx = mpu6050->Gx_Filtered - gyro_offset[0];
    gy = mpu6050->Gy_Filtered - gyro_offset[1];
    gz = mpu6050->Gz_Filtered - gyro_offset[2];

    /* 3. 回写偏置估计，便于 OLED 等实时查看自适应结果 */
    mpu6050->Gyro_X_Offset = gyro_offset[0];
    mpu6050->Gyro_Y_Offset = gyro_offset[1];
    mpu6050->Gyro_Z_Offset = gyro_offset[2];

    accel_roll = atan2f(mpu6050->Ay_Filtered, mpu6050->Az_Filtered) * 180.0f / M_PI;
    accel_pitch = atan2f(-mpu6050->Ax_Filtered,
                        sqrtf(mpu6050->Ay_Filtered * mpu6050->Ay_Filtered +
                              mpu6050->Az_Filtered * mpu6050->Az_Filtered)) * 180.0f / M_PI;

    roll = 0.98f * (roll + gx * dt) + 0.02f * accel_roll;
    pitch = 0.98f * (pitch + gy * dt) + 0.02f * accel_pitch;

    /* yaw 无外部参考：补偿后再加死区，低于死区的残差/慢转不积分 */
    d_yaw = gz * dt;
    dead = MPU6050_YAW_DEADBAND_DPS * dt;
    yaw += (d_yaw >= dead || d_yaw <= -dead) ? d_yaw : 0.0f;

    mpu6050->Roll = roll;
    mpu6050->Pitch = pitch;
    mpu6050->Yaw = yaw;
}
