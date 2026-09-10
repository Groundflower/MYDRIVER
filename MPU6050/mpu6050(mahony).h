#ifndef __MPU6050_H
#define __MPU6050_H

#include "stdint.h"
#include "string.h"
#include "stm32f1xx_hal.h"

/* ==================== 用户配置区 ==================== */
/* 根据实际硬件修改以下引脚定义 */
#define MPU6050_SCL_PIN       GPIO_PIN_1
#define MPU6050_SDA_PIN       GPIO_PIN_2
#define MPU6050_GPIO_PORT     GPIOA

/* ==================== 自适应零漂配置 ==================== */
/* 原理：上电先做一次静态粗校准，运行期再通过"静止检测 + 慢速跟踪"
 * 持续修正三轴零漂，从而抑制温度、供电等引起的动态漂移，无需外部参考。
 *
 * 各轴"静止可学习"判据（全部满足才把该轴残差吸进偏置）：
 *   1) 加速度模长 ≈ 1g 且短时稳定  → 无平移/震动、载体没有姿态运动
 *   2) 该轴扣除当前偏置后的残差 |ω_残| < MPU6050_BIAS_MOTION_DPS
 *   3) 上述状态累计达到 MPU6050_BIAS_STILL_FRAMES 帧（缺帧只扣不减，抗抖动）
 * 学习用一阶慢速跟踪，时间常数约 5s，并限制单帧修正步长与偏置上下限。
 *
 * 代价：低于 MOTION_DPS 的真实极慢转动（默认 0.5°/s 以下）会被逐渐当作
 * 零漂吃掉（这正是 yaw 零漂与慢转的固有取舍），请按应用最低可用转速调整。
 *
 * 注意：若上电校准瞬间设备没静止、初始偏差很大，由于残差超过 MOTION_DPS
 * 将不会自动回收，请重新静止上电校准（或调用 MPU6050_AutoBias_Reset()）。
 * 缓慢的温漂不会卡住：它必先经过小于 MOTION_DPS 的区间，会被持续跟踪。 */
#define MPU6050_AUTO_BIAS_ENABLE   1          /* 1=开运行期自适应；0=仅上电静态校准 */
#define MPU6050_BIAS_MOTION_DPS    0.8f       /* 扣除偏置后低于该角速度(°/s)视为"没在转" */
#define MPU6050_BIAS_STILL_FRAMES  50         /* 连续静止帧数（100Hz 下约 0.5s） */
#define MPU6050_BIAS_LEARN_GAIN    0.002f     /* 单帧学习率，对应时间常数约 5s */
#define MPU6050_BIAS_MAX_STEP_DPS  0.02f      /* 每帧偏置最大修正量(°/s)，防止异常样本拉飞 */
#define MPU6050_BIAS_LIMIT_DPS     5.0f       /* 偏置估计上下限(°/s) */
#define MPU6050_ACCEL_NORM_TOL     0.05f      /* |加速度模长-1g| 容差(g)，超此视为有运动 */
#define MPU6050_ACCEL_JITTER_TOL   0.02f      /* 模长短时抖动容差(g)，超此视为有震动 */
#define MPU6050_INIT_CALIB_SAMPLES 20         /* 上电静态校准采样次数（设备须静止） */

/* yaw 死区：扣除偏置后仍小于该角速度(°/s)则不积分（低于它的慢转/残差被消除） */
#define MPU6050_YAW_DEADBAND_DPS   0.5f

/* ==================== 姿态融合选择 ==================== */
/* 0 = 原互补滤波（默认）：与旧版行为一致，yaw 无限累积，
 *     roll/pitch 由 加速度计修正 + 陀螺仪积分 的互补滤波得到。
 * 1 = Mahony 四元数融合：接近 ±90° 甚至翻转也不 gimbal lock；
 *     yaw 从四元数解出，范围会折叠到 ±180°（要无限累积需在应用层 unwrap）。
 * 两种模式共用同一套"自适应零漂 + yaw 死区"，互不干扰，改宏烧录即可对比。 */
#define MPU6050_USE_MAHONY       1

/* Mahony 增益(仅 MPU6050_USE_MAHONY=1 生效)：
 * Kp 越大加速度计修正越强，但过大会在动态/倾斜时把姿态往错误方向拉，
 * 建议 0.5~2.0（不要动辄调到十几）。Ki 默认 0：陀螺偏置已由自适应零漂
 * 处理，Ki 只会在转动时把误差积分进陀螺、拖慢真实转动。 */
#define MPU6050_MAHONY_KP        2.0f
#define MPU6050_MAHONY_KI        0.03f

/* ==================== MPU6050寄存器定义 ==================== */
#define MPU6050_ADDR            0xD0    /* AD0接地时的地址 */
#define MPU6050_REG_SMPLRT_DIV  0x19    /* 采样率分频器 */
#define MPU6050_REG_CONFIG      0x1A    /* 配置寄存器 */
#define MPU6050_REG_GYRO_CONFIG 0x1B    /* 陀螺仪配置 */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /* 加速度计配置 */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   /* 加速度计X轴高字节 */
#define MPU6050_REG_TEMP_OUT_H  0x41    /* 温度高字节 */
#define MPU6050_REG_GYRO_XOUT_H 0x43    /* 陀螺仪X轴高字节 */
#define MPU6050_REG_PWR_MGMT_1  0x6B    /* 电源管理1 */
#define MPU6050_REG_WHO_AM_I    0x75    /* 设备ID */

/* ==================== 枚举定义 ==================== */
typedef enum {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18
} MPU6050_AccelRange_t;

typedef enum {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18
} MPU6050_GyroRange_t;

typedef enum {
    DLPF_256HZ = 0x00,
    DLPF_188HZ = 0x01,
    DLPF_98HZ = 0x02,
    DLPF_42HZ = 0x03,
    DLPF_20HZ = 0x04,
    DLPF_10HZ = 0x05,
    DLPF_5HZ = 0x06
} MPU6050_DLPF_t;

/* ==================== 数据结构定义 ==================== */
/* 单位说明：Ax~Az 为 g，Gx~Gz 为 °/s，*_Filtered 同原值，
 * Gyro_*_Offset 为自适应零漂估计（°/s），Temp 为 ℃ */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    int16_t Temp_RAW;
    
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float Temp;
    
    float Ax_Filtered;
    float Ay_Filtered;
    float Az_Filtered;
    float Gx_Filtered;
    float Gy_Filtered;
    float Gz_Filtered;
    
    float Roll;
    float Pitch;
    float Yaw;
    float Gyro_Z_Offset;
    float Gyro_X_Offset;   /* 自适应零漂估计，每帧姿态解算后回写（°/s） */
    float Gyro_Y_Offset;   /* 自适应零漂估计，每帧姿态解算后回写（°/s） */
    float Q0, Q1, Q2, Q3;  /* Mahony 姿态四元数(w,x,y,z)，仅 USE_MAHONY=1 时更新 */
    float Dt;              /* 最近一次姿态解算的实际采样周期(s)，便于核对 */
} MPU6050_t;

typedef struct {
    float alpha;
    float last_value;
    uint8_t initialized;
} LPFilter_t;

/* ==================== 函数声明 ==================== */
void MPU6050_DWT_Init(void);
void MPU6050_Delay_Us(uint32_t us);
uint8_t MPU6050_Init(MPU6050_t *mpu6050);
uint8_t MPU6050_Read_All(MPU6050_t *mpu6050);
uint8_t MPU6050_Read_Accel(MPU6050_t *mpu6050);
uint8_t MPU6050_Read_Gyro(MPU6050_t *mpu6050);
uint8_t MPU6050_Read_Temp(MPU6050_t *mpu6050);
void MPU6050_Set_Accel_Range(MPU6050_AccelRange_t range);
void MPU6050_Set_Gyro_Range(MPU6050_GyroRange_t range);
void MPU6050_Set_DLPF(MPU6050_DLPF_t dlpf);
void MPU6050_Set_Sample_Rate(uint8_t rate);
void MPU6050_LPFilter_Init(LPFilter_t *filter, float alpha);
float MPU6050_LPFilter_Update(LPFilter_t *filter, float new_value);
void MPU6050_Apply_Filter(MPU6050_t *mpu6050);
void MPU6050_Calculate_Angles(MPU6050_t *mpu6050);
void MPU6050_AutoBias_Reset(void);   /* 复位自适应零漂状态 */
void MPU6050_Fusion_Init(void);      /* 初始化/复位姿态融合状态（四元数等） */
static uint8_t MPU6050_I2C_Read(uint8_t reg);

#endif /* __MPU6050_H */
