#include "bsp_bmi088.h"

// ==================== 静态变量 ====================
static SPI_HandleTypeDef *bmi088_spi = NULL;

static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static float accel_scale = 1.0f;
static bool calib_done = false;

static float accel_sen = BMI088_ACCEL_6G_SEN;
static float gyro_sen = BMI088_GYRO_2000_SEN;

static bool dwt_inited = false;

// ==================== 片选控制 ====================
#ifndef BMI088_ACCEL_CS_GPIO_Port
#define BMI088_ACCEL_CS_GPIO_Port  CS1_ACCEL_GPIO_Port
#define BMI088_ACCEL_CS_Pin        CS1_ACCEL_Pin
#define BMI088_GYRO_CS_GPIO_Port   CS1_GYRO_GPIO_Port
#define BMI088_GYRO_CS_Pin         CS1_GYRO_Pin
#endif

static inline void ACCEL_CS_Low(void)  { HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET); }
static inline void ACCEL_CS_High(void) { HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET); }
static inline void GYRO_CS_Low(void)   { HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET); }
static inline void GYRO_CS_High(void)  { HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET); }

// ==================== 底层读写函数 ====================
static uint8_t Accel_ReadWriteByte(uint8_t tx)
{
    return BSP_SPI_ReadWriteByte(bmi088_spi, tx);
}

// 注意: BMI088 加速度计的 SPI 读时序在地址字节后固定插入 1 个 dummy 字节,
//       陀螺仪没有。单读/多读都必须先丢弃 dummy,否则读到的是错位数据。
static void Accel_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    ACCEL_CS_Low();
    Accel_ReadWriteByte(reg | 0x80);
    Accel_ReadWriteByte(0x55); // dummy byte
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Accel_ReadWriteByte(0x55);
    }
    ACCEL_CS_High();
}

static void Accel_WriteReg(uint8_t reg, uint8_t data)
{
    ACCEL_CS_Low();
    Accel_ReadWriteByte(reg);
    Accel_ReadWriteByte(data);
    ACCEL_CS_High();
}

static uint8_t Accel_ReadReg(uint8_t reg)
{
    uint8_t ret;
    ACCEL_CS_Low();
    Accel_ReadWriteByte(reg | 0x80);
    Accel_ReadWriteByte(0x55); // dummy byte
    ret = Accel_ReadWriteByte(0x55);
    ACCEL_CS_High();
    return ret;
}

static uint8_t Gyro_ReadWriteByte(uint8_t tx)
{
    return BSP_SPI_ReadWriteByte(bmi088_spi, tx);
}

static void Gyro_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    GYRO_CS_Low();
    Gyro_ReadWriteByte(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Gyro_ReadWriteByte(0x55);
    }
    GYRO_CS_High();
}

static void Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    GYRO_CS_Low();
    Gyro_ReadWriteByte(reg);
    Gyro_ReadWriteByte(data);
    GYRO_CS_High();
}

static uint8_t Gyro_ReadReg(uint8_t reg)
{
    uint8_t ret;
    GYRO_CS_Low();
    Gyro_ReadWriteByte(reg | 0x80);
    ret = Gyro_ReadWriteByte(0x55);
    GYRO_CS_High();
    return ret;
}

// ==================== 延时封装 ====================
static inline void Delay_ms(uint32_t ms)
{
    DWT_Delay((float)ms / 1000.0f);
}

static inline void Delay_us(uint32_t us)
{
    DWT_Delay((float)us / 1000000.0f);
}

// ==================== 传感器初始化 ====================
static uint8_t Accel_Init(void)
{
    Accel_WriteReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);

    if (Accel_ReadReg(BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) {
        return 0xFF;
    }

    Accel_WriteReg(BMI088_ACC_PWR_CTRL, 0x04);
    Delay_ms(1);
    Accel_WriteReg(BMI088_ACC_PWR_CONF, 0x00);
    Delay_ms(1);
    Accel_WriteReg(BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set);
    Delay_ms(1);
    Accel_WriteReg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
    Delay_ms(1);
    Accel_WriteReg(BMI088_INT1_IO_CTRL, 0x0A);
    Delay_ms(1);
    Accel_WriteReg(BMI088_INT_MAP_DATA, 0x04);
    Delay_ms(1);

    return 0;
}

static uint8_t Gyro_Init(void)
{
    Gyro_WriteReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);

    if (Gyro_ReadReg(BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) {
        return 0xFF;
    }

    Gyro_WriteReg(BMI088_GYRO_RANGE, BMI088_GYRO_2000);
    Delay_ms(1);
    Gyro_WriteReg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set);
    Delay_ms(1);
    Gyro_WriteReg(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    Delay_ms(1);
    Gyro_WriteReg(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
    Delay_ms(1);
    Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_CONF, 0x08);
    Delay_ms(1);
    Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_MAP, 0x01);
    Delay_ms(1);

    return 0;
}

// ==================== 自动校准 ====================
static void CalibrateOffset(void)
{
    #define CALI_TIMES 6000
    uint8_t buf[8];
    float gyro_sum[3] = {0, 0, 0};
    float g_norm = 0.0f;

    for (uint16_t i = 0; i < CALI_TIMES; i++) {
        // 读加速度
        Accel_ReadMulti(BMI088_ACCEL_XOUT_L, buf, 6);
        int16_t raw_x = (buf[1] << 8) | buf[0];
        int16_t raw_y = (buf[3] << 8) | buf[2];
        int16_t raw_z = (buf[5] << 8) | buf[4];
        float ax = raw_x * accel_sen;
        float ay = raw_y * accel_sen;
        float az = raw_z * accel_sen;
        g_norm += sqrt((double)(ax*ax + ay*ay + az*az));

        // 读陀螺仪
        Gyro_ReadMulti(BMI088_GYRO_X_L, buf, 6);
        int16_t raw_gx = (buf[1] << 8) | buf[0];
        int16_t raw_gy = (buf[3] << 8) | buf[2];
        int16_t raw_gz = (buf[5] << 8) | buf[4];
        gyro_sum[0] += raw_gx * gyro_sen;
        gyro_sum[1] += raw_gy * gyro_sen;
        gyro_sum[2] += raw_gz * gyro_sen;

        Delay_us(500);
    }

    float g_norm_avg = g_norm / CALI_TIMES;

    gyro_offset[0] = gyro_sum[0] / CALI_TIMES;
    gyro_offset[1] = gyro_sum[1] / CALI_TIMES;
    gyro_offset[2] = gyro_sum[2] / CALI_TIMES;

    if (g_norm_avg > 0.1f) {
        accel_scale = 9.81f / g_norm_avg;
    } else {
        accel_scale = 1.0f;
    }

    calib_done = true;
    #undef CALI_TIMES
}

// ==================== 对外接口 ====================
uint8_t BSP_BMI088_Init(SPI_HandleTypeDef *hspi, bool calibrate)
{
    bmi088_spi = hspi;

    if (!dwt_inited) {
        uint32_t cpu_freq_hz = HAL_RCC_GetSysClockFreq();
        uint32_t cpu_freq_mhz = cpu_freq_hz / 1000000;
        DWT_Init(cpu_freq_mhz);
        dwt_inited = true;
    }

    uint8_t err = 0;
    if (Accel_Init() != 0) err |= 0x01; // bit0: 加速度计芯片 ID 读取失败
    if (Gyro_Init() != 0) err |= 0x02;  // bit1: 陀螺仪芯片 ID 读取失败

    if (err) return err;

    if (calibrate) {
        CalibrateOffset();
    } else {
        gyro_offset[0] = 0.0f; gyro_offset[1] = 0.0f; gyro_offset[2] = 0.0f;
        accel_scale = 1.0f;
        calib_done = true;
    }

    return 0;
}

void BSP_BMI088_Read(BSP_IMU_Data_t *data)
{
    uint8_t buf[8];

    Accel_ReadMulti(BMI088_ACCEL_XOUT_L, buf, 6);
    int16_t raw_x = (buf[1] << 8) | buf[0];
    int16_t raw_y = (buf[3] << 8) | buf[2];
    int16_t raw_z = (buf[5] << 8) | buf[4];
    data->Accel[0] = raw_x * accel_sen * accel_scale;
    data->Accel[1] = raw_y * accel_sen * accel_scale;
    data->Accel[2] = raw_z * accel_sen * accel_scale;

    Gyro_ReadMulti(BMI088_GYRO_X_L, buf, 6);
    int16_t raw_gx = (buf[1] << 8) | buf[0];
    int16_t raw_gy = (buf[3] << 8) | buf[2];
    int16_t raw_gz = (buf[5] << 8) | buf[4];
    data->Gyro[0] = raw_gx * gyro_sen - gyro_offset[0];
    data->Gyro[1] = raw_gy * gyro_sen - gyro_offset[1];
    data->Gyro[2] = raw_gz * gyro_sen - gyro_offset[2];

    Accel_ReadMulti(BMI088_TEMP_M, buf, 2);
    int16_t temp_raw = (buf[0] << 3) | (buf[1] >> 5);
    if (temp_raw > 1023) temp_raw -= 2048;
    data->Temperature = temp_raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BSP_BMI088_SetGyroOffset(float ox, float oy, float oz)
{
    gyro_offset[0] = ox;
    gyro_offset[1] = oy;
    gyro_offset[2] = oz;
    calib_done = true;
}

void BSP_BMI088_SetAccelScale(float scale)
{
    accel_scale = scale;
}

void BSP_BMI088_GetGyroOffset(float *ox, float *oy, float *oz)
{
    *ox = gyro_offset[0];
    *oy = gyro_offset[1];
    *oz = gyro_offset[2];
}