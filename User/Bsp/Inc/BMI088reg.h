#ifndef BMI088REG_H
#define BMI088REG_H

// ==================== 加速度计寄存器 ====================
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E

#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACCEL_XOUT_L         0x12
#define BMI088_ACCEL_XOUT_M         0x13
#define BMI088_ACCEL_YOUT_L         0x14
#define BMI088_ACCEL_YOUT_M         0x15
#define BMI088_ACCEL_ZOUT_L         0x16
#define BMI088_ACCEL_ZOUT_M         0x17

#define BMI088_SENSORTIME_DATA_L    0x18
#define BMI088_SENSORTIME_DATA_M    0x19
#define BMI088_SENSORTIME_DATA_H    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D
#define BMI088_TEMP_M               0x22
#define BMI088_TEMP_L               0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_CONF_MUST_Set    0x80
#define BMI088_ACC_OSR4             0x00
#define BMI088_ACC_OSR2             0x10
#define BMI088_ACC_NORMAL           0x20
#define BMI088_ACC_800_HZ           0x0A
#define BMI088_ACC_1600_HZ          0x0C

#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_RANGE_3G         0x00
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_ACC_RANGE_12G        0x02
#define BMI088_ACC_RANGE_24G        0x03

#define BMI088_INT1_IO_CTRL         0x53
#define BMI088_INT_MAP_DATA         0x58
#define BMI088_ACC_SELF_TEST        0x6D
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E
#define BMI088_ACC_SOFTRESET_VALUE  0xB6

// ==================== 陀螺仪寄存器 ====================
#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

#define BMI088_GYRO_X_L             0x02
#define BMI088_GYRO_X_H             0x03
#define BMI088_GYRO_Y_L             0x04
#define BMI088_GYRO_Y_H             0x05
#define BMI088_GYRO_Z_L             0x06
#define BMI088_GYRO_Z_H             0x07

#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_2000            0x00
#define BMI088_GYRO_1000            0x01
#define BMI088_GYRO_500             0x02
#define BMI088_GYRO_250             0x03
#define BMI088_GYRO_125             0x04

#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_230_HZ     0x01

#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_NORMAL_MODE     0x00
#define BMI088_GYRO_SUSPEND_MODE    0x80

#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL            0x15
#define BMI088_DRDY_ON              0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP  0x18

#define BMI088_GYRO_SELF_TEST       0x3C

// ==================== 常量 ====================
#define BMI088_TEMP_FACTOR          0.125f
#define BMI088_TEMP_OFFSET          23.0f

#define BMI088_ACCEL_6G_SEN         0.00179443359375f
#define BMI088_GYRO_2000_SEN        0.001065264436031695f

#define BMI088_LONG_DELAY_TIME      80

#endif
