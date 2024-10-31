#ifndef BNO055_DRIVE_H
#define BNO055_DRIVE_H

#include "main.h"

#define BNO055_I2C_ADDR 0x29 // Адрес устройства BNO055

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0x06
#define BNO055_PAGE_ID_ADDR 0x07

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_SYS_TRIGGER_ADDR 0x3F

#define BNO055_EULER_H_LSB_ADDR 0x1A

#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_NDOF 0x0C

HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BNO055_GetEulerAngles(I2C_HandleTypeDef *hi2c, float *yaw, float *roll, float *pitch);

#endif // BNO055_DRIVE_H
