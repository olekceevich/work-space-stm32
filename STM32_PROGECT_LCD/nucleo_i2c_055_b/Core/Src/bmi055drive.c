#include "bno055_drive.h"

HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t mode;
    HAL_StatusTypeDef status;

    // Сброс устройства
    uint8_t reset_cmd = 0x20;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR << 1, BNO055_SYS_TRIGGER_ADDR, I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    HAL_Delay(650); // Ждем завершения сброса

    // Установка режима работы NDOF
    mode = BNO055_OPERATION_MODE_NDOF;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR << 1, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);

    return status;
}

HAL_StatusTypeDef BNO055_GetEulerAngles(I2C_HandleTypeDef *hi2c, float *yaw, float *roll, float *pitch)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR << 1, BNO055_EULER_H_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    *yaw = (float)((int16_t)((data[1] << 8) | data[0])) / 16.0f;
    *roll = (float)((int16_t)((data[3] << 8) | data[2])) / 16.0f;
    *pitch = (float)((int16_t)((data[5] << 8) | data[4])) / 16.0f;

    return HAL_OK;
}
