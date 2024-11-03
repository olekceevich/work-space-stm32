#ifndef CAN_H
#define CAN_H

#include "stm32f4xx_hal.h"  // Подключаем HAL для STM32
#include <stdint.h>         // Для работы с типами данных

// Прототипы функций, используемых в can.c
void CAN1_Init_Custom(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void LED_Manage_Output(uint8_t led_no);
void Send_response(uint32_t StdId);
void Error_handler(void);

#endif /* CAN_H */
