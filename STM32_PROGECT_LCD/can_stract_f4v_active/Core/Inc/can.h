/*
 * can.h
 *
 *  Created on: Nov 3, 2024
 *      Author: user
 */

#ifndef CAN_H  // Защита от многократного включения
#define CAN_H

#include "stm32f4xx_hal.h"  // Подключение HAL для STM32F4

// Прототипы функций для инициализации и работы с CAN
void CAN1_Init(void);               // Инициализация CAN
void CAN1_Tx(uint8_t *data, uint8_t length);  // Отправка сообщения по CAN
void CAN1_Rx(void);                 // Прием сообщения по CAN

// Обработчики прерываний для CAN
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);  // Прерывание для почтового ящика 0
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);  // Прерывание для почтового ящика 1
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);  // Прерывание для почтового ящика 2
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);   // Прерывание для приема сообщения в FIFO0
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);               // Прерывание для обработки ошибок CAN

// Функция обработки ошибок
void Error_handler(void);

#endif // CAN_H
