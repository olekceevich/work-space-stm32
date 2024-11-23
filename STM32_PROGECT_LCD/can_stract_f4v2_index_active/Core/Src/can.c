#include "can.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// Экземпляры и внешние переменные
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern uint8_t buttonPressed;  // Флаг нажатия кнопки

// Константы для состояний Heartbeat
#define HEARTBEAT_STARTUP_STATE         0x00
#define HEARTBEAT_CONNECTION_STATE      0x02
#define HEARTBEAT_STOP_STATE            0x04
#define HEARTBEAT_OPERATING_STATE       0x05
#define HEARTBEAT_PRE_OPERATION_STATE   0x7F

// Интервал Heartbeat-сообщения в миллисекундах (по умолчанию 2 секунды)
#define HEARTBEAT_INTERVAL_MS           2000

// Идентификатор COB_ID для Heartbeat (0x700 + motor ID)
#define HEARTBEAT_FUNCTION_CODE         0x700

// Текущее состояние устройства
uint8_t current_state = HEARTBEAT_STARTUP_STATE;

// Функция инициализации CAN
void CAN1_Init(void) {
    // Настройка CAN
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Init Failed!\r\n", 18, HAL_MAX_DELAY);
        Error_handler();
    }

    // Настройка фильтра для приема heartbeat-сообщений
    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterIdHigh = (HEARTBEAT_FUNCTION_CODE << 5) & 0xFFFF;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0xFFFF;
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Filter Config Failed!\r\n", 27, HAL_MAX_DELAY);
        Error_handler();
    }

    // Включение уведомлений для CAN
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Notification Activation Failed!\r\n", 38, HAL_MAX_DELAY);
        Error_handler();
    }

    // Запуск CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Start Failed!\r\n", 19, HAL_MAX_DELAY);
        Error_handler();
    }
}

// Функция отправки Heartbeat-сообщения
void CAN_SendHeartbeat(uint8_t node_id) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t data[1] = {current_state};
    uint32_t txMailbox;

    // Настраиваем COB_ID для Heartbeat сообщения
    txHeader.StdId = HEARTBEAT_FUNCTION_CODE + node_id;
    txHeader.DLC = 1;            // Один байт данных (текущее состояние)
    txHeader.IDE = CAN_ID_STD;   // Стандартный формат идентификатора
    txHeader.RTR = CAN_RTR_DATA; // Тип сообщения - данные

    // Отправка Heartbeat-сообщения
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Heartbeat Tx Error\r\n", 20, HAL_MAX_DELAY);
        Error_handler();
    }
}

// Обработчик приема CAN сообщений (для приема Heartbeat)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rcvd_msg[1];
    char msg[50];

    // Получение сообщения
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rcvd_msg) == HAL_OK) {
        sprintf(msg, "Heartbeat Received: State = 0x%02X\r\n", rcvd_msg[0]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Rx Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    }
}

// Обновление состояния Heartbeat и отправка сообщения
void UpdateHeartbeatState(uint8_t new_state, uint8_t node_id) {
    current_state = new_state;
    CAN_SendHeartbeat(node_id);
}

// Функция обработки нажатия кнопки
void CheckButtonAndSendHeartbeat(uint8_t node_id) {
    if (buttonPressed) {  // Проверка флага нажатия кнопки
        buttonPressed = 0;  // Сброс флага
        UpdateHeartbeatState(current_state, node_id);
    }
}

// Основной цикл (Heartbeat отправляется каждые 2 секунды)
void Heartbeat_Loop(uint8_t node_id) {
    while (1) {
        CheckButtonAndSendHeartbeat(node_id);
        HAL_Delay(HEARTBEAT_INTERVAL_MS);  // Интервал отправки 2 секунды
        UpdateHeartbeatState(current_state, node_id);
    }
}

// Функция обработки ошибок
void Error_handler(void) {
    __disable_irq();  // Отключаем все прерывания
    while (1) {
        // Можно добавить мигание светодиодом для индикации ошибки
    }
}
