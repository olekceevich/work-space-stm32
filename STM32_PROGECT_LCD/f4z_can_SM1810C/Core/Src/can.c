#include "can.h"
#include "main.h"

// Экземпляры и внешние переменные
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

// Функция инициализации CAN с настройкой фильтра для ID 0x65D и 0x651
void CAN1_Init_Custom(void) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Starting CAN Initialization...\r\n", 31, HAL_MAX_DELAY);

    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterIdHigh = 0x0000;  // Устанавливаем ID 0x65D (основной ID)
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;  // Маска для точного совпадения с ID
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Filter Config Failed!\r\n", 27, HAL_MAX_DELAY);
        Error_handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Filter Config Successful\r\n", 30, HAL_MAX_DELAY);
    }

    // Включение уведомлений для приема сообщений
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Notification Activation Failed!\r\n", 38, HAL_MAX_DELAY);
        Error_handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Notifications Activated\r\n", 30, HAL_MAX_DELAY);
    }

    // Запуск CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Start Failed!\r\n", 19, HAL_MAX_DELAY);
        Error_handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Started Successfully\r\n", 27, HAL_MAX_DELAY);
    }
}

// Обработчик прерываний для приема сообщений по CAN (FIFO0)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN RX Callback Entered\r\n", 26, HAL_MAX_DELAY);

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rcvd_msg[8];
    char msg[50];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN RX Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    } else {
        // Успешное получение сообщения
        HAL_UART_Transmit(&huart2, (uint8_t *)"Message Received!\r\n", 20, HAL_MAX_DELAY);

        if (RxHeader.StdId == 0x65D && RxHeader.RTR == 0) {
            // Интерпретируем данные как строку
            rcvd_msg[RxHeader.DLC] = '\0';  // Завершаем строку нулевым символом
            sprintf(msg, "Data Frame with ID 0x65D, Message: %s\r\n", rcvd_msg);
            LED_Manage_Output(rcvd_msg[0]);
        } else if (RxHeader.StdId == 0x651 && RxHeader.RTR == 1) {
            sprintf(msg, "Remote Frame with ID 0x651\r\n");
            Send_response(RxHeader.StdId);
        } else if (RxHeader.StdId == 0x651 && RxHeader.RTR == 0) {
            sprintf(msg, "Reply Frame with ID 0x651, Data: %#X\r\n", rcvd_msg[0] << 8 | rcvd_msg[1]);
        } else {
            sprintf(msg, "Unknown ID: 0x%x\r\n", RxHeader.StdId);
        }

        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
// Функция управления светодиодом
void LED_Manage_Output(uint8_t led_no) {
    // Здесь можно добавить логику управления светодиодом в зависимости от `led_no`
    char msg[50];
    sprintf(msg, "LED Manage Output: %d\r\n", led_no);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Функция для отправки ответа на запрос
void Send_response(uint32_t StdId) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t response[2] = {0xAB, 0xCD};  // Пример ответа

    TxHeader.DLC = 2;
    TxHeader.StdId = StdId;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, response, &TxMailbox) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Tx Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Response Sent Successfully\r\n", 28, HAL_MAX_DELAY);
    }
}

// Функция обработки ошибок
void Error_handler(void) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Entering Error Handler\r\n", 24, HAL_MAX_DELAY);
    __disable_irq();  // Отключаем прерывания
    while (1) {
        // Здесь можно добавить индикацию ошибки с помощью светодиода
    }
}
