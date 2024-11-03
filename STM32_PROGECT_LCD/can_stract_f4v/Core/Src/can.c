#include "can.h"
#include "main.h"

// Экземпляры и внешние переменные
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

// Функция инициализации CAN
void CAN1_Init(void) {
    // Базовая инициализация CAN
    HAL_UART_Transmit(&huart2, (uint8_t *)"Initializing CAN...\r\n", 21, HAL_MAX_DELAY);
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Init Failed!\r\n", 18, HAL_MAX_DELAY);
        Error_handler();
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Initialized Successfully\r\n", 30, HAL_MAX_DELAY);

    // Настройка фильтра для приема сообщений CAN
    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Filter Config Failed!\r\n", 27, HAL_MAX_DELAY);
        Error_handler();
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Filter Configured Successfully\r\n", 37, HAL_MAX_DELAY);

    // Включение уведомлений для CAN
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Notification Activation Failed!\r\n", 38, HAL_MAX_DELAY);
        Error_handler();
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Notifications Activated\r\n", 30, HAL_MAX_DELAY);

    // Запуск CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Start Failed!\r\n", 19, HAL_MAX_DELAY);
        Error_handler();
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Started Successfully\r\n", 27, HAL_MAX_DELAY);
}

// Функция передачи сообщения по CAN
void CAN1_Tx(uint8_t *data, uint8_t length) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // Настройка заголовка сообщения CAN
    TxHeader.DLC = length;
    TxHeader.StdId = 0x65D;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    char msg[50];
    sprintf(msg, "Sending CAN Message: %s\r\n", data);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);  // Логирование

    // Отправка сообщения
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Tx Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Message Sent\r\n", 18, HAL_MAX_DELAY);
    }
}

// Функция приема сообщения по CAN и передачи его по UART
void CAN1_Rx(void) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t receivedData[8];
    char msg[50];

    // Получение сообщения из FIFO0
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, receivedData) == HAL_OK) {
        sprintf(msg, "Message Received: %s\r\n", receivedData);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Rx Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    }
}

// Обработчики прерываний для завершения передачи по почтовым ящикам CAN
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Message Transmitted: Mailbox 0\r\n", 33, HAL_MAX_DELAY);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Message Transmitted: Mailbox 1\r\n", 33, HAL_MAX_DELAY);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Message Transmitted: Mailbox 2\r\n", 33, HAL_MAX_DELAY);
}

// Обработчик прерываний для приема сообщений по CAN (FIFO0)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rcvd_msg[8];
    char msg[50];

    // Чтение сообщения из FIFO0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rcvd_msg) == HAL_OK) {
        sprintf(msg, "Message Received: %s\r\n", rcvd_msg);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Rx Error\r\n", 14, HAL_MAX_DELAY);
        Error_handler();
    }
}

// Обработчик ошибок CAN
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"CAN Error Detected\r\n", 20, HAL_MAX_DELAY);
}

// Функция обработки ошибок
void Error_handler(void) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"Entering Error Handler\r\n", 24, HAL_MAX_DELAY);
    __disable_irq();  // Отключаем прерывания
    while (1) {
        // Можно добавить мигание светодиодом для индикации ошибки
    }
}
