#include "can.h"
#include "stm32f1xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

#define LEFT_WHEEL_ID 3
#define RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE 0x700
#define RAW_CAN_RESPONSE_ARBITRATION_CODE 0x580
#define CANOPEN_SDO_INDEX_STATUS_WORD 0x6041
#define CANOPEN_SDO_SUB_INDEX_STATUS_WORD 0x00

#define CANOPEN_SDO_VALUE_STATUS_WORD_SERVO_READY 0x0001
#define CANOPEN_SDO_VALUE_STATUS_WORD_SERVO_RUNNING_ENABLED 0x0004

#define CANOPEN_SDO_INDEX_MODE_OF_OPERATION 0x6060
#define CANOPEN_SDO_SUB_INDEX_MODE_OF_OPERATION 0x00
#define CANOPEN_SDO_VALUE_MODE_OF_OPERATION_POSITION 0x01

#define CANOPEN_SDO_INDEX_POSITION_MODE_ACCELERATION_TIME 0x6083
#define CANOPEN_SDO_SUB_INDEX_POSITION_MODE_ACCELERATION_TIME 0x00

#define CANOPEN_SDO_INDEX_POSITION_MODE_DECELERATION_TIME 0x6084
#define CANOPEN_SDO_SUB_INDEX_POSITION_MODE_DECELERATION_TIME 0x00

#define CANOPEN_SDO_INDEX_POSITION_MODE_TARGET_POSITION 0x607A
#define CANOPEN_SDO_SUB_INDEX_POSITION_MODE_TARGET_POSITION 0x00

#define CANOPEN_SDO_INDEX_POSITION_MODE_MAX_SPEED 0x6081
#define CANOPEN_SDO_SUB_INDEX_POSITION_MODE_MAX_SPEED 0x00

const uint8_t RAW_CAN_BOOTLOADER_HEARTBEAT_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t CANOPEN_CONFIRMATION_DATA[1] = {0x7F};

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;



ActivationState currentState = STATE_INIT;
uint32_t stateTimer = 0;
volatile uint8_t heartbeat_received = 0;
volatile uint8_t canopen_confirmation_received = 0;

static void CAN_ConfigFilter(void);
static uint8_t wait_for_heartbeat(uint32_t timeout_ms);
static uint8_t send_command(uint8_t *data, uint8_t len);
void CAN_Init(void);
void LogMessage(const char *message);
void LogError(const char *message);

void CAN_Init(void) {
    CAN_ConfigFilter();
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        LogError("CAN_Init: Error starting CAN");
    } else {
        LogMessage("CAN_Init: CAN started successfully");
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        LogError("CAN_Init: Error activating CAN notifications");
    } else {
        LogMessage("CAN_Init: Notifications activated");
    }
}

void StartActivationProcess(void) {
    // Переход в Operational Mode
    uint8_t set_operational[] = {0x01};  // Команда для Operational Mode
    send_command(set_operational, sizeof(set_operational));
    HAL_Delay(1000);  // Ожидание перехода

    currentState = STATE_WAIT_BOOTLOADER_HEARTBEAT;
    stateTimer = HAL_GetTick();
    LogMessage("Starting activation process...");
}



static uint16_t _get_sdo_uint16(uint16_t index, uint8_t sub_index) {
    // Буфер данных для передачи и приема
    uint8_t request[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)((index >> 8) & 0xFF), sub_index, 0x00, 0x00, 0x00, 0x00};
    uint16_t value = 0;

    // Конфигурация заголовка CAN для запроса
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = LEFT_WHEEL_ID;  // Замените на ID устройства
    TxHeader.DLC = 8;                // Длина данных 8 байт
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;

    uint32_t TxMailbox;

    // Отправка запроса через CAN
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, request, &TxMailbox) != HAL_OK) {
        // Обработка ошибки при отправке
        return 0xFFFF;  // Вернуть значение, указывающее на ошибку
    }

    // Ожидание ответа с тайм-аутом
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < 1000) { // Тайм-аут 1 секунда
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        // Проверяем, есть ли новые сообщения
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
            if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                // Проверяем, что ответ от нужного устройства и содержит данные SDO
                if (RxHeader.StdId == (LEFT_WHEEL_ID | RAW_CAN_RESPONSE_ARBITRATION_CODE) &&
                    RxData[1] == (uint8_t)(index & 0xFF) &&
                    RxData[2] == (uint8_t)((index >> 8) & 0xFF) &&
                    RxData[3] == sub_index) {
                    // Извлекаем значение (2 байта) из ответа
                    value = RxData[4] | (RxData[5] << 8);
                    return value;  // Возвращаем значение
                }
            }
        }
        HAL_Delay(10);  // Небольшая задержка для предотвращения перегрузки
    }

    // Если ответ не получен, вернуть значение, указывающее на ошибку
    return 0xFFFF;
}

void CAN_ProcessStateMachine(void) {
    uint8_t activation_data[2] = {0x10, 0x10};

    switch (currentState) {
        case STATE_WAIT_BOOTLOADER_HEARTBEAT:
            LogMessage("STATE_WAIT_BOOTLOADER_HEARTBEAT");
            if (wait_for_heartbeat(6000)) {
                LogMessage("Bootloader heartbeat received.");
                currentState = STATE_GET_BOOTLOADER_VERSION;
                stateTimer = HAL_GetTick();
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to receive bootloader heartbeat.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_GET_BOOTLOADER_VERSION:
            LogMessage("STATE_GET_BOOTLOADER_VERSION");
            if (send_command((uint8_t*) "\x20\x20", 2)) {
                if (wait_for_heartbeat(6000)) {
                    LogMessage("Bootloader version received.");
                    currentState = STATE_SEND_ACTIVATION;
                    stateTimer = HAL_GetTick();
                }
            } else {
                LogError("Failed to send bootloader version request.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_SEND_ACTIVATION:
            LogMessage("STATE_SEND_ACTIVATION");
            if (send_command(activation_data, 2)) {
                if (wait_for_heartbeat(6000)) {
                    LogMessage("Activation command acknowledged.");
                    currentState = STATE_CONFIRM_CANOPEN_MODE;
                    stateTimer = HAL_GetTick();
                }
            } else {
                LogError("Failed to send activation command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CONFIRM_CANOPEN_MODE:
            LogMessage("STATE_CONFIRM_CANOPEN_MODE");
            if (wait_for_heartbeat(5000)) {
                LogMessage("CANopen mode confirmed.");
                currentState = STATE_ROTATE_WHEEL;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to confirm CANopen mode.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;
        case STATE_ROTATE_WHEEL: {
            LogMessage("STATE_ROTATE_WHEEL: Preparing wheel for position mode");

            // Переход в состояние Operational
            uint8_t set_operational[] = {0x01};  // NMT команд на "Operational"
            send_command(set_operational, sizeof(set_operational));
            HAL_Delay(2000);

            // Установка режима позиционирования
            uint8_t set_position_mode[] = {0x2F, 0x60, 0x60, 0x00, 0x01};  // Позиционный режим
            send_command(set_position_mode, sizeof(set_position_mode));
            HAL_Delay(2000);

            // Проверка статуса перед установкой позиции
            uint16_t status = _get_sdo_uint16(CANOPEN_SDO_INDEX_STATUS_WORD, CANOPEN_SDO_SUB_INDEX_STATUS_WORD);
            if (!(status & CANOPEN_SDO_VALUE_STATUS_WORD_SERVO_READY)) {
                LogError("Wheel is not ready for position mode.");
                currentState = STATE_ACTIVATION_FAILED;
                break;
            }

            // Установка времени разгона
            uint8_t set_acceleration_time[] = {0x23, 0x83, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00};  // 1000 мс
            send_command(set_acceleration_time, sizeof(set_acceleration_time));
            HAL_Delay(2000);

            // Установка времени торможения
            uint8_t set_deceleration_time[] = {0x23, 0x84, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00};  // 1000 мс
            send_command(set_deceleration_time, sizeof(set_deceleration_time));
            HAL_Delay(2000);

            // Установка максимальной скорости
            uint8_t set_max_speed[] = {0x23, 0x81, 0x60, 0x00, 0x88, 0x13, 0x00, 0x00};  // 5000 (0.1 rpm)
            send_command(set_max_speed, sizeof(set_max_speed));
            HAL_Delay(2000);

            // Установка целевой позиции на 3 оборота
            uint8_t set_target_position[] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0xC0, 0x00, 0x00};  // 3 оборота
            send_command(set_target_position, sizeof(set_target_position));
            LogMessage("Wheel target position set to 3 turns.");
            HAL_Delay(2000);

            // Подготовка и включение сервопривода
            uint8_t servo_preparation[] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00};
            send_command(servo_preparation, sizeof(servo_preparation));
            HAL_Delay(2000);

            // Включение сервопривода
            uint8_t servo_enable[] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00};  // Полное включение
            send_command(servo_enable, sizeof(servo_enable));
            HAL_Delay(2000);

            // Старт привода
            uint8_t servo_start[] = {0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00};
            send_command(servo_start, sizeof(servo_start));
            LogMessage("Servo started for position mode.");
            HAL_Delay(2000);

            // Ожидание достижения цели
            uint32_t start_time = HAL_GetTick();
            while ((HAL_GetTick() - start_time) < 15000) {
                status = _get_sdo_uint16(CANOPEN_SDO_INDEX_STATUS_WORD, CANOPEN_SDO_SUB_INDEX_STATUS_WORD);
                if (status & CANOPEN_SDO_VALUE_STATUS_WORD_SERVO_RUNNING_ENABLED) {
                    LogMessage("Wheel reached target position.");
                    currentState = STATE_ACTIVATION_COMPLETE;
                    break;
                }
                HAL_Delay(100);  // Периодическая проверка
            }

            // Проверка на неудачу
            if (currentState != STATE_ACTIVATION_COMPLETE) {
                LogError("Wheel failed to reach target position in time.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;
        }



        case STATE_ACTIVATION_FAILED:
            LogError("Activation failed.");
            currentState = STATE_INIT;
            break;

        case STATE_ACTIVATION_COMPLETE:
            LogMessage("Activation process completed successfully.");
            currentState = STATE_INIT;
            break;

        default:
            break;
    }
}


void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC) {
    char logBuffer[200];
    snprintf(logBuffer, sizeof(logBuffer),
             "Received CAN message: ID=0x%X, DLC=%d, Data=[%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
             received_ID, DLC, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    LogMessage(logBuffer);

    if (received_ID == (LEFT_WHEEL_ID | RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE) &&
        memcmp(data, RAW_CAN_BOOTLOADER_HEARTBEAT_DATA, 8) == 0) {
        LogMessage("Bootloader heartbeat message received.");
        heartbeat_received = 1;
    }
    else if (received_ID == (LEFT_WHEEL_ID | RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE) &&
             data[0] == CANOPEN_CONFIRMATION_DATA[0]) {
        LogMessage("CANopen confirmation message received.");
        canopen_confirmation_received = 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        process_CAN_message(RxHeader.StdId, RxData, RxHeader.DLC);
    }
}

static uint8_t send_command(uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.StdId = LEFT_WHEEL_ID;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        LogError("Error sending command to CAN.");
        return 0;
    } else {
        LogMessage("Command sent successfully to CAN");
    }

    uint32_t startTime = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {
        if (HAL_GetTick() - startTime > 500) {
            LogError("Timeout waiting for message to be sent.");
            return 0;
        }
    }
    LogMessage("Message sent successfully from TxMailbox");
    return 1;
}

static uint8_t wait_for_heartbeat(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();

    heartbeat_received = 0;
    canopen_confirmation_received = 0;

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (currentState == STATE_CONFIRM_CANOPEN_MODE && canopen_confirmation_received) {
            LogMessage("CANopen mode confirmed.");
            return 1;
        }
        if (heartbeat_received) {
            LogMessage("Expected heartbeat received.");
            return 1;
        }
        HAL_Delay(10);
    }
    LogMessage("Timeout: heartbeat not received.");
    return 0;
}

static void CAN_ConfigFilter(void) {
    CAN_FilterTypeDef FilterConfig = {0};
    FilterConfig.FilterBank = 0;
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0000;
    FilterConfig.FilterIdLow = 0x0000;
    FilterConfig.FilterMaskIdHigh = 0x0000;
    FilterConfig.FilterMaskIdLow = 0x0000;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    FilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK) {
        LogError("CAN_ConfigFilter: Filter configuration failed");
        Error_Handler();
    } else {
        LogMessage("CAN_ConfigFilter: Filter configured successfully (all messages pass)");
    }
}

void LogMessage(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

void LogError(const char *message) {
    LogMessage(message);
}
