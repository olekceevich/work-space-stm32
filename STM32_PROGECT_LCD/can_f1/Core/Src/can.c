#include "can.h"
#include "stm32f1xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

// Идентификаторы колес
#define LEFT_WHEEL_ID 3

// Коды арбитрации для загрузчика и ответа
#define RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE 0x700
#define RAW_CAN_RESPONSE_ARBITRATION_CODE 0x580

// Данные для загрузчика и подтверждения CANopen
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
void change_node_id_and_save(uint8_t new_id);
uint8_t received_data[8]; // Глобальный массив для хранения полученных данных

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
    currentState = STATE_WAIT_BOOTLOADER_HEARTBEAT;
    stateTimer = HAL_GetTick();
    LogMessage("Starting activation process...");
}

void CAN_ProcessStateMachine(void) {
    uint8_t activation_data[2] = {0x10, 0x10};
    uint8_t firmware_confirmation_data[2] = {0xAA, 0xBB};
    uint8_t bootloader_version_query_data[2] = {0x20, 0x20};
    uint8_t expected_response_id[] = {0xCC, 0xDD};
    uint8_t expected_version[] = {0x00, 0x01};
    uint8_t soft_reset_command[] = {0x81, 0x00};  // Команда для сброса устройства
    uint8_t diagnostic_counter = 0;  // Счетчик для диагностических сообщений

    switch (currentState) {
        case STATE_WAIT_BOOTLOADER_HEARTBEAT:
            LogMessage("STATE_WAIT_BOOTLOADER_HEARTBEAT");
            if (wait_for_heartbeat(6000)) {
                LogMessage("Bootloader heartbeat received.");
                currentState = STATE_SEND_ACTIVATION;
                stateTimer = HAL_GetTick();
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to receive bootloader heartbeat.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_SEND_ACTIVATION:
            LogMessage("STATE_SEND_ACTIVATION");
            if (send_command(activation_data, sizeof(activation_data))) {
                LogMessage("Activation command sent, waiting for firmware response.");
                stateTimer = HAL_GetTick();
                currentState = STATE_WAIT_FIRMWARE_CONFIRMATION;
            } else {
                LogError("Failed to send activation command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_FIRMWARE_CONFIRMATION:
            LogMessage("STATE_WAIT_FIRMWARE_CONFIRMATION");
            if (HAL_GetTick() - stateTimer > 3000) {
                LogError("Timeout waiting for firmware confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            } else if (heartbeat_received || canopen_confirmation_received) {
                if (memcmp(received_data, firmware_confirmation_data, sizeof(firmware_confirmation_data)) == 0) {
                    LogMessage("Firmware confirmation received (AA BB). Proceeding to bootloader version query.");
                    currentState = STATE_GET_BOOTLOADER_VERSION;
                    stateTimer = HAL_GetTick();
                } else {
                    LogMessage("Received message, but it does not match firmware confirmation (AA BB).");
                }
            }
            break;

        case STATE_GET_BOOTLOADER_VERSION:
            LogMessage("STATE_GET_BOOTLOADER_VERSION");
            if (send_command(bootloader_version_query_data, sizeof(bootloader_version_query_data))) {
                LogMessage("Bootloader version query command [0x20 0x20] sent, waiting for response.");

                uint32_t start_time = HAL_GetTick();

                while (HAL_GetTick() - start_time < 6000) {
                    if (heartbeat_received) {
                        LogMessage("Received expected heartbeat during version check.");
                        heartbeat_received = 0; // сбрасываем флаг после получения сообщения
                    }
                    HAL_Delay(10);
                }

                if (!heartbeat_received) {
                    LogError("Timeout waiting for bootloader version response.");
                    currentState = STATE_ACTIVATION_FAILED;
                }
            } else {
                LogError("Failed to send bootloader version query command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

/*
        case STATE_GET_BOOTLOADER_VERSION:
            LogMessage("STATE_GET_BOOTLOADER_VERSION");
            if (send_command(bootloader_version_query_data, sizeof(bootloader_version_query_data))) {
                LogMessage("Bootloader version query command sent, waiting for response.");

                uint32_t start_time = HAL_GetTick();
                while (HAL_GetTick() - start_time < 6000) {
                    if (heartbeat_received) {
                        char logBuffer[100];
                        snprintf(logBuffer, sizeof(logBuffer),
                                 "Received CAN message during version check: ID=0x%X, Data=[%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
                                 received_data[0], received_data[1], received_data[2], received_data[3],
                                 received_data[4], received_data[5], received_data[6], received_data[7]);
                        LogMessage(logBuffer);

                        // Логируем только первые 5 сообщений и далее каждое пятое
                        if (diagnostic_counter < 5 || diagnostic_counter % 5 == 0) {
                            LogMessage("Diagnostic: Received unexpected message format. Logging data for analysis.");
                        }
                        diagnostic_counter++;
                    }
                    HAL_Delay(10);
                }

                if (diagnostic_counter == 0) {
                    LogError("Timeout waiting for bootloader version response.");
                    currentState = STATE_ACTIVATION_FAILED;
                }
            } else {
                LogError("Failed to send bootloader version query command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;
*/
        case STATE_SOFT_RESET:
            LogMessage("STATE_SOFT_RESET: Performing soft reset");
            if (send_command(soft_reset_command, sizeof(soft_reset_command))) {
                LogMessage("Soft reset command sent successfully.");
                currentState = STATE_INIT;  // Возврат в начальное состояние после сброса
            } else {
                LogError("Failed to send soft reset command.");
                currentState = STATE_ACTIVATION_FAILED;  // Переход в состояние ошибки, если сброс не удался
            }
            break;

        case STATE_CONFIRM_CANOPEN_MODE:
            LogMessage("STATE_CONFIRM_CANOPEN_MODE");
            if (wait_for_heartbeat(5000)) {
                LogMessage("CANopen mode confirmed.");
                currentState = STATE_CHANGE_NODE_ID;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to confirm CANopen mode.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CHANGE_NODE_ID:
            LogMessage("STATE_CHANGE_NODE_ID: Changing Node-ID");
            change_node_id_and_save(0x02);
            currentState = STATE_ACTIVATION_COMPLETE;
            break;

        case STATE_ACTIVATION_COMPLETE:
            LogMessage("Activation process completed successfully.");
            currentState = STATE_SOFT_RESET;  // Переход в состояние программного сброса
            break;

        case STATE_ACTIVATION_FAILED:
            LogError("Activation failed.");
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

    // Проверка на ответ версии загрузчика
    if (received_ID == (RAW_CAN_RESPONSE_ARBITRATION_CODE | LEFT_WHEEL_ID) &&
        data[0] == 0xCC && data[1] == 0xDD && data[2] == 0x00 && data[3] == 0x01) {
        LogMessage("Bootloader version response received: Version 0x0100.");
        heartbeat_received = 1;  // Используем флаг для перехода к следующему шагу
    }
}



/*
void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC) {
    memcpy(received_data, data, DLC); // Копируем данные из сообщения в глобальный массив
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
*/
void change_node_id_and_save(uint8_t new_id) {
    uint8_t change_id_command[] = {0x2F, 0x01, 0x18, 0x00, new_id, 0x00, 0x00, 0x00};

    if (send_command(change_id_command, sizeof(change_id_command))) {
        LogMessage("Node-ID change command sent successfully.");
        HAL_Delay(100);

        uint8_t save_id_command[] = {0x2B, 0x10, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00};
        if (send_command(save_id_command, sizeof(save_id_command))) {
            LogMessage("Node-ID saved successfully.");
            HAL_Delay(100);

            uint8_t reset_command[] = {0x81, 0x00};
            if (send_command(reset_command, sizeof(reset_command))) {
                LogMessage("Soft reset command sent successfully.");
            } else {
                LogError("Failed to send soft reset command.");
            }
        } else {
            LogError("Failed to save new Node-ID.");
        }
    } else {
        LogError("Failed to send Node-ID change command.");
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
