#include "can.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>
//4/23 2b
#define LEFT_WHEEL_ID 3
#define RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE 0x700
#define RAW_CAN_RESPONSE_ARBITRATION_CODE 0x580
const uint8_t RAW_CAN_BOOTLOADER_HEARTBEAT_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t CANOPEN_CONFIRMATION_DATA[1] = {0x7F};
#define CAN_COMMAND_TIMEOUT_MS 6000

// Глобальные флаги для подтверждения ответов
volatile uint8_t heartbeat_received = 0;
volatile uint8_t canopen_confirmation_received = 0;
volatile uint8_t bootloader_version_received = 0;
volatile uint8_t activation_confirmed = 0;

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

ActivationState currentState = STATE_INIT;
uint32_t stateTimer = 0;

static void CAN_ConfigFilter(void);
static uint8_t send_command(uint8_t *data, uint8_t len, uint32_t timeout_ms);
void CAN_Init(void);
void LogMessage(const char *message);
void LogError(const char *message);
void change_node_id_and_save(uint8_t current_id, uint8_t new_id);

void CAN_Init(void) {
    CAN_ConfigFilter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        LogError("CAN_Init: Error starting CAN");
    } else {
        LogMessage("CAN_Init: CAN started successfully");
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
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

    switch (currentState) {
        case STATE_WAIT_BOOTLOADER_HEARTBEAT:
            if (heartbeat_received) {
                LogMessage("STATE_WAIT_BOOTLOADER_HEARTBEAT: Bootloader heartbeat received.");
                currentState = STATE_GET_BOOTLOADER_VERSION;
                stateTimer = HAL_GetTick();
                heartbeat_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to receive bootloader heartbeat.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_GET_BOOTLOADER_VERSION:
            if (send_command((uint8_t*) "\x20\x20", 2, 6000)) {
                LogMessage("STATE_GET_BOOTLOADER_VERSION: Bootloader version request sent.");
                currentState = STATE_WAIT_BOOTLOADER_VERSION_RESPONSE;
                stateTimer = HAL_GetTick();
            } else {
                LogError("Failed to send bootloader version request.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_BOOTLOADER_VERSION_RESPONSE:
            if (bootloader_version_received) {
                LogMessage("Bootloader version confirmed.");
                currentState = STATE_SEND_ACTIVATION;
                stateTimer = HAL_GetTick();
                bootloader_version_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to receive bootloader version confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_SEND_ACTIVATION:
            if (send_command(activation_data, 2, 6000)) {
                LogMessage("STATE_SEND_ACTIVATION: Activation command sent.");
                currentState = STATE_WAIT_ACTIVATION_RESPONSE;
                stateTimer = HAL_GetTick();
            } else {
                LogError("Failed to send activation command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_ACTIVATION_RESPONSE:
            if (activation_confirmed) {
                LogMessage("Activation confirmed.");
                currentState = STATE_CONFIRM_CANOPEN_MODE;
                stateTimer = HAL_GetTick();
                activation_confirmed = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to receive activation confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CONFIRM_CANOPEN_MODE:
            if (canopen_confirmation_received) {
                LogMessage("STATE_CONFIRM_CANOPEN_MODE: CANopen mode confirmed.");
                currentState = STATE_READY_FOR_NODE_ID_CHANGE;
                stateTimer = HAL_GetTick();
                canopen_confirmation_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("Failed to confirm CANopen mode.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_READY_FOR_NODE_ID_CHANGE:
            if (HAL_GetTick() - stateTimer > 1000) {
                LogMessage("STATE_READY_FOR_NODE_ID_CHANGE: Ready to change Node-ID.");
                currentState = STATE_CHANGE_NODE_ID;
            }
            break;

        case STATE_CHANGE_NODE_ID:
            LogMessage("STATE_CHANGE_NODE_ID: Changing Node-ID.");
            change_node_id(LEFT_WHEEL_ID, 0x01);
            currentState = STATE_SOFT_RESET;
            stateTimer = HAL_GetTick();
            break;

        case STATE_SOFT_RESET:
            LogMessage("STATE_SOFT_RESET: Sending soft reset command.");
            uint8_t reset_command[] = {0x81, 0x00};
            if (send_command(reset_command, sizeof(reset_command), 6000)) {
                LogMessage("Soft reset command sent successfully.");
                currentState = STATE_ACTIVATION_SUCCESS;
            } else {
                LogError("Failed to send soft reset command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_ACTIVATION_SUCCESS:
            LogMessage("Activation and ID change completed successfully.");
            currentState = STATE_INIT;
            break;

        case STATE_ACTIVATION_FAILED:
            LogError("Activation or ID change failed.");
            currentState = STATE_INIT;
            break;

        default:
            break;
    }
}




/*
void change_node_id(uint8_t current_id, uint8_t new_id) {
    // Расчет адреса (COB-ID) для отправки команды
    uint32_t cob_id = 0x600 + current_id;
    uint8_t change_id_command[] = {0x2F, 0x26, 0x20, 0x00, new_id, 0x00, 0x00, 0x00};

    // Логирование отправки команды
    char logBuffer[100];
    snprintf(logBuffer, sizeof(logBuffer),
             "Sending Node-ID change command to COB-ID: 0x%03X with new ID: %d", cob_id, new_id);
    LogMessage(logBuffer);

    // Настройка заголовка CAN сообщения
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = cob_id;             // Устанавливаем ID для отправки
    TxHeader.RTR = CAN_RTR_DATA;         // Данные (не запрос)
    TxHeader.IDE = CAN_ID_STD;           // Стандартный формат ID
    TxHeader.DLC = sizeof(change_id_command);  // Длина данных

    // Отправка команды
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, change_id_command, &TxMailbox) != HAL_OK) {
        LogError("Failed to send Node-ID change command.");
        return;
    }

    // Ожидание завершения отправки
    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
        if (HAL_GetTick() - startTick > 500) { // Тайм-аут 6 секунд
            LogError("Timeout waiting for Node-ID change command to be sent.");
            return;
        }
    }

    // Лог успешной отправки
    LogMessage("Node-ID change command sent successfully.");

    // Ожидание применения нового ID (можно настроить при необходимости)
    HAL_Delay(100);  // Короткая задержка, чтобы устройство обработало команду
}
*/

void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC) {
    char logBuffer[200];
    snprintf(logBuffer, sizeof(logBuffer),
             "Received CAN message: ID=0x%X, DLC=%d, Data=[%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
             received_ID, DLC, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    LogMessage(logBuffer);

    if (received_ID == (LEFT_WHEEL_ID | RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE) &&
        memcmp(data, RAW_CAN_BOOTLOADER_HEARTBEAT_DATA, 8) == 0) {
        heartbeat_received = 1;
    } else if (received_ID == (LEFT_WHEEL_ID | RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE) &&
               data[0] == CANOPEN_CONFIRMATION_DATA[0]) {
        canopen_confirmation_received = 1;
    } else if (memcmp(data, (uint8_t[]){0xAA, 0xBB}, 2) == 0) {
        activation_confirmed = 1;
    } else if (memcmp(data, (uint8_t[]){0xCC, 0xDD}, 2) == 0) {
        bootloader_version_received = 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        process_CAN_message(RxHeader.StdId, RxData, RxHeader.DLC);
    }
}

static uint8_t send_command(uint8_t *data, uint8_t len, uint32_t timeout_ms) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.StdId = LEFT_WHEEL_ID;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
        LogError("Error sending command to CAN.");
        return 0;
    }

    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
        if (HAL_GetTick() - startTick > timeout_ms) {
            LogError("Timeout waiting for message to be sent.");
            return 0;
        }
    }
    LogMessage("Command sent successfully from TxMailbox");
    return 1;
}


void change_node_id(uint8_t current_id, uint8_t new_id) {
    if (current_id > 0x7F || new_id > 0x7F) {
        LogError("change_node_id: Invalid Node ID. Must be in range 0x01 to 0x7F.");
        return;
    }

    uint32_t cob_id = 0x600 + current_id;
    uint8_t change_id_command[] = {0x2F, 0x26, 0x20, 0x00, new_id, 0x00, 0x00, 0x00};

    // Логирование команды изменения Node-ID
    char logBuffer[200];
    snprintf(logBuffer, sizeof(logBuffer),
             "Sending Node-ID change command:\nCOB-ID: 0x%03lX\nData: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
             (unsigned long)cob_id,
             change_id_command[0], change_id_command[1], change_id_command[2], change_id_command[3],
             change_id_command[4], change_id_command[5], change_id_command[6], change_id_command[7]);
    LogMessage(logBuffer);

    // Настройка заголовка CAN сообщения
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = cob_id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = sizeof(change_id_command);

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, change_id_command, &TxMailbox) != HAL_OK) {
        LogError("change_node_id: Failed to send Node-ID change command.");
        return;
    }

    // Ожидание завершения отправки
    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
            LogError("change_node_id: Timeout waiting for Node-ID change command to be sent.");
            return;
        }
    }

    LogMessage("change_node_id: Node-ID change command sent successfully.");
    HAL_Delay(100);

    // Сохранение изменений
    uint8_t save_command[] = {0x23, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65};  // 'save' команда
    TxHeader.DLC = sizeof(save_command);

    snprintf(logBuffer, sizeof(logBuffer),
             "Sending Save Command:\nCOB-ID: 0x%03lX\nData: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
             (unsigned long)cob_id,
             save_command[0], save_command[1], save_command[2], save_command[3],
             save_command[4], save_command[5], save_command[6], save_command[7]);
    LogMessage(logBuffer);

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, save_command, &TxMailbox) != HAL_OK) {
        LogError("change_node_id: Failed to send save command.");
        return;
    }

    startTick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
            LogError("change_node_id: Timeout waiting for save command to be sent.");
            return;
        }
    }

    LogMessage("change_node_id: Changes saved successfully.");
    HAL_Delay(100);
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

    if (HAL_CAN_ConfigFilter(&hcan1, &FilterConfig) != HAL_OK) {
        LogError("CAN_ConfigFilter: Filter configuration failed");
    } else {
        LogMessage("CAN_ConfigFilter: Filter configured successfully");
    }
}

void LogMessage(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

void LogError(const char *message) {
    LogMessage(message);
}
