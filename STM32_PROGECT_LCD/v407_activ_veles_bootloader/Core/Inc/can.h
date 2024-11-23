#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define CAN_COMMAND_TIMEOUT_MS 6000  // Тайм-аут для CAN-команд в миллисекундах
extern TIM_HandleTypeDef htim2;

void CAN_Init(void);
void StartActivationProcess(void);
void CAN_ProcessStateMachine(void);
void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC);
void change_node_id(uint8_t current_id, uint8_t new_id);

typedef enum {
    STATE_INIT,                        // Начальное состояние
    STATE_WAIT_BOOTLOADER_HEARTBEAT,   // Ожидание heartbeat от загрузчика
    STATE_GET_BOOTLOADER_VERSION,      // Запрос версии загрузчика
    STATE_WAIT_BOOTLOADER_VERSION_RESPONSE,  // Ожидание ответа на запрос версии
    STATE_SEND_ACTIVATION,             // Отправка команды активации
    STATE_WAIT_ACTIVATION_RESPONSE,    // Ожидание ответа на активацию
    STATE_CONFIRM_CANOPEN_MODE,        // Подтверждение CANopen режима
    STATE_READY_FOR_NODE_ID_CHANGE,    // Готовность к изменению Node-ID
    STATE_CHANGE_NODE_ID,              // Изменение Node-ID
    STATE_SOFT_RESET,                  // Выполнение программного сброса
    STATE_ACTIVATION_SUCCESS,          // Успешная активация
    STATE_ACTIVATION_FAILED,           // Сбой активации
    STATE_IDLE                         // Ожидание (неактивное состояние)
} ActivationState;
//void delay_us(uint32_t us);
//void delay_ms(uint32_t ms);
//uint8_t is_timeout(uint32_t start_time, uint32_t timeout_us);

void LogMessage(const char *message);
void LogError(const char *message);
#ifdef __cplusplus
}
#endif
#endif // CAN_H



/*
 * замена на таймер
 * #include "can.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

#define LEFT_WHEEL_ID 2
#define RAW_CAN_BOOTLOADER_HEARTBEAT_ARBITRATION_CODE 0x700
#define RAW_CAN_RESPONSE_ARBITRATION_CODE 0x580
#define CAN_COMMAND_TIMEOUT_MS 6000

const uint8_t RAW_CAN_BOOTLOADER_HEARTBEAT_DATA[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t CANOPEN_CONFIRMATION_DATA[1] = {0x7F};

// Глобальные флаги для подтверждения ответов
volatile uint8_t heartbeat_received = 0;
volatile uint8_t canopen_confirmation_received = 0;
volatile uint8_t bootloader_version_received = 0;
volatile uint8_t activation_confirmed = 0;

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

ActivationState currentState = STATE_INIT;

// Вспомогательные функции для таймера
void delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Сброс счётчика таймера
    while (__HAL_TIM_GET_COUNTER(&htim2) < us) {
        // Ждём заданное количество микросекунд
    }
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);  // Задержка на 1 мс
    }
}

uint8_t is_timeout(uint32_t start_time, uint32_t timeout_us) {
    uint32_t elapsed = __HAL_TIM_GET_COUNTER(&htim2) - start_time;
    return (elapsed >= timeout_us);
}

// Инициализация CAN
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

// Запуск процесса активации
void StartActivationProcess(void) {
    currentState = STATE_WAIT_BOOTLOADER_HEARTBEAT;
    LogMessage("Starting activation process...");
}

// Обработка машины состояний
void CAN_ProcessStateMachine(void) {
    uint8_t activation_data[2] = {0x10, 0x10};
    static uint32_t stateStartTime = 0;

    switch (currentState) {
        case STATE_WAIT_BOOTLOADER_HEARTBEAT:
            if (stateStartTime == 0) {
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
            }

            if (heartbeat_received) {
                LogMessage("STATE_WAIT_BOOTLOADER_HEARTBEAT: Bootloader heartbeat received.");
                currentState = STATE_GET_BOOTLOADER_VERSION;
                stateStartTime = 0;  // Сброс времени
                heartbeat_received = 0;
            } else if (is_timeout(stateStartTime, 5000 * 1000)) {  // 5000 мс = 5000000 мкс
                LogError("Failed to receive bootloader heartbeat.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_GET_BOOTLOADER_VERSION:
            if (send_command((uint8_t*) "\x20\x20", 2, CAN_COMMAND_TIMEOUT_MS)) {
                LogMessage("STATE_GET_BOOTLOADER_VERSION: Bootloader version request sent.");
                currentState = STATE_WAIT_BOOTLOADER_VERSION_RESPONSE;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
            } else {
                LogError("Failed to send bootloader version request.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_BOOTLOADER_VERSION_RESPONSE:
            if (bootloader_version_received) {
                LogMessage("Bootloader version confirmed.");
                currentState = STATE_SEND_ACTIVATION;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
                bootloader_version_received = 0;
            } else if (is_timeout(stateStartTime, 5000 * 1000)) {
                LogError("Failed to receive bootloader version confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_SEND_ACTIVATION:
            if (send_command(activation_data, 2, CAN_COMMAND_TIMEOUT_MS)) {
                LogMessage("STATE_SEND_ACTIVATION: Activation command sent.");
                currentState = STATE_WAIT_ACTIVATION_RESPONSE;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
            } else {
                LogError("Failed to send activation command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_ACTIVATION_RESPONSE:
            if (activation_confirmed) {
                LogMessage("Activation confirmed.");
                currentState = STATE_CONFIRM_CANOPEN_MODE;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
                activation_confirmed = 0;
            } else if (is_timeout(stateStartTime, 5000 * 1000)) {
                LogError("Failed to receive activation confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CONFIRM_CANOPEN_MODE:
            if (canopen_confirmation_received) {
                LogMessage("STATE_CONFIRM_CANOPEN_MODE: CANopen mode confirmed.");
                currentState = STATE_READY_FOR_NODE_ID_CHANGE;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
                canopen_confirmation_received = 0;
            } else if (is_timeout(stateStartTime, 5000 * 1000)) {
                LogError("Failed to confirm CANopen mode.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_READY_FOR_NODE_ID_CHANGE:
            if (is_timeout(stateStartTime, 1000 * 1000)) {  // 1000 мс = 1000000 мкс
                LogMessage("STATE_READY_FOR_NODE_ID_CHANGE: Ready to change Node-ID.");
                currentState = STATE_CHANGE_NODE_ID;
                stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
            }
            break;

        case STATE_CHANGE_NODE_ID:
            LogMessage("STATE_CHANGE_NODE_ID: Changing Node-ID.");
            change_node_id(LEFT_WHEEL_ID, 0x03);
            currentState = STATE_SOFT_RESET;
            stateStartTime = __HAL_TIM_GET_COUNTER(&htim2);
            break;

        case STATE_SOFT_RESET:
            LogMessage("STATE_SOFT_RESET: Sending soft reset command.");
            uint8_t reset_command[] = {0x81, 0x00};
            if (send_command(reset_command, sizeof(reset_command), CAN_COMMAND_TIMEOUT_MS)) {
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

// Остальные функции остаются без изменений
 *
 *
 *
 *
 * */
