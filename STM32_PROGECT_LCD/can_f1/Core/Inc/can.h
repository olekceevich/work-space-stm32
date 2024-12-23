#ifndef CAN_H
#define CAN_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

// Инициализация CAN
void CAN_Init(void);
void StartActivationProcess(void);
void CAN_ProcessStateMachine(void);
void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC);
void send_move_command(void);
void send_nmt_command(uint8_t command, uint8_t node_id);
typedef enum {
    STATE_INIT,
    STATE_WAIT_BOOTLOADER_HEARTBEAT,
    STATE_SEND_ACTIVATION,
    STATE_WAIT_FIRMWARE_CONFIRMATION,
    STATE_GET_BOOTLOADER_VERSION,
    STATE_CONFIRM_CANOPEN_MODE,
    STATE_CHANGE_NODE_ID,
    STATE_ACTIVATION_FAILED,
    STATE_ACTIVATION_COMPLETE,
    STATE_SOFT_RESET  // Новое состояние для выполнения сброса
} ActivationState;

// Функции логирования
void LogMessage(const char *message);
void LogError(const char *message);

#endif // CAN_H
