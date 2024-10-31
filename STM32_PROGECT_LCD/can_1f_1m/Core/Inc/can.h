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

// Состояния активации
typedef enum {
    STATE_INIT,
    STATE_WAIT_BOOTLOADER_HEARTBEAT,
    STATE_GET_BOOTLOADER_VERSION,
    STATE_SEND_ACTIVATION,
    STATE_CONFIRM_CANOPEN_MODE,
    STATE_ROTATE_WHEEL,      // Добавлено состояние для вращения колеса
    STATE_ACTIVATION_FAILED,
    STATE_ACTIVATION_COMPLETE
} ActivationState;

// Объявление глобальных переменных
extern ActivationState currentState;
extern uint32_t stateTimer;
extern volatile uint8_t heartbeat_received;
extern volatile uint8_t canopen_confirmation_received;

// Функции логирования
void LogMessage(const char *message);
void LogError(const char *message);

#endif // CAN_H
