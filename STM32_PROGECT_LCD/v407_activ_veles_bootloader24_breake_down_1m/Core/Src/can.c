#include "can.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

// Макросы для SDO-запросов
#define CANOPEN_SDO_INDEX_MODE_OF_OPERATION 0x6060
#define CANOPEN_SDO_SUB_INDEX_MODE_OF_OPERATION 0x00
#define CANOPEN_SDO_VALUE_MODE_OF_OPERATION_SPEED 0x03

#define CANOPEN_SDO_INDEX_CONTROL_WORD 0x6040
#define CANOPEN_SDO_SUB_INDEX_CONTROL_WORD 0x00

#define CANOPEN_SDO_VALUE_CONTROL_WORD_CLEAR_EXCEPTION 0x0080
#define CANOPEN_SDO_VALUE_CONTROL_WORD_SERVO_PREPARATION 0x0006
#define CANOPEN_SDO_VALUE_CONTROL_WORD_WAIT_SERVO_ENABLE 0x0007
#define CANOPEN_SDO_VALUE_CONTROL_WORD_SERVO_RUNNING 0x000F
#define CANOPEN_SDO_VALUE_CONTROL_WORD_FAST_DOWN_TIME 0x0002

#define CANOPEN_SDO_INDEX_SPEED_MODE_SPEED 0x60FF
#define CANOPEN_SDO_SUB_INDEX_SPEED_MODE_SPEED 0x00

// Временные задержки
#define WHEEL_ROTATION_DELAY_MS 1000 // Задержка для проверки вращения колеса

// Идентификаторы
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
int32_t calculate_speed_for_distance(float distance_meters);

static void CAN_ConfigFilter(void);
static uint8_t send_command(uint8_t *data, uint8_t len, uint32_t timeout_ms);
void CAN_Init(void);
void LogMessage(const char *message);
void LogError(const char *message);

void release_brake(uint8_t node_id);
void rotate_wheel_one_meter(uint8_t node_id); // Добавлена функция вращения колеса

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
    uint8_t reset_command[] = {0x81, 0x00};

    switch (currentState) {
        case STATE_INIT:
            LogMessage("STATE_INIT: Initializing...");
            currentState = STATE_WAIT_BOOTLOADER_HEARTBEAT;
            stateTimer = HAL_GetTick();
            break;

        case STATE_WAIT_BOOTLOADER_HEARTBEAT:
            if (heartbeat_received) {
                LogMessage("STATE_WAIT_BOOTLOADER_HEARTBEAT: Bootloader heartbeat received.");
                currentState = STATE_GET_BOOTLOADER_VERSION;
                stateTimer = HAL_GetTick();
                heartbeat_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("STATE_WAIT_BOOTLOADER_HEARTBEAT: Failed to receive bootloader heartbeat.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_GET_BOOTLOADER_VERSION:
            if (send_command((uint8_t*) "\x20\x20", 2, CAN_COMMAND_TIMEOUT_MS)) {
                LogMessage("STATE_GET_BOOTLOADER_VERSION: Bootloader version request sent.");
                currentState = STATE_WAIT_BOOTLOADER_VERSION_RESPONSE;
                stateTimer = HAL_GetTick();
            } else {
                LogError("STATE_GET_BOOTLOADER_VERSION: Failed to send bootloader version request.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_BOOTLOADER_VERSION_RESPONSE:
            if (bootloader_version_received) {
                LogMessage("STATE_WAIT_BOOTLOADER_VERSION_RESPONSE: Bootloader version confirmed.");
                currentState = STATE_SEND_ACTIVATION;
                stateTimer = HAL_GetTick();
                bootloader_version_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("STATE_WAIT_BOOTLOADER_VERSION_RESPONSE: Failed to receive bootloader version.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_SEND_ACTIVATION:
            if (send_command(activation_data, 2, CAN_COMMAND_TIMEOUT_MS)) {
                LogMessage("STATE_SEND_ACTIVATION: Activation command sent.");
                currentState = STATE_WAIT_ACTIVATION_RESPONSE;
                stateTimer = HAL_GetTick();
            } else {
                LogError("STATE_SEND_ACTIVATION: Failed to send activation command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_WAIT_ACTIVATION_RESPONSE:
            if (activation_confirmed) {
                LogMessage("STATE_WAIT_ACTIVATION_RESPONSE: Activation confirmed.");
                currentState = STATE_CONFIRM_CANOPEN_MODE;
                stateTimer = HAL_GetTick();
                activation_confirmed = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("STATE_WAIT_ACTIVATION_RESPONSE: Failed to receive activation confirmation.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CONFIRM_CANOPEN_MODE:
            if (canopen_confirmation_received) {
                LogMessage("STATE_CONFIRM_CANOPEN_MODE: CANopen mode confirmed.");
                currentState = STATE_CHECK_WHEEL_SDO;
                stateTimer = HAL_GetTick();
                canopen_confirmation_received = 0;
            } else if (HAL_GetTick() - stateTimer > 5000) {
                LogError("STATE_CONFIRM_CANOPEN_MODE: Failed to confirm CANopen mode.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_CHECK_WHEEL_SDO:
            LogMessage("STATE_CHECK_WHEEL_SDO: Checking wheel SDOs...");
            check_wheel_sdo(LEFT_WHEEL_ID);
            currentState = STATE_RELEASE_BRAKE;
            stateTimer = HAL_GetTick();
            break;

        case STATE_RELEASE_BRAKE:
            LogMessage("STATE_RELEASE_BRAKE: Releasing brake...");
            release_brake(LEFT_WHEEL_ID);
            currentState = STATE_ROTATE_WHEEL_ONE_METER;
            stateTimer = HAL_GetTick();
            break;

        case STATE_ROTATE_WHEEL_ONE_METER:
            LogMessage("STATE_ROTATE_WHEEL_ONE_METER: Rotating wheel one meter...");
            rotate_wheel_one_meter(LEFT_WHEEL_ID);
            currentState = STATE_SOFT_RESET;
            stateTimer = HAL_GetTick();
            break;

        case STATE_SOFT_RESET:
            LogMessage("STATE_SOFT_RESET: Sending soft reset command.");
            if (send_command(reset_command, sizeof(reset_command), CAN_COMMAND_TIMEOUT_MS)) {
                LogMessage("STATE_SOFT_RESET: Soft reset command sent successfully.");
                currentState = STATE_ACTIVATION_SUCCESS;
            } else {
                LogError("STATE_SOFT_RESET: Failed to send soft reset command.");
                currentState = STATE_ACTIVATION_FAILED;
            }
            break;

        case STATE_ACTIVATION_SUCCESS:
            LogMessage("STATE_ACTIVATION_SUCCESS: Activation, SDO check, brake release, and wheel rotation completed successfully.");
            currentState = STATE_INIT;
            break;

        case STATE_ACTIVATION_FAILED:
            LogError("STATE_ACTIVATION_FAILED: Activation, SDO check, brake release, or wheel rotation failed.");
            currentState = STATE_INIT;
            break;

        default:
            LogError("Unknown state.");
            currentState = STATE_INIT;
            break;
    }
}






void process_CAN_message(uint16_t received_ID, uint8_t *data, uint8_t DLC) {
    static uint16_t last_received_ID = 0;
    static uint8_t last_data[8] = {0};

    // Сравнение текущего сообщения с последним
    if (received_ID == last_received_ID && memcmp(data, last_data, 8) == 0) {
        return; // Пропускаем дублирующее сообщение
    }

    // Сохраняем текущее сообщение как последнее
    last_received_ID = received_ID;
    memcpy(last_data, data, 8);

    // Логируем новое сообщение
    char logBuffer[200];
    snprintf(logBuffer, sizeof(logBuffer),
             "Received CAN message: ID=0x%X, DLC=%d, Data=[%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
             received_ID, DLC, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    LogMessage(logBuffer);

    // Обработка сообщений
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




	void check_wheel_sdo(uint8_t node_id) {
	    uint32_t cob_id = 0x600 + node_id; // COB-ID для SDO-запроса
	    uint8_t sdo_request[8];
	    char logBuffer[200];

	    // Логируем начало работы
	    snprintf(logBuffer, sizeof(logBuffer), "Checking SDOs for Node-ID: %d", node_id);
	    LogMessage(logBuffer);

	    // Примерный список индексов SDO для проверки
	    uint16_t sdo_indices[] = {0x6040, 0x6041, 0x6060, 0x607A}; // Индексы для проверки
	    uint8_t sdo_subindices[] = {0x00, 0x00, 0x00, 0x00};       // Подиндексы

	    for (size_t i = 0; i < sizeof(sdo_indices) / sizeof(sdo_indices[0]); i++) {
	        uint16_t index = sdo_indices[i];
	        uint8_t subindex = sdo_subindices[i];

	        // Формируем SDO-запрос на чтение
	        sdo_request[0] = 0x40;                    // Команда чтения
	        sdo_request[1] = index & 0xFF;           // Младший байт индекса
	        sdo_request[2] = (index >> 8) & 0xFF;    // Старший байт индекса
	        sdo_request[3] = subindex;               // Подиндекс
	        memset(&sdo_request[4], 0, 4);           // Остальные байты пустые

	        // Настраиваем передачу через CAN
	        CAN_TxHeaderTypeDef TxHeader;
	        uint32_t TxMailbox;

	        TxHeader.StdId = cob_id;                 // COB-ID
	        TxHeader.RTR = CAN_RTR_DATA;             // Это данные, не запрос
	        TxHeader.IDE = CAN_ID_STD;               // Стандартный формат ID
	        TxHeader.DLC = sizeof(sdo_request);      // Длина сообщения

	        // Логируем отправку
	        snprintf(logBuffer, sizeof(logBuffer),
	                 "Sending SDO request: Index=0x%04X, SubIndex=0x%02X", index, subindex);
	        LogMessage(logBuffer);

	        // Отправка сообщения
	        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, sdo_request, &TxMailbox) != HAL_OK) {
	            LogError("Failed to send SDO request.");
	            continue; // Переходим к следующему SDO
	        }

	        // Ожидаем завершения передачи
	        uint32_t startTick = HAL_GetTick();
	        while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	            if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	                LogError("Timeout while sending SDO request.");
	                break; // Переходим к следующему SDO
	            }
	        }

	        // Ожидание ответа
	        uint8_t response_data[8];
	        CAN_RxHeaderTypeDef RxHeader;
	        uint8_t response_received = 0;

	        startTick = HAL_GetTick();
	        while (HAL_GetTick() - startTick < CAN_COMMAND_TIMEOUT_MS) {
	            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, response_data) == HAL_OK) {
	                // Проверяем, что это ответ на наш запрос
	                if (RxHeader.StdId == (0x580 + node_id) &&
	                    response_data[1] == (index & 0xFF) &&
	                    response_data[2] == ((index >> 8) & 0xFF) &&
	                    response_data[3] == subindex) {
	                    response_received = 1;
	                    break;
	                }
	            }
	        }

	        if (!response_received) {
	            LogError("Timeout waiting for SDO response.");
	            continue; // Переходим к следующему SDO
	        }

	        // Обрабатываем данные ответа
	        uint32_t value = response_data[4] |
	                         (response_data[5] << 8) |
	                         (response_data[6] << 16) |
	                         (response_data[7] << 24);

	        // Логируем результат
	        snprintf(logBuffer, sizeof(logBuffer),
	                 "SDO response: Index=0x%04X, SubIndex=0x%02X, Value=0x%08X",
	                 index, subindex, value);
	        LogMessage(logBuffer);
	    }
	}

	void release_brake(uint8_t node_id) {
		    uint32_t cob_id = 0x600 + node_id;
		    uint8_t brake_command[] = {0x2F, 0x58, 0x20, 0x00, 0x03, 0x00, 0x00, 0x00};

		    CAN_TxHeaderTypeDef TxHeader;
		    uint32_t TxMailbox;

		    TxHeader.StdId = cob_id;              // Устанавливаем COB-ID
		    TxHeader.RTR = CAN_RTR_DATA;          // Это данные, не запрос
		    TxHeader.IDE = CAN_ID_STD;            // Стандартный формат ID
		    TxHeader.DLC = sizeof(brake_command); // Длина данных

		    // Логирование отправки команды
		    char logBuffer[100];
		    snprintf(logBuffer, sizeof(logBuffer),
		             "Sending brake release command to Node-ID: %d, COB-ID: 0x%03lX", node_id, (unsigned long)cob_id);
		    LogMessage(logBuffer);

		    // Отправка команды
		    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, brake_command, &TxMailbox) != HAL_OK) {
		        LogError("release_brake: Failed to send brake release command.");
		        return;
		    }

		    // Ожидание завершения отправки
		    uint32_t startTick = HAL_GetTick();
		    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
		        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
		            LogError("release_brake: Timeout waiting for brake release command to be sent.");
		            return;
		        }
		    }

		    LogMessage("release_brake: Brake release command sent successfully.");
		}
	void rotate_wheel_one_meter(uint8_t node_id) {
	    uint32_t cob_id = 0x600 + node_id;  // COB-ID для SDO-запросов
	    CAN_TxHeaderTypeDef TxHeader;
	    uint32_t TxMailbox;
	    uint8_t command[8];
	    char logBuffer[200];

	    // 1. Установить режим скорости (Speed Mode)
	    command[0] = 0x2F;  // Write 1 byte
	    command[1] = 0x60;  // Index 0x6060 (Mode of Operation)
	    command[2] = 0x60;
	    command[3] = 0x00;  // Sub-index 0x00
	    command[4] = 0x03;  // Speed Mode
	    command[5] = 0x00;
	    command[6] = 0x00;
	    command[7] = 0x00;

	    snprintf(logBuffer, sizeof(logBuffer), "Setting Speed Mode for Node-ID: %d", node_id);
	    LogMessage(logBuffer);

	    TxHeader.StdId = cob_id;
	    TxHeader.RTR = CAN_RTR_DATA;
	    TxHeader.IDE = CAN_ID_STD;
	    TxHeader.DLC = sizeof(command);

	    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, command, &TxMailbox) != HAL_OK) {
	        LogError("rotate_wheel_one_meter: Failed to send Speed Mode command.");
	        return;
	    }

	    uint32_t startTick = HAL_GetTick();
	    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	            LogError("rotate_wheel_one_meter: Timeout waiting for Speed Mode command to be sent.");
	            return;
	        }
	    }
	    LogMessage("Speed Mode set successfully.");

	    // 2. Подготовка сервопривода (очистка исключений, включение привода)
	    uint16_t control_words[] = {0x0080, 0x0006, 0x0007, 0x000F};
	    const char* control_word_steps[] = {
	        "Clear exceptions",
	        "Servo preparation",
	        "Servo waiting to enable",
	        "Servo enable"};

	    for (int i = 0; i < 4; i++) {
	        command[0] = 0x2B;  // Write 2 bytes
	        command[1] = 0x40;  // Index 0x6040 (Control Word)
	        command[2] = 0x60;
	        command[3] = 0x00;  // Sub-index 0x00
	        command[4] = (uint8_t)(control_words[i] & 0xFF);
	        command[5] = (uint8_t)((control_words[i] >> 8) & 0xFF);
	        command[6] = 0x00;
	        command[7] = 0x00;

	        snprintf(logBuffer, sizeof(logBuffer), "Sending %s for Node-ID: %d", control_word_steps[i], node_id);
	        LogMessage(logBuffer);

	        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, command, &TxMailbox) != HAL_OK) {
	            LogError("rotate_wheel_one_meter: Failed to send control word command.");
	            return;
	        }

	        startTick = HAL_GetTick();
	        while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	            if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	                LogError("rotate_wheel_one_meter: Timeout waiting for control word command to be sent.");
	                return;
	            }
	        }

	        LogMessage("Control word command sent successfully.");
	    }

	    // 3. Установить время разгона и торможения
	    uint32_t timing_indices[] = {0x201C, 0x201D};  // Acceleration, Deceleration
	    const char* timing_steps[] = {"Acceleration time", "Deceleration time"};
	    uint32_t timing_values[] = {500, 500};  // 500 мс

	    for (int i = 0; i < 2; i++) {
	        command[0] = 0x23;  // Write 4 bytes
	        command[1] = (uint8_t)(timing_indices[i] & 0xFF);
	        command[2] = (uint8_t)((timing_indices[i] >> 8) & 0xFF);
	        command[3] = 0x00;  // Sub-index 0x00
	        command[4] = (uint8_t)(timing_values[i] & 0xFF);
	        command[5] = (uint8_t)((timing_values[i] >> 8) & 0xFF);
	        command[6] = (uint8_t)((timing_values[i] >> 16) & 0xFF);
	        command[7] = (uint8_t)((timing_values[i] >> 24) & 0xFF);

	        snprintf(logBuffer, sizeof(logBuffer), "Setting %s for Node-ID: %d", timing_steps[i], node_id);
	        LogMessage(logBuffer);

	        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, command, &TxMailbox) != HAL_OK) {
	            LogError("rotate_wheel_one_meter: Failed to send timing command.");
	            return;
	        }

	        startTick = HAL_GetTick();
	        while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	            if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	                LogError("rotate_wheel_one_meter: Timeout waiting for timing command to be sent.");
	                return;
	            }
	        }

	        LogMessage("Timing command sent successfully.");
	    }

	    // 4. Установить целевую скорость
	    int32_t target_speed = (int32_t)(calculate_speed_for_distance(1.0) * 1.5);  // Увеличение скорости на 50%
	    command[0] = 0x23;  // Write 4 bytes
	    command[1] = 0xFF;  // Index 0x60FF (Target Speed)
	    command[2] = 0x60;
	    command[3] = 0x00;  // Sub-index 0x00
	    command[4] = (uint8_t)(target_speed & 0xFF);
	    command[5] = (uint8_t)((target_speed >> 8) & 0xFF);
	    command[6] = (uint8_t)((target_speed >> 16) & 0xFF);
	    command[7] = (uint8_t)((target_speed >> 24) & 0xFF);

	    snprintf(logBuffer, sizeof(logBuffer), "Setting target speed for Node-ID: %d, Speed: %d", node_id, target_speed);
	    LogMessage(logBuffer);

	    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, command, &TxMailbox) != HAL_OK) {
	        LogError("rotate_wheel_one_meter: Failed to send target speed command.");
	        return;
	    }

	    startTick = HAL_GetTick();
	    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	            LogError("rotate_wheel_one_meter: Timeout waiting for target speed command to be sent.");
	            return;
	        }
	    }

	    LogMessage("Target speed command sent successfully.");

	    // 5. Подождать завершения вращения
	    HAL_Delay(19000);  // Примерная задержка для вращения на 1 метр

	    // 6. Остановка вращения
	    command[4] = 0x00;  // Установить скорость 0
	    command[5] = 0x00;
	    command[6] = 0x00;
	    command[7] = 0x00;

	    snprintf(logBuffer, sizeof(logBuffer), "Stopping wheel for Node-ID: %d", node_id);
	    LogMessage(logBuffer);

	    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, command, &TxMailbox) != HAL_OK) {
	        LogError("rotate_wheel_one_meter: Failed to send stop command.");
	        return;
	    }

	    startTick = HAL_GetTick();
	    while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
	        if (HAL_GetTick() - startTick > CAN_COMMAND_TIMEOUT_MS) {
	            LogError("rotate_wheel_one_meter: Timeout waiting for stop command to be sent.");
	            return;
	        }
	    }

	    LogMessage("Wheel stopped successfully.");
	}




	int32_t calculate_speed_for_distance(float distance_meters) {
	    const float conversion_factor = 9000.0; // Коэффициент пересчёта.
	    return (int32_t)(distance_meters * conversion_factor);
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
