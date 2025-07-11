#ifndef MKS_SERVO_ALLINONE_H
#define MKS_SERVO_ALLINONE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h> // Для abs()
#include "../../include/fd_status.h" // Подключаем fd_status.h

#define MKS_SERVO_RX_BUF_SIZE 128
#define MKS_SERVO_STEPS_PER_REV 122880 // Количество шагов на оборот (уточните при необходимости)

// Режимы работы сервопривода (для MksServo_SetWorkMode)
#define MKS_SERVO_MODE_CR_OPEN  0  // Pulse interface, 400 RPM, fixed current
#define MKS_SERVO_MODE_CR_CLOSE 1  // Pulse interface, 1500 RPM, fixed current  
#define MKS_SERVO_MODE_CR_vFOC  2  // Pulse interface, 3000 RPM, self-adaption current
#define MKS_SERVO_MODE_SR_OPEN  3  // Serial interface, 400 RPM, fixed current
#define MKS_SERVO_MODE_SR_CLOSE 4  // Serial interface, 1500 RPM, fixed current
#define MKS_SERVO_MODE_SR_vFOC  5  // Serial interface, 3000 RPM, self-adaption current

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MKS_SERVO_STATE_UNKNOWN = 0,
    MKS_SERVO_STATE_STOPPED,
    MKS_SERVO_STATE_MOVING,
    MKS_SERVO_STATE_ERROR
} MksServoState_t;

// Структура для хранения состояния мотора
typedef struct {
    MksServoState_t state;
    uint8_t last_status_code;
    uint32_t last_update_tick;
    uint8_t locked_rotor;
    uint16_t encoder_position;
    int32_t position;         // Абсолютная позиция в шагах
    uint8_t is_stopped;       // Флаг остановки (1 - стоп, 0 - движется)
} MksServoStatus_t;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dere_port;
    uint16_t dere_pin;
    uint8_t device_address;
    volatile uint8_t rx_buf[MKS_SERVO_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_byte;
    MksServoStatus_t status;
} MksServo_t;

// --- API ---
void    MksServo_Init(MksServo_t *servo, UART_HandleTypeDef *huart, GPIO_TypeDef *dere_port, uint16_t dere_pin, uint8_t address);
void    MksServo_UART_IRQHandler(MksServo_t *servo, UART_HandleTypeDef *huart);
uint8_t MksServo_SpeedModeRun(MksServo_t *servo, uint8_t dir, uint16_t speed, uint8_t acc);
uint8_t MksServo_WaitForACK(MksServo_t *servo, uint8_t len, uint32_t timeout_ms);
uint8_t MksServo_GetCheckSum(uint8_t *buffer, uint8_t len);
int     MksServo_RxGetByte(MksServo_t *servo, uint8_t *data);
void    MksServo_UpdateStateFromACK(MksServo_t *servo, const uint8_t *ack, uint8_t len);
uint8_t MksServo_SetMicrostep(MksServo_t *servo, uint8_t microstep);
uint8_t MksServo_SetWorkMode(MksServo_t *servo, uint8_t mode);
uint8_t MksServo_ReadStatus(MksServo_t *servo, uint32_t timeout_ms);
uint8_t MksServo_ReadPositionError(MksServo_t *servo, int32_t *error, uint32_t timeout_ms);
uint8_t MksServo_Calibrate(MksServo_t *servo, uint32_t timeout_ms);
uint8_t MksServo_MoveSteps(MksServo_t *servo, uint8_t dir, uint32_t steps, uint8_t acc);
uint8_t MksServo_MoveDegrees(MksServo_t *servo, float degrees, uint8_t acc);
void    MksServo_SendRaw(MksServo_t *servo, const uint8_t *data, uint16_t len);
// --- DEBUG ---
uint8_t MksServo_ReadResponse(MksServo_t *servo, uint8_t *rx, uint8_t rx_len, uint32_t timeout_ms);
// --- POSITION ---
uint8_t MksServo_PositionMode1Run(MksServo_t *servo, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses);
uint8_t MksServo_PositionMode2Run(MksServo_t *servo, int32_t rel_steps, uint16_t speed, uint8_t acc);
void PollPositionErrorTask(void);
// --- EMERGENCY STOP ---
uint8_t MksServo_EmergencyStop(MksServo_t *servo);
void MksServo_EmergencyStop_Simple(MksServo_t *servo);
// --- ENCODER READOUT ---
uint8_t MksServo_GetCarry(MksServo_t *servo, int32_t *carry, uint16_t *value, uint32_t timeout_ms);
uint8_t MksServo_GetAdditionValue(MksServo_t *servo, int64_t *addition_value, uint32_t timeout_ms);
uint8_t MksServo_AbsoluteMotionByAxis_F5(MksServo_t *servo, int64_t *addition_value, uint32_t timeout_ms);
uint8_t MksServo_AbsoluteMotionByPulse_FE(MksServo_t *servo, uint16_t speed, uint8_t acc, int32_t rel_steps );
uint8_t MksServo_CurrentAxisToZero_92(MksServo_t *servo);
extern uint8_t last_f6_status;
void reset_last_f6_status(void);
extern uint8_t last_f5_status;
void reset_last_f5_status(void);
uint8_t MksServo_QueryStatus_F1(MksServo_t *servo, uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif

#endif // MKS_SERVO_ALLINONE_H

// Заменяем вызовы на корректную функцию
#define MksServo_PositionModeRun(servo, pos, speed, acc) \
    MksServo_PositionMode1Run((servo), ((pos) >= (servo)->status.position ? 1 : 0), (speed), (acc), (uint32_t)abs((pos) - (servo)->status.position))
