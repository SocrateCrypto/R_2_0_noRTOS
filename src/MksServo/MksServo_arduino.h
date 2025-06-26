#ifndef MKS_SERVO_ARDUINO_H
#define MKS_SERVO_ARDUINO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define MKS_SERVO_RX_BUF_SIZE 128

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dere_port;
    uint16_t dere_pin;
    uint8_t device_address;
    // RX ring buffer
    volatile uint8_t rx_buf[MKS_SERVO_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_byte; // for interrupt
} MksServo_t;

void MksServo_Init(MksServo_t *servo, UART_HandleTypeDef *huart, GPIO_TypeDef *dere_port, uint16_t dere_pin, uint8_t address);
void MksServo_UART_IRQHandler(MksServo_t *servo, UART_HandleTypeDef *huart);

// Основная команда: управление скоростью (аналог speedModeRun)
uint8_t MksServo_SpeedModeRun(MksServo_t *servo, uint8_t dir, uint16_t speed, uint8_t acc);

// Ожидание ответа (аналог waitingForACK)
uint8_t MksServo_WaitForACK(MksServo_t *servo, uint8_t len, uint32_t timeout_ms);

// Вспомогательные
uint8_t MksServo_GetCheckSum(uint8_t *buffer, uint8_t len);
int MksServo_RxGetByte(MksServo_t *servo, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // MKS_SERVO_ARDUINO_H
