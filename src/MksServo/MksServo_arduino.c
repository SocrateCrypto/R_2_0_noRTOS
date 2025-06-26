#include "MksServo_arduino.h"
#include <string.h>

void MksServo_Init(MksServo_t *servo, UART_HandleTypeDef *huart, GPIO_TypeDef *dere_port, uint16_t dere_pin, uint8_t address)
{
    servo->huart = huart;
    servo->dere_port = dere_port;
    servo->dere_pin = dere_pin;
    servo->device_address = address;
    servo->rx_head = 0;
    servo->rx_tail = 0;
    // Запуск приема первого байта по прерыванию
    HAL_UART_Receive_IT(servo->huart, (uint8_t*)&servo->rx_byte, 1);
}

// Вызывать из HAL_UART_RxCpltCallback
void MksServo_UART_IRQHandler(MksServo_t *servo, UART_HandleTypeDef *huart)
{
    if (huart == servo->huart) {
        uint16_t next_head = (servo->rx_head + 1) % MKS_SERVO_RX_BUF_SIZE;
        if (next_head != servo->rx_tail) {
            servo->rx_buf[servo->rx_head] = servo->rx_byte;
            servo->rx_head = next_head;
        }
        // Перезапуск приема
        HAL_UART_Receive_IT(servo->huart, (uint8_t*)&servo->rx_byte, 1);
    }
}

// Получить байт из буфера (0 - нет данных, 1 - есть данные)
int MksServo_RxGetByte(MksServo_t *servo, uint8_t *data)
{
    if (servo->rx_head == servo->rx_tail) return 0;
    *data = servo->rx_buf[servo->rx_tail];
    servo->rx_tail = (servo->rx_tail + 1) % MKS_SERVO_RX_BUF_SIZE;
    return 1;
}

uint8_t MksServo_GetCheckSum(uint8_t *buffer, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += buffer[i];
    return (uint8_t)(sum & 0xFF);
}

// Отправка команды скорости (аналог speedModeRun)
uint8_t MksServo_SpeedModeRun(MksServo_t *servo, uint8_t dir, uint16_t speed, uint8_t acc)
{
    uint8_t tx[7];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF6;
    tx[3] = (dir << 7) | ((speed >> 8) & 0x0F);
    tx[4] = speed & 0xFF;
    tx[5] = acc;
    tx[6] = MksServo_GetCheckSum(tx, 6);

    // Включить передачу
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1); // 1 мс для стабилизации
    HAL_UART_Transmit(servo->huart, tx, 7, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}

// Ожидание ответа (аналог waitingForACK)
uint8_t MksServo_WaitForACK(MksServo_t *servo, uint8_t len, uint32_t timeout_ms)
{
    uint8_t rx[32];
    uint32_t start = HAL_GetTick();
    uint8_t rx_cnt = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            if (rx_cnt != 0) {
                rx[rx_cnt++] = b;
            } else if (b == 0xFB) {
                rx[rx_cnt++] = b;
            }
            if (rx_cnt == len) {
                if (rx[len-1] == MksServo_GetCheckSum(rx, len-1)) {
                    return rx[3]; // статус
                } else {
                    rx_cnt = 0; // ошибка CRC, ждем следующий пакет
                }
            }
        }
    }
    return 0; // timeout
}
