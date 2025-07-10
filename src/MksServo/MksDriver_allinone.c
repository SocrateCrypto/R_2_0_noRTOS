// Объединённая реализация всех функций управления MKS Servo
#include "MksDriver_allinone.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "fd_status.h"
#include <inttypes.h>
#include <stdint.h>

extern MksServo_t mksServo;
extern uint8_t last_f5_status;
extern void reset_last_f5_status(void);
extern uint8_t last_f6_status;
extern void reset_last_f6_status(void);

// --- Основные функции из MksDriver.c ---
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
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 7, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}

uint8_t MksServo_SetMicrostep(MksServo_t *servo, uint8_t microstep)
{
    uint8_t tx[5];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF8;
    tx[3] = microstep;
    tx[4] = MksServo_GetCheckSum(tx, 4);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 5, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}

/**
 * @brief Установка режима работы сервопривода
 * @param servo указатель на структуру MksServo_t
 * @param mode режим работы (0-5):
 *             0 = CR_OPEN, 1 = CR_CLOSE, 2 = CR_vFOC
 *             3 = SR_OPEN, 4 = SR_CLOSE, 5 = SR_vFOC
 * @retval 1 если успешно, 0 если ошибка
 */
uint8_t MksServo_SetWorkMode(MksServo_t *servo, uint8_t mode)
{
    if (mode > 5) {
        printf("[MKS] Error: Invalid work mode %d (must be 0-5)\r\n", mode);
        return 0;
    }
    
    uint8_t tx[5];
    tx[0] = 0xFA;                        // Head
    tx[1] = servo->device_address;       // Slave addr
    tx[2] = 0x82;                        // Function (Set work mode)
    tx[3] = mode;                        // Data (mode 0-5)
    tx[4] = MksServo_GetCheckSum(tx, 4); // CRC
    
    printf("[MKS] Setting work mode to %d\r\n", mode);
    
    // Включаем передачу RS485
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    
    // Отправляем команду
    HAL_UART_Transmit(servo->huart, tx, 5, 100);
    
    // Выключаем передачу RS485
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    
    // Ждем ответ от сервопривода (FB 01 82 status CRC)
    return MksServo_WaitForACK(servo, 5, 1000);
}

uint8_t MksServo_Calibrate(MksServo_t *servo, uint32_t timeout_ms)
{
    uint8_t tx[5];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0x80;
    tx[3] = 0x00;
    tx[4] = MksServo_GetCheckSum(tx, 4);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 5, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return MksServo_WaitForACK(servo, 5, timeout_ms);
}

void MksServo_UpdateStateFromACK(MksServo_t *servo, const uint8_t *ack, uint8_t len)
{
    if (len < 5) return;
    uint8_t status = ack[3];
    servo->status.last_status_code = status;
    servo->status.last_update_tick = HAL_GetTick();
    // --- Обновление состояния ---
    if (ack[2] == 0xFD) {
        if (status == 0x01) {
            servo->status.state = MKS_SERVO_STATE_MOVING;
            servo->status.is_stopped = 0;
        } else if (status == 0x02) {
            servo->status.state = MKS_SERVO_STATE_STOPPED;
            servo->status.is_stopped = 1;
        } else {
            servo->status.state = MKS_SERVO_STATE_UNKNOWN;
            servo->status.is_stopped = 0;
        }
        // Обновление позиции (позиция приходит в ack[4..7] как int32_t LE)
        if (len >= 8) {
            servo->status.position = (int32_t)(
                ((uint32_t)ack[4]) |
                ((uint32_t)ack[5] << 8) |
                ((uint32_t)ack[6] << 16) |
                ((uint32_t)ack[7] << 24)
            );
        }
    } else {
        servo->status.state = MKS_SERVO_STATE_UNKNOWN;
        servo->status.is_stopped = 0;
    }
}

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
                printf("[MKS] ACK RX: ");
                for (int i = 0; i < len; i++) printf("%02X ", rx[i]);
                printf("\r\n");
                if (rx[len-1] == MksServo_GetCheckSum(rx, len-1)) {
                    MksServo_UpdateStateFromACK(servo, rx, len);
                    return rx[3];
                } else {
                    rx_cnt = 0;
                }
            }
        }
    }
    return 0;
}

uint8_t MksServo_ReadStatus(MksServo_t *servo, uint32_t timeout_ms)
{
    uint8_t tx[4];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF2;
    tx[3] = MksServo_GetCheckSum(tx, 3);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    uint8_t rx[8] = {0};
    uint32_t debug_start = HAL_GetTick();
    uint8_t debug_cnt = 0;
    while ((HAL_GetTick() - debug_start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            printf("[MKS][F2] RX BYTE: %02X\r\n", b);
            if (debug_cnt < 8) rx[debug_cnt++] = b;
            if (debug_cnt == 8) break;
        }
    }
    if (debug_cnt < 8) {
        printf("[MKS][F2] RX TIMEOUT, received %d bytes\r\n", debug_cnt);
        return 0;
    }
    printf("[MKS][F2] RX PACKET: ");
    for (int i = 0; i < 8; i++) printf("%02X ", rx[i]);
    printf("\r\n");
    if (rx[1] != servo->device_address || rx[2] != 0xF2) {
        printf("[MKS][F2] Wrong addr/cmd: %02X %02X\r\n", rx[1], rx[2]);
        return 0;
    }
    servo->status.locked_rotor = rx[3];
    servo->status.encoder_position = rx[4] | (rx[5] << 8);
    return 1;
}

uint8_t MksServo_ReadPositionError(MksServo_t *servo, int32_t *error, uint32_t timeout_ms)
{
    uint8_t tx[4];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0x39;
    tx[3] = MksServo_GetCheckSum(tx, 3);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    uint8_t rx[8] = {0};
    uint32_t start = HAL_GetTick();
    uint8_t found = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            printf("[MKS][39] RX BYTE: %02X\r\n", b);
            if (!found) {
                if (b == 0xFB) {
                    rx[0] = b;
                    found = 1;
                }
            } else {
                rx[8 - (8 - found)] = b;
                found++;
                if (found == 8) break;
            }
        }
    }
    if (found < 8) {
        printf("[MKS][39] RX TIMEOUT, received %d bytes\r\n", found);
        return 0;
    }
    printf("[MKS][39] RX PACKET: ");
    for (int i = 0; i < 8; i++) printf("%02X ", rx[i]);
    printf("\r\n");
    if (rx[1] != servo->device_address || rx[2] != 0x39) {
        printf("[MKS][39] Wrong addr/cmd: %02X %02X\r\n", rx[1], rx[2]);
        return 0;
    }
    *error = ((int32_t)rx[3] << 24) | ((int32_t)rx[4] << 16) | ((int32_t)rx[5] << 8) | ((int32_t)rx[6]);
    return 1;
}

// --- EXT ---
uint8_t MksServo_MoveSteps(MksServo_t *servo, uint8_t dir, uint32_t steps, uint8_t acc)
{
    uint8_t tx[10];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xA3;
    tx[3] = (dir << 7) | ((steps >> 16) & 0x3F);
    tx[4] = (steps >> 8) & 0xFF;
    tx[5] = steps & 0xFF;
    tx[6] = acc;
    tx[7] = 0;
    tx[8] = 0;
    tx[9] = MksServo_GetCheckSum(tx, 9);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 10, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}

uint8_t MksServo_MoveDegrees(MksServo_t *servo, float degrees, uint8_t acc)
{
    int dir = (degrees >= 0) ? 1 : 0;
    float abs_deg = fabsf(degrees);
    uint32_t steps = (uint32_t)((abs_deg / 360.0f) * MKS_SERVO_STEPS_PER_REV);
    return MksServo_MoveSteps(servo, dir, steps, acc);
}

// --- DEBUG ---
uint8_t MksServo_ReadResponse(MksServo_t *servo, uint8_t *rx, uint8_t rx_len, uint32_t timeout_ms)
{
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
            if (rx_cnt == rx_len) {
                printf("[MKS] RX PACKET: ");
                for (int i = 0; i < rx_len; i++) printf("%02X ", rx[i]);
                printf("\r\n");
                if (rx[rx_len-1] == MksServo_GetCheckSum(rx, rx_len-1)) {
                    return 1;
                } else {
                    printf("[MKS] RX CRC ERROR\r\n");
                    return 0;
                }
            }
        }
    }
    printf("[MKS] RX TIMEOUT\r\n");
    return 0;
}

// --- POSITION ---
uint8_t MksServo_PositionMode1Run(MksServo_t *servo, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    uint8_t tx[11];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xFD;
    tx[3] = (dir << 7) | ((speed >> 8) & 0x0F);
    tx[4] = speed & 0xFF;
    tx[5] = acc;
    tx[6] = (pulses >> 24) & 0xFF;
    tx[7] = (pulses >> 16) & 0xFF;
    tx[8] = (pulses >> 8) & 0xFF;
    tx[9] = (pulses >> 0) & 0xFF;
    tx[10] = MksServo_GetCheckSum(tx, 10);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 11, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}

uint8_t MksServo_PositionMode2Run(MksServo_t *servo, int32_t rel_steps, uint16_t speed, uint8_t acc)
{
    uint8_t tx[11];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF4;
    tx[3] = (speed >> 8) & 0xFF;
    tx[4] = speed & 0xFF;
    tx[5] = acc;
    // Little Endian порядок байтов для rel_steps
    tx[6] = (uint8_t)(rel_steps & 0xFF);
    tx[7] = (uint8_t)((rel_steps >> 8) & 0xFF);
    tx[8] = (uint8_t)((rel_steps >> 16) & 0xFF);
    tx[9] = (uint8_t)((rel_steps >> 24) & 0xFF);
    tx[10] = MksServo_GetCheckSum(tx, 10);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 11, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}
uint8_t MksServo_CurrentAxisToZero_92(MksServo_t *servo)
{
uint8_t tx[5];
    tx[0] = 0xFA; // Head
    tx[1] = servo->device_address; // Slave addr
    tx[2] = 0x92; // Function (Current axis to zero)
    tx[3] = 0x00; // Data (not used)
    
    tx[4] = MksServo_GetCheckSum(tx, 4); // CRC (XOR)

    // Включаем DERE для передачи
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 5, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);

    return 1;

}

uint8_t MksServo_AbsoluteMotionByPulse_FE(MksServo_t *servo, uint16_t speed, uint8_t acc, int32_t absPulses)
{
    uint8_t tx[11];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xFE;
    tx[3] = (speed >> 8) & 0xFF;         // speed high byte
    tx[4] = speed & 0xFF;                // speed low byte
    tx[5] = acc;
    tx[6] = (absPulses >> 24) & 0xFF;    // absPulses high byte
    tx[7] = (absPulses >> 16) & 0xFF;
    tx[8] = (absPulses >> 8) & 0xFF;
    tx[9] = absPulses & 0xFF;            // absPulses low byte
    tx[10] = MksServo_GetCheckSum(tx, 10);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 11, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    return 1;
}


void PollPositionErrorTask(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t lastErrorPollTick = 0;
    if (now - lastErrorPollTick >= 1000) // 1 секунда
    {
        lastErrorPollTick = now;
        int32_t pos_error = 0;
        if (MksServo_ReadPositionError(&mksServo, &pos_error, 100)) {
            float deg = pos_error * 360.0f / 51200.0f;
            if (deg > -1.0f && deg < 1.0f) {
                printf("[MKS] [Poll] Position error: <1 deg\r\n");
            } else {
                int ideg = (int)(deg);
                printf("[MKS] [Poll] Position error: %d deg\r\n", ideg);
            }
        } else {
            printf("[MKS] [Poll] Position error read error\r\n");
        }
    }
}

uint8_t MksServo_EmergencyStop(MksServo_t *servo) {
    uint8_t tx[4];
    tx[0] = 0xFA; // Head
    tx[1] = servo->device_address; // Slave addr
    tx[2] = 0xF7; // Function
    tx[3] = tx[0] ^ tx[1] ^ tx[2]; // CRC (XOR)

    // Включаем DERE для передачи
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);

    // Ожидание ответа через циклический буфер
    uint8_t rx[6];
    uint32_t start = HAL_GetTick();
    uint8_t rx_cnt = 0;
    while ((HAL_GetTick() - start) < 100) { // 100 мс таймаут
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            if (rx_cnt != 0) {
                rx[rx_cnt++] = b;
            } else if (b == 0xFB) {
                rx[rx_cnt++] = b;
            }
            if (rx_cnt == 6) {
                printf("[MKS][F7] RX PACKET: ");
                for (int i = 0; i < 6; i++) printf("%02X ", rx[i]);
                printf("\r\n");
                uint8_t crc = rx[0] ^ rx[1] ^ rx[2] ^ rx[3] ^ rx[4];
                if (crc != rx[5]) {
                    printf("[MKS][F7] CRC ERROR: %02X != %02X\r\n", crc, rx[5]);
                    return 0;
                }
                if (rx[1] == servo->device_address && rx[2] == 0xF7) {
                    printf("[MKS][F7] STATUS: %d\r\n", rx[3]);
                    return rx[3] == 1 ? 1 : 0;
                } else {
                    printf("[MKS][F7] ADDR/CMD ERROR: %02X %02X\r\n", rx[1], rx[2]);
                    return 0;
                }
            }
        }
    }
    printf("[MKS][F7] RX TIMEOUT, received %d bytes\r\n", rx_cnt);
    return 0;
}

// Fire-and-forget аварийная остановка по простому протоколу (0xAA 0x08 0x00 0x00 0x55)
void MksServo_EmergencyStop_Simple(MksServo_t *servo) {
    uint8_t buffer[5] = {0xAA, 0xF7, 0x00, 0x00, 0x55};
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, buffer, 5, 100);
   
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    printf("[MKS][SIMPLE STOP] Sent: AA F7 00 00 55\r\n");
}

// --- ENCODER READOUT ---
// Чтение carry и value (команда 0x30)
uint8_t MksServo_GetCarry(MksServo_t *servo, int32_t *carry, uint16_t *value, uint32_t timeout_ms) {
    // Очистка кольцевого буфера перед отправкой команды
    servo->rx_head = 0;
    servo->rx_tail = 0;
    uint8_t tx[4];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0x30; // Команда 0x30 для carry
    tx[3] = MksServo_GetCheckSum(tx, 3);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);

    // Ожидаем ответ: FB addr 30 [carry(4)] [value(2)] CRC
    uint8_t rx[10];
    uint32_t start = HAL_GetTick();
    uint8_t rx_cnt = 0;
    uint8_t found = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            if (!found) {
                if (b == 0xFB) {
                    rx[0] = b;
                    rx_cnt = 1;
                    found = 1;
                }
            } else {
                rx[rx_cnt++] = b;
                if (rx_cnt == 10) {
                    // Выводим весь принятый пакет для отладки
                    printf("[MKS][0x30] RX PACKET: ");
                    for (int i = 0; i < 10; i++) printf("%02X ", rx[i]);
                    printf("\r\n");
                    // Проверка CRC
                    if (rx[9] == MksServo_GetCheckSum(rx, 9)) {
                        if (rx[2] == 0x30) {
                            // Изменён порядок байтов carry на little-endian
                            *carry = (int32_t)(
                                ((uint32_t)rx[6]) |
                                ((uint32_t)rx[5] << 8) |
                                ((uint32_t)rx[4] << 16) |
                                ((uint32_t)rx[3] << 24)
                            );
                            *value = (uint16_t)(rx[7] | (rx[8] << 8));
                            printf("[MKS] Encoder carry: %ld, value: %u\r\n", (long)*carry, *value);
                            return 1;
                        }
                    }
                    found = 0;
                    rx_cnt = 0;
                }
            }
        }
    }
    printf("[MKS] Encoder carry read timeout or CRC error\r\n");
    return 0;
}

// Чтение addition value (команда 0x31)
uint8_t MksServo_GetAdditionValue(MksServo_t *servo, int64_t *addition_value, uint32_t timeout_ms) {
    uint8_t tx[4];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0x31;
    tx[3] = MksServo_GetCheckSum(tx, 3);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);

    // Ожидаем ответ: FB addr 31 [addition_value(6)] CRC
    uint8_t rx[11];
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
            if (rx_cnt == 10) {
                // Проверка CRC
                if (rx[9] == MksServo_GetCheckSum(rx, 9)) {
                    if (rx[2] == 0x31) {
                        *addition_value = 0;
                        // Big-endian: rx[3] - старший, rx[8] - младший
                        for (int i = 0; i < 6; ++i) {
                            *addition_value |= ((int64_t)rx[3 + i]) << (8 * (5 - i));
                        }
                        // Sign extension для int48_t (big-endian)
                        if (rx[3] & 0x80) {
                            *addition_value |= (int64_t)0xFFFF000000000000;
                        }
                        printf("[MKS] Encoder addition value: %" PRId64 "\r\n", *addition_value);
                        return 1;
                    }
                }
                rx_cnt = 0;
            }
        }
    }
    printf("[MKS] Encoder addition value read timeout or CRC error\r\n");
    return 0;
}

uint8_t MksServo_AbsoluteMotionByAxis_F5(MksServo_t *servo, int64_t *addition_value, uint32_t timeout_ms)
{
    // Очистка буфера приема перед отправкой команды
    servo->rx_head = 0;
    servo->rx_tail = 0;
    uint8_t tx[11];
    uint16_t speed = 300;
    uint8_t acc = 255;
    int32_t absAxis = (int32_t)(*addition_value);
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF5;
    tx[3] = (speed >> 8) & 0xFF;
    tx[4] = speed & 0xFF;
    tx[5] = acc;
    tx[6] = (absAxis >> 24) & 0xFF;
    tx[7] = (absAxis >> 16) & 0xFF;
    tx[8] = (absAxis >> 8) & 0xFF;
    tx[9] = absAxis & 0xFF;
    tx[10] = MksServo_GetCheckSum(tx, 10);
    uint32_t start = HAL_GetTick();
    uint8_t rx[5];
    uint8_t rx_cnt = 0;
    uint8_t last_nonf5_code = 0;
    uint8_t last_nonf5_rx[5] = {0};
    while ((HAL_GetTick() - start) < timeout_ms) {
        HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_UART_Transmit(servo->huart, tx, 11, 100);
        HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
        printf("[MKS][F5] Sent: ");
        for (int i = 0; i < 11; i++) printf("%02X ", tx[i]);
        printf("\r\n");
        uint32_t wait_start = HAL_GetTick();
        rx_cnt = 0;
        while ((HAL_GetTick() - wait_start) < 200 && (HAL_GetTick() - start) < timeout_ms) {
            uint8_t b;
            if (MksServo_RxGetByte(servo, &b)) {
                if (rx_cnt != 0) {
                    rx[rx_cnt++] = b;
                } else if (b == 0xFB) {
                    rx[rx_cnt++] = b;
                }
                if (rx_cnt == 5) {
                    printf("[MKS][F5] RX: ");
                    for (int i = 0; i < 5; i++) printf("%02X ", rx[i]);
                    printf("\r\n");
                    if (rx[4] == MksServo_GetCheckSum(rx, 4)) {
                        if (rx[2] == 0xF5) {
                            uint8_t status = rx[3];
                            last_f5_status = status;
                            printf("[MKS][F5] Status: %d\r\n", status);
                            if (status == 1) {
                                // Движение начато — сразу успех
                                return 1;
                            }
                            if (status == 2 || status == 3) {
                                printf("[MKS][F5] Motion complete (status %d)\r\n", status);
                                return 1;
                            }
                            if (status == 0) {
                                printf("[MKS][F5] Unexpected status: 0, retrying after 20ms\r\n");
                                HAL_Delay(20);
                                break;
                            }
                            printf("[MKS][F5] Unexpected status: %d\r\n", status);
                            return 0;
                        } else if (rx[2] == 0xF6) {
                            last_f6_status = rx[3];
                            printf("[MKS][F5] F6 status seen in F5 wait: %d\r\n", rx[3]);
                            rx_cnt = 0;
                            continue;
                        } else {
                            last_nonf5_code = rx[2];
                            memcpy(last_nonf5_rx, rx, 5);
                            printf("[MKS][F5] Unexpected packet code: %02X (expected F5), ignoring\r\n", rx[2]);
                            rx_cnt = 0;
                            continue;
                        }
                    } else {
                        printf("[MKS][F5] CRC error, ignoring packet\r\n");
                        rx_cnt = 0;
                    }
                }
            }
        }
        if ((HAL_GetTick() - start) >= timeout_ms) break;
    }
    printf("[MKS][F5] Timeout waiting for F5 response\r\n");
    if (last_nonf5_code) {
        printf("[MKS][F5] Last non-F5 packet: ");
        for (int i = 0; i < 5; i++) printf("%02X ", last_nonf5_rx[i]);
        printf("\r\n");
    }
    return 0;
}

// --- Отправка "сырых" пакетов по UART3 (RS485) ---
// --- Сохраняет CRC последней отправленной команды для анализа статусов FD ---
void MksServo_SendRaw(MksServo_t *servo, const uint8_t *data, uint16_t len) {
    // Save CRC of the last sent command in the history buffer
    if (len > 0) {
        extern uint8_t last_cmd_crc_history[];
        extern uint8_t last_cmd_crc_index;
        last_cmd_crc_history[last_cmd_crc_index] = data[len-1];
        last_cmd_crc_index = (last_cmd_crc_index + 1) % CMD_CRC_HISTORY;
        last_fd_status.last_cmd_crc = data[len-1]; // для совместимости
    }
    // Enable RS485 transmit
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_UART_Transmit(servo->huart, (uint8_t*)data, len, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
}

uint8_t MksServo_QueryStatus_F1(MksServo_t *servo, uint32_t timeout_ms)
{
    uint8_t tx[4];
    tx[0] = 0xFA;
    tx[1] = servo->device_address;
    tx[2] = 0xF1;
    tx[3] = MksServo_GetCheckSum(tx, 3);
    printf("[MKS][F1] Sent: ");
    for (int i = 0; i < 4; i++) printf("%02X ", tx[i]);
    printf("\r\n");
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo->huart, tx, 4, 100);
    HAL_GPIO_WritePin(servo->dere_port, servo->dere_pin, GPIO_PIN_RESET);
    uint32_t start = HAL_GetTick();
    uint8_t rx[5];
    uint8_t rx_cnt = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(servo, &b)) {
            if (rx_cnt != 0) {
                rx[rx_cnt++] = b;
            } else if (b == 0xFB) {
                rx[rx_cnt++] = b;
            }
            if (rx_cnt == 5) {
                printf("[MKS][F1] RX: ");
                for (int i = 0; i < 5; i++) printf("%02X ", rx[i]);
                printf("\r\n");
                if (rx[4] == MksServo_GetCheckSum(rx, 4) && rx[2] == 0xF1) {
                    uint8_t status = rx[3];
                    printf("[MKS][F1] Status: %d\r\n", status);
                    return status;
                } else {
                    printf("[MKS][F1] Unexpected packet or CRC error\r\n");
                }
                rx_cnt = 0;
            }
        }
    }
    printf("[MKS][F1] Timeout waiting for F1 response\r\n");
    return 0;
}