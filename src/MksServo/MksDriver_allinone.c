// Объединённая реализация всех функций управления MKS Servo
#include "MksDriver_allinone.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
extern MksServo_t mksServo;
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
                            printf("[MKS] Encoder carry: %ld, value: %u\r\n", *carry, *value);
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
                        for (int i = 0; i < 6; ++i) {
                            *addition_value |= ((int64_t)rx[3 + i]) << (8 * i);
                        }
                        printf("[MKS] Encoder addition value: %lld\r\n", *addition_value);
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