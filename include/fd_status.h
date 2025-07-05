#ifndef FD_STATUS_H
#define FD_STATUS_H

#include <stdint.h>

// --- Структура для хранения последнего принятого FD-ответа (статусного пакета) ---
typedef struct {
    uint8_t head;      // 0xFB
    uint8_t addr;      // Slave addr
    uint8_t func;      // 0xFD
    uint8_t status;    // status (0,1,2)
    uint8_t crc;       // CRC
    uint8_t last_cmd_crc; // CRC последней отправленной команды
} FD_status_t;

extern FD_status_t last_fd_status;

#define CMD_CRC_HISTORY 3
extern uint8_t last_cmd_crc_history[CMD_CRC_HISTORY];
extern uint8_t last_cmd_crc_index;

#endif // FD_STATUS_H
