#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Адрес последней страницы Flash памяти для хранения данных
// STM32F103C8 имеет Flash размером 64KB (0x08000000 - 0x0800FFFF)
// Используем последнюю страницу (1KB) для хранения настроек
#define FLASH_STORAGE_START_ADDR    0x0800FC00  // Последняя страница для 64KB Flash
#define FLASH_STORAGE_PAGE_SIZE     1024        // Размер страницы Flash
#define FLASH_STORAGE_MAGIC         0xABCD1234  // Магическое число для проверки валидности

// Структура данных для хранения в Flash
typedef struct __attribute__((packed)) {
    uint32_t magic;                    // Магическое число для проверки
    uint8_t remote_address[5];         // Адрес пульта NRF24L01 (радио педалей)
    uint32_t binding_count;            // Счетчик привязок (для отладки)
    uint32_t last_binding_time;        // Время последней привязки
    uint32_t checksum;                 // Контрольная сумма
    uint8_t reserved[32];              // Резерв для будущих параметров
} FlashStorage_t;

// Функции для работы с Flash памятью
HAL_StatusTypeDef FlashStorage_Init(void);
HAL_StatusTypeDef FlashStorage_SaveData(const FlashStorage_t* data);
HAL_StatusTypeDef FlashStorage_LoadData(FlashStorage_t* data);
HAL_StatusTypeDef FlashStorage_EraseData(void);
bool FlashStorage_IsDataValid(void);

// Вспомогательные функции
uint32_t FlashStorage_CalculateChecksum(const FlashStorage_t* data);
void FlashStorage_SetDefaultValues(FlashStorage_t* data);

// Функции для работы с адресом пульта
HAL_StatusTypeDef FlashStorage_SaveRemoteAddress(const uint8_t address[5]);
HAL_StatusTypeDef FlashStorage_LoadRemoteAddress(uint8_t address[5]);
bool FlashStorage_HasValidRemoteAddress(void);
void FlashStorage_PrintStoredAddress(void);

// Отладочные функции
void FlashStorage_DebugDumpFlash(void);
HAL_StatusTypeDef FlashStorage_ForceErase(void);

#ifdef __cplusplus
}
#endif

#endif // FLASH_STORAGE_H
