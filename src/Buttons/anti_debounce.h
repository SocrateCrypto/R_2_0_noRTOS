#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t stable_state;
    uint32_t last_change_time;
    uint8_t debounce_time_ms;  // время игнорирования дребезга в мс
} DebounceButton;

// Инициализация структуры
void debounce_init(DebounceButton* btn, GPIO_TypeDef* port, uint16_t pin, uint8_t debounce_time_ms);

// Проверка состояния с подавлением дребезга (вызывать в цикле)
// Мгновенно реагирует на первое изменение, затем игнорирует дребезг debounce_time_ms миллисекунд
uint8_t debounce_read(DebounceButton* btn);
