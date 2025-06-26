#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t stable_state;
    uint8_t last_state;
    uint8_t counter;
    uint8_t threshold;
} DebounceButton;

// Инициализация структуры
void debounce_init(DebounceButton* btn, GPIO_TypeDef* port, uint16_t pin, uint8_t threshold);

// Проверка состояния с подавлением дребезга (вызывать в цикле)
uint8_t debounce_read(DebounceButton* btn);
