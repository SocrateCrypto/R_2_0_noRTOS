#include "anti_debounce.h"

void debounce_init(DebounceButton* btn, GPIO_TypeDef* port, uint16_t pin, uint8_t threshold) {
    btn->port = port;
    btn->pin = pin;
    btn->stable_state = HAL_GPIO_ReadPin(port, pin);
    btn->last_state = btn->stable_state;
    btn->counter = 0;
    btn->threshold = threshold;
}

uint8_t debounce_read(DebounceButton* btn) {
    uint8_t current = HAL_GPIO_ReadPin(btn->port, btn->pin);
    if (current == btn->last_state) {
        if (btn->counter < btn->threshold) btn->counter++;
    } else {
        btn->counter = 0;
    }
    btn->last_state = current;
    if (btn->counter >= btn->threshold) {
        btn->stable_state = current;
    }
    return btn->stable_state;
}