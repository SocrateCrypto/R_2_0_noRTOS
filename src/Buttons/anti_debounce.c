#include "anti_debounce.h"

void debounce_init(DebounceButton* btn, GPIO_TypeDef* port, uint16_t pin, uint8_t debounce_time_ms) {
    btn->port = port;
    btn->pin = pin;
    btn->stable_state = HAL_GPIO_ReadPin(port, pin);
    btn->last_change_time = 0;
    btn->debounce_time_ms = debounce_time_ms;
}

uint8_t debounce_read(DebounceButton* btn) {
    uint8_t current = HAL_GPIO_ReadPin(btn->port, btn->pin);
    uint32_t now = HAL_GetTick();
    
    // Если состояние изменилось и прошло достаточно времени с последнего изменения
    if (current != btn->stable_state) {
        if (now - btn->last_change_time >= btn->debounce_time_ms) {
            // Мгновенно принимаем новое состояние
            btn->stable_state = current;
            btn->last_change_time = now;
        }
        // Если времени прошло мало - игнорируем (это дребезг)
    }
    
    return btn->stable_state;
}