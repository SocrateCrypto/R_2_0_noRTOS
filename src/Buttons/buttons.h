#ifndef BUTTONS_H
#define BUTTONS_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_OFF = 0,
    BUTTON_ON = 1
} ButtonState;

typedef struct {
    ButtonState gyro;
    ButtonState turn_right;
    ButtonState turn_left;
    ButtonState calibrate;
    ButtonState bind_mode;
} ButtonsState;

// Добавьте этот enum для событий двойного нажатия
typedef enum {
    DOUBLE_BTN_NONE = 0,
    DOUBLE_BTN_SHORT = 1,   // Короткое удержание (>500 мс, <5 сек)
    DOUBLE_BTN_LONG = 2,    // Долгое удержание (>5 сек)
    DOUBLE_BTN_PRESS = 3,   // Первое одновременное нажатие
    DOUBLE_BTN_RELEASE = 4  // Одновременное отпускание
} DoubleButtonEvent;

extern ButtonsState buttonsState;

// Функции обновления состояния кнопок
void buttonsDebounceInit(void);
bool updateButtonsState();
bool updateGyroButtonState();
bool updateTurnRightButtonState();
bool updateTurnLeftButtonState();
bool updateCalibrateButtonState();
bool updateBindModeButtonState();
DoubleButtonEvent updateDoubleButtonsState(bool autoReset);

// Функции для прямого чтения педалей без антидребезга (для быстрой остановки в Manual)
inline bool isLeftPedalPressed_Raw() {
    return (HAL_GPIO_ReadPin(GPIOB, LEFT_Pin) == GPIO_PIN_RESET);
}

inline bool isRightPedalPressed_Raw() {
    return (HAL_GPIO_ReadPin(GPIOB, RIGHT_Pin) == GPIO_PIN_RESET);
}

inline bool areBothPedalsReleased_Raw() {
    return (!isLeftPedalPressed_Raw() && !isRightPedalPressed_Raw());
}

#ifdef __cplusplus
}
#endif

#endif // BUTTONS_H
