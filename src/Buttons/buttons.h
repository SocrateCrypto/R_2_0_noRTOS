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
    DOUBLE_BTN_SHORT = 1,
    DOUBLE_BTN_LONG = 2
} DoubleButtonEvent;

extern ButtonsState buttonsState;

// Функции для работы с кнопками
void buttonsDebounceInit(void);
bool updateButtonsState(void);
bool updateGyroButtonState(void);
bool updateTurnRightButtonState(void);
bool updateTurnLeftButtonState(void);
bool updateCalibrateButtonState(void);
bool updateBindModeButtonState(void);

// Добавьте прототип функции для двойного нажатия
DoubleButtonEvent updateDoubleButtonsState(bool autoReset);

#ifdef __cplusplus
}
#endif

#endif // BUTTONS_H
