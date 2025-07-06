#include "buttons.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "anti_debounce.h"
#include <stdio.h>

ButtonsState buttonsState = {BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF};

// --- Добавьте эти строки ---
static DebounceButton leftDebounce;
static DebounceButton rightDebounce;

// ---------------------------

void buttonsDebounceInit(void)
{
    debounce_init(&leftDebounce, GPIOB, LEFT_Pin, 15);   // 15 мс — время игнорирования дребезга
    debounce_init(&rightDebounce, GPIOB, RIGHT_Pin, 15); // можно подобрать оптимальное значение
}

bool updateGyroButtonState()
{
    ButtonState prev = buttonsState.gyro;
    buttonsState.gyro = (HAL_GPIO_ReadPin(GPIOA, COMP_Pin) == GPIO_PIN_RESET) ? BUTTON_ON : BUTTON_OFF;
    return (prev != buttonsState.gyro);
}

bool updateTurnRightButtonState()
{
    ButtonState prev = buttonsState.turn_right;
    buttonsState.turn_right = (debounce_read(&rightDebounce) == GPIO_PIN_RESET) ? BUTTON_ON : BUTTON_OFF;
    return (prev != buttonsState.turn_right);
}

bool updateTurnLeftButtonState()
{
    ButtonState prev = buttonsState.turn_left;
    buttonsState.turn_left = (debounce_read(&leftDebounce) == GPIO_PIN_RESET) ? BUTTON_ON : BUTTON_OFF;
    return (prev != buttonsState.turn_left);
}

bool updateCalibrateButtonState()
{
    ButtonState prev = buttonsState.calibrate;
    buttonsState.calibrate = (HAL_GPIO_ReadPin(GPIOA, CALL_Pin) == GPIO_PIN_RESET) ? BUTTON_ON : BUTTON_OFF;
    return (prev != buttonsState.calibrate);
}

bool updateBindModeButtonState()
{
    ButtonState prev = buttonsState.bind_mode;
    buttonsState.bind_mode = (HAL_GPIO_ReadPin(GPIOA, BIND_Pin) == GPIO_PIN_RESET) ? BUTTON_ON : BUTTON_OFF;
    return (prev != buttonsState.bind_mode);
}

DoubleButtonEvent updateDoubleButtonsState(bool autoReset)
{
    static uint8_t prev = 0;
    static uint32_t press_start = 0;
    static uint8_t event_sent = 0; // 0: ничего, 1: PRESS, 2: SHORT, 3: LONG
    static uint8_t released_sent = 0;

    uint8_t now = (buttonsState.turn_left == BUTTON_ON && buttonsState.turn_right == BUTTON_ON);
    DoubleButtonEvent result = DOUBLE_BTN_NONE;

    if (now) {
        if (!prev) {
            // Первое одновременное нажатие
            press_start = HAL_GetTick();
            event_sent = 0;
            released_sent = 0;
            result = DOUBLE_BTN_PRESS;
            event_sent = 1;
            printf("[DBTN] PRESS: both pedals pressed, t=%lu\n", HAL_GetTick());
        } else {
            uint32_t held = HAL_GetTick() - press_start;
            if (held >= 5000 && event_sent < 3) {
                result = DOUBLE_BTN_LONG;
                event_sent = 3;
                printf("[DBTN] LONG: held %lu ms\n", held);
            } else if (held >= 500 && event_sent < 2) {
                result = DOUBLE_BTN_SHORT;
                event_sent = 2;
                printf("[DBTN] SHORT: held %lu ms\n", held);
            }
        }
    } else {
        if (prev && !released_sent) {
            result = DOUBLE_BTN_RELEASE;
            released_sent = 1;
            printf("[DBTN] RELEASE: both pedals released, t=%lu\n", HAL_GetTick());
        }
        press_start = 0;
        event_sent = 0;
    }
    prev = now;
    return result;
}

bool updateButtonsState()
{
    static ButtonsState prevState = {BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF, BUTTON_OFF};
    bool changed = false;

    changed |= updateBindModeButtonState();
    changed |= updateCalibrateButtonState();
    changed |= updateGyroButtonState();
    changed |= updateTurnLeftButtonState();
    changed |= updateTurnRightButtonState();

    // Логирование однократного события нажатия/отпускания любой кнопки
    if (
        prevState.bind_mode != buttonsState.bind_mode ||
        prevState.calibrate != buttonsState.calibrate ||
        prevState.gyro != buttonsState.gyro ||
        prevState.turn_left != buttonsState.turn_left ||
        prevState.turn_right != buttonsState.turn_right
    ) {
        #define BTN_STR(x) ((x) == BUTTON_ON ? "ON" : "OFF")
        printf("[Buttons] State changed: bind=%s, cal=%s, gyro=%s, left=%s, right=%s\n",
            BTN_STR(buttonsState.bind_mode),
            BTN_STR(buttonsState.calibrate),
            BTN_STR(buttonsState.gyro),
            BTN_STR(buttonsState.turn_left),
            BTN_STR(buttonsState.turn_right)
        );
        prevState = buttonsState;
        #undef BTN_STR
    }

    return changed;
}
