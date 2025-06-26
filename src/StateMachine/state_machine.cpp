#include "state_machine.h"
#include "main.h" // Добавьте этот include ПЕРЕД другими
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "../Buttons/buttons.h"
#include "Potentiometr\Potentiometer.h"
#include <stdio.h>

StateMachine::StateMachine() : currentState(State::Initial) {}

State StateMachine::getState() const { return currentState; }
void StateMachine::setState(State newState) { currentState = newState; }
bool StateMachine::is(State state) const { return currentState == state; }

StateMachine stateMachine; // Экземпляр stateMachine
// В начале файла (например, state_machine_task.cpp)

const char *stateToStr(State state)
{
    switch (state)
    {
    case State::Initial:
        return "Initial";
    case State::Manual:
        return "Manual";
    case State::GiroScope:
        return "GiroScope";
    case State::Scan:
        return "Scan";
    case State::BindMode:
        return "BindMode";
    case State::Calibrate:
        return "Calibrate";
    case State::CalibrateAndBind:
        return "CalibrateAndBind";
    default:
        return "Unknown";
    }
}

void StateMachine_loop(void)
{
   
    

    if (updateButtonsState())
    {
        // 1. Приоритет: CalibrateAndBind
        // Вывод значения потенциометра в процентах
        printf("[FSM] Potentiometer: %d%%\n", getPotentiometerValuePercentage());
        if (buttonsState.calibrate == BUTTON_ON && buttonsState.bind_mode == BUTTON_ON)
        {
            if (!stateMachine.is(State::CalibrateAndBind))
            {
                stateMachine.setState(State::CalibrateAndBind);
                printf("[FSM] -> CalibrateAndBind\n");
                HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET); // Выключить лампочку
            }
        }
        // 2. Приоритет: Bind
        else if (buttonsState.bind_mode == BUTTON_ON)
        {
            if (!stateMachine.is(State::BindMode))
            {
                stateMachine.setState(State::BindMode);
                printf("[FSM] -> BindMode\n");
                HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET); // Выключить лампочку
            }
        }
        // 3. Приоритет: Calibrate
        else if (buttonsState.calibrate == BUTTON_ON)
        {
            if (!stateMachine.is(State::Calibrate))
            {
                stateMachine.setState(State::Calibrate);
                printf("[FSM] -> Calibrate\n");
                HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET); // Выключить лампочку
            }
        }
        // 4. Gyro работает если FSM в Manual или Initial и не нажаты Bind/Calibrate
        else if (
            (stateMachine.is(State::Manual) || stateMachine.is(State::Initial)) &&
            buttonsState.gyro == BUTTON_ON)
        {
            stateMachine.setState(State::GiroScope);
            printf("[FSM] -> GiroScope\n");
            HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_SET);
        }
        // 5. Если были в GiroScope и gyro отпущена — вернуться в Manual
        else if (stateMachine.is(State::GiroScope) && buttonsState.gyro == BUTTON_OFF)
        {
            stateMachine.setState(State::Manual);
            HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET); // Выключить лампочку

            printf("[FSM] -> Manual (from GiroScope)\n");
        }
        // 6. Если были в Calibrate/Bind/CalibrateAndBind и gyro нажата — возврат в GiroScope
        else if (
            buttonsState.gyro == BUTTON_ON &&
            (stateMachine.is(State::Calibrate) || stateMachine.is(State::BindMode) || stateMachine.is(State::CalibrateAndBind)))
        {
            stateMachine.setState(State::GiroScope);
            printf("[FSM] -> GiroScope (after Calibrate/Bind)\n");
            HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_SET); // Выключить лампочку
        }
        // 7. Если были в Initial и нажата любая педаль — Manual
        else if (stateMachine.is(State::Initial) &&
                 (buttonsState.turn_left == BUTTON_ON || buttonsState.turn_right == BUTTON_ON))
        {
            stateMachine.setState(State::Manual);
            printf("[FSM] -> Manual (from Initial)\n");
        }
        // Только для BindMode, Calibrate, CalibrateAndBind (и GiroScope, если нужно)
        else if (
            (stateMachine.is(State::BindMode) ||
             stateMachine.is(State::Calibrate) ||
             stateMachine.is(State::CalibrateAndBind)) &&
            buttonsState.bind_mode == BUTTON_OFF &&
            buttonsState.calibrate == BUTTON_OFF &&
            buttonsState.gyro == BUTTON_OFF &&
            buttonsState.turn_left == BUTTON_OFF &&
            buttonsState.turn_right == BUTTON_OFF)
        {
            stateMachine.setState(State::Initial);
            printf("[FSM] -> Initial (from Bind/Calibrate)\n");
        }
        printf("Current state: %s\n", stateToStr(stateMachine.getState()));
    }

    
}

void StateMachine_setup(void)
{

 stateMachine.setState(State::Initial);
  buttonsDebounceInit();

}
