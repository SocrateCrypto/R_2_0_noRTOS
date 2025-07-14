#include "state_machine.h"
#include "main.h" // Добавьте этот include ПЕРЕД другими
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "../Buttons/buttons.h"
#include "Potentiometr\Potentiometer.h"
#include "MksServo/MksDriver_allinone.h"
#include "NRF/nrf.h" // Добавляем для функций привязки
#include <stdio.h>

bool isSynchronized = false; // синхронно ли нажаты педали 
AngleSetting mode_scan = ANGLE_SCAN;
StateMachine::StateMachine() : currentState(State::Initial), homeCarry(0), homeValue(0), homePositionSet(false) {}

State StateMachine::getState() const { return currentState; }
void StateMachine::setState(State newState) { currentState = newState; }
bool StateMachine::is(State state) const { return currentState == state; }

// Методы для работы с домашней позицией энкодера
void StateMachine::setHomePosition(int32_t carry, uint16_t value)
{
    homeCarry = carry;
    homeValue = value;
    homePositionSet = true;
    printf("[StateMachine] Home position set: carry=%ld, value=%d\r\n", (long)carry, value);
}

void StateMachine::getHomePosition(int32_t *carry, uint16_t *value) const
{
    if (carry)
        *carry = homeCarry;
    if (value)
        *value = homeValue;
}

bool StateMachine::hasHomePosition() const
{
    return homePositionSet;
}

void StateMachine::clearHomePosition()
{
    homeCarry = 0;
    homeValue = 0;
    homePositionSet = false;
    printf("[StateMachine] Home position cleared\r\n");
}

StateMachine stateMachine; // Экземпляр stateMachine
// В начале файла (например, state_machine_task.cpp)
extern MksServo_t mksServo; // Глобальная переменная для MKS Servo
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

#define ENTRY_SCAN_BLOCK_MS 100 // Время блокировки повторной записи entry_scan_point (мс)



// Глобальный экземпляр структуры для хранения значений энкодера
EncoderScanPoints encoderScanPoints = {0, 0};

void StateMachine_loop(void)
{
    DoubleButtonEvent updateDoubleButtons = updateDoubleButtonsState(false);
    if (updateDoubleButtons == DOUBLE_BTN_PRESS)
    {
        printf("[DBN] Double button pressed\n");
    }
    else if (updateDoubleButtons == DOUBLE_BTN_RELEASE)
    {
        printf("[DBN] Double button released\n");
    }
    else if (updateDoubleButtons == DOUBLE_BTN_SHORT)
    {
        printf("[DBN] Double button short press\n");
    }
    else if (updateDoubleButtons == DOUBLE_BTN_LONG)
    {
        if (stateMachine.is(State::Scan))
        {
            // Если в Scan, то выходим из него

            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); // Выключить лампочку
            mode_scan = ANGLE_ADJUST;
            nrf_send_angle_agiust_mode(); // Отправляем режим ANGLE_ADJUST
        }

        printf("[DBN] Double button long press\n");
    }
   
    static bool flag_blocked_for_debounce = 0; // Флаг блокировки Scan
    static uint32_t debounce_timer = 0;
    static int last_speed = 0;     // Последняя скорость для ANGLE_ADJUST
    extern AngleSetting mode_scan; // Используем глобальную переменную

    if (updateButtonsState())
    {

  static uint32_t first_btn_press_time = 0;
    static uint8_t first_btn = 0; // 1 - left, 2 - right
    static uint8_t double_btn_sync_checked = 0;
    // Проверка асинхронности двойного нажатия
    if (!double_btn_sync_checked) {
        if (buttonsState.turn_left == BUTTON_ON && first_btn == 0) {
            first_btn_press_time = HAL_GetTick();
            first_btn = 1;
        } else if (buttonsState.turn_right == BUTTON_ON && first_btn == 0) {
            first_btn_press_time = HAL_GetTick();
            first_btn = 2;
        }
        // Если вторая кнопка нажата
        if (first_btn == 1 && buttonsState.turn_right == BUTTON_ON) {
            uint32_t dt = HAL_GetTick() - first_btn_press_time;
            if (dt < 400) {
                printf("[DBN] Double button press: SYNC (dt=%lu ms)\n", dt);
                isSynchronized = true; // Устанавливаем флаг синхронизации
            } else {
                printf("[DBN] Double button press: ASYNC (dt=%lu ms)\n", dt);
                isSynchronized = false; // Сброс флага синхронизации
            }
            double_btn_sync_checked = 1;
        } else if (first_btn == 2 && buttonsState.turn_left == BUTTON_ON) {
            uint32_t dt = HAL_GetTick() - first_btn_press_time;
            if (dt < 400) {
                printf("[DBN] Double button press: SYNC (dt=%lu ms)\n", dt);
                isSynchronized = true; // Устанавливаем флаг синхронизации
            } else {
                printf("[DBN] Double button press: ASYNC (dt=%lu ms)\n", dt);
                isSynchronized = false; // Сброс флага синхронизации
            }
            double_btn_sync_checked = 1;
        }
    }
    // Сброс при отпускании обеих кнопок
    if (buttonsState.turn_left == BUTTON_OFF && buttonsState.turn_right == BUTTON_OFF) {
        first_btn = 0;
        first_btn_press_time = 0;
        double_btn_sync_checked = 0;
    }

        // 1. Приоритет: CalibrateAndBind
        // Вывод значения потенциометра в процентах
        printf("[FSM] Potentiometer: %d%%\n", getPotentiometerValuePercentage());
        if (buttonsState.calibrate == BUTTON_ON && buttonsState.bind_mode == BUTTON_ON)
        {
            if (!stateMachine.is(State::CalibrateAndBind))
            {
                stateMachine.setState(State::CalibrateAndBind);
                printf("[FSM] -> CalibrateAndBind\n");
                nrf_enter_binding_mode();                                    // Входим в режим привязки NRF
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
                nrf_enter_binding_mode();                                    // Входим в режим привязки NRF
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
            int64_t add_val = 0;
            MksServo_GetAdditionValue(&mksServo, &add_val, 100);
            encoderScanPoints.giro_point = add_val;
           // encoderScanPoints.cumulativeYaw =cumulativeYaw; // Сохраняем накопленный угол поворота
            encoderScanPoints.flag_first_run = true; // Устанавливаем флаг первого запуска
            // PID.reset();
        }
        // 5. Если были в GiroScope и gyro отпущена — вернуться в Manual
        else if (stateMachine.is(State::GiroScope) && buttonsState.gyro == BUTTON_OFF)
        {
              MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0); 
            stateMachine.setState(State::Manual);
            HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET); // Выключить лампочку
                       // stop servo
            HAL_Delay(100); // Задержка для стабилизации после остановки
             MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0);              // stop servo
            printf("[FSM] -> Manual (from GiroScope)\n");
        }
        // 6. Если были в Calibrate/Bind/CalibrateAndBind и gyro нажата — возврат в GiroScope
        else if (
            buttonsState.gyro == BUTTON_ON &&
            (stateMachine.is(State::Calibrate) || stateMachine.is(State::BindMode) || stateMachine.is(State::CalibrateAndBind)))
        {
            // Если выходим из режимов с привязкой - отключаем режим привязки
            if (stateMachine.is(State::BindMode) || stateMachine.is(State::CalibrateAndBind))
            {
                nrf_exit_binding_mode(); // Выходим из режима привязки NRF
            }

            stateMachine.setState(State::GiroScope);
            printf("[FSM] -> GiroScope (after Calibrate/Bind)\n");
            HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_SET); // Выключить лампочку
        }
        // 7. Если были в Initial и нажата любая педаль — Manual
        else if (stateMachine.is(State::Initial) &&
                 (buttonsState.turn_left == BUTTON_ON || buttonsState.turn_right == BUTTON_ON))
        {
            stateMachine.setState(State::Manual);
            MksServo_SpeedModeRun(&mksServo, 0x00, 0, 250); // stop servo
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
            // Если выходим из режимов с привязкой - отключаем режим привязки
            if (stateMachine.is(State::BindMode) || stateMachine.is(State::CalibrateAndBind))
            {
                nrf_exit_binding_mode(); // Выходим из режима привязки NRF
            }

            stateMachine.setState(State::Initial);
            printf("[FSM] -> Initial (from Bind/Calibrate)\n");
        }
        printf("Current state: %s\n", stateToStr(stateMachine.getState()));

        // логика для Scan
        if (stateMachine.is(State::Initial) ||
            stateMachine.is(State::Manual) ||
            stateMachine.is(State::GiroScope))
        {
            if ((buttonsState.turn_left == BUTTON_ON && buttonsState.turn_right == BUTTON_ON) &&
                !flag_blocked_for_debounce)
            {

                stateMachine.setState(State::Scan);
              
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 250); // stop servo
                HAL_Delay(100); // Задержка для стабилизации после остановки
                // --- Перемещение двигателя в last_scan_point перед ожиданием статуса F5 ---
                
               
                
                //int64_t target = encoderScanPoints.entry_scan_point;
                //printf("[FSM] Moving motor to last_scan_point: %lld\n", target);
                //uint8_t move_ok = MksServo_AbsoluteMotionByAxis_F5(&mksServo, &target, 3000);
                //if (!move_ok)
               // {
               //     printf("[FSM][ERROR] Failed to send AbsoluteMotionByAxis_F5 command!\n");
               // }
              // HAL_Delay(500);
              if (isSynchronized)
              {
                MksServo_AbsoluteMotionByAxis_F5(&mksServo, &encoderScanPoints.entry_scan_point, 1000);
              }
              
                
                flag_blocked_for_debounce = 1;
                debounce_timer = HAL_GetTick();
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // Выключить лампочку
                nrf_send_long_beep();
                // Короткая пауза для надійності
                HAL_Delay(100); // Задержка 
                MksServo_CurrentAxisToZero_92(&mksServo); // Сброс текущей оси в ноль
                printf("[FSM] -> Scan (double_pedal pressed)\n");
            }
        }
        if (stateMachine.is(State::Scan))
        {
            if (mode_scan == ANGLE_SCAN)
            {
                // Обычная логика Scan (ничего не меняем)
                // Пример: обновление скорости от потенциометра
                last_speed = getPotentiometerValuePercentage();
            }
            else if (mode_scan == ANGLE_ADJUST)
            {
                // Скорость не обновляется, используем last_speed
                // Угол осцилляции вычисляется по потенциометру
                int pot_percent = getPotentiometerValuePercentage();
                int angle = (pot_percent * 360) / 100;

                // Здесь используйте angle для вашей логики осцилляции
                // Например:
                printf("[FSM] ANGLE_ADJUST: angle=%d deg, speed=%d\n", angle, last_speed);
                // Управление сервоприводом или другой логикой — по вашему проекту
            }
            if (!flag_blocked_for_debounce && (buttonsState.turn_left == BUTTON_ON || buttonsState.turn_right == BUTTON_ON))
            {
                // Очищаем домашнюю позицию при выходе из Scan
                // stateMachine.clearHomePosition();
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                mode_scan = ANGLE_SCAN;
                stateMachine.setState(State::Manual);
                nrgf_send_angle_agiust_exit();                  // Выходим из режима ANGLE_ADJUST
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 250); // stop servo
                printf("[FSM] -> Manual (pedal released)\n");
                HAL_Delay(100); // Задержка для предотвращения дребезга
            }
        }
    }

    // Сброс блокировки для антидребезга во время
    // отпускания  педалей при заходе в Scan через 0.8 сек
    if (flag_blocked_for_debounce)
    {
        if (HAL_GetTick() - debounce_timer > 800)
        {
            flag_blocked_for_debounce = 0;
        }
    }
  
}

void StateMachine_setup(void)
{

    stateMachine.setState(State::Initial);
    buttonsDebounceInit();
}