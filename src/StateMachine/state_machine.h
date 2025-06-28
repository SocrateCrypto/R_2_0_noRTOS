#pragma once
#include <stdint.h>

enum class State
{
    Initial,//початковий стан, електричне утримання ротора двигуна вимнено
    Manual,
    GiroScope,
    Calibrate,
    Scan,
    BindMode,
    CalibrateAndBind,
    AngleAdjust
};

class StateMachine
{
public:
    StateMachine();

    State getState() const;
    void setState(State newState);
    bool is(State state) const;

private:
    State currentState;
};
extern StateMachine stateMachine; // Экземпляр stateMachine

void StateMachine_loop(void);
void StateMachine_setup(void);

#ifdef __cplusplus
extern "C" {
#endif
extern uint8_t scan_blocked;
extern uint32_t scan_blocked_timer;
#ifdef __cplusplus
}
#endif