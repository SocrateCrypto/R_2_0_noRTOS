#pragma once

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