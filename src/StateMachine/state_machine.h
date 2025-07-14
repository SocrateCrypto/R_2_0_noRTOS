#pragma once
#include <stdint.h>

// Enum для установки угла
enum AngleSetting {
    ANGLE_SCAN,
    ANGLE_ADJUST
};

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
    
    // Методы для работы с домашней позицией энкодера
    void setHomePosition(int32_t carry, uint16_t value);
    void getHomePosition(int32_t* carry, uint16_t* value) const;
    bool hasHomePosition() const;
    void clearHomePosition();

private:
    State currentState;
    
    // Домашняя позиция энкодера (сохраняется при входе в Scan)
    int32_t homeCarry;
    uint16_t homeValue;
    bool homePositionSet;
};
// Структура для хранения показаний энкодера (дополнительное значение)
typedef struct
{
    int64_t last_scan_point;  // Последняя точка сканирования
    int64_t entry_scan_point; // Точка входа в сканирование
    int64_t giro_point;  // Точка входа в режим гироскопа
    float cumulativeYaw; // Накопленный угол поворота
    bool flag_first_run; // Флаг первого запуска педали
} EncoderScanPoints;

extern StateMachine stateMachine; // Экземпляр stateMachine
extern AngleSetting mode_scan; // Глобальная переменная для установки угла
extern EncoderScanPoints encoderScanPoints; // Глобальная переменная для хранения точек сканирования
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