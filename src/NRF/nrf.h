#pragma once
#include <stdint.h> // Include the header for uint8_t

// Структура для хранения состояний виртуальных радиокнопок
typedef struct
{
    uint8_t left : 1;           // Одиночное нажатие влево
    uint8_t right : 1;          // Одиночное нажатие вправо
    uint8_t left_end_right : 1; // Одновременное нажатие обеих кнопок
    uint8_t both : 1;          // Обе кнопки
    uint8_t reserved : 3;      // Зарезервировано
} RadioButtons;

// Глобальная переменная для состояний виртуальных кнопок
extern RadioButtons RadioButtonsStates;

// Функции для работы с радиомодулем
void nrf_loop(uint8_t irq);
void nrf_init_next(void);
void update_radio_buttons(const char* data);

// Функции привязки
void nrf_enter_binding_mode(void);
void nrf_exit_binding_mode(void);
uint8_t nrf_is_binding_mode(void);
void nrf_send_bind_response(void);