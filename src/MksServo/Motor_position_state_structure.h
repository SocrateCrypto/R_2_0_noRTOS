#pragma once
#include <stdint.h>

typedef struct {
    int32_t position_from_encoder;      // Текущая позиция в шагах
    int32_t position;      // Текущая позиция в шагах
    int32_t home_position; // Домашняя позиция (нулевая)
    int32_t target;        // Целевая позиция
    int32_t speed;         // Текущая скорость
    int32_t max_speed;     // Максимальная скорость
    int16_t oscillation_angle;   // Угол размаха поворота (туда-сюда) в шагах
    int8_t direction;      // Направление движения: -1, 0, 1
    uint8_t is_moving;     // Флаг движения
    uint8_t is_stopped;    // Флаг остановки
} Motor;

extern Motor motor;

