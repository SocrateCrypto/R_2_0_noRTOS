#pragma once
// Функция для нормализации угла в диапазон от 0 до 360
#include <cstdint>
#include <cmath>
float normalizeAngle(float angle, float multi)
{
    while (compareFloat(angle, 0.0f) == -1)
        angle += 360 * multi;
    while (compareFloat(angle, 360.0f) == 1 || compareFloat(angle, 0) == 360.0f)
        angle -= 360 * multi;
    return angle;
}

float realAngle(uint64_t angle)
{
    return (angle/ 19.2) * (-1);
    return angle;
}



int8_t compareFloat(float a, float b)
{
    const float eps = 0.001;
    if (fabsf(a - b) > eps) {
        if ((a - b) > eps) {
            return 1;
        } // a>b
        if ((a - b) < eps) {
            return -1;
        } // a<b
    }
    return 0;
}