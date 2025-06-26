#include "Potentiometer.h"
#include "stm32f1xx_hal.h" // или ваш HAL header

#define ADC_MAX_VALUE 4095 // 12 бит АЦП

extern ADC_HandleTypeDef hadc1; // Объявите ваш ADC handler, если он называется иначе — исправьте

int16_t getPotentiometerValuePercentage(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_7; // Для PA7 на STM32F103
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // Переводим в проценты
    int16_t percent = (int16_t)((value * 100) / ADC_MAX_VALUE);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}