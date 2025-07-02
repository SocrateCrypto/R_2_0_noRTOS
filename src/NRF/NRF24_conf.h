/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef _NRF_24_CONF_H_
#define _NRF_24_CONF_H_

#include "main.h"

#define NRF24_SPI hspi2
#define NRF24_SPI_TIMEOUT 1000

#define NRF24_CSN_PORT GPIOB
#define NRF24_CSN_PIN CSN_Pin

#define NRF24_CE_PORT GPIOB
#define NRF24_CE_PIN CE_Pin

extern SPI_HandleTypeDef hspi2;

#endif
