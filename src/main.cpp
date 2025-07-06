/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "StateMachine/state_machine.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MksServo/MksDriver_allinone.h"
#include "MksServo/Motor_position_state_structure.h"
#include "Buttons/buttons.h"
#include "NRF/nrf.h"
#include "EEPROM/flash_storage.h"
#include "UserConfig/user_config.h"
#include "fd_status.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "NRF/NRF24.h"
#include "NRF/NRF24_conf.h"
#include "NRF/NRF24_reg_addresses.h"
#include "NRF/nrf.h"
#include "Potentiometr/Potentiometer.h"

  // Объявление внешней переменной irq
  extern volatile uint8_t irq;

  // Прототип фонового парсера UART3 для вывода только 5-байтовых пакетов статуса
  // ВАЖНО: Не включайте этот прототип в main.h, чтобы избежать ошибок компиляции в других C-файлах!
  void MksServo_BackgroundPacketDebug(MksServo_t *servo);

#ifdef __cplusplus
}
#endif

#ifndef isfinite
#define isfinite(x) ((x) == (x) && (x) != INFINITY && (x) != -INFINITY)
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_FULL_ASSERT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Глобальная переменная для MKS Servo
// Глобальный буфер для приема
uint8_t uart3_rx_byte;
MksServo_t mksServo;

// Определяем переменные для NRF24
#define PLD_S 32 // payload size should be equal to transmitter

// Флаг для включения подробного режима отладки (закомментировать для отключения)
// #define VERBOSE_DEBUG

// Определение типа состояния сканирования для режима Scan
typedef enum
{
  SCAN_INIT,        // Инициализация
  SCAN_MOVING_LEFT, // Движение влево (-angle/2)
  SCAN_MOVING_RIGHT // Движение вправо (+angle/2)
} ScanState;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FD_status_t last_fd_status = {0};
int last_fd_status_printed = -1; // Для контроля вывода смены статуса
#define CMD_CRC_HISTORY 3
uint8_t last_cmd_crc_history[CMD_CRC_HISTORY] = {0};
uint8_t last_cmd_crc_index = 0;
// Глобальная переменная для CRC последней отправленной команды (движение/стоп)
uint8_t last_cmd_crc = 0;
// Флаг для Scan-режима: разрешено ли отправлять команду движения
int scan_ready_to_move = 1;
// Глобальная переменная для угла сканирования
int angle_of_scan = 180;
// Глобальный флаг успешного обнуления координат
int scan_zeroing_done = 0;
// Глобальная переменная для передаточного числа редуктора (можно менять)
float gear_ratio = 19.2f;

// --- Новая функция пересчёта угла в шаги с учётом редуктора и микрошагов ---
// steps_per_rev = 200 (шагов на оборот двигателя)
// microsteps = 32 (микрошагов)
// gear_ratio = 19.2 (редуктор)
// steps_for_360 = 200 * 32 * 19.2 = 122880
int32_t angle_to_steps(float angle_deg)
{
  const float steps_per_rev = 200.0f;
  const float microsteps = 32.0f;
  const float gear_ratio = 19.2f;
  const float steps_for_360 = steps_per_rev * microsteps * gear_ratio;
  return (int32_t)(steps_for_360 * angle_deg / 360.0f);
}

// --- Функция вычисления CRC для команд SERVO42D/57D (сумма всех байт кроме CRC) ---
uint8_t calc_crc(const uint8_t *data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len - 1; ++i)
  {
    crc += data[i];
  }
  return crc;
}

// --- Переводит угол в градусах в тики энкодера (0x4000 тиков = 360°) с учетом редуктора ---
int32_t angle_deg_to_encoder_ticks(float angle_deg) {
    return (int32_t)(angle_deg * ENCODER_PULSES_PER_360_DEGREE * MOTOR_GEAR_RATIO / 360.0f);
}

// --- ENUM-ы для статусов пакетов SERVO42D/57D ---
typedef enum
{
  FD_STATUS_STOP_FAIL = 0,
  FD_STATUS_STARTING = 1,
  FD_STATUS_RUN_COMPLETE = 2,
  FD_STATUS_UNKNOWN = 255
} FD_StatusEnum;

typedef enum
{
  FE_STATUS_RUN_FAIL = 0,
  FE_STATUS_RUN_STARTING = 1,
  FE_STATUS_RUN_COMPLETE = 2,
  FE_STATUS_END_LIMIT_STOPPED = 3,
  FE_STATUS_UNKNOWN = 255
} FE_StatusEnum;

typedef enum
{
  ZERO_STATUS_FAIL = 0,
  ZERO_STATUS_SUCCESS = 1,
  ZERO_STATUS_UNKNOWN = 255
} Zero_StatusEnum;

// --- Глобальные переменные для хранения последних статусов ---
FD_StatusEnum last_fd_status_enum = FD_STATUS_UNKNOWN;
FE_StatusEnum last_fe_status_enum = FE_STATUS_UNKNOWN;
Zero_StatusEnum last_zero_status_enum = ZERO_STATUS_UNKNOWN;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  StateMachine_setup(); // Инициализация машины состояний

  printf("NRF24 RX Test Started\r\n");
  printf("UART1 printf redirection working!\r\n");

  // Инициализация Flash памяти
  printf("Initializing Flash storage...\r\n");
  if (FlashStorage_Init() == HAL_OK)
  {
    printf("Flash storage initialized successfully\r\n");

    // Выводим информацию о сохраненном адресе пульта
    FlashStorage_PrintStoredAddress();

    // Если есть сохраненный адрес пульта, загружаем его в NRF24
    if (FlashStorage_HasValidRemoteAddress())
    {
      uint8_t stored_address[5];
      if (FlashStorage_LoadRemoteAddress(stored_address) == HAL_OK)
      {
        printf("[NRF] Loading stored remote address for NRF24: 0x%02X%02X%02X%02X%02X\r\n",
               stored_address[0], stored_address[1], stored_address[2], stored_address[3], stored_address[4]);
        // Устанавливаем загруженный адрес как рабочий для NRF24
        nrf_set_working_address(stored_address);
      }
    }
    else
    {
      printf("[NRF] No remote address stored, device needs pairing\r\n");

      // ВРЕМЕННО: очищаем Flash при обнаружении некорректных данных
      // Это нужно для перехода на новый алгоритм контрольной суммы
      printf("[NRF] Clearing Flash due to checksum algorithm update...\r\n");
      FlashStorage_ForceErase();
    }
  }
  else
  {
    printf("Flash storage initialization failed\r\n");
  }

  // Инициализация NRF24
  ce_low();       // Сначала CE в LOW
  HAL_Delay(100); // Увеличиваем задержку для стабилизации
  csn_high();     // Затем CSN в HIGH
  HAL_Delay(100); // Еще подождем

  nrf24_init();
  nrf_init_next(); // Инициализация NRF24 с передачей irq

  // Тест UART

  printf("UART1 Printf Test: %d\r\n", 123);
  setvbuf(stdout, NULL, _IONBF, 0); // Отключить буферизацию
  /* USER CODE END 2 */
  
  MksServo_Init(&mksServo, &huart3, RS485_DERE_GPIO_Port, RS485_DERE_Pin, 1);
  MksServo_SetMicrostep(&mksServo, 0x05); // Установка микрошагов в 16
HAL_Delay(2000); // Пауза после инициализации
  // Устанавливаем режим работы сервопривода по умолчанию
  uint8_t servo_mode = 4; // Режим SR _CLOSE
  if (MksServo_SetWorkMode(&mksServo, servo_mode))
  {
    printf("[MKS] Work mode set to default: %d\r\n", servo_mode);
  }
  else
  {
    printf("[MKS] Failed to set work mode\r\n");
  }
  Motor motor; // Инициализация глобальной переменной motor

  // Инициализируем значения по умолчанию
  motor.oscillation_angle = 180; // Значение по умолчанию на случай ошибки загрузки

  // Загружаем угол размаха из Flash памяти
  int16_t motor_angle;
  if (FlashStorage_LoadOscillationAngle(&motor_angle) == HAL_OK)
  {
    motor.oscillation_angle = motor_angle;
    printf("Motor angle loaded from Flash: %d steps\r\n", motor.oscillation_angle);
  }
  else
  {
    printf("Failed to load motor angle from Flash, using default: %d steps\r\n", motor.oscillation_angle);
  }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    nrf_loop(irq);
    MksServo_BackgroundPacketDebug(&mksServo); // Фоновый парсер UART3

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    static bool flag_first_run = false; // Флаг для первого запуска педали

    State current_state = stateMachine.getState();

    StateMachine_loop();

    // Условный выбор действий по текущему состоянию
    switch (current_state)
    {
    case State::Initial:
      // TODO: действия для Initial
      break;
    case State::Manual:
      // Manual режим с улучшенным антидребезгом
      {
        static uint32_t last_pot_tick = 0;
        static uint8_t cached_pot_percent = 0;
        int32_t carry = 0;
        uint16_t value = 0;
        uint32_t now = HAL_GetTick();
        
        // Обновляем кэшированное значение потенциометра каждые 100 мс
        if (now - last_pot_tick >= 100) {
          last_pot_tick = now;
          cached_pot_percent = getPotentiometerValuePercentage();
        }
        
        // Используем улучшенный антидребезг для всех событий (мгновенная реакция + защита от дребезга)
        if (buttonsState.turn_left == BUTTON_ON) // Если левая педаль нажата
        {
          flag_first_run = true;
          MksServo_GetCarry(&mksServo, &carry, &value, 100);
          MksServo_SpeedModeRun(&mksServo, 0x01, (cached_pot_percent*5+50), 250);
          printf("[MKS] Servo running left\r\n");
        }
        else if (buttonsState.turn_right == BUTTON_ON) // Если правая педаль нажата
        {
          flag_first_run = true;
          MksServo_SpeedModeRun(&mksServo, 0x00, (cached_pot_percent*5+50), 250);
          printf("[MKS] Servo running right\r\n");
        }
        else if ((buttonsState.turn_left == BUTTON_OFF && buttonsState.turn_right == BUTTON_OFF) && flag_first_run)
        {
          // Обе педали отпущены - останавливаем двигатель
          flag_first_run = false;  
          MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0); // stop servo
          
          printf("[MKS] Servo stopped (antibouce)\r\n");
        }
      }
      break;
    case State::GiroScope:

      break;
    case State::Scan: {
      // --- SCAN SWEEP LOGIC (возвращаем GetAdditionValue + анализируем проблему с остановкой) ---
      static int scan_initialized = 0;
      static int scan_direction = 1; // 1 = вправо, 0 = влево
      static int scan_state = 0; // 0 - движение, 1 - стоп/пауза
      static uint32_t stop_time = 0;
      static int32_t scan_limit_ticks = 0; // граница сканирования
      static uint32_t last_carry_poll = 0;
      static uint32_t last_speed_update = 0; // для периодического обновления скорости
      
      if (!scan_initialized) {
        scan_direction = 1;
        scan_state = 0;
        stop_time = 0;
        
        // Пересчитываем пределы сканирования из motor.oscillation_angle (в градусах) в тики энкодера
        scan_limit_ticks = angle_deg_to_encoder_ticks((float)motor.oscillation_angle) / 2;
        
        int64_t addition_init = 0;
        if (MksServo_GetAdditionValue(&mksServo, &addition_init, 100)) {
            printf("[SCAN][DEBUG] Initial addition = %" PRId64 "\n", addition_init);
        } else {
            printf("[SCAN][DEBUG] Initial addition: GetAdditionValue failed\n");
        }
        printf("[SCAN][DEBUG] motor.oscillation_angle = %d\n", motor.oscillation_angle);
        printf("[SCAN][DEBUG] scan_limit_ticks = %ld\n", (long)scan_limit_ticks);
        
        scan_initialized = 1;
        
        // Запускаем движение вправо
        uint8_t scan_pot_percent = getPotentiometerValuePercentage();
        int scan_speed = scan_pot_percent*5+50;
        MksServo_SpeedModeRun(&mksServo, 1, scan_speed, 250);
        printf("[SCAN] Start right, limit=%ld (encoder ticks), speed=%d\n", (long)scan_limit_ticks, scan_speed);
        last_speed_update = HAL_GetTick(); // Инициализируем таймер
      }
      
      // Периодически опрашиваем координаты (раз в 20 мс)
      uint32_t now = HAL_GetTick();
      
      if (now - last_carry_poll >= 20) {
        last_carry_poll = now;
        int64_t addition = 0;
        if (MksServo_GetAdditionValue(&mksServo, &addition, 100)) {
          // Абсолютная позиция в тиках: addition value (int64_t)
          if (scan_state == 0) { // Движение
            printf("[SCAN][TRACE] addition=%" PRId64 ", limit=+-%ld, dir=%d\n", addition, (long)scan_limit_ticks, scan_direction);
            
            // Проверяем достижение границы
            int boundary_reached = 0;
            if (scan_direction == 1 && addition >= scan_limit_ticks) {
              boundary_reached = 1;
            } else if (scan_direction == 0 && addition <= -scan_limit_ticks) {
              boundary_reached = 1;
            }
            
            if (boundary_reached) {
              // Достигли границы — стоп
              MksServo_SpeedModeRun(&mksServo, scan_direction, 0, 250);
              printf("[SCAN] Stop at %" PRId64 " (limit=+-%ld)\n", addition, (long)scan_limit_ticks);
              scan_state = 1;
              stop_time = now;
            } else {
              // ПРОБЛЕМА БЫЛА ЗДЕСЬ: обновление скорости во время движения конфликтует с остановкой
              // Добавим проверку что мы не слишком близко к границе перед обновлением скорости
              int64_t distance_to_boundary;
              if (scan_direction == 1) {
                distance_to_boundary = scan_limit_ticks - addition;
              } else {
                distance_to_boundary = addition - (-scan_limit_ticks);
              }
              
              // Обновляем скорость только если далеко от границы (больше 10% от лимита)
              if (distance_to_boundary > (scan_limit_ticks / 10) && (now - last_speed_update >= 100)) {
                last_speed_update = now;
                uint8_t scan_pot_percent = getPotentiometerValuePercentage();
                int scan_speed = scan_pot_percent*5+50;
                MksServo_SpeedModeRun(&mksServo, scan_direction, scan_speed, 250);
                printf("[SCAN] Speed updated: dir=%d, speed=%d, dist_to_boundary=%" PRId64 "\n", 
                       scan_direction, scan_speed, distance_to_boundary);
              }
            }
          } else if (scan_state == 1) { // Стоп/пауза
            if (now - stop_time > 100) { // 100 мс пауза для надежности
              // Меняем направление
              scan_direction = !scan_direction;
              uint8_t scan_pot_percent = getPotentiometerValuePercentage();
              int scan_speed = scan_pot_percent*5+50;
              MksServo_SpeedModeRun(&mksServo, scan_direction, scan_speed, 250);
              printf("[SCAN] Reverse, dir=%d, addition=%" PRId64 ", speed=%d\n", scan_direction, addition, scan_speed);
              scan_state = 0;
              last_speed_update = now; // Сбрасываем таймер обновления скорости
            }
          }
        } else {
          printf("[SCAN][ERROR] GetAdditionValue failed\n");
        }
      }
      break;
    }
    case State::BindMode:
      // TODO: действия для BindMode
      break;
    case State::Calibrate:
      // TODO: действия для Calibrate
      break;
    case State::CalibrateAndBind:
      // TODO: действия для CalibrateAndBind
      break;
    case State::AngleAdjust:
      // TODO: действия для AngleAdjust
      break;
    default:
      // TODO: обработка неизвестного состояния
      break;
    }
    HAL_Delay(1); // Задержка перед выходом из цикла
  } // <-- Закрываем while(1)
} // <-- Закрываем main

/**
   * @brief System Clock Configuration
   * @retval None
   */
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /**
   * @brief ADC1 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_ADC1_Init(void)
  {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
  }

  /**
   * @brief I2C1 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_I2C1_Init(void)
  {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
  }

  /**
   * @brief SPI2 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_SPI2_Init(void)
  {

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
  }

  /**
   * @brief USART1 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_USART1_UART_Init(void)
  {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
  }

  /**
   * @brief USART2 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_USART2_UART_Init(void)
  {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
  }

  /**
   * @brief USART3 Initialization Function
   * @param None
   * @retval None
   */
  static void MX_USART3_UART_Init(void)
  {

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 38400;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
  }

  /**
   * @brief GPIO Initialization Function
   * @param None
   * @retval None
   */
  static void MX_GPIO_Init(void)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_Pin | LED2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LAMP_Pin | IMU_EN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, RS485_DERE_Pin | CSN_Pin | CE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_Pin LED2_Pin */
    GPIO_InitStruct.Pin = LED_Pin | LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : LAMP_Pin IMU_EN_Pin */
    GPIO_InitStruct.Pin = LAMP_Pin | IMU_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : CALL_Pin COMP_Pin BIND_Pin */
    GPIO_InitStruct.Pin = CALL_Pin | COMP_Pin | BIND_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LEFT_Pin RIGHT_Pin */
    GPIO_InitStruct.Pin = LEFT_Pin | RIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : IRQ_Pin */
    GPIO_InitStruct.Pin = IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // ИСПРАВЛЕНИЕ: убираем pull-up, т.к. IRQ уже подтянут на плате
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Enable and set EXTI Line interrupt */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /*Configure GPIO pins : RS485_DERE_Pin CSN_Pin CE_Pin */
    GPIO_InitStruct.Pin = RS485_DERE_Pin | CSN_Pin | CE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; // ИСПРАВЛЕНИЕ: добавляем pull-down для DERE
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
  }

/* USER CODE BEGIN 4 */
#ifdef __cplusplus
  extern "C"
  {
#endif

    // Глобальная переменная для сигнализации о завершении приема UART3
    // Callback функции для работы с UART прерываниями

    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
      // Callback при ошибке UART
      // Ничего дополнительного делать не нужно, HAL сам управляет состоянием
    }

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
      if (huart == &huart3)
      {
        // Здесь обработайте принятый байт: uart3_rx_byte
        // Например, положите в свой буфер или обработайте сразу
        MksServo_UART_IRQHandler(&mksServo, huart);

        // Перезапустить прием следующего байта
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
      }
    }

    int __io_putchar(int ch)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
    }

    int _write(int file, char *ptr, int len)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
      return len;
    }

#ifdef __cplusplus
  }
#endif
  /* USER CODE END 4 */

  /**
   * @brief  Period elapsed callback in non blocking mode
   * @note   This function is called  when TIM3 interrupt took place, inside
   * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
   * a global variable "uwTick" used as application time base.
   * @param  htim : TIM handle
   * @retval None
   */
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
  {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM3)
    {
      HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
  }

  /**
   * @brief  This function is executed in case of error occurrence.
   * @retval None
   */
  void Error_Handler(void)
  {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
  }

#ifdef USE_FULL_ASSERT
  /**
   * @brief  Reports the name of the source file and the source line number
   *         where the assert_param error has occurred.
   * @param  file: pointer to the source file name
   * @param  line: assert_param error line source number
   * @retval None
   */
  void assert_failed(uint8_t *file, uint32_t line)
  {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
    /* USER CODE END 6 */
  }
#endif /* USE_FULL_ASSERT */

// --- Фоновый парсер UART3 для вывода только 5-байтовых пакетов статуса и обработки 0x92 ---
// Теперь парсер не использует CRC из ответа, а только last_cmd_crc, который обновляется при отправке команды
#ifdef __cplusplus
  extern "C"
  {
#endif
    State stateMachine_getState();
#ifdef __cplusplus
  }
#endif

  void MksServo_BackgroundPacketDebug(MksServo_t * servo)
  {
    static uint8_t packet[16]; // запас по длине
    static uint8_t idx = 0;
    static uint8_t collecting = 0;
    uint8_t byte;
    extern int scan_ready_to_move;  // флаг для Scan-режима
    static int zeroing_success = 0; // 1 - успешно обнулили
    while (MksServo_RxGetByte(servo, &byte))
    {
      printf("[UART3][RAW] RX byte: %02X\n", byte);
      if (!collecting)
      {
        if (byte == 0xFB)
        {
          packet[0] = byte;
          idx = 1;
          collecting = 1;
        }
      }
      else
      {
        packet[idx++] = byte;
        if (idx == 3)
        {
          // На третьем байте (packet[2]) определяем тип пакета
          switch (packet[2])
          {
          case 0xFD: // Статусный пакет
            // Ждем 5 байт
            break;
          case 0x92: // Ответ на Set Current Axis to Zero
            // Ждем 5 байт
            break;
          case 0xFE: // Ответ на AbsoluteMotionByPulse_FE
            // Ждем 5 байт
            break;
          default:
            // Пока не поддерживаем другие типы
            collecting = 0;
            idx = 0;
            continue;
          }
        }
        // Для статусного пакета: FB xx FD ... ...
        if (packet[2] == 0xFD && idx == 5)
        {
          FD_status_t fd;
          fd.head = packet[0];
          fd.addr = packet[1];
          fd.func = packet[2];
          fd.status = packet[3];
          fd.crc = packet[4];
          extern uint8_t last_cmd_crc;
          // --- Обновляем глобальный enum статуса ---
          switch (fd.status)
          {
          case 0:
            last_fd_status_enum = FD_STATUS_STOP_FAIL;
            break;
          case 1:
            last_fd_status_enum = FD_STATUS_STARTING;
            break;
          case 2:
            last_fd_status_enum = FD_STATUS_RUN_COMPLETE;
            break;
          default:
            last_fd_status_enum = FD_STATUS_UNKNOWN;
            break;
          }
          if ((int)fd.status != last_fd_status_printed)
          {
            printf("[UART3][FD] status: %u - ", fd.status);
            if (last_cmd_crc == 0xFA)
            {
              printf("STOP (last_cmd_crc=0xFA)\n");
            }
            else
            {
              printf("RUN (last_cmd_crc=0x%02X)\n", last_cmd_crc);
            }
            switch (fd.status)
            {
            case 0:
              printf("stop the motor fail\n");
              break;
            case 1:
              if (last_cmd_crc == 0xFA)
              {
                printf("stop the motor starting...\n");
              }
              else
              {
                printf("run starting...\n");
              }
              break;
            case 2:
              if (stateMachine_getState() == State::Scan)
              {
                scan_ready_to_move = 1;
                printf("[SCAN][FSM] scan_ready_to_move=1 (run complete)\n");
              }
              if (last_cmd_crc == 0xFA)
              {
                printf("stop the motor complete\n");
                scan_ready_to_move = 1;
              }
              else
              {
                printf("run complete\n");
              }
              break;
            default:
              printf("unknown status\n");
              break;
            }
            last_fd_status_printed = fd.status;
          }
          last_fd_status = fd;
          printf("[UART3][PKT] ");
          for (int i = 0; i < 5; ++i)
            printf("%02X ", packet[i]);
          printf("\n");
          collecting = 0;
          idx = 0;
        }
        // Для пакета 0x92: FB xx 92 status crc
        if (packet[2] == 0x92 && idx == 5)
        {
          uint8_t status = packet[3];
          // --- Обновляем глобальный enum статуса ---
          switch (status)
          {
          case 0:
            last_zero_status_enum = ZERO_STATUS_FAIL;
            break;
          case 1:
            last_zero_status_enum = ZERO_STATUS_SUCCESS;
            break;
          default:
            last_zero_status_enum = ZERO_STATUS_UNKNOWN;
            break;
          }
          printf("[UART3][ZERO] status: %u - ", status);
          if (status == 1)
          {
            printf("set success\n");
            zeroing_success = 1;
          }
          else
          {
            printf("set fail\n");
          }
          printf("[UART3][PKT] ");
          for (int i = 0; i < 5; ++i)
            printf("%02X ", packet[i]);
          printf("\n");
          collecting = 0;
          idx = 0;
        }
        // Для пакета 0xFE: FB xx FE status crc
        if (packet[2] == 0xFE && idx == 5)
        {
          uint8_t status = packet[3];
          // --- Обновляем глобальный enum статуса ---
          switch (status)
          {
          case 0:
            last_fe_status_enum = FE_STATUS_RUN_FAIL;
            break;
          case 1:
            last_fe_status_enum = FE_STATUS_RUN_STARTING;
            break;
          case 2:
            last_fe_status_enum = FE_STATUS_RUN_COMPLETE;
            break;
          case 3:
            last_fe_status_enum = FE_STATUS_END_LIMIT_STOPPED;
            break;
          default:
            last_fe_status_enum = FE_STATUS_UNKNOWN;
            break;
          }
          printf("[UART3][FE] status: %u - ", status);
          switch (status)
          {
          case 0:
            printf("run fail\n");
            break;
          case 1:
            printf("run starting...\n");
            break;
          case 2:
            printf("run complete\n");
            break;
          case 3:
            printf("end limit stopped\n");
            break;
          default:
            printf("unknown status\n");
            break;
          }
          printf("[UART3][PKT] ");
          for (int i = 0; i < 5; ++i)
            printf("%02X ", packet[i]);
          printf("\n");
          collecting = 0;
          idx = 0;
        }
      }
    }
    // Обновляем глобальный флаг успешного обнуления
    if (zeroing_success)
    {
      scan_zeroing_done = 1;
      zeroing_success = 0;
    }
  }

#ifdef __cplusplus
  extern "C"
  {
#endif
    State stateMachine_getState()
    {
      return stateMachine.getState();
    }
#ifdef __cplusplus
  }
#endif

