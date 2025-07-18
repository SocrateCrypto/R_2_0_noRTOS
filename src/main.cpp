/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Головний файл програми
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * Це програмне забезпечення ліцензовано згідно з умовами, які можна знайти у файлі LICENSE у кореневому каталозі цього програмного компонента.
 * Якщо файл LICENSE відсутній, програмне забезпечення надається "ЯК Є".
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
#include "bno055.h"
#include "bno055_stm32.h"
#include "pid/Pid.h"
#ifdef __cplusplus
extern "C"
{
#endif
#include "AHRS_algorithm/Fusion.h"
#include "NRF/NRF24.h"
#include "NRF/NRF24_conf.h"
#include "NRF/NRF24_reg_addresses.h"
#include "NRF/nrf.h"
#include "Potentiometr/Potentiometer.h"
#include "Float_transform/Float_transform.h"

  // Оголошення зовнішньої змінної irq
  extern volatile uint8_t irq;

  // Прототип фоновго парсера UART3 для виводу лише 5-байтових пакетів статусу
  // ВАЖЛИВО: Не включайте цей прототип у main.h, щоб уникнути помилок компіляції в інших C-файлах!
  void MksServo_BackgroundPacketDebug(MksServo_t *servo);

#ifdef __cplusplus
}
#endif

#ifndef isfinite
#define isfinite(x) ((x) == (x) && (x) != INFINITY && (x) != -INFINITY)
#endif

/* USER CODE END Includes */
/* Controller parameters */

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
// Глобальна змінна для MKS Servo
// Глобальний буфер для прийому
uint8_t uart3_rx_byte;
MksServo_t mksServo;

// Оголошення змінних для NRF24
#define PLD_S 32 // розмір корисного навантаження має відповідати передавачу

// Прапорець для увімкнення детального режиму відладки (закоментуйте для вимкнення)
// #define VERBOSE_DEBUG

// Оголошення типу стану сканування для режиму Scan
typedef enum
{
  SCAN_INIT,        // Ініціалізація
  SCAN_MOVING_LEFT, // Рух вліво (-angle/2)
  SCAN_MOVING_RIGHT // Рух вправо (+angle/2)
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
int last_fd_status_printed = -1; // Для контролю виводу зміни статусу
#define CMD_CRC_HISTORY 3
uint8_t last_cmd_crc_history[CMD_CRC_HISTORY] = {0};
uint8_t last_cmd_crc_index = 0;
// Глобальна змінна для CRC останньої відправленої команди (рух/стоп)
uint8_t last_cmd_crc = 0;
// Прапорець для Scan-режиму: чи дозволено відправляти команду руху
int scan_ready_to_move = 1;
// Глобальна змінна для кута сканування
int angle_of_scan = 180;
// Глобальний прапорець успішного обнулення координат
int scan_zeroing_done = 0;
// Глобальна змінна для передавального числа редуктора (можна змінювати)
float gear_ratio = 19.2f;

// --- Нова функція перерахунку кута в кроки з урахуванням редуктора та мікрокроків ---
// steps_per_rev = 200 (кроків на оберт двигуна)
// microsteps = 32 (мікрокроків)
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

// --- Функція обчислення CRC для команд SERVO42D/57D (сума всіх байт окрім CRC) ---
uint8_t calc_crc(const uint8_t *data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len - 1; ++i)
  {
    crc += data[i];
  }
  return crc;
}

// --- Перетворює кут у градусах у тики енкодера (0x4000 тиків = 360°) з урахуванням редуктора ---
int32_t angle_deg_to_encoder_ticks(float angle_deg)
{
  return (int32_t)(angle_deg * ENCODER_PULSES_PER_360_DEGREE * MOTOR_GEAR_RATIO / 360.0f);
}

// --- ENUM-и для статусів пакетів SERVO42D/57D ---
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

// --- Глобальні змінні для зберігання останніх статусів ---
FD_StatusEnum last_fd_status_enum = FD_STATUS_UNKNOWN;
FE_StatusEnum last_fe_status_enum = FE_STATUS_UNKNOWN;
Zero_StatusEnum last_zero_status_enum = ZERO_STATUS_UNKNOWN;
uint8_t last_f5_status = 255; // Глобальна змінна для статусу F5
uint8_t last_f6_status = 0;
void reset_last_f6_status(void) { last_f6_status = 0; }

// === DWT (Data Watchpoint and Trace) for microsecond timing ===
static inline void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint64_t micros(void)
{
  return (uint32_t)(DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000));
}
// ===================================================================
/* USER CODE END 0 */
// Функция для нормалізації угла в діапазоні [−180, 180] градусів
float normalizeAngle(float angle)
{
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;
  return angle;
}
/**
 * @brief  Точка входу в додаток.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float previousYaw = 0;   // Предыдущее значение yaw
  float cumulativeYaw = 0; // Кумулятивное значение yaw
  float currentYaw = 0;    // Текущее значение yaw с датчика IMU
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Скидання всіх периферійних пристроїв, ініціалізація Flash та Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Налаштування системного тактового генератора */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Ініціалізація всіх налаштованих периферійних пристроїв */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  StateMachine_setup(); // Ініціалізація машини станів

  // ================= DWT microsecond timer init =================
  DWT_Init();
  // =============================================================

  printf("NRF24 RX Test Started\r\n");
  printf("UART1 printf redirection working!\r\n");

  // Ініціалізація Flash пам'яті
  printf("Initializing Flash storage...\r\n");
  if (FlashStorage_Init() == HAL_OK)
  {
    printf("Flash storage initialized successfully\r\n");

    // Виводимо інформацію про збережену адресу пульта
    FlashStorage_PrintStoredAddress();

    // Якщо є збережена адреса пульта, завантажуємо її в NRF24
    if (FlashStorage_HasValidRemoteAddress())
    {
      uint8_t stored_address[5];
      if (FlashStorage_LoadRemoteAddress(stored_address) == HAL_OK)
      {
        printf("[NRF] Loading stored remote address for NRF24: 0x%02X%02X%02X%02X%02X\r\n",
               stored_address[0], stored_address[1], stored_address[2], stored_address[3], stored_address[4]);
        // Встановлюємо завантажену адресу як робочу для NRF24
        nrf_set_working_address(stored_address);
      }
    }
    else
    {
      printf("[NRF] No remote address stored, device needs pairing\r\n");

      // ТИМЧАСОВО: очищаємо Flash при виявленні некоректних даних
      // Це потрібно для переходу на новий алгоритм контрольної суми
      printf("[NRF] Clearing Flash due to checksum algorithm update...\r\n");
      FlashStorage_ForceErase();
    }
  }
  else
  {
    printf("Flash storage initialization failed\r\n");
  }

  // Ініціалізація NRF24
  ce_low();       // Спочатку CE в LOW
  HAL_Delay(100); // Збільшуємо затримку для стабілізації
  csn_high();     // Потім CSN в HIGH
  HAL_Delay(100); // Ще трохи чекаємо

  nrf24_init();
  nrf_init_next(); // Ініціалізація NRF24 з передачею irq

  // Тест UART

  printf("UART1 Printf Test: %d\r\n", 123);
  setvbuf(stdout, NULL, _IONBF, 0); // Вимкнути буферизацію
  /* USER CODE END 2 */

  MksServo_Init(&mksServo, &huart3, RS485_DERE_GPIO_Port, RS485_DERE_Pin, 1);
  MksServo_SetMicrostep(&mksServo, 0x05); // Встановлення мікрокроків у 16
  HAL_Delay(2000);                        // Пауза після ініціалізації
  // Встановлюємо режим роботи сервоприводу за замовчуванням
  uint8_t servo_mode = 5; // Режим SR _CLOSE
  if (MksServo_SetWorkMode(&mksServo, servo_mode))
  {
    printf("[MKS] Work mode set to default: %d\r\n", servo_mode);
  }
  else
  {
    printf("[MKS] Failed to set work mode\r\n");
  }
  Motor motor; // Ініціалізація глобальної змінної motor

  // Ініціалізуємо значення за замовчуванням
  motor.oscillation_angle = 180; // Значення за замовчуванням на випадок помилки завантаження

  // Завантажуємо кут розмаху з Flash пам'яті
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
  HAL_Delay(100);
  HAL_GPIO_WritePin(IMU_EN_GPIO_Port, IMU_EN_Pin, GPIO_PIN_SET); // Увімкнення живлення IMU

  bno055_assignI2C(&hi2c1);
  HAL_Delay(100);

  bno055_enableExternalCrystal(); // Use external crystal if available
  HAL_Delay(30);
  bno055_setup();
  HAL_Delay(50);
  bno055_setOperationMode(BNO055_OPERATION_MODE_ACCGYRO);
  HAL_Delay(30);
  // === Настройка IMU: спочатку діапазони, потім смуги ===
  bno055_setAccelConfig(BNO055_ACCEL_RANGE_2G, 0b100); // 0x01 — фильтр 31.25 Гц (пример) // Максимальна чутливість акселерометра (±2g)
  HAL_Delay(30);
  bno055_setGyroConfig(BNO055_GYRO_RANGE_250DPS, 0b010); // 0x04 — фильтр 47 Гц (пример) // Максимальна чутливість гироскопа (±250 dps)
  HAL_Delay(30);
  // Можно раскомментировать и выставить фильтр, если нужно:
  // bno055_setAccelBandwidth(0x01); // 31.25 Гц, 1000 Гц обновления
  // HAL_Delay(30);
  // bno055_setGyroBandwidth(0x04); // 47 Гц, 200 Гц обновления
  // HAL_Delay(30);

  // === Контрольные чтения регистров після установки діапазонів ===
  uint8_t acc_config = 0, gyro_config = 0;
  bno055_setPage(1);
  bno055_readData(BNO055_ACC_CONFIG, &acc_config, 1);
  bno055_readData(BNO055_GYRO_CONFIG_0, &gyro_config, 1);
  bno055_setPage(0);
  printf("ACC_CONFIG after set: 0x%02X\n", acc_config);
  printf("GYRO_CONFIG_0 after set: 0x%02X\n", gyro_config);

  // === Переводимо IMU в робочий режим ===
  
  
  // bno055_enableGyroDataReadyInterrupt(); // Увімкнення переривання на нові дані гироскопа
  // HAL_Delay(50);
  // === Контрольні читання регістрів ===
  // Повторно використовуемо acc_config и gyro_config, не объявляем их заново
  bno055_setPage(1);
  bno055_readData(BNO055_ACC_CONFIG, &acc_config, 1);
  bno055_readData(BNO055_GYRO_CONFIG_0, &gyro_config, 1);
  bno055_setPage(0);
  printf("ACC_CONFIG: 0x%02X, GYRO_CONFIG_0: 0x%02X\r\n", acc_config, gyro_config);
  /*
  Расшифровка значений регистров:

ACC_CONFIG: 0xF8

Младшие 2 бита (0xF8 & 0x03 = 0): диапазон ±2g (это правильно для максимальной чувствительности).
Остальные биты — это настройки bandwidth и power mode, что нормально.
GYRO_CONFIG_0: 0x02

Младшие 3 бита (0x02 & 0x07 = 2): диапазон ±125 dps (это правильно для максимальной чувствительности).
Вывод:
Оба регистра установлены корректно:

Акселерометр: ±2g
Гироскоп: ±125 dps
  все в порядке, максимальная чувствительность установлена.

  */

  /* USER CODE BEGIN WHILE */

  // Define calibration (replace with actual calibration data if available)
  const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f}; // значения уже в dps
  const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
  const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f}; // значения уже в g
  const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
  // const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  // const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

  // Initialise algorithms
  FusionOffset offset;
  FusionAhrs ahrs;

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.5f,
      .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
      .accelerationRejection = 10.0f,
      .magneticRejection = 10.0f,
      .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
  };
  // FusionAhrsSetSettings(&ahrs, &settings);
  PIDController PID(PID_KP, PID_KI, PID_KD, PID_RAMP, PID_LIMIT);

  while (1)
  {

    nrf_loop(irq);
    MksServo_BackgroundPacketDebug(&mksServo); // Фоновий парсер UART3

    /* USER CODE END WHILE */
    // --- Считывание IMU с частотой 50 Гц (каждые 20 мс) ---
    static uint32_t last_imu_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    if (now_tick - last_imu_tick >= 9) // 5 мс = 200 Гц
    {
      last_imu_tick = now_tick;
      // Acquire latest sensor data

      static uint32_t call_count = 0;
      static uint32_t last_time = 0;
      call_count++;
      uint32_t now = HAL_GetTick();
      if (now - last_time >= 1000)
      { // раз в 1 секунду
        printf("Frequency: %lu Hz\n", call_count);
        call_count = 0;
        last_time = now;
      }
      // --- Получение и обработка данных IMУ ---
      FusionVector accelerometer, gyroscope;
      bno055_vector_t acc_vector = bno055_readRawAccelRegisters();
      accelerometer.axis.x = acc_vector.x / 1000;
      accelerometer.axis.y = acc_vector.y / 1000;
      accelerometer.axis.z = acc_vector.z / 1000;
      bno055_vector_t gyro_vector = bno055_readRawGyroRegisters();
      gyroscope.axis.x = gyro_vector.x / 128;
      gyroscope.axis.y = gyro_vector.y / 128;
      gyroscope.axis.z = gyro_vector.z / 128;
      // Update AHRS algorithm
      static uint32_t previousTick = 0;
      float deltaTime = 0.0f;
      uint32_t now_tick_dt = HAL_GetTick();
      if (previousTick != 0)
      {
        uint32_t diff = now_tick_dt - previousTick;
        deltaTime = diff/1000.0f;
      }
      previousTick = now_tick_dt;
      FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

      const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
      // --- Вывод значений euler через Float_transform раз в секунду ---
      static uint32_t lastPrintTick = 0;
      if (now_tick_dt - lastPrintTick >= 1000) // 1 секунда
      {
        lastPrintTick = now_tick_dt;
        uint8_t sign_roll, sign_pitch, sign_yaw;
        int int_roll, int_pitch, int_yaw;
        uint32_t frac_roll, frac_pitch, frac_yaw;
        Float_transform(euler.angle.roll, 3, &sign_roll, &int_roll, &frac_roll);
        Float_transform(euler.angle.pitch, 3, &sign_pitch, &int_pitch, &frac_pitch);
        Float_transform(euler.angle.yaw, 3, &sign_yaw, &int_yaw, &frac_yaw);
        printf("Yaw: %s%d.%03lu\n", sign_yaw ? "-" : "", int_yaw, frac_yaw);
        currentYaw = euler.angle.yaw;
        float deltaYaw = normalizeAngle(currentYaw - previousYaw);
        cumulativeYaw += deltaYaw;
        previousYaw = currentYaw;
        Float_transform(cumulativeYaw, 3, &sign_yaw, &int_yaw, &frac_yaw);
        printf("Com_Yaw: %s%d.%03lu\n", sign_yaw ? "-" : "", int_yaw, frac_yaw);
      }
      // --- Временный вывод сырых значений акселерометра і гироскопа через Float_transform ---
      static uint32_t lastPrintRawTick = 0;
      if (now_tick_dt - lastPrintRawTick >= 1000) // 1 секунда
      {
        lastPrintRawTick = now_tick_dt;
        uint8_t sign_ax, sign_ay, sign_az;
        int int_ax, int_ay, int_az;
        uint32_t frac_ax, frac_ay, frac_az;
        Float_transform(acc_vector.x, 3, &sign_ax, &int_ax, &frac_ax);
        Float_transform(acc_vector.y, 3, &sign_ay, &int_ay, &frac_ay);
        Float_transform(acc_vector.z, 3, &sign_az, &int_az, &frac_az);
        printf("[RAW ACC] X: %s%d.%03lu Y: %s%d.%03lu Z: %s%d.%03lu ", sign_ax ? "-" : "", int_ax, frac_ax, sign_ay ? "-" : "", int_ay, frac_ay, sign_az ? "-" : "", int_az, frac_az);
        uint8_t sign_gx, sign_gy, sign_gz;
        int int_gx, int_gy, int_gz;
        uint32_t frac_gx, frac_gy, frac_gz;
        Float_transform(gyro_vector.x, 3, &sign_gx, &int_gx, &frac_gx);
        Float_transform(gyro_vector.y, 3, &sign_gy, &int_gy, &frac_gy);
        Float_transform(gyro_vector.z, 3, &sign_gz, &int_gz, &frac_gz);
        printf("[RAW GYRO] X: %s%d.%03lu Y: %s%d.%03lu Z: %s%d.%03lu ", sign_gx ? "-" : "", int_gx, frac_gx, sign_gy ? "-" : "", int_gy, frac_gy, sign_gz ? "-" : "", int_gz, frac_gz);
        float acc_norm = sqrtf(acc_vector.x * acc_vector.x + acc_vector.y * acc_vector.y + acc_vector.z * acc_vector.z);
        uint8_t sign_accnorm;
        int int_accnorm;
        uint32_t frac_accnorm;
        Float_transform(acc_norm, 3, &sign_accnorm, &int_accnorm, &frac_accnorm);
        // printf("|acc|: %s%d.%03lu\n", sign_accnorm ? "-" : "", int_accnorm, frac_accnorm);
      }

      // --- Логика индикации LED2 по изменению cumulativeYaw ---
      static float yaw_ref = 0.0f;
      static uint8_t led2_on = 0;
      static uint32_t led2_on_time = 0;
      if (!led2_on && fabsf(cumulativeYaw - yaw_ref) >= 0.3f) {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        led2_on = 1;
        led2_on_time = HAL_GetTick();
        yaw_ref = cumulativeYaw;
      }
      if (led2_on && (HAL_GetTick() - led2_on_time >= 1000)) {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        led2_on = 0;
      }
    }
    /* USER CODE BEGIN 3 */
    static bool flag_first_run = false; // Прапорець для першого запуску педалі

    State current_state = stateMachine.getState();

    StateMachine_loop();

    // Умовний вибір дій за поточним станом
    switch (current_state)
    {
    case State::Initial:
      // TODO: дії для Initial
      break;
    case State::Manual:
      // Manual режим з покращеним антидребезгом
      {
        static uint32_t last_pot_tick = 0;
        static uint8_t cached_pot_percent = 0;

        uint32_t now = HAL_GetTick();

        // Оновлюємо кешоване значення потенціометра кожні 100 мс
        if (now - last_pot_tick >= 100)
        {
          last_pot_tick = now;
          cached_pot_percent = getPotentiometerValuePercentage();
        }

        // Використовуємо покращений антидребезг для всіх подій (миттєва реакція + захист від дребезгу)
        if (buttonsState.turn_left == BUTTON_ON) // Якщо ліва педаль натиснута
        {
          if (!flag_first_run)
          {
            int64_t add_val = 0;
            if (MksServo_GetAdditionValue(&mksServo, &add_val, 100))
            {
              encoderScanPoints.entry_scan_point = add_val;
              printf("[ENC] Entry scan point set: 0x%08lX%08lX\n", (uint32_t)((add_val >> 32) & 0xFFFFFFFF), (uint32_t)(add_val & 0xFFFFFFFF));
            }
            else
            {
              printf("[ENC] Failed to read entry scan point!\n");
            }
          }

          flag_first_run = true;
          MksServo_SpeedModeRun(&mksServo, 0x01, (cached_pot_percent * 6 + 50), 250);
          printf("[MKS] Servo running left\r\n");
        }
        else if (buttonsState.turn_right == BUTTON_ON) // Якщо права педаль натиснута
        {

          if (!flag_first_run)
          {
            int64_t add_val = 0;
            if (MksServo_GetAdditionValue(&mksServo, &add_val, 100))
            {
              encoderScanPoints.entry_scan_point = add_val;
              printf("[ENC] Entry scan point set: 0x%08lX%08lX\n", (uint32_t)((add_val >> 32) & 0xFFFFFFFF), (uint32_t)(add_val & 0xFFFFFFFF));
            }
            else
            {
              printf("[ENC] Failed to read entry scan point!\n");
            }
          }
          flag_first_run = true;
          MksServo_SpeedModeRun(&mksServo, 0x00, (cached_pot_percent * 6 + 50), 250);
          printf("[MKS] Servo running right\r\n");
        }
        else if ((buttonsState.turn_left == BUTTON_OFF && buttonsState.turn_right == BUTTON_OFF) && flag_first_run)
        {
          // Обидві педалі відпущені - зупиняємо двигун
          flag_first_run = false;
          MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0); // зупинка сервоприводу
          HAL_Delay(100);
          int64_t add_val = 0;
          // невелика затримка для стабільності
          if (MksServo_GetAdditionValue(&mksServo, &add_val, 100))
          {
            encoderScanPoints.entry_scan_point = add_val;
            printf("[ENC] Entry scan point set: 0x%08lX%08lX\n", (uint32_t)((add_val >> 32) & 0xFFFFFFFF), (uint32_t)(add_val & 0xFFFFFFFF));
          }
          else
          {
            printf("[ENC] Failed to read entry scan point!\n");
          }
          printf("[MKS] Servo stopped (antibouce)\r\n");
        }
      }
      break;
    case State::GiroScope:
    {
        
      
        if (encoderScanPoints.flag_first_run)
        {
            encoderScanPoints.flag_first_run = false;
            encoderScanPoints.cumulativeYaw = cumulativeYaw; // Сохраняем начальное значение кумулятивного yaw
            PID.reset();                                     // Сброс PID при первом запуске
            printf("[GYRO] First run: initial_motor_pos=0x%08lX%08lX, initial_yaw=%.2f\n",
                (uint32_t)((encoderScanPoints.giro_point >> 32) & 0xFFFFFFFF),
                (uint32_t)(encoderScanPoints.giro_point & 0xFFFFFFFF), cumulativeYaw);
        }
        int64_t initial_motor_pos = encoderScanPoints.giro_point;
        float initial_yaw = encoderScanPoints.cumulativeYaw;
        int64_t current_motor_pos = 0;
        static bool flag_for_blocking = false; // Прапорець для блокування логіки утриманняазимуту під час управління  кнопками
        
        
        
        
        if (!flag_for_blocking)
        {
            while (!MksServo_GetAdditionValue(&mksServo, &current_motor_pos, 100))
            {
                HAL_Delay(1); // Неблокуюча пауза для стабільності
                printf("[GYRO] error reed encoder: 0x%08lX%08lX\n",
                    (uint32_t)((current_motor_pos >> 32) & 0xFFFFFFFF),
                    (uint32_t)(current_motor_pos & 0xFFFFFFFF));
            }

            // Формула ошибки с учётом редуктора и направления
            #define ENCODER_DIR 1
            #define ENCODER_K 0.1f
            float encoder_delta = (float)(current_motor_pos - initial_motor_pos) * (360.0f / ENCODER_PULSES_PER_360_DEGREE) / MOTOR_GEAR_RATIO;
            float yaw_delta = (cumulativeYaw - initial_yaw);
            float error = (yaw_delta - encoder_delta) * ENCODER_DIR;

            float speed = 0; // PID.update(error);
            if (fabs(error) > 0.1f)
            {
                float candidate = error * 10.0f;
                if (fabs(candidate) > fabs(speed))
                {
                    speed = candidate;
                    printf("candidate speed\r\n");
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                    MksServo_SpeedModeRun(&mksServo, speed > 0 ? 1 : 0, fabs(speed), 254);
                }
            }

            static uint32_t last_speed_cmd_time = 0;
            uint32_t now_ = HAL_GetTick();
            if (now_ - last_speed_cmd_time >= 9)
            {
                last_speed_cmd_time = now_;
                float abs_speed = fabs(speed);
                if (abs_speed < 3.0f)
                {
                    abs_speed = 0.0f;
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                    
                }

                MksServo_SpeedModeRun(&mksServo, speed > 0 ? 1 : 0, abs_speed, 250);
                HAL_Delay(1);
            }
            if (buttonsState.gyro == BUTTON_OFF)
            {
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0);
                printf("[GYRO] Stopped by button release\r\n");
                HAL_Delay(200);
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0);
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0);
            }
        }

        // Manual режим з покращеним антидребезгом
        {
            static uint32_t last_pot_tick = 0;
            static uint8_t cached_pot_percent = 0;

            uint32_t now = HAL_GetTick();

            if (now - last_pot_tick >= 100)
            {
                last_pot_tick = now;
                cached_pot_percent = getPotentiometerValuePercentage();
            }

            if (buttonsState.turn_left == BUTTON_ON)
            {
                flag_for_blocking = true;
                if (!flag_first_run)
                {
                    int64_t add_val = 0;
                    // ... закомментированный код ...
                }
                flag_first_run = true;
                MksServo_SpeedModeRun(&mksServo, 0x01, (cached_pot_percent * 6 + 50), 250);
                printf("[MKS] Servo running left\r\n");
            }
            else if (buttonsState.turn_right == BUTTON_ON)
            {
              flag_for_blocking = true;
                if (!flag_first_run)
                {
                    int64_t add_val = 0;
                    // ... закомментированный код ...
                }
                flag_first_run = true;
                MksServo_SpeedModeRun(&mksServo, 0x00, (cached_pot_percent * 6 + 50), 250);
                printf("[MKS] Servo running right\r\n");
            }
            else if ((buttonsState.turn_left == BUTTON_OFF && buttonsState.turn_right == BUTTON_OFF) && flag_first_run)
            {
                flag_first_run = false;
                MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0);
                HAL_Delay(100);
                while (!MksServo_GetAdditionValue(&mksServo, &encoderScanPoints.giro_point, 100))
                {
                    printf("[ENC] Failed to read entry scan point!\n");
                }
                encoderScanPoints.entry_scan_point = encoderScanPoints.giro_point;
                flag_for_blocking = false;
                encoderScanPoints.flag_first_run = true; // Скидаємо прапорець для наступного запуску
                printf("[ENC] Entry scan point set: 0x%08lX%08lX\n",
                    (uint32_t)((current_motor_pos >> 32) & 0xFFFFFFFF),
                    (uint32_t)(current_motor_pos & 0xFFFFFFFF));
                printf("[MKS] Servo stopped (antibouce)\r\n");
            }
        }
    }
    break;

  case State::Scan:
  {
    // --- ЛОГІКА СКАНУВАННЯ через структуру-функтор з enum FSM ---
    extern AngleSetting mode_scan; // Глобальна змінна режиму
    struct ScanSweepFSM
    {
      enum FSMState
      {
        FSM_INIT = 0,
        FSM_MOVING,
        FSM_PAUSE
      };
      FSMState state = FSM_INIT;
      int direction = 1; // 1 = вправо, 0 = вліво
      uint32_t stop_time = 0;
      int32_t limit_ticks = 0;
      uint32_t last_carry_poll = 0;
      uint32_t last_speed_update = 0;
      int initialized = 0;
      int last_speed = 0; // Для ANGLE_ADJUST
      void reset()
      {
        state = FSM_INIT;
        initialized = 0;
      }
      void operator()(MksServo_t &mksServo, Motor &motor)
      {
        uint32_t now = HAL_GetTick();
        switch (state)
        {
        case FSM_INIT:
        {
          direction = 1;
          stop_time = 0;
          // --- Вибір кута осциляції ---
          int angle = 0;
          if (mode_scan == ANGLE_SCAN)
          {
            angle = motor.oscillation_angle;
          }
          else
          {
            int pot = getPotentiometerValuePercentage();
            angle = (pot * 360) / 100;
            if (angle < 1)
              angle = 1; // Мінімальний кут
          }
          limit_ticks = angle_deg_to_encoder_ticks((float)angle) / 2;
          int64_t addition_init = 0;
          if (MksServo_GetAdditionValue(&mksServo, &addition_init, 500))
          {
            printf("[SCAN][DEBUG] Initial addition = %" PRId64 "\n", addition_init);
            // Определяем ближайшую границу и направление
            if (addition_init >= limit_ticks)
            {
              direction = 0; // едем вліво
            }
            else if (addition_init <= -limit_ticks)
            {
              direction = 1; // едем вправо
            }
            else
            {
              // Находимся между границами — выбираем ближайшую
              direction = (abs(limit_ticks - addition_init) < abs(-limit_ticks - addition_init)) ? 1 : 0;
            }
          }
          else
          {
            printf("[SCAN][DEBUG] Initial addition: GetAdditionValue failed\n");
          }
          printf("[SCAN][DEBUG] angle = %d\n", angle);
          printf("[SCAN][DEBUG] scan_limit_ticks = %ld\n", (long)limit_ticks);
          uint8_t pot = getPotentiometerValuePercentage();
          int speed = pot * 6 + 50;
          last_speed = speed;
          MksServo_SpeedModeRun(&mksServo, direction, speed, 250);
          printf("[SCAN] Start %s, limit=%ld (encoder ticks), speed=%d\n", direction ? "right" : "left", (long)limit_ticks, speed);
          last_speed_update = now;
          last_carry_poll = now;
          state = FSM_MOVING;
          break;
        }
        case FSM_MOVING:
        {
          if (now - last_carry_poll >= 20)
          {
            last_carry_poll = now;
            int64_t addition = 0;
            int angle = 0;
            if (mode_scan == ANGLE_ADJUST)
            {
              int pot = getPotentiometerValuePercentage();
              angle = (pot * 360) / 100;
              if (angle < 1)
                angle = 1;
              limit_ticks = angle_deg_to_encoder_ticks((float)angle) / 2;
            }
            if (MksServo_GetAdditionValue(&mksServo, &addition, 100))
            {
              printf("[SCAN][TRACE] addition=%" PRId64 ", limit=+-%ld, dir=%d\n", addition, (long)limit_ticks, direction);
              int boundary_reached = 0;
              if (direction == 1 && addition >= limit_ticks)
                boundary_reached = 1;
              else if (direction == 0 && addition <= -limit_ticks)
                boundary_reached = 1;
              // В режимі ANGLE_ADJUST: якщо мотор вийшов за нову межу — одразу розворот
              if (boundary_reached)
              {
                MksServo_SpeedModeRun(&mksServo, direction, 0, 252);
                HAL_Delay(150); // експериментальна пауза для зупинки(замінити на неблок.)
                printf("[SCAN] Stop at %" PRId64 " (limit=+-%ld)\n", addition, (long)limit_ticks);
                stop_time = now;
                state = FSM_PAUSE;
              }
              else
              {
                int64_t distance_to_boundary = (direction == 1) ? (limit_ticks - addition) : (addition + limit_ticks);
                if (distance_to_boundary > (limit_ticks / 10) && (now - last_speed_update >= 100))
                {
                  last_speed_update = now;
                  int speed = 0;
                  if (mode_scan == ANGLE_SCAN)
                  {
                    uint8_t pot = getPotentiometerValuePercentage();
                    speed = pot * 6 + 50;
                    last_speed = speed;
                  }
                  else
                  {
                    speed = last_speed; // Не оновлюємо швидкість
                  }
                  MksServo_SpeedModeRun(&mksServo, direction, speed, 250);
                  printf("[SCAN] Speed updated: dir=%d, speed=%d, dist_to_boundary=%" PRId64 "\n", direction, speed, distance_to_boundary);
                }
              }
            }
            else
            {
              printf("[SCAN][ERROR] GetAdditionValue failed\n");
            }
          }
          break;
        }
        case FSM_PAUSE:
        {
          if (now - stop_time > 100)
          {
            direction = !direction;
            // --- Перераховуємо кут для ANGLE_ADJUST ---
            int angle = 0;
            if (mode_scan == ANGLE_SCAN)
            {
              angle = motor.oscillation_angle;
            }
            else
            {
              int pot = getPotentiometerValuePercentage();
              angle = (pot * 360) / 100;
              if (angle < 1)
                angle = 1;
              limit_ticks = angle_deg_to_encoder_ticks((float)angle) / 2;
              printf("[SCAN][ADJUST] angle=%d, limit_ticks=%ld\n", angle, (long)limit_ticks);
            }
            int speed = (mode_scan == ANGLE_SCAN) ? (getPotentiometerValuePercentage() * 6 + 50) : last_speed;
            MksServo_SpeedModeRun(&mksServo, direction, speed, 250);
            printf("[SCAN] Reverse, dir=%d, speed=%d\n", direction, speed);
            last_speed_update = now;
            state = FSM_MOVING;
          }
          break;
        }
        }
      }
    };
    static ScanSweepFSM scanFSM;
    scanFSM(mksServo, motor);
    break;
  }
  case State::BindMode:
    // TODO: дії для BindMode
    break;
  case State::Calibrate:
    // TODO: дії для Calibrate
    break;
  case State::CalibrateAndBind:
    // TODO: дії для CalibrateAndBind
    break;
  case State::AngleAdjust:
    break;

  default:
    // TODO: обробка невідомого стану
    break;
  }
  HAL_Delay(1); // Затримка перед виходом з циклу
  // <-- Закриваємо if (now_tick - last_imu_tick >= 20)
} // <-- Закриваємо while(1)
} // <-- Закриваємо main

/**
 * @brief Налаштування системного тактового генератора
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Ініціалізація генераторів RCC згідно з параметрами
   * у структурі RCC_OscInitTypeDef.
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

  /** Ініціалізація тактових сигналів CPU, AHB та APB
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
 * @brief Ініціалізація ADC1
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

  /** Загальна конфігурація
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

  /** Налаштування регулярного каналу
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
 * @brief Ініціалізація I2C1
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
  hi2c1.Init.ClockSpeed = 400000;
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
 * @brief Ініціалізація SPI2
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* Налаштування SPI2 */
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
 * @brief Ініціалізація USART1
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
 * @brief Ініціалізація USART2
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
 * @brief Ініціалізація USART3
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
 * @brief Ініціалізація GPIO
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* Увімкнення тактування портів GPIO */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Встановити рівень виходу для пінів */
  HAL_GPIO_WritePin(GPIOC, LED_Pin | LED2_Pin, GPIO_PIN_RESET);

  /* Встановити рівень виходу для пінів */
  HAL_GPIO_WritePin(GPIOA, LAMP_Pin | IMU_EN_Pin, GPIO_PIN_RESET);

  /* Встановити рівень виходу для пінів */
  HAL_GPIO_WritePin(GPIOB, RS485_DERE_Pin | CSN_Pin | CE_Pin, GPIO_PIN_RESET);

  /* Налаштування пінів : LED_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED_Pin | LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Налаштування пінів : LAMP_Pin IMU_EN_Pin */
  GPIO_InitStruct.Pin = LAMP_Pin | IMU_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Налаштування пінів : CALL_Pin COMP_Pin BIND_Pin */
  GPIO_InitStruct.Pin = CALL_Pin | COMP_Pin | BIND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Налаштування пінів : LEFT_Pin RIGHT_Pin */
  GPIO_InitStruct.Pin = LEFT_Pin | RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Налаштування піну : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // ВИПРАВЛЕННЯ: прибираємо pull-up, оскільки IRQ вже підтягнутий на платі
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Увімкнення та налаштування переривання EXTI Line */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* Налаштування пінів : RS485_DERE_Pin CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = RS485_DERE_Pin | CSN_Pin | CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // ВИПРАВЛЕННЯ: додаємо pull-down для DERE
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // --- Настройка PB3 як вход з прерыванием по спаду ---
  GPIO_InitStruct.Pin = PB3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PB3_INT_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(PB3_INT_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PB3_INT_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __cplusplus
extern "C"
{
#endif

  // Глобальна змінна для сигналізації про завершення прийому UART3
  // Callback-функції для роботи з UART перериваннями

  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    // Callback при помилці UART
    // Додаткових дій не потрібно, HAL сам керує станом
  }

  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
    if (huart == &huart3)
    {
      // Тут обробіть прийнятий байт: uart3_rx_byte
      // Наприклад, покладіть у свій буфер або обробіть одразу
      MksServo_UART_IRQHandler(&mksServo, huart);

      // Перезапустити прийом наступного байта
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
 * @brief  Callback, що викликається при переповненні таймера у неблокуючому режимі
 * @note   Ця функція викликається, коли відбувається переривання TIM3, всередині
 * HAL_TIM_IRQHandler(). Вона напряму викликає HAL_IncTick() для інкременту
 * глобальної змінної "uwTick", яка використовується як таймер додатку.
 * @param  htim : дескриптор таймера
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
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
 * @brief  Ця функція виконується у разі виникнення помилки.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Користувач може додати власну реалізацію для повідомлення про помилку HAL */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Виводить ім'я вихідного файлу та номер рядка,
 *         де виникла помилка assert_param.
 * @param  file: вказівник на ім'я вихідного файлу
 * @param  line: номер рядка, де виникла помилка
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Користувач може додати власну реалізацію для виводу імені файлу та рядка,
     наприклад: printf("Wrong parameters value: file %s on line %d\r\n", file, line); */
  printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

// --- Фоновий парсер UART3 для виводу лише 5-байтових пакетів статусу та обробки 0x92 ---
// Тепер парсер не використовує CRC з відповіді, а лише last_cmd_crc, який оновлюється при відправці команди
#ifdef __cplusplus
extern "C"
{
#endif
  State stateMachine_getState();
#ifdef __cplusplus
}
#endif

void MksServo_BackgroundPacketDebug(MksServo_t *servo)
{
  static uint8_t packet[16]; // запас по довжині
  static uint8_t idx = 0;
  static uint8_t collecting = 0;
  uint8_t byte;
  extern int scan_ready_to_move;  // прапорець для Scan-режиму
  static int zeroing_success = 0; // 1 - успішно обнулили
  while (MksServo_RxGetByte(servo, &byte))
  {
    // printf("[UART3][RAW] RX byte: %02X\n", byte);
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
        // На третьому байті (packet[2]) визначаємо тип пакета
        switch (packet[2])
        {
        case 0xFD: // Статусний пакет
          // Чекаємо 5 байт
          break;
        case 0x92: // Відповідь на Set Current Axis to Zero
          // Чекаємо 5 байт
          break;
        case 0xFE: // Відповідь на AbsoluteMotionByPulse_FE
          // Чекаємо 5 байт
          break;
        case 0xF5: // F5 AbsoluteMotionByAxis
          // Чекаємо 5 байт
          break;
        case 0xF6: // F6 Stop motor
          // Чекаємо 5 байт
          break;
        default:
          // Поки не підтримуємо інші типи
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
        // --- Оновлюємо глобальний enum статусу ---
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
        // --- Оновлюємо глобальний enum статусу ---
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
        // --- Оновлюємо глобальний enum статусу ---
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
      // Для пакета 0xF5: FB xx F5 status crc
      if (packet[2] == 0xF5 && idx == 5)
      {
        uint8_t status = packet[3];
        // --- Оновлюємо глобальний enum статусу для F5 ---
        switch (status)
        {
        case 0:
          printf("[UART3][F5] status: 0 - run fail\n");
          break;
        case 1:
          printf("[UART3][F5] status: 1 - run starting...\n");
          break;
        case 2:
          printf("[UART3][F5] status: 2 - run complete\n");
          break;
        case 3:
          printf("[UART3][F5] status: 3 - end limit stopped\n");
          break;
        default:
          printf("[UART3][F5] status: %u - unknown\n", status);
          break;
        }
        printf("[UART3][PKT] ");
        for (int i = 0; i < 5; ++i)
          printf("%02X ", packet[i]);
        printf("\n");
        collecting = 0;
        idx = 0;
      }
      // Для пакета 0xF6: FB xx F6 status crc
      if (packet[2] == 0xF6 && idx == 5)
      {
        uint8_t status = packet[3];
        //  printf("[UART3][F6] status: %u - ", status);
        switch (status)
        {
        case 0:
          //  printf("stop the motor fail\n");
          break;
        case 1:
          //  printf("start to stop the motor...\n");
          break;
        case 2:
          // printf("stop the motor success\n");
          break;
        default:
          // printf("unknown status\n");
          break;
        }
        // printf("[UART3][PKT] ");
        for (int i = 0; i < 5; ++i)
          //   printf("%02X ", packet[i]);
          // printf("\n");
          collecting = 0;
        idx = 0;
      }
    }
  }
  // Оновлюємо глобальний прапорець успішного обнулення
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

// --- Функція для сброса статуса F5 ---
void reset_last_f5_status()
{
  last_f5_status = 255;
}

// Глобальна змінна для сигналізації про переривання на PB3
volatile uint8_t pb3_int_flag = 0;

void EXTI3_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(PB3_INT_Pin) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(PB3_INT_Pin);
    pb3_int_flag = 1;
  }
}
