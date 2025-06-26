# MKS Servo Driver Library

Удобная C++ библиотека для управления шаговыми двигателями MKS SERVOxxD на STM32 HAL без RTOS.

## Особенности

- **Неблокирующая работа**: использует прерывания и циклический буфер для UART
- **RS485 поддержка**: автоматическое управление пином DERE для полудуплексной связи
- **Высокоуровневый API**: простые методы для управления движением
- **Инкапсуляция**: вся низкоуровневая логика скрыта от пользователя
- **Удобные утилиты**: преобразование между различными единицами измерения

## Структура библиотеки

```
MksServo/
├── MksServo_arduino.h/c     # Низкоуровневая C-библиотека (прерывания, UART, CRC)
├── MksServoDriver.h/cpp     # Высокоуровневый C++ интерфейс
└── README.md               # Данная документация
```

## Быстрый старт

### 1. Инициализация

```cpp
#include "MksServo/MksServoDriver.h"

// Глобальный экземпляр драйвера
MksServoDriver servo;

void setup() {
    // Инициализация (UART3, GPIOB, пин DERE, адрес устройства)
    servo.init(&huart3, GPIOB, RS485_DERE_Pin, 1);
    
    // Базовая настройка двигателя
    servo.setWorkMode(2);           // CR_vFOC режим (векторное управление по току)
    servo.setWorkingCurrent(500);   // 500mA рабочий ток
    servo.enableMotor(true);        // включить двигатель
    servo.resetPosition();          // сброс позиции энкодера
}
```

### 2. Обработка прерываний UART

```cpp
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        servo.handleUartInterrupt(huart);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}
```

### 3. Основное использование

```cpp
void loop() {
    // Поворот на определенный угол
    servo.moveByAngle(90.0f, 150);     // 90 градусов со скоростью 150
    
    // Движение к абсолютной позиции
    servo.moveToPosition(2.5f, 200);   // 2.5 оборота со скоростью 200
    
    // Непрерывное вращение
    servo.setSpeed(30.0f, true);       // 30 RPM по часовой стрелке
    servo.setSpeed(50.0f, false);      // 50 RPM против часовой стрелки
    servo.stopAndHold();               // остановка с удержанием позиции
    
    // Чтение текущего состояния
    float revolutions = servo.getCurrentRevolutions();
    float degrees = servo.getCurrentDegrees();
    float rpm = servo.getCurrentRPM();
    bool moving = servo.isMoving();
    bool error = servo.hasError();
}
```

## API Reference

### Инициализация и настройка

| Метод | Описание |
|-------|----------|
| `init(huart, port, pin, addr)` | Инициализация драйвера |
| `setWorkMode(mode)` | Установка режима работы (0-5) |
| `setWorkingCurrent(ma)` | Установка рабочего тока в мА |
| `setSubdivision(steps)` | Установка микрошагов |
| `enableMotor(enable)` | Включение/выключение двигателя |
| `resetPosition()` | Сброс позиции энкодера в ноль |

### Управление движением

| Метод | Описание |
|-------|----------|
| `moveByAngle(degrees, speed)` | Поворот на угол (градусы) |
| `moveToPosition(revolutions, speed)` | Движение к позиции (обороты) |
| `rotateRevolutions(revolutions, speed)` | Поворот на N оборотов |
| `setSpeed(rpm, clockwise)` | Непрерывное вращение (об/мин) |
| `stopAndHold()` | Остановка с удержанием |

### Чтение состояния

| Метод | Возвращает |
|-------|------------|
| `getCurrentRevolutions()` | Текущая позиция в оборотах |
| `getCurrentDegrees()` | Текущая позиция в градусах |
| `getCurrentRPM()` | Текущая скорость в об/мин |
| `isMoving()` | true если двигатель движется |
| `hasError()` | true при наличии ошибки |

### Низкоуровневые методы

| Метод | Описание |
|-------|----------|
| `setSpeedMode(dir, speed, acc)` | Режим управления скоростью |
| `setPositionMode(pos, speed, acc)` | Режим позиционирования |
| `readPosition(position)` | Чтение сырой позиции энкодера |
| `readSpeed(speed)` | Чтение текущей скорости |
| `readStatus(status)` | Чтение статуса двигателя |

## Константы и единицы измерения

- **Шаги на оборот**: 1600 (200 шагов × 8 микрошагов)
- **Разрешение энкодера**: 16384 импульса на оборот
- **Максимальная скорость**: до 3000 RPM (в режиме vFOC)
- **Диапазон углов**: ±180° или ±π радиан

## Режимы работы двигателя

| Код | Режим | Макс. RPM | Особенности |
|-----|-------|-----------|-------------|
| 0 | CR_OPEN | 400 RPM | Разомкнутое управление по току, работает без энкодера |
| 1 | CR_CLOSE | 1500 RPM | Замкнутое управление по току |
| 2 | **CR_vFOC** | **3000 RPM** | **Векторное управление по току (по умолчанию)** |
| 3 | SR_OPEN | 400 RPM | Разомкнутое управление по скорости, работает без энкодера |
| 4 | SR_CLOSE | 1500 RPM | Замкнутое управление по скорости |
| 5 | SR_vFOC | 3000 RPM | Векторное управление по скорости |

**Примечания:**
- По умолчанию используется режим CR_vFOC (код 2)
- Режимы OPEN (0, 3) могут работать без энкодера
- vFOC режимы обеспечивают самоадаптацию и максимальную производительность

## Примеры использования

### Простое позиционирование

```cpp
// Поворот на 90 градусов вправо
servo.moveByAngle(90.0f, 100);

// Ожидание завершения движения
while (servo.isMoving()) {
    HAL_Delay(10);
}

// Возврат в исходную позицию
servo.moveByAngle(-90.0f, 100);
```

### Сканирование

```cpp
void scanMode() {
    static uint32_t lastScanTime = 0;
    static float scanAngle = -90.0f;
    static bool scanDirection = true;
    
    if (HAL_GetTick() - lastScanTime > 1000) {
        lastScanTime = HAL_GetTick();
        
        servo.moveByAngle(scanDirection ? 10.0f : -10.0f, 50);
        scanAngle += scanDirection ? 10.0f : -10.0f;
        
        if (scanAngle >= 90.0f || scanAngle <= -90.0f) {
            scanDirection = !scanDirection;
        }
    }
}
```

### Стабилизация по гироскопу

```cpp
void stabilizeByGyro() {
    float gyroAngle = getGyroAngle(); // получить угол с гироскопа
    float currentAngle = servo.getCurrentDegrees();
    float error = -gyroAngle - currentAngle;
    
    if (abs(error) > 1.0f) { // мертвая зона ±1°
        servo.moveByAngle(error * 0.5f, 100); // пропорциональное управление
    }
}
```

## Отладка и диагностика

```cpp
void printServoStatus() {
    printf("Servo Status:\r\n");
    printf("  Position: %.2f rev (%.1f deg)\r\n", 
           servo.getCurrentRevolutions(), servo.getCurrentDegrees());
    printf("  Speed: %.1f RPM\r\n", servo.getCurrentRPM());
    printf("  Moving: %s\r\n", servo.isMoving() ? "YES" : "NO");
    printf("  Error: %s\r\n", servo.hasError() ? "YES" : "NO");
}
```

## Технические детали

### Структура пакета UART

**Команда**: `FA [ADDR] [CMD] [DATA...] [CRC]`
**Ответ**: `FB [ADDR] [CMD] [DATA...] [CRC]`

### Тайминги RS485

- Переключение DERE: 1мс до передачи
- Скорость UART: 38400 бод
- Таймаут ответа: 1000мс

### Обработка ошибок

- Автоматическая проверка CRC
- Повторные попытки при ошибках связи
- Диагностика состояния двигателя

## Совместимость

- STM32F1xx, F4xx и другие серии с HAL
- Компиляторы: GCC ARM, IAR, Keil
- Без использования RTOS
- Минимальные требования к RAM: ~256 байт

## Контакты

**Разработчик**: Анатолий  
**Telegram**: @CyNNpd

Для вопросов, предложений и технической поддержки обращайтесь в Telegram.

---

*Библиотека разработана для проекта управления шаговыми двигателями MKS SERVO на платформе STM32. Предназначена для использования в системах позиционирования, сканирования и стабилизации.*
