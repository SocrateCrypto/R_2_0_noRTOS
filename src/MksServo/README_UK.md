# Бібліотека MKS Servo Driver

Зручна C++ бібліотека для керування кроковими двигунами MKS SERVOxxD на STM32 HAL без RTOS.

## Особливості

- **Неблокуюча робота**: використовує переривання та циклічний буфер для UART
- **Підтримка RS485**: автоматичне керування піном DERE для напівдуплексного зв'язку
- **Високорівневий API**: прості методи для керування рухом
- **Інкапсуляція**: вся низькорівнева логіка прихована від користувача
- **Зручні утиліти**: перетворення між різними одиницями вимірювання

## Структура бібліотеки

```
MksServo/
├── MksServo_arduino.h/c     # Низькорівнева C-бібліотека (переривання, UART, CRC)
├── MksServoDriver.h/cpp     # Високорівневий C++ інтерфейс
└── README.md               # Дана документація
```

## Швидкий старт

### 1. Ініціалізація

```cpp
#include "MksServo/MksServoDriver.h"

// Глобальний екземпляр драйвера
MksServoDriver servo;

void setup() {
    // Ініціалізація (UART3, GPIOB, пін DERE, адреса пристрою)
    servo.init(&huart3, GPIOB, RS485_DERE_Pin, 1);
    
    // Базове налаштування двигуна
    servo.setWorkMode(2);           // CR_vFOC режим (векторне керування по струму)
    servo.setWorkingCurrent(500);   // 500mA робочий струм
    servo.enableMotor(true);        // увімкнути двигун
    servo.resetPosition();          // скидання позиції енкодера
}
```

### 2. Обробка переривань UART

```cpp
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        servo.handleUartInterrupt(huart);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}
```

### 3. Основне використання

```cpp
void loop() {
    // Поворот на певний кут
    servo.moveByAngle(90.0f, 150);     // 90 градусів зі швидкістю 150
    
    // Рух до абсолютної позиції
    servo.moveToPosition(2.5f, 200);   // 2.5 оберта зі швидкістю 200
    
    // Безперервне обертання
    servo.setSpeed(30.0f, true);       // 30 RPM за годинниковою стрілкою
    servo.setSpeed(50.0f, false);      // 50 RPM проти годинникової стрілки
    servo.stopAndHold();               // зупинка з утриманням позиції
    
    // Читання поточного стану
    float revolutions = servo.getCurrentRevolutions();
    float degrees = servo.getCurrentDegrees();
    float rpm = servo.getCurrentRPM();
    bool moving = servo.isMoving();
    bool error = servo.hasError();
}
```

## Довідник API

### Ініціалізація та налаштування

| Метод | Опис |
|-------|------|
| `init(huart, port, pin, addr)` | Ініціалізація драйвера |
| `setWorkMode(mode)` | Встановлення режиму роботи (0-5) |
| `setWorkingCurrent(ma)` | Встановлення робочого струму в мА |
| `setSubdivision(steps)` | Встановлення мікрокроків |
| `enableMotor(enable)` | Увімкнення/вимкнення двигуна |
| `resetPosition()` | Скидання позиції енкодера в нуль |

### Керування рухом

| Метод | Опис |
|-------|------|
| `moveByAngle(degrees, speed)` | Поворот на кут (градуси) |
| `moveToPosition(revolutions, speed)` | Рух до позиції (оберти) |
| `rotateRevolutions(revolutions, speed)` | Поворот на N обертів |
| `setSpeed(rpm, clockwise)` | Безперервне обертання (об/хв) |
| `stopAndHold()` | Зупинка з утриманням |

### Читання стану

| Метод | Повертає |
|-------|----------|
| `getCurrentRevolutions()` | Поточна позиція в обертах |
| `getCurrentDegrees()` | Поточна позиція в градусах |
| `getCurrentRPM()` | Поточна швидкість в об/хв |
| `isMoving()` | true якщо двигун рухається |
| `hasError()` | true при наявності помилки |

### Низькорівневі методи

| Метод | Опис |
|-------|------|
| `setSpeedMode(dir, speed, acc)` | Режим керування швидкістю |
| `setPositionMode(pos, speed, acc)` | Режим позиціонування |
| `readPosition(position)` | Читання сирої позиції енкодера |
| `readSpeed(speed)` | Читання поточної швидкості |
| `readStatus(status)` | Читання статусу двигуна |

## Константи та одиниці вимірювання

- **Кроки на оберт**: 1600 (200 кроків × 8 мікрокроків)
- **Роздільна здатність енкодера**: 16384 імпульси на оберт
- **Максимальна швидкість**: до 3000 RPM (в режимі vFOC)
- **Діапазон кутів**: ±180° або ±π радіан

## Режими роботи двигуна

| Код | Режим | Макс. RPM | Особливості |
|-----|-------|-----------|-------------|
| 0 | CR_OPEN | 400 RPM | Розімкнене керування по струму, працює без енкодера |
| 1 | CR_CLOSE | 1500 RPM | Замкнене керування по струму |
| 2 | **CR_vFOC** | **3000 RPM** | **Векторне керування по струму (за замовчуванням)** |
| 3 | SR_OPEN | 400 RPM | Розімкнене керування по швидкості, працює без енкодера |
| 4 | SR_CLOSE | 1500 RPM | Замкнене керування по швидкості |
| 5 | SR_vFOC | 3000 RPM | Векторне керування по швидкості |

**Примітки:**
- За замовчуванням використовується режим CR_vFOC (код 2)
- Режими OPEN (0, 3) можуть працювати без енкодера
- vFOC режими забезпечують самоадаптацію та максимальну продуктивність

## Приклади використання

### Просте позиціонування

```cpp
// Поворот на 90 градусів праворуч
servo.moveByAngle(90.0f, 100);

// Очікування завершення руху
while (servo.isMoving()) {
    HAL_Delay(10);
}

// Повернення в початкову позицію
servo.moveByAngle(-90.0f, 100);
```

### Сканування

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

### Стабілізація по гіроскопу

```cpp
void stabilizeByGyro() {
    float gyroAngle = getGyroAngle(); // отримати кут з гіроскопа
    float currentAngle = servo.getCurrentDegrees();
    float error = -gyroAngle - currentAngle;
    
    if (fabs(error) > 1.0f) { // мертва зона ±1°
        servo.moveByAngle(error * 0.5f, 100); // пропорційне керування
    }
}
```

## Налагодження та діагностика

```cpp
void printServoStatus() {
    printf("Статус серво:\r\n");
    printf("  Позиція: %.2f об (%.1f град)\r\n", 
           servo.getCurrentRevolutions(), servo.getCurrentDegrees());
    printf("  Швидкість: %.1f RPM\r\n", servo.getCurrentRPM());
    printf("  Рухається: %s\r\n", servo.isMoving() ? "ТАК" : "НІ");
    printf("  Помилка: %s\r\n", servo.hasError() ? "ТАК" : "НІ");
}
```

## Технічні деталі

### Структура пакета UART

**Команда**: `FA [ADDR] [CMD] [DATA...] [CRC]`
**Відповідь**: `FB [ADDR] [CMD] [DATA...] [CRC]`

### Тайминги RS485

- Перемикання DERE: 1мс до передачі
- Швидкість UART: 38400 бод
- Таймаут відповіді: 1000мс

### Обробка помилок

- Автоматична перевірка CRC
- Повторні спроби при помилках зв'язку
- Діагностика стану двигуна

## Сумісність

- STM32F1xx, F4xx та інші серії з HAL
- Компілятори: GCC ARM, IAR, Keil
- Без використання RTOS
- Мінімальні вимоги до RAM: ~256 байт

## Контакти

**Розробник**: Анатолій  
**Telegram**: @CyNNpd

Для питань, пропозицій та технічної підтримки звертайтесь у Telegram.



*Бібліотека розроблена для проекту керування кроковими двигунами MKS SERVO на платформі STM32. Призначена для використання в системах позиціонування, сканування та стабілізації.*
