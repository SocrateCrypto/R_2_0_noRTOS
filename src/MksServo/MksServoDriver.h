/**
 * @file MksServoDriver.h
 * @brief Высокоуровневый C++ драйвер для управления шаговыми двигателями MKS SERVOxxD
 * 
 * Этот драйвер предоставляет удобный интерфейс для управления шаговыми двигателями
 * MKS SERVO через UART с поддержкой RS485 (управление DERE), используя прерывания
 * и циклический буфер для неблокирующего приема данных.
 * 
 * Основные возможности:
 * - Инициализация и настройка двигателя
 * - Управление движением (скоростной и позиционный режимы)
 * - Чтение позиции энкодера, скорости, статуса
 * - Настройка рабочих параметров (ток, микрошаги, режим работы)
 * - Утилиты для преобразования позиции в обороты/градусы
 * 
 * Пример использования:
 * @code
 * // Глобальный экземпляр драйвера
 * MksServoDriver servo;
 * 
 * // В main() или setup():
 * servo.init(&huart3, GPIOB, RS485_DERE_Pin, 1); // адрес устройства = 1
 * servo.setWorkMode(2);           // CR_vFOC режим
 * servo.setWorkingCurrent(500);   // 500mA рабочий ток
 * servo.enableMotor(true);        // включить двигатель
 * servo.resetPosition();          // сброс позиции энкодера
 * 
 * // В основном цикле:
 * servo.setSpeedMode(1, 200, 2);  // направление=1, скорость=200, ускорение=2
 * 
 * int32_t pos;
 * if (servo.readPosition(pos)) {
 *     float revolutions = servo.positionToRevolutions(pos);
 *     printf("Position: %.2f revolutions\r\n", revolutions);
 * }
 * 
 * // Позиционирование:
 * servo.setPositionMode(1600, 100); // переход на позицию 1600 (1 оборот)
 * 
 * // В HAL_UART_RxCpltCallback:
 * servo.handleUartInterrupt(huart);
 * @endcode
 */

#ifndef MKS_SERVO_DRIVER_H
#define MKS_SERVO_DRIVER_H

#include "stm32f1xx_hal.h"
#include "MksServo_arduino.h"
#include <stdint.h>

class MksServoDriver {
private:
    MksServo_t servo;
    bool initialized;
    
    // Внутренние методы для отправки команд
    bool sendCommand(uint8_t command, uint8_t* data = nullptr, uint8_t dataLen = 0);
    bool readResponse(uint8_t expectedLen, uint8_t* buffer, uint32_t timeout_ms = 1000);
    
public:
    MksServoDriver();
    ~MksServoDriver();
    
    // Инициализация
    bool init(UART_HandleTypeDef* huart, GPIO_TypeDef* derePort, uint16_t derePin, uint8_t address);
    
    // Управление движением
    bool setSpeedMode(uint8_t direction, uint16_t speed, uint8_t acceleration = 2);
    bool setPositionMode(int32_t position, uint16_t speed = 100, uint8_t acceleration = 2);
    bool stop();
    bool enableMotor(bool enable = true);
    
    // Высокоуровневые методы управления
    bool moveToPosition(float revolutions, uint16_t speed = 100);
    bool moveByAngle(float degrees, uint16_t speed = 100);
    bool rotateRevolutions(float revolutions, uint16_t speed = 100);
    bool setSpeed(float rpm, bool clockwise = true);
    bool stopAndHold();
    
    // Чтение состояния
    bool readPosition(int32_t& position);
    bool readSpeed(int16_t& speed);
    bool readStatus(uint8_t& status);
    
    // Методы опроса состояния
    float getCurrentRevolutions();
    float getCurrentDegrees();
    float getCurrentRPM();
    bool isMoving();
    bool hasError();
    
    // Настройка параметров
    bool setWorkMode(uint8_t mode); // CR_vFOC, SR_vFOC, etc.
    bool setWorkingCurrent(uint16_t current_ma);
    bool setSubdivision(uint8_t microsteps);
    bool resetPosition();
    
    // Утилиты
    float positionToRevolutions(int32_t position);
    int32_t revolutionsToPosition(float revolutions);
    float positionToDegrees(int32_t position);
    int32_t degreesToPosition(float degrees);
    
    // Обработка прерываний UART (вызывать из HAL_UART_RxCpltCallback)
    void handleUartInterrupt(UART_HandleTypeDef* huart);
    
    // Диагностика для отладки
    uint16_t getBufferAvailable();
    
    // Проверка состояния
    bool isInitialized() const { return initialized; }
};

#endif // MKS_SERVO_DRIVER_H
