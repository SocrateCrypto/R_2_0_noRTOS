#include "MksServoDriver.h"
#include <cstring>

// Константы для команд MKS Servo
#define CMD_SPEED_MODE      0xF6
#define CMD_POSITION_MODE   0xFD
#define CMD_READ_POSITION   0x31
#define CMD_READ_SPEED      0x32
#define CMD_READ_STATUS     0x33
#define CMD_SET_MODE        0x82
#define CMD_SET_CURRENT     0x83
#define CMD_SET_SUBDIVISION 0x84
#define CMD_ENABLE_MOTOR    0xF3
#define CMD_RESET_POSITION  0x0A

// Константы для режимов работы
#define MODE_CR_OPEN        0x00
#define MODE_CR_CLOSE       0x01
#define MODE_CR_vFOC        0x02
#define MODE_SR_OPEN        0x03
#define MODE_SR_CLOSE       0x04
#define MODE_SR_vFOC        0x05

MksServoDriver::MksServoDriver() : initialized(false) {
    memset(&servo, 0, sizeof(servo));
}

MksServoDriver::~MksServoDriver() {
    // Деструктор
}

bool MksServoDriver::init(UART_HandleTypeDef* huart, GPIO_TypeDef* derePort, uint16_t derePin, uint8_t address) {
    MksServo_Init(&servo, huart, derePort, derePin, address);
    initialized = true;
    return true;
}

bool MksServoDriver::sendCommand(uint8_t command, uint8_t* data, uint8_t dataLen) {
    if (!initialized) return false;
    
    uint8_t tx[20];
    tx[0] = 0xFA;
    tx[1] = servo.device_address;
    tx[2] = command;
    
    uint8_t txLen = 3;
    if (data && dataLen > 0) {
        memcpy(&tx[3], data, dataLen);
        txLen += dataLen;
    }
    
    tx[txLen] = MksServo_GetCheckSum(tx, txLen);
    txLen++;
    
    // Отправка через RS485
    HAL_GPIO_WritePin(servo.dere_port, servo.dere_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(servo.huart, tx, txLen, 100);
    HAL_GPIO_WritePin(servo.dere_port, servo.dere_pin, GPIO_PIN_RESET);
    
    return true;
}

bool MksServoDriver::readResponse(uint8_t expectedLen, uint8_t* buffer, uint32_t timeout_ms) {
    if (!initialized || !buffer) return false;
    
    // Очистить буфер
    uint8_t dummy;
    while (MksServo_RxGetByte(&servo, &dummy));
    
    uint32_t start = HAL_GetTick();
    uint8_t rx_cnt = 0;
    
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (MksServo_RxGetByte(&servo, &b)) {
            if (rx_cnt != 0) {
                buffer[rx_cnt++] = b;
            } else if (b == 0xFB) {
                buffer[rx_cnt++] = b;
            }
            
            if (rx_cnt == expectedLen) {
                // Проверка CRC
                if (buffer[expectedLen-1] == MksServo_GetCheckSum(buffer, expectedLen-1)) {
                    return true;
                } else {
                    rx_cnt = 0; // Ошибка CRC, начать заново
                }
            }
        }
    }
    return false;
}

bool MksServoDriver::setSpeedMode(uint8_t direction, uint16_t speed, uint8_t acceleration) {
    uint8_t data[3];
    data[0] = (direction << 7) | ((speed >> 8) & 0x0F);
    data[1] = speed & 0xFF;
    data[2] = acceleration;
    
    if (!sendCommand(CMD_SPEED_MODE, data, 3)) return false;
    
    // Ждем подтверждение
    uint8_t response[5];
    return readResponse(5, response, 3000);
}

bool MksServoDriver::setPositionMode(int32_t position, uint16_t speed, uint8_t acceleration) {
    uint8_t data[7];
    data[0] = (position >> 24) & 0xFF;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = (position >> 0) & 0xFF;
    data[4] = (speed >> 8) & 0xFF;
    data[5] = speed & 0xFF;
    data[6] = acceleration;
    
    if (!sendCommand(CMD_POSITION_MODE, data, 7)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 3000);
}

bool MksServoDriver::stop() {
    return setSpeedMode(0, 0, 2); // Скорость 0 = остановка
}

bool MksServoDriver::enableMotor(bool enable) {
    uint8_t data[1];
    data[0] = enable ? 0x01 : 0x00;
    
    if (!sendCommand(CMD_ENABLE_MOTOR, data, 1)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 1000);
}

bool MksServoDriver::readPosition(int32_t& position) {
    if (!sendCommand(CMD_READ_POSITION)) return false;
    
    uint8_t response[10];
    if (!readResponse(10, response, 1000)) return false;
    
    // Позиция в байтах 5-8 (как в Arduino примере)
    position = (int32_t)(
        ((uint32_t)response[5] << 24) |
        ((uint32_t)response[6] << 16) |
        ((uint32_t)response[7] << 8)  |
        ((uint32_t)response[8] << 0)
    );
    
    return true;
}

bool MksServoDriver::readSpeed(int16_t& speed) {
    if (!sendCommand(CMD_READ_SPEED)) return false;
    
    uint8_t response[7];
    if (!readResponse(7, response, 1000)) return false;
    
    // Скорость в байтах 3-4
    speed = (int16_t)((response[3] << 8) | response[4]);
    
    return true;
}

bool MksServoDriver::readStatus(uint8_t& status) {
    if (!sendCommand(CMD_READ_STATUS)) return false;
    
    uint8_t response[5];
    if (!readResponse(5, response, 1000)) return false;
    
    status = response[3];
    return true;
}

bool MksServoDriver::setWorkMode(uint8_t mode) {
    uint8_t data[1];
    data[0] = mode;
    
    if (!sendCommand(CMD_SET_MODE, data, 1)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 1000);
}

bool MksServoDriver::setWorkingCurrent(uint16_t current_ma) {
    uint8_t data[2];
    data[0] = (current_ma >> 8) & 0xFF;
    data[1] = current_ma & 0xFF;
    
    if (!sendCommand(CMD_SET_CURRENT, data, 2)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 1000);
}

bool MksServoDriver::setSubdivision(uint8_t microsteps) {
    uint8_t data[1];
    data[0] = microsteps;
    
    if (!sendCommand(CMD_SET_SUBDIVISION, data, 1)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 1000);
}

bool MksServoDriver::resetPosition() {
    if (!sendCommand(CMD_RESET_POSITION)) return false;
    
    uint8_t response[5];
    return readResponse(5, response, 1000);
}

float MksServoDriver::positionToRevolutions(int32_t position) {
    return (float)position / 16384.0f; // 16384 импульса = 1 оборот
}

int32_t MksServoDriver::revolutionsToPosition(float revolutions) {
    return (int32_t)(revolutions * 16384.0f);
}

float MksServoDriver::positionToDegrees(int32_t position) {
    return (float)position * 360.0f / 16384.0f;
}

int32_t MksServoDriver::degreesToPosition(float degrees) {
    return (int32_t)(degrees * 16384.0f / 360.0f);
}

void MksServoDriver::handleUartInterrupt(UART_HandleTypeDef* huart) {
    MksServo_UART_IRQHandler(&servo, huart);
}

uint16_t MksServoDriver::getBufferAvailable() {
    return (servo.rx_head >= servo.rx_tail) ? 
           (servo.rx_head - servo.rx_tail) : 
           (MKS_SERVO_RX_BUF_SIZE - servo.rx_tail + servo.rx_head);
}

// Реализация высокоуровневых методов

bool MksServoDriver::moveToPosition(float revolutions, uint16_t speed) {
    int32_t targetPosition = revolutionsToPosition(revolutions);
    return setPositionMode(targetPosition, speed);
}

bool MksServoDriver::moveByAngle(float degrees, uint16_t speed) {
    int32_t currentPos;
    if (!readPosition(currentPos)) return false;
    
    float currentDegrees = positionToDegrees(currentPos);
    float targetDegrees = currentDegrees + degrees;
    int32_t targetPosition = degreesToPosition(targetDegrees);
    
    return setPositionMode(targetPosition, speed);
}

bool MksServoDriver::rotateRevolutions(float revolutions, uint16_t speed) {
    int32_t currentPos;
    if (!readPosition(currentPos)) return false;
    
    float currentRevs = positionToRevolutions(currentPos);
    float targetRevs = currentRevs + revolutions;
    
    return moveToPosition(targetRevs, speed);
}

bool MksServoDriver::setSpeed(float rpm, bool clockwise) {
    // Преобразование RPM в внутренние единицы скорости
    // Максимальная скорость зависит от режима работы:
    // CR_OPEN/SR_OPEN: 400 RPM, CR_CLOSE/SR_CLOSE: 1500 RPM, vFOC: 3000 RPM
    uint16_t speed = (uint16_t)(rpm * 10.0f);  // Примерное соотношение для vFOC режима
    if (speed > 3000) speed = 3000;  // Ограничение максимальной скорости
    
    uint8_t direction = clockwise ? 1 : 0;
    return setSpeedMode(direction, speed, 2);
}

bool MksServoDriver::stopAndHold() {
    return setSpeedMode(0, 0, 2); // Остановка с удержанием позиции
}

float MksServoDriver::getCurrentRevolutions() {
    int32_t position;
    if (readPosition(position)) {
        return positionToRevolutions(position);
    }
    return 0.0f;
}

float MksServoDriver::getCurrentDegrees() {
    int32_t position;
    if (readPosition(position)) {
        return positionToDegrees(position);
    }
    return 0.0f;
}

float MksServoDriver::getCurrentRPM() {
    int16_t speed;
    if (readSpeed(speed)) {
        // Преобразование внутренних единиц скорости в RPM
        return (float)speed / 10.0f;  // Обратное преобразование
    }
    return 0.0f;
}

bool MksServoDriver::isMoving() {
    int16_t speed;
    if (readSpeed(speed)) {
        return (speed != 0);
    }
    return false;
}

bool MksServoDriver::hasError() {
    uint8_t status;
    if (readStatus(status)) {
        // Проверка битов ошибок в статусе
        return (status & 0x0F) != 0; // Младшие 4 бита - ошибки
    }
    return true; // Если не можем прочитать статус, считаем что есть ошибка
}
