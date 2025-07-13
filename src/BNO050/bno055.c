#include "bno055.h"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1 << 14);    // 2^14

void bno055_setPage(uint8_t page)
{
    bno055_writeData(BNO055_PAGE_ID, page);
}

bno055_opmode_t bno055_getOperationMode()
{
    bno055_opmode_t mode;
    bno055_readData(BNO055_OPR_MODE, &mode, 1);
    return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode)
{
    bno055_writeData(BNO055_OPR_MODE, mode);
    if (mode == BNO055_OPERATION_MODE_CONFIG) {
        bno055_delay(19);
    } else {
        bno055_delay(7);
    }
}

void bno055_setOperationModeConfig()
{
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF()
{
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setExternalCrystalUse(bool state)
{
    bno055_setPage(0);
    uint8_t tmp = 0;
    bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
    tmp |= (state == true) ? 0x80 : 0x0;
    bno055_writeData(BNO055_SYS_TRIGGER, tmp);
    bno055_delay(700);
}

void bno055_enableExternalCrystal()
{
    bno055_setExternalCrystalUse(true);
}
void bno055_disableExternalCrystal()
{
    bno055_setExternalCrystalUse(false);
}

void bno055_reset()
{
    bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
    bno055_delay(700);
}

int8_t bno055_getTemp()
{
    bno055_setPage(0);
    uint8_t t;
    bno055_readData(BNO055_TEMP, &t, 1);
    return t;
}

void bno055_setup()
{
    bno055_reset();
    uint8_t id = 0;
    bno055_readData(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_ID) {
        printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n",
                id);
    }
    bno055_setPage(0);
    bno055_writeData(BNO055_SYS_TRIGGER, 0x0);
    // Select BNO055 config mode
    bno055_setOperationModeConfig();
    bno055_delay(10);
    
    // ===================================================
}

int16_t bno055_getSWRevision()
{
    bno055_setPage(0);
    uint8_t buffer[2];
    bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
    return (int16_t) ((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision()
{
    bno055_setPage(0);
    uint8_t tmp;
    bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
    return tmp;
}

uint8_t bno055_getSystemStatus()
{
    bno055_setPage(0);
    uint8_t tmp;
    bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
    return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult()
{
    bno055_setPage(0);
    uint8_t tmp;
    bno055_self_test_result_t res = { .mcuState = 0, .gyrState = 0, .magState =
            0, .accState = 0 };
    bno055_readData(BNO055_ST_RESULT, &tmp, 1);
    res.mcuState = (tmp >> 3) & 0x01;
    res.gyrState = (tmp >> 2) & 0x01;
    res.magState = (tmp >> 1) & 0x01;
    res.accState = (tmp >> 0) & 0x01;
    return res;
}

uint8_t bno055_getSystemError()
{
    bno055_setPage(0);
    uint8_t tmp;
    bno055_readData(BNO055_SYS_ERR, &tmp, 1);
    return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState()
{
    bno055_setPage(0);
    bno055_calibration_state_t cal =
            { .sys = 0, .gyro = 0, .mag = 0, .accel = 0 };
    uint8_t calState = 0;
    bno055_readData(BNO055_CALIB_STAT, &calState, 1);
    cal.sys = (calState >> 6) & 0x03;
    cal.gyro = (calState >> 4) & 0x03;
    cal.accel = (calState >> 2) & 0x03;
    cal.mag = calState & 0x03;
    return cal;
}

bno055_calibration_data_t bno055_getCalibrationData()
{
    bno055_calibration_data_t calData;
    uint8_t buffer[22];
    bno055_opmode_t operationMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(0);

    bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

    // Assumes little endian processor
    memcpy(&calData.offset.accel, buffer, 6);
    memcpy(&calData.offset.mag, buffer + 6, 6);
    memcpy(&calData.offset.gyro, buffer + 12, 6);
    memcpy(&calData.radius.accel, buffer + 18, 2);
    memcpy(&calData.radius.mag, buffer + 20, 2);

    bno055_setOperationMode(operationMode);

    return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData)
{
    uint8_t buffer[22];
    bno055_opmode_t operationMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(0);

    // Assumes litle endian processor
    memcpy(buffer, &calData.offset.accel, 6);
    memcpy(buffer + 6, &calData.offset.mag, 6);
    memcpy(buffer + 12, &calData.offset.gyro, 6);
    memcpy(buffer + 18, &calData.radius.accel, 2);
    memcpy(buffer + 20, &calData.radius.mag, 2);

    for (uint8_t i = 0; i < 22; i++) {
        // TODO(oliv4945): create multibytes write
        bno055_writeData(BNO055_ACC_OFFSET_X_LSB + i, buffer[i]);
    }

    bno055_setOperationMode(operationMode);
}

bno055_vector_t bno055_getVector(uint8_t vec)
{
    bno055_setPage(0);
    uint8_t buffer[8];    // Quaternion need 8 bytes

    if (vec == BNO055_VECTOR_QUATERNION)
        bno055_readData(vec, buffer, 8);
    else
        bno055_readData(vec, buffer, 6);

    double scale = 1;

    if (vec == BNO055_VECTOR_MAGNETOMETER) {
        scale = magScale;
    } else if (vec == BNO055_VECTOR_ACCELEROMETER
            || vec == BNO055_VECTOR_LINEARACCEL
            || vec == BNO055_VECTOR_GRAVITY) {
        scale = accelScale;
    } else if (vec == BNO055_VECTOR_GYROSCOPE) {
        scale = angularRateScale;
    } else if (vec == BNO055_VECTOR_EULER) {
        scale = eulerScale;
    } else if (vec == BNO055_VECTOR_QUATERNION) {
        scale = quaScale;
    }

    bno055_vector_t xyz = { .w = 0, .x = 0, .y = 0, .z = 0 };
    if (vec == BNO055_VECTOR_QUATERNION) {
        xyz.w = (int16_t) ((buffer[1] << 8) | buffer[0]) / scale;
        xyz.x = (int16_t) ((buffer[3] << 8) | buffer[2]) / scale;
        xyz.y = (int16_t) ((buffer[5] << 8) | buffer[4]) / scale;
        xyz.z = (int16_t) ((buffer[7] << 8) | buffer[6]) / scale;
    } else {
        xyz.x = (int16_t) ((buffer[1] << 8) | buffer[0]) / scale;
        xyz.y = (int16_t) ((buffer[3] << 8) | buffer[2]) / scale;
        xyz.z = (int16_t) ((buffer[5] << 8) | buffer[4]) / scale;
    }

    return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer()
{
    return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer()
{
    return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope()
{
    return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler()
{
    return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel()
{
    return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity()
{
    return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion()
{
    return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void bno055_setAxisMap(bno055_axis_map_t axis)
{
    uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
    uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1)
            | (axis.z_sign);
    bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
    bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}

bno055_vector_t bno055_getVectorAccelerometer_scaled(int scale)
{
    // Теперь функция возвращает только сырые значения акселерометра (int16_t)
    bno055_setPage(0);
    uint8_t buffer[6];
    bno055_readData(BNO055_VECTOR_ACCELEROMETER, buffer, 6);
    bno055_vector_t xyz = {0};
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]);
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]);
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]);
    return xyz;
}

bno055_vector_t bno055_getVectorGyroscope_scaled(int scale)
{
    // Функция возвращает вектор угловой скорости с пользовательским масштабом
    // scale — параметр масштабирования (например, 16 для получения значения в град/сек)
    // Данные считываются напрямую из регистра гироскопа BNO055
    // После этого делятся на scale для получения значения в нужных единицах
    //
    // Пример использования:
    // bno055_vector_t gyro = bno055_getVectorGyroscope_scaled(16);
    //
    // Если требуется учесть индивидуальные параметры калибровки (misalignment, sensitivity, offset),
    // их нужно применить к полученным данным отдельно, после вызова этой функции.
    bno055_setPage(0);
    uint8_t buffer[6];
    bno055_readData(BNO055_VECTOR_GYROSCOPE, buffer, 6);
    bno055_vector_t xyz = {0};
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / (double)scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / (double)scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / (double)scale;
    return xyz;
}

/*
 * Функция для установки диапазона (чувствительности) гироскопа
 * range - одно из значений:
 *   BNO055_GYRO_RANGE_2000DPS, BNO055_GYRO_RANGE_1000DPS, ...
 * После смены диапазона переменная angularRateScale обновляется автоматически.
 * Масштаб (scale) зависит от выбранного диапазона:
 *   ±2000 dps: 16 LSB/dps
 *   ±1000 dps: 32 LSB/dps
 *   ±500 dps:  64 LSB/dps
 *   ±250 dps: 128 LSB/dps
 *   ±125 dps: 256 LSB/dps
 */
void bno055_setGyroRange(uint8_t range) {
    bno055_opmode_t prevMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(1);
    // Сохраняем остальные биты регистра
    uint8_t reg = 0;
    bno055_readData(BNO055_GYRO_CONFIG_0, &reg, 1);
    reg = (reg & ~0x07) | (range & 0x07); // Младшие 3 бита - диапазон
    bno055_writeData(BNO055_GYRO_CONFIG_0, reg);
    // Обновляем переменную масштаба
    switch (range & 0x07) {
        case BNO055_GYRO_RANGE_2000DPS: angularRateScale = 16; break;
        case BNO055_GYRO_RANGE_1000DPS: angularRateScale = 32; break;
        case BNO055_GYRO_RANGE_500DPS:  angularRateScale = 64; break;
        case BNO055_GYRO_RANGE_250DPS:  angularRateScale = 128; break;
        case BNO055_GYRO_RANGE_125DPS:  angularRateScale = 256; break;
        default: angularRateScale = 16; break;
    }
    bno055_setPage(0);
    bno055_setOperationMode(prevMode);
}

// Новая функция: установка bandwidth акселерометра (биты 2-4 ACC_CONFIG)
void bno055_setAccelBandwidth(uint8_t bw) {
    bno055_opmode_t prevMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(1);
    uint8_t reg = 0;
    bno055_readData(BNO055_ACC_CONFIG, &reg, 1);
    reg = (reg & ~(0x1C)) | ((bw & 0x07) << 2); // Биты 2-4
    bno055_writeData(BNO055_ACC_CONFIG, reg);
    bno055_setPage(0);
    bno055_setOperationMode(prevMode);
}

// Новая функция: установка bandwidth гироскопа (биты 3-5 GYRO_CONFIG_0)
void bno055_setGyroBandwidth(uint8_t bw) {
    bno055_opmode_t prevMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(1);
    uint8_t reg = 0;
    bno055_readData(BNO055_GYRO_CONFIG_0, &reg, 1);
    reg = (reg & ~(0x38)) | ((bw & 0x07) << 3); // Биты 3-5
    bno055_writeData(BNO055_GYRO_CONFIG_0, reg);
    bno055_setPage(0);
    bno055_setOperationMode(prevMode);
}

/*
 * Функция для установки диапазона (чувствительности) акселерометра
 * range - одно из значений:
 *   BNO055_ACCEL_RANGE_2G, BNO055_ACCEL_RANGE_4G, ...
 * После смены диапазона переменная accelScale обновляется автоматически.
 * Масштаб (scale) зависит от выбранного диапазона:
 *   ±2g:  1000 LSB/g
 *   ±4g:   500 LSB/g
 *   ±8g:   250 LSB/g
 *   ±16g:  125 LSB/g
 */
void bno055_setAccelRange(uint8_t range) {
    bno055_opmode_t prevMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(1);
    // Сохраняем остальные биты регистра
    uint8_t reg = 0;
    bno055_readData(BNO055_ACC_CONFIG, &reg, 1);
    reg = (reg & ~0x03) | (range & 0x03); // Младшие 2 бита - диапазон
    bno055_writeData(BNO055_ACC_CONFIG, reg);
    // Обновляем переменную масштаба
    switch (range & 0x03) {
        case BNO055_ACCEL_RANGE_2G:  accelScale = 100; break; // 1 m/s^2 = 100 LSB
        case BNO055_ACCEL_RANGE_4G:  accelScale = 50; break;
        case BNO055_ACCEL_RANGE_8G:  accelScale = 25; break;
        case BNO055_ACCEL_RANGE_16G: accelScale = 12; break;
        default: accelScale = 100; break;
    }
    bno055_setPage(0);
    bno055_setOperationMode(prevMode);
}

// --- Включение прерывания по Data Ready гироскопа (INT) ---
void bno055_enableGyroDataReadyInterrupt(void)
{
    bno055_opmode_t prevMode = bno055_getOperationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(1);
    // Включаем бит 2 (GYRO_DRDY) в INT_MSK и INT_EN
    uint8_t int_msk = 0;
    bno055_readData(BNO055_INT_MSK, &int_msk, 1);
    printf("[BNO055] INT_MSK before: 0x%02X\n", int_msk);
    int_msk |= 0x04; // Bit 2: GYRO_DRDY
    bno055_writeData(BNO055_INT_MSK, int_msk);
    bno055_readData(BNO055_INT_MSK, &int_msk, 1);
    printf("[BNO055] INT_MSK after:  0x%02X\n", int_msk);
    uint8_t int_en = 0;
    bno055_readData(BNO055_INT_EN, &int_en, 1);
    printf("[BNO055] INT_EN before:  0x%02X\n", int_en);
    int_en |= 0x04; // Bit 2: GYRO_DRDY
    bno055_writeData(BNO055_INT_EN, int_en);
    bno055_readData(BNO055_INT_EN, &int_en, 1);
    printf("[BNO055] INT_EN after:   0x%02X\n", int_en);
    // INT_SETTINGS (Page 1, 0x0F)
    uint8_t int_settings = 0;
    bno055_readData(BNO055_GYR_INT_SETTINGS, &int_settings, 1);
    printf("[BNO055] INT_SETTINGS:   0x%02X\n", int_settings);
    bno055_setPage(0);
    bno055_setOperationMode(prevMode);
}

// Чтение "сырых" данных акселерометра напрямую по отдельным регистрам (0x08...0x0D)
bno055_vector_t bno055_readRawAccelRegisters(void)
{
    uint8_t x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
    bno055_setPage(0);
    bno055_readData(0x08, &x_lsb, 1); // ACC_DATA_X_LSB
    bno055_readData(0x09, &x_msb, 1); // ACC_DATA_X_MSB
    bno055_readData(0x0A, &y_lsb, 1); // ACC_DATA_Y_LSB
    bno055_readData(0x0B, &y_msb, 1); // ACC_DATA_Y_MSB
    bno055_readData(0x0C, &z_lsb, 1); // ACC_DATA_Z_LSB
    bno055_readData(0x0D, &z_msb, 1); // ACC_DATA_Z_MSB
    bno055_vector_t xyz = {0};
    xyz.x = (int16_t)((x_msb << 8) | x_lsb);
    xyz.y = (int16_t)((y_msb << 8) | y_lsb);
    xyz.z = (int16_t)((z_msb << 8) | z_lsb);
    return xyz;
}

// Чтение "сырых" данных гироскопа напрямую по отдельным регистрам (0x14...0x19)
bno055_vector_t bno055_readRawGyroRegisters(void)
{
    uint8_t x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
    bno055_setPage(0);
    bno055_readData(0x14, &x_lsb, 1); // GYR_DATA_X_LSB
    bno055_readData(0x15, &x_msb, 1); // GYR_DATA_X_MSB
    bno055_readData(0x16, &y_lsb, 1); // GYR_DATA_Y_LSB
    bno055_readData(0x17, &y_msb, 1); // GYR_DATA_Y_MSB
    bno055_readData(0x18, &z_lsb, 1); // GYR_DATA_Z_LSB
    bno055_readData(0x19, &z_msb, 1); // GYR_DATA_Z_MSB
    bno055_vector_t xyz = {0};
    xyz.x = (int16_t)((x_msb << 8) | x_lsb);
    xyz.y = (int16_t)((y_msb << 8) | y_lsb);
    xyz.z = (int16_t)((z_msb << 8) | z_lsb);
    return xyz;
}

bool bno055_isDataReady(void) {
    uint8_t int_sta = 0;
    // INT_STA (0x37) страница 0: бит 0 - GYRO_DATA_RDY, бит 1 - ACC_DATA_RDY
    bno055_readData(0x37, &int_sta, 1);
    // Можно проверять оба бита, если нужен любой из датчиков:
    return (int_sta & 0x03) != 0;
}
