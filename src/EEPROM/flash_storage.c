#include "flash_storage.h"
#include <string.h>
#include <stdio.h>

static FlashStorage_t flash_data_cache;
static bool cache_valid = false;

/**
 * @brief Инициализация модуля работы с Flash памятью
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef FlashStorage_Init(void)
{
    // Проверяем, есть ли валидные данные во Flash
    if (FlashStorage_IsDataValid()) {
        // Загружаем данные в кэш
        return FlashStorage_LoadData(&flash_data_cache);
    } else {
        // Устанавливаем значения по умолчанию
        FlashStorage_SetDefaultValues(&flash_data_cache);
        cache_valid = true;
        printf("[FLASH] No valid data found, using defaults\r\n");
        return HAL_OK;
    }
}

/**
 * @brief Вычисление контрольной суммы данных
 * @param data указатель на структуру данных
 * @retval контрольная сумма
 */
uint32_t FlashStorage_CalculateChecksum(const FlashStorage_t* data)
{
    uint32_t checksum = 0;
    
    // Вычисляем контрольную сумму только для стабильных полей
    // Исключаем last_binding_time и checksum
    
    // Magic number (4 bytes)
    const uint8_t* magic_ptr = (const uint8_t*)&data->magic;
    for (size_t i = 0; i < sizeof(data->magic); i++) {
        checksum += magic_ptr[i];
    }
    
    // Remote address (5 bytes)
    for (size_t i = 0; i < 5; i++) {
        checksum += data->remote_address[i];
    }
    
    // Binding count (4 bytes)
    const uint8_t* count_ptr = (const uint8_t*)&data->binding_count;
    for (size_t i = 0; i < sizeof(data->binding_count); i++) {
        checksum += count_ptr[i];
    }
    
    // Oscillation angle (2 bytes)
    const uint8_t* angle_ptr = (const uint8_t*)&data->oscillation_angle;
    for (size_t i = 0; i < sizeof(data->oscillation_angle); i++) {
        checksum += angle_ptr[i];
    }
    
    // Reserved bytes
    for (size_t i = 0; i < sizeof(data->reserved); i++) {
        checksum += data->reserved[i];
    }
    
    printf("[FLASH] Calculated checksum (stable fields only): 0x%08X\r\n", (unsigned int)checksum);
    
    return checksum;
}

/**
 * @brief Установка значений по умолчанию
 * @param data указатель на структуру данных
 */
void FlashStorage_SetDefaultValues(FlashStorage_t* data)
{
    data->magic = FLASH_STORAGE_MAGIC;
    
    // Адрес пульта по умолчанию
    data->remote_address[0] = 0x11;
    data->remote_address[1] = 0x22;
    data->remote_address[2] = 0x33;
    data->remote_address[3] = 0x44;
    data->remote_address[4] = 0x55;
    
    // Инициализируем счетчики
    data->binding_count = 0;
    data->last_binding_time = 0;
    
    // Устанавливаем угол размаха по умолчанию
    data->oscillation_angle = 180;  // Значение по умолчанию: 180 шагов
    
    // Очищаем резерв
    memset(data->reserved, 0, sizeof(data->reserved));
    
    // Вычисляем контрольную сумму
    data->checksum = FlashStorage_CalculateChecksum(data);
}

/**
 * @brief Проверка валидности данных во Flash
 * @retval true если данные валидны
 */
bool FlashStorage_IsDataValid(void)
{
    FlashStorage_t* flash_ptr = (FlashStorage_t*)FLASH_STORAGE_START_ADDR;
    
    printf("[FLASH] Checking Flash validity at 0x%08X\r\n", (unsigned int)FLASH_STORAGE_START_ADDR);
    printf("[FLASH] Magic number in Flash: 0x%08X (expected: 0x%08X)\r\n", 
           (unsigned int)flash_ptr->magic, (unsigned int)FLASH_STORAGE_MAGIC);
    
    // Проверяем магическое число
    if (flash_ptr->magic != FLASH_STORAGE_MAGIC) {
        printf("[FLASH] Magic number mismatch!\r\n");
        return false;
    }
    
    // Проверяем контрольную сумму
    uint32_t calculated_checksum = FlashStorage_CalculateChecksum(flash_ptr);
    printf("[FLASH] Checksum in Flash: 0x%08X, calculated: 0x%08X\r\n",
           (unsigned int)flash_ptr->checksum, (unsigned int)calculated_checksum);
    
    if (flash_ptr->checksum != calculated_checksum) {
        printf("[FLASH] Checksum mismatch! Calculated: 0x%08X, stored: 0x%08X\r\n",
               (unsigned int)calculated_checksum, (unsigned int)flash_ptr->checksum);
        FlashStorage_DebugDumpFlash();
        return false;
    }
    
    printf("[FLASH] Flash data is valid!\r\n");
    return true;
}

/**
 * @brief Стирание страницы Flash памяти
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef FlashStorage_EraseData(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;
    
    // Разблокируем Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    
    // Настраиваем параметры стирания
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_STORAGE_START_ADDR;
    eraseInit.NbPages = 1;
    
    // Стираем страницу
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    
    // Блокируем Flash
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        printf("[FLASH] Erase failed with error: 0x%X\r\n", (unsigned int)pageError);
    }
    
    return status;
}

/**
 * @brief Сохранение данных во Flash память
 * @param data указатель на данные для сохранения
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef FlashStorage_SaveData(const FlashStorage_t* data)
{
    HAL_StatusTypeDef status;
    
    printf("[FLASH] Saving data to Flash...\r\n");
    printf("[FLASH] Data to save:\r\n");
    printf("[FLASH] Magic: 0x%08X\r\n", (unsigned int)data->magic);
    printf("[FLASH] Remote address: %02X %02X %02X %02X %02X\r\n",
           data->remote_address[0], data->remote_address[1], data->remote_address[2],
           data->remote_address[3], data->remote_address[4]);
    printf("[FLASH] Checksum: 0x%08X\r\n", (unsigned int)data->checksum);
    
    // Вычисляем контрольную сумму для сравнения
    uint32_t calc_checksum = FlashStorage_CalculateChecksum(data);
    printf("[FLASH] Calculated checksum before save: 0x%08X\r\n", (unsigned int)calc_checksum);
    
    // Стираем старые данные
    status = FlashStorage_EraseData();
    if (status != HAL_OK) {
        printf("[FLASH] Failed to erase Flash\r\n");
        return status;
    }
    
    // Разблокируем Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    
    // Записываем данные по 32-битным словам
    const uint32_t* data_ptr = (const uint32_t*)data;
    uint32_t address = FLASH_STORAGE_START_ADDR;
    size_t words_count = (sizeof(FlashStorage_t) + 3) / 4; // Округляем вверх
    
    for (size_t i = 0; i < words_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_ptr[i]);
        if (status != HAL_OK) {
            printf("[FLASH] Write failed at address 0x%X\r\n", (unsigned int)address);
            break;
        }
        address += 4;
    }
    
    // Блокируем Flash
    HAL_FLASH_Lock();
    
    if (status == HAL_OK) {
        // Обновляем кэш
        memcpy(&flash_data_cache, data, sizeof(FlashStorage_t));
        cache_valid = true;
        printf("[FLASH] Data saved successfully\r\n");
    }
    
    return status;
}

/**
 * @brief Загрузка данных из Flash памяти
 * @param data указатель на структуру для загрузки данных
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef FlashStorage_LoadData(FlashStorage_t* data)
{
    if (!FlashStorage_IsDataValid()) {
        printf("[FLASH] Invalid data in Flash\r\n");
        return HAL_ERROR;
    }
    
    // Копируем данные из Flash
    FlashStorage_t* flash_ptr = (FlashStorage_t*)FLASH_STORAGE_START_ADDR;
    memcpy(data, flash_ptr, sizeof(FlashStorage_t));
    
    // Обновляем кэш
    memcpy(&flash_data_cache, data, sizeof(FlashStorage_t));
    cache_valid = true;
    
    printf("[FLASH] Data loaded successfully\r\n");
    return HAL_OK;
}



/**
 * @brief Сохранение адреса пульта в Flash память
 * @param address адрес пульта (5 байт)
 * @retval HAL_OK если успешно
 */
HAL_StatusTypeDef FlashStorage_SaveRemoteAddress(const uint8_t address[5]) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    // Обновляем адрес пульта в кэше
    memcpy(flash_data_cache.remote_address, address, 5);
    flash_data_cache.binding_count++;
    flash_data_cache.last_binding_time = HAL_GetTick();
    
    // Пересчитываем контрольную сумму с новым алгоритмом
    flash_data_cache.checksum = FlashStorage_CalculateChecksum(&flash_data_cache);
    
    printf("[Flash] Saving remote address: 0x%02X%02X%02X%02X%02X\r\n",
           address[0], address[1], address[2], address[3], address[4]);
    
    // Сохраняем в Flash
    return FlashStorage_SaveData(&flash_data_cache);
}

/**
 * @brief Загрузка адреса пульта из Flash памяти
 * @param address буфер для адреса пульта (5 байт)
 * @retval HAL_OK если успешно
 */
HAL_StatusTypeDef FlashStorage_LoadRemoteAddress(uint8_t address[5]) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    memcpy(address, flash_data_cache.remote_address, 5);
    
    printf("[Flash] Loaded remote address: 0x%02X%02X%02X%02X%02X\r\n",
           address[0], address[1], address[2], address[3], address[4]);
    
    return HAL_OK;
}

/**
 * @brief Проверка наличия валидного адреса пульта
 * @retval true если адрес пульта сохранен и не нулевой
 */
bool FlashStorage_HasValidRemoteAddress(void) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            return false;
        }
    }
    
    // Дополнительная проверка целостности данных из Flash
    if (!FlashStorage_IsDataValid()) {
        printf("[FLASH] Flash data validation failed\r\n");
        return false;
    }
    
    // Проверяем, что адрес не нулевой (все нули означают "не привязан")
    for (int i = 0; i < 5; i++) {
        if (flash_data_cache.remote_address[i] != 0x00) {
            printf("[FLASH] Valid remote address found in flash\r\n");
            return true;
        }
    }

    return false;
}

/**
 * @brief Вывод информации о сохраненном адресе пульта
 */
void FlashStorage_PrintStoredAddress(void) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            printf("[Flash] Failed to initialize Flash storage\r\n");
            return;
        }
    }
    
    printf("[Flash] === Stored Remote Control Info ===\r\n");
    printf("[Flash] Remote address: 0x%02X%02X%02X%02X%02X\r\n",
           flash_data_cache.remote_address[0], flash_data_cache.remote_address[1], 
           flash_data_cache.remote_address[2], flash_data_cache.remote_address[3], 
           flash_data_cache.remote_address[4]);
    printf("[Flash] Binding count: %u\r\n", (unsigned int)flash_data_cache.binding_count);
    printf("[Flash] Last binding time: %u ms\r\n", (unsigned int)flash_data_cache.last_binding_time);
    
    if (FlashStorage_HasValidRemoteAddress()) {
        printf("[Flash] Status: BOUND to remote control\r\n");
    } else {
        printf("[Flash] Status: NOT BOUND (waiting for pairing)\r\n");
    }
    printf("[Flash] ===================================\r\n");
}

/**
 * @brief Отладочная функция для вывода содержимого Flash памяти
 */
void FlashStorage_DebugDumpFlash(void) {
    FlashStorage_t* flash_ptr = (FlashStorage_t*)FLASH_STORAGE_START_ADDR;
    
    printf("[FLASH] === DEBUG FLASH DUMP ===\r\n");
    printf("[FLASH] Magic: 0x%08X\r\n", (unsigned int)flash_ptr->magic);
    printf("[FLASH] Remote address: 0x%02X%02X%02X%02X%02X\r\n",
           flash_ptr->remote_address[0], flash_ptr->remote_address[1], 
           flash_ptr->remote_address[2], flash_ptr->remote_address[3], 
           flash_ptr->remote_address[4]);
    printf("[FLASH] Binding count: %u\r\n", (unsigned int)flash_ptr->binding_count);
    printf("[FLASH] Last binding time: %u\r\n", (unsigned int)flash_ptr->last_binding_time);
    printf("[FLASH] Oscillation angle: %d steps\r\n", flash_ptr->oscillation_angle);
    printf("[FLASH] Stored checksum: 0x%08X\r\n", (unsigned int)flash_ptr->checksum);
    printf("[FLASH] Structure size: %u bytes\r\n", (unsigned int)sizeof(FlashStorage_t));
    printf("[FLASH] === END DUMP ===\r\n");
}

/**
 * @brief Принудительная очистка Flash памяти (для отладки)
 */
HAL_StatusTypeDef FlashStorage_ForceErase(void) {
    printf("[FLASH] Force erasing Flash memory...\r\n");
    
    HAL_StatusTypeDef status = FlashStorage_EraseData();
    if (status == HAL_OK) {
        printf("[FLASH] Flash memory erased successfully\r\n");
        // Сбрасываем кэш
        cache_valid = false;
        
        // Инициализируем заново с дефолтными значениями
        FlashStorage_SetDefaultValues(&flash_data_cache);
        cache_valid = true;
    } else {
        printf("[FLASH] Failed to erase Flash memory\r\n");
    }
    
    return status;
}

/**
 * @brief Сохранение угла размаха мотора в Flash память
 * @param angle угол размаха в шагах
 * @retval HAL_OK если успешно
 */
HAL_StatusTypeDef FlashStorage_SaveOscillationAngle(int16_t angle) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    // Обновляем угол размаха в кэше
    flash_data_cache.oscillation_angle = angle;
    
    // Пересчитываем контрольную сумму с новым углом
    flash_data_cache.checksum = FlashStorage_CalculateChecksum(&flash_data_cache);
    
    printf("[Flash] Saving oscillation angle: %d steps\r\n", angle);
    
    // Сохраняем в Flash
    return FlashStorage_SaveData(&flash_data_cache);
}

/**
 * @brief Загрузка угла размаха мотора из Flash памяти
 * @param angle указатель на переменную для загрузки угла
 * @retval HAL_OK если успешно
 */
HAL_StatusTypeDef FlashStorage_LoadOscillationAngle(int16_t* angle) {
    if (!cache_valid) {
        if (FlashStorage_Init() != HAL_OK) {
            return HAL_ERROR;
        }
    }
    
    if (angle == NULL) {
        return HAL_ERROR;
    }
    
    *angle = flash_data_cache.oscillation_angle;
    
    printf("[Flash] Loaded oscillation angle: %d steps\r\n", *angle);
    
    return HAL_OK;
}
