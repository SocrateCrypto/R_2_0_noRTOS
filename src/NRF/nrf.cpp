#include "nrf.h"
#include "main.h"
#include "EEPROM/flash_storage.h"

#include "NRF/NRF24_conf.h"
#include "NRF/NRF24_reg_addresses.h"
#include "NRF/NRF24.h"

#include <stdint.h>
#include <cstdio>
#include <string.h>
#include <cstring> // Для memcpy

// Определение глобальной переменной для состояний виртуальных кнопок
RadioButtons RadioButtonsStates = {0}; // Инициализируем все биты нулями

#define PLD_S 32  // Define PLD_S with an appropriate value
#define RADIO_TIMEOUT 1500 // Увеличенный таймаут 1.5 секунды для надежности

// Константы для привязки
#define BIND_CHANNEL 76                                    // Канал для привязки (тот же что и рабочий)
#define BIND_PACKET_TYPE 0xBB                             // Тип пакета привязки
#define BIND_RESPONSE_TYPE 0xBC                           // Тип ответа на привязку "BIND_OK"

// Адреса
uint8_t tx_addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55};     // Дефолтный адрес ПРИЕМНИКА (отличается от передатчика)
uint8_t bind_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};   // Стандартный адрес для привязки

// Структура пакета привязки
typedef struct {
    uint8_t packet_type;     // 0xBB = Bind request
    uint8_t remote_address[5]; // Адрес пульта для связи
    uint8_t checksum;        // Простая контрольная сумма
} bind_packet_t;

// Структура ответа на привязку
typedef struct {
    uint8_t packet_type;     // 0xBC = Bind response "BIND_OK"
    uint8_t receiver_address[5]; // Адрес приемника (подтверждение)
    uint8_t checksum;        // Простая контрольная сумма
} bind_response_t;

uint16_t data = 0;

uint8_t rx_ack_pld[PLD_S] = {"OK"};

volatile uint8_t data_ready = 0;

uint8_t dataR[PLD_S];
uint32_t processed_data = 0;  // Counter for processed packets
uint32_t last_packet_time = 0; // Время последнего принятого пакета
static uint8_t timeout_reported = 0; // Флаг для отслеживания вывода по таймауту

// Переменные для режима привязки
static uint8_t binding_mode = 0;      // Флаг режима привязки
static uint8_t bind_response_count = 0; // Счетчик отправленных BIND_OK ответов
static uint32_t last_bind_packet_time = 0; // Время последнего bind-пакета
#define MAX_BIND_RESPONSES 3          // Максимальное количество BIND_OK ответов
#define BIND_PACKET_TIMEOUT 1000      // Таймаут для bind-пакетов (5 секунд)

// Функция обнуления всех кнопок
static void reset_radio_buttons(void) {
    RadioButtonsStates.left = 0;
    RadioButtonsStates.right = 0;
    RadioButtonsStates.left_end_right = 0;
    RadioButtonsStates.both = 0;
}

// Функция расчета контрольной суммы для пакета привязки
static uint8_t calculate_bind_checksum(const bind_packet_t* packet) {
    uint8_t checksum = packet->packet_type;
    for(int i = 0; i < 5; i++) {
        checksum += packet->remote_address[i];
    }
    return checksum;
}

// Функция расчета контрольной суммы для ответа на привязку
static uint8_t calculate_bind_response_checksum(const bind_response_t* packet) {
    uint8_t checksum = packet->packet_type;
    for(int i = 0; i < 5; i++) {
        checksum += packet->receiver_address[i];
    }
    return checksum;
}

// Функция отправки ответа на привязку
void nrf_send_bind_response(void) {
    uint8_t response_buffer[PLD_S] = {0}; // Буфер фиксированного размера
    bind_response_t* response = (bind_response_t*)response_buffer;
    
    response->packet_type = BIND_RESPONSE_TYPE;
    memcpy(response->receiver_address, tx_addr, 5);  // Отправляем новый адрес назад
    response->checksum = calculate_bind_response_checksum(response);
    
    printf("Sending BIND_OK response...\r\n");
    printf("Response type: 0x%02X\r\n", response->packet_type);
    printf("Response address: 0x%02X%02X%02X%02X%02X\r\n",
           response->receiver_address[0], response->receiver_address[1],
           response->receiver_address[2], response->receiver_address[3],
           response->receiver_address[4]);
    printf("Response checksum: 0x%02X\r\n", response->checksum);
    
    // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Переключаемся в режим передачи
    printf("Switching to TX mode for bind response...\r\n");
    nrf24_stop_listen();  // Останавливаем прием
    ce_low();
    
    // ВАЖНО: Настраиваем TX pipe на bind_addr для ответа!
    printf("Setting TX pipe to bind address for response\r\n");
    nrf24_open_tx_pipe(bind_addr);  // Отправляем ответ на bind_addr
    
    HAL_Delay(10);        // Небольшая задержка для стабилизации
    
    // Отправляем ответ привязки через NRF24
    nrf24_transmit(response_buffer, PLD_S);  // Фактическая отправка пакета
    printf("BIND_OK response transmitted!\r\n");
    
    // Возвращаемся в режим приема
    printf("Returning to RX mode...\r\n");
    HAL_Delay(10);        // Даем время на завершение передачи
    nrf24_listen();       // Включаем режим приема
    ce_high();
    printf("BIND_OK response sent and back to RX mode!\r\n");
}

// Функция обработки пакета привязки
static void process_bind_packet(const uint8_t* data) {
    bind_packet_t* bind_packet = (bind_packet_t*)data;
    
    // Проверяем тип пакета
    if(bind_packet->packet_type != BIND_PACKET_TYPE) {
        printf("Invalid bind packet type: 0x%02X\r\n", bind_packet->packet_type);
        return;
    }
    
    // Проверяем контрольную сумму
    uint8_t calculated_checksum = calculate_bind_checksum(bind_packet);
    if(calculated_checksum != bind_packet->checksum) {
        printf("Bind packet checksum error: calc=0x%02X, recv=0x%02X\r\n", 
               calculated_checksum, bind_packet->checksum);
        return;
    }
    
    printf("*** VALID BIND PACKET RECEIVED! ***\r\n");
    printf("Remote address: 0x%02X%02X%02X%02X%02X\r\n",
           bind_packet->remote_address[0], bind_packet->remote_address[1],
           bind_packet->remote_address[2], bind_packet->remote_address[3],
           bind_packet->remote_address[4]);
    
    // Копируем новый адрес в рабочий адрес
    memcpy(tx_addr, bind_packet->remote_address, 5);
    
    // Сохраняем адрес пульта в Flash память
    if (FlashStorage_SaveRemoteAddress(bind_packet->remote_address) == HAL_OK) {
        printf("[FLASH] Remote address saved successfully\r\n");
    } else {
        printf("[FLASH] Failed to save remote address\r\n");
    }
    
    printf("Binding successful! New working address set.\r\n");
    printf("New address: 0x%02X%02X%02X%02X%02X\r\n",
           tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
    
    // Отправляем ответ BIND_OK (пока еще на bind адресе)
    nrf_send_bind_response();
    
    // Мигаем лампочкой LAMP 10 раз с коротким периодом
    printf("Starting LAMP blinking sequence...\r\n");
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_SET);
        HAL_Delay(100);  // Короткая вспышка 100мс
        nrf_send_bind_response();
        HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);  // Короткая пауза 100мс
    }
    printf("LAMP blinking sequence completed!\r\n");
    
    
    
    printf("*** BINDING COMPLETE! ***\r\n");
    
    // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Немедленно переключаемся на новый рабочий адрес
    printf("Switching to new working address...\r\n");
    nrf24_stop_listen();
    ce_low();
    
    // Конфигурируем новый рабочий адрес
    nrf24_open_tx_pipe(tx_addr);
    nrf24_open_rx_pipe(0, tx_addr);
    
    // Очищаем буферы
    nrf24_flush_tx();
    nrf24_flush_rx();
    nrf24_clear_rx_dr();
    nrf24_clear_tx_ds();
    nrf24_clear_max_rt();
    
    // Возвращаемся в режим приема на новом адресе
    nrf24_listen();
    ce_high();
    
    printf("Now listening on new address: 0x%02X%02X%02X%02X%02X\r\n",
           tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
    
    // Автоматически выходим из режима привязки после успешной привязки
    binding_mode = 0;
    printf("Automatically exiting binding mode after successful binding\r\n");
    
    // Адрес пульта уже сохранен в Flash память выше
    
    processed_data++; // Увеличиваем счетчик обработанных пакетов
}

// Функция входа в режим привязки (вызывается из машины состояний)
void nrf_enter_binding_mode(void) {
    binding_mode = 1;
    bind_response_count = 0;  // Сбрасываем счетчик BIND_OK ответов
    
    printf("\r\n*** RECEIVER ENTERING BINDING MODE ***\r\n");
    printf("Binding mode flag set to: %d\r\n", binding_mode);
    printf("Waiting for remote control binding packet...\r\n");
    
    // Переключаемся на стандартный адрес привязки
    nrf24_stop_listen();
    ce_low();
    
    // Включаем auto ACK для быстрой привязки
    nrf24_auto_ack_all(enable);
    
    // ДОБАВЛЯЕМ ДИАГНОСТИКУ КАНАЛА
    printf("DEBUG: Setting channel to %d\r\n", BIND_CHANNEL);
    nrf24_set_channel(BIND_CHANNEL);  // Убеждаемся, что канал установлен правильно
    
    // ДОБАВЛЯЕМ ДИАГНОСТИКУ АДРЕСА
    printf("DEBUG: Setting bind address: 0x%02X%02X%02X%02X%02X\r\n",
           bind_addr[0], bind_addr[1], bind_addr[2], bind_addr[3], bind_addr[4]);
    
    nrf24_open_tx_pipe(bind_addr);
    nrf24_open_rx_pipe(0, bind_addr);
    
    // Очищаем буферы
    nrf24_flush_tx();
    nrf24_flush_rx();
    nrf24_clear_rx_dr();
    nrf24_clear_tx_ds();
    nrf24_clear_max_rt();
    
    // ДОБАВЛЯЕМ ДИАГНОСТИКУ КОНФИГУРАЦИИ
    uint8_t current_channel = nrf24_get_channel();
    printf("DEBUG: Current channel: %d\r\n", current_channel);
    
    nrf24_listen();
    ce_high();
    
    printf("Listening on binding address: 0x%02X%02X%02X%02X%02X\r\n", 
           bind_addr[0], bind_addr[1], bind_addr[2], bind_addr[3], bind_addr[4]);
    printf("Channel: %d, Data rate: 250kbps (Auto ACK enabled)\r\n", current_channel);
    printf("*** BINDING MODE ACTIVE ***\r\n\r\n");
}

void nrf_loop(uint8_t irq)
{
    uint32_t current_time = HAL_GetTick();
    
    // Проверяем таймаут
    if(current_time - last_packet_time > RADIO_TIMEOUT) {
        if (!timeout_reported) { // Выводим сообщение только один раз при таймауте
            reset_radio_buttons();
            printf("Connection lost! Button states: L:%d,R:%d,LR:%d,B:%d #%lu\r\n",
                   RadioButtonsStates.left,
                   RadioButtonsStates.right,
                   RadioButtonsStates.left_end_right,
                   RadioButtonsStates.both,
                   processed_data);
            timeout_reported = 1;
        }
    } else {
        timeout_reported = 0; // Сбрасываем флаг, если связь восстановилась
    }

    if (irq) {
      irq = 0;  // Сбрасываем флаг
      
      // Быстрая обработка прерывания без блокирующих операций
      uint8_t status = nrf24_r_status();
      
      // Проверяем получение данных (RX_DR бит)
      if (status & (1 << 6)) {  // RX_DR = бит 6
        
        if (nrf24_data_available()) {
          nrf24_receive(dataR, PLD_S);
          dataR[PLD_S - 1] = '\0';
          
          // Отмечаем, что данные готовы к обработке
          data_ready = 1;
          last_packet_time = current_time; // Обновляем время последнего пакета
        }
        
        // Очищаем флаг RX_DR
        nrf24_clear_rx_dr();
      }
      
      // Проверяем успешную отправку (TX_DS бит)
      if (status & (1 << 5)) {  // TX_DS = бит 5
        nrf24_clear_tx_ds();
      }
      
      // Проверяем превышение лимита повторов (MAX_RT бит)
      if (status & (1 << 4)) {  // MAX_RT = бит 4
        printf("Warning: Max retries reached, clearing buffers\r\n");
        nrf24_clear_max_rt();
        nrf24_flush_tx();
        nrf24_flush_rx();  // Также очищаем RX буфер
      }
    }
    
    // Обработка полученных данных в основном цикле (вне прерывания)
    if(data_ready)
    {
      data_ready = 0;
      
      printf("Data received (%d bytes): ", PLD_S);
      for(int i = 0; i < 8; i++) {  // Выводим первые 8 байт
          printf("0x%02X ", dataR[i]);
      }
      printf("\r\n");
      
      // Проверяем, находимся ли мы в режиме привязки
      if(binding_mode) {
          printf("Processing in BINDING MODE (flag=%d)\r\n", binding_mode);
          
          // В режиме привязки обрабатываем пакеты привязки
          if(dataR[0] == BIND_PACKET_TYPE) {
              printf("*** BIND PACKET DETECTED! ***\r\n");
              process_bind_packet(dataR);
          } else {
              printf("Non-bind packet in binding mode (type=0x%02X), ignoring\r\n", dataR[0]);
              
              // Очищаем буферы от "мусорных" пакетов в режиме биндинга
              nrf24_flush_rx();
              nrf24_clear_rx_dr();
              printf("RX buffers cleared due to non-bind packet in binding mode\r\n");
          }
      } else {
         // printf("Processing in NORMAL MODE\r\n");
          
          // Специальная обработка: если получили bind-пакет в normal mode
          if(dataR[0] == BIND_PACKET_TYPE) {
              printf("WARNING: Received bind packet in NORMAL MODE!\r\n");
              
              uint32_t current_time = HAL_GetTick();
              
              // Проверяем таймаут с момента последнего bind-пакета
              if(current_time - last_bind_packet_time > BIND_PACKET_TIMEOUT) {
                  printf("Bind packet timeout expired. Resetting response counter.\r\n");
                  bind_response_count = 0;  // Сбрасываем счетчик после таймаута
              }
              
              last_bind_packet_time = current_time;
              
              // Ограничиваем количество BIND_OK ответов
              if(bind_response_count < MAX_BIND_RESPONSES) {
                  printf("This means transmitter is still in binding mode.\r\n");
                  printf("Sending BIND_OK response #%d to help transmitter exit binding...\r\n", bind_response_count + 1);
                  
                  // Отправляем BIND_OK ответ
                  nrf_send_bind_response();
                  bind_response_count++;
              } else {
                  printf("Already sent %d BIND_OK responses. Ignoring further bind packets.\r\n", MAX_BIND_RESPONSES);
                  printf("Transmitter should have received the response by now.\r\n");
                  printf("Please restart transmitter or wait %d seconds for timeout.\r\n", BIND_PACKET_TIMEOUT/1000);
              }
              
              // Не обрабатываем как кнопки
              processed_data++;  // Увеличиваем счетчик
              return;
          }
          
          // В обычном режиме обрабатываем пакеты кнопок
          // Вызываем функцию обновления состояния кнопок
          update_radio_buttons((const char*)dataR);
          
          // Выводим состояние структуры кнопок
          printf("Button states: L:%d,R:%d,LR:%d,B:%d #%lu\r\n",
                 RadioButtonsStates.left,
                 RadioButtonsStates.right,
                 RadioButtonsStates.left_end_right,
                 RadioButtonsStates.both,
                 processed_data);
                 
          processed_data++;
      }
    }
    
}

void nrf_init_next(void)
{

printf("NRF24 initialized\r\n");
  
  // Настройка NRF24 как приемника с улучшенной надежностью
  nrf24_auto_ack_all(auto_ack);  // Включаем автоподтверждения
  nrf24_en_ack_pld(disable);
  nrf24_en_dyn_ack(disable);
  nrf24_dpl(disable);
  
  // Включаем 2-байтовый CRC для лучшего обнаружения ошибок
  nrf24_set_crc(1, 0);  // 2-байтовый CRC для лучшей надежности
  nrf24_tx_pwr(_0dbm);           // Максимальная мощность передачи
  nrf24_data_rate(_250kbps);     // Низкая скорость для лучшей чувствительности
  nrf24_set_channel(76);         // Канал 76 (2.476 ГГц) - менее загружен WiFi
  nrf24_set_addr_width(5);       // 5-байтовый адрес
  
  // Отключение динамических пакетов для всех каналов
  nrf24_set_rx_dpl(0, disable);
  nrf24_set_rx_dpl(1, disable);
  nrf24_set_rx_dpl(2, disable);
  nrf24_set_rx_dpl(3, disable);
  nrf24_set_rx_dpl(4, disable);
  nrf24_set_rx_dpl(5, disable);
  
  nrf24_pipe_pld_size(0, PLD_S);
  
  // Увеличиваем задержку и количество повторов для надежности
  nrf24_auto_retr_delay(15);     // Задержка 4000 мкс (максимальная)
  nrf24_auto_retr_limit(15);     // 15 повторов (максимальное значение)
  
  nrf24_open_tx_pipe(tx_addr);
  nrf24_open_rx_pipe(0, tx_addr);
  
  // Очищаем буферы перед началом работы
  nrf24_flush_tx();
  nrf24_flush_rx();
  
  // Очищаем все флаги состояния
  nrf24_clear_rx_dr();
  nrf24_clear_tx_ds();
  nrf24_clear_max_rt();
  
  nrf24_listen();  // Включаем режим приема
  ce_high();
  
  printf("NRF24 configured as receiver on channel 76\r\n");
  printf("Address: 0x%02X%02X%02X%02X%02X\r\n", 
         tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
  printf("Data rate: 250 kbps, Payload size: %d bytes\r\n", PLD_S);



}

void update_radio_buttons(const char* data) {
    int L = 0, R = 0, B = 0, LR = 0;
    
    // Парсим строку формата "L:1,R:0,B:0,LR:0"
    if(sscanf(data, "L:%d,R:%d,B:%d,LR:%d", &L, &R, &B, &LR) == 4) {
        // Обновляем состояния кнопок
        RadioButtonsStates.left = L;
        RadioButtonsStates.right = R;
        RadioButtonsStates.both = B;
        
        // Если обе кнопки нажаты, устанавливаем left_end_right
        RadioButtonsStates.left_end_right = LR ? 1 : 0;
    }
}

// Функция выхода из режима привязки
void nrf_exit_binding_mode(void) {
    binding_mode = 0;
    bind_response_count = 0;  // Сбрасываем счетчик BIND_OK ответов
    
    printf("Exiting binding mode\r\n");
    
    // Возвращаемся на рабочий адрес
    nrf24_stop_listen();
    ce_low();
    
    // Включаем обратно auto ACK для нормальной работы
    nrf24_auto_ack_all(auto_ack);
    
    nrf24_open_tx_pipe(tx_addr);
    nrf24_open_rx_pipe(0, tx_addr);
    
    // Очищаем буферы
    nrf24_flush_tx();
    nrf24_flush_rx();
    nrf24_clear_rx_dr();
    nrf24_clear_tx_ds();
    nrf24_clear_max_rt();
    
    nrf24_listen();
    ce_high();
    
    printf("Returned to working address: 0x%02X%02X%02X%02X%02X (Auto ACK enabled)\r\n", 
           tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
}

// Функция проверки, находимся ли в режиме привязки
uint8_t nrf_is_binding_mode(void) {
    return binding_mode;
}

// Функция для установки рабочего адреса NRF24
void nrf_set_working_address(const uint8_t address[5]) {
    // Копируем новый адрес в рабочий адрес
    memcpy(tx_addr, address, 5);
    
    printf("[NRF] Working address set to: 0x%02X%02X%02X%02X%02X\r\n",
           tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
}
