#include "nrf.h"

#include "NRF/NRF24_conf.h"
#include "NRF/NRF24_reg_addresses.h"
#include "NRF/NRF24.h"

#include <stdint.h>
#include <cstdio>

// Определение глобальной переменной для состояний виртуальных кнопок
RadioButtons RadioButtonsStates = {0}; // Инициализируем все биты нулями

#define PLD_S 32  // Define PLD_S with an appropriate value
#define RADIO_TIMEOUT 1500 // Увеличенный таймаут 1.5 секунды для надежности

uint8_t tx_addr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};

uint16_t data = 0;

uint8_t rx_ack_pld[PLD_S] = {"OK"};

volatile uint8_t data_ready = 0;

uint8_t dataR[PLD_S];
uint32_t processed_data = 0;  // Counter for processed packets
uint32_t last_packet_time = 0; // Время последнего принятого пакета
static uint8_t timeout_reported = 0; // Флаг для отслеживания вывода по таймауту

// Функция обнуления всех кнопок
static void reset_radio_buttons(void) {
    RadioButtonsStates.left = 0;
    RadioButtonsStates.right = 0;
    RadioButtonsStates.left_end_right = 0;
    RadioButtonsStates.both = 0;
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
