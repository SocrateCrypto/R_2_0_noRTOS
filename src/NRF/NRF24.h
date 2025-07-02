/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef NRF_24_H
#define NRF_24_H

#include <stdint.h>
#include "main.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constants for configuration
#define no_crc 0
#define en_crc 1
#define _1byte 0
#define _2byte 1
#define _1mbps 0
#define _2mbps 1
#define _250kbps 2
#define n18dbm 0
#define n12dbm 1
#define n6dbm 2
#define _0dbm 3
#define auto_ack 1
#define no_auto_ack 0
#define enable 1
#define disable 0

// Basic functions
void ce_low(void);
void ce_high(void);
void csn_low(void);
void csn_high(void);
void nrf24_init(void);

// Configuration functions
void nrf24_rx_mode(void);
void nrf24_tx_mode(void);
void nrf24_listen(void);

// Radio settings
void nrf24_auto_ack_all(uint8_t en);
void nrf24_en_ack_pld(uint8_t en);
void nrf24_en_dyn_ack(uint8_t en);
void nrf24_dpl(uint8_t en);
void nrf24_set_crc(uint8_t en_crc_value, uint8_t crc_bytes);
void nrf24_tx_pwr(uint8_t pwr);
void nrf24_data_rate(uint8_t rate);
void nrf24_set_channel(uint8_t ch);
void nrf24_set_addr_width(uint8_t width);
void nrf24_set_rx_dpl(uint8_t pipe, uint8_t en);
void nrf24_pipe_pld_size(uint8_t pipe, uint8_t size);
void nrf24_auto_retr_delay(uint8_t delay);
void nrf24_auto_retr_limit(uint8_t count);

// TX/RX functions
void nrf24_open_tx_pipe(uint8_t *addr);
void nrf24_open_rx_pipe(uint8_t pipe, uint8_t *addr);
uint8_t nrf24_data_available(void);
void nrf24_receive(uint8_t *data, uint8_t length);
uint8_t nrf24_r_status(void);
void nrf24_clear_rx_dr(void);
void nrf24_clear_tx_ds(void);
void nrf24_clear_max_rt(void);
void nrf24_flush_tx(void);
void nrf24_flush_rx(void);

// Power management
void nrf24_pwr_up(void);
void nrf24_pwr_dwn(void);

// Additional features
void nrf24_cls_rx_pipe(uint8_t pipe);
uint8_t nrf24_read_bit(uint8_t reg, uint8_t bit);
void nrf24_set_bit(uint8_t reg, uint8_t bit, uint8_t val);
uint8_t nrf24_r_pld_wid(void);
void nrf24_stop_listen(void);
void nrf24_auto_ack(uint8_t pipe, uint8_t ack);

// Low level functions
void nrf24_w_reg(uint8_t reg, uint8_t *data, uint8_t size);
uint8_t nrf24_r_reg(uint8_t reg, uint8_t size);
void nrf24_w_spec_cmd(uint8_t cmd);
void nrf24_w_spec_reg(uint8_t *data, uint8_t size);
void nrf24_r_spec_reg(uint8_t *data, uint8_t size);

// Utility functions
void nrf24_type_to_uint8_t(size_t in, uint8_t* out, uint16_t size);
size_t nrf24_uint8_t_to_type(uint8_t* in, uint16_t size);
uint8_t nrf24_carrier_detect(void);
uint8_t nrf24_transmit(uint8_t *data, uint8_t size);
void nrf24_transmit_no_ack(uint8_t *data, uint8_t size);
void nrf24_transmit_rx_ack_pld(uint8_t pipe, uint8_t *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* NRF_24_H */
