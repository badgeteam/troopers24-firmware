#pragma once

#include <esp_err.h>
#include <stdint.h>

#include "types.h"
#include "cc1200-arch.h"
#include "cc1200-conf.h"
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "driver_cc1200_misc.h"
#include "driver_cc1200_spi.h"
#include "driver_cc1200_statemachine.h"
#include "contiki.h"

__BEGIN_DECLS

void      driver_cc1200_configure(CC1200* device);
esp_err_t cc1200_init(CC1200* device);
uint8_t   driver_cc1200_read_reg(CC1200* device, uint16_t addr);
uint8_t   driver_cc1200_write_reg(CC1200* device, uint16_t addr, uint8_t data);
int       cc1200_rx_interrupt(void);
esp_err_t driver_cc1200_tx_packet(uint8_t *data, uint8_t len);
esp_err_t driver_cc1200_rx_packet(cc1200_message *msg);
void      driver_cc1200_set_txpower(CC1200* device, int8_t txpower_dbm);
void      driver_cc1200_set_cca_threshold(CC1200* device, int8_t threshold_dbm);
int       driver_cc1200_set_frequency(uint32_t freq);
uint32_t  driver_cc1200_get_frequency(CC1200* device);
int       driver_cc1200_get_state(CC1200* device);
int       driver_cc1200_set_frequency(uint32_t freq);
esp_err_t driver_cc1200_on();
esp_err_t driver_cc1200_off();

__END_DECLS
