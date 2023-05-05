#pragma once

/* Send a command strobe. */
uint8_t strobe(CC1200* device, uint8_t strobe);
/* Reset CC1200. */
esp_err_t cc1200_reset(CC1200* device);
/* Write a single byte to the specified address. */
uint8_t single_write(CC1200* device, uint16_t addr, uint8_t value);
/* Read a single byte from the specified address. */
uint8_t single_read(CC1200* device, uint16_t addr);
/* Write a burst of bytes starting at the specified address. */
esp_err_t burst_write(CC1200* device, uint16_t addr, const uint8_t *data, uint8_t data_len);
/* Read a burst of bytes starting at the specified address. */
esp_err_t burst_read(CC1200* device, uint16_t addr, uint8_t *data, uint8_t data_len);
/* Write a list of register settings. */
void write_reg_settings(CC1200* device, const registerSetting_t *reg_settings, uint16_t sizeof_reg_settings);
