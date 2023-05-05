
#include "include/driver_cc1200.h"

/* Send a command strobe. */
uint8_t strobe(CC1200* device, uint8_t strobe) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_select(device);
    uint8_t res = cc1200_arch_spi_rw_byte(device, strobe);
    cc1200_arch_spi_deselect(device);
    xSemaphoreGive(device->spi_semaphore);
    return res;
}
/*---------------------------------------------------------------------------*/
/* Reset CC1200. */
esp_err_t cc1200_reset(CC1200* device) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_rw_byte(device, CC1200_SRES);
    /*
     * Here we should wait for SO to go low again.
     * As we don't have access to this pin we just wait for 100µs.
     */
    vTaskDelay(10);
    xSemaphoreGive(device->spi_semaphore);
    return ESP_OK;
}

/*---------------------------------------------------------------------------*/
/* Write a single byte to the specified address. */
uint8_t single_write(CC1200* device, uint16_t addr, uint8_t val) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_select(device);
    if (CC1200_IS_EXTENDED_ADDR(addr)) {
        cc1200_arch_spi_rw_byte(device, CC1200_EXTENDED_WRITE_CMD);
        cc1200_arch_spi_rw_byte(device, (uint8_t) addr);
    } else {
        cc1200_arch_spi_rw_byte(device, addr | CC1200_WRITE_BIT);
    }
    uint8_t res = cc1200_arch_spi_rw_byte(device, val);
    cc1200_arch_spi_deselect(device);
    xSemaphoreGive(device->spi_semaphore);
    return res;
}
/*---------------------------------------------------------------------------*/
/* Read a single byte from the specified address. */
uint8_t single_read(CC1200* device, uint16_t addr) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_select(device);
    if (CC1200_IS_EXTENDED_ADDR(addr)) {
        cc1200_arch_spi_rw_byte(device, CC1200_EXTENDED_READ_CMD);
        cc1200_arch_spi_rw_byte(device, (uint8_t) addr);
    } else {
        cc1200_arch_spi_rw_byte(device, addr | CC1200_READ_BIT);
    }
    uint8_t res = cc1200_arch_spi_rw_byte(device, 0);
    cc1200_arch_spi_deselect(device);
    xSemaphoreGive(device->spi_semaphore);
    return res;
}
/*---------------------------------------------------------------------------*/
/* Write a burst of bytes starting at the specified address. */
esp_err_t burst_write(CC1200* device, uint16_t addr, const uint8_t *data, uint8_t data_len) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_select(device);
    if (CC1200_IS_EXTENDED_ADDR(addr)) {
        cc1200_arch_spi_rw_byte(device, CC1200_EXTENDED_BURST_WRITE_CMD);
        cc1200_arch_spi_rw_byte(device, (uint8_t) addr);
    } else {
        cc1200_arch_spi_rw_byte(device, addr | CC1200_WRITE_BIT | CC1200_BURST_BIT);
    }
    esp_err_t res = cc1200_arch_spi_rw(device, NULL, data, data_len);
    cc1200_arch_spi_deselect(device);
    xSemaphoreGive(device->spi_semaphore);
    return res;
}
/*---------------------------------------------------------------------------*/
/* Read a burst of bytes starting at the specified address. */
esp_err_t burst_read(CC1200* device, uint16_t addr, uint8_t *data, uint8_t data_len) {
    if (xSemaphoreTake(device->spi_semaphore, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;  // Wait for SPI mutex to become available
    cc1200_arch_spi_select(device);
    if (CC1200_IS_EXTENDED_ADDR(addr)) {
        cc1200_arch_spi_rw_byte(device, CC1200_EXTENDED_BURST_READ_CMD);
        cc1200_arch_spi_rw_byte(device, (uint8_t) addr);
    } else {
        cc1200_arch_spi_rw_byte(device, addr | CC1200_READ_BIT | CC1200_BURST_BIT);
    }
    esp_err_t res = cc1200_arch_spi_rw(device, data, NULL, data_len);
    cc1200_arch_spi_deselect(device);
    xSemaphoreGive(device->spi_semaphore);
    return res;
}
/*---------------------------------------------------------------------------*/
/* Write a list of register settings. */
void write_reg_settings(CC1200* device, const registerSetting_t *reg_settings, uint16_t sizeof_reg_settings) {
    int i = sizeof_reg_settings / sizeof(registerSetting_t);

    if (reg_settings != NULL) {
        while (i--) {
            single_write(device, reg_settings->addr, reg_settings->val);
            reg_settings++;
        }
    }
}
