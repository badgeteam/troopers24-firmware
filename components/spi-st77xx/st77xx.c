/**
 * Copyright (c) 2024 Tom Bennellick & Malte Heinzelmann <malte@cybaer.ninja>
 * Based on the spi ILI9341 driver by Nicolai Electronics.
 *
 * SPDX-License-Identifier: MIT
 */

#include "include/st77xx.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>
#include <soc/spi_reg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "st77XX";

static void IRAM_ATTR st77xx_spi_pre_transfer_callback(spi_transaction_t *t) {
//    ILI9341* device = ((ILI9341*) t->user);
//    gpio_set_level(device->pin_dcx, device->dc_level);
}

const uint8_t st77xx_init_data[] = {
    // Turn off display
    ST77XX_DISPOFF,    0,
    // Exit sleep mode
    ST77XX_SLPOUT,     0,
    // MADCTL: memory data access control Old: 0x88
    ST77XX_MADCTL,     1, 0xa8, /* Page address order RGB order */
    // COLMOD: Interface Pixel format (16-bits per pixel)
    ST77XX_COLMOD,     1, 0x55, /* 16 bits per pixel */
    // PORCTRK: Porch setting
    ST77XX_PORCTRK,    5, 0x0C, 0x0C, 0x00, 0x33, 0x33, /* Back porch,  Front Porch, Separate porch control, Back porch idle, Back porch partial */
    // GCTRL: Gate Control
    ST77XX_GCTRL,      1, 0x35, /* Probably dont change */
    // VCOMS: VCOM setting
    ST77XX_VCOMS,      1, 0x2B, /* Probably dont change */
    // LCMCTRL: LCM Control
    ST77XX_LCMCTRL,    1, 0x2C, /* Not sure what this does */
    // VDVVRHEN: VDV and VRH Command Enable
    ST77XX_VDVVRHEN,   2, 0x01, 0xFF, /* Enable the below */
    // VRHS: VRH set
    ST77XX_VRHS,       1, 0x11, /* Maybe colour correction? */
    // VDVS: VDV Set
    ST77XX_VDVS,       1, 0x20, /* Maybe colour correction? */
    // FRCTRL2: Frame Rate control in normal mode
    ST77XX_FRCTRL2,    1, 0x0F, /* 60Hz */
    // PWCTRL1: Power Control 1
    ST77XX_PWCTRL1,    2, 0xA4, 0xA1, /* Set voltages)*/
    // PVGAMCTRL: Positive Voltage Gamma control
    ST77XX_PVGAMCTRL,  14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,
    // NVGAMCTRL: Negative Voltage Gamma control
    ST77XX_NVGAMCTRL,  14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,
    // X address set
    ST77XX_CASET,      4, 0x00, 0x00, 0x01, 0x3F,
    // Y address set
    ST77XX_RASET,      4, 0x00, 0x00, 0x00, 0xEF,
    // Display on
    ST77XX_DISPON,     0,
    0x00,
};

esp_err_t st77xx_send(ST77XX* device, const uint8_t *data, const int len) {
    if (len == 0) return ESP_OK;
    if (device->spi_device == NULL) return ESP_FAIL;
    spi_transaction_t transaction = {
        .length = len * 8,  // transaction length is in bits
        .tx_buffer = data,
        .user = (void*) device,
    };
    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}


esp_err_t st77xx_send_command(ST77XX* device, const uint8_t cmd)
{
    esp_err_t err;
    gpio_set_level(device->pin_dcx, 0);
    err = st77xx_send(device, &cmd, 1);
    return err;
}

esp_err_t st77xx_send_data(ST77XX* device, const uint8_t* data, const uint16_t length)
{
    esp_err_t err;
    gpio_set_level(device->pin_dcx, 1);
    err = st77xx_send(device, data, length);
    return err;
}


esp_err_t st77xx_reset(ST77XX* device) {
    if (device->mutex != NULL) xSemaphoreTake(device->mutex, portMAX_DELAY);
    esp_err_t res;
    res = gpio_set_level(device->pin_reset, false);
    if (res != ESP_OK) {
        if (device->mutex != NULL) xSemaphoreGive(device->mutex);
        return res;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    res = gpio_set_level(device->pin_reset, true);
    if (res != ESP_OK) {
        if (device->mutex != NULL) xSemaphoreGive(device->mutex);
        return res;
    }

    vTaskDelay(120 / portTICK_PERIOD_MS); /* This could possibly be shorter if a problem. */

    ESP_LOGD(TAG, "Reset done");
    if (device->mutex != NULL) xSemaphoreGive(device->mutex);
    return ESP_OK;
}

esp_err_t st77xx_write_init_data(ST77XX* device, const uint8_t * data) {
    if (device->spi_device == NULL) return ESP_FAIL;
    esp_err_t res;
    uint8_t cmd, len;
    while (true) {
        cmd = *data++;
        if (!cmd) break;
        len = *data++;
//        ESP_LOGD(TAG, "Sending command %x", cmd);
        res = st77xx_send_command(device, cmd);
        if (res != ESP_OK) break;
        if (len > 0) {
//            ESP_LOGD(TAG, "Sending %d bytes of data", len);
            res = st77xx_send_data(device, data, len);
            if (res != ESP_OK) break;
        }
        data += len;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t st77xx_init(ST77XX* device) {
    esp_err_t res;
    
    if (device->pin_dcx < 0) return ESP_FAIL;
    if (device->pin_cs < 0) return ESP_FAIL;
    if (device->pin_reset < 0) return ESP_FAIL;

    /*if (device->mutex == NULL) {
        device->mutex = xSemaphoreCreateMutex();
    }*/

    if (device->mutex != NULL) xSemaphoreGive(device->mutex);

    ESP_LOGD(TAG, "pin_reset: %d", device->pin_reset);
    ESP_LOGD(TAG, "pin_dcx: %d", device->pin_dcx);

    /* Setup reset */
    gpio_config_t reset_io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1LL << device->pin_reset,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    res = gpio_config(&reset_io_conf);
    if (res != ESP_OK) return res;

    /* Setup DS */
    gpio_config_t dc_io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1LL << device->pin_dcx,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    res = gpio_config(&dc_io_conf);
    if (res != ESP_OK) return res;
    res = gpio_set_level(device->pin_dcx, true);
    if (res != ESP_OK) return res;

    /* Setup SPI */
    if (device->spi_device == NULL) {
        spi_device_interface_config_t devcfg = {
            .command_bits     = 0,
            .address_bits     = 0,
            .dummy_bits       = 0,
            .mode             = 0, // SPI mode 0
            .duty_cycle_pos   = 128,
            .cs_ena_pretrans  = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz   = device->spi_speed,
            .input_delay_ns   = 0,
            .spics_io_num     = device->pin_cs,
            .flags            = SPI_DEVICE_HALFDUPLEX,
            .queue_size       = 1,
            .pre_cb           = st77xx_spi_pre_transfer_callback, // Handles D/C line
            .post_cb          = NULL
        };
        res = spi_bus_add_device(device->spi_bus, &devcfg, &device->spi_device);
        if (res != ESP_OK) return res;
    }

    if (device->callback != NULL) {
        device->callback(false);
    }

    ESP_LOGE(TAG, "IO Setup complete ");

    //Reset the LCD display
    res = st77xx_reset(device);
    if (res != ESP_OK) return res;

    ESP_LOGE(TAG, "DC pin %d", device->pin_dcx);

    //Send the initialization data to the LCD display
    res = st77xx_write_init_data(device, st77xx_init_data);
    if (res != ESP_OK) return res;

    res = st77xx_set_cfg(device, device->rotation, device->color_mode);
    if (res != ESP_OK) return res;

    return ESP_OK;
}

esp_err_t st77xx_deinit(ST77XX* device) {
    return ESP_OK;
}

esp_err_t st77xx_write(ST77XX* device, const uint8_t *buffer) {
    return st77xx_write_partial_direct(device, buffer, 0, 0, ST77XX_WIDTH, ST77XX_HEIGHT);
}

#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04 ///< LCD refresh right to left

esp_err_t st77xx_set_cfg(ST77XX * device, uint8_t rotation, bool color_mode) {
    rotation = rotation & 0x07;
    uint8_t m = 0;

    switch (rotation) {
        case 0:
            m |= MADCTL_MX;
            break;
        case 1:
            m |= MADCTL_MV;
            break;
        case 2:
            m |= MADCTL_MY;
            break;
        case 3:
            m |= (MADCTL_MX | MADCTL_MY | MADCTL_MV);
            break;
        case 4:
            m |= (MADCTL_MY | MADCTL_MV);
            break;
    }

    if (color_mode) {
        m |= MADCTL_BGR;
    } else {
        m |= MADCTL_RGB;
    }
    ESP_LOGD(TAG, "MADCTL = 0x%x", m);

    uint8_t data[1] = {m};
    esp_err_t res = st77xx_send_command(device, ST77XX_MADCTL);
    if (res != ESP_OK) return res;
    res = st77xx_send_data(device, data, 1);
    return res;
}

esp_err_t st77xx_send_u32(ST77XX* device, const uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (data>>24)&0xFF;
    buffer[1] = (data>>16)&0xFF;
    buffer[2] = (data>> 8)&0xFF;
    buffer[3] = data      &0xFF;
    return st77xx_send_data(device, buffer, 4);
}

esp_err_t st77xx_set_addr_window(ST77XX* device, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
    uint32_t ya = ((uint32_t)y << 16) | (y+h-1);
    esp_err_t res;
//    ESP_LOGD(TAG, "CASET %x, RASET %x", xa, ya);
    res = st77xx_send_command(device, ST77XX_CASET);
    if (res != ESP_OK) return res;
    res = st77xx_send_u32(device, xa);
    if (res != ESP_OK) return res;
    res = st77xx_send_command(device, ST77XX_RASET);
    if (res != ESP_OK) return res;
    res = st77xx_send_u32(device, ya);
    if (res != ESP_OK) return res;
    res = st77xx_send_command(device, ST77XX_RAMWR);
    return res;
}

esp_err_t st77xx_write_partial_direct(ST77XX* device, const uint8_t *buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    if (device->spi_device == NULL) return ESP_FAIL;
    if (device->mutex != NULL) xSemaphoreTake(device->mutex, portMAX_DELAY);
    esp_err_t res;
    res = st77xx_set_addr_window(device, x, y, width, height);
    if (res != ESP_OK) {
        if (device->mutex != NULL) xSemaphoreGive(device->mutex);
        return res;
    }

    uint32_t position = 0;
    while (width * height * 2 - position > 0) {
        uint32_t length = device->spi_max_transfer_size;
        if (width * height * 2 - position < device->spi_max_transfer_size) length = width * height * 2 - position;

        res = st77xx_send_data(device, &buffer[position], length);
        if (res != ESP_OK) {
            if (device->mutex != NULL) xSemaphoreGive(device->mutex);
            return res;
        }
        position += length;
    }
    if (device->mutex != NULL) xSemaphoreGive(device->mutex);
    return res;
}

esp_err_t st77xx_set_display(ST77XX* device, const bool state) {
    esp_err_t res;
    ESP_LOGI(TAG, "sleep display %s", state ? "on" : "off");
    if (device->mutex != NULL) xSemaphoreTake(device->mutex, portMAX_DELAY);

    if (state) {
        res = st77xx_send_command(device, ST77XX_DISPON);
        if (res != ESP_OK) return res;
    } else {
        res = st77xx_send_command(device, ST77XX_DISPOFF);
        if (res != ESP_OK) return res;
    }
    if (device->mutex != NULL) xSemaphoreGive(device->mutex);
    return res;
}
