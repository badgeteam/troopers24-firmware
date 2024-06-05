/**
 * Copyright (c) 2024 Tom Bennellick & Malte Heinzelmann <malte@cybaer.ninja>
 * Based on the ILI9341 driver by Nicolai Electronics.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define ST77XX_WIDTH       320
#define ST77XX_HEIGHT      240
#define ST77XX_BUFFER_SIZE ST77XX_WIDTH * ST77XX_HEIGHT * 2 // Each pixel takes 16 bits
#define ST77XX_BPP         16

// Registers
#define ST77XX_NOP         0x00
#define ST77XX_SWRESET     0x01 // Software Reset
#define ST77XX_RDDID       0x04 // Read Display ID
#define ST77XX_RDDST       0x09 // Read Display Status
#define ST77XX_RDDPM       0x0A // Read Display Power
#define ST77XX_RDDMADCTL   0x0B // Read Display Memory Data Access Mode
#define ST77XX_RDDCOLMOD   0x0C // Read Display Pixel
#define ST77XX_RDDIM       0x0D // Read Display Image
#define ST77XX_RDDSM       0x0E // Read Display Signal
#define ST77XX_RDDSDR      0x0F // Read Display Self Diagnostics
#define ST77XX_SLPIN       0x10 // Sleep In
#define ST77XX_SLPOUT      0x11 // Sleep Out
#define ST77XX_PTLON       0x12 // Partial Mode On
#define ST77XX_NORON       0x13 // Partial Mode Off
#define ST77XX_INVOFF      0x20 // Display Invert Off
#define ST77XX_INVON       0x21 // Display Invert On
#define ST77XX_GAMSET      0x26 // Display Invert On Gamma
#define ST77XX_DISPOFF     0x28 // Display Off
#define ST77XX_DISPON      0x29 // Display On
#define ST77XX_CASET       0x2A // Column Address Set
#define ST77XX_RASET       0x2B // Row Address Set
#define ST77XX_RAMWR       0x2C // Memory Write
#define ST77XX_RAMRD       0x2E // Memory Read
#define ST77XX_PTLAR       0x30 // Partial Start/End Address Set
#define ST77XX_VSCRDEF     0x33 // Vertical Scrolling Definition
#define ST77XX_TEOFF       0x34 // Tearing Effect Line Off
#define ST77XX_TEON        0x35 // Tearing Effect Line On
#define ST77XX_MADCTL      0x36 // Memory Data Access Control
#define ST77XX_VSCRSADD    0x37 // Vertical Scrolling Start Address
#define ST77XX_IDMOFF      0x38 // Idle Mode Off
#define ST77XX_IDMON       0x39 // Idle Mode On
#define ST77XX_COLMOD      0x3A // Interface Pixel Format
#define ST77XX_RAMWRC      0x3C // Memory Write Continue
#define ST77XX_RAMRDC      0x3E // Memory Read Continue
#define ST77XX_TESCAN      0x44 // Set Tear Scan Line
#define ST77XX_RDTESCAN    0x45 // Get Tear Scan Line
#define ST77XX_WRDISBV     0x51 // Set Display Brightness
#define ST77XX_RDDISBV     0x52 // Get Display Brightness
#define ST77XX_WRCTRLD     0x53 // Set Display Control
#define ST77XX_RDCTRLD     0x54 // Get Display Control
#define ST77XX_WRCACE      0x55 // Write content adaptive brightness control and Color enhancement
#define ST77XX_RDCABC      0x56 // Read content adaptive brightness control and Color enhancement
#define ST77XX_WRCABCMB    0x5E // Write CABC minimum brightness
#define ST77XX_RDCABCMB    0x5F // Read CABC minimum brightness
#define ST77XX_RDABCSDR    0x68 // Read Automatic Brightness Control Self-Diagnostic Result
#define ST77XX_PORCTRK     0xB2 // Porch setting
#define ST77XX_GCTRL       0xB7 // Gate Control
#define ST77XX_VCOMS       0xBB // VCOM setting
#define ST77XX_LCMCTRL     0xC0 // LCM Control
#define ST77XX_VDVVRHEN    0xC2 // VDV and VRH Command Enable
#define ST77XX_VRHS        0xC3 // VRH Set
#define ST77XX_VDVS        0xC4 // VDV Set
#define ST77XX_FRCTRL2     0xC6 // Frame Rate control in normal mode
#define ST77XX_PWCTRL1     0xD0 // Power Control 1
#define ST77XX_RDID1       0xDA // Read ID1
#define ST77XX_RDID2       0xDB // Read ID2
#define ST77XX_RDID3       0xDC // Read ID3
#define ST77XX_PVGAMCTRL   0xE0 // Positive Voltage Gamma control
#define ST77XX_NVGAMCTRL   0xE1 // Negative Voltage Gamma control

// Extended command set
#define ST77XX_RGB_INTERFACE 0xB0 // RGB Interface Signal Control
#define ST77XX_FRMCTR1       0xB1 // Frame Rate Control (In Normal Mode)
#define ST77XX_FRMCTR2       0xB2 // Frame Rate Control (In Idle Mode)
#define ST77XX_FRMCTR3       0xB3 // Frame Rate Control (In Partial Mode)
#define ST77XX_INVTR         0xB4 // Display Inversion Control
#define ST77XX_BPC           0xB5 // Blanking Porch Control register
#define ST77XX_DFC           0xB6 // Display Function Control register
#define ST77XX_ETMOD         0xB7 // Entry Mode Set
#define ST77XX_BACKLIGHT1    0xB8 // Backlight Control 1
#define ST77XX_BACKLIGHT2    0xB9 // Backlight Control 2
#define ST77XX_BACKLIGHT3    0xBA // Backlight Control 3
#define ST77XX_BACKLIGHT4    0xBB // Backlight Control 4
#define ST77XX_BACKLIGHT5    0xBC // Backlight Control 5
#define ST77XX_BACKLIGHT7    0xBE // Backlight Control 7
#define ST77XX_BACKLIGHT8    0xBF // Backlight Control 8
#define ST77XX_POWER1        0xC0 // Power Control 1 register
#define ST77XX_POWER2        0xC1 // Power Control 2 register
#define ST77XX_VCOM1         0xC5 // VCOM Control 1 register
#define ST77XX_VCOM2         0xC7 // VCOM Control 2 register
#define ST77XX_NVMWR         0xD0 // NV Memory Write
#define ST77XX_NVMPKEY       0xD1 // NV Memory Protection Key
#define ST77XX_RDNVM         0xD2 // NV Memory Status Read
#define ST77XX_READ_ID4      0xD3 // Read ID4
#define ST77XX_PGAMMA        0xE0 // Positive Gamma Correction register
#define ST77XX_NGAMMA        0xE1 // Negative Gamma Correction register
#define ST77XX_DGAMCTRL1     0xE2 // Digital Gamma Control 1
#define ST77XX_DGAMCTRL2     0xE3 // Digital Gamma Control 2
#define ST77XX_INTERFACE     0xF6 // Interface control register

// Extend register commands
#define ST77XX_POWERA        0xCB // Power control A register
#define ST77XX_POWERB        0xCF // Power control B register
#define ST77XX_DTCA          0xE8 // Driver timing control A
#define ST77XX_DTCB          0xEA // Driver timing control B
#define ST77XX_POWER_SEQ     0xED // Power on sequence register
#define ST77XX_3GAMMA_EN     0xF2 // 3 Gamma enable register
#define ST77XX_PRC           0xF7 // Pump ratio control register

typedef void (*ST77XX_cb_t)(bool); // Callback for init / deinit

typedef struct ST77XX {
    // Pins
    int spi_bus;
    int pin_cs;
    int pin_dcx;
    int pin_reset;
    // Configuration
    uint8_t rotation;
    bool color_mode;
    uint32_t spi_speed;
    uint32_t spi_max_transfer_size;
    ST77XX_cb_t callback;
    // Internal state
    spi_device_handle_t spi_device;
    // Mutex
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t spi_semaphore;
} ST77XX;

esp_err_t st77xx_init(ST77XX* device);
esp_err_t st77xx_deinit(ST77XX* device);

esp_err_t st77xx_set_display(ST77XX* device, const bool state);
esp_err_t st77xx_set_cfg(ST77XX* device, uint8_t rotation, bool color_mode);

esp_err_t st77xx_write(ST77XX* device, const uint8_t *data);
esp_err_t st77xx_write_partial_direct(ST77XX* device, const uint8_t *buffer, uint16_t x, uint16_t y, uint16_t width, uint16_t height);

#ifdef __cplusplus
}
#endif //__cplusplus
