#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define ST25R3911
#define ST25R_SELFTEST

#include "rfal_defConfig.h"
#include "rfal_platform.h"
#include "rfal_nfc.h"

typedef struct ST25R3911B {
    int spi_bus;
    int pin_cs;
    int pin_irq;
    uint32_t spi_speed;
    spi_device_handle_t spi_device;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t spi_semaphore;
    TaskHandle_t  intr_task_handle;
    SemaphoreHandle_t intr_trigger;
    void (*irq_callback)(void);
} ST25R3911B;

esp_err_t st25r3911b_init(ST25R3911B * device);
esp_err_t st25r3911b_chip_id(uint8_t *id);
esp_err_t st25r3911b_discover(rfalNfcDevice *nfcDevice, uint32_t timeout_ms);

esp_err_t st25r3911b_rxtx(ST25R3911B *device, const uint8_t* tx, const uint8_t* rx, uint8_t length);
