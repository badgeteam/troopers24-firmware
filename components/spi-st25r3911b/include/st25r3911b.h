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

#define MAX_NFC_BUFFER_SIZE 540

#include "rfal_defConfig.h"
#include "rfal_platform.h"
#include "rfal_nfc.h"
#include "rfal_rf.h"

#define LWIP_ERR_TIMEOUT ERR_TIMEOUT
#undef ERR_TIMEOUT
#include "ndef_poller.h"
#undef ERR_TIMEOUT
#define ERR_TIMEOUT LWIP_ERR_TIMEOUT
#undef LWIP_ERR_TIMEOUT

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

typedef enum {
    DISCOVER_MODE_LISTEN_NFCA,
    DISCOVER_MODE_P2P_PASSIVE,
    DISCOVER_MODE_P2P_ACTIVE,
} st25r3911b_discover_mode;

typedef esp_err_t (*NfcDeviceCallback)(rfalNfcDevice *nfcDevice);

esp_err_t st25r3911b_init(ST25R3911B * device);
esp_err_t st25r3911b_chip_id(uint8_t *id);
esp_err_t st25r3911b_discover(NfcDeviceCallback callback, uint32_t timeout_ms, st25r3911b_discover_mode discover_mode);
esp_err_t st25r3911b_read_data(rfalNfcDevice *nfcDevice, ndefConstBuffer* bufConstRawMessage);
esp_err_t st25r3911b_poll_active_p2p(uint32_t timeout_ms);
esp_err_t st25r3911b_listen_p2p(uint32_t timeout_ms);

esp_err_t st25r3911b_rxtx(ST25R3911B *device, const uint8_t* tx, const uint8_t* rx, uint8_t length);
