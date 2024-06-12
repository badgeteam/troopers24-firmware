#include "include/st25r3911b.h"

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

#include "st25r3911_com.h"
#include "st25r3911.h"

static const char *TAG = "st25r3911b";


esp_err_t rfid_reg_read(ST25R3911B *device, uint8_t reg, uint8_t *value) {
    if (device->spi_device == NULL) return ESP_FAIL;
    if (reg > 0x3f) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = 0x40 | reg;
    tx[1] = 0;
    rx[0] = 0xff;
    rx[1] = 0xff;

    spi_transaction_t transaction = {
        .length    = 16,  // transaction length is in bits
        .tx_buffer = &tx,
        .rx_buffer = &rx,
        .rxlength  = 0, /* As .length*/
        .user      = (void *) device,
    };

    //    ESP_LOGE(TAG, "Before: %x %x %x %x", tx[0], tx[1], rx[0], rx[1] );

    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    //    ESP_LOGE(TAG, "After: %x %x %x %x", tx[0], tx[1], rx[0], rx[1] );
    *value = rx[1];
    return res;
}

/* TODO: Combine with read ?, only 1 byte different*/
esp_err_t rfid_reg_write(ST25R3911B *device, uint8_t reg, uint8_t value) {
    if (device->spi_device == NULL) return ESP_FAIL;
    if (reg > 0x3f) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2];
    tx[0]                         = 0x00 | reg;
    tx[1]                         = value;
    spi_transaction_t transaction = {
        .length    = 16,  // transaction length is in bits
        .tx_buffer = &tx,
        .user = (void *) device,
    };

    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}

/* TODO: Combine with read ?, only 1 byte different*/
esp_err_t rfid_send_command(ST25R3911B *device, uint8_t command) {
    if (device->spi_device == NULL) return ESP_FAIL;

    /* Dont check. ST include the flags in the value */
    //    if (command > 0x3f) return ESP_ERR_INVALID_ARG;

    uint8_t data = 0xC0 | command;

    spi_transaction_t transaction = {
        .length    = 8,  // transaction length is in bits
        .tx_buffer = &data,
        .user      = (void *) device,
    };
    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &transaction);
    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}

esp_err_t st25r_test(ST25R3911B *device) {
    /* This is the same sequence as st25r3911Initialize() in en.STSW-ST25RFAL001/source/st25r3911/st25r3911.c*/
    rfid_send_command(device, ST25R3911_CMD_SET_DEFAULT);                       /* 0xc1*/
    rfid_reg_write(device, ST25R3911_REG_OP_CONTROL, 0);                        /* 0x02 0x00*/
    rfid_reg_write(device, ST25R3911_REG_IO_CONF1, ST25R3911_REG_IO_CONF1_osc); /* 0x00 0x08*/
    rfid_reg_write(device, ST25R3911_REG_IO_CONF2, 0);                          /* 0x01 0x00*/

    uint8_t rmr = 0;
    rfid_reg_read(device, ST25R3911_REG_IO_CONF2, &rmr);                                                                     /* 0x41 0xNN */
    rfid_reg_write(device, ST25R3911_REG_IO_CONF2, rmr | ST25R3911_REG_IO_CONF2_miso_pd1 | ST25R3911_REG_IO_CONF2_miso_pd2); /* 0x01 0x18*/

    uint8_t   id = 0;
    esp_err_t e  = rfid_reg_read(device, ST25R3911_REG_IC_IDENTITY, &id);
    if ((id & ST25R3911_REG_IC_IDENTITY_mask_ic_type) == ST25R3911_REG_IC_IDENTITY_ic_type) {
        switch (id & ST25R3911_REG_IC_IDENTITY_mask_ic_rev) {
            case 0x02:
                ESP_LOGI(TAG, "silicon rev. r3.1");
                break;
            case 0x03:
                ESP_LOGI(TAG, "silicon rev. r3.3");
                break;
            case 0x04:
                ESP_LOGI(TAG, "silicon rev. r4.0");
                break;
            case 0x05:
                ESP_LOGI(TAG, "silicon rev. r4.1");
                break;
            default:
                ESP_LOGI(TAG, "Currently unknown silicon rev. %x", id & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
                break;
        }
    } else {
        ESP_LOGE(TAG, "Invalid response for ID register: %d", id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t st25r3911b_init(ST25R3911B *device) {
    esp_err_t res;

    if (device->pin_cs < 0) return ESP_FAIL;
    if (device->spi_speed > 6000000) return ESP_FAIL;

    /*if (device->mutex == NULL) {
        device->mutex = xSemaphoreCreateMutex();
    }*/

    //    if (device->mutex != NULL) xSemaphoreGive(device->mutex);

    if (device->spi_device == NULL) {
        spi_device_interface_config_t devcfg = {.command_bits     = 0,
                                                .address_bits     = 0,
                                                .dummy_bits       = 0,
                                                .mode             = 1,
                                                .duty_cycle_pos   = 128,
                                                .cs_ena_pretrans  = 1,
                                                .cs_ena_posttrans = 0,
                                                .clock_speed_hz   = device->spi_speed,
                                                .input_delay_ns   = 0,
                                                .spics_io_num     = device->pin_cs,
                                                //.flags            = SPI_DEVICE_HALFDUPLEX,
                                                .queue_size = 1,
                                                .pre_cb     = NULL,
                                                .post_cb    = NULL};
        res                                  = spi_bus_add_device(device->spi_bus, &devcfg, &device->spi_device);
        if (res != ESP_OK) return res;
    }

    st25r_test(device);

    //    if (device->callback != NULL) {
    //        device->callback(false);
    //    }

    return ESP_OK;
}
