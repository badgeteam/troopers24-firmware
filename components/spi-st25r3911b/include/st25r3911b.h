#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

typedef struct ST25R3911B {
    int spi_bus;
    int pin_cs;
    int pin_irq;
    uint32_t spi_speed;
    spi_device_handle_t spi_device;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t spi_semaphore;
} ST25R3911B;

esp_err_t st25r3911b_init(ST25R3911B * device);
