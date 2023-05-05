#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/spi_master.h>

typedef struct CC1200 {
    // Pins
    int spi_bus;
    int pin_cs;
    int pin_intr;
    int pin_reset;
    // Configuration
    uint32_t spi_speed;
    // Internal state
    spi_device_handle_t spi_device;
    bool dc_level;
    // Mutex
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t spi_semaphore;
} CC1200;

typedef struct cc1200_message {
    short    len;
    uint8_t *data;
    int8_t   rssi;
    uint8_t  crc_lqi;
} cc1200_message;