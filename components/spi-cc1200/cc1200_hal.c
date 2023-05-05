#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <sdkconfig.h>

#include "driver/gpio.h"
#include "driver_cc1200.h"

static const char* TAG = "driver_cc1200";

// static void IRAM_ATTR ili9341_spi_pre_transfer_callback(spi_transaction_t *t) {
//     CC1200* device = ((CC1200*) t->user);
//     gpio_set_level(device->pin_dcx, device->dc_level);
// }

esp_err_t cc1200_arch_init(CC1200* device) {
    esp_err_t res;

    if (device->pin_cs < 0) return ESP_FAIL;

    // Initialize chip select GPIO pin
    res = gpio_set_direction(device->pin_cs, GPIO_MODE_OUTPUT);
    if (res != ESP_OK) return res;

    if (device->spi_device == NULL) {
        spi_device_interface_config_t devcfg = {
            .command_bits   = 0,
            .address_bits   = 0,
            .dummy_bits   = 0,
            .mode           = 0,  // SPI mode 0
            .duty_cycle_pos   = 128,
            .cs_ena_pretrans  = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = device->spi_speed,
//            .spics_io_num = device->pin_cs,
            .spics_io_num = -1,
            .input_delay_ns   = 0,
            .queue_size     = 1,
            .pre_cb         = NULL,
            .post_cb        = NULL,
        };
        res = spi_bus_add_device(device->spi_bus, &devcfg, &device->spi_device);
        if (res != ESP_OK) return res;
    }

    ESP_LOGD(TAG, "SPI bus initialised");
    return ESP_OK;
}

// set IOCFG2 to 54 for 40khz clock out on GPIO2.
// IOCFG2 is addr 0x01

#define CMD_WR 0
#define CMD_RD 2

void cc1200_arch_spi_select(CC1200* device) { gpio_set_level(device->pin_cs, 0); }

void cc1200_arch_spi_deselect(CC1200* device) { gpio_set_level(device->pin_cs, 1); }

uint8_t cc1200_arch_spi_rw_byte(CC1200* device, uint8_t c) {
    uint8_t           rx;
    spi_transaction_t t = {
        .tx_buffer = &c,
        .rx_buffer = &rx,
        .length    = 8 * 1,
    };
    esp_err_t res = spi_device_transmit(device->spi_device, &t);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to write spi: %d", res);
    }
    return rx;
}

int cc1200_arch_spi_rw(CC1200* device, uint8_t* read_buf, const uint8_t* write_buf, uint16_t len) {
    spi_transaction_t t = {
        .tx_buffer = write_buf,
        .rx_buffer = read_buf,
        .length    = 8 * len,
    };
//    if (device->spi_semaphore != NULL) xSemaphoreTake(device->spi_semaphore, portMAX_DELAY);
    esp_err_t res = spi_device_transmit(device->spi_device, &t);
//    if (device->spi_semaphore != NULL) xSemaphoreGive(device->spi_semaphore);
    return res;
}

void gpio0_isr(void* arg) { cc1200_rx_interrupt(); }

void cc1200_arch_gpio0_setup_irq(CC1200* device, int rising) {
    gpio_isr_handler_add(device->pin_intr, gpio0_isr, NULL);

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = 1LL << device->pin_intr,
    };

    gpio_config(&io_conf);
}

void cc1200_arch_gpio0_enable_irq(CC1200* device) { gpio_intr_enable(device->pin_intr); }

void cc1200_arch_gpio0_disable_irq(CC1200* device) { gpio_intr_disable(device->pin_intr); }

int cc1200_arch_gpio0_read_pin(CC1200* device) {
    uint8_t in;
    in = gpio_get_level(device->pin_intr);
    ESP_LOGI(TAG, "gpio0 level: %d", in);
    return in;
}
