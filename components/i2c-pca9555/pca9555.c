#include "pca9555.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdbool.h>

#include "managed_i2c.h"

static const char* TAG = "pca9555";

/* I2C access */
static inline esp_err_t read_reg(PCA9555* device, uint8_t reg, uint8_t* data, size_t data_len) {
    if (device->i2c_semaphore != NULL) xSemaphoreTake(device->i2c_semaphore, portMAX_DELAY);
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_addr, reg, data, data_len);
    if (device->i2c_semaphore != NULL) xSemaphoreGive(device->i2c_semaphore);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "read reg error %d", res);
    }
    return res;
}

static inline esp_err_t write_reg(PCA9555* device, uint8_t reg, uint8_t* data, size_t data_len) {
    if (device->i2c_semaphore != NULL) xSemaphoreTake(device->i2c_semaphore, portMAX_DELAY);
    esp_err_t res = i2c_write_reg_n(device->i2c_bus, device->i2c_addr, reg, data, data_len);
    if (device->i2c_semaphore != NULL) xSemaphoreGive(device->i2c_semaphore);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "write reg error %d", res);
    }
    return res;
}

/* Public functions */

esp_err_t pca9555_init(PCA9555* device) {
    ESP_LOGD(TAG, "init called");
    esp_err_t res;

    // Configure pins
    device->reg_config[0] = 0xFF;  // By default, set all pins to input
    device->reg_config[1] = 0xFF;

    res = write_reg(device, REG_CONFIG_0, device->reg_config, 2);  // Writes port mode to both config 0 and config 1 registers
    if (res != ESP_OK) return res;

    device->reg_polarity[0] = 0xFF;
    device->reg_polarity[1] = 0xFF;

    res = write_reg(device, REG_POLARITY_0, device->reg_polarity, 2);  // Writes port polarity to both port 0 and port 1 registers
    if (res != ESP_OK) return res;

    device->reg_output[0] = 0x00;
    device->reg_output[1] = 0x00;

    ESP_LOGD(TAG, "init done");
    return ESP_OK;
}

int pca9555_set_gpio_direction(PCA9555* device, int pin, bool direction) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port          = (pin >= 8) ? 1 : 0;
    uint8_t bit           = pin % 8;
    bool    current_state = ((device->reg_config[port] >> bit) & 1) ? 0 : 1;
    if (direction != current_state) {
        if (direction) {
            device->reg_config[port] &= ~(1 << bit);  // Set the pin to output
        } else {
            device->reg_config[port] |= (1 << bit);   // Set the pin to input
        }
    }
    esp_err_t res = write_reg(device, REG_CONFIG_0, device->reg_config, 2);
    if (res != ESP_OK) return -1;
    return 0;
}

int pca9555_get_gpio_direction(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    return ((device->reg_config[port] >> bit) & 1) ? 0 : 1;  // Return 0 when the pin is an input and 1 when the pin is output
}

int pca9555_set_gpio_polarity(PCA9555* device, int pin, bool polarity) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port          = (pin >= 8) ? 1 : 0;
    uint8_t bit           = pin % 8;
    bool    current_state = ((device->reg_polarity[port] >> bit) & 1) ? 0 : 1;
    if (polarity != current_state) {
        if (polarity) {
            device->reg_polarity[port] &= ~(1 << bit);  // Set the pin to output
        } else {
            device->reg_polarity[port] |= (1 << bit);   // Set the pin to input
        }
    }
    esp_err_t res = write_reg(device, REG_POLARITY_0, device->reg_polarity, 2);
    if (res != ESP_OK) return -1;
    return 0;
}

int pca9555_get_gpio_polarity(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    return ((device->reg_polarity[port] >> bit) & 1) ? 0 : 1;  // Return 0 when the pin is in normal mode and 1 when the pin is in inverted mode
}

int pca9555_set_gpio_value(PCA9555* device, int pin, bool value) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    if (!pca9555_get_gpio_direction(device, pin)) return -1;  // Pin is an input
    if (value) {
        device->reg_output[port] |= (1 << bit);
    } else {
        device->reg_output[port] &= ~(1 << bit);
    }
    uint8_t   reg = port ? REG_OUTPUT_1 : REG_OUTPUT_0;
    esp_err_t res = write_reg(device, reg, &device->reg_output[port], 1);
    if (res != ESP_OK) return -1;
    return 0;
}

int pca9555_get_gpio_value(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    uint8_t reg;
    if (pca9555_get_gpio_direction(device, pin)) {
        reg = port ? REG_OUTPUT_1 : REG_OUTPUT_0;
    } else {
        reg = port ? REG_INPUT_1 : REG_INPUT_0;
    }
    uint8_t   reg_value;
    esp_err_t res = read_reg(device, reg, &reg_value, 1);
    if (res != ESP_OK) return -1;
    return (reg_value >> bit) & 1;
}

int pca9555_get_gpio_values(PCA9555* device, uint16_t* output) {
    uint8_t   data[] = {0, 0};
    esp_err_t res = read_reg(device, REG_INPUT_0, data, 2);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read input state of device %d", device->i2c_addr);
        return ESP_FAIL;
    }
    *output = data[0] + (data[1] << 8);
    return ESP_OK;
}