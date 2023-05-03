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

xSemaphoreHandle intr_trigger     = NULL;   // semaphore to trigger PCA95XX interrupt handling
TaskHandle_t     intr_task_handle = NULL;
bool             intr_init        = false;  // True if at least one PCA9555 was initialized
PCA9555*         initialized_devices[MAX_DEVICES];
int              initialized_devices_num = 0;

/* I2C access */
static inline esp_err_t read_reg(PCA9555* device, uint8_t reg, uint8_t* data, size_t data_len) {
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_addr, reg, data, data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "read reg error %d", res);
    }
    return res;
}

static inline esp_err_t write_reg(PCA9555* device, uint8_t reg, uint8_t* data, size_t data_len) {
    esp_err_t res = i2c_write_reg_n(device->i2c_bus, device->i2c_addr, reg, data, data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "write reg error %d", res);
    }
    return res;
}

/* Interrupt handling */
_Noreturn void intr_task(void* arg) {
    esp_err_t res;
    uint8_t   data[] = {0, 0};
    uint16_t  current_state;

    while (1) {
        if (xSemaphoreTake(intr_trigger, portMAX_DELAY)) {
            ESP_LOGD(TAG, "Received interrupt");
            for (int n = 0; n < initialized_devices_num; n++) {
                PCA9555* device = initialized_devices[n];
                ESP_LOGD(TAG, "got device with address %x", device->i2c_addr);
                res             = read_reg(device, REG_INPUT_0, data, 2);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "failed to read input state of device %d", n);
                    continue;
                }
                current_state = data[0] + (data[1] << 8);
                ESP_LOGD(TAG, "Current state: %x", current_state);
                for (int i = 0; i < 16; i++) {
                    if ((current_state & (1 << i)) != (device->previous_state & (1 << i))) {
                        ESP_LOGD(TAG, "input state changed: %d now %d", i, (current_state & (1 << i)) == (1 << i));
                        bool value = (current_state & (1 << i)) > 0;
                        xSemaphoreTake(device->mutex, portMAX_DELAY);
                        intr_t handler = device->handlers[i];
                        xSemaphoreGive(device->mutex);
                        if (handler != NULL) handler(i, value);
                    }
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
                device->previous_state = current_state;
            }
        }
    }
}

void intr_handler(void* arg) { /* in interrupt handler */
    xSemaphoreGiveFromISR(intr_trigger, NULL);
}

/* Public functions */
void set_interrupt_handler(PCA9555* device, uint8_t pin, intr_t handler) {
    if (pin >= 16) return;
    xSemaphoreTake(device->mutex, portMAX_DELAY);
    device->handlers[pin] = handler;
    xSemaphoreGive(device->mutex);
}

esp_err_t pca9555_init(PCA9555* device, int intr_pin) {
    ESP_LOGD(TAG, "init called");
    if (initialized_devices_num >= MAX_DEVICES) {
        ESP_LOGE(TAG, "Unable to register another PCA9555 maximum number of %d reached", initialized_devices_num);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t res;

    // Create mutex
    device->mutex = xSemaphoreCreateMutex();
    if (device->mutex == NULL) return ESP_ERR_NO_MEM;

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

    device->previous_state = 0;
    for (int i = 0; i < 16; i++) {
        device->handlers[i] = NULL;
    }

    if (!intr_init) {
        // Create interrupt trigger
        intr_trigger = xSemaphoreCreateBinary();
        if (intr_trigger == NULL) return ESP_ERR_NO_MEM;

        // Attach interrupt to interrupt pin (if available)
        res = gpio_isr_handler_add(intr_pin, intr_handler, NULL);
        if (res != ESP_OK) return res;

        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_NEGEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << intr_pin,
            .pull_down_en = 0,
            .pull_up_en   = 1,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;

        xTaskCreate(&intr_task, "PCA9555 interrupt task", 4096, NULL, 10, &intr_task_handle);
        xSemaphoreGive(intr_trigger);

        // Store that we already initialized the interrupt handler
        intr_init = true;
    }

    initialized_devices[initialized_devices_num] = device;
    initialized_devices_num++;

    ESP_LOGD(TAG, "init done");
    return ESP_OK;
}

int set_gpio_direction(PCA9555* device, int pin, bool direction) {
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

int get_gpio_direction(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    return ((device->reg_config[port] >> bit) & 1) ? 0 : 1;  // Return 0 when the pin is an input and 1 when the pin is output
}

int set_gpio_polarity(PCA9555* device, int pin, bool polarity) {
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

int get_gpio_polarity(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    return ((device->reg_polarity[port] >> bit) & 1) ? 0 : 1;  // Return 0 when the pin is in normal mode and 1 when the pin is in inverted mode
}

int set_gpio_value(PCA9555* device, int pin, bool value) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    if (!get_gpio_direction(device, pin)) return -1;  // Pin is an input
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

int get_gpio_value(PCA9555* device, int pin) {
    if ((pin < 0) || (pin > 15)) return -1;  // Out of range
    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit  = pin % 8;
    uint8_t reg;
    if (get_gpio_direction(device, pin)) {
        reg = port ? REG_OUTPUT_1 : REG_OUTPUT_0;
    } else {
        reg = port ? REG_INPUT_1 : REG_INPUT_0;
    }
    uint8_t   reg_value;
    esp_err_t res = read_reg(device, reg, &reg_value, 1);
    if (res != ESP_OK) return -1;
    return (reg_value >> bit) & 1;
}