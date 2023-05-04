#include "keyboard.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdbool.h>

#include "pca9555.h"

static const char* TAG = "keyboard";

PCA9555 front     = {0};
PCA9555 keyboard1 = {0};
PCA9555 keyboard2 = {0};

int get_front_key(uint8_t pin) {
    switch (pin) {
        case PIN_BTN_DOWN:
            return BTN_DOWN;
        case PIN_BTN_LEFT:
            return BTN_LEFT;
        case PIN_BTN_RIGHT:
            return BTN_RIGHT;
        case PIN_BTN_UP:
            return BTN_UP;
        case PIN_BTN_SELECT:
            return BTN_SELECT;
        case PIN_BTN_A:
            return BTN_ACCEPT;
        case PIN_BTN_B:
            return BTN_BACK;
        case PIN_BTN_START:
            return BTN_START;
    }
    return -1;
}

int get_keyboard1_key(uint8_t pin) {
    switch (pin) {
        case PIN_KEY_P:
            return KEY_P;
        case PIN_KEY_L:
            return KEY_L;
        case PIN_KEY_RETURN:
            return KEY_RETURN;
        case PIN_KEY_SHIFT:
            return KEY_SHIFT;
        case PIN_KEY_O:
            return KEY_O;
        case PIN_KEY_K:
            return KEY_K;
        case PIN_KEY_M:
            return KEY_M;
        case PIN_KEY_BACKSPACE:
            return KEY_BACKSPACE;
        case PIN_KEY_I:
            return KEY_I;
        case PIN_KEY_J:
            return KEY_J;
        case PIN_KEY_N:
            return KEY_N;
        case PIN_KEY_U:
            return KEY_U;
        case PIN_KEY_H:
            return KEY_H;
        case PIN_KEY_B:
            return KEY_B;
        case PIN_KEY_Y:
            return KEY_Y;
        case PIN_KEY_G:
            return KEY_G;
    }
    return -1;
}

int get_keyboard2_key(uint8_t pin) {
    switch (pin) {
        case PIN_KEY_V:
            return KEY_V;
        case PIN_KEY_T:
            return KEY_T;
        case PIN_KEY_F:
            return KEY_F;
        case PIN_KEY_C:
            return KEY_C;
        case PIN_KEY_FN:
            return KEY_FN;
        case PIN_KEY_R:
            return KEY_R;
        case PIN_KEY_D:
            return KEY_D;
        case PIN_KEY_X:
            return KEY_X;
        case PIN_KEY_SHIELD:
            return KEY_SHIELD;
        case PIN_KEY_E:
            return KEY_E;
        case PIN_KEY_S:
            return KEY_S;
        case PIN_KEY_Z:
            return KEY_Z;
        case PIN_KEY_W:
            return KEY_W;
        case PIN_KEY_A:
            return KEY_A;
        case PIN_KEY_Q:
            return KEY_Q;
        case PIN_KEY_SPACE:
            return KEY_SPACE;
    }
    return -1;
}

void send_key_to_queue(Keyboard* device, int key, bool state) {
    if (key < 0) return;
    keyboard_input_message_t message;
    message.input = key;
    message.state = state;
    ESP_LOGD(TAG, "Key event %d, %d", key, state);
    xQueueSend(device->queue, &message, portMAX_DELAY);
}

void handle_front(Keyboard* device, uint8_t pin, bool state) { send_key_to_queue(device, get_front_key(pin), state); }

void handle_keyboard1(Keyboard* device, uint8_t pin, bool state) { send_key_to_queue(device, get_keyboard1_key(pin), state); }

void handle_keyboard2(Keyboard* device, uint8_t pin, bool state) { send_key_to_queue(device, get_keyboard2_key(pin), state); }

/* Interrupt handling */
esp_err_t handle_pca9555_input_change(Keyboard* keyboard, PCA9555* device, send_fn_t send_fn) {
    uint16_t current_state;
    esp_err_t res = pca9555_get_gpio_values(device, &current_state);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read input state of device %d", device->i2c_addr);
        return res;
    }
    for (int i = 0; i < 16; i++) {
        if (((current_state >> i) & 0x01) != (device->previous_state & (1 << i))) {
            bool value = (current_state >> i) & 0x01;
            send_fn(keyboard, i, value);
        }
    }
    return ESP_OK;
}

_Noreturn void intr_task(void* arg) {
    esp_err_t res;
    Keyboard* device = (Keyboard*) arg;

    while (1) {
        if (xSemaphoreTake(device->intr_trigger, portMAX_DELAY)) {
            ESP_LOGD(TAG, "Received interrupt");

            res = handle_pca9555_input_change(device, device->front, &handle_front);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "error while processing front pca9555 data");
            }

            res = handle_pca9555_input_change(device, device->keyboard1, &handle_keyboard1);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "error while processing keyboard1 pca9555 data");
            }

            res = handle_pca9555_input_change(device, device->keyboard2, &handle_keyboard2);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "error while processing keyboard2 pca9555 data");
            }
        }
    }
}

void intr_handler(void* arg) { /* in interrupt handler */
    Keyboard* device = (Keyboard*) arg;
    xSemaphoreGiveFromISR(device->intr_trigger, NULL);
    portYIELD_FROM_ISR();
}

/* Initialization */

esp_err_t keyboard_init(Keyboard* device) {
    esp_err_t res;
    ESP_LOGD(TAG, "Initializing keyboard");

    device->queue = xQueueCreate(8, sizeof(keyboard_input_message_t));

    front.i2c_bus       = device->i2c_bus;
    front.i2c_addr      = device->addr_front;
    front.i2c_semaphore = device->i2c_semaphore;
    device->front       = &front;
    pca9555_init(device->front);

    keyboard1.i2c_bus       = device->i2c_bus;
    keyboard1.i2c_addr      = device->addr_keyboard1;
    keyboard1.i2c_semaphore = device->i2c_semaphore;
    device->keyboard1 = &keyboard1;
    pca9555_init(device->keyboard1);

    keyboard2.i2c_bus       = device->i2c_bus;
    keyboard2.i2c_addr      = device->addr_keyboard2;
    keyboard2.i2c_semaphore = device->i2c_semaphore;
    device->keyboard2 = &keyboard2;
    pca9555_init(device->keyboard2);

    if (device->intr_pin) {
        // Create interrupt trigger
        device->intr_trigger = xSemaphoreCreateBinary();
        if (device->intr_trigger == NULL) return ESP_ERR_NO_MEM;

        // Attach interrupt to interrupt pin (if available)
        res = gpio_isr_handler_add(device->intr_pin, intr_handler, (void*) device);
        if (res != ESP_OK) return res;

        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_NEGEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << device->intr_pin,
            .pull_down_en = 0,
            .pull_up_en   = 1,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;

        xTaskCreate(&intr_task, "PCA9555 interrupt task", 4096, device, 10, &device->intr_task_handle);
        xSemaphoreGive(device->intr_trigger);
    }

    ESP_LOGD(TAG, "Done initializing keyboard");
    return ESP_OK;
}