#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>

#include "pca9555.h"

__BEGIN_DECLS

/* Front */
#define PIN_BTN_DOWN   0
#define PIN_BTN_LEFT   1
#define PIN_BTN_RIGHT  2
#define PIN_BTN_UP     3
#define PIN_BTN_SELECT 4
#define PIN_BTN_A      5
#define PIN_BTN_B      6
#define PIN_BTN_START  7

/* Keyboard 1 */
#define PIN_KEY_P         0
#define PIN_KEY_L         1
#define PIN_KEY_RETURN    2
#define PIN_KEY_SHIFT     3
#define PIN_KEY_O         4
#define PIN_KEY_K         5
#define PIN_KEY_M         6
#define PIN_KEY_BACKSPACE 7

#define PIN_KEY_I 8
#define PIN_KEY_J 9
#define PIN_KEY_N 10
#define PIN_KEY_U 11
#define PIN_KEY_H 12
#define PIN_KEY_B 13
#define PIN_KEY_Y 14
#define PIN_KEY_G 15

/* Keyboard 2 */
#define PIN_KEY_V  0
#define PIN_KEY_T  1
#define PIN_KEY_F  2
#define PIN_KEY_C  3
#define PIN_KEY_FN 4
#define PIN_KEY_R  5
#define PIN_KEY_D  6
#define PIN_KEY_X  7

#define PIN_KEY_SHIELD 8
#define PIN_KEY_E      9
#define PIN_KEY_S      10
#define PIN_KEY_Z      11
#define PIN_KEY_W      12
#define PIN_KEY_A      13
#define PIN_KEY_Q      14
#define PIN_KEY_SPACE  15

/* Keys */

enum Key {
    BTN_ACCEPT = 0,
    BTN_BACK,
    BTN_START,
    BTN_SELECT,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
    KEY_SHIELD,
    KEY_FN,
    KEY_SPACE,
    KEY_BACKSPACE,
    KEY_SHIFT,
    KEY_RETURN,
    KEY_Q,
    KEY_W,
    KEY_E,
    KEY_R,
    KEY_T,
    KEY_Y,
    KEY_U,
    KEY_I,
    KEY_O,
    KEY_P,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_N,
    KEY_M,
};

/* Structs */

typedef struct _keyboard_input_message {
    uint8_t input;
    bool    state;
} keyboard_input_message_t;

typedef struct Keyboard {
    // Pins
    int     i2c_bus;
    int     intr_pin;
    uint8_t addr_front;
    uint8_t addr_keyboard1;
    uint8_t addr_keyboard2;
    // Internal state
    PCA9555* front;
    PCA9555* keyboard1;
    PCA9555* keyboard2;
    QueueHandle_t queue;
    TaskHandle_t  intr_task_handle;
    // Mutex
    SemaphoreHandle_t intr_trigger;
    SemaphoreHandle_t i2c_semaphore;
} Keyboard;

typedef void (*send_fn_t)(Keyboard*, uint8_t, bool);

/* Public methods */

esp_err_t keyboard_init(Keyboard* device);

__END_DECLS
