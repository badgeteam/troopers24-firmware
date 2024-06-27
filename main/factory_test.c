#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <sys/cdefs.h>
#include <unistd.h>

#include "audio.h"
#include "filesystems.h"
#include "hardware.h"
#include "pax_gfx.h"
#include "settings.h"
#include "test_common.h"
#include "wifi_defaults.h"
#include "ws2812.h"

static const char* TAG = "factory";


bool wait_for_key_pressed(Keyboard* keyboard, Key key) {
    keyboard_input_message_t buttonMessage = {0};
    int i = 0;

    while (i < 20) {
        if (xQueueReceive(keyboard->queue, &buttonMessage, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            if (buttonMessage.state && buttonMessage.input == key) {
                return true;
            }
            i--;
        }
        i++;
    }

    return false;
}

bool check_key(Keyboard* keyboard, Key key, char * key_name, uint32_t* rc) {
    char key_request[30];
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font = pax_font_sky_mono;
    snprintf(key_request, sizeof(key_request), "Press %s...", key_name);

    ESP_LOGI(TAG, "%s",key_request);
    pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
    pax_draw_text(pax_buffer, 0xFF0000FF, font, 36, 0, pax_buffer->height - 36, key_request);
    display_flush();
    if (!wait_for_key_pressed(keyboard, key)) {
        ESP_LOGE(TAG, "Timeout reached");
        pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
        display_flush();
        *rc = (uint32_t) 2;
        return false;
    }
    return true;
}


bool test_keyboard_init(uint32_t* rc) {
    Keyboard* keyboard = get_keyboard();
    if (keyboard == NULL) {
        *rc = (uint32_t) 1;
        return false;
    }

    pax_buf_t*        pax_buffer = get_pax_buffer();
    check_key(keyboard, BUTTON_SELECT, "SELECT",rc);
    check_key(keyboard, BUTTON_START, "START",rc);
    check_key(keyboard, BUTTON_BACK, "B",rc);
    check_key(keyboard, BUTTON_ACCEPT, "A",rc);
    check_key(keyboard, JOYSTICK_UP, "UP",rc);
    check_key(keyboard, JOYSTICK_DOWN, "DOWN",rc);
    check_key(keyboard, JOYSTICK_LEFT, "LEFT",rc);
    check_key(keyboard, JOYSTICK_RIGHT, "RIGHT",rc);
    check_key(keyboard, JOYSTICK_PUSH, "PUSH",rc);
    /* TODO, think about the return code*/

    pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
    display_flush();

    return true;
}

bool test_sdcard_init(uint32_t* rc) {
    bool sdcard_mounted = (mount_sdcard_filesystem() == ESP_OK);
    if (!sdcard_mounted) {
        ESP_LOGI(TAG, "No SD card found");
        *rc = (uint32_t) 1;
        return false;
    }

    ESP_LOGI(TAG, "Trying to create file on SD card...");
    FILE* test_fd = fopen("/sd/factory_test", "w");
    if (test_fd == NULL) {
        ESP_LOGI(TAG, "Unable to open file");
        *rc = (uint32_t) 2;
        return false;
    }

    char* test_str = "TROOPERS24 TEST";
    int test_str_len = 15;

    fwrite(test_str, 1, test_str_len, test_fd);
    fclose(test_fd);

    ESP_LOGI(TAG, "Wrote file, trying to read. You should see \"TROOPERS24 TEST\" in the next line");

    test_fd = fopen("/sd/factory_test", "r");
    if (test_fd == NULL) {
        ESP_LOGI(TAG, "Unable to open file");
        *rc = (uint32_t) 3;
        return false;
    }
    char* buf = malloc(test_str_len + 1);
    fread(buf, 1, test_str_len, test_fd);
    buf[test_str_len] = 0;
    fclose(test_fd);

    puts(buf);

    if (strncmp(test_str, buf, test_str_len) != 0) {
        *rc = (uint32_t) 4;
        return false;
    }

    *rc = (uint32_t) 0;
    return (unmount_sdcard_filesystem() == ESP_OK);
}

/* Test routines */
bool test_stuck_buttons(uint32_t* rc) {
    Keyboard* keyboard = get_keyboard();
    if (keyboard == NULL) {
        *rc = 0xFFFFFFFF - 1;
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    uint16_t  state;

    esp_err_t res = pca9555_get_gpio_values(keyboard->pca, &state);
    if (res != ESP_OK) {
        *rc = 0xFFFFFFFF - 2;
        return false;
    }
    state &= 0x1ff;

    ESP_LOGI(TAG, "keyboard state: %04x", state);

    *rc = state;

    return (state == 0x0000);
}

/* Test routines */
bool test_nfc_init(uint32_t* rc) {
    ST25R3911B* nfc = get_nfc();
    if (nfc == NULL) {
        *rc = 0xFFFFFFFF - 1;
        return false;
    }

    uint8_t id = 0;
    esp_err_t res = st25r3911b_chip_id(&id);
    if (res != ESP_OK) {
        *rc = 0xFFFFFFFF - 2;
        return false;
    }

    *rc = id;

    return (*rc == 0x05);
}

bool test_nfc_read_uid(uint32_t* rc) {
    ST25R3911B* nfc = get_nfc();
    if (nfc == NULL) {
        *rc = 0xFFFFFFFF - 1;
        return false;
    }

    bool found = false;
    int tries = 30;
    int remaining =  tries;

    while (!found && remaining-- > 0) {
        esp_err_t res = st25r3911b_discover(NULL, 1000, DISCOVER_MODE_LISTEN_NFCA);
        if (res == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (res == ESP_OK) {
            found = true;
        }
    }



    *rc = (uint32_t) tries - remaining;
    return (found == true);
}

bool run_basic_tests() {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font;
    int               line = 0;
    bool              ok   = true;

    /* Screen init */
    font = pax_font_sky_mono;

    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0x8060f0);
    display_flush();

    /* Run mandatory tests */
    RUN_TEST_MANDATORY("KEYBOARD", test_keyboard_init);
    RUN_TEST_MANDATORY("NFC", test_nfc_init);

    /* Run tests */
    RUN_TEST("STUCK BUTTONS", test_stuck_buttons);
    RUN_TEST("SD CARD", test_sdcard_init);
    RUN_TEST("NFC READ", test_nfc_read_uid);

error:
    /* Fail result on screen */
    if (!ok) pax_draw_text(pax_buffer, 0xffff0000, font, 36, 0, 20 * line, "FAIL");
    display_flush();
    return ok;
}

uint8_t led_green[NUM_LEDS*3] = {0};
uint8_t led_red[NUM_LEDS*3]   = {0};
uint8_t led_blue[NUM_LEDS*3]  = {0};
uint8_t led_white[NUM_LEDS*3]  = {0};

void factory_test() {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_green[3*i] = 50;
        led_green[3*i+1] = 0;
        led_green[3*i+2] = 0;

        led_red[3*i] = 0;
        led_red[3*i+1] = 50;
        led_red[3*i+2] = 0;

        led_blue[3*i] = 0;
        led_blue[3*i+1] = 0;
        led_blue[3*i+2] = 50;

        led_white[3*i] = 50;
        led_white[3*i+1] = 50;
        led_white[3*i+2] = 50;
    }

    pax_buf_t* pax_buffer        = get_pax_buffer();
    uint8_t    factory_test_done = nvs_get_u8_default("system", "factory_test", 0);
    ESP_LOGI(TAG, "factory_test_done %d", factory_test_done);

    if (!key_currently_pressed(BUTTON_START) || !key_currently_pressed(BUTTON_SELECT)) {
        return;
    }

    st77xx_backlight(true);
    bool result;

    ESP_LOGI(TAG, "Factory test start");

    result = run_basic_tests();

    if (result) {
        ws2812_send_data(led_blue, sizeof(led_blue));
    } else {
        ws2812_send_data(led_red, sizeof(led_red));
    }

    if (result) {
        esp_err_t res = nvs_set_u8_fixed("system", "factory_test", 0x01);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to store test result %d\n", res);
            result = false;
            ws2812_send_data(led_red, sizeof(led_red));
            pax_noclip(pax_buffer);
            pax_background(pax_buffer, 0xa85a32);
            display_flush();
        }
        wifi_set_defaults();
        pax_noclip(pax_buffer);
        pax_background(pax_buffer, 0x00FF00);
        display_flush();
        ws2812_send_data(led_green, sizeof(led_green));

        ESP_LOGI(TAG, "Make sure the speaker is NOT muted and a sound is playing");
        pax_draw_text(pax_buffer, 0xffff0000, pax_font_sky_mono, 36, 0, 20, "SUCCESS!");
        pax_draw_text(pax_buffer, 0xffff0000, pax_font_sky_mono, 16, 0, 56, "Does the speaker work?");
        display_flush();

        while (true) {
            play_bootsound();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
