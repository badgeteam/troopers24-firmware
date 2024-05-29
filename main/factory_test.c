#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "filesystems.h"

#include "audio.h"
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

    while (i < 10) {
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

bool test_keyboard_init(uint32_t* rc) {
    Keyboard* keyboard = get_keyboard();
    if (keyboard == NULL) {
        *rc = (uint32_t) 1;
        return false;
    }

    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font = pax_font_sky_mono;


    ESP_LOGI(TAG, "Press START...");
    pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
    pax_draw_text(pax_buffer, 0xFF0000FF, font, 36, 0, pax_buffer->height - 36, "Press START");
    display_flush();
    if (!wait_for_key_pressed(keyboard, BUTTON_START)) {
        ESP_LOGE(TAG, "Timeout reached");
        pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
        display_flush();
        *rc = (uint32_t) 2;
        return false;
    }

    pax_simple_rect(pax_buffer, 0xFFFFFFFF, 0, pax_buffer->height - 36, pax_buffer->width, 36);
    display_flush();

    return (keyboard != NULL);
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

    char* test_str = "TROOPERS23 TEST";
    int test_str_len = 15;

    fwrite(test_str, 1, test_str_len, test_fd);
    fclose(test_fd);

    ESP_LOGI(TAG, "Wrote file, trying to read. You should see \"TROOPERS23 TEST\" in the next line");

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
        *rc = (uint32_t) 1;
        return false;
    }

    // TODO: Replace
//    RP2040*   rp2040 = get_rp2040();
    uint16_t  state = 0;
//    esp_err_t res = rp2040_read_buttons(rp2040, &state);
//    if (res != ESP_OK) {
//        *rc = 0xFFFFFFFF;
//        return false;
//    }

//    state &= ~(1 << FPGA_CDONE);  // Ignore FPGA CDONE

    *rc = state;

    return (state == 0x0000);
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

    /* Run tests */
    RUN_TEST("STUCK BUTTONS", test_stuck_buttons);
    RUN_TEST("SD CARD", test_sdcard_init);

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
    if (!factory_test_done) {
        bool result;

        ESP_LOGI(TAG, "Factory test start");

        result = run_basic_tests();

        if (result) {
            ws2812_send_data(led_blue, sizeof(led_blue));
        } else {
            ws2812_send_data(led_red, sizeof(led_red));
        }

        if (!result) goto test_end;

    // Wait for the operator to unplug the badge
    test_end:

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
            nvs_set_u8_fixed("system", "force_sponsors", 0x01);  // Force showing sponsors on first boot
            wifi_set_defaults();
            pax_noclip(pax_buffer);
            pax_background(pax_buffer, 0x00FF00);
            display_flush();
            ws2812_send_data(led_green, sizeof(led_green));

            ESP_LOGI(TAG, "Make sure the speaker is NOT muted and a sound is playing");

            while (true) {
                if (result) play_bootsound();
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
        }

        while (true) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
