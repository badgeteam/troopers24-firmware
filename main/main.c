#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <pax_codecs.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#include "appfs.h"
#include "appfs_wrapper.h"
#include "audio.h"
#include "bootscreen.h"
#include "driver/uart.h"
#include "efuse.h"
#include "esp32/rom/crc.h"
#include "esp_ota_ops.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "factory_test.h"
#include "filesystems.h"
#include "graphics_wrapper.h"
#include "gui_element_header.h"
#include "hardware.h"
#include "menus/start.h"
#include "pax_gfx.h"
#include "pca9555.h"
#include "rtc_memory.h"
#include "settings.h"
#include "system_wrapper.h"
#include "wifi_cert.h"
#include "wifi_connection.h"
#include "wifi_defaults.h"
#include "wifi_ota.h"
#include "ws2812.h"
#include "keyboard.h"

extern const uint8_t logo_screen_png_start[] asm("_binary_logo_screen_png_start");
extern const uint8_t logo_screen_png_end[] asm("_binary_logo_screen_png_end");

static const char* TAG = "main";

void display_fatal_error(const char* line0, const char* line1, const char* line2, const char* line3) {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font       = pax_font_saira_regular;
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xa85a32);
    if (line0 != NULL) pax_draw_text(pax_buffer, 0xFFFFFFFF, font, 23, 0, 20 * 0, line0);
    if (line1 != NULL) pax_draw_text(pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 1, line1);
    if (line2 != NULL) pax_draw_text(pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 2, line2);
    if (line3 != NULL) pax_draw_text(pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 3, line3);
    display_flush();
}

void stop() {
    ESP_LOGW(TAG, "*** HALTED ***");
    uint8_t led_off[15]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t led_red[15]  = {0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0};
    uint8_t led_red2[15] = {0, 0xFF, 0, 0, 0xFF, 0, 0, 0xFF, 0, 0, 0xFF, 0, 0, 0xFF, 0};
    while (true) {
        ws2812_send_data(led_red2, sizeof(led_red2));
        vTaskDelay(pdMS_TO_TICKS(200));
        ws2812_send_data(led_red, sizeof(led_red));
        vTaskDelay(pdMS_TO_TICKS(200));
        ws2812_send_data(led_off, sizeof(led_off));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

const char* fatal_error_str = "A fatal error occured";
const char* reset_board_str = "Reset the board to try again";

static void audio_player_task(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    play_bootsound();
    uint8_t leds[15] = {0};
    for (uint8_t part = 0; part < 50; part++) {
        // Center of the kite: green.
        leds[3 * 0 + 1] = part;
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (uint8_t part = 0; part < 50; part++) {
        // Left of the kite: red.
        leds[3 * 1 + 0] = part;
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (uint8_t part = 0; part < 50; part++) {
        // Top of the kit: blue.
        leds[3 * 2 + 2] = part;
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (uint8_t part = 0; part < 50; part++) {
        // Right of the kite: yellow.
        leds[3 * 3 + 0] = part;
        leds[3 * 3 + 1] = part;
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (uint8_t part = 0; part < 50; part++) {
        // Bottom of the kite: blue.
        leds[3 * 4 + 2] = part;
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (uint8_t part = 0; part < 50; part++) {
        // Center of the kite: green.
        leds[3 * 0 + 1] = 49 - part;
        // Left of the kite: red.
        leds[3 * 1 + 0] = 49 - part;
        // Top of the kit: blue.
        leds[3 * 2 + 2] = 49 - part;
        // Right of the kite: yellow.
        leds[3 * 3 + 0] = 49 - part;
        leds[3 * 3 + 1] = 49 - part;
        // Bottom of the kite: blue.
        leds[3 * 4 + 2] = 49 - part;

        // Send the LED data.
        ws2812_send_data(leds, sizeof(leds));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t res;

    audio_init();

    const esp_app_desc_t* app_description = esp_ota_get_app_description();
    printf("BADGE.TEAM %s launcher firmware v%s\r\n", app_description->project_name, app_description->version);

    /* Initialize hardware */

    efuse_protect();

    if (bsp_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize basic board support functions");
        esp_restart();
    }

    /* Initialize the LEDs */
    ws2812_init(GPIO_LED_DATA, 150);
    const uint8_t led_off[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    ws2812_send_data(led_off, sizeof(led_off));

    /* Turning the backlight on */
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1LL << GPIO_LCD_BL,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    res = gpio_config(&io_conf);
    printf("set pin direction");
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "LCD Backlight set_direction failed: %d", res);
        display_fatal_error(fatal_error_str, "Failed to set LCD backlight pin mode", "Flash may be corrupted", reset_board_str);
        stop();
    }
    res = gpio_set_level(GPIO_LCD_BL, true);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "LCD Backlight set_level failed: %d", res);
        display_fatal_error(fatal_error_str, "Failed to turn on LCD backlight", "Flash may be corrupted", reset_board_str);
        stop();
    }

    /* Initialize LCD screen */
    pax_buf_t* pax_buffer = get_pax_buffer();
    display_boot_screen("Starting...");

    /* Start NVS */
    res = nvs_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %d", res);
        display_fatal_error(fatal_error_str, "NVS failed to initialize", "Flash may be corrupted", NULL);
        stop();
    }

    nvs_handle_t handle;
    res = nvs_open("system", NVS_READWRITE, &handle);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %d", res);
        display_fatal_error(fatal_error_str, "Failed to open NVS namespace", "Flash may be corrupted", reset_board_str);
        stop();
    }

    factory_test();

    /* Start AppFS */
    res = appfs_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "AppFS init failed: %d", res);
        display_fatal_error(fatal_error_str, "Failed to initialize AppFS", "Flash may be corrupted", reset_board_str);
        stop();
    }

    /* Start internal filesystem */
    if (mount_internal_filesystem() != ESP_OK) {
        display_fatal_error(fatal_error_str, "Failed to initialize flash FS", "Flash may be corrupted", reset_board_str);
        stop();
    }

    /* Start SD card filesystem */
    bool sdcard_mounted = (mount_sdcard_filesystem() == ESP_OK);
    if (sdcard_mounted) {
        ESP_LOGI(TAG, "SD card filesystem mounted");
    }

    /* Start WiFi */
    wifi_init();

    if (!wifi_check_configured()) {
        if (wifi_set_defaults()) {
            const pax_font_t* font = pax_font_saira_regular;
            pax_background(pax_buffer, 0xFFFFFF);
            pax_draw_text(pax_buffer, 0xFF000000, font, 18, 5, 240 - 18, "🅰 continue");
            render_message("Default WiFi settings\nhave been restored!\nPress A to continue...");
            display_flush();
            wait_for_button();
        } else {
            display_fatal_error(fatal_error_str, "Failed to configure WiFi", "Flash may be corrupted", reset_board_str);
            stop();
        }
    }

    res = init_ca_store();
    if (res != ESP_OK) {
        display_fatal_error(fatal_error_str, "Failed to initialize", "TLS certificate storage", reset_board_str);
        stop();
    }

    /* Clear RTC memory */
    rtc_memory_clear();

    /* Crash check */
    appfs_handle_t crashed_app = appfs_detect_crash();
    if (crashed_app != APPFS_INVALID_FD) {
        const char* app_name = NULL;
        appfsEntryInfo(crashed_app, &app_name, NULL);
        pax_background(pax_buffer, 0xFFFFFF);
        render_header(pax_buffer, 0, 0, pax_buffer->width, 34, 18, 0xFFfa448c, 0xFF491d88, NULL, "App crashed");
        pax_draw_text(pax_buffer, 0xFF491d88, pax_font_saira_regular, 18, 5, 52, "Failed to start app,");
        pax_draw_text(pax_buffer, 0xFF491d88, pax_font_saira_regular, 18, 5, 52 + 20, "check console for more details.");
        if (app_name != NULL) {
            char buffer[64];
            buffer[sizeof(buffer) - 1] = '\0';
            snprintf(buffer, sizeof(buffer) - 1, "App: %s", app_name);
            pax_draw_text(pax_buffer, 0xFF491d88, pax_font_saira_regular, 18, 5, 52 + 40, buffer);
        }
        pax_draw_text(pax_buffer, 0xFF491d88, pax_font_saira_regular, 18, 5, pax_buffer->height - 18, "🅰 continue");
        display_flush();
        wait_for_button();
    }

    /* Rick that roll */
    xTaskCreate(audio_player_task, "audio_player_task", 2048, NULL, 12, NULL);

    /* Launcher menu */
    while (true) {
        // TODO: Use the queue of our own keyboard module
        menu_start(get_keyboard()->queue, app_description->version);
    }

    nvs_close(handle);
}
