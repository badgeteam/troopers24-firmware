#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "app_management.h"
#include "efuse.h"
#include "esp_http_client.h"
#include "graphics_wrapper.h"
#include "hardware.h"
#include "http_download.h"
#include "menu.h"
#include "ntp_helper.h"
#include "pax_codecs.h"
#include "pax_gfx.h"
#include "rtc_wdt.h"
#include "system_wrapper.h"
#include "utils.h"
#include "wifi_connect.h"
#include "qrcodegen.h"

static const char* TAG = "nfcreader";

char VCARD[MAX_NFC_BUFFER_SIZE];

extern const uint8_t badge_png_start[] asm("_binary_badge_png_start");
extern const uint8_t badge_png_end[] asm("_binary_badge_png_end");

static void render_background(pax_buf_t* pax_buffer, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, text);
}

static void render_topbar(pax_buf_t* pax_buffer, pax_buf_t* icon, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_simple_rect(pax_buffer, 0xff131313, 0, 0, 320, 34);
    pax_draw_image(pax_buffer, icon, 1, 1);
    pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 34, 8, text);
}

static esp_err_t handle_device(rfalNfcDevice *nfcDevice) {
    ESP_LOGI(TAG, "Found NFC device");
    ndefConstBuffer bufConstRawMessage;

    esp_err_t res = st25r3911b_read_data(nfcDevice, &bufConstRawMessage);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read data: %d", res);
        return res;
    }

    // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.3.0%2Fnfc_ndef_format_dox.html
    uint8_t tnf = bufConstRawMessage.buffer[0] & 0b111;
    // Short records have 1 byte length field, otherwise 4
    bool sr = bufConstRawMessage.buffer[0] & (1 << 4);
    bool il = bufConstRawMessage.buffer[0] & (1 << 3);
    int i = 1;
    uint8_t type_len = bufConstRawMessage.buffer[i++];
    uint32_t length = bufConstRawMessage.buffer[i++];
    if (!sr) {
        length = length << 24;
        length += bufConstRawMessage.buffer[i++] << 16;
        length += bufConstRawMessage.buffer[i++] << 8;
        length += bufConstRawMessage.buffer[i++];
    }
    uint8_t id_len = 0;
    if (il) {
        // ID length
        id_len = bufConstRawMessage.buffer[i++];
    }
    i += type_len;
    i += id_len;
    int payload_start = i;


#define MAX_PER_LINE 25
#define MAX_LINES 8
    char lines[MAX_LINES * (MAX_PER_LINE + 1)] = {0};
    int cursor = 0;
    char c;

    for (i = 0; i < length; i++) {
        c = (char) bufConstRawMessage.buffer[i + payload_start];
        if (c < 33 || c > 126) {
            // Skip non ASCII
            continue;
        }
        if ((cursor > 0 && (cursor + 1) % (MAX_PER_LINE + 1) == 0) && cursor + 1 < MAX_LINES * (MAX_PER_LINE + 1)) {
            lines[cursor++] = '\n';
        }
        if (cursor == MAX_LINES * (MAX_PER_LINE + 1) - 1) {
            lines[cursor] = 0;
            break;
        }
        lines[cursor++] = c;
    }

    char hint[20];
    sprintf(hint, "Received %4d bytes", length);

    pax_draw_text(get_pax_buffer(), 0xffeaa307, pax_font_saira_regular, 12, 4, 40, hint);
    pax_draw_text(get_pax_buffer(), 0xffeaa307, pax_font_sky_mono, 16, 4, 58, lines);
    display_flush();

    ESP_LOGI(TAG, "%.*s\n", length, bufConstRawMessage.buffer + payload_start);
    return ESP_OK;
}

static void read_nfc() {
    esp_err_t res;

    clear_keyboard_queue();
    ESP_LOGI(TAG, "Reading NFC");
    while (1) {
        res = st25r3911b_discover(&handle_device, 1000, DISCOVER_MODE_LISTEN_NFCA);
        if (res == ESP_ERR_TIMEOUT) {
            if (key_was_pressed(BUTTON_BACK)) {
                break;
            }
            rtc_wdt_feed();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Retrying...");
            continue;
        }
    }
}

void menu_nfcreader(xQueueHandle button_queue) {
    pax_buf_t* pax_buffer = get_pax_buffer();

    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFF131313);
    render_background(pax_buffer, "🅱 Cancel");
    pax_buf_t icon_badge;
    pax_decode_png_buf(&icon_badge, (void*) badge_png_start, badge_png_end - badge_png_start, PAX_BUF_32_8888ARGB, 0);
    render_topbar(pax_buffer, &icon_badge, "Read NFC-A Tag");
    display_flush();

    read_nfc();

    pax_buf_destroy(&icon_badge);
}
