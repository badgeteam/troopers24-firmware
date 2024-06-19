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

static const char* TAG = "contacts";

static const char* self_path        = "/internal/apps/contacts/self.json";
static const char* database_path    = "/internal/apps/contacts/db.json";

static const char* DEFAULT_SELF = "{\"id\": 0, \"name\": null, \"tel\": null, \"email\": null, \"url\": null, \"nick\": null}";
static const char* DEFAULT_DATABASE = "{}";

char VCARD[MAX_NFC_BUFFER_SIZE];

extern const uint8_t agenda_png_start[] asm("_binary_calendar_png_start");
extern const uint8_t agenda_png_end[] asm("_binary_calendar_png_end");

extern const uint8_t clock_png_start[] asm("_binary_clock_png_start");
extern const uint8_t clock_png_end[] asm("_binary_clock_png_end");

extern const uint8_t bookmark_png_start[] asm("_binary_bookmark_png_start");
extern const uint8_t bookmark_png_end[] asm("_binary_bookmark_png_end");

typedef enum action {
    ACTION_NONE,
    ACTION_EDIT_SELF,
    ACTION_SHARE,
    ACTION_IMPORT,
} menu_contacts_action_t;

#define MIN(a, b) ((a < b) ? a : b)

static char*  data_self = NULL;
static size_t size_self = 0;
static cJSON* json_self = NULL;

static char*  data_db = NULL;
static size_t size_db = 0;
static cJSON* json_db = NULL;

static void agenda_render_background(pax_buf_t* pax_buffer) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅰 Accept 🅱 Exit");
}

static void details_render_background(pax_buf_t* pax_buffer, bool tracks, bool days) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    if (tracks) {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back  ← → Track 🅴 Bookmark");
    } else if (days) {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back  ← → Day 🅴 Bookmark");
    } else {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 Back 🅴 Bookmark");
    }
}

static void render_topbar(pax_buf_t* pax_buffer, pax_buf_t* icon, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_simple_rect(pax_buffer, 0xff131313, 0, 0, 320, 34);
    pax_draw_image(pax_buffer, icon, 1, 1);
    pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 34, 8, text);
}

static void details_day(pax_buf_t* pax_buffer, xQueueHandle button_queue, cJSON* data, cJSON* my_day, cJSON* bookmarks, pax_buf_t* icon_top, pax_buf_t* icon_bookmarked) {
//    int track = 0;
//    cJSON* tracks = cJSON_GetObjectItem(data, "tracks");
//    int track_count = cJSON_GetArraySize(tracks);
//    if (track_count == 0) {
//        render_message("No tracks found");
//        display_flush();
//        wait_for_button();
//        return;
//    }
//
//    bool render = true;
//    bool exit = false;
//
//    int    talk         = 0;
//    int    render_talks = 3;
//    int slot_height = 62;
//    int talk_count;
//    keyboard_input_message_t buttonMessage = {0};
//
//    cJSON* talks = cJSON_GetObjectItem(cJSON_GetArrayItem(tracks, track), "talks");
//    cJSON* talk_data;
//
//    while(!exit) {
//        if (render) {
//            talk_count = render_track(pax_buffer, icon_top, icon_bookmarked, tracks, track, talk, render_talks, slot_height);
//            render = false;
//        }
//
//        clear_keyboard_queue();
//        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
//            if (buttonMessage.state) {
//                switch (buttonMessage.input) {
//                    case JOYSTICK_DOWN:
//                        talk = (talk + 1) % talk_count;
//                        render = true;
//                        break;
//                    case JOYSTICK_UP:
//                        talk = (talk - 1 + talk_count) % talk_count;
//                        render = true;
//                        break;
//                    case JOYSTICK_LEFT:
//                        track = (track - 1 + track_count) % track_count;
//                        // TODO: Do we need to reset the talk?
//                        // talk = 0;
//                        render = true;
//                        break;
//                    case JOYSTICK_RIGHT:
//                        track = (track + 1) % track_count;
//                        // TODO: Do we need to reset the talk?
//                        // talk = 0;
//                        render = true;
//                        break;
//                    case BUTTON_BACK:
//                        exit = true;
//                        break;
//                    case BUTTON_SELECT:
//                    case JOYSTICK_PUSH:
//                        talk_data = cJSON_GetArrayItem(talks, talk);
//                        if (cJSON_IsTrue(cJSON_GetObjectItem(talk_data, "special"))) {
//                            // Don't allow adding breaks to bookmarks
//                            break;
//                        }
//                        toggle_bookmark(cJSON_GetArrayItem(tracks, track), talk_data, my_day, bookmarks);
//                        render = true;
//                        break;
//                    default:
//                        break;
//                }
//            }
//        }
//    }
}

static void my_agenda(pax_buf_t* pax_buffer, xQueueHandle button_queue, cJSON* day1, cJSON* day2, pax_buf_t* icon) {
    return;
//    if (day1 == NULL || day2 == NULL) {
//        render_message("No talks found");
//        display_flush();
//        wait_for_button();
//        return;
//    }
//
//    bool render = true;
//    bool exit = false;
//
//    int days = 2;
//    int day = 0;
//    int    talk         = 0;
//    int    render_talks = 3;
//    int slot_height = 62;
//    int talk_count;
//    keyboard_input_message_t buttonMessage = {0};
//
//
//    cJSON* data = day1;
//    cJSON* talk_data;
//    cJSON* my_day;
//    cJSON* bookmarks;
//
//    while(!exit) {
//        if (render) {
//            talk_count = render_bookmarks(pax_buffer, icon, data, day, talk, render_talks, slot_height);
//            render = false;
//        }
//
//        clear_keyboard_queue();
//        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
//            if (buttonMessage.state) {
//                switch (buttonMessage.input) {
//                    case JOYSTICK_DOWN:
//                        talk = (talk + 1) % talk_count;
//                        render = true;
//                        break;
//                    case JOYSTICK_UP:
//                        talk = (talk - 1 + talk_count) % talk_count;
//                        render = true;
//                        break;
//                    case JOYSTICK_LEFT:
//                        day = (day - 1 + days) % days;
//                        data = day == 0 ? day1 : day2;
//                        render = true;
//                        break;
//                    case JOYSTICK_RIGHT:
//                        day = (day + 1) % days;
//                        data = day == 0 ? day1 : day2;
//                        render = true;
//                        break;
//                    case BUTTON_SELECT:
//                    case JOYSTICK_PUSH:
//                        talk_data = cJSON_GetArrayItem(data, talk % talk_count);
//                        my_day = cJSON_GetObjectItem(json_my, (day == 0) ? "day1" : "day2");
//                        bookmarks = (day == 0) ? json_my_day1 : json_my_day2;
//                        toggle_bookmark(cJSON_GetObjectItem(talk_data, "track"), cJSON_GetObjectItem(talk_data, "talk"), my_day, bookmarks);
//                        render = true;
//                        break;
//                    case BUTTON_BACK:
//                        exit = true;
//                        break;
//                    default:
//                        break;
//                }
//            }
//        }
//    }
}

static uint find_correct_position(cJSON* contacts, long id) {
    cJSON* next = cJSON_GetArrayItem(contacts, 0);
    int i = 0;
    while(next != NULL && cJSON_GetNumberValue(cJSON_GetObjectItem(cJSON_GetObjectItem(next, "talk"), "start")) <= id) {
        i++;
        next = next->next;
    }
    return i;
}

static bool do_init() {
    if (file_exists(self_path)) {
        return true;
    }

    FILE* self_fd = fopen(self_path, "w");
    if (self_fd == NULL) {
        ESP_LOGE(TAG, "Failed to create own contact details file: %s", self_path);
        return false;
    }
    fwrite(DEFAULT_SELF, 1, strlen(DEFAULT_SELF), self_fd);
    fclose(self_fd);

    FILE* db_fd = fopen(database_path, "w");
    if (db_fd == NULL) {
        ESP_LOGE(TAG, "Failed to create contacts file: %s", database_path);
        return false;
    }
    fwrite(DEFAULT_DATABASE, 1, strlen(DEFAULT_SELF), db_fd);
    fclose(db_fd);

    return true;
}

static bool load_data() {
    if (!do_init()) {
        render_message("Failed to initialize files");
        display_flush();
        return false;
    }

    if (!load_file(TAG, self_path, &data_self, &size_self)) {
        ESP_LOGE(TAG, "Failed to read own contact details file: %s", self_path);
        render_message("Failed to read own contact details");
        display_flush();
        return false;
    }

    if (!load_file(TAG, database_path, &data_db, &size_db)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", database_path);
        render_message("Failed to read contacts");
        display_flush();
        return false;
    }


    if (json_self != NULL) {
        cJSON_Delete(json_self);
    }
    json_self = cJSON_ParseWithLength(data_self, size_self);
    if (json_self == NULL) {
        ESP_LOGE(TAG, "Failed to parse own contact details file: %s", self_path);
        render_message("Failed to parse own contact details");
        display_flush();
        return false;
    }

    if (!cJSON_HasObjectItem(json_self, "name") || cJSON_GetStringValue(cJSON_GetObjectItem(json_self, "name")) == NULL) {
        cJSON_DeleteItemFromObject(json_self, "name");
    }

    if (!cJSON_HasObjectItem(json_self, "name")) {
        char name[15] = {0};
        snprintf(name, 15, "Trooper #%d", badge_id());
        cJSON_AddStringToObject(json_self, "name", name);
    }

    // Always overwrite with badge_id
    if (cJSON_HasObjectItem(json_self, "id")) {
        cJSON_SetIntValue(cJSON_GetObjectItem(json_self, "id"), badge_id());
    } else {
        cJSON_AddNumberToObject(json_self, "id", badge_id());
    }

    ESP_LOGI(TAG, "%s", cJSON_Print(json_self));

    if (json_db != NULL) {
        cJSON_Delete(json_db);
    }
    json_db = cJSON_ParseWithLength(data_db, size_db);
    if (json_db == NULL) {
        ESP_LOGE(TAG, "Failed to parse contacts file: %s", database_path);
        render_message("Failed to parse contacts");
        display_flush();
        return false;
    }

    ESP_LOGI(TAG, "%s", cJSON_Print(json_db));

    return true;
}

static void append_str(char **dst, char *src, size_t len) {
    memcpy(*dst, src, len);
    *dst += len;
}

static void add_if_not_null(char **dst, const char *prefix, const char *key, size_t maxLen) {
    cJSON* elem = cJSON_GetObjectItem(json_self, key);
    char* str = NULL;
    if (cJSON_IsNumber(elem)) {
        char buf[4];
        sprintf(buf, "%d", ((uint16_t) cJSON_GetNumberValue(elem)) % 999);
        str = buf;
    } else {
        str = cJSON_GetStringValue(elem);
    }
    if (str == NULL) {
        return;
    }

    ESP_LOGI(TAG, "elem is string");
    append_str(dst, (char *) prefix, strlen(prefix));
    ESP_LOGI(TAG, "appended prefix");
    ESP_LOGI(TAG, "%s", VCARD);

    ESP_LOGI(TAG, "adding string str=%p", str);
    uint len = strlen(str);
    ESP_LOGI(TAG, "adding string len=%d, str=%s", len, str);
    append_str(dst, str, MIN(len, maxLen));
}

static void create_vcard(size_t *len) {
    memset(VCARD, 0, MAX_NFC_BUFFER_SIZE);
    char* current = VCARD;

//    append_str(&current, "jtext/vcard", 11);
    append_str(&current, "BEGIN:VCARD\n", 12);
    append_str(&current, "VERSION:3.0", 11);

    add_if_not_null(&current, "\nUID:", "id", 3);
    add_if_not_null(&current, "\nFN:", "name", 64);
    add_if_not_null(&current, "\nTEL:", "tel", 32);
    add_if_not_null(&current, "\nEMAIL:", "email", 128);
    add_if_not_null(&current, "\nURL:", "url", 242);

    append_str(&current, "\nEND:VCARD\n", 11);

    // maximum without header: 12 + 11 + 11 + 5 + 3 + 4 + 64 + 5 + 32 + 7 + 128 + 5 + 242 = 529 bytes
    // maximum:           11 + 12 + 11 + 11 + 5 + 3 + 4 + 64 + 5 + 32 + 7 + 128 + 5 + 242 = 540 bytes
    if (len != NULL) {
        *len = current - VCARD;
    }
}

static esp_err_t show_qr_code(char *text) {
    enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW;  // Error correction level

    // Make and print the QR Code symbol
    uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
    uint8_t tempBuffer[qrcodegen_BUFFER_LEN_MAX];
    bool ok = qrcodegen_encodeText(text, tempBuffer, qrcode, errCorLvl, qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
    if (!ok) {
        return ESP_FAIL;
    }

    int size = qrcodegen_getSize(qrcode);

    int maxWidth = ST77XX_WIDTH;
    int maxHeight = ST77XX_HEIGHT - 34 - 20;
    int pixelSize = MIN(maxWidth / size, maxHeight / size);
    int dx = (maxWidth - pixelSize * size) / 2;
    int dy = 34 + (maxHeight - pixelSize * size) / 2;

    int y, x;
    for (y = 0; y < size; y++) {
        for (x = 0; x < size; x++) {
            pax_draw_rect(get_pax_buffer(), qrcodegen_getModule(qrcode, x, y) ? 0xFFFFFFFF : 0xFF131313, dx + x * pixelSize, dy + y * pixelSize, pixelSize, pixelSize);
        }
    }
    display_flush();

    return ESP_OK;
}

static esp_err_t handle_device(rfalNfcDevice *nfcDevice) {
    ESP_LOGI(TAG, "Found NFC device");
    ndefConstBuffer bufConstRawMessage;

    esp_err_t res = st25r3911b_read_data(nfcDevice, &bufConstRawMessage);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read data: %d", res);
        return res;
    }

    printf("%.*s\n", bufConstRawMessage.length, bufConstRawMessage.buffer);
    return ESP_OK;
}

static esp_err_t handle_device_p2p(rfalNfcDevice *nfcDevice) {
    ESP_LOGI(TAG, "Found NFC device");



    ndefConstBuffer bufConstRawMessage;

    esp_err_t res = st25r3911b_read_data(nfcDevice, &bufConstRawMessage);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read data: %d", res);
        return res;
    }

    printf("%.*s\n", bufConstRawMessage.length, bufConstRawMessage.buffer);
    return ESP_OK;
}

static void read_nfc() {
    esp_err_t res;

    clear_keyboard_queue();
    ESP_LOGI(TAG, "Reading NFC");
    while (1) {
        res = st25r3911b_discover(&handle_device, 1000, DISCOVER_MODE_LISTEN_NFCA);
        if (res == ESP_ERR_TIMEOUT) {
            rtc_wdt_feed();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Retrying...");
            continue;
        }
        if (res == ESP_OK) {
            break;
        } else if (key_was_pressed(BUTTON_BACK)) {
            break;
        }
    }

    size_t len;
    create_vcard(&len);
    ESP_LOGI(TAG, "VCARD with len %d:\n%.*s\n", len, len, VCARD);
}

static void passive_p2p() {
    esp_err_t res;

    clear_keyboard_queue();
    ESP_LOGI(TAG, "Waiting for NFC P2P connection as PASSIVE");
    while (1) {
        res = st25r3911b_listen_p2p(1000);
        if (res == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "Retrying...");
            rtc_wdt_feed();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        if (res == ESP_OK) {
            break;
        } else if (key_was_pressed(BUTTON_BACK)) {
            break;
        }
    }

    size_t len;
    create_vcard(&len);
    ESP_LOGI(TAG, "VCARD with len %d:\n%.*s\n", len, len, VCARD);
}

static void p2p_active() {
    esp_err_t res;

    clear_keyboard_queue();
    ESP_LOGI(TAG, "Waiting for NFC P2P connection as PASSIVE");
    while (1) {
        res = st25r3911b_discover(&handle_device_p2p, 1000, DISCOVER_MODE_P2P_ACTIVE);
        if (res == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "Retrying...");
            rtc_wdt_feed();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        if (res == ESP_OK) {
            break;
        } else if (key_was_pressed(BUTTON_BACK)) {
            break;
        }
    }

    size_t len;
    create_vcard(&len);
    ESP_LOGI(TAG, "VCARD with len %d:\n%.*s\n", len, len, VCARD);
}

void menu_contacts(xQueueHandle button_queue) {
    pax_buf_t* pax_buffer = get_pax_buffer();

    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFF131313);

    // Ensure directory exists
    if (!create_dir("/internal/apps")) {
        ESP_LOGE(TAG, "Failed to create directory in internal storage");
        render_message("Failed to create data dir");
        display_flush();
        wait_for_button();
        return;
    }
    if (!create_dir("/internal/apps/contacts")) {
        ESP_LOGE(TAG, "Failed to create directory in internal storage");
        render_message("Failed to create data dir");
        display_flush();
        wait_for_button();
        return;
    }

    if (!load_data()) {
        wait_for_button();
        return;
    }

    create_vcard(NULL);
    show_qr_code(VCARD);

//    read_nfc();
    passive_p2p();
//    p2p_active();

    menu_t*    menu       = menu_alloc("TROOPERS24 - Agenda", 34, 18);

    menu->fgColor           = 0xFFF1AA13;
    menu->bgColor           = 0xFF131313;
    menu->bgTextColor       = 0xFF000000;
    menu->selectedItemColor = 0xFFF1AA13;
    menu->borderColor       = 0xFF1E1E1E;
    menu->titleColor        = 0xFFF1AA13;
    menu->titleBgColor      = 0xFF1E1E1E;
    menu->scrollbarBgColor  = 0xFFCCCCCC;
    menu->scrollbarFgColor  = 0xFF555555;

    pax_buf_t icon_agenda;
    pax_decode_png_buf(&icon_agenda, (void*) agenda_png_start, agenda_png_end - agenda_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_clock;
    pax_decode_png_buf(&icon_clock, (void*) clock_png_start, clock_png_end - clock_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_bookmark;
    pax_decode_png_buf(&icon_bookmark, (void*) bookmark_png_start, bookmark_png_end - bookmark_png_start, PAX_BUF_32_8888ARGB, 0);

    menu_set_icon(menu, &icon_agenda);
//    menu_insert_item_icon(menu, "Bookmarks", NULL, (void*) ACTION_MY_AGENDA, -1, &icon_bookmark);
//    if (ntp_synced) {
//        menu_insert_item_icon(menu, "Next up", NULL, (void*) ACTION_NEXT_UP, -1, &icon_clock);
//    }
//    menu_insert_item_icon(menu, "Wednesday", NULL, (void*) ACTION_WEDNESDAY, -1, &icon_agenda);
//    menu_insert_item_icon(menu, "Thursday", NULL, (void*) ACTION_THURSDAY, -1, &icon_agenda);

    bool                render = true;
    menu_contacts_action_t action = ACTION_NONE;

    bool full_redraw = true;
    bool exit = false;
    while (!exit) {
        if (render) {
            if (full_redraw) {
                agenda_render_background(pax_buffer);
            }

            if (full_redraw) {
                menu_render_grid(pax_buffer, menu, 0, 0, 320, 220);
                display_flush();
            } else {
                menu_render_grid_changes(pax_buffer, menu, 0, 0, 320, 220);
                display_flush();
            }

            render      = false;
            full_redraw = false;
        }

        clear_keyboard_queue();
        keyboard_input_message_t buttonMessage = {0};
        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            if (buttonMessage.state) {
                switch (buttonMessage.input) {
                    case JOYSTICK_DOWN:
                        menu_navigate_next_row(menu);
                        render = true;
                        full_redraw = true;
                        break;
                    case JOYSTICK_UP:
                        menu_navigate_previous_row(menu);
                        render = true;
                        full_redraw = true;
                        break;
                    case JOYSTICK_LEFT:
                        menu_navigate_previous(menu);
                        render = true;
                        break;
                    case JOYSTICK_RIGHT:
                        menu_navigate_next(menu);
                        render = true;
                        break;
                    case BUTTON_BACK:
                        exit = true;
                        break;
                    case BUTTON_ACCEPT:
                    case BUTTON_SELECT:
                        action = (menu_contacts_action_t) menu_get_callback_args(menu, menu_get_position(menu));
                        break;
                    default:
                        break;
                }
            }
        }

        if (action != ACTION_NONE) {
//            if (action == ACTION_NEXT_UP) {
//                details_upcoming(pax_buffer, get_current_day(), &icon_clock);
//            } else if (action == ACTION_MY_AGENDA) {
//                my_agenda(pax_buffer, button_queue, json_my_day1, json_my_day2, &icon_bookmark);
//            } else if (action == ACTION_WEDNESDAY) {
//                details_day(pax_buffer, button_queue, json_day1, cJSON_GetObjectItem(json_my, "day1"), json_my_day1, &icon_agenda, &icon_bookmark);
//            } else if (action == ACTION_THURSDAY) {
//                details_day(pax_buffer, button_queue, json_day2, cJSON_GetObjectItem(json_my, "day2"), json_my_day2, &icon_agenda, &icon_bookmark);
//            }
            action      = ACTION_NONE;
            render      = true;
            full_redraw = true;
        }
    }

    menu_free(menu);

//    cJSON_Delete(json_my_day1);
//    json_my_day1 = NULL;
//
//    cJSON_Delete(json_my_day2);
//    json_my_day2 = NULL;
//
//    cJSON_Delete(json_my);
//    json_my = NULL;
//
//    // Delete the data loaded from JSON
//    cJSON_Delete(json_day1);
//    json_day1 = NULL;
//    cJSON_Delete(json_day2);
//    json_day2 = NULL;

    pax_buf_destroy(&icon_agenda);
    pax_buf_destroy(&icon_clock);
}
