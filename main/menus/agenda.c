#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "app_management.h"
#include "esp_http_client.h"
#include "graphics_wrapper.h"
#include "hardware.h"
#include "http_download.h"
#include "ntp_helper.h"
#include "menu.h"
#include "pax_codecs.h"
#include "pax_gfx.h"
#include "system_wrapper.h"
#include "wifi_connect.h"

static const char* TAG = "agenda";

#define DEBUG_INFRA 1

static const char* last_update_path = "/internal/apps/agenda/last_update";
static const char* day1_path        = "/internal/apps/agenda/0.json";
static const char* day1_path_tmp    = "/internal/apps/agenda/0.json.tmp";
static const char* day2_path        = "/internal/apps/agenda/1.json";
static const char* day2_path_tmp    = "/internal/apps/agenda/1.json.tmp";

#if DEBUG_INFRA
static const char* last_update_url  = "http://con.troopers.de/agenda/last_update";
static const char* day1_url         = "http://con.troopers.de/agenda/0.json";
static const char* day2_url         = "http://con.troopers.de/agenda/1.json";
#else
static const char* last_update_url  = "https://con.troopers.de/agenda/last_update";
static const char* day1_url         = "https://con.troopers.de/agenda/0.json";
static const char* day2_url         = "https://con.troopers.de/agenda/1.json";
#endif


extern const uint8_t agenda_png_start[] asm("_binary_calendar_png_start");
extern const uint8_t agenda_png_end[] asm("_binary_calendar_png_end");

extern const uint8_t clock_png_start[] asm("_binary_clock_png_start");
extern const uint8_t clock_png_end[] asm("_binary_clock_png_end");

typedef enum action {
    ACTION_NONE,
    ACTION_NEXT_UP,
    ACTION_WEDNESDAY,
    ACTION_THURSDAY
} menu_agenda_action_t;


static char*  data_day1 = NULL;
static size_t size_day1 = 0;
static cJSON* json_day1 = NULL;

static char*  data_day2 = NULL;
static size_t size_day2 = 0;
static cJSON* json_day2 = NULL;

void agenda_render_background(pax_buf_t* pax_buffer) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅰 accept 🅱 back");
}

void details_render_background(pax_buf_t* pax_buffer, bool tracks) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    if (tracks) {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 back  ← → change track");
    } else {
        pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 back");
    }
}

void render_topbar(pax_buf_t* pax_buffer, pax_buf_t* icon, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_simple_rect(pax_buffer, 0xff131313, 0, 0, 320, 34);
    pax_draw_image(pax_buffer, icon, 1, 1);
    pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 34, 8, text);
}

int render_track(pax_buf_t* pax_buffer, pax_buf_t* icon, cJSON* tracks, int track, int talk, int render_talks, int slot_height) {
    const pax_font_t* font = pax_font_saira_regular;

    cJSON* track_data = cJSON_GetArrayItem(tracks, track);
    cJSON* title = cJSON_GetObjectItem(track_data, "title");
    cJSON* talks = cJSON_GetObjectItem(track_data, "talks");

    int talk_count = cJSON_GetArraySize(talks);
    if (talk >= talk_count) {
        talk = talk_count - 1;
    }

    details_render_background(pax_buffer, true);
    render_topbar(pax_buffer, icon, title->valuestring);

    // First talk
    int start_talk = talk;
    int end_talk = talk + render_talks - 1;
    if (end_talk >= talk_count) {
        start_talk -= end_talk - talk_count + 1;
        end_talk = talk_count - 1;
    }
    if (start_talk < 0) {
        start_talk = 0;
    }

    for (int current = start_talk; current <= end_talk; current++) {
        int i = current - start_talk;
        int y = i * slot_height + 34;
        pax_col_t bg_color = (current == talk) ? 0xFFF1AA13 : 0xFF131313;
        pax_col_t fg_color = (current == talk) ? 0xFF131313 : 0xFFF1AA13;
        pax_simple_rect(pax_buffer, bg_color, 0, y, 320, slot_height);

        cJSON* talk_data = cJSON_GetArrayItem(talks, current);
        cJSON* time = cJSON_GetObjectItem(talk_data, "time");
        cJSON* talk_title = cJSON_GetObjectItem(talk_data, "title");
        cJSON* speakers = cJSON_GetObjectItem(talk_data, "speakers");

        pax_draw_text(pax_buffer, fg_color, font, 14, 5, y + 2, time->valuestring);

        pax_draw_text(pax_buffer, fg_color, font, 18, 5, y + 2 + 18, talk_title->valuestring);

        int max_len = 255;
        char speaker_str[max_len];
        speaker_str[max_len-1] = 0;
        uint offset = 1;
        for (int a = 0; a < cJSON_GetArraySize(speakers); a++) {
            cJSON* speaker = cJSON_GetArrayItem(speakers, a);
            uint len = strlen(speaker->valuestring);
            strncat(speaker_str, speaker->valuestring, max_len - 1 - offset);
            offset += len;
            if (a < cJSON_GetArraySize(speakers) - 1) {
                strncat(speaker_str, ", ", max_len - 1 - offset);
                offset += 2;
            }
            if (offset >= max_len - 2) {
                break;
            }
        }

        pax_draw_text(pax_buffer, fg_color, font, 14, 15, y + 2 + 18 + 20, speaker_str);
    }

    display_flush();
    return talk_count;
}

void details_day(pax_buf_t* pax_buffer, xQueueHandle button_queue, cJSON* data, pax_buf_t* icon) {
    int track = 0;
    cJSON* tracks = cJSON_GetObjectItem(data, "tracks");
    int track_count = cJSON_GetArraySize(tracks);
    if (track_count == 0) {
        render_message("No tracks found");
        display_flush();
        wait_for_button();
        return;
    }

    bool render = true;
    bool exit = false;

    int    talk         = 0;
    int    render_talks = 3;
    int slot_height = 62;
    int talk_count;
    keyboard_input_message_t buttonMessage = {0};

    while(!exit) {
        if (render) {
            talk_count = render_track(pax_buffer, icon, tracks, track, talk, render_talks, slot_height);
            render = false;
        }

        clear_keyboard_queue();
        if (xQueueReceive(button_queue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            if (buttonMessage.state) {
                switch (buttonMessage.input) {
                    case JOYSTICK_DOWN:
                        talk = (talk + 1) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_UP:
                        talk = (talk - 1 + talk_count) % talk_count;
                        render = true;
                        break;
                    case JOYSTICK_LEFT:
                        track = (track - 1 + track_count) % track_count;
                        // TODO: Do we need to reset the talk?
                        // talk = 0;
                        render = true;
                        break;
                    case JOYSTICK_RIGHT:
                        track = (track + 1) % track_count;
                        // TODO: Do we need to reset the talk?
                        // talk = 0;
                        render = true;
                        break;
                    case BUTTON_BACK:
                        exit = true;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

void details_upcoming(pax_buf_t* pax_buffer, cJSON* data, pax_buf_t* icon) {
    const pax_font_t* font = pax_font_saira_regular;

    if (data == NULL) {
        render_message("No talks found");
        display_flush();
        wait_for_button();
        return;
    }

    cJSON* tracks = cJSON_GetObjectItem(data, "tracks");
    int track_count = cJSON_GetArraySize(tracks);
    if (track_count == 0) {
        render_message("No tracks found");
        display_flush();
        wait_for_button();
        return;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    details_render_background(pax_buffer, false);
    render_topbar(pax_buffer, icon, "Upcoming talks");

    // Remaining vertical space is 186px -> 62px per track
    for (int track = 0; track < track_count; track++) {
        cJSON* track_data = cJSON_GetArrayItem(tracks, track);
        cJSON* title = cJSON_GetObjectItem(track_data, "title");
        cJSON* talks = cJSON_GetObjectItem(track_data, "talks");

        int talk1, talk2 = -1;

        int talk_count = cJSON_GetArraySize(talks);
        for (int talk = 0; talk < talk_count; talk++) {
            cJSON* talk_data = cJSON_GetArrayItem(talks, talk);
            cJSON* start = cJSON_GetObjectItem(talk_data, "ts");
            long ts = (long) cJSON_GetNumberValue(start);
            if (ts > now) {
                talk2 = talk;
                break;
            }
        }

        if (talk2 == 0) {
            talk1 = 0;
            talk2 = 1;
        } else if (talk2 == talk_count - 1) {
            talk1 = talk2;
            talk2 = -1;
        } else {
            talk1 = talk2 - 1;
        }

        // Draw track title
        pax_simple_rect(pax_buffer, 0xFFF1AA13, 0, 34 + 62 * track, 320, 22);
        pax_draw_text(pax_buffer, 0xff131313, font, 18, 5, 34 + 62 * track + 2, title->valuestring);

        // First talk
        pax_simple_rect(pax_buffer, 0xff131313, 0, 34 + 62 * track + 22, 320, 20);
        if (talk1 >= 0) {
            cJSON* talk1_data = cJSON_GetArrayItem(talks, talk1);
            cJSON* talk1_title = cJSON_GetObjectItem(talk1_data, "title");
            cJSON* talk1_start = cJSON_GetObjectItem(talk1_data, "time");

            pax_vec1_t talk1_start_size = pax_text_size(font, 18, talk1_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5, 34 + 62 * track + 22 + 2, talk1_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5 + talk1_start_size.x + 5, 34 + 62 * track + 22 + 2, talk1_title->valuestring);
        }

        // Second talk
        pax_simple_rect(pax_buffer, 0xff131313, 0, 34 + 62 * track + 22 + 20, 320, 20);
        if (talk2 >= 0) {
            cJSON* talk2_data = cJSON_GetArrayItem(talks, talk2);
            cJSON* talk2_title = cJSON_GetObjectItem(talk2_data, "title");
            cJSON* talk2_start = cJSON_GetObjectItem(talk2_data, "time");

            pax_vec1_t talk2_start_size = pax_text_size(font, 18, talk2_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5, 34 + 62 * track + 22 + 20 + 2, talk2_start->valuestring);
            pax_draw_text(pax_buffer, 0xFFF1AA13, font, 18, 5 + talk2_start_size.x + 5, 34 + 62 * track + 22 + 20 + 2, talk2_title->valuestring);
        }
    }
    display_flush();
    wait_for_button();
}

bool need_update(unsigned long *remote_last_update) {
    FILE* last_update_fd = fopen(last_update_path, "r");
    if (last_update_fd == NULL) {
        return true;
    }

    char* buf = malloc(11);
    buf[10] = '\0';
    fread(buf, 1, 10, last_update_fd);
    fclose(last_update_fd);

    unsigned long local_last_update = atol(buf);

    static char*  remote_buf = NULL;
    static size_t remote_buf_len = 0;
    bool success = download_ram(last_update_url, (uint8_t**) &remote_buf, &remote_buf_len);
    if (!success) return false; // Do not try to update if the first requests didn't work
    if (remote_buf_len > 10) return true;

    char* buf2 = malloc(11);
    buf2[10] = '\0';
    memcpy(buf2, remote_buf, 10);
    *remote_last_update = atol(buf2);

    ESP_LOGI(TAG, "local=%lu, remote=%lu", local_last_update, *remote_last_update);
    ESP_LOGI(TAG, "local=%s, remote=%s", buf, buf2);

    return *remote_last_update > local_last_update;
}

bool load_file(const char* filename, char** buf, size_t* len) {
    FILE* fd = fopen(filename, "r");
    if (fd == NULL) {
        ESP_LOGE(TAG, "Unable to open file: %s", filename);
        return false;
    }

    /* Go to the end of the file. */
    if (fseek(fd, 0L, SEEK_END) == 0) {
        /* Get the size of the file. */
        *len = ftell(fd);

        if (*buf != NULL) {
            free(*buf);
        }

        /* Allocate our buffer to that size. */
        *buf = malloc(*len);

        /* Go back to the start of the file. */
        if (fseek(fd, 0L, SEEK_SET) != 0) {
            free(*buf);
            *len = 0;
            ESP_LOGE(TAG, "Failed to seek to start");
            return false;
        }

        /* Read the entire file into memory. */
        fread(*buf, 1, *len, fd);
        int err = ferror(fd);
        if (err != 0) {
            free(*buf);
            *len = 0;
            ESP_LOGE(TAG, "Failed to read file: %d", err);
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to seek to end");
        return false;
    }
    fclose(fd);
    return true;
}

bool test_load_data() {
    if (!load_file(day1_path_tmp, &data_day1, &size_day1)) return false;

    if (!load_file(day2_path_tmp, &data_day2, &size_day2)) return false;

    json_day1 = cJSON_ParseWithLength(data_day1, size_day1);
    if (json_day1 == NULL) {
        return false;
    }

    json_day2 = cJSON_ParseWithLength(data_day2, size_day2);
    if (json_day2 == NULL) {
        return false;
    }

    return true;
}

bool load_data() {
    if (!load_file(day1_path, &data_day1, &size_day1)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", day1_path);
        render_message("Failed to read agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    if (!load_file(day2_path, &data_day2, &size_day2)) {
        ESP_LOGE(TAG, "Failed to read agenda file: %s", day2_path);
        render_message("Failed to read agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    json_day1 = cJSON_ParseWithLength(data_day1, size_day1);
    if (json_day1 == NULL) {
        ESP_LOGE(TAG, "Failed to parse agenda file: %s", day1_path);
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    json_day2 = cJSON_ParseWithLength(data_day2, size_day2);
    if (json_day2 == NULL) {
        ESP_LOGE(TAG, "Failed to parse agenda file: %s", day2_path);
        render_message("Failed to parse agenda. 🅰 to retry.");
        display_flush();
        return false;
    }

    return true;
}

bool rename_or_replace(const char* old, const char* new) {
    if (access(new, F_OK) == 0) {
        ESP_LOGD(TAG, "Destination file exists, deleting...");
        // File exists, try to delete
        if (remove(new) != 0) {
            ESP_LOGD(TAG, "Destination file could not be deleted");
            // Deleting failed
            return false;
        }
    }

    return rename(old, new) != 0;
}

bool update_agenda(xQueueHandle button_queue, bool force) {
    render_message("Updating agenda...");
    display_flush();

    // Ensure directory exists
    if (!create_dir("/internal/apps/agenda")) {
        ESP_LOGE(TAG, "Failed to create directory in internal storage");
        render_message("Failed to create data dir");
        display_flush();
        return false;
    }

    if (!wifi_connect_to_stored()) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        render_message("Failed to connect to WiFi");
        display_flush();
        return false;
    }

    unsigned long last_update;
    if (!need_update(&last_update) && !force) {
        ESP_LOGI(TAG, "No update needed");
        wifi_disconnect_and_disable();
        return false;
    }

    ESP_LOGI(TAG, "Updating agenda");

    if (!download_file(day1_url, day1_path_tmp)) {
        ESP_LOGE(TAG, "Failed to download %s to %s", day1_url, day1_path);
        render_message("Failed to download file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return false;
    }

    if (!download_file(day2_url, day2_path_tmp)) {
        ESP_LOGE(TAG, "Failed to download %s to %s", day2_url, day2_path);
        render_message("Failed to download file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return false;
    }

    if (!test_load_data()) {
        ESP_LOGE(TAG, "Failed to load updated data, not replacing old");
        return false;
    }



    if (rename_or_replace(day1_path_tmp,day1_path)) {
        ESP_LOGE(TAG, "Failed to rename %s to %s", day1_path_tmp, day1_path);
        render_message("Failed to store file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return true;
    }

    if (rename_or_replace(day2_path_tmp, day2_path)) {
        ESP_LOGE(TAG, "Failed to rename %s to %s", day2_path_tmp, day2_path);
        render_message("Failed to store file");
        display_flush();
        wifi_disconnect_and_disable();
        if (button_queue != NULL) wait_for_button();
        return true;
    }

    // Remember that we updated
    FILE* last_update_fd = fopen(last_update_path, "w");
    if (last_update_fd == NULL) {
        ESP_LOGE(TAG, "Unable to persist last update timestamp");
        wifi_disconnect_and_disable();
        return true;
    }

    char str[11];
    sprintf(str, "%lu", last_update);
    fwrite(str, 1, 10, last_update_fd);
    fclose(last_update_fd);
    wifi_disconnect_and_disable();
    return true;
}

cJSON* get_current_day() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year != (2023 - 1900) || timeinfo.tm_mon != 5) {
        return NULL;
    }

    if (timeinfo.tm_mday == 28) return json_day1;
    if (timeinfo.tm_mday == 29) return json_day2;
    // TODO: remove after testing
    return json_day1;
//    return NULL;
}

bool try_update_or_load(xQueueHandle button_queue, bool first_attempt) {
    if (update_agenda(button_queue, !first_attempt)) {
        return true;
    }
    if (load_data()) {
        return true;
    }

    return false;
}

void menu_agenda(xQueueHandle button_queue) {
    pax_buf_t* pax_buffer = get_pax_buffer();

    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFF131313);

    bool first_attempt = true;
    do {
        // Try to update and load data
        if (try_update_or_load(button_queue, first_attempt)) break;
        // Only force download after the first attempt failed
        first_attempt = false;
        // If there was an error the user is asked if the processes should be retried
        clear_keyboard_queue();
        if (!wait_for_button()) {
            return;
        }
    } while (1);

    menu_t*    menu       = menu_alloc("Troopers 2023 - Agenda", 34, 18);

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

    menu_set_icon(menu, &icon_agenda);
    if (ntp_synced) {
        menu_insert_item_icon(menu, "Next up", NULL, (void*) ACTION_NEXT_UP, -1, &icon_clock);
    }
    menu_insert_item_icon(menu, "Wednesday", NULL, (void*) ACTION_WEDNESDAY, -1, &icon_agenda);
    menu_insert_item_icon(menu, "Thursday", NULL, (void*) ACTION_THURSDAY, -1, &icon_agenda);

    bool                render = true;
    menu_agenda_action_t action = ACTION_NONE;

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
                        action = (menu_agenda_action_t) menu_get_callback_args(menu, menu_get_position(menu));
                        break;
                    default:
                        break;
                }
            }
        }

        if (action != ACTION_NONE) {
            if (action == ACTION_NEXT_UP) {
                details_upcoming(pax_buffer, get_current_day(), &icon_clock);
            } else if (action == ACTION_WEDNESDAY) {
                details_day(pax_buffer, button_queue, json_day1, &icon_agenda);
            } else if (action == ACTION_THURSDAY) {
                details_day(pax_buffer, button_queue, json_day2, &icon_agenda);
            }
            action      = ACTION_NONE;
            render      = true;
            full_redraw = true;
        }
    }

    menu_free(menu);
    pax_buf_destroy(&icon_agenda);
    pax_buf_destroy(&icon_clock);
}
