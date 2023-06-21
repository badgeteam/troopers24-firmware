#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#include "agenda.h"
#include "app_update.h"
#include "bootscreen.h"
#include "dev.h"
#include "hardware.h"
#include "hatchery.h"
#include "id.h"
#include "launcher.h"
#include "math.h"
#include "menu.h"
#include "nametag.h"
#include "pax_codecs.h"
#include "pax_gfx.h"
#include "sao.h"
#include "sao_eeprom.h"
#include "settings.h"
#include "wifi_ota.h"

extern const uint8_t home_png_start[] asm("_binary_home_png_start");
extern const uint8_t home_png_end[] asm("_binary_home_png_end");

extern const uint8_t tag_png_start[] asm("_binary_tag_png_start");
extern const uint8_t tag_png_end[] asm("_binary_tag_png_end");

extern const uint8_t apps_png_start[] asm("_binary_apps_png_start");
extern const uint8_t apps_png_end[] asm("_binary_apps_png_end");

extern const uint8_t hatchery_png_start[] asm("_binary_hatchery_png_start");
extern const uint8_t hatchery_png_end[] asm("_binary_hatchery_png_end");

extern const uint8_t dev_png_start[] asm("_binary_dev_png_start");
extern const uint8_t dev_png_end[] asm("_binary_dev_png_end");

extern const uint8_t settings_png_start[] asm("_binary_settings_png_start");
extern const uint8_t settings_png_end[] asm("_binary_settings_png_end");

extern const uint8_t update_png_start[] asm("_binary_update_png_start");
extern const uint8_t update_png_end[] asm("_binary_update_png_end");

//extern const uint8_t sao_png_start[] asm("_binary_sao_png_start");
//extern const uint8_t sao_png_end[] asm("_binary_sao_png_end");

extern const uint8_t agenda_png_start[] asm("_binary_calendar_png_start");
extern const uint8_t agenda_png_end[] asm("_binary_calendar_png_end");

extern const uint8_t id_png_start[] asm("_binary_sao_png_start");
extern const uint8_t id_png_end[] asm("_binary_sao_png_end");

typedef enum action {
    ACTION_NONE,
    ACTION_APPS,
    ACTION_LAUNCHER,
    ACTION_HATCHERY,
    ACTION_NAMETAG,
    ACTION_DEV,
    ACTION_SETTINGS,
    ACTION_UPDATE,
    ACTION_OTA,
    ACTION_SAO,
    ACTION_ID,
    ACTION_AGENDA
} menu_start_action_t;

void render_background(pax_buf_t* pax_buffer, const char* text) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xFF1E1E1E);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅰 accept");
    pax_vec1_t version_size = pax_text_size(font, 18, text);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 320 - 5 - version_size.x, 240 - 18, text);
}

void menu_start(xQueueHandle button_queue, const char* version, bool wakeup_deepsleep) {
    if (wakeup_deepsleep) {
        show_nametag(button_queue);
    }

    pax_buf_t* pax_buffer = get_pax_buffer();
    menu_t*    menu       = menu_alloc("Troopers 2023", 34, 18);

    menu->fgColor           = 0xFFF1AA13;
    menu->bgColor           = 0xFF131313;
    menu->bgTextColor       = 0xFF000000;
    menu->selectedItemColor = 0xFFF1AA13;
    menu->borderColor       = 0xFF1E1E1E;
    menu->titleColor        = 0xFFF1AA13;
    menu->titleBgColor      = 0xFF1E1E1E;
    menu->scrollbarBgColor  = 0xFFCCCCCC;
    menu->scrollbarFgColor  = 0xFF555555;

    pax_buf_t icon_home;
    pax_decode_png_buf(&icon_home, (void*) home_png_start, home_png_end - home_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_tag;
    pax_decode_png_buf(&icon_tag, (void*) tag_png_start, tag_png_end - tag_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_apps;
    pax_decode_png_buf(&icon_apps, (void*) apps_png_start, apps_png_end - apps_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_hatchery;
    pax_decode_png_buf(&icon_hatchery, (void*) hatchery_png_start, hatchery_png_end - hatchery_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_dev;
    pax_decode_png_buf(&icon_dev, (void*) dev_png_start, dev_png_end - dev_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_settings;
    pax_decode_png_buf(&icon_settings, (void*) settings_png_start, settings_png_end - settings_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_update;
    pax_decode_png_buf(&icon_update, (void*) update_png_start, update_png_end - update_png_start, PAX_BUF_32_8888ARGB, 0);
//    pax_buf_t icon_hardware;
//    pax_decode_png_buf(&icon_hardware, (void*) sao_png_start, sao_png_end - sao_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_id;
    pax_decode_png_buf(&icon_id, (void*) id_png_start, id_png_end - id_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_agenda;
    pax_decode_png_buf(&icon_agenda, (void*) agenda_png_start, agenda_png_end - agenda_png_start, PAX_BUF_32_8888ARGB, 0);

    menu_set_icon(menu, &icon_home);
    menu_insert_item_icon(menu, "Name tag", NULL, (void*) ACTION_NAMETAG, -1, &icon_tag);
    menu_insert_item_icon(menu, "ID", NULL, (void*) ACTION_ID, -1, &icon_tag);
    menu_insert_item_icon(menu, "Agenda", NULL, (void*) ACTION_AGENDA, -1, &icon_agenda);
    menu_insert_item_icon(menu, "Apps", NULL, (void*) ACTION_LAUNCHER, -1, &icon_apps);
    menu_insert_item_icon(menu, "Hatchery", NULL, (void*) ACTION_HATCHERY, -1, &icon_hatchery);
    menu_insert_item_icon(menu, "Tools", NULL, (void*) ACTION_DEV, -1, &icon_dev);
//    menu_insert_item_icon(menu, "SAO", NULL, (void*) ACTION_SAO, -1, &icon_hardware);
    menu_insert_item_icon(menu, "Settings", NULL, (void*) ACTION_SETTINGS, -1, &icon_settings);
    menu_insert_item_icon(menu, "App update", NULL, (void*) ACTION_UPDATE, -1, &icon_update);
    menu_insert_item_icon(menu, "OS update", NULL, (void*) ACTION_OTA, -1, &icon_update);
    bool                render = true;
    menu_start_action_t action = ACTION_NONE;

    bool full_redraw = true;
    while (1) {
        if (render) {
            if (full_redraw) {
                char textBuffer[64];
                snprintf(textBuffer, sizeof(textBuffer), "v%s", version);
                render_background(pax_buffer, textBuffer);
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
                    case BUTTON_ACCEPT:
                    case BUTTON_SELECT:
                        action = (menu_start_action_t) menu_get_callback_args(menu, menu_get_position(menu));
                        break;
                    default:
                        break;
                }
            }
        }

        if (action != ACTION_NONE) {
            if (action == ACTION_HATCHERY) {
                menu_hatchery(button_queue);
            } else if (action == ACTION_NAMETAG) {
                show_nametag(button_queue);
            } else if (action == ACTION_SETTINGS) {
                menu_settings(button_queue);
            } else if (action == ACTION_DEV) {
                menu_dev(button_queue);
            } else if (action == ACTION_LAUNCHER) {
                menu_launcher(button_queue);
            } else if (action == ACTION_UPDATE) {
                update_apps(button_queue);
            } else if (action == ACTION_OTA) {
                ota_update(false);
            } else if (action == ACTION_SAO) {
                menu_sao(button_queue);
            } else if (action == ACTION_ID) {
                menu_id(button_queue);
            } else if (action == ACTION_AGENDA) {
                while (menu_agenda(button_queue)) {}
            }
            action      = ACTION_NONE;
            render      = true;
            full_redraw = true;
        }
    }

    menu_free(menu);
    pax_buf_destroy(&icon_home);
    pax_buf_destroy(&icon_tag);
    pax_buf_destroy(&icon_apps);
    pax_buf_destroy(&icon_hatchery);
    pax_buf_destroy(&icon_dev);
    pax_buf_destroy(&icon_settings);
    pax_buf_destroy(&icon_update);
    pax_buf_destroy(&icon_id);
}
