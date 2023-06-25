#include "id.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "efuse.h"
#include "hardware.h"
#include "pax_codecs.h"
#include "pax_gfx.h"
#include "system_wrapper.h"

static const char* TAG = "id";

extern const uint8_t shield_png_start[] asm("_binary_id_shield_png_start");
extern const uint8_t shield_png_end[] asm("_binary_id_shield_png_end");

extern const uint8_t ernw_png_start[] asm("_binary_id_ernw_png_start");
extern const uint8_t ernw_png_end[] asm("_binary_id_ernw_png_end");

extern const uint8_t fucss_png_start[] asm("_binary_id_fucss_png_start");
extern const uint8_t fucss_png_end[] asm("_binary_id_fucss_png_end");

extern const uint8_t fishbowl_png_start[] asm("_binary_id_fishbowl_png_start");
extern const uint8_t fishbowl_png_end[] asm("_binary_id_fishbowl_png_end");

extern const uint8_t badgeteam_png_start[] asm("_binary_id_badgeteam_png_start");
extern const uint8_t badgeteam_png_end[] asm("_binary_id_badgeteam_png_end");

void render_icon(pax_buf_t* pax_buffer, int pos, const uint8_t start[], const uint8_t end[]) {
    // Place 4 icons on a horizontal line, each icon is 80x80 pixels
    int x = pos * 80;
    pax_insert_png_buf(pax_buffer, start, end - start, x, 80, 0);
}

void render_icon_id(pax_buf_t* pax_buffer, int pos, int index) {
    switch(index) {
        case 0:
            render_icon(pax_buffer, pos, shield_png_start, shield_png_end);
            break;
        case 1:
            render_icon(pax_buffer, pos, fucss_png_start, fucss_png_end);
            break;
        case 2:
            render_icon(pax_buffer, pos, fishbowl_png_start, fishbowl_png_end);
            break;
        case 3:
            render_icon(pax_buffer, pos, ernw_png_start, ernw_png_end);
            break;
        case 4:
            render_icon(pax_buffer, pos, badgeteam_png_start, badgeteam_png_end);
            break;
    }
}

void render_icons(pax_buf_t* pax_buffer, uint16_t id) {
    // Background
    const pax_font_t* font = pax_font_saira_regular;
    pax_background(pax_buffer, 0xff152139);
    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, 0xff131313, 0, 220, 320, 20);
    pax_draw_text(pax_buffer, 0xffffffff, font, 18, 5, 240 - 18, "🅱 back");

    // Encode number
    int remainder = id;
    int d = remainder / 125;
    remainder -= d * 125;
    int c = remainder / 25;
    remainder -= c * 25;
    int b = remainder / 5;
    remainder -= b * 5;
    int a = remainder % 5;

    // Render the 4 images
    render_icon_id(pax_buffer, 0, d);
    render_icon_id(pax_buffer, 1, c);
    render_icon_id(pax_buffer, 2, b);
    render_icon_id(pax_buffer, 3, a);

    // Flush the display
    display_flush();
}

void menu_id() {
    pax_buf_t* pax_buffer = get_pax_buffer();
    render_icons(pax_buffer, badge_id());
    wait_for_button();
}
