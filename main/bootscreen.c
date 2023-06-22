#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#include "hardware.h"
#include "pax_codecs.h"
#include "pax_gfx.h"

extern const uint8_t troopers1_png_start[] asm("_binary_troopers1_png_start");
extern const uint8_t troopers1_png_end[] asm("_binary_troopers1_png_end");
extern const uint8_t hourglass_png_start[] asm("_binary_hourglass_png_start");
extern const uint8_t hourglass_png_end[] asm("_binary_hourglass_png_end");

extern const uint8_t frame0_png_start[] asm("_binary_boot0_png_start");
extern const uint8_t frame0_png_end[] asm("_binary_boot0_png_end");

extern const uint8_t frame1_png_start[] asm("_binary_boot1_png_start");
extern const uint8_t frame1_png_end[] asm("_binary_boot1_png_end");

extern const uint8_t frame2_png_start[] asm("_binary_boot2_png_start");
extern const uint8_t frame2_png_end[] asm("_binary_boot2_png_end");

extern const uint8_t frame3_png_start[] asm("_binary_boot3_png_start");
extern const uint8_t frame3_png_end[] asm("_binary_boot3_png_end");

extern const uint8_t frame4_png_start[] asm("_binary_boot4_png_start");
extern const uint8_t frame4_png_end[] asm("_binary_boot4_png_end");

extern const uint8_t frame5_png_start[] asm("_binary_boot5_png_start");
extern const uint8_t frame5_png_end[] asm("_binary_boot5_png_end");

extern const uint8_t frame6_png_start[] asm("_binary_boot6_png_start");
extern const uint8_t frame6_png_end[] asm("_binary_boot6_png_end");

void display_frame(const uint8_t start[], const uint8_t end[]) {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFFFFFF);
    pax_insert_png_buf(pax_buffer, start, end - start, 0, 87, 0);
    display_flush();
}

void display_boot_animation() {
    display_frame(frame0_png_start, frame0_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame1_png_start, frame1_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame2_png_start, frame2_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame3_png_start, frame3_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame4_png_start, frame4_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame5_png_start, frame5_png_end);
    vTaskDelay(pdMS_TO_TICKS(200));

    display_frame(frame6_png_start, frame6_png_end);
    vTaskDelay(pdMS_TO_TICKS(800));
}

void display_story_splash() {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font       = pax_font_saira_regular;
    const char* text = "You enter a forest\nclearing, and find\nan old shield\nresting on the\nground.\n\nMaybe this will\nprotect you on the\njourney ahead...";
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0x131313);
    pax_vec1_t size = pax_text_size(font, 22, text);
    pax_draw_text(pax_buffer, 0xFFF1AA13, font, 22, 160 - (size.x/2), 120 - (size.y/2), text);
    display_flush();
    vTaskDelay(pdMS_TO_TICKS(500));
}

void display_boot_screen(const char* text) {
    pax_buf_t*        pax_buffer = get_pax_buffer();
    const pax_font_t* font       = pax_font_saira_regular;
    pax_noclip(pax_buffer);
    pax_background(pax_buffer, 0xFFFFFF);
    pax_insert_png_buf(pax_buffer, troopers1_png_start, troopers1_png_end - troopers1_png_start, 0, 0, 0);
    pax_vec1_t size = pax_text_size(font, 18, text);
    pax_draw_text(pax_buffer, 0xFFFFFFFF, font, 18, 0, 240 - size.y, text);
    display_flush();
}

void display_busy() {
    pax_buf_t* pax_buffer = get_pax_buffer();
    pax_noclip(pax_buffer);
    pax_buf_t icon;
    pax_decode_png_buf(&icon, (void*) hourglass_png_start, hourglass_png_end - hourglass_png_start, PAX_BUF_32_8888ARGB, 0);
    float x = (pax_buffer->width - icon.width) / 2;
    float y = (pax_buffer->height - icon.height) / 2;
    pax_simple_rect(pax_buffer, 0xFFFFFFFF, x - 1, y - 1, icon.width + 2, icon.height + 2);
    pax_outline_rect(pax_buffer, 0xff491d88, x - 1, y - 1, icon.width + 2, icon.height + 2);
    pax_draw_image(pax_buffer, &icon, x, y);
    pax_buf_destroy(&icon);
    display_flush();
}
