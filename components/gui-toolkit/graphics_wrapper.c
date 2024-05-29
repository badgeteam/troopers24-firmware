#include "graphics_wrapper.h"

#include <string.h>

#include "esp_log.h"
#include "hardware.h"
#include "pax_keyboard.h"

static const char* TAG = "graphic_wrapper";

void render_outline(float position_x, float position_y, float width, float height, pax_col_t border_color, pax_col_t background_color) {
    pax_buf_t* pax_buffer = get_pax_buffer();
    pax_simple_rect(pax_buffer, background_color, position_x, position_y, width, height);
    pax_outline_rect(pax_buffer, border_color, position_x, position_y, width, height);
}

void render_message(char* message) {
    pax_buf_t* pax_buffer = get_pax_buffer();
    const pax_font_t* font    = pax_font_saira_regular;
    pax_vec1_t        size    = pax_text_size(font, 18, message);
    float             margin  = 4;
    float             width   = size.x + (margin * 2);
    float             posX    = (pax_buffer->width - width) / 2;
    float             height  = size.y + (margin * 2);
    float             posY    = (pax_buffer->height - height) / 2;
    pax_col_t         fgColor = 0xFF131313;
    pax_col_t         bgColor = 0xFFFFFFFF;
    pax_simple_rect(pax_buffer, bgColor, posX, posY, width, height);
    pax_outline_rect(pax_buffer, fgColor, posX, posY, width, height);
    pax_clip(pax_buffer, posX + 1, posY + 1, width - 2, height - 2);
    pax_center_text(pax_buffer, fgColor, font, 18, pax_buffer->width / 2, ((pax_buffer->height - height) / 2) + margin, message);
    pax_noclip(pax_buffer);
}


bool keyboard(xQueueHandle buttonQueue, float aPosX, float aPosY, float aWidth, float aHeight, const char* aTitle,
              const char* aHint, char* aOutput, size_t aOutputSize) {
    pax_buf_t* pax_buffer = get_pax_buffer();
    const pax_font_t* font     = pax_font_saira_regular;
    bool              accepted = false;
    pkb_ctx_t         kb_ctx;
    pkb_init(pax_buffer, &kb_ctx, 1024);
    pkb_set_content(&kb_ctx, aOutput);
    kb_ctx.kb_font   = font;
    kb_ctx.text_font = font;

    pax_col_t bgColor      = 0xFFFFFFFF;
    pax_col_t shadowColor  = 0xFFC0C3C8;
    pax_col_t borderColor  = 0xFF0000AA;
    pax_col_t titleBgColor = 0xFF080764;
    pax_col_t titleColor   = 0xFFFFFFFF;
    pax_col_t selColor     = 0xff007fff;

    kb_ctx.text_col     = borderColor;
    kb_ctx.sel_text_col = bgColor;
    kb_ctx.sel_col      = selColor;
    kb_ctx.bg_col       = bgColor;

    kb_ctx.kb_font_size = 18;

    float titleHeight = 20;
    float hintHeight  = 14;

    pax_noclip(pax_buffer);
    pax_simple_rect(pax_buffer, shadowColor, aPosX + 5, aPosY + 5, aWidth, aHeight);
    pax_simple_rect(pax_buffer, bgColor, aPosX, aPosY, aWidth, aHeight);
    pax_outline_rect(pax_buffer, borderColor, aPosX, aPosY, aWidth, aHeight);
    pax_simple_rect(pax_buffer, titleBgColor, aPosX, aPosY, aWidth, titleHeight);
    pax_simple_line(pax_buffer, titleColor, aPosX + 1, aPosY + titleHeight, aPosX + aWidth - 2, aPosY + titleHeight - 1);
    pax_clip(pax_buffer, aPosX + 1, aPosY + 1, aWidth - 2, titleHeight - 2);
    pax_draw_text(pax_buffer, titleColor, font, titleHeight - 2, aPosX + 1, aPosY + 1, aTitle);
    pax_clip(pax_buffer, aPosX + 1, aPosY + aHeight - hintHeight, aWidth - 2, hintHeight);
    pax_draw_text(pax_buffer, borderColor, font, hintHeight - 2, aPosX + 1, aPosY + aHeight - hintHeight, aHint);
    pax_noclip(pax_buffer);

    kb_ctx.x      = aPosX + 1;
    kb_ctx.y      = aPosY + titleHeight + 1;
    kb_ctx.width  = aWidth - 2;
    kb_ctx.height = aHeight - 3 - titleHeight - hintHeight;

    bool running = true;
    while (running) {
        keyboard_input_message_t buttonMessage = {0};
        if (xQueueReceive(buttonQueue, &buttonMessage, portMAX_DELAY) == pdTRUE) {
            uint8_t button = buttonMessage.input;
            bool    value = buttonMessage.state;

            switch (button) {
                case JOYSTICK_DOWN:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_DOWN);
                    } else {
                        pkb_release(&kb_ctx, PKB_DOWN);
                    }
                    break;
                case JOYSTICK_UP:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_UP);
                    } else {
                        pkb_release(&kb_ctx, PKB_UP);
                    }
                    break;
                case JOYSTICK_LEFT:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_LEFT);
                    } else {
                        pkb_release(&kb_ctx, PKB_LEFT);
                    }
                    break;
                case JOYSTICK_RIGHT:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_RIGHT);
                    } else {
                        pkb_release(&kb_ctx, PKB_RIGHT);
                    }
                    break;
                case BUTTON_ACCEPT:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_CHARSELECT);
                    } else {
                        pkb_release(&kb_ctx, PKB_CHARSELECT);
                    }
                    break;
                case BUTTON_BACK:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_DELETE_BEFORE);
                    } else {
                        pkb_release(&kb_ctx, PKB_DELETE_BEFORE);
                    }
                    break;
                case BUTTON_SELECT:
                    if (value) {
                        pkb_press(&kb_ctx, PKB_MODESELECT);
                    } else {
                        pkb_release(&kb_ctx, PKB_MODESELECT);
                    }
                    break;
                case BUTTON_START:
                    if (value) {
                        running = false;
                    }
                    break;
                default:

                    break;
            }
        }
        pkb_loop(&kb_ctx);
        if (kb_ctx.dirty) {
            pkb_redraw(pax_buffer, &kb_ctx);
            display_flush();
        }
        if (kb_ctx.input_accepted) {
            memset(aOutput, 0, aOutputSize);
            strncpy(aOutput, kb_ctx.content, aOutputSize - 1);
            running  = false;
            accepted = true;
        }
    }
    pkb_destroy(&kb_ctx);
    return accepted;
}
