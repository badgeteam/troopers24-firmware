/*
	MIT License

	Copyright (c) 2023 Renze Nicolai
	Copyright (c) 2022 Julian Scheffers

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include <physical_keyboard.h>
#include <esp_timer.h>
#include <string.h>
#include <malloc.h>

/* ==== Miscellaneous ==== */

// Initialise the context with default settings.
void pkb_init(pax_buf_t *buf, pkb_ctx_t *ctx, size_t buffer_cap) {
	// Allocate a bufffer.
	char *buffer = malloc(buffer_cap);
	memset(buffer, 0, buffer_cap);
	
	// Some defaults.
	*ctx = (pkb_ctx_t) {
		// Position on screen of the keyboard.
		.x              = 0,
		.y              = 0,
		// Maximum size of the keyboard.
		.width          = buf->width,
		.height         = buf->height,
		
		// Content of the keyboard.
		.content        = buffer,
		// Size in bytes of capacity of the content buffer.
		.content_cap    = buffer_cap,
		
		// Starting position of the selection in the text box.
		.selection      = 0,
		// Cursor position of the text box.
		.cursor         = 0,
		
		// Cursor position of the keyboard.
		.key_x          = 3,
		.key_y          = 1,
		// The currently held input.
		.held           = PKB_NO_INPUT,
		// The time that holding the input started.
		.hold_start     = 0,
		// The last time pkb_press was called.
		.last_press     = 0,
		
		// Whether the keyboard is multi-line.
		.multiline      = false,
		// Whether the keyboard is in insert mode.
		.insert         = false,
		// The board that is currently selected.
		.board_sel      = PKB_LOWERCASE,
		
		// The font to use for the keyboard.
		.kb_font        = PAX_FONT_DEFAULT,
		// The font size to use for the keyboard.
		.kb_font_size   = 27,
		// The font to use for the text.
		.text_font      = PAX_FONT_DEFAULT,
		// The font size to use for the text.
		.text_font_size = 18,
		// The text color to use.
		.text_col       = 0xffffffff,
		// The text color to use when a character is being held down.
		.sel_text_col   = 0xff000000,
		// The selection color to use.
		.sel_col        = 0xff007fff,
		// The background color to use.
		.bg_col         = 0xff000000,
		
		// Whether something has changed since last draw.
		.dirty          = true,
		// Whether the text has changed since last draw.
		.text_dirty     = true,
		// Whether the keyboard has changed since last draw.
		.kb_dirty       = true,
		// Whether just the selected character has changed since last draw.
		.sel_dirty      = true,
		// Previous cursor position of the keyboard.
		// Used for sel_dirty.
		.last_key_x     = 3,
		.last_key_y     = 1,
		
		// Indicates that the input has been accepted.
		.input_accepted = false,
	};
	
	// TODO: Pick fancier text sizes.
}

// Free any memory associated with the context.
void pkb_destroy(pkb_ctx_t *ctx) {
	free(ctx->content);
	ctx->content = NULL;
	ctx->content_cap = 0;
}


// Replaces the text in the keyboard with the given text.
// Makes a copy of the given text.
void pkb_set_content(pkb_ctx_t *ctx, const char *content) {
	// Replace the content.
	strncpy(ctx->content, content, ctx->content_cap - 1);
	ctx->content[ctx->content_cap-1] = 0;
	ctx->cursor = strlen(content);
}

// Draw just the board part.
static void pkb_render_keyb(pax_buf_t *buf, pkb_ctx_t *ctx, bool do_bg) {
	// Draw background.
	if (do_bg) {
		pax_draw_rect(buf, ctx->bg_col, ctx->x, ctx->y + ctx->height - ctx->kb_font_size*4, ctx->width, ctx->kb_font_size*4);
	}
}

// Draw just the text part.
static void pkb_render_text(pax_buf_t *buf, pkb_ctx_t *ctx, bool do_bg) {
	// Draw background.
	if (do_bg) {
		pax_draw_rect(buf, ctx->bg_col, ctx->x-0.01, ctx->y-0.01, ctx->width+0.02, ctx->height - ctx->kb_font_size*4 + 0.02);
	}
	if (ctx->key_y == -1) {
		// Outline us.
		pax_outline_rect(buf, ctx->sel_col, ctx->x, ctx->y, ctx->width - 1, ctx->height - ctx->kb_font_size*4 - 1);
	}
	
	// Some setup.
	float x = ctx->x + 2;
	float y = ctx->y + 2;
	char tmp[2] = {0, 0};
	
	// Draw everything.
	for (int i = 0; i < strlen(ctx->content); i++) {
		if (ctx->cursor == i) {
			// The cursor in between the input.
			pax_draw_line(buf, ctx->sel_col, x, y, x, y + ctx->text_font_size - 1);
		}
		
		// The character of the input.
		tmp[0] = ctx->content[i];
		pax_vec1_t dims = pax_text_size(ctx->text_font, ctx->text_font_size, tmp);
		
		if (x + dims.x > ctx->width - 2) {
			// Word wrap.
			x  = ctx->x + 2;
			y += ctx->text_font_size;
		}
		pax_draw_text(buf, ctx->text_col, ctx->text_font, ctx->text_font_size, x, y, tmp);
		x += dims.x;
	}
	if (ctx->cursor == strlen(ctx->content)) {
		// The cursor after the input.
		pax_draw_line(buf, ctx->sel_col, x, y, x, y + ctx->text_font_size - 1);
	}
}

// Draw one specific key.
static void pkb_render_key(pax_buf_t *buf, pkb_ctx_t *ctx, int key_x, int key_y) {
	if (key_y == -1) {
		// If key_y is -1, the text box is selected to render.
		pkb_render_text(buf, ctx, true);
		return;
	}
}

// Redraw the complete on-screen keyboard.
void pkb_render(pax_buf_t *buf, pkb_ctx_t *ctx) {
	if (matrix_2d_is_identity(buf->stack_2d.value)
		&& ctx->x == 0 && ctx->y == 0
		&& ctx->width  == buf->width
		&& ctx->height == buf->height) {
		// We can just fill the entire screen.
		pax_background(buf, ctx->bg_col);
	} else {
		// We'll need to fill a rectangle.
		pax_draw_rect(
			buf, ctx->bg_col,
			ctx->x, ctx->y,
			ctx->width, ctx->height
		);
	}
	
	// Draw the board.
	pkb_render_keyb(buf, ctx, false);
	// Time to draw some text.
	pkb_render_text(buf, ctx, false);
	
	// Mark as not dirty.
	ctx->dirty      = false;
	ctx->kb_dirty   = false;
	ctx->sel_dirty  = false;
	ctx->text_dirty = false;
	ctx->last_key_x = ctx->key_x;
	ctx->last_key_y = ctx->key_y;
}

// Redraw only the changed parts of the on-screen keyboard.
void pkb_redraw(pax_buf_t *buf, pkb_ctx_t *ctx) {
	if (ctx->text_dirty) {
		pkb_render_text(buf, ctx, true);
	}
	if (ctx->kb_dirty) {
		pkb_render_keyb(buf, ctx, true);
	} else if (ctx->sel_dirty) {
		pkb_render_key(buf, ctx, ctx->last_key_x, ctx->last_key_y);
		pkb_render_key(buf, ctx, ctx->key_x,      ctx->key_y);
	}
	
	// Mark as not dirty.
	ctx->dirty      = false;
	ctx->kb_dirty   = false;
	ctx->sel_dirty  = false;
	ctx->text_dirty = false;
	ctx->last_key_x = ctx->key_x;
	ctx->last_key_y = ctx->key_y;
}

/* ==== Text editing ==== */

// Handling of delete or backspace.
static void pkb_delete(pkb_ctx_t *ctx, bool is_backspace) {
	size_t oldlen = strlen(ctx->content);
	if (!is_backspace && ctx->cursor == oldlen) {
		// No forward deleting at the end of the line.
		return;
	} else if (is_backspace && ctx->cursor == 0) {
		// No backward deleting at the start of the line.
		return;
	} else if (!is_backspace) {
		// Advanced backspace.
		ctx->cursor ++;
	}
	
	// Copy back everything including null terminator.
	ctx->cursor --;
	for (int i = ctx->cursor; i < oldlen; i++) {
		ctx->content[i] = ctx->content[i+1];
	}
	
	ctx->text_dirty = true;
}

// Handling of normal input.
static void pkb_append(pkb_ctx_t *ctx, char value) {
	size_t oldlen = strlen(ctx->content);
	if (oldlen + 2 >= ctx->content_cap) {
		// That's too big.
		return;
	}
	
	// Copy over the remainder of the buffer.
	// If there's no text this still copies the null terminator.
	for (int i = oldlen; i >= ctx->cursor; i --) {
		ctx->content[i + 1] = ctx->content[i];
	}
	
	// And finally insert at the character.
	ctx->content[ctx->cursor] = value;
	ctx->cursor ++;
	ctx->text_dirty = true;
}

/* ==== Input handling ==== */

// The loop that allows input repeating.
void pkb_loop(pkb_ctx_t *ctx) {
	int64_t now = esp_timer_get_time();
	if (!ctx->held) return;
	bool is_dir = (ctx->held >= PKB_UP) && (ctx->held <= PKB_RIGHT);
	
	if ((ctx->hold_start + 1000000 < now) || (is_dir && ctx->hold_start + 250000 < now)) {
		// 8 repeats per second.
		if (ctx->last_press + 125000 < now) {
			pkb_press(ctx, ctx->held);
		}
	}
}

// A pressing of the input.
void pkb_press(pkb_ctx_t *ctx, pkb_input_t input) {
/*	const char **board = boards[ctx->board_sel];
	ctx->last_press = esp_timer_get_time();
	switch (input) {
		case PKB_CHARSELECT:
			ctx->sel_dirty |= ctx->held != PKB_CHARSELECT;
			if (ctx->key_y == 3) {
				switch (ctx->key_x) {
					case 0:
						// Board selector.
						if (ctx->board_sel == PKB_NUMBERS) ctx->board_sel = PKB_SYMBOLS;
						else if (ctx->board_sel == PKB_SYMBOLS) ctx->board_sel = PKB_LOWERCASE;
						else ctx->board_sel = PKB_NUMBERS;
						ctx->kb_dirty = true;
						break;
					case 1:
						// Magic.
						pkb_append(ctx, *board[4]);
						break;
					default:
						// Spacebar.
						pkb_append(ctx, ' ');
						break;
					case 7:
						// mAGIC.
						pkb_append(ctx, *board[5]);
						break;
					case 8:
						// Enter idk.
						ctx->input_accepted = true;
						break;
				}
			} else if (ctx->key_y == 2) {
				if (ctx->key_x == 0) {
					// cAPS LOCK KEY.
					if (ctx->held == PKB_CHARSELECT) ctx->held = PKB_NO_INPUT;
					ctx->board_sel ^= 1;
					ctx->kb_dirty   = true;
				} else if (ctx->key_x == strlen(board[2])-1) {
					// Backspace.
					pkb_delete(ctx, true);
				} else {
					// nORMAL CHAR.
					pkb_append(ctx, board[2][ctx->key_x]);
				}
			} else if (ctx->key_y != -1) {
				// Normal char.
				pkb_append(ctx, board[ctx->key_y][ctx->key_x]);
			}
			break;
			
			// Shift key, the pressening.
		case PKB_SHIFT:
			ctx->board_sel |= 1;
			ctx->kb_dirty = true;
			break;
			
			// Next keyboard.
		case PKB_MODESELECT:
			ctx->board_sel ++;
			ctx->board_sel %= 4;
			rowlen = strlen(board[ctx->key_y]);
			if (ctx->key_x >= rowlen) ctx->key_x = rowlen - 1;
			ctx->kb_dirty = true;
			break;
			
			// Backspace.
		case PKB_DELETE_BEFORE:
			pkb_delete(ctx, true);
			break;
			
			// Delete.
		case PKB_DELETE_AFTER:
			pkb_delete(ctx, false);
			break;
		default:
			break;
	}
	if (input != PKB_SHIFT && input != ctx->held) {
		ctx->held = input;
		ctx->hold_start = esp_timer_get_time();
	}
	ctx->dirty = true;*/
}

// A relealing of the input.
void pkb_release(pkb_ctx_t *ctx, pkb_input_t input) {
	switch (input) {
			// Shift key, the releasening.
		case PKB_SHIFT:
			ctx->dirty = true;
			ctx->board_sel &= ~1;
			ctx->kb_dirty = true;
			break;
			
			// Unpress them char.
		case PKB_CHARSELECT:
			ctx->sel_dirty = true;
			break;
			
		default:
			break;
	}
	if (ctx->held == input) {
		ctx->held = PKB_NO_INPUT;
		ctx->dirty = true;
	}
}
