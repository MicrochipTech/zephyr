/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/input/input.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

/*
 * Keyboard matrix key name map [col][row].
 *
 * Based on a standard 18-column x 8-row laptop keyboard matrix.
 * Adjust this table to match your physical keyboard wiring.
 * NULL means the position is not wired / reserved.
 *
 * The driver reports events as:
 *   INPUT_EV_ABS / INPUT_ABS_X  -> column
 *   INPUT_EV_ABS / INPUT_ABS_Y  -> row
 *   INPUT_EV_KEY / INPUT_BTN_TOUCH -> 1=pressed, 0=released
 */

#define MAX_MATRIX_KEY_COLS 18
#define MAX_MATRIX_KEY_ROWS 8

/* clang-format off */
/*
 * Fujitsu N860-7401-TOO1 layout (16 cols used out of 18).
 * Columns 16-17 are spare/unused on this keyboard.
 *
 *         Row7    Row6    Row5    Row4    Row3    Row2    Row1    Row0
 */
static const char * const keymap[MAX_MATRIX_KEY_COLS][MAX_MATRIX_KEY_ROWS] = {
/* Col 0 */  { NULL,  "`~",   "F1",   "Tab",  "1!",   NULL,   "CapsLk", NULL   },
/* Col 1 */  { "F5",  "F6",   "Esc",  NULL,   "F7",   "Q",    "W",      NULL   },
/* Col 2 */  { "F2",  NULL,   "F3",   "F4",   "2@",   NULL,   "F8",     NULL   },
/* Col 3 */  { "V",   "F",    "C",    "4$",   "3#",   "E",    "R",      NULL   },
/* Col 4 */  { "G",   "H",    "T",    "B",    "5%",   "6^",   "Y",      "N"    },
/* Col 5 */  { "U",   "S",    "D",    "J",    "7&",   "A",    "M",      "Space"},
/* Col 6 */  { "K",   "X",    "Z",    NULL,   "8*",   ",<",   "I",      "F9"   },
/* Col 7 */  { "O",   "Left", NULL,   "L",    "9(",   "]}",   "=+",     NULL   },
/* Col 8 */  { "'\"", NULL,   NULL,   ";:",   "[{",   "/?",   "0)",     "-_"   },
/* Col 9 */  { NULL,  NULL,   NULL,   NULL,   "P",    "NumLk","Pause",  "F10"  },
/* Col 10 */ { "Down","WinApp",NULL,  NULL,   ".>",   "F11",  "\\|",    "BkSp" },
/* Col 11 */ { "Right","Up",  NULL,   NULL,   "F12",  "Ins",  "Del",    "Enter"},
/* Col 12 */ { "Fn",  "L-Win",NULL,   NULL,   NULL,   NULL,   "R-Win",  NULL   },
/* Col 13 */ { NULL,  NULL,   NULL,   "LShft","RShft",NULL,   NULL,     NULL   },
/* Col 14 */ { "RAlt","LAlt", NULL,   NULL,   NULL,   NULL,   NULL,     NULL   },
/* Col 15 */ { NULL,  NULL,   "LCtrl",NULL,   NULL,   NULL,   NULL,     "RCtrl"},
/* Col 16 */ { NULL,  NULL,   NULL,   NULL,   NULL,   NULL,   NULL,     NULL   },
/* Col 17 */ { NULL,  NULL,   NULL,   NULL,   NULL,   NULL,   NULL,     NULL   },
};
/* clang-format on */

/* Track current row and column from ABS events before the BTN_TOUCH fires */
static int cur_col;
static int cur_row;

static void kbd_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case INPUT_EV_ABS:
		if (evt->code == INPUT_ABS_X) {
			cur_col = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			cur_row = evt->value;
		}
		break;

	case INPUT_EV_KEY:
		if (evt->code == INPUT_BTN_TOUCH) {
			const char *key_name = NULL;

			if (cur_col < MAX_MATRIX_KEY_COLS &&
			    cur_row < MAX_MATRIX_KEY_ROWS) {
				key_name = keymap[cur_col][cur_row];
			}

			if (key_name != NULL) {
				LOG_INF("[%s] %s (col=%d row=%d)",
					key_name,
					evt->value ? "pressed" : "released",
					cur_col, cur_row);
			} else {
				LOG_INF("[???] %s (col=%d row=%d)",
					evt->value ? "pressed" : "released",
					cur_col, cur_row);
			}
		}
		break;

	default:
		break;
	}
}

INPUT_CALLBACK_DEFINE(NULL, kbd_input_cb, NULL);

int main(void)
{
	printk("Keyboard matrix input sample\n");
	printk("Waiting for key events...\n");
	return 0;
}
