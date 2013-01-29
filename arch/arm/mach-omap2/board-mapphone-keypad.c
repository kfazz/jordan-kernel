/*
 * arch/arm/mach-omap2/board-mapphone-keypad.c
 *
 * Copyright (C) 2009 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <plat/keypad.h>

#include <mach/board-mapphone.h>
#include "mux.h"

#define GPIO_SILENCE_KEY		100
#define GPIO_SLIDER			177

static unsigned int mapphone_col_gpios[] = { 43, 53, 54, 55, 56, 57, 58, 63 };
static unsigned int mapphone_row_gpios[] = { 34, 35, 36, 37, 38, 39, 40, 41 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(mapphone_row_gpios) + (row))

static const unsigned short mapphone_p3_keymap[ARRAY_SIZE(mapphone_col_gpios) *
					     ARRAY_SIZE(mapphone_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_9,
	[KEYMAP_INDEX(0, 1)] = KEY_R,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND, /* n/c dummy for CALLSEND testing*/
	[KEYMAP_INDEX(0, 3)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 4)] = KEY_F4,   /* n/c dummy for CALLEND testing */
	[KEYMAP_INDEX(0, 5)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 6)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 7)] = KEY_D,

	[KEYMAP_INDEX(1, 0)] = KEY_7,
	[KEYMAP_INDEX(1, 1)] = KEY_M,
	[KEYMAP_INDEX(1, 2)] = KEY_L,
	[KEYMAP_INDEX(1, 3)] = KEY_K,
	[KEYMAP_INDEX(1, 4)] = KEY_N,
	[KEYMAP_INDEX(1, 5)] = KEY_C,
	[KEYMAP_INDEX(1, 6)] = KEY_Z,
	[KEYMAP_INDEX(1, 7)] = KEY_RIGHTSHIFT,

	[KEYMAP_INDEX(2, 0)] = KEY_1,
	[KEYMAP_INDEX(2, 1)] = KEY_Y,
	[KEYMAP_INDEX(2, 2)] = KEY_I,
	[KEYMAP_INDEX(2, 3)] = KEY_COMMA,
	[KEYMAP_INDEX(2, 4)] = KEY_LEFTALT,
	[KEYMAP_INDEX(2, 5)] = KEY_DOT,
	[KEYMAP_INDEX(2, 6)] = KEY_G,
	[KEYMAP_INDEX(2, 7)] = KEY_E,

/*	[KEYMAP_INDEX(3, 0)] = KEY_, */
	[KEYMAP_INDEX(3, 1)] = KEY_6,
	[KEYMAP_INDEX(3, 2)] = KEY_3,
	[KEYMAP_INDEX(3, 3)] = KEY_DOWN,
	[KEYMAP_INDEX(3, 4)] = KEY_UP,
	[KEYMAP_INDEX(3, 5)] = KEY_LEFT,
	[KEYMAP_INDEX(3, 6)] = KEY_RIGHT,
	[KEYMAP_INDEX(3, 7)] = KEY_REPLY,	/* d-pad center key */

	[KEYMAP_INDEX(4, 0)] = KEY_5,
	[KEYMAP_INDEX(4, 1)] = KEY_J,
	[KEYMAP_INDEX(4, 2)] = KEY_B,
	[KEYMAP_INDEX(4, 3)] = KEY_CAMERA-1,	/* camera 1 key, steal KEY_HP*/
	[KEYMAP_INDEX(4, 4)] = KEY_T,
	[KEYMAP_INDEX(4, 5)] = KEY_CAMERA,	/* "camera 2" key */
	[KEYMAP_INDEX(4, 6)] = KEY_MENU,
	[KEYMAP_INDEX(4, 7)] = KEY_X,

	[KEYMAP_INDEX(5, 0)] = KEY_8,
	[KEYMAP_INDEX(5, 1)] = KEY_SPACE,
	[KEYMAP_INDEX(5, 2)] = KEY_RIGHTALT,
/*	[KEYMAP_INDEX(5, 3)] = KEY_, */
	[KEYMAP_INDEX(5, 4)] = KEY_SLASH,
	[KEYMAP_INDEX(5, 5)] = KEY_EMAIL,	/* @ */
	[KEYMAP_INDEX(5, 6)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(5, 7)] = KEY_A,

	[KEYMAP_INDEX(6, 0)] = KEY_2,
	[KEYMAP_INDEX(6, 1)] = KEY_0,
	[KEYMAP_INDEX(6, 2)] = KEY_F,
	[KEYMAP_INDEX(6, 3)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(6, 4)] = KEY_ENTER,
	[KEYMAP_INDEX(6, 5)] = KEY_O,
	[KEYMAP_INDEX(6, 6)] = KEY_H,
	[KEYMAP_INDEX(6, 7)] = KEY_Q,

	[KEYMAP_INDEX(7, 0)] = KEY_4,
	[KEYMAP_INDEX(7, 1)] = KEY_V,
	[KEYMAP_INDEX(7, 2)] = KEY_S,
	[KEYMAP_INDEX(7, 3)] = KEY_P,
	[KEYMAP_INDEX(7, 4)] = KEY_QUESTION,
	[KEYMAP_INDEX(7, 5)] = KEY_MUTE,
	[KEYMAP_INDEX(7, 6)] = KEY_U,
	[KEYMAP_INDEX(7, 7)] = KEY_W,
};

static struct gpio_event_matrix_info mapphone_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = mapphone_p3_keymap,
	.output_gpios = mapphone_col_gpios,
	.input_gpios = mapphone_row_gpios,
	.noutputs = ARRAY_SIZE(mapphone_col_gpios),
	.ninputs = ARRAY_SIZE(mapphone_row_gpios),
	.settle_time = 40 * NSEC_PER_USEC,
	.poll_time = 20 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry mapphone_keypad_switch_map[] = {
	{ GPIO_SILENCE_KEY,	SW_HEADPHONE_INSERT },
	{ GPIO_SLIDER,		SW_LID}
};

static struct gpio_event_input_info mapphone_keypad_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = mapphone_keypad_switch_map,
	.keymap_size = ARRAY_SIZE(mapphone_keypad_switch_map)
};

static struct gpio_event_info *mapphone_keypad_info[] = {
	&mapphone_keypad_matrix_info.info,
	&mapphone_keypad_switch_info.info,
};

static struct gpio_event_platform_data mapphone_keypad_data = {
	.name = "mapphone-keypad",
	.info = mapphone_keypad_info,
	.info_count = ARRAY_SIZE(mapphone_keypad_info)
};

static struct platform_device mapphone_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &mapphone_keypad_data,
	},
};

static int mapphone_reset_keys_up[] = {
	BTN_MOUSE,		/* XXX */
        0
};

static struct keyreset_platform_data mapphone_reset_keys_pdata = {
	//.crash_key = KEY_RIGHTSHIFT,
	.keys_up = mapphone_reset_keys_up,
	.keys_down = {
		KEY_LEFTSHIFT,
		KEY_LEFTALT,
		KEY_BACKSPACE,
		0
	},
};

struct platform_device mapphone_reset_keys_device = {
         .name = KEYRESET_NAME,
         .dev.platform_data = &mapphone_reset_keys_pdata,
};

static int __init mapphone_init_keypad(void)
{
	int ret = 0;
	printk("Entering Mapphone Keypad Init\n");

	/* keypad rows */
	omap_mux_init_gpio(34, OMAP_PIN_INPUT);
	omap_mux_init_gpio(35, OMAP_PIN_INPUT);
	omap_mux_init_gpio(36, OMAP_PIN_INPUT);
	omap_mux_init_gpio(37, OMAP_PIN_INPUT);
	omap_mux_init_gpio(38, OMAP_PIN_INPUT);
	omap_mux_init_gpio(39, OMAP_PIN_INPUT);
	omap_mux_init_gpio(40, OMAP_PIN_INPUT);
	omap_mux_init_gpio(41, OMAP_PIN_INPUT);

	/* keypad columns */
	omap_mux_init_gpio(43, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(53, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(54, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(55, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(56, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(58, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(63, OMAP_PIN_OUTPUT);

	/* switches */
	omap_mux_init_gpio(177, OMAP_PIN_INPUT);
	omap_mux_init_gpio(100, OMAP_PIN_INPUT);

	platform_device_register(&mapphone_reset_keys_device);
	ret = platform_device_register(&mapphone_keypad_device);
	printk("returning %d \n", ret);
}

device_initcall(mapphone_init_keypad);
