/*
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/types.h>
#include <linux/input/touch_platform.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/prom.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/qtouch_obp_ts.h>
#include "dt_path.h"
#include "mux.h"
#include <linux/interrupt.h>

void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info);
static void __init mapphone_legacy_qtouch_init(struct i2c_board_info *i2c_info);
static void __init mapphone_legacy_qtouch_gpio_init(
		struct i2c_board_info *i2c_info);
static ssize_t mapphone_legacy_qtouch_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static struct kobj_attribute mapphone_legacy_qtouch_virtual_keys_attr;
static struct attribute *mapphone_legacy_qtouch_properties_attrs[];
static struct attribute_group mapphone_legacy_qtouch_properties_attr_group;
struct attribute_group *mapphone_touch_vkey_prop_attr_group;


static int mapphone_legacy_qtouch_reset(void);



static struct vkey sholes_touch_vkeys[] = {
	{
		.code		= KEY_BACK,
		.center_x	= 32,
		.center_y	= 906,
		.width		= 63,
		.height		= 57,
	},
	{
		.code		= KEY_MENU,
		.center_x	= 162,
		.center_y	= 906,
		.width		= 89,
		.height		= 57,
	},
	{
		.code		= KEY_HOME,
		.center_x	= 292,
		.center_y	= 906,
		.width		= 89,
		.height		= 57,
	},
	{
		.code		= KEY_SEARCH,
		.center_x	= 439,
		.center_y	= 906,
		.width		= 63,
		.height		= 57,
	},
};

static struct qtm_touch_keyarray_cfg sholes_key_array_data[] = {
	{
		.ctrl		= 0,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0,
		.y_size		= 0,
		.aks_cfg	= 0,
		.burst_len	= 0,
		.tch_det_thr	= 0,
		.tch_det_int	= 0,
		.rsvd1		= 0,
		.rsvd2		= 0,
	},
	{
		.ctrl		= 0,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0,
		.y_size		= 0,
		.aks_cfg	= 0,
		.burst_len	= 0,
		.tch_det_thr	= 0,
		.tch_det_int	= 0,
		.rsvd1		= 0,
		.rsvd2		= 0,
	},
};

static struct qtouch_ts_platform_data qtouch_pdata = {
	.flags		= (QTOUCH_SWAP_XY |
			   QTOUCH_USE_MULTITOUCH |
			   QTOUCH_CFG_BACKUPNV |
			   QTOUCH_EEPROM_CHECKSUM),
	.irqflags	= (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
	.abs_min_x	= 20,
	.abs_max_x	= 1004,
	.abs_min_y	= 0,
	.abs_max_y	= 960,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.x_delta	= 400,
	.y_delta	= 250,
	.nv_checksum	= 0xfaf5,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.boot_i2c_addr	= 0x5f,
	.hw_reset	= mapphone_legacy_qtouch_reset,
	.key_array = {
		.cfg		= sholes_key_array_data,
		.keys		= NULL,
		.num_keys	= 0,
	},
	.power_cfg	= {
		.idle_acq_int	= 0xff,
		.active_acq_int	= 0xff,
		.active_idle_to	= 0x01,
	},
	.acquire_cfg	= {
		.charge_time	= 12,
		.atouch_drift	= 5,
		.touch_drift	= 20,
		.drift_susp	= 20,
		.touch_autocal	= 0x96,
		.sync		= 0,
		.atch_cal_suspend_time	= 0,
		.atch_cal_suspend_thres	= 0,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x0b,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 12,
		.y_size		= 7,
		.aks_cfg	= 0,
		.burst_len	= 0x40,
		.tch_det_thr	= 0x12,
		.tch_det_int	= 0x2,
		.orient		= 0x00,
		.mrg_to		= 25,
		.mov_hyst_init	= 0xe,
		.mov_hyst_next	= 0xe,
		.mov_filter	= 0x9,
		.num_touch	= 5,
		.merge_hyst	= 0,
		.merge_thresh	= 3,
		.amp_hyst       = 2,
		.x_res		= 0x0000,
		.y_res		= 0x0000,
		.x_low_clip	= 0x00,
		.x_high_clip	= 0x00,
		.y_low_clip	= 0x00,
		.y_high_clip	= 0x00,
		.x_edge_ctrl	= 0,
		.x_edge_dist	= 0,
		.y_edge_ctrl	= 0,
		.y_edge_dist	= 0,
	},
	.linear_tbl_cfg = {
		.ctrl		= 0x01,
		.x_offset	= 0x0000,
		.x_segment = {
			0x48, 0x3f, 0x3c, 0x3E,
			0x3f, 0x3e, 0x3e, 0x3e,
			0x3f, 0x42, 0x41, 0x3f,
			0x41, 0x40, 0x41, 0x46
		},
		.y_offset = 0x0000,
		.y_segment = {
			0x44, 0x38, 0x37, 0x3e,
			0x3e, 0x41, 0x41, 0x3f,
			0x42, 0x41, 0x42, 0x42,
			0x41, 0x3f, 0x41, 0x45
		},
	},
	.comms_config_cfg = {
		.ctrl		= 0,
		.command	= 0,
	},
	.gpio_pwm_cfg = {
		.ctrl			= 0,
		.report_mask		= 0,
		.pin_direction		= 0,
		.internal_pullup	= 0,
		.output_value		= 0,
		.wake_on_change		= 0,
		.pwm_enable		= 0,
		.pwm_period		= 0,
		.duty_cycle_0		= 0,
		.duty_cycle_1		= 0,
		.duty_cycle_2		= 0,
		.duty_cycle_3		= 0,
		.trigger_0		= 0,
		.trigger_1		= 0,
		.trigger_2		= 0,
		.trigger_3		= 0,
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x00,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.reserve0	= 0x00,
		.szthr1		= 0x00,
		.szthr2		= 0x00,
		.shpthr1	= 0x00,
		.shpthr2	= 0x00,
		.supextto	= 0x00,
	},
	.noise_suppression_cfg = {
		.ctrl			= 0,
		.outlier_filter_len	= 0,
		.reserve0		= 0,
		.gcaf_upper_limit	= 0,
		.gcaf_lower_limit	= 0,
		.gcaf_low_count		= 0,
		.noise_threshold	= 0,
		.reserve1		= 0,
		.freq_hop_scale		= 0,
		.burst_freq_0		= 0,
		.burst_freq_1		= 0,
		.burst_freq_2		= 0,
		.burst_freq_3		= 0,
		.burst_freq_4		= 0,
		.idle_gcaf_valid	= 0,
	},
	.touch_proximity_cfg = {
		.ctrl			= 0,
		.x_origin		= 0,
		.y_origin		= 0,
		.x_size			= 0,
		.y_size			= 0,
		.reserve0		= 0,
		.blen			= 0,
		.tch_thresh		= 0,
		.tch_detect_int		= 0,
		.average		= 0,
		.rate			= 0,
	},
	.one_touch_gesture_proc_cfg = {
		.ctrl			= 0,
		.reserve0		= 0,
		.gesture_enable		= 0,
		.pres_proc		= 0,
		.tap_time_out		= 0,
		.flick_time_out		= 0,
		.drag_time_out		= 0,
		.short_press_time_out	= 0,
		.long_press_time_out	= 0,
		.repeat_press_time_out	= 0,
		.flick_threshold	= 0,
		.drag_threshold		= 0,
		.tap_threshold		= 0,
		.throw_threshold	= 0,
	},
	.self_test_cfg = {
		.ctrl			= 0,
		.command		= 0,
		.high_signal_limit_0	= 0,
		.low_signal_limit_0	= 0,
		.high_signal_limit_1	= 0,
		.low_signal_limit_1	= 0,
		.high_signal_limit_2	= 0,
		.low_signal_limit_2	= 0,
	},
	.two_touch_gesture_proc_cfg = {
		.ctrl			= 0,
		.reserved0		= 0,
		.reserved1		= 0,
		.gesture_enable		= 0,
		.rotate_threshold	= 0,
		.zoom_threshold		= 0,
	},
	.cte_config_cfg = {
		.ctrl			= 1,
		.command		= 0,
		.mode			= 3,
		.idle_gcaf_depth	= 4,
		.active_gcaf_depth	= 8,
		.voltage		= 0,
	},
	.noise1_suppression_cfg = {
		.ctrl		= 0x01,
		.version	= 0x01,
		.atch_thr	= 0x64,
		.duty_cycle	= 0x08,
		.drift_thr	= 0x00,
		.clamp_thr	= 0x00,
		.diff_thr	= 0x00,
		.adjustment	= 0x00,
		.average	= 0x0000,
		.temp		= 0x00,
		.offset = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		},
		.bad_chan = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00
		},
		.x_short	= 0x00,
	},
	.vkeys			= { 
		.keys		= sholes_touch_vkeys,
		.count		= ARRAY_SIZE(sholes_touch_vkeys),
	},
};

void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info)
{
	printk(KERN_INFO "%s: Starting touch init...\n", __func__);

	if (i2c_info == NULL) {
		printk(KERN_ERR "%s: NULL i2c_board_info pointer passed.\n",
			__func__);
		goto fail;
	}

	mapphone_legacy_qtouch_init(i2c_info);  /* Legacy support */
	mapphone_legacy_qtouch_gpio_init(i2c_info);  /* Legacy support */
	/* The properties group register in board-mapphone.c*/
	mapphone_touch_vkey_prop_attr_group =
	&mapphone_legacy_qtouch_properties_attr_group;
fail:
	return;
}


static void __init mapphone_legacy_qtouch_init(struct i2c_board_info *i2c_info)
{
	i2c_info->platform_data = &qtouch_pdata;
	return;
}

static void __init mapphone_legacy_qtouch_gpio_init(
		struct i2c_board_info *i2c_info)
{
	int retval = 0;
	int pin = 0;
	int err = 0;

	pin = get_gpio_by_name("touch_panel_rst");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_reset");
		if (err >= 0) {
			err = gpio_direction_output(pin, 1);
			omap_mux_init_gpio(pin,OMAP_PIN_OUTPUT);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config reset.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Reset GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = pin;
		goto legacy_qtouch_gpio_init_fail;
	}

	pin = get_gpio_by_name("touch_panel_int");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_irq");
		if (err >= 0) {
			err = gpio_direction_input(pin);
			omap_mux_init_gpio(pin,OMAP_PIN_INPUT);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config irq.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: IRQ GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		goto legacy_qtouch_gpio_init_fail;
	}
	i2c_info->irq = gpio_to_irq(pin);

	/*	AFAIK no touch_panel_wake pin on sholes/milestone */
/*
	pin = get_gpio_by_name("touch_panel_wake");
	if (pin >= 0) {
		printk(KERN_INFO "%s: Acquire wake pin.\n", __func__);
		err = gpio_request(pin, "touch_wake");
		if (err >= 0) {
			err = gpio_direction_output(pin, 1);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config wake.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Wake GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	} */

legacy_qtouch_gpio_init_fail:
	if (retval < 0) {
		printk(KERN_ERR "%s: GPIO init failed with error code %d.\n",
				__func__, retval);
	}
	return;
}

static ssize_t mapphone_legacy_qtouch_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	/* center: x: home: 55, menu: 185, back: 305, search 425, y: 835 */
	/* keys are specified by setting the x,y of the center, the width,
	 * and the height, as such keycode:center_x:center_y:width:height */
	return sprintf(buf, __stringify(EV_KEY) ":"
		       __stringify(KEY_BACK) ":32:906:63:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_MENU) ":162:906:89:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_HOME) ":292:906:89:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_SEARCH) ":439:906:63:57"
		       "\n");
}

static struct kobj_attribute mapphone_legacy_qtouch_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.qtouch-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mapphone_legacy_qtouch_virtual_keys_show,
};

static struct attribute *mapphone_legacy_qtouch_properties_attrs[] = {
	&mapphone_legacy_qtouch_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mapphone_legacy_qtouch_properties_attr_group = {
	.attrs = mapphone_legacy_qtouch_properties_attrs,
};

static int mapphone_legacy_qtouch_reset(void)
{
	int reset_pin;
	int retval = 0;

	reset_pin = get_gpio_by_name("touch_panel_rst");
	if (reset_pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = reset_pin;
	} else {
		gpio_direction_output(reset_pin, 1);
		msleep(1);
		gpio_set_value(reset_pin, 0);
		msleep(QTM_OBP_SLEEP_RESET_HOLD);
		gpio_set_value(reset_pin, 1);
	}

	return retval;
}
