/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bootmem.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/led-lm3530.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>
#include <linux/regulator/machine.h>

#include <../drivers/w1/w1_family.h> /* for W1_EEPROM_DS2502 */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/gpmc-smc91x.h>
#include <plat/usb.h>
#include <plat/system.h>
#include <plat/mux.h>
#include <plat/hdq.h>
#include <plat/omap-serial.h>
#include <plat/omap_hsi.h>

#ifdef CONFIG_EMU_UART_DEBUG
#include <plat/board-mapphone-emu_uart.h>
#endif

#include <mach/board-mapphone.h>

#include "board-flash.h"
#include "mux.h"
#include "sdram-toshiba-hynix-numonyx.h"
#include "omap_ion.h"
#include "dt_path.h"
#include "pm.h"
#include "hsmmc.h"
#include "omap_ram_console.h"
#include "control.h"

#define WILINK_UART_DEV_NAME "/dev/ttyO1"
#define MAPPHONE_POWER_OFF_GPIO 176
#define MAPPHONE_WIFI_PMENA_GPIO 186
#define MAPPHONE_WIFI_IRQ_GPIO 65
#define MAPPHONE_BT_RESET_GPIO 21 //get_gpio_by_name("bt_reset_b")

char *bp_model = "CDMA";

#if !defined(CONFIG_ARM_APPENDED_DTB)
static char boot_mode[BOOT_MODE_MAX_LEN+1];

int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	boot_mode[BOOT_MODE_MAX_LEN] = '\0';
	pr_debug("boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);
#endif

/* Flat dev tree address */
#define ATAG_FLAT_DEV_TREE_ADDRESS 0xf100040A
struct tag_flat_dev_tree_address {
	u32 address;
	u32 size;
};


static const char *mapphone_dt_match[] __initconst = {
	"moto,mapphone_",
	NULL
};


static u32 fdt_start_address;
static u32 fdt_size;

/* process flat device tree for hardware configuration */
static int __init parse_tag_flat_dev_tree_address(const struct tag *tag)
{
	struct tag_flat_dev_tree_address *fdt_addr =
		(struct tag_flat_dev_tree_address *)&tag->u;

	if (fdt_addr->size) {
		fdt_start_address = (u32)phys_to_virt(fdt_addr->address);
		fdt_size = fdt_addr->size;
	}

	/*have_of = 1;*/
	printk(KERN_INFO
		"flat_dev_tree_virt_address=0x%08x, flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		fdt_start_address,
		fdt_addr->address,
		fdt_addr->size);

	return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);


static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static int __init omap_hdq_init(void)
{
	omap_hdq_dev.dev.platform_data = &mapphone_hdq_data;
	return platform_device_register(&omap_hdq_dev);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode                   = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode                   = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode                   = MUSB_PERIPHERAL,
#endif
	.power                  = 100,
};

static void __init mapphone_musb_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int use_utmi = 0;
	u16 power = 100;
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);

	if (node) {
		prop = of_get_property(node, "feature_musb_utmi", &size);
		if (prop && size) {
			use_utmi = *(u8 *)prop;
			pr_debug("Using %s as the MUSB Mode \n",
				use_utmi ? "UTMI" : "ULPI");

		} else
			pr_debug("USB Defaulting to ULPI \n");
		of_node_put(node);
	}

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_CHOSEN_MUSBHS_EXTPOWER, &size);
		if (prop && size) {
			power = *(u16 *)prop;
			pr_debug("Current supplied by ext power: %d\n", power);
		}
		of_node_put(node);
	}

	if (use_utmi)
		musb_board_data.interface_type = MUSB_INTERFACE_UTMI;

	if (power > 100 && power <= 500 )
		musb_board_data.power = power;

	usb_musb_init(&musb_board_data);
}

static bool uart_req;
static struct wake_lock st_wk_lock;

/* wl127x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = MAPPHONE_BT_RESET_GPIO,
	.dev_name = WILINK_UART_DEV_NAME,
	.baud_rate = 3686400,
	.flow_cntrl = 1,
	.suspend = 0,
	.resume = 0,
};

static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device beagle_cam_device = {
.name	= "beagle_cam",
.id	= -1,
};


static struct platform_device *mapphone_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
	&beagle_cam_device,
};

static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_26, 
};

int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;
	printk("Powering %s wifi\n", (power_on ? "on" : "off"));
	if (power_on == power_state) {
		return 0;
	}
	power_state = power_on;
	if (power_on) {
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 1);
	} else {
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 0);
	}
	return 0;
}

static void mapphone_wifi_init(void)
{
	int ret;
	printk("mapphone_wifi_init\n");

	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_PMENA_GPIO);
		return;
	}
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);
	mapphone_wlan_data.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO);

	if (wl12xx_set_platform_data(&mapphone_wlan_data))
	{
		pr_err("Error setting wl12xx data\n");
	}
	printk("Wifi init done\n");
	return;
}

static void __init mapphone_bp_model_init(void)
{
#ifdef CONFIG_OMAP_RESET_CLOCKS
	struct clk *clkp;
#endif
	struct device_node *bp_node;
	const void *bp_prop;

	if ((bp_node = of_find_node_by_path(DT_PATH_CHOSEN))) {
		if ((bp_prop = of_get_property(bp_node, \
			DT_PROP_CHOSEN_BP, NULL)))
			bp_model = (char *)bp_prop;
		printk("BP MODEL:%s\n",bp_model);
		of_node_put(bp_node);
	}
#ifdef CONFIG_OMAP_RESET_CLOCKS
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp) {
             clk_enable(clkp);
             printk("sad2d_ick enabled\n");
	}
#endif
}

static void mapphone_pm_power_off(void)
{
	printk(KERN_INFO "mapphone_pm_power_off start...\n");
	local_irq_disable();

	/* config gpio 176 back from safe mode to reset the device */
	omap_writew(0x4, 0x480021D2);
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 0);

	do {} while (1);

	local_irq_enable();
}

static void mapphone_pm_reset(void)
{
	arch_reset('h', NULL);
}

static int cpcap_charger_connected_probe(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_reset;
	return 0;
}

static int cpcap_charger_connected_remove(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_power_off;
	return 0;
}

static struct platform_driver cpcap_charger_connected_driver = {
	.probe          = cpcap_charger_connected_probe,
	.remove         = cpcap_charger_connected_remove,
	.driver         = {
		.name   = "cpcap_charger_connected",
		.owner  = THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);

	/* config gpio176 into safe mode with the pull up enabled to avoid
	 * glitch at reboot */
	omap_writew(0x1F, 0x480021D2);
	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
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
static void __init mapphone_voltage_init(void)
{
	/* cpcap is the default power supply for core and iva */
	omap_cpcap_init();
}

void __init mapphone_create_board_props(void)
{
	struct kobject *board_props_kobj;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_obj;

	if (mapphone_touch_vkey_prop_attr_group) {
		ret = sysfs_create_group(board_props_kobj,
				mapphone_touch_vkey_prop_attr_group);
		if (ret)
			goto err_board_obj;
	}

err_board_obj:
	if (!board_props_kobj || ret)
		pr_err("failed to create board_properties\n");

}


static void __init omap_mapphone_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(JEDEC_JESD209A_sdrc_params,
				   JEDEC_JESD209A_sdrc_params);

	if (fdt_start_address) {
		void *mem;
		mem = __alloc_bootmem(fdt_size, __alignof__(int), 0);
		BUG_ON(!mem);
		memcpy(mem, (const void *)fdt_start_address, fdt_size);
		initial_boot_params = (struct boot_param_header *)mem;
		pr_info("Unflattening device tree from ATAG: 0x%08x\n", (u32)mem);
		unflatten_device_tree();
	}

}


#include "common-board-devices.h"
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <plat/nand.h>
#include <plat/gpmc.h>

/*these get overwritten by cmdline mtdparts*/
static struct mtd_partition nand_dummy_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,			/* Offset = 0x00000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x160000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};

/*
Old timings:
root@android:/ # time dd if=/dev/zero of=/cache/test bs=1048576 count=20
20+0 records in
20+0 records out
20971520 bytes transferred in 5.656 secs (3707835 bytes/sec)
    0m5.66s real     0m0.00s user     0m5.47s system 
passing no:
root@android:/ # time dd if=/dev/zero of=/cache/test bs=1048576 count=20
20+0 records in
20+0 records out
20971520 bytes transferred in 5.653 secs (3709803 bytes/sec)
    0m6.11s real     0m0.00s user     0m5.83s system 
New timings:
root@android:/ # time dd if=/dev/zero of=/cache/test bs=1048576 count=20
20+0 records in
20+0 records out
20971520 bytes transferred in 5.728 secs (3661229 bytes/sec)
    0m5.75s real     0m0.00s user     0m5.47s system

slower, but hopefully more stable
*/


/* all of these without a declared unit are in nanoseconds */
static struct gpmc_timings nand_timings = {
	.sync_clk 	= 0, 	/* Minimum clock period for synchronous mode (in picoseconds) */

	.cs_on 		= 0,	 /* Assertion time */
	.cs_rd_off	= 36,	 /* Read deassertion time */
	.cs_wr_off	= 48,	/* Write deassertion time */

	.adv_on 	= 6,	/* Assertion time */
	.adv_rd_off	= 24,	/* Read deassertion time */
	.adv_wr_off	= 48,	/* Write deassertion time */

	.we_on		= 24,	/* WE assertion time */
	.we_off		= 36,	/* WE deassertion time */

	.oe_on		= 24,	/* OE assertion time */
	.oe_off		= 42,	/* OE deassertion time */

	.access		= 54,	/* Start-cycle to first data valid delay */
	.rd_cycle	= 72,	/* Total read cycle time */
	.wr_cycle	= 72,	/* Total write cycle time */

	.wr_access	= 30,	/* WRACCESSTIME */
	.wr_data_mux_bus= 0,	/* WRDATAONADMUXBUS */
};

static struct omap_nand_platform_data board_nand_data = {
	.nand_setup	= NULL,
	.gpmc_t		= &nand_timings, /* review these*/
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	=  1,
	.xfer_type	= NAND_OMAP_PREFETCH_IRQ, // may need polled mode for apanic but NAND_OMAP_PREFETCH_IRQ may be better
	.devsize	= NAND_BUSWIDTH_16,
	.cs		= 0,
	.parts		= nand_dummy_partitions,
	.nr_parts	= ARRAY_SIZE(nand_dummy_partitions),
	.ecc_opt	= OMAP_ECC_HAMMING_CODE_HW,
	.gpmc_irq	= INT_34XX_GPMC_IRQ,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
		 { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_START   0x8E000000
#define RAM_CONSOLE_SIZE    0x20000
static struct resource ram_console_resource = {
       .start  = RAM_CONSOLE_START,
       .end    = (RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1),
       .flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
       .name = "ram_console",
       .id = 0,
       .num_resources  = 1,
       .resource       = &ram_console_resource,
};

static inline void mapphone_ramconsole_init(void)
{
	platform_device_register(&ram_console_device);
}

static inline void omap2_ramconsole_reserve_sdram(void)
{
	reserve_bootmem(RAM_CONSOLE_START, RAM_CONSOLE_SIZE, 0);
}
#else
static inline void sholes_ramconsole_init(void) {}

static inline void omap2_ramconsole_reserve_sdram(void) {}
#endif

#define SHOLES_BP_READY_AP_GPIO		141
#define SHOLES_BP_READY2_AP_GPIO	59
#define SHOLES_BP_RESOUT_GPIO		139
#define SHOLES_BP_PWRON_GPIO		137
#define SHOLES_AP_TO_BP_PSHOLD_GPIO	138
#define SHOLES_AP_TO_BP_FLASH_EN_GPIO	157

#define SHOLES_BPWAKE_STROBE_GPIO	157
#define SHOLES_APWAKE_TRIGGER_GPIO      141
#define SHOLES_IPC_USB_SUSP_GPIO	142


#include <linux/omap_mdm_ctrl.h>


static struct omap_mdm_ctrl_platform_data omap_mdm_ctrl_platform_data = {
	.bp_ready_ap_gpio = SHOLES_BP_READY_AP_GPIO,
	.bp_ready2_ap_gpio = SHOLES_BP_READY2_AP_GPIO,
	.bp_resout_gpio = SHOLES_BP_RESOUT_GPIO,
	.bp_pwron_gpio = SHOLES_BP_PWRON_GPIO,
	.ap_to_bp_pshold_gpio = SHOLES_AP_TO_BP_PSHOLD_GPIO,
	.ap_to_bp_flash_en_gpio = SHOLES_AP_TO_BP_FLASH_EN_GPIO,
};

static struct platform_device omap_mdm_ctrl_platform_device = {
	.name = OMAP_MDM_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &omap_mdm_ctrl_platform_data,
	},
};

static int __init sholes_omap_mdm_ctrl_init(void)
{
	gpio_request(SHOLES_BP_READY2_AP_GPIO, "BP Flash Ready");
	gpio_direction_input(SHOLES_BP_READY2_AP_GPIO);

	//omap_cfg_reg(T4_34XX_GPIO59_DOWN);

	omap_mux_init_gpio(SHOLES_BP_READY2_AP_GPIO,OMAP_PIN_INPUT_PULLDOWN);

	gpio_request(SHOLES_BP_RESOUT_GPIO, "BP Reset Output");
	gpio_direction_input(SHOLES_BP_RESOUT_GPIO);
	//omap_cfg_reg(AE3_34XX_GPIO139_DOWN);
	omap_mux_init_gpio(SHOLES_BP_RESOUT_GPIO,OMAP_PIN_INPUT_PULLDOWN);

	gpio_request(SHOLES_BP_PWRON_GPIO, "BP Power On");
	gpio_direction_output(SHOLES_BP_PWRON_GPIO, 0);
	//omap_cfg_reg(AH3_34XX_GPIO137_OUT);
	omap_mux_init_gpio(SHOLES_BP_PWRON_GPIO,OMAP_PIN_OUTPUT);

	gpio_request(SHOLES_AP_TO_BP_PSHOLD_GPIO, "AP to BP PS Hold");
	gpio_direction_output(SHOLES_AP_TO_BP_PSHOLD_GPIO, 0);
	//omap_cfg_reg(AF3_34XX_GPIO138_OUT);
	omap_mux_init_gpio(SHOLES_AP_TO_BP_PSHOLD_GPIO,OMAP_PIN_OUTPUT);

}

/* for sdio wl1271 */
static void __init config_mmc2_init(void)
{
	u32 val;
	/* Set internal loopback clock */
	val = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1); 
	omap_ctrl_writel((val | OMAP2_MMCSDIO2ADPCLKISEL),
				OMAP343X_CONTROL_DEVCONF1);
}


static int wake_gpio_strobe = SHOLES_BPWAKE_STROBE_GPIO;
static struct wake_lock uart_lock;

static void mapphone_uart_hold_wakelock(void *up, int flag);
static void mapphone_uart_probe(struct uart_omap_port *up);
static void mapphone_uart_remove(struct uart_omap_port *up);
static void mapphone_uart_wake_peer(struct uart_port *up);

/* Give 1s wakelock time for each port */
static u8 wakelock_length[OMAP_MAX_HSUART_PORTS] = {2, 0, 0};

static struct omap_device_pad mapphone_uart1_pads[] __initdata = {
	{
		.name	= "uart1_cts.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rts.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad mapphone_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
			OMAP_OFFOUT_EN | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

//#ifdef CONFIG_EMU_UART_DEBUG
static struct omap_device_pad mapphone_uart3_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rts_sd.uart3_rts_sd",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};
//#endif

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma		= 0,
		.dma_rx_buf_size	= DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate	= DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout		= DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout		= 1000, /* Reduce idle time, 5s -> 1s */
		.flags			= 1,
		.plat_hold_wakelock	= mapphone_uart_hold_wakelock,
		.board_uart_probe	= mapphone_uart_probe,
		.board_uart_remove	= mapphone_uart_remove,
		.wake_peer	= mapphone_uart_wake_peer,
		.ctsrts			= 0,
		.is_clear_fifo	= 0,
		.rx_safemode = 0,
		.auto_sus_timeout	= 3000,
		.wer	= (OMAP_UART_WER_RLSI | \
				OMAP_UART_WER_RHRI | OMAP_UART_WER_RX | \
				OMAP_UART_WER_DCDCD | OMAP_UART_WER_RI | \
				OMAP_UART_WER_DSR | OMAP_UART_WER_CTS),
	},
	{
		.use_dma		= 0,
		.dma_rx_buf_size	= DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate	= DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout		= DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout		= 1000, /* Reduce idle time, 5s -> 1s */
		.flags			= 1,
		.plat_hold_wakelock	= mapphone_uart_hold_wakelock,
		.ctsrts			= 0,
		.is_clear_fifo	= 1,
		.rx_safemode = 0,
		.auto_sus_timeout	= 3000,
		.wer	= (OMAP_UART_WER_RLSI | \
				OMAP_UART_WER_RHRI | OMAP_UART_WER_RX | \
				OMAP_UART_WER_DCDCD | OMAP_UART_WER_RI | \
				OMAP_UART_WER_DSR | OMAP_UART_WER_CTS),
	},
	{
		.use_dma		= 0,
		.dma_rx_buf_size	= DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate	= DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout		= DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout		= 1000, /* Reduce idle time, 5s -> 1s */
		.flags			= 1,
		.plat_hold_wakelock	= mapphone_uart_hold_wakelock,
		.ctsrts			= 0,
		.is_clear_fifo	= 1,
		.rx_safemode = 0,
		.auto_sus_timeout	= 3000,
		.wer	= (OMAP_UART_WER_RLSI | \
				OMAP_UART_WER_RHRI | OMAP_UART_WER_RX | \
				OMAP_UART_WER_DCDCD | OMAP_UART_WER_RI | \
				OMAP_UART_WER_DSR | OMAP_UART_WER_CTS),
	},
};

static void mapphone_uart_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;

	/* Supply 500ms precision on wakelock */
	if (wakelock_length[up2->pdev->id])
		wake_lock_timeout(&uart_lock,
				wakelock_length[up2->pdev->id]*HZ/2);

	return;
}

static void mapphone_uart_probe(struct uart_omap_port *up)
{
	if (wake_gpio_strobe >= 0) {
		if (gpio_request(wake_gpio_strobe,
				 "UART wakeup strobe")) {
			printk(KERN_ERR "Error requesting GPIO\n");
		} else {
			gpio_direction_output(wake_gpio_strobe, 0);
		}
	}
}

static void mapphone_uart_remove(struct uart_omap_port *up)
{
	if (wake_gpio_strobe >= 0)
		gpio_free(wake_gpio_strobe);
}

static void mapphone_uart_wake_peer(struct uart_port *up)
{
	if (wake_gpio_strobe >= 0) {
		gpio_direction_output(wake_gpio_strobe, 1);
		udelay(5);
		gpio_direction_output(wake_gpio_strobe, 0);
		udelay(5);
	}
}

void mapphone_serial_init(void)
{
	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");
	omap_serial_init_port_pads(0, mapphone_uart1_pads,
		ARRAY_SIZE(mapphone_uart1_pads), &omap_serial_platform_data[0]);
	omap_serial_init_port_pads(1, mapphone_uart2_pads,
		ARRAY_SIZE(mapphone_uart2_pads), &omap_serial_platform_data[1]);
//#ifdef CONFIG_EMU_UART_DEBUG
	omap_serial_init_port_pads(2, mapphone_uart3_pads,
		ARRAY_SIZE(mapphone_uart3_pads), &omap_serial_platform_data[2]);
//#endif
}



static void __init omap_mapphone_init(void)
{

	mapphone_power_off_init();
	/*
	* This will allow unused regulator to be shutdown. This flag
	* should be set in the board file. Before regulators are registered.
	*/
	regulator_has_full_constraints();

	omap_serial_init(NULL);

	mapphone_mdm_ctrl_init();
	//mapphone_serial_init();
	mapphone_bp_model_init();
	mapphone_voltage_init();
	mapphone_gpio_mapping_init();

	mapphone_i2c_init();

	mapphone_padconf_init();	/*mux is done here, so anything dependant should go after*/
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBC); //for testing

	/* keypad column fixup after mux_init */
/*	omap_mux_init_gpio(43, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(53, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(54, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(55, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(56, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(58, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(63, OMAP_PIN_OUTPUT);
	gpio_direction_output(43 ,1);
	gpio_direction_output(53 ,1);
	gpio_direction_output(54 ,1);
	gpio_direction_output(55 ,1);
	gpio_direction_output(56 ,1);
	gpio_direction_output(57 ,1);
	gpio_direction_output(58 ,1);
	gpio_direction_output(63 ,1); */

	//sholes_omap_mdm_ctrl_init(); //might need for old ril

	omap_register_ion();
	platform_add_devices(mapphone_devices, ARRAY_SIZE(mapphone_devices));
	mapphone_spi_init();
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif
	mapphone_cpcap_client_init();
	mapphone_panel_init();
	mapphone_als_init();
	omap_hdq_init();
	mapphone_musb_init();
	mapphone_usbhost_init();
	config_mmc2_init(); //setup mux and loopback clock before probe
	mapphone_hsmmc_init();
	gpmc_nand_init(&board_nand_data);
	mapphone_wifi_init();
	omap_enable_smartreflex_on_init();
	mapphone_create_board_props();
	mapphone_gadget_init();

}

static void __init mapphone_reserve(void)
{
	omap_ram_console_init(PLAT_PHYS_OFFSET + 0x8E00000,0x200000); //Last 2Mb of ram 

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}

MACHINE_START(MAPPHONE, "mapphone_")
	.boot_params	= 0x80C00100,
	.reserve	= mapphone_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_mapphone_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_mapphone_init,
	.timer		= &omap_timer,
	.dt_compat	= &mapphone_dt_match,
MACHINE_END
