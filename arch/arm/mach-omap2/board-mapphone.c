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


#define WILINK_UART_DEV_NAME "/dev/ttyO3"
#define MAPPHONE_POWER_OFF_GPIO 176

#define MAPPHONE_WIFI_PMENA_GPIO 179 //nshutdown
#define MAPPHONE_WIFI_IRQ_GPIO 178 // hostwake

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
	int use_utmi = 1;
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

//	if (use_utmi)
//		musb_board_data.interface_type = MUSB_INTERFACE_UTMI;

//	if (power > 100 && power <= 500 )
//		musb_board_data.power = power;

	usb_musb_init(&musb_board_data);
}

static bool uart_req;
static struct wake_lock st_wk_lock;

static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}


/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 174, //TODO: Need check this value on defy!
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = 0,
	.resume = 0,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *mapphone_devices[] __initdata = {
	&wl128x_device,
	&btwilink_device,
};

static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = -1, /* OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),*/
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;
	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));
	if (power_on == power_state)
		return 0;
	power_state = power_on;
	if (power_on) {
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 1);
		mdelay(15);
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 0);
		mdelay(1);
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 1);
		mdelay(70);
	} else
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 0);
	return 0;
}

static void mapphone_wifi_init(void)
{
	int ret;
	printk("mapphone_wifi_init\n");

	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %ld\n", __func__,
			MAPPHONE_WIFI_PMENA_GPIO);
		return;
	}

	ret = gpio_request(MAPPHONE_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_IRQ_GPIO);
		return;
	}

	gpio_direction_input(MAPPHONE_WIFI_IRQ_GPIO);
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);
	mapphone_wlan_data.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO);

	if (wl12xx_set_platform_data(&mapphone_wlan_data)) {
		pr_err("Error setting wl12xx data\n");
	return;
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
#if 0
	clkp = clk_get(NULL, "l3_ick");
		if (clkp) {
             clk_enable(clkp);
             printk("l3_ick enabled\n");
	}

	clkp = clk_get(NULL, "core_l3_ick");
		if (clkp) {
             clk_enable(clkp);
             printk("core_l3_ick enabled\n");
	}


	clkp = clk_get(NULL, "hsotgusb_ick");
		if (clkp) {
             clk_enable(clkp);
             printk("hsotgusb_ick enabled\n");
	}
#endif
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
//#define NAND_BLOCK_SIZE (64 * 2048)

static struct mtd_partition nand_partitions[] = {
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


/* all of tehse without a declared unit are in nanoseconds */
static struct gpmc_timings nand_timings = {
	.sync_clk 	= 0, 	/* Minimum clock period for synchronous mode (in picoseconds) */

	.cs_on 		= 0,	 /* Assertion time */
	.cs_rd_off	= 60,	 /* Read deassertion time */
	.cs_wr_off	= 48,	/* Write deassertion time */

	.adv_on 	= 6,	/* Assertion time */
	.adv_rd_off	= 24,	/* Read deassertion time */
	.adv_wr_off	= 48,	/* Write deassertion time */

	.we_on		= 24,	/* WE assertion time */
	.we_off		= 36,	/* WE deassertion time */

	.oe_on		= 24,	/* OE assertion time */
	.oe_off		= 42,	/* OE deassertion time */

	.access		= 36,	/* Start-cycle to first data valid delay */
	.rd_cycle	= 36,	/* Total read cycle time */
	.wr_cycle	= 36,	/* Total write cycle time */

	.wr_access	= 30,	/* WRACCESSTIME */
	.wr_data_mux_bus= 0,	/* WRDATAONADMUXBUS */
};

static struct omap_nand_platform_data board_nand_data = {
	.nand_setup	= NULL,
	.gpmc_t		= &nand_timings, /* review these*/
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	=  1,
	.xfer_type	= 0, // may need polled mode for apanic but NAND_OMAP_PREFETCH_IRQ may be better
	.devsize	= NAND_BUSWIDTH_16,
	.cs		= 0,
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
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


static void __init omap_mapphone_init(void)
{

	//omap3_mux_init(board_mux, OMAP_PACKAGE_CBB); //Necessary?

	/*
	* This will allow unused regulator to be shutdown. This flag
	* should be set in the board file. Before regulators are registered.
	*/
	//regulator_has_full_constraints();

	omap_serial_init();
	mapphone_bp_model_init();
	mapphone_voltage_init();
	mapphone_gpio_mapping_init();
	mapphone_i2c_init();
	mapphone_padconf_init();
	omap_register_ion();
	platform_add_devices(mapphone_devices, ARRAY_SIZE(mapphone_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	mapphone_spi_init();
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif
	mapphone_mdm_ctrl_init();
	mapphone_cpcap_client_init();
	mapphone_panel_init();
	mapphone_als_init();
	omap_hdq_init();

	usb_musb_init(NULL);

	mapphone_usbhost_init();
	//mapphone_musb_init();



	mapphone_power_off_init();
	mapphone_hsmmc_init();
	gpmc_nand_init(&board_nand_data);
	mapphone_wifi_init(); //Disabled until i check gpios, irqs, and mmmc slot


	

	omap_enable_smartreflex_on_init();
	mapphone_create_board_props();
	mapphone_gadget_init();


	//mapphone_ramconsole_init();
}

static void __init mapphone_reserve(void)
{
	//omap2_ramconsole_reserve_sdram();
	omap_ram_console_init(PLAT_PHYS_OFFSET + 0x8E00000,0x200000); //  0x0x9000000  0x8E00000 0x200000

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
