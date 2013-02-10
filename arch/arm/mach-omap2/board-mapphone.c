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

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl127x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = MAPPHONE_BT_RESET_GPIO,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};

static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *mapphone_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
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
	.xfer_type	= 0, // may need polled mode for apanic but NAND_OMAP_PREFETCH_IRQ may be better
	.devsize	= NAND_BUSWIDTH_16,
	.cs		= 0,
	.parts		= nand_dummy_partitions,
	.nr_parts	= ARRAY_SIZE(nand_dummy_partitions),
	.ecc_opt	= OMAP_ECC_HAMMING_CODE_HW,
	.gpmc_irq	= INT_34XX_GPMC_IRQ,
};
/* 2.6.32 Mux Data:
<4>[    0.000000] MUX: setup AF26_34XX_GPIO0 (0xfa0021e0): 0x011f -> 0x0104
<4>[    0.000000] MUX: setup AA9_34XX_UART1_RTS (0xfa00217e): 0x0007 -> 0x0000
<4>[    0.000000] MUX: setup W8_34XX_UART1_CTS (0xfa002180): 0x0007 -> 0x0100
<4>[    0.000000] MUX: setup AA25_34XX_UART2_TX (0xfa002178): 0x011f -> 0x0000
<4>[    0.000000] MUX: setup AD25_34XX_UART2_RX (0xfa00217a): 0x011f -> 0x0118

<4>[    0.000000] MUX: setup AB25_34XX_UART2_RTS (0xfa002176): 0x011f -> 0x0000
<4>[    0.000000] MUX: setup AB26_34XX_UART2_CTS (0xfa002174): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AC27_34XX_GPIO92 (0xfa002108): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup AH22_34XX_DSI_DY0 (0xfa0020de): 0x0001 -> 0x0201
<4>[    0.000000] MUX: setup AH23_34XX_DSI_DY1 (0xfa0020e2): 0x0001 -> 0x0201

<4>[    0.000000] MUX: setup AH24_34XX_DSI_DY2 (0xfa0020e6): 0x0001 -> 0x0201
<4>[    0.000000] MUX: setup Y3_34XX_GPIO180 (0xfa0021da): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup AG25_34XX_GPIO10 (0xfa002a1a): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup B26_34XX_GPIO111 (0xfa00212e): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup Y4_34XX_GPIO181 (0xfa0021dc): 0x011f -> 0x0004

<4>[    0.000000] MUX: setup AB10_34XX_GPIO28_OUT (0xfa0025f8): 0x010c -> 0x0004
<4>[    0.000000] MUX: setup AC3_34XX_GPIO175 (0xfa0021d0): 0x011f -> 0x0104
<4>[    0.000000] MUX: setup A24_34XX_CAM_HS (0xfa00210c): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup A23_34XX_CAM_VS (0xfa00210e): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup C25_34XX_CAM_XCLKA (0xfa002110): 0x010f -> 0x0000

<4>[    0.000000] MUX: setup C27_34XX_CAM_PCLK (0xfa002112): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup C23_34XX_CAM_FLD (0xfa002114): 0x010f -> 0x0004
<4>[    0.000000] MUX: setup AG17_34XX_CAM_D0 (0xfa002116): 0x010f -> 0x011c
<4>[    0.000000] MUX: setup AH17_34XX_CAM_D1 (0xfa002118): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup B24_34XX_CAM_D2 (0xfa00211a): 0x010f -> 0x0100

<4>[    0.000000] MUX: setup C24_34XX_CAM_D3 (0xfa00211c): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup D24_34XX_CAM_D4 (0xfa00211e): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup A25_34XX_CAM_D5 (0xfa002120): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup K28_34XX_CAM_D6 (0xfa002122): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup L28_34XX_CAM_D7 (0xfa002124): 0x010f -> 0x0100

<4>[    0.000000] MUX: setup K27_34XX_CAM_D8 (0xfa002126): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup L27_34XX_CAM_D9 (0xfa002128): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup B25_34XX_CAM_D10 (0xfa00212a): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup C26_34XX_CAM_D11 (0xfa00212c): 0x010f -> 0x0100
<4>[    0.000000] MUX: setup B23_34XX_CAM_WEN (0xfa002130): 0x010f -> 0x0002

<4>[    0.000000] MUX: setup D25_34XX_CAM_STROBE (0xfa002132): 0x010f -> 0x0000
<4>[    0.000000] MUX: setup K8_34XX_GPMC_WAIT2 (0xfa0020d0): 0x011f -> 0x0004
<4>[    0.000000] MUX: setup H19_34XX_GPIO164_OUT (0xfa00219c): 0x011f -> 0x0004
<4>[    0.000000] MUX: setup AG17_34XX_GPIO99 (0xfa002116): 0x011c -> 0x0104
<4>[    0.000000] MUX: setup P21_OMAP34XX_MCBSP2_FSX (0xfa00213c): 0x010f -> 0x0108

<4>[    0.000000] MUX: setup N21_OMAP34XX_MCBSP2_CLKX (0xfa00213e): 0x010f -> 0x0108
<4>[    0.000000] MUX: setup R21_OMAP34XX_MCBSP2_DR (0xfa002140): 0x010f -> 0x0108
<4>[    0.000000] MUX: setup M21_OMAP34XX_MCBSP2_DX (0xfa002142): 0x010f -> 0x0000
<4>[    0.000000] MUX: setup K26_OMAP34XX_MCBSP3_FSX (0xfa002196): 0x010f -> 0x010a
<4>[    0.000000] MUX: setup W21_OMAP34XX_MCBSP3_CLKX (0xfa002198): 0x010f -> 0x010a

<4>[    0.000000] MUX: setup U21_OMAP34XX_MCBSP3_DR (0xfa002192): 0x010f -> 0x010a
<4>[    0.000000] MUX: setup V21_OMAP34XX_MCBSP3_DX (0xfa002190): 0x010f -> 0x0002
<4>[    0.000000] MUX: setup AE5_34XX_GPIO143 (0xfa002172): 0x010f -> 0x0104
<4>[    0.000000] MUX: setup AF5_34XX_GPIO142 (0xfa002170): 0x0004 -> 0x0104
<4>[    0.000000] MUX: setup AD1_3430_USB3FS_PHY_MM3_RXRCV (0xfa002186): 0x0107 -> 0x010e

<4>[    0.000000] MUX: setup AD2_3430_USB3FS_PHY_MM3_TXDAT (0xfa002188): 0x0007 -> 0x010e
<4>[    0.000000] MUX: setup AC1_3430_USB3FS_PHY_MM3_TXEN_N (0xfa00218a): 0x0007 -> 0x0006
<4>[    0.000000] MUX: setup AE1_3430_USB3FS_PHY_MM3_TXSE0 (0xfa002184): 0x0107 -> 0x010e
<4>[    0.000000] MUX: setup H16_34XX_SDRC_CKE0 (0xfa002262): 0x011f -> 0x0000
<4>[    0.000000] MUX: setup H17_34XX_SDRC_CKE1 (0xfa002264): 0x011f -> 0x0000

<4>[    0.000000] MUX: setup AE2_34XX_MMC2_CLK (0xfa002158): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup AG5_34XX_MMC2_CMD (0xfa00215a): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AH5_34XX_MMC2_DAT0 (0xfa00215c): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AH4_34XX_MMC2_DAT1 (0xfa00215e): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AG4_34XX_MMC2_DAT2 (0xfa002160): 0x011f -> 0x0118

<4>[    0.000000] MUX: setup AF4_34XX_MMC2_DAT3 (0xfa002162): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AE22_34XX_GPIO186_OUT (0xfa0021e2): 0x010f -> 0x0004
<4>[    0.000000] MUX: setup J8_3430_GPIO65 (0xfa0020d2): 0x011f -> 0x0104
<4>[    0.000000] MUX: setup J25_34XX_HDQ_SIO (0xfa0021c6): 0x011f -> 0x0100
<4>[    0.000000] MUX: setup T3_34XX_GPIO179 (0xfa0021d8): 0x010f -> 0x0004

<4>[    0.000000] MUX: setup AF21_34XX_GPIO8_OUT (0xfa002a16): 0x0100 -> 0x0004
<4>[    0.000000] MUX: setup W7_34XX_GPIO178_DOWN (0xfa0021d6): 0x010f -> 0x010c
<4>[    0.000000] MUX: setup N28_34XX_MMC1_CLK (0xfa002144): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup M27_34XX_MMC1_CMD (0xfa002146): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup N27_34XX_MMC1_DAT0 (0xfa002148): 0x010f -> 0x0118

<4>[    0.000000] MUX: setup N26_34XX_MMC1_DAT1 (0xfa00214a): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup N25_34XX_MMC1_DAT2 (0xfa00214c): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup P28_34XX_MMC1_DAT3 (0xfa00214e): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup P27_34XX_MMC1_DAT4 (0xfa002150): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup P26_34XX_MMC1_DAT5 (0xfa002152): 0x010f -> 0x0118

<4>[    0.000000] MUX: setup R27_34XX_MMC1_DAT6 (0xfa002154): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup R25_34XX_MMC1_DAT7 (0xfa002156): 0x010f -> 0x0118
<4>[    0.000000] MUX: setup AB1_34XX_GPIO176_OUT (0xfa0021d2): 0x401c -> 0x0004
<4>[    0.000000] MUX: setup AE15_34XX_I2C2_SDA (0xfa0021c0): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AF15_34XX_I2C2_SCL (0xfa0021be): 0x011f -> 0x0118

<4>[    0.000000] MUX: setup AG14_34XX_I2C3_SDA (0xfa0021c4): 0x011f -> 0x0118
<4>[    0.000000] MUX: setup AF14_34XX_I2C3_SCL (0xfa0021c2): 0x011f -> 0x0118
<4>[   12.993041] MUX: setup K3_34XX_GPIO43_OUT (0xfa00208c): 0x011c -> 0x0004
<4>[   13.000213] MUX: setup V8_34XX_GPIO53_OUT (0xfa0020b2): 0x011c -> 0x0004
<4>[   13.007476] MUX: setup U8_34XX_GPIO54_OUT (0xfa0020b4): 0x011c -> 0x0004

<4>[   13.014617] MUX: setup T8_34XX_GPIO55_OUT (0xfa0020b6): 0x011c -> 0x0004
<4>[   13.021881] MUX: setup R8_34XX_GPIO56_OUT (0xfa0020b8): 0x011c -> 0x0004
<4>[   13.029144] MUX: setup P8_34XX_GPIO57_OUT (0xfa0020ba): 0x011c -> 0x0004
<4>[   13.036285] MUX: setup N8_34XX_GPIO58_OUT (0xfa0020bc): 0x011c -> 0x0004
<4>[   13.043548] MUX: setup L8_34XX_GPIO63_OUT (0xfa0020ce): 0x011c -> 0x0004

<4>[   13.050689] MUX: setup AB2_34XX_GPIO177 (0xfa0021d4): 0x011f -> 0x0104

*/


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


#define OLD_MODEM_CONTROL
#ifdef OLD_MODEM_CONTROL
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

	//omap_mux_init_gpio(SHOLES_BP_READY2_AP_GPIO,OMAP_PIN_INPUT_PULLDOWN);

	gpio_request(SHOLES_BP_RESOUT_GPIO, "BP Reset Output");
	gpio_direction_input(SHOLES_BP_RESOUT_GPIO);
	//omap_cfg_reg(AE3_34XX_GPIO139_DOWN);
	//omap_mux_init_gpio(SHOLES_BP_RESOUT_GPIO,OMAP_PIN_INPUT_PULLDOWN);

	gpio_request(SHOLES_BP_PWRON_GPIO, "BP Power On");
	gpio_direction_output(SHOLES_BP_PWRON_GPIO, 0);
	//omap_cfg_reg(AH3_34XX_GPIO137_OUT);
	//omap_mux_init_gpio(SHOLES_BP_PWRON_GPIO,OMAP_PIN_OUTPUT);

	gpio_request(SHOLES_AP_TO_BP_PSHOLD_GPIO, "AP to BP PS Hold");
	gpio_direction_output(SHOLES_AP_TO_BP_PSHOLD_GPIO, 0);
	//omap_cfg_reg(AF3_34XX_GPIO138_OUT);
	//omap_mux_init_gpio(SHOLES_AP_TO_BP_PSHOLD_GPIO,OMAP_PIN_OUTPUT);

	return platform_device_register(&omap_mdm_ctrl_platform_device);
}
#endif

/* for sdio wl1271 */
static void __init config_mmc2_init(void)
{
	u32 val;

	/* MMC2 */
	//omap_mux_init_signal("mmc2_clk",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("mmc2_cmd",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("mmc2_dat0",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("mmc2_dat1",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("mmc2_dat2",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("mmc2_dat3",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("sys_nirq",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP );
	//omap_mux_init_signal("sys_nirq",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP ); 

	/* Set internal loopback clock */
	val = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1); 
	omap_ctrl_writel((val | OMAP2_MMCSDIO2ADPCLKISEL),
				OMAP343X_CONTROL_DEVCONF1);
}

static void power_modem(void)
{
	int ret=0, i=0;
	ret =	gpio_request(SHOLES_BP_READY2_AP_GPIO, "BP Flash Ready");
	printk("%d %s\n",ret,"req flash ready");
	ret =	gpio_direction_input(SHOLES_BP_READY2_AP_GPIO);
	printk("%d %s\n",ret,"dir flash ready");
	ret =	omap_mux_init_gpio(SHOLES_BP_READY2_AP_GPIO,OMAP_PIN_INPUT_PULLDOWN);
	printk("%d %s\n",ret,"mux flash ready");

	ret =	gpio_request(SHOLES_BP_RESOUT_GPIO, "BP Reset Output");
	printk("%d %s\n",ret,"req reset out");
	ret =	gpio_direction_input(SHOLES_BP_RESOUT_GPIO);
	printk("%d %s\n",ret,"dir reset out");
	ret =	omap_mux_init_gpio(SHOLES_BP_RESOUT_GPIO,OMAP_PIN_INPUT_PULLDOWN);
	printk("%d %s\n",ret,"mux reset out");

	ret =	gpio_request(SHOLES_BP_PWRON_GPIO, "BP Power On");
	printk("%d %s\n",ret,"req power on");
	ret =	gpio_direction_output(SHOLES_BP_PWRON_GPIO, 0);
	printk("%d %s\n",ret,"dir power on");
	ret =	omap_mux_init_gpio(SHOLES_BP_PWRON_GPIO,OMAP_PIN_OUTPUT);
	printk("%d %s\n",ret,"mux power on");

	ret =	gpio_request(SHOLES_AP_TO_BP_PSHOLD_GPIO, "AP to BP PS Hold");
	printk("%d %s\n",ret,"req ps hold");
	ret =	gpio_direction_output(SHOLES_AP_TO_BP_PSHOLD_GPIO, 0);
	printk("%d %s\n",ret,"dir ps hold");
	ret =	omap_mux_init_gpio(SHOLES_AP_TO_BP_PSHOLD_GPIO,OMAP_PIN_OUTPUT);
	printk("%d %s\n",ret,"mux ps hold");

	ret =	gpio_request(SHOLES_AP_TO_BP_FLASH_EN_GPIO, "BP Flash En");
	printk("%d %s\n",ret,"req flash en");
	ret =	gpio_direction_output(SHOLES_AP_TO_BP_FLASH_EN_GPIO, 0);
	printk("%d %s\n",ret,"dir flash en");
	ret =	omap_mux_init_gpio(SHOLES_AP_TO_BP_FLASH_EN_GPIO,OMAP_PIN_OUTPUT);
	printk("%d %s\n",ret,"mux flash en");



	ret = gpio_direction_output(SHOLES_AP_TO_BP_FLASH_EN_GPIO, 0);

	printk("powering up modem \n");
	ret=1;
	/* Press modem Power Button */
	gpio_direction_output(SHOLES_BP_PWRON_GPIO, 1);
	mdelay(100);
	gpio_direction_output(SHOLES_BP_PWRON_GPIO, 0);
	/* Wait up to 5 seconds for the modem to power up */
	for (i = 0; i < 10; i++) {
		if (gpio_get_value(SHOLES_BP_RESOUT_GPIO)) {
			printk("modem powered up.\n");
			ret = 0;
			break;
		}
		mdelay(500);
	}
	if (ret!=0)
		printk ("modem power fail\n");

}



static void __init omap_mapphone_init(void)
{

	//omap3_mux_init(board_mux, OMAP_PACKAGE_CBB); //needed for mux funcs to work

	/* Configure BP wake - flash mode gpios.
	 * both types of modem control will need this,
	 * otherwise the modem always powers up in flash mode.
	 */
	/*
	omap_mux_init_gpio(SHOLES_BPWAKE_STROBE_GPIO,OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(SHOLES_BP_RESOUT_GPIO,OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_gpio(SHOLES_BP_READY2_AP_GPIO,OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_gpio(SHOLES_BP_RESOUT_GPIO,OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_gpio(SHOLES_BP_PWRON_GPIO,OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(SHOLES_AP_TO_BP_PSHOLD_GPIO,OMAP_PIN_OUTPUT);
	mapphone_mdm_ctrl_init(); */

	//sholes_omap_mdm_ctrl_init();


	mapphone_power_off_init();
	/*
	* This will allow unused regulator to be shutdown. This flag
	* should be set in the board file. Before regulators are registered.
	*/
	//regulator_has_full_constraints();
	//omap_mux_init_signal("sys_nirq",OMAP_MUX_MODE4 | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_PIN_INPUT_PULLDOWN );

	omap_serial_init();
	mapphone_bp_model_init();
	mapphone_voltage_init();
	mapphone_gpio_mapping_init();
	mapphone_i2c_init();
	mapphone_padconf_init();
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
	//usb_musb_init(NULL);
	mapphone_musb_init();
	mapphone_usbhost_init();

	config_mmc2_init(); //setup mux and loopback clock before probe
	mapphone_hsmmc_init();
	gpmc_nand_init(&board_nand_data);
	mapphone_wifi_init();
	omap_enable_smartreflex_on_init();
	mapphone_create_board_props();
	mapphone_gadget_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	//omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	power_modem();
}

static void __init mapphone_reserve(void)
{
	//omap2_ramconsole_reserve_sdram();
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
