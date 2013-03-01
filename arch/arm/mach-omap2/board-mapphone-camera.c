#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <plat/i2c.h>
#include <linux/delay.h>
#include <media/mt9p012.h>
#include "devices.h"
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <plat/omap-pm.h>

#if defined(CONFIG_VIDEO_OMAP3)
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#include <../drivers/media/video/isp/isp.h>
#include <../drivers/media/video/isp/ispcsi2.h>
//#include <../drivers/media/video/omap3isp/isp.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif

#define GPIO_MT9P012_RESET		98

static void sholes_camera_lines_safe_mode(void);
static void sholes_camera_lines_func_mode(void);
static enum v4l2_power previous_power = V4L2_POWER_OFF;
static struct regulator *regulator;
static struct regulator *reg_vio;

static struct pm_qos_request_list pm_qos_handler;
#define SET_MPU_CONSTRAINT         12
#define CLEAR_MPU_CONSTRAINT       -1

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
static int hplens_power_set(enum v4l2_power power)
{
	(void)power;

	return 0;
}

static int hplens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 2;
	hwc->dev_minor = 5;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

struct hplens_platform_data mapphone_hplens_platform_data = {
	.power_set = hplens_power_set,
	.priv_data_set = hplens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static struct omap34xxcam_sensor_config mt9p012_cam_hwc = {
	.sensor_isp = 0,
	//.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
	.isp_if = ISP_PARLL,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;
	//hwc->u.sensor.xclk = mt9p012_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = mt9p012_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = mt9p012_cam_hwc.capture_mem;
	hwc->dev_index = 2;
	hwc->dev_minor = 5;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_hs_vs = 0, //TODO what's it for? do we need it?
	//.wait_bayer_frame = 0,
	//.wait_yuv_frame = 1,
	//.dcsub = 42,
	.cam_mclk = 144000000,
	//.cam_mclk_src_div = 6,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,

};

# if 0
u32 mt9p012_set_xclk(u32 xclkfreq)
{
	return isp_set_xclk(xclkfreq, OMAP34XXCAM_XCLK_A);
}
#endif

#define MT9P012_XCLK_48MHZ		48000000

static int mt9p012_sensor_power_set(struct v4l2_int_device *s,
						enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);
	//static struct regulator *regulator;
	int error = 0;
	switch (power) {
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "mt9p012_sensor_power_set(%s)\n",
			(power == V4L2_POWER_OFF) ? "OFF" : "STANDBY");
		/* Power Down Sequence */
		gpio_direction_output(GPIO_MT9P012_RESET, 0);
		gpio_free(GPIO_MT9P012_RESET);
		isp_set_xclk(vdev->cam->isp, 0, 0);

		/* Turn off power */
		if (regulator != NULL) {
			regulator_disable(regulator);
			regulator_put(regulator);
			regulator = NULL;
		} else {
			sholes_camera_lines_safe_mode();
			pr_err("%s: Regulator for vcam is not "\
					"initialized\n", __func__);
			return -EIO;
		}

		/* Remove pm constraints */
		omap_pm_set_min_bus_tput(vdev->cam->isp,
						OCP_INITIATOR_AGENT, 0);
		pm_qos_update_request(&pm_qos_handler,
							CLEAR_MPU_CONSTRAINT);

		/* Make sure not to disable the MCLK twice in a row */
		if (previous_power == V4L2_POWER_ON)
			isp_disable_mclk(isp);

		sholes_camera_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			printk(KERN_DEBUG "mt9p012_sensor_power_set(ON)\n");

			sholes_camera_lines_func_mode();

			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 885735);

			/* Hold a constraint to keep MPU in C1 */
			pm_qos_update_request(&pm_qos_handler,
								SET_MPU_CONSTRAINT);
			/* Configure ISP */
			isp_configure_interface(vdev->cam->isp,&mt9p012_if_config);

			/* Request and configure gpio pins */
			if (gpio_request(GPIO_MT9P012_RESET,
						"mt9p012 camera reset") != 0) {
				error = -EIO;
				goto out;
			}

			/* set to output mode */
			gpio_direction_output(GPIO_MT9P012_RESET, 0);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);

			/* turn on digital power */
			if (regulator != NULL) {
				pr_warning("%s: Already have "\
						"regulator\n", __func__);
			} else {
				regulator = regulator_get(NULL, "vcam");
				if (IS_ERR(regulator)) {
					pr_err("%s: Cannot get vcam "\
						"regulator, err=%ld\n",
						__func__, PTR_ERR(regulator));
					error = PTR_ERR(regulator);
					goto out;
				}
			}

			if (regulator_enable(regulator) != 0) {
				pr_err("%s: Cannot enable vcam regulator\n",
						__func__);
				error = -EIO;
				goto out;
			}
		}

		isp_set_xclk(vdev->cam->isp, 48000000, 0);
		msleep(3);
		//udelay(1000);

		if (previous_power == V4L2_POWER_OFF) {
			/* trigger reset */
			gpio_direction_output(GPIO_MT9P012_RESET, 0);

			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);

			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
			 * enough
			 */
			udelay(300);
		}
		break;
out:
		isp_set_xclk(vdev->cam->isp, 0, 0);
		/* Remove pm constraints */
		omap_pm_set_min_bus_tput(vdev->cam->isp,
						OCP_INITIATOR_AGENT, 0);
		pm_qos_update_request(&pm_qos_handler,
							CLEAR_MPU_CONSTRAINT);
		sholes_camera_lines_safe_mode();
		return error;
	case V4L2_POWER_STANDBY:
		/* Stand By Sequence */
		printk("V4L2_POWER_STANDBY\n");
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

#if 0
static void sholes_lock_cpufreq(int lock)
{
	static struct device ov_dev;
	static int flag;

	if (lock == 1) {
		resource_request("vdd1_opp",
			&ov_dev, omap_pm_get_max_vdd1_opp());
		flag = 1;
	} else {
		if (flag == 1) {
			resource_release("vdd1_opp", &ov_dev);
			flag = 0;
		}
	}
}
#endif

static u8 sholes_get_config_flags(void)
{
	return 0;
}


struct mt9p012_platform_data mapphone_mt9p012_platform_data = {
	.power_set       = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	//.lock_cpufreq = sholes_lock_cpufreq,
	.get_config_flags = sholes_get_config_flags,
	.csi2_lane_count        = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update       = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id    = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update       = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0  = isp_csi2_calc_phy_cfg0,
};

#endif /* #ifdef CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */

/* We can't change the IOMUX config after bootup
 * with the current pad configuration architecture,
 * the next two functions are hack to configure the
 * camera pads at runtime to save power in standby */

void sholes_camera_lines_safe_mode(void)
{
	omap_writew(0x0007, 0x4800210c);
	omap_writew(0x0007, 0x4800210e);
	omap_writew(0x0007, 0x48002110);
	omap_writew(0x0007, 0x48002112);
	omap_writew(0x0007, 0x48002114);
	omap_writew(0x011F, 0x4800211a);
	omap_writew(0x011F, 0x4800211c);
	omap_writew(0x011F, 0x4800211e);
	omap_writew(0x011F, 0x48002120);
	omap_writew(0x001F, 0x48002122);
	omap_writew(0x001F, 0x48002124);
	omap_writew(0x001F, 0x48002126);
	omap_writew(0x001F, 0x48002128);
	omap_writew(0x011F, 0x4800212a);
	omap_writew(0x011F, 0x4800212c);
}

void sholes_camera_lines_func_mode(void)
{
	omap_writew(0x0118, 0x4800210c);
	omap_writew(0x0118, 0x4800210e);
	omap_writew(0x0000, 0x48002110);
	omap_writew(0x0118, 0x48002112);
	omap_writew(0x0004, 0x48002114);
	omap_writew(0x0100, 0x4800211a);
	omap_writew(0x0100, 0x4800211c);
	omap_writew(0x0100, 0x4800211e);
	omap_writew(0x0100, 0x48002120);
	omap_writew(0x0100, 0x48002122);
	omap_writew(0x0100, 0x48002124);
	omap_writew(0x0100, 0x48002126);
	omap_writew(0x0100, 0x48002128);
	omap_writew(0x0100, 0x4800212a);
	omap_writew(0x0100, 0x4800212c);
}


static void __init mapphone_camera_init (void)
{
	regulator = regulator_get(NULL, "vcam");
		if (IS_ERR(regulator))
			pr_err("%s: cannot get vcam regulator\n", __func__);
		else
			regulator_enable(regulator);

	/*BANG ON MUX REGS DIRECTLY.*/
	omap_writew(0x0118, 0x4800210c); //CAM_HS , MUX_MODE0 , INPUT_PULLUP 
	omap_writew(0x0118, 0x4800210e); //CAM_VS

	omap_writew(0x0000, 0x48002110); //XCLKA, MODE0, OUTPUT
	omap_writew(0x0118, 0x48002112); //CAM_PCLK , MUX_MODE0 , INPUT_PULLUP

	omap_writew(0x0004, 0x48002114); // CAM_FLD, MODE4, OUTPUT (GPIO 98 CAM RESET)
	//16 is ts gpio
	omap_writew(0x0104, 0x48002118); // cam_d1, mode4 input (gpio 100)
	omap_writew(0x0100, 0x4800211a); // cam_d2 

	omap_writew(0x0100, 0x4800211c); //...
	omap_writew(0x0100, 0x4800211e);

	omap_writew(0x0100, 0x48002120);
	omap_writew(0x0100, 0x48002122);

	omap_writew(0x0100, 0x48002124);
	omap_writew(0x0100, 0x48002126);

	omap_writew(0x0100, 0x48002128);
	omap_writew(0x0100, 0x4800212a); //...

	omap_writew(0x0100, 0x4800212c); //cam_d11

	omap_writew(0x0002, 0x48002130); //cam_wen, mode2 "cam_shutter"
	omap_writew(0x0000, 0x48002132); //cam_strobe

	omap_writew(0x0004, 0x480020d0); //gpmc wait2 mode4 output (gpio 64)

	//sholes_camera_lines_safe_mode();

	struct clk *mclk;
	unsigned int rate;
	int err = 0;

#if 0
	mclk = clk_get(NULL, "dpll4_m5_ck");
	if (IS_ERR(mclk))
		printk("couldn't get dpll4_m5_ck\n");
	else
		rate = clk_get_rate(mclk);
	printk("mclk rate %lu hz\n", rate);
	err = clk_set_rate(mclk, 72000000); //192mhz is an even multiple of 48mhz, our target xclk rate
	if (err)
		printk("error setting dpll4_m5_ck\n");
	clk_enable(mclk);
	mclk = clk_get(NULL, "cam_mclk");
	clk_enable(mclk);

	gpio_request(64, "cam_pwrdn");
	gpio_direction_output(64,0);//TODO 
#endif
	gpio_request(98, "cam_rst");
	gpio_direction_output(98,0); 


}
late_initcall(mapphone_camera_init);
