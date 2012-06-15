/* arch/arm/mach-msm/board-shooter-camera.c
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2012 The CyanogenMod Project.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/msm_flashlight.h>

// Common Camera Resources
static struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static void config_gpio_table(uint32_t *table, int len);

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_CAM_I2C_SCL,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_SP3D_MCLK,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(SHOOTER_SP3D_INT,	 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/*sharp INT*/
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,	 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(SHOOTER_WEBCAM_STB,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(SHOOTER_CAM_SEL,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_MCLK,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(SHOOTER_SP3D_INT,	 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,	 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK,	 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_STB,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_SEL,	 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int shooter_config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));

	return 0;
}

static int camera_sensor_power_enable(char *power, unsigned volt)
{
	struct regulator *sensor_power;
	int rc;

	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	rc = regulator_set_voltage(sensor_power, volt, volt);
	if (rc)
		pr_err("[CAM] %s: unable to set %s voltage to %d rc:%d\n",
			__func__, power, volt, rc);

	rc = regulator_enable(sensor_power);
	if (rc)
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);

	regulator_put(sensor_power);
	return rc;
}

static int camera_sensor_power_enable_8901(char *power)
{
	struct regulator *sensor_power;
	int rc;

	pr_info("%s %s",__func__,power);
	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	rc = regulator_enable(sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);
		regulator_put(sensor_power);
		return -ENODEV;
	}
	regulator_put(sensor_power);
	return rc;
}

static int camera_sensor_power_disable(char *power)
{
	struct regulator *sensor_power;
	int rc;

	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	rc = regulator_disable(sensor_power);
	if (rc)
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);

	regulator_put(sensor_power);
	return rc;
}

#ifdef CONFIG_SP3D
static uint32_t sp3d_spi_gpio[] = {
	/* or this? the i/o direction and up/down are much more correct */
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};

static int shooter_sp3d_vreg_on(void)
{
	int rc;

	pr_info("[CAM] %s\n", __func__);
	/* VDDIO*/
	if ((system_rev == 2 && engineerid >= 3) || system_rev == 0x80 ) { /*VERSION A*/
		rc = camera_sensor_power_enable_8901("8901_lvs2");
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
		udelay(26);
		/*DVDD18 */
		rc = camera_sensor_power_enable_8901("8901_lvs3");
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
		udelay(26);
		/*DVDD18 */
		rc = camera_sensor_power_enable("8058_l9", 1800000);
		pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
	}

	gpio_set_value(SHOOTER_SP3D_CORE_GATE, 1); // CORE GATE
	gpio_set_value(SHOOTER_SP3D_SYS_RST, 1); // RST
	gpio_set_value(SHOOTER_SP3D_PDX, 1); // PDX
	gpio_set_value(SHOOTER_SP3D_GATE, 1); // GATE

	/* main camera AVDD */
	rc = camera_sensor_power_enable("8058_l15", 2800000);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2800) == %d\n", rc);
	/* main camera MVDD */
	rc = camera_sensor_power_enable("8058_l10", 2800000);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2800) == %d\n", rc);

	gpio_tlmm_config(sp3d_spi_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[3], GPIO_CFG_ENABLE);

	return rc;
}

static int shooter_sp3d_vreg_off(void)
{
	int rc;

	pr_info("[CAM] %s\n", __func__);

	gpio_set_value(SHOOTER_SP3D_PDX, 0); // PDX
	/* main camera MVDD */
	rc = camera_sensor_power_disable("8058_l10");
	udelay(10);

	//according to logic Jason Kao, only tutn off l15 when sharp
	if (!(engineerid == 7) || (system_rev == 0x80 && engineerid == 0x1)) {
	/* main camera AVDD */
		rc = camera_sensor_power_disable("8058_l15");
		udelay(10);
	}

	/* main camera DVDD18 */
	if ((system_rev == 2 && engineerid >= 3) || system_rev == 0x80)
		rc = camera_sensor_power_disable("8901_lvs3");
	else
		rc = camera_sensor_power_disable("8058_l9");

	gpio_set_value(SHOOTER_SP3D_SYS_RST, 0); // RST
	gpio_set_value(SHOOTER_SP3D_CORE_GATE, 0); // CORE GATE
	gpio_set_value(SHOOTER_SP3D_GATE, 0); // GATE

	/*VDDIO*/
	if ((system_rev == 2 && engineerid >= 3) || system_rev == 0x80)
		rc = camera_sensor_power_disable("8901_lvs2");
	else
		rc = camera_sensor_power_disable("8058_l8");

	return rc;
}

static void shooter_config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

	gpio_set_value(SHOOTER_SP3D_SPI_DO, 0);
	gpio_set_value(SHOOTER_SP3D_SPI_CS, 0);
	gpio_set_value(SHOOTER_SP3D_SPI_CLK, 0);
	gpio_set_value(SHOOTER_SP3D_MCLK, 0);
	gpio_set_value(SHOOTER_CAM_SEL, 0);
}

static void shooter_maincam_clk_switch(void)
{
	pr_info("[CAM] Doing clk switch (Main Cam)\n");
	gpio_set_value(SHOOTER_CAM_SEL, 0);
}

static struct msm_camera_device_platform_data msm_camera_device_data_sp3d = {
	.camera_gpio_on		= shooter_config_camera_on_gpios,
	.camera_gpio_off	= shooter_config_camera_off_gpios,
	.ioext.csiphy		= 0x04800000,
	.ioext.csisz		= 0x00000400,
	.ioext.csiirq		= CSI_0_IRQ,
	.ioclk.mclk_clk_rate	= 24000000,
	.ioclk.vfe_clk_rate	= 228570000,
};

static int flashlight_control(int mode)
{
	return aat1277_flashlight_control(mode);
}

static struct spi_board_info sp3d_spi_board_info[] __initdata = {
	{
		.modalias	= "sp3d_spi",
		.mode		= SPI_MODE_3,
#ifdef CONFIG_MACH_SHOOTER
		.bus_num	= 1,
#else
		.bus_num	= 2,
#endif
		.chip_select	= 0,
		.max_speed_hz	= 15060000,
	}
};

static struct msm_camera_sensor_flash_src msm_flash_src_sp3d = {
	.flash_sr_type		= MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash		= flashlight_control,
};

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.low_temp_limit		= 10,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_flash_data flash_sp3d = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src_sp3d
};

static struct msm_camera_sensor_info msm_camera_sensor_sp3d_data = {
	.sensor_name		= "sp3d",
	.vcm_enable		= 0,
	.camera_power_on	= shooter_sp3d_vreg_on,
	.camera_power_off	= shooter_sp3d_vreg_off,
	.camera_clk_switch	= shooter_maincam_clk_switch,
	.pdata			= &msm_camera_device_data_sp3d,
	.resource		= msm_camera_resources,
	.num_resources		= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_sp3d,
	.flash_cfg		= &msm_camera_sensor_flash_cfg,
	.stereo_low_cap_limit	= 15,
	.mirror_mode		= 0,
	.csi_if			= 1,
	.dev_node		= 0,
};

struct platform_device msm_camera_sensor_sp3d = {
	.name	= "msm_camera_sp3d",
	.dev	= {
		.platform_data = &msm_camera_sensor_sp3d_data,
	},
};

static void __init sp3d_init_camera(void)
{
	// Nothing to do for this device
}
#endif

#ifdef CONFIG_MSM_CAMERA_FLASH
static void config_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		GPIO_CFG(SHOOTER_TORCH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_FLASH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};

	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data flashlight_data = {
	.gpio_init		= config_flashlight_gpios,
	.torch			= SHOOTER_TORCH_EN,
	.flash			= SHOOTER_FLASH_EN,
	.torch_set1		= PM8058_GPIO_PM_TO_SYS(SHOOTER_TORCH_SET1),
	.torch_set2		= PM8058_GPIO_PM_TO_SYS(SHOOTER_TORCH_SET2),
	.flash_duration_ms	= 600,
	.chip_model		= AAT1277,
};

static struct platform_device flashlight_device = {
	.name	= FLASHLIGHT_NAME,
	.dev	= {
		.platform_data = &flashlight_data,
	},
};
#endif

#ifdef CONFIG_S5K6AAFX
static int shooter_s5k6aafx_vreg_on(void)
{
	int rc;

	pr_info("[CAM] %s\n", __func__);

	/* main / 2nd camera analog power */
#ifdef CONFIG_MACH_SHOOTER
	rc = camera_sensor_power_enable("8058_l3", 2850000);
	pr_info("[CAM] sensor_power_enable(\"8058_l3\", 2850) == %d\n", rc);
#else
	rc = camera_sensor_power_enable("8901_l6", 2850000);
	pr_info("[CAM] sensor_power_enable(\"8901_l6\", 2850) == %d\n", rc);
#endif
	mdelay(5);

	/* main / 2nd camera digital power */
	rc = camera_sensor_power_enable_8901("8901_lvs2");
	pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
	mdelay(5);

	/*IO*/
	rc = camera_sensor_power_enable_8901("8901_lvs3");
	pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
	mdelay(1);
	return rc;
}

static int shooter_s5k6aafx_vreg_off(void)
{
	int rc;

	pr_info("[CAM] %s\n", __func__);

	/* IO power off */
	rc = camera_sensor_power_disable("8901_lvs3");
	pr_info("[CAM] sensor_power_disable(\"8901_lvs3\") == %d\n", rc);
	mdelay(1);

	/* main / 2nd camera digital power */
	rc = camera_sensor_power_disable("8901_lvs2");
	pr_info("[CAM] sensor_power_disable(\"8901_lvs2\") == %d\n", rc);
	mdelay(1);

	/* main / 2nd camera analog power */
#ifdef CONFIG_MACH_SHOOTER
	rc = camera_sensor_power_disable("8058_l3");
	pr_info("[CAM] sensor_power_disable(\"8058_l3\") == %d\n", rc);
#else
	rc = camera_sensor_power_disable("8901_l6");
	pr_info("[CAM] sensor_power_disable(\"8901_l6\") == %d\n", rc);
#endif
	return rc;
}

static void shooter_seccam_clk_switch(void)
{
	pr_info("[CAM] Doing clk switch (2nd Cam)\n");
	gpio_set_value(SHOOTER_CAM_SEL, 1);
}

static struct msm_camera_sensor_flash_data flash_s5k6aafx = {
	.flash_type		= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_device_platform_data msm_camera_device_data_s5k6aafx = {
	.camera_gpio_on		= shooter_config_camera_on_gpios,
	.camera_gpio_off	= shooter_config_camera_off_gpios,
	.ioext.csiphy		= 0x04900000,
	.ioext.csisz		= 0x00000400,
	.ioext.csiirq		= CSI_1_IRQ,
	.ioclk.mclk_clk_rate	= 24000000,
	.ioclk.vfe_clk_rate	= 228570000,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name		= "s5k6aafx",
	.sensor_reset		= SHOOTER_WEBCAM_RST,/*2nd Cam RST*/
	.sensor_pwd		= SHOOTER_WEBCAM_STB,/*2nd Cam PWD*/
	.vcm_enable		= 0,
	.camera_power_on	= shooter_s5k6aafx_vreg_on,
	.camera_power_off	= shooter_s5k6aafx_vreg_off,
	.camera_clk_switch	= shooter_seccam_clk_switch,
	.pdata			= &msm_camera_device_data_s5k6aafx, /* Fro front CAM */
	.resource		= msm_camera_resources,
	.num_resources		= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_s5k6aafx,
	.mirror_mode		= 0,
	.csi_if			= 1,
	.dev_node		= 1,
};

struct platform_device msm_camera_sensor_s5k6aafx;

static void __init s5k6aafx_init_camera(void)
{
	msm_camera_sensor_s5k6aafx.name = "msm_camera_webcam";
	msm_camera_sensor_s5k6aafx.dev.platform_data = &msm_camera_sensor_s5k6aafx_data;
}

static struct i2c_board_info msm_s5k6aafx_camera_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5a >> 1), /* COB type */
	},
};
#endif
