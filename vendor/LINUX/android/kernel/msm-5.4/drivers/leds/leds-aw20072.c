/*
 * leds-aw20072.c   aw20072 led module
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Joseph <zhangzetao@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/leds.h>
#include "leds-aw20072.h"
#include "leds-aw20072-reg.h"
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW20072_I2C_NAME "aw20072_led"

#define AW20072_DRIVER_VERSION "V1.5.0"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

struct pinctrl *pinctrl_aw20072;
struct pinctrl_state *aw20072_pins_cfg, *aw20072_rst_output0,
    *aw20072_rst_output1;
unsigned int aw20072debounce;

const struct of_device_id aw20072_of_match[] = {
	{.compatible = "awinic,aw20072_led",},
	{},
};

/******************************************************
 *
 * aw20072 led parameter
 *
 ******************************************************/
#define AW20072_CFG_NAME_MAX        64
#if defined(AW20072_BIN_CONFIG)
static char aw20072_cfg_name[][AW20072_CFG_NAME_MAX] = {
	{"aw20072_led_all_on.bin"},
	{"aw20072_led_red_on.bin"},
	{"aw20072_led_green_on.bin"},
	{"aw20072_led_blue_on.bin"},
	{"aw20072_led_breath_forever.bin"},
	{"aw20072_cfg_led_off.bin"},
};
#elif defined(AW20072_ARRAY_CONFIG)
AW20072_CFG aw20072_cfg_array[] = {
	{aw20072_led_all_on, sizeof(aw20072_led_all_on)}
	,
	{aw20072_led_red_on, sizeof(aw20072_led_red_on)}
	,
	{aw20072_led_green_on, sizeof(aw20072_led_green_on)}
	,
	{aw20072_led_blue_on, sizeof(aw20072_led_blue_on)}
	,
	{aw20072_led_breath_forever, sizeof(aw20072_led_breath_forever)}
	,
	{aw20072_cfg_led_off, sizeof(aw20072_cfg_led_off)}
};
#else
    /*Nothing */
#endif

#define AW20072_IMAX_NAME_MAX       32
static char aw20072_imax_name[][AW20072_IMAX_NAME_MAX] = {
	{"AW20072_IMAX_10mA"},
	{"AW20072_IMAX_20mA"},
	{"AW20072_IMAX_30mA"},
	{"AW20072_IMAX_40mA"},
	{"AW20072_IMAX_60mA"},
	{"AW20072_IMAX_80mA"},
	{"AW20072_IMAX_120mA"},
	{"AW20072_IMAX_160mA"},
	{"AW20072_IMAX_3P3mA"},
	{"AW20072_IMAX_6P7mA"},
	{"AW20072_IMAX_10P0mA"},
	{"AW20072_IMAX_13P3mA"},
	{"AW20072_IMAX_20P0mA"},
	{"AW20072_IMAX_26P7mA"},
	{"AW20072_IMAX_40P0mA"},
	{"AW20072_IMAX_53P3mA"},
};

/******************************************************
 *
 * aw20072 i2c write/read
 *
 ******************************************************/
static int aw20072_i2c_write(struct aw20072 *aw20072,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw20072->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw20072_i2c_read(struct aw20072 *aw20072,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw20072->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw20072_i2c_write_bits(struct aw20072 *aw20072,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	unsigned char reg_val;

	aw20072_i2c_read(aw20072, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw20072_i2c_write(aw20072, reg_addr, reg_val);

	return 0;
}

/*****************************************************
 *
 * aw20072 led cfg
 *
 *****************************************************/
static int aw20072_reg_page_cfg(struct aw20072 *aw20072, unsigned char page)
{
	aw20072_i2c_write(aw20072, REG_PAGE, page);
	return 0;
}

static int aw20072_imax_cfg(struct aw20072 *aw20072, unsigned char imax)
{
	if (imax > 0xF)
		imax = 0xF;

	aw20072_reg_page_cfg(aw20072, AW20072_REG_PAGE0);
	aw20072_i2c_write_bits(aw20072, REG_GCCR, BIT_IMAX_MASK, imax << 4);

	return 0;
}

static int aw20072_dbgdim_cfg(struct aw20072 *aw20072, unsigned int data)
{
	int i;

	aw20072_i2c_write(aw20072, 0xF0, 0xC4);
	for (i = 0; i < AW20072_REG_NUM_PAG4; i = i + 2) {
		aw20072->rgbcolor = data;
		aw20072_i2c_write(aw20072, i, aw20072->rgbcolor);
	}
	return 0;
}

static int aw20072_dbgfdad_cfg(struct aw20072 *aw20072, unsigned int data)
{
	int i;

	aw20072_i2c_write(aw20072, 0xF0, 0xC4);
	for (i = 1; i < AW20072_REG_NUM_PAG4; i = i + 2) {
		aw20072->rgbcolor = data;
		aw20072_i2c_write(aw20072, i, aw20072->rgbcolor);
	}

	return 0;
}

static void aw20072_brightness_work(struct work_struct *work)
{
	struct aw20072 *aw20072 = container_of(work, struct aw20072,
					       brightness_work);

	pr_info("%s: enter\n", __func__);

	if (aw20072->cdev.brightness > aw20072->cdev.max_brightness)
		aw20072->cdev.brightness = aw20072->cdev.max_brightness;

	if (aw20072->cdev.brightness) {
		aw20072_i2c_write(aw20072, 0xF0, 0xC0);
		aw20072_i2c_write(aw20072, 0x01, 0x00);
		aw20072_dbgdim_cfg(aw20072, AW20072_DBGCTR_DIM);
		aw20072_dbgfdad_cfg(aw20072, aw20072->cdev.brightness);
	} else {
		aw20072_dbgdim_cfg(aw20072, 0x00);
		aw20072_dbgfdad_cfg(aw20072, 0);
		aw20072_i2c_write(aw20072, 0xF0, 0xC0);
		aw20072_i2c_write(aw20072, 0x01, 0x80);
	}
}

static void aw20072_set_brightness(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct aw20072 *aw20072 = container_of(cdev, struct aw20072, cdev);

	aw20072->cdev.brightness = brightness;

	schedule_work(&aw20072->brightness_work);
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
#if defined(AW20072_ARRAY_CONFIG)
static void aw20072_update_cfg_array(struct aw20072 *aw20072,
				     unsigned char *p_cfg_data,
				     unsigned int cfg_size)
{
	unsigned int i = 0;
	unsigned char page = 0;

	for (i = 0; i < cfg_size; i += 2) {
		aw20072_i2c_write(aw20072, p_cfg_data[i], p_cfg_data[i + 1]);
		if (p_cfg_data[i] == 0xf0)
			page = p_cfg_data[i + 1];
		if ((page == AW20072_REG_PAGE0)
		    && (p_cfg_data[i] == REG_SWRST)
		    && (p_cfg_data[i + 1] == 0x01))
			usleep_range(2000, 2500);
	}
}

static int aw20072_cfg_update_array(struct aw20072 *aw20072)
{
	pr_info("%s: enter\n", __func__);

	aw20072_update_cfg_array(aw20072,
				 (aw20072_cfg_array[aw20072->effect].p),
				 aw20072_cfg_array[aw20072->effect].count);
	return 0;
}
#endif
#if defined(AW20072_BIN_CONFIG)
static void aw20072_cfg_loaded(const struct firmware *cont, void *context)
{
	struct aw20072 *aw20072 = context;
	int i = 0;
	unsigned char page = 0;
	unsigned char reg_addr = 0;
	unsigned char reg_val = 0;

	pr_info("%s: enter\n", __func__);

	if (!cont) {
		pr_info("%s: failed to read %s\n", __func__,
			aw20072_cfg_name[aw20072->effect]);
		release_firmware(cont);
		return;
	}
	mutex_lock(&aw20072->cfg_lock);
	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw20072_cfg_name[aw20072->effect], cont ? cont->size : 0);
	for (i = 0; i < cont->size; i += 2) {
		if (*(cont->data + i) == 0xf0)
			page = *(cont->data + i + 1);
		aw20072_i2c_write(aw20072, *(cont->data + i),
				  *(cont->data + i + 1));
		pr_debug("%s: addr:0x%02x, data:0x%02x\n", __func__,
			 *(cont->data + i), *(cont->data + i + 1));

		if (page == AW20072_REG_PAGE0) {
			reg_addr = *(cont->data + i);
			reg_val = *(cont->data + i + 1);
			/* gcr chip enable delay */
			if ((reg_addr == REG_SWRST) && (reg_val == 0x01))
				usleep_range(2000, 2500);
		}
	}

	release_firmware(cont);
	mutex_unlock(&aw20072->cfg_lock);
	pr_info("%s: cfg update complete\n", __func__);

}

static int aw20072_cfg_update(struct aw20072 *aw20072)
{
	int ret;

	pr_info("%s: enter\n", __func__);
	ret = 0;

	if (aw20072->effect < (sizeof(aw20072_cfg_name) / AW20072_CFG_NAME_MAX)) {
		pr_info("%s: cfg name=%s\n", __func__,
			aw20072_cfg_name[aw20072->effect]);
	} else {
		pr_err("%s: effect 0x%02x over s value\n", __func__,
		       aw20072->effect);
		return (-1);
	}

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw20072_cfg_name[aw20072->effect],
				       aw20072->dev, GFP_KERNEL, aw20072,
				       aw20072_cfg_loaded);
}
#endif

static int aw20072_hw_reset(struct aw20072 *aw20072)
{
	pr_info("%s: enter\n", __func__);

	if (aw20072 && gpio_is_valid(aw20072->reset_gpio)) {
		gpio_set_value_cansleep(aw20072->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw20072->reset_gpio, 1);
		usleep_range(2000, 2500);
	} else {
		dev_err(aw20072->dev, "%s:  failed\n", __func__);
	}
	pr_info("%s: enter out\n", __func__);
	return 0;
}

static int aw20072_hw_off(struct aw20072 *aw20072)
{
	pr_info("%s: enter\n", __func__);
	if (aw20072 && gpio_is_valid(aw20072->reset_gpio)) {
		gpio_set_value_cansleep(aw20072->reset_gpio, 0);
		msleep(1);
	} else {
		dev_err(aw20072->dev, "%s:  failed\n", __func__);
	}

	return 0;
}

/******************************************************
 *
 * irq
 *
 ******************************************************/

static irqreturn_t aw20072_irq(int irq, void *data)
{
	struct aw20072 *aw20072 = data;
	unsigned char reg_val;

	pr_info("%s: enter\n", __func__);

	aw20072_i2c_read(aw20072, REG_ISRFLT, &reg_val);
	pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);
	pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw20072_parse_dt(struct device *dev, struct aw20072 *aw20072,
			    struct device_node *np)
{

	aw20072->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw20072->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw20072->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw20072->irq_gpio < 0) {
		dev_err(dev,
			"%s: no irq gpio provided, will not suppport intterupt\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: irq gpio provided ok\n", __func__);
	}

	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw20072_read_chipid(struct aw20072 *aw20072)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	aw20072_reg_page_cfg(aw20072, AW20072_REG_PAGE0);

	while (cnt++ < AW_READ_CHIPID_RETRIES) {
		ret = aw20072_i2c_read(aw20072, REG_CHIPID, &reg_val);
		if (reg_val == AW20072_CHIPID) {
			pr_info("This Chip is  AW20072    REG_ID: 0x%x\n",
				reg_val);
			return 0;
		} else if (ret < 0) {
			dev_err(aw20072->dev,
				"%s: failed to AW20072_REG_ID: %d\n", __func__,
				ret);
		} else {
			pr_info("This Chip    read register   REG_ID: 0x%x\n",
				reg_val);
		}
		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}
	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw20072_reg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw20072_i2c_write(aw20072, databuf[0], databuf[1]);
	return count;
}

static ssize_t aw20072_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned char reg_page = 0;

	aw20072_reg_page_cfg(aw20072, AW20072_REG_PAGE0);
	for (i = 0; i < AW20072_REG_MAX; i++) {
		if (!reg_page) {
			if (!(aw20072_reg_access[i] & REG_RD_ACCESS))
				continue;
		}
		aw20072_i2c_read(aw20072, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw20072_hwen_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] == 1)
			aw20072_hw_reset(aw20072);
		else
			aw20072_hw_off(aw20072);
	}

	return count;
}

static ssize_t aw20072_hwen_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw20072->reset_gpio));

	return len;
}

static ssize_t aw20072_effect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);
#if defined(AW20072_BIN_CONFIG)
	for (i = 0; i < sizeof(aw20072_cfg_name) / AW20072_CFG_NAME_MAX; i++) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %s\n", i,
			     aw20072_cfg_name[i]);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "current cfg = %s\n",
		     aw20072_cfg_name[aw20072->effect]);
#elif defined(AW20072_ARRAY_CONFIG)
	for (i = 0; i < sizeof(aw20072_cfg_array) / sizeof(struct aw20072_cfg);
	     i++) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %pf\n", i,
			     aw20072_cfg_array[i].p);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "current cfg = %pf\n",
		     aw20072_cfg_array[aw20072->effect].p);
#else
	/*Nothing */
#endif
	return len;
}

static ssize_t aw20072_effect_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw20072->effect = databuf[0];
#if defined(AW20072_BIN_CONFIG)
	aw20072_cfg_update(aw20072);
#elif defined(AW20072_ARRAY_CONFIG)
	aw20072_cfg_update_array(aw20072);
#else
	/*Nothing */
#endif

	return len;
}

static ssize_t aw20072_imax_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw20072->imax = databuf[0];

	if (aw20072->imax > 0xF)
		aw20072->imax = 0xF;

	aw20072_imax_cfg(aw20072, aw20072->imax);

	return len;
}

static ssize_t aw20072_imax_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	for (i = 0; i < sizeof(aw20072_imax_name) / AW20072_IMAX_NAME_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"imax[%x] = %s\n", i, aw20072_imax_name[i]);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
		     "current id = 0x%02x, imax = %s\n", aw20072->imax,
		     aw20072_imax_name[aw20072->imax]);

	return len;
}

static ssize_t aw20072_rgbcolor_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	unsigned int databuf[2] = { 0, 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw20072_i2c_write(aw20072, 0xF0, 0xC0);
		aw20072_i2c_write(aw20072, 0x01, 0x00);
		aw20072_i2c_write(aw20072, 0xF0, 0xC1);

		aw20072->rgbcolor = (databuf[1] & 0x00ff0000) >> 16;
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, databuf[0] * 3, aw20072->rgbcolor);

		aw20072->rgbcolor = (databuf[1] & 0x0000ff00) >> 8;
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, databuf[0] * 3 + 1,
				  aw20072->rgbcolor);

		aw20072->rgbcolor = (databuf[1] & 0x000000ff);
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, databuf[0] * 3 + 2,
				  aw20072->rgbcolor);
	}
	return len;
}

static ssize_t aw20072_rgbbrightness_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t len)
{
	unsigned int databuf[2] = { 0, 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw20072_i2c_write(aw20072, 0xF0, 0xC0);
		aw20072_i2c_write(aw20072, 0x01, 0x00);
		aw20072_i2c_write(aw20072, 0xF0, 0xC2);
		aw20072->rgbbrightness = (databuf[1] & 0x00ff0000) >> 16;
		aw20072_i2c_write(aw20072, databuf[0] * 3,
				  aw20072->rgbbrightness);

		aw20072->rgbbrightness = (databuf[1] & 0x0000ff00) >> 8;
		aw20072_i2c_write(aw20072, databuf[0] * 3 + 1,
				  aw20072->rgbbrightness);

		aw20072->rgbbrightness = (databuf[1] & 0x000000ff);
		aw20072_i2c_write(aw20072, databuf[0] * 3 + 2,
				  aw20072->rgbbrightness);
	}
	return len;
}

static ssize_t aw20072_allrgbcolor_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	unsigned int databuf[1];
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw20072->rgbcolor = databuf[0];
	aw20072_i2c_write(aw20072, 0xF0, 0xC0);
	aw20072_i2c_write(aw20072, 0x01, 0x00);
	/*Set pag 1 DIM0-DIM35 */
	aw20072_i2c_write(aw20072, 0xF0, 0xC1);
	for (i = 0; i < AW20072_REG_NUM_PAG1; i += 3) {
		aw20072->rgbcolor = (databuf[0] & 0x00ff0000) >> 16;
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, i, aw20072->rgbcolor);

		aw20072->rgbcolor = (databuf[0] & 0x0000ff00) >> 8;
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, i + 1, aw20072->rgbcolor);

		aw20072->rgbcolor = (databuf[0] & 0x000000ff);
		aw20072->rgbcolor = (aw20072->rgbcolor * 64) / 256;
		aw20072_i2c_write(aw20072, i + 2, aw20072->rgbcolor);
		pr_debug("%s: addr:0x%02x, data:0x%02x\n", __func__, i,
			 aw20072->rgbcolor);
	}
	return len;
}

static ssize_t aw20072_allrgbbrightness_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	unsigned int databuf[2];
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20072 *aw20072 = container_of(led_cdev, struct aw20072, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw20072_i2c_write(aw20072, 0xF0, 0xC0);
	aw20072_i2c_write(aw20072, 0x01, 0x00);
	/*Set pag 2 PAD0-PAD35 */
	aw20072_i2c_write(aw20072, 0xF0, 0xC2);
	for (i = 0; i < AW20072_REG_NUM_PAG2; i += 3) {
		aw20072->rgbbrightness = (databuf[0] & 0x00ff0000) >> 16;
		aw20072_i2c_write(aw20072, i, aw20072->rgbbrightness);

		aw20072->rgbbrightness = (databuf[0] & 0x0000ff00) >> 8;
		aw20072_i2c_write(aw20072, i + 1, aw20072->rgbbrightness);

		aw20072->rgbbrightness = (databuf[0] & 0x000000ff);
		aw20072_i2c_write(aw20072, i + 2, aw20072->rgbbrightness);
		pr_debug("%s: addr:0x%02x, data:0x%02x\n", __func__, i,
			 aw20072->rgbbrightness);
	}
	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw20072_reg_show, aw20072_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw20072_hwen_show,
		   aw20072_hwen_store);
static DEVICE_ATTR(effect, S_IWUSR | S_IRUGO, aw20072_effect_show,
		   aw20072_effect_store);
static DEVICE_ATTR(imax, S_IWUSR | S_IRUGO, aw20072_imax_show,
		   aw20072_imax_store);
static DEVICE_ATTR(rgbcolor, S_IWUSR | S_IRUGO, NULL, aw20072_rgbcolor_store);
static DEVICE_ATTR(rgbbrightness, S_IWUSR | S_IRUGO, NULL,
		   aw20072_rgbbrightness_store);
static DEVICE_ATTR(allrgbcolor, S_IWUSR | S_IRUGO, NULL,
		   aw20072_allrgbcolor_store);
static DEVICE_ATTR(allrgbbrightness, S_IWUSR | S_IRUGO, NULL,
		   aw20072_allrgbbrightness_store);

static struct attribute *aw20072_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_effect.attr,
	&dev_attr_imax.attr,
	&dev_attr_rgbcolor.attr,
	&dev_attr_allrgbcolor.attr,
	&dev_attr_rgbbrightness.attr,
	&dev_attr_allrgbbrightness.attr,
	NULL
};

static struct attribute_group aw20072_attribute_group = {
	.attrs = aw20072_attributes
};

/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw20072_led_init(struct aw20072 *aw20072)
{
	pr_info("%s: enter\n", __func__);

	aw20072_i2c_write(aw20072, 0xF0, 0xC0);
	aw20072_i2c_write(aw20072, 0x02, 0x01);
	msleep(2);
	aw20072_i2c_write(aw20072, 0X03, 0x10);	/*SET default */
	aw20072_imax_cfg(aw20072, aw20072->imax);	/*set imax */
	aw20072_i2c_write_bits(aw20072, REG_GCCR, BIT_ALLON_MASK,
			       BIT_GCR_ALLON_ENABLE);
	aw20072_i2c_write(aw20072, 0x80, 0x05);

	return 0;
}

static int aw20072_parse_led_cdev(struct aw20072 *aw20072,
				  struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw20072,name",
					      &aw20072->cdev.name);
		if (ret < 0) {
			dev_err(aw20072->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20072,imax",
					   &aw20072->imax);
		if (ret < 0) {
			dev_err(aw20072->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20072,brightness",
					   &aw20072->cdev.brightness);
		if (ret < 0) {
			dev_err(aw20072->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20072,max_brightness",
					   &aw20072->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw20072->dev,
				"Failure reading max brightness, ret = %d\n",
				ret);
			goto free_pdata;
		}
	}
	aw20072_led_init(aw20072);
	INIT_WORK(&aw20072->brightness_work, aw20072_brightness_work);
	aw20072->cdev.brightness_set = aw20072_set_brightness;
	ret = led_classdev_register(aw20072->dev, &aw20072->cdev);
	if (ret) {
		dev_err(aw20072->dev, "unable to register led ret=%d\n", ret);
		goto free_pdata;
	}
	ret =
	    sysfs_create_group(&aw20072->cdev.dev->kobj,
			       &aw20072_attribute_group);
	if (ret) {
		dev_err(aw20072->dev, "led sysfs ret: %d\n", ret);
		goto free_class;
	}
	return 0;

 free_class:
	led_classdev_unregister(&aw20072->cdev);
 free_pdata:
	return ret;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw20072_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aw20072 *aw20072;
	struct device_node *np = i2c->dev.of_node;
	int ret;
	int irq_flags;

	pr_info("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw20072 = devm_kzalloc(&i2c->dev, sizeof(struct aw20072), GFP_KERNEL);
	if (aw20072 == NULL)
		return -ENOMEM;

	aw20072->dev = &i2c->dev;
	aw20072->i2c = i2c;

	i2c_set_clientdata(i2c, aw20072);

	mutex_init(&aw20072->cfg_lock);

	/* aw20072 rst & int */
	if (np) {
		ret = aw20072_parse_dt(&i2c->dev, aw20072, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw20072->reset_gpio = -1;
		aw20072->irq_gpio = -1;
	}

	if (gpio_is_valid(aw20072->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw20072->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw20072_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(aw20072->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw20072->irq_gpio,
					    GPIOF_DIR_IN, "aw20072_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	/* hardware reset */
	aw20072_hw_reset(aw20072);

	/* aw20072 chip id */
	ret = aw20072_read_chipid(aw20072);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw20072_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw22xxx irq */
	if (gpio_is_valid(aw20072->irq_gpio) &&
	    !(aw20072->flags & AW20072_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw20072->irq_gpio),
						NULL, aw20072_irq, irq_flags,
						"aw20072", aw20072);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw20072->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw20072->flags |= AW20072_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw20072);

	aw20072_parse_led_cdev(aw20072, np);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s error creating led class dev\n",
			__func__);
		goto err_sysfs;
	}

	pr_info("%s probe completed successfully!\n", __func__);
	return 0;

 err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw20072->irq_gpio), aw20072);
 err_irq:
 err_id:
	devm_gpio_free(&i2c->dev, aw20072->reset_gpio);
	devm_gpio_free(&i2c->dev, aw20072->irq_gpio);
 err_gpio_request:
 err_parse_dt:
	devm_kfree(&i2c->dev, aw20072);
	aw20072 = NULL;
	return ret;
}

static int aw20072_i2c_remove(struct i2c_client *i2c)
{
	struct aw20072 *aw20072 = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);
	sysfs_remove_group(&aw20072->cdev.dev->kobj, &aw20072_attribute_group);
	led_classdev_unregister(&aw20072->cdev);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw20072->irq_gpio), aw20072);

	if (gpio_is_valid(aw20072->reset_gpio))
		devm_gpio_free(&i2c->dev, aw20072->reset_gpio);
	if (gpio_is_valid(aw20072->irq_gpio))
		devm_gpio_free(&i2c->dev, aw20072->irq_gpio);

	devm_kfree(&i2c->dev, aw20072);
	aw20072 = NULL;

	return 0;
}

static const struct i2c_device_id aw20072_i2c_id[] = {
	{AW20072_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw20072_i2c_id);

static const struct of_device_id aw20072_dt_match[] = {
	{.compatible = "awinic,aw20072_led"},
	{},
};

static struct i2c_driver aw20072_i2c_driver = {
	.driver = {
		   .name = AW20072_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw20072_dt_match),
		   },
	.probe = aw20072_i2c_probe,
	.remove = aw20072_i2c_remove,
	.id_table = aw20072_i2c_id,
};

static int __init aw20072_i2c_init(void)
{
	int ret = 0;

	pr_info("aw20072 driver version %s\n", AW20072_DRIVER_VERSION);

	ret = i2c_add_driver(&aw20072_i2c_driver);
	if (ret) {
		pr_err("fail to add aw20072 device into i2c\n");
		return ret;
	}
	return 0;
}

module_init(aw20072_i2c_init);

static void __exit aw20072_i2c_exit(void)
{
	i2c_del_driver(&aw20072_i2c_driver);
}

module_exit(aw20072_i2c_exit);

MODULE_DESCRIPTION("AW20072 LED Driver");
MODULE_LICENSE("GPL v2");
