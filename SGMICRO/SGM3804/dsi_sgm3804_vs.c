// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved
 */

#include <linux/string.h>
#include <linux/version.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define SGM3804_I2C_NAME "sgmicro-sgm3804"

#define SGM3804_DEBUG_ON       1
#define SGM3804_DEBUG_FUNC_ON  1
#define SGM3804_DEBUG_ARRAY_ON 1

/* Log define */
#define SGM3804_INFO(fmt,arg...)\
           printk("<<SGM3804-INF>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define SGM3804_ERROR(fmt,arg...)\
          printk("<<SGM3804-ERR>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define SGM3804_DEBUG(fmt,arg...)          do{\
	if(SGM3804_DEBUG_ON)\
	printk("<<SGM3804-DBG>>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
}while(0)
#define SGM3804_DEBUG_ARRAY(array, num)    do{\
	s32 i;\
	u8* a = array;\
	if(SGM3804_DEBUG_ARRAY_ON)\
	{\
		printk("<<SGM3804-DBG>>");\
		for (i = 0; i < (num); i++)\
		{\
			printk("%02x ", (a)[i]);\
			if ((i + 1 ) %10 == 0)\
			{\
				printk("\n<<SGM3804-DBG>>");\
			}\
		}\
		printk("\n");\
	}\
}while(0)
#define SGM3804_DEBUG_FUNC()               do{\
	if(SGM3804_DEBUG_FUNC_ON)\
	printk("<<SGM3804-FUNC>> Func:%s@Line:%d\n",__func__,__LINE__);\
}while(0)

struct i2c_client *sgm3804_i2c_client;
static int is_sgm3804_probed = 0;

static int sgm3804_register_powermanger(void);
static int sgm3804_unregister_powermanger(void);

/**
 * sgm3804_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
static int sgm3804_i2c_write(u8 addr, u8 * buffer, s32 len)
{
	int ret;
	u8 *addr_buf;
	struct i2c_msg msg;

	addr_buf = kmalloc(len + 1, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = addr & 0xFF;
	memcpy(&addr_buf[1], buffer, len);

	msg.flags = 0;
	msg.addr = sgm3804_i2c_client->addr;
	msg.buf = addr_buf;
	msg.len = len + 1;

	ret = i2c_transfer(sgm3804_i2c_client->adapter, &msg, 1);
	kfree(addr_buf);
	return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

#if 0
/**
 * sgm3804_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
static int sgm3804_i2c_read(u8 addr, u8 * buffer, s32 len)
{
	int ret;
	struct i2c_msg msgs[2];
	u8 addr_buf[1] = { addr };

	msgs[0].flags = 0;
	msgs[0].addr  = sgm3804_i2c_client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = addr_buf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = sgm3804_i2c_client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buffer;

	ret = i2c_transfer(sgm3804_i2c_client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}
#endif

int sgm3804_reg_init(void)
{
	u8 val;
	int i;
	int ret = -1;
	u8 reg_array[6] = {0x00, 0x0f, 0x01, 0x0f, 0x03, 0x03};
	if (!is_sgm3804_probed)
		return -1;

	if (!i2c_check_functionality(sgm3804_i2c_client->adapter, I2C_FUNC_I2C)) {
		SGM3804_ERROR("I2C check functionality failed.");
		return -ENODEV;
	} else {
		for (i = 0; i < 3; i++) {
			SGM3804_INFO("reg[%d] addr:0x%08x val:0x%08x",
				 i, reg_array[i * 2], reg_array[i * 2 + 1]);
			val = reg_array[i * 2 + 1];
			ret = sgm3804_i2c_write(reg_array[i * 2], &val, 1);
			if (ret < 0) {
				SGM3804_ERROR("I2C transfer error!");
			} else if (0 == ret) {
				SGM3804_INFO("write reg[%d] addr:0x%08x val:0x%08x success",
					 i, reg_array[i * 2], reg_array[i * 2 + 1] );
			} else {
				SGM3804_ERROR("write reg[%d] addr:0x%08x val:0x%08x fail",
					 i, reg_array[i * 2], reg_array[i * 2 + 1] );
			}
		}
		return ret;
	}
}

/**
 * sgm3804_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, <0: failed.
 */
static int sgm3804_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	SGM3804_INFO("SGM3804 driver probeing...");
	is_sgm3804_probed = 1;
	sgm3804_i2c_client = client;
	sgm3804_register_powermanger();

	return 0;
}

/**
 * sgm3804_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int sgm3804_remove(struct i2c_client *client)
{
	SGM3804_INFO("SGM3804 driver removing...");
	sgm3804_unregister_powermanger();

	return 0;
}

#if   defined(CONFIG_FB)
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block sgm3804_fb_notifier;

static int sgm3804_fb_notifier_callback(struct notifier_block *noti,
			unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

#ifdef CONFIG_SGM3804_INCELL_PANEL
#ifndef FB_EARLY_EVENT_BLANK
#error Need add FB_EARLY_EVENT_BLANK to fbmem.c
#endif

	if (ev_data && ev_data->data && event == FB_EARLY_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			SGM3804_DEBUG("Resume by fb notifier.");
		}
	}
#else
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			SGM3804_DEBUG("Resume by fb notifier.");
		}
	}
#endif

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			SGM3804_DEBUG("Suspend by fb notifier.");
		}
	}

	return 0;
}
#elif defined(CONFIG_PM)
/**
 * sgm3804_pm_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int sgm3804_pm_suspend(struct device *dev)
{
	return 0;
}

/**
 * sgm3804_pm_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int sgm3804_pm_resume(struct device *dev)
{
	return 0;
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops sgm3804_pm_ops = {
	.suspend = sgm3804_pm_suspend,
	.resume = sgm3804_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void sgm3804_early_suspend(struct early_suspend *h)
{
}

static void sgm3804_early_resume(struct early_suspend *h)
{
}

static struct early_suspend sgm3804_early_pm_ops = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = sgm3804_early_suspend,
	.resume = sgm3804_early_resume,
};
#endif

static int sgm3804_register_powermanger(void)
{
#if   defined(CONFIG_FB)
	sgm3804_fb_notifier.notifier_call = sgm3804_fb_notifier_callback;
	fb_register_client(&sgm3804_fb_notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&sgm3804_early_pm_ops);
#endif
	return 0;
}

static int sgm3804_unregister_powermanger(void)
{
#if   defined(CONFIG_FB)
	fb_unregister_client(&sgm3804_fb_notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&sgm3804_early_pm_ops);
#endif
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sgm3804_match_table[] = {
	{.compatible = "sgmicro,sgm3804",},
	{ },
};
#endif

static const struct i2c_device_id sgm3804_id_table[] = {
	{SGM3804_I2C_NAME, 0},
	{}
};

static struct i2c_driver sgm3804_driver = {
	.probe = sgm3804_probe,
	.remove = sgm3804_remove,
	.id_table = sgm3804_id_table,
	.driver = {
		.name = SGM3804_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sgm3804_match_table,
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		.pm = &sgm3804_pm_ops,
#endif
	},
};

/**
 * sgm3804_init - Driver Install function.
 * Return   0---succeed.
 */
int sgm3804_register(void)
{
	SGM3804_INFO("SGM3804 driver installing...");
	return i2c_add_driver(&sgm3804_driver);
}

/**
 * sgm3804_exit - Driver uninstall function.
 * Return   0---succeed.
 */
void sgm3804_unregister(void)
{
	SGM3804_INFO("SGM3804 driver remove...");
	i2c_del_driver(&sgm3804_driver);
}
