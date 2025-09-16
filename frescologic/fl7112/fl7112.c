/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/export.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include "fl7112.h"


#define FL7112_CNT	1
#define FL7112_NAME	"fl7112"
#define FL7112_VERSION_NUM  (uint32_t)0x0104410c

struct fl7112 {
	struct device *dev;
	u8 i2c_addr;

	u8 i2c_wbuf[FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK];
	u8 i2c_rbuf[FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK];
	struct i2c_client *i2c_client;
	enum fw_upgrade_status fw_status;
	uint32_t version;
    struct workqueue_struct *wq;  /* upgrade queue */
    struct work_struct wk;        /* upgrade work */
    u32 reset_pin;   /*reset pin */
    u32 pwr_1v2;
    u32 pwr_3v3;
};

struct fl7112_reg_cfg {
	u8 reg;
	u8 val;
};

struct fl7112_dev {
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	struct device_node	*nd;
	int major;
	void *private_data;
};
static struct fl7112_dev fl7112dev;

static int fl7112_read(struct fl7112 *pdata, int reg, char *buf, u32 size);
static bool fl7112_mtpread_firmware(struct fl7112 *pdata,int address,BYTE* DataBuffer,int DataBufferLength);
static bool fl7112_mtpwrite_firmware(struct fl7112 *pdata,int Address,const u8* DataBuffer,int DataBufferLength,bool* IsError,bool* IsBusy);
static void fl7112_mtppowermodedisable(struct fl7112 *pdata);
static bool fl7112_mtpwriteunlock(struct fl7112 *pdata);
static void fl7112_mtpwritelock(struct fl7112 *pdata);
static bool fl7112_mtpcheckstatus(struct fl7112 *pdata,bool* IsError,bool* IsBusy);
static void fl7112_reset(struct fl7112 *pdata);

/*
 * Write one reg with more values;
 * Reg -> value0, value1, value2.
 */
static int fl7112_write(struct fl7112 *pdata, int reg,
		const u8 *buf, int size)
{
	struct i2c_client *client = pdata->i2c_client;
	u8 regAddr[2];

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = size + 2,
		.buf = pdata->i2c_wbuf,
	};

	regAddr[0] = (reg >> 8) & 0xFF;
	regAddr[1] = reg & 0xFF;
	pdata->i2c_wbuf[0] = regAddr[0];
	pdata->i2c_wbuf[1] = regAddr[1];
	if (size > (FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK - 1)) {
		pr_err("invalid write buffer size %d\n", size);
		return -EINVAL;
	}

	memcpy(pdata->i2c_wbuf + 2, buf, size);

	if (i2c_transfer(client->adapter, &msg, 1) < 1) {
		pr_err("i2c write failed\n");
		return -EIO;
	}

	return 0;
}

/*
 * Write one reg with one value;
 * Reg -> value
 */
static int fl7112_write_byte(struct fl7112 *pdata, int reg, u8 value)
{
	struct i2c_client *client = pdata->i2c_client;
	u8 regAddr[2];

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = pdata->i2c_wbuf,
	};

    regAddr[0] = (reg >> 8) & 0xFF;
	regAddr[1] = reg & 0xFF;

	memset(pdata->i2c_wbuf, 0, FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK);
	pdata->i2c_wbuf[0] = regAddr[0];
	pdata->i2c_wbuf[1] = regAddr[1];
	pdata->i2c_wbuf[2] = value;

	if (i2c_transfer(client->adapter, &msg, 1) < 1) {
		pr_err("i2c write failed\n");
		return -EIO;
	}

	return 0;
}

/* return: false - OK    true - NG */
static int fl7112_read(struct fl7112 *pdata, int reg, char *buf, u32 size)
{
	struct i2c_client *client = pdata->i2c_client;
	u8 regAddr[2];

 	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(regAddr),
			.buf = regAddr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = pdata->i2c_rbuf,
		}
	};

    regAddr[0] = (reg >> 8) & 0xFF;
	regAddr[1] = reg & 0xFF;

	if (size > FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK) {
		pr_err("invalid read buff size %d\n", size);
		return -EINVAL;
	}

	memset(pdata->i2c_wbuf, 0x0, FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK);
	memset(pdata->i2c_rbuf, 0x0, FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK);

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		pr_err("i2c read failed\n");
		return -EIO;
	}

	memcpy(buf, pdata->i2c_rbuf, size);

	return 0;
}

/* func : Check if the special bit is set
 * return : [TRUE] -read Ok  [FALSE] - read error
 */
static bool i2cex_bitcheck(struct fl7112 *pdata,
    int i2coffset,int Bit,bool* IsOne)
{
    bool bStatus = true;    /* error occur  */
    bool ret = true;
    BYTE byteData = 0;

    bStatus = fl7112_read(pdata,
                          i2coffset,
                          &byteData,
                          1 );

	if (bStatus){
    /* read error */
		pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
		goto Exit;
	}

    if (IS_BIT_SET(byteData, Bit))
        *IsOne = true;
    else
        *IsOne = false;
Exit:
    return ret;
}

/* func : write byte with check
 * return TRUE - write OK , FALSE - write NG
 */
static bool i2cex_writebytecheck(struct fl7112 *pdata,
    int i2coffset,BYTE ByteData)
{
    bool bStatus = true;
    bool ret = true;
    BYTE byteDataReadBack;

    byteDataReadBack = 0;
    bStatus = fl7112_read(pdata,
                          i2coffset,
                          &byteDataReadBack,
                          1 );
	if (bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
    }

    if ( byteDataReadBack != ByteData ) {
   		bStatus = fl7112_write_byte( pdata,
                           i2coffset,
                           ByteData);
        if(bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
        }

    }
    return ret;
}

/* func   :  Write Protect Enable.
 * return :  none
 */
static void fl7112_mtpwritelock(struct fl7112 *pdata)
{
    bool bStatus;
    bool isOne;

    /* check if bit 3 is set */
    bStatus = i2cex_bitcheck(pdata,
                            FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                            3,
                            &isOne);
    /* bit is set */
    if (isOne){
        /* write [0X9010] 0x0 to enable Write Protect */
		bStatus = fl7112_write_byte( pdata,
                           FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                           0x00);
        if(bStatus){
            /* write error */
            pr_err("fl7112 %s error line:%d write [0X9010] failed\n", __func__, __LINE__);
        }
    }

    return;
}

/* func : bit set process
 * return : [TRUE] - set OK, [FALSE] - set NG
 */
static bool i2cex_bitset(struct fl7112 *pdata,
    int i2coffset,int bit)
{
    bool bStatus = true;
    bool ret = true;
    BYTE byteData= 0;

    bStatus = fl7112_read( pdata,
                          i2coffset,
                          &byteData,
                          1 );
	if (bStatus){
		pr_err("fl7112 %s error line:%d read NG\n", __func__, __LINE__);
        ret = false;
		goto Exit;
	}

    BIT_SET( byteData, bit );

    bStatus = fl7112_write_byte( pdata,
                           i2coffset,
                           byteData);
	if (bStatus){
        if(i2coffset != FL7112_FIRMWARE_INTERNAL_REGISTER_0XA003)
		    pr_err("fl7112 %s error line:%d write NG\n", __func__, __LINE__);
        ret = false;
	}

Exit:
    return ret;
}

/* func : bit clear process
 * return : [TRUE] - set OK, [FALSE] - set NG
 */
bool i2cex_bitclear(struct fl7112 *pdata,
    int i2coffset,int bit)
{
    bool bStatus;
    BYTE byteData;

    byteData = 0;

    bStatus = fl7112_read( pdata,
                          i2coffset,
                          &byteData,
                          1 );
	if (bStatus){
		pr_err("fl7112 %s error line:%d read NG\n", __func__, __LINE__);
        bStatus = false;
		goto Exit;
	}

    /*clear bit  */
    byteData &= ~(1 << bit);

    bStatus = fl7112_write_byte( pdata,
                           i2coffset,
                           byteData);
	if (bStatus){
		pr_err("fl7112 %s error line:%d write NG\n", __func__, __LINE__);
        bStatus = false;
	}

Exit:
    return bStatus;
}

/* func :   setup page
 * return : [TRUE] - set OK ,[FLASE] - set NG
 */
static bool fl7112_mtppagesetup(struct fl7112 *pdata,
    BYTE* PageCurrent,int Address)
{
    bool bStatus = true;
    bool ret = true;
    int page;

    page = ( BYTE )( Address / FL7112_MTP_MAX_PAGE_SIZE );
    if ( *PageCurrent != page ) {
        BYTE byteData;
        *PageCurrent = page;
        byteData = 0;
		bStatus = fl7112_read(pdata,
					FL7112_FIRMWARE_INTERNAL_REGISTER_0X9003, &byteData, 1);
        if (bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
        byteData &= 0x0F;
        byteData |= ( *PageCurrent << 4 );
		bStatus = fl7112_write_byte(pdata,
                    FL7112_FIRMWARE_INTERNAL_REGISTER_0X9003,
                    byteData);
        if (bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
    }else{
        bStatus = TRUE;
    }

Exit:
    return ret;
}

/* func :  Write Protect Disable
 * return TRUE - Disable OK ,FLASE - Disable NG
 */
static bool fl7112_mtpwriteunlock(struct fl7112 *pdata)
{
    bool bStatus = true;
    bool ret = true;
    bool isOne;

    bStatus = i2cex_bitcheck(pdata,
                              FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                              3,
                              &isOne );
    if (!bStatus){
        pr_err("fl7112 %s error line:%d bit check NG!\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    /* bit 3 not set */
    if (!isOne) {
        bStatus = fl7112_write_byte(pdata,
                                   FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                                   0x55);
        if (bStatus){
            pr_err("fl7112 %s error line:%d 0x55\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        bStatus = fl7112_write_byte(pdata,
                                   FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                                   0xAA);
        if (bStatus){
            pr_err("fl7112 %s error line:%d 0xAA\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        bStatus = fl7112_write_byte(pdata,
                                   FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                                   0xFF);
        if (bStatus){
            pr_err("fl7112 %s error line:%d 0xFF\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        bStatus = fl7112_write_byte(pdata,
                                   FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010,
                                   0x18);
        if (bStatus){
            pr_err("fl7112 %s error line:%d 0x18\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
    }
Exit:
    return ret;
}

/* func :  state check
 * return : TRUE - state ok , FALSE - state NG
 */
static bool fl7112_mtpcheckstatus(
    struct fl7112 *pdata,
    bool* IsError,
    bool* IsBusy
    )
{
    bool bStatus;
    bool ret = true;
    BYTE byteData;
    int retryMax;
    int retry;
    bool isError;
    bool isBusy;

    retryMax = FL7112_MTP_CHECK_STATUS_RETRY_MAX;
    bStatus = true;
    isError = false;
    isBusy = false;

    for ( retry = 0; retry < retryMax; retry++ )
    {
        byteData = 0;

        /* read 0x9001  */
		bStatus = fl7112_read(pdata,
					FL7112_FIRMWARE_INTERNAL_REGISTER_0X9001, &byteData, 1);
        if (bStatus){
            pr_err("%s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        /* check error */
        if ( IS_BIT_SET( byteData, 6 ) )
            isError = true;
        else
            isError = false;

        /* check busy */
        if ( IS_BIT_SET( byteData, 7 ) )
            isBusy = true;
        else
            isBusy = false;

        *IsError = isError;
        *IsBusy = isBusy;

        if (isError) {
            // Error Bit.
            ret = false;
            goto Exit;
        }

        if (!isBusy) {
            // Good.
            ret = true;
            break;
        } else {
            // Busy.
            if ( retry == ( retryMax - 1 ) ) {
                ret = false;
                break;
            }
        }
    }
Exit:
    return ret;
}

/* func   : Power Saving Disable
 * return : TRUE - OK ,FLASE - NG */
static void fl7112_mtppowermodedisable(struct fl7112 *pdata)
{
    bool bStatus;

    /* write [0X9000]  0x71  */
    bStatus = i2cex_writebytecheck(pdata,
                                    FL7112_FIRMWARE_INTERNAL_REGISTER_0X9000,
                                    0x71 );
    if(!bStatus){
        pr_err("fl7112 %s error line:%d Power Saving Disable failed\n", __func__, __LINE__);
    }

    return ;
}

/* func :  Recovery Firmware Block
 * return : TRUE - Recovery ok , [FALSE] - Recovery NG
 */
bool FL7112_MtpWrite_Firmware_Recovery(
    struct fl7112 *pdata,
    int Address,
    DWORD DataBuffer,
    bool* IsError,
    bool* IsBusy
    )
{
    bool bStatus;
    bool ret = true;
    bool isError = false;
    bool isBusy = false;
    DWORD_MAPPING dwordMappingReadBack;
    PDWORD_MAPPING dwordMapping;
    int indexOfByte;

    // MTP Read Back.
    fl7112_mtpwritelock( pdata );

    dwordMappingReadBack.Value = 0;
    bStatus = fl7112_read( pdata,
                          Address,
                          (char *)&dwordMappingReadBack.Value,
                          4 );
    if (bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    bStatus = fl7112_mtpwriteunlock( pdata );
    if(!bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    // Write each bytes.
    dwordMapping = ( PDWORD_MAPPING )&DataBuffer;
    for ( indexOfByte = 0; indexOfByte < 4; indexOfByte++ )
    {
        // Low 4.
        dwordMappingReadBack.ByteData[ indexOfByte ] &= 0xF0;
        dwordMappingReadBack.ByteData[ indexOfByte ] |= ( dwordMapping->ByteData[ indexOfByte ] & 0x0F );

        bStatus = fl7112_write( pdata,
                                Address,
                                (u8*)&dwordMappingReadBack.Value,
                                4 );
        if (bStatus){
            pr_err("fl7112 %s error line:%d  fl7112_write failed\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus( pdata,
                                         &isError,
                                         &isBusy );
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        // Full.
        dwordMappingReadBack.ByteData[ indexOfByte ] = dwordMapping->ByteData[ indexOfByte ];

        bStatus = fl7112_write( pdata,
                               Address,
                               (u8*)&dwordMappingReadBack.Value,
                               4 );
        if (bStatus){
            pr_err("fl7112 %s error line:%d  fl7112_write failed\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus( pdata,
                                         &isError,
                                         &isBusy );
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
    }

Exit:

    *IsError = isError;
    *IsBusy = isBusy;

    return ret;
}

/* func :  firmware write process
 * return : TRUE - write ok , [FALSE] - process or State NG
 */
static bool fl7112_mtpwrite_firmware(
    struct fl7112 *pdata,
    int Address,
    const u8* DataBuffer,
    int DataBufferLength,
    bool* IsError,
    bool* IsBusy
    )
{
    bool bStatus;
    bool ret = true;
    int indexOfDword;
    const u8* byteData = DataBuffer;
    BYTE pageCurrent;
    int addressOfMtp;
    int offset;
    int offsetInPage;
    bool isError;
    bool isBusy;

    isError = false;
    isBusy = false;

    fl7112_mtppowermodedisable(pdata);

    bStatus = fl7112_mtpwriteunlock(pdata);
    if(!bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    offset = Address;
    pageCurrent = 0xFF;
    for ( indexOfDword = 0; indexOfDword < DataBufferLength; indexOfDword += 4 )
    {
        bStatus = fl7112_mtppagesetup(pdata,
                                      &pageCurrent,
                                      offset);
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        offsetInPage = offset % FL7112_MTP_MAX_PAGE_SIZE;
        addressOfMtp = FL7112_FIRMWARE_INTERNAL_REGISTER_0X5000 + offsetInPage;
        bStatus = fl7112_write(pdata,
                               addressOfMtp,
                               byteData,
                               4);
        if (bStatus){
            pr_err("fl7112 %s error line:%d  fl7112_write failed\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus( pdata,
                                         &isError,
                                         &isBusy );
#if FL7112_MTP_RECOVERY
        if (!bStatus && isError)
        {
            PDWORD_MAPPING dwordMapping;
            int indexOfRecovery;

            dwordMapping = (PDWORD_MAPPING)byteData;

            for (indexOfRecovery = 0; indexOfRecovery < FL7112_MTP_WRITE_RECOVERY_MAX; indexOfRecovery++)
            {
                bStatus = FL7112_MtpWrite_Firmware_Recovery( pdata,
                                                             addressOfMtp,
                                                             dwordMapping->Value,
                                                             &isError,
                                                             &isBusy );
                if (bStatus){
                    break;
                }
            }
            if (!bStatus){
                pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
                ret = false;
                goto Exit;
            }
        }
#else
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
#endif
        byteData += 4;
        offset += 4;
    }

Exit:
    *IsError = isError;
    *IsBusy = isBusy;

    return ret;
}

/* func : reset chip by register
 * return :void
 */
static void fl7112_reset(struct fl7112 *pdata)
{
    // set bit3 for register [0xA003]
    (void)i2cex_bitset(pdata,FL7112_FIRMWARE_INTERNAL_REGISTER_0XA003,3);
}

void fl7112_chip_reset(struct fl7112 *pdata)
{

    /*  LT7211_RST - H |  gpio_40 - L */
    gpio_direction_output(pdata->reset_pin, 0);
    /* sleep */
    msleep(100);
    /*  LT7211_RST - L |  gpio_40 - H */
    gpio_direction_output(pdata->reset_pin, 1);
}

static int fl7112_parse_dt(struct device *dev,struct fl7112 *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;

    /* reset gpio */
    pdata->reset_pin = of_get_named_gpio(np, "fl7112,reset-gpio", 0);
	if (gpio_is_valid(pdata->reset_pin)) {
		ret = gpio_request(pdata->reset_pin, "reset_pin");
	} else {
		dev_err(dev, "%s: gpio %d is invalid!", __func__, pdata->reset_pin);
        // return -1;
	}

    /* pwr 1v2 */
    pdata->pwr_1v2 = of_get_named_gpio(np, "fl7112,pwr-1v2", 0);
	if (gpio_is_valid(pdata->pwr_1v2)) {
		ret = gpio_request(pdata->pwr_1v2, "pwr_1v2");
	} else {
		dev_err(dev, "%s: gpio %d is invalid!", __func__, pdata->pwr_1v2);
        return -1;
	}

    /* pwr 3v3 */
    pdata->pwr_3v3 = of_get_named_gpio(np, "fl7112,pwr-3v3", 0);
	if (gpio_is_valid(pdata->pwr_3v3)) {
		ret = gpio_request(pdata->pwr_3v3, "pwr_3v3");
	} else {
		dev_err(dev, "%s: gpio %d is invalid!", __func__, pdata->pwr_3v3);
        return -1;
	}

    return ret;
}

/* func :  Recovery InformationBlock
 * return : TRUE - Recovery ok , [FALSE] - Recovery NG
 */
bool FL7112_MtpWrite_InformationBlock_Recovery(
    struct fl7112 *pdata,
    int Address,
    DWORD Data,
    bool* IsError,
    bool* IsBusy
    )
{
    bool bStatus;
    bool ret = true;
    bool isError = false;
    bool isBusy = false;
    DWORD_MAPPING dwordMappingReadBack;
    PDWORD_MAPPING dwordMapping;
    int indexOfByte;

    // MTP Read Back.
    fl7112_mtpwritelock( pdata );

    dwordMappingReadBack.Value = 0;
    bStatus = fl7112_read( pdata,
                          Address,
                          (char *)&dwordMappingReadBack.Value,
                          4 );
    if (bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    bStatus = fl7112_mtpwriteunlock( pdata );
    if(!bStatus){
        pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    // Write each bytes.
    dwordMapping = ( PDWORD_MAPPING )&Data;
    for ( indexOfByte = 0; indexOfByte < 4; indexOfByte++ )
    {
        // Low 4.
        dwordMappingReadBack.ByteData[ indexOfByte ] &= 0xF0;
        dwordMappingReadBack.ByteData[ indexOfByte ] |= ( dwordMapping->ByteData[ indexOfByte ] & 0x0F );

        bStatus = fl7112_write( pdata,
                                Address,
                                (u8*)&dwordMappingReadBack.Value,
                                4 );
        if (bStatus){
            pr_err("fl7112 %s error line:%d  fl7112_write failed\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus( pdata,
                                         &isError,
                                         &isBusy );
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        // Full.
        dwordMappingReadBack.ByteData[ indexOfByte ] = dwordMapping->ByteData[ indexOfByte ];

        bStatus = fl7112_write( pdata,
                               Address,
                               (u8*)&dwordMappingReadBack.Value,
                               4 );
        if (bStatus){
            pr_err("fl7112 %s error line:%d  fl7112_write failed\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus( pdata,
                                         &isError,
                                         &isBusy );
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
    }

Exit:

    *IsError = isError;
    *IsBusy = isBusy;

    return ret;
}

/* func :  informationblock write process
 * return : TRUE - write ok , [FALSE] - process or State NG
 */
bool fl7112_mtpwrite_informationblock(
    struct fl7112 *pdata,
    int Address,
    u8* DataBuffer,
    int DataBufferLength,
    bool* IsError,
    bool* IsBusy
    )
{
    bool bStatus;
    bool ret = true;
    int indexOfDword;
    u8* byteData;
    bool isError;
    bool isBusy;
    int addressOfMtp;

    isError = false;
    isBusy = false;

    fl7112_mtppowermodedisable(pdata);

    bStatus = fl7112_mtpwriteunlock(pdata);
     if (!bStatus){
        pr_err("fl7112 %s error line:%d Write Unlock NG!\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    byteData = DataBuffer;
    for ( indexOfDword = 0; indexOfDword < DataBufferLength; indexOfDword += 4 )
    {
        addressOfMtp = Address + indexOfDword;
        bStatus = fl7112_write(pdata,
                               addressOfMtp,
                               byteData,
                               4);
        if (bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        isError = false;
        isBusy = false;
        bStatus = fl7112_mtpcheckstatus(pdata,
                                         &isError,
                                         &isBusy);
#if FL7112_MTP_RECOVERY
        if (!bStatus && isError)
        {
            PDWORD_MAPPING dwordMapping;
            int indexOfRecovery;

            dwordMapping = ( PDWORD_MAPPING )byteData;

            for ( indexOfRecovery = 0; indexOfRecovery < FL7112_MTP_WRITE_RECOVERY_MAX; indexOfRecovery++ )
            {
                bStatus = FL7112_MtpWrite_InformationBlock_Recovery( pdata,
                                                                     addressOfMtp,
                                                                     dwordMapping->Value,
                                                                     &isError,
                                                                     &isBusy );
                if (bStatus){
                    break;
                }
            }
            if (!bStatus){
                pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
                ret = false;
                goto Exit;
            }
        }
#else
        if (!bStatus){
            pr_err("fl7112 %s error line:%d state NG!\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
#endif
        byteData += 4;
    }

Exit:
    *IsError = isError;
    *IsBusy = isBusy;
    return ret;
}

/* func : read back informationblock data
 * return : [TRUE] - read ok,  [FALSE] - read NG
 */
bool fl7112_mtpread_informationblock(
    struct fl7112 *pdata,
    int Address,
    BYTE* DataBuffer,
    int DataBufferLength
)
{
    bool bStatus = true;
    bool ret = true;
    int indexOfDword;
    BYTE* byteData = NULL;
    int addressOfMtp;

    /* Disable Power Saving */
    fl7112_mtppowermodedisable(pdata);

    /* Write Protect Enable */
    fl7112_mtpwritelock(pdata);

    byteData = DataBuffer;
    for (indexOfDword = 0; indexOfDword < DataBufferLength; indexOfDword += 4)
    {
        addressOfMtp = Address + indexOfDword;
        // Read the information block data in the MTP memory
        bStatus = fl7112_read(pdata,
                              addressOfMtp,
                              byteData,
                              4);
        if(bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        byteData += 4;
    }

Exit:
    return ret;
}
/* func   : firmware upgrade process
 * return : [TRUE] - upgrade OK,  [FALSE] - upgrade NG
 */
static bool fl7112_firmware_upgrade(struct fl7112 *pdata,
			const struct firmware *cfg)
{
	int data_len = (int)cfg->size;
    bool bStatus = false;
    bool ret = true;
    BYTE* dataBufferInformationBlock = NULL;
    BYTE* dataBufferFirmwareReadBack = NULL;
    BYTE* dataBufferInformationBlockReadBack = NULL;
    bool isError = false;
    bool isBusy = false;

    if(!cfg){
        pr_err("fl7112 %s error line:%d param err : cfg\n", __func__, __LINE__);
        return false;
    }

    if (data_len != 0x6300) {
        pr_info("fl7112 FW total size is incorrect\n");
        return false;
    }

    dataBufferInformationBlock =  (u8*)cfg->data;
    dataBufferInformationBlock += FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK;
    dataBufferFirmwareReadBack = (BYTE*)kmalloc(FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK, GFP_KERNEL);
    memset(dataBufferFirmwareReadBack, 0, FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK);

    dataBufferInformationBlockReadBack = (BYTE*)kmalloc(FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK, GFP_KERNEL);
    memset(dataBufferInformationBlockReadBack, 0, FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK);

	pdata->fw_status = UPDATE_RUNNING;

    pr_info("fl7112 Firmware total size 0x%x\n", data_len);

    /* 1. Hold MCU Reset */
    bStatus = i2cex_bitset(pdata,
                           FL7112_FIRMWARE_INTERNAL_REGISTER_0XA002,
                           5);
    if(!bStatus){
        pr_err("fl7112 %s error line:%d Hold MCU Reset NG\n", __func__, __LINE__);
        ret = false;
		goto Exit;
    }else{
        pr_info("fl7112 Hold MCU Reset OK! \n");
    }

    /* 2. Firmware write */
    bStatus = fl7112_mtpwrite_firmware( pdata,
                                        0x0,
                                        cfg->data,
                                        FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK,
                                        &isError,&isBusy );
    if(!bStatus){
        pr_info("fl7112 firmware write NG! \n");
        ret = false;
        goto Exit;
    }else{
        pr_info("fl7112 firmware write OK! \n");
    }

    /* 3. read back and compare firmware  */
    bStatus = fl7112_mtpread_firmware(pdata,
                                       0x0,
                                       dataBufferFirmwareReadBack,
                                       FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK);
	if (!bStatus){
		pr_err("fl7112 %s error line:%d - read back firmware NG!\n", __func__, __LINE__);
	}

    bStatus = memcmp(cfg->data, dataBufferFirmwareReadBack,
                     FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK );
    if(0 == bStatus){
        pr_info("fl7112 firmware Compare OK! \n");
    }else{
        pr_err("fl7112 %s error line:%d - firmware Compare NG!\n", __func__, __LINE__);
        ret = false;
        goto Exit;
    }

    /* 4. InformationBlock write */
    bStatus = fl7112_mtpwrite_informationblock( pdata,
                                                FL7112_FIRMWARE_INTERNAL_REGISTER_0X9800,
                                                dataBufferInformationBlock,
                                                FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK,
                                                &isError,
                                                &isBusy );
    if (!bStatus){
		pr_err("fl7112 %s error line:%d - write Information Block NG!\n", __func__, __LINE__);
        ret = false;
		goto Exit;
	}else{
        pr_info("fl7112 Information block write OK! \n");
    }

    /* 5. read back and compare informationblock */
    bStatus = fl7112_mtpread_informationblock( pdata,
                                               FL7112_FIRMWARE_INTERNAL_REGISTER_0X9800,
                                               dataBufferInformationBlockReadBack,
                                               FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK
                                               );

	if (!bStatus){
		pr_err("fl7112 %s error line:%d - read back informationblock NG!\n", __func__, __LINE__);
	}

    bStatus = memcmp( dataBufferInformationBlock,
                        dataBufferInformationBlockReadBack,
                        FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK );
    if(0 == bStatus){
        pr_info("fl7112 InformationBlock Compare OK! \n");
    }else{
        pr_err("fl7112 %s error line:%d - InformationBlock Compare NG!\n", __func__, __LINE__);
        ret = false;
    }

     pr_info("fl7112 FW upgrade success!\n");

Exit:
    /* Release MCU Reset */
    bStatus = i2cex_bitclear(pdata,
                        FL7112_FIRMWARE_INTERNAL_REGISTER_0XA002,
                        5);

    kfree(dataBufferFirmwareReadBack);
    kfree(dataBufferInformationBlockReadBack);
    return ret;
}

static void fl7112_firmware_cb(const struct firmware *cfg, void *data)
{
	struct fl7112 *pdata = (struct fl7112 *)data;

	if (!cfg) {
		pr_err("fl7112 get firmware failed\n");
		return;
	}

    /* FW upgrade */
	if(!fl7112_firmware_upgrade(pdata, cfg)){
       pr_err("fl7112 upgrade firmware failed\n");
    }

    msleep(3000);
    /* reset chip */
    fl7112_reset(pdata);

    /* release */
	release_firmware(cfg);
}

BYTE fl7112_read_addr_info(struct fl7112 *pdata,int addr)
{
	BYTE rDat = 0;
    bool bStatus = true;

    bStatus = fl7112_read(pdata, addr, &rDat, 1);
	if (bStatus){
        pr_err("fl7112 read addr 0x%x failed, ret: %d\n",bStatus);
    }else{
        pr_info("fl7112 read addr 0x%x : %x\n",addr,rDat);
    }

	return rDat;
}


/* func : read firmware info by address
 * return : TRUE - read OK, FALSE - read NG
 */
bool fl7112_mtpread_firmware(struct fl7112 *pdata,int address,BYTE* DataBuffer,
    int DataBufferLength)
{
    bool bStatus= true;
    bool ret = true;
    int index;
    BYTE* byteData;
    int offset;
    int offsetInPage;
    BYTE pageCurrent;

    fl7112_mtppowermodedisable(pdata);

    fl7112_mtpwritelock(pdata);

    byteData = DataBuffer;
    offset = address;
    pageCurrent = 0xFF;
    for ( index = 0; index < DataBufferLength; index += 4 ){

        bStatus = fl7112_mtppagesetup( pdata,
                                       &pageCurrent,
                                       offset);
        if(!bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }

        offsetInPage = offset % FL7112_MTP_MAX_PAGE_SIZE;
    	bStatus = fl7112_read( pdata,
                               FL7112_FIRMWARE_INTERNAL_REGISTER_0X5000 + offsetInPage,
                               byteData,
                               4);
        if (bStatus){
            pr_err("fl7112 %s error line:%d\n", __func__, __LINE__);
            ret = false;
            goto Exit;
        }
        byteData += 4;
        offset += 4;
    }
Exit:
    return ret;
}

/*  func   : get firmware version  info
 *  return : [TRUE] - read version OK, [FALSE] - read version NG
 */
static bool fl7112_firmware_get_version(struct fl7112 *pdata, BYTE* VersionString)
{
    bool bStatus = true;

    bStatus = fl7112_mtpread_firmware(pdata,
                                      FL7112_FIRMWARE_OFFSET_0X1000,
                                      VersionString,
                                      4);
    if (!bStatus){
		pr_err("%s error line:%d get version Failed!\n", __func__, __LINE__);
        return false;
	}

    return true;
}

/* func : read firmware version
 * return : TRUE - read OK, FALSE - read NG
 */
bool fl7112_read_fw_version(struct fl7112 *pdata, uint32_t* Version)
{
    bool bStatus = true;
    BYTE VersionString[4] = {0};
    uint32_t ver = 0;

    bStatus = fl7112_firmware_get_version(pdata, VersionString);
    if (!bStatus){
		pr_err("fl7112 %s error line:%d read version failed!\n", __func__, __LINE__);
        return false;
	}

    ver = ((VersionString[3] << 24) | (VersionString[2] << 16) | (VersionString[1] << 8) | VersionString[0]);
    *Version = ver;

    return true;
}

static void fl7112_fw_upgrade_work(struct work_struct *work)
{
	int32_t rc = 0;
    struct fl7112 *pdata = container_of(work, struct fl7112,wk);

	pr_info("[fl7112] %s enter\n",__func__);

    if (!pdata) {
		pr_err("fl7112 %s : pdata is NULL\n",__func__);
		return ;
	}

	rc = request_firmware_nowait(THIS_MODULE, true,
				"fl7112_fw.bin", &pdata->i2c_client->dev, GFP_KERNEL,
				pdata, fl7112_firmware_cb);
	if (rc < 0){
		pr_err("[fl7112] request firmware upgrade failed\n");
	}

	return;
}

static int fl7112_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fl7112 *pdata;
	int ret = 0;

	pr_info("fl7112_probe enter!\n");

	if (!client || !client->dev.of_node) {
		pr_err("invalid input\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("fl7112 device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev,
		sizeof(struct fl7112), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = fl7112_parse_dt(&client->dev, pdata);
	if (ret) {
		pr_err("Fl7112 Parse Device Tree NG! %d\n",ret);
	}else{
		/* reset GPIO */
		if(gpio_direction_output(pdata->reset_pin, 0)){
			pr_err("Fl7112 reset ctl failed!\n");
		}
		/* 1v2 - HIGH */
		if(gpio_direction_output(pdata->pwr_1v2, 1)){
			pr_err("Fl7112 pwr 1v2 failed!\n");
		}
		/* 3v3 - HIGH */
		if(gpio_direction_output(pdata->pwr_3v3, 1)){
			pr_err("Fl7112 pwr 3v3 failed!\n");
		}
    }

	pdata->dev = &client->dev;
	pdata->i2c_client = client;

	i2c_set_clientdata(client, pdata);
	dev_set_drvdata(&client->dev, pdata);

    /* init upgrade queue */
    pdata->wq = create_singlethread_workqueue("fl7112_wq");
    if (!pdata->wq){
        pr_err("fl7112 create workqueue failed!\n");
	}
    /*init upgrade work */
    INIT_WORK(&(pdata->wk), fl7112_fw_upgrade_work);

#if FL7112_MTP_AUTO_UPGRADE
    /* read fw version */
    if(fl7112_read_fw_version(pdata, &pdata->version)){
        /* compare version */
        if(FL7112_VERSION_NUM != pdata->version){
            pr_info("%s: check version NG [0x%08x] expect version [0x%08x]\n", __func__,pdata->version,FL7112_VERSION_NUM);
            /* need to upgrade */
            queue_work(pdata->wq, &(pdata->wk));
        }else{
            /* version correct and nothing todo */
            pr_info("%s: check version OK [0x%08x]\n", __func__,pdata->version);
            fl7112_reset(pdata);
        }
    }else{
        pr_err("fl7112 read FW version failed!\n");
    }
#endif

	return 0;
}

static int fl7112_remove(struct i2c_client *client)
{
	cdev_del(&fl7112dev.cdev);
	unregister_chrdev_region(fl7112dev.devid, FL7112_CNT);

	device_destroy(fl7112dev.class, fl7112dev.devid);
	class_destroy(fl7112dev.class);
	return 0;
}

static const struct of_device_id fl7112_of_match[] = {
	{ .compatible = "Parade,fl7112-fs8822" },
	{ }
};
MODULE_DEVICE_TABLE(of, geni_i2c_dt_match);

static struct i2c_driver fl7112_driver = {
	.probe = fl7112_probe,
	.remove = fl7112_remove,
	.driver = {
		.name = "fl7112",
		.of_match_table = fl7112_of_match,
	},
};


module_i2c_driver(fl7112_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Parade, FL7112");
