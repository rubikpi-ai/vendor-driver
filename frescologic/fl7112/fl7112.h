/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __FL7112_H_
#define __FL7112_H_

#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X5000        0x5000
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9000        0x9000
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9001        0x9001
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9003        0x9003
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9010        0x9010
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9800        0x9800
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9B00        0x9B00
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0X9BF8        0x9BF8
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0XA002        0xA002
#define FL7112_FIRMWARE_INTERNAL_REGISTER_0XA003        0xA003
#define FL7112_MTP_MAX_PAGE_SIZE                        0x1000
#define FL7112_MTP_MAX_NUMBER_OF_PAGES                  6
#define FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK           ( FL7112_MTP_MAX_NUMBER_OF_PAGES * FL7112_MTP_MAX_PAGE_SIZE )
#define FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK        0x300
#define FL7112_FIRMWARE_IMAGE_SIZE                      ( FL7112_MTP_MAX_SIZE_OF_FIRMWARE_BLOCK + FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK )
#define FL7112_NUMBER_OF_REGISTER_COMMAND               ( ( FL7112_MTP_MAX_SIZE_OF_INFORMATION_BLOCK - 24 ) / 4 )
#define FL7112_REGISTER_COMMAND_END                     0xFFFFFFFF
#define FL7112_REGISTER_SIGNATURE                       0x96C32D0F
#define FL7112_FIRMWARE_OFFSET_0X1000                   0x1000
#define FL7112_I2C_SLAVE_ADDRESS                        0x74
#define FL7112_MTP_WRITE_RECOVERY_MAX                   3
#define FL7112_MTP_CHECK_STATUS_RETRY_MAX               20
#define FL7112_MTP_POWER_CURRENT_TABLE_SIZE             8
#define FL7112_MTP_RECOVERY                             1
#define FL7112_MTP_AUTO_UPGRADE                         0
#define BIT_SET( data, bit ) ( data |= ( 1 << bit ) )
#define IS_BIT_SET(  data, bit ) ( data & ( 1 << bit ) )
#define EXIT_FALSE( status )  \
    if ( !status )           \
    {                           \
        pr_err("%s error line:%d\n", __func__, __LINE__); \
        goto Exit;              \
    }

typedef unsigned char BYTE;
typedef unsigned long DWORD;
typedef unsigned short WORD;

typedef union
{
    struct
    {
        DWORD RegisterSignature;
        DWORD RegisterSizeInDword;
        DWORD RegisterCRC32;
        DWORD FirmwareSignature;
        DWORD FirmwareSizeInDword;
        DWORD FirmwareCRC32;
        DWORD Command[ FL7112_NUMBER_OF_REGISTER_COMMAND ];
    } Map;
    DWORD Data[ FL7112_NUMBER_OF_REGISTER_COMMAND + 6 ];
} FL7112_INFORMATION_BLOCK;

typedef union _DWORD_MAPPING_
{
    struct
    {
        DWORD Byte0:8;
        DWORD Byte1:8;
        DWORD Byte2:8;
        DWORD Byte3:8;
    } Map;
    BYTE ByteData[4];
    WORD WordData[2];
    DWORD Value;
} DWORD_MAPPING, *PDWORD_MAPPING;

typedef union
{
    float fvalue;
    unsigned char cvalue[4];
}CharToFloat;

enum fw_upgrade_status {
  	UPDATE_SUCCESS = 0,
  	UPDATE_RUNNING = 1,
  	UPDATE_FAILED = 2,
  	UPDATE_UNKNOWN = 3,
  	UPDATE_SUBFAILED = 4,
};
#endif /* __FL7112_H_ */
