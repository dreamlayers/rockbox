/*
 * This config file is for the Sansa C100 series
 */

#define MODEL_NAME "RCA RC3000A"

/* For Rolo and bootloader */
#define MODEL_NUMBER 30

/* define hardware samples rate caps mask */
#define HW_SAMPR_CAPS   (SAMPR_CAP_8 | SAMPR_CAP_11 | SAMPR_CAP_12 | \
                         SAMPR_CAP_16 | SAMPR_CAP_22 | SAMPR_CAP_24 | \
                         SAMPR_CAP_32 | SAMPR_CAP_44 | SAMPR_CAP_48)

/* define this if you have a bitmap LCD display */
#define HAVE_LCD_BITMAP

/* define this if you can flip your LCD */
/*#define HAVE_LCD_FLIP*/

/* define this if you can invert the colours on your LCD */
/*#define HAVE_LCD_INVERT*/

/* define this if you have access to the quickscreen */
#define HAVE_QUICKSCREEN

/* define this if you would like tagcache to build on this target */
#define HAVE_TAGCACHE

/* define this if you have a flash memory storage */
//#define HAVE_FLASH_STORAGE

/* Only v1 */
//#define CONFIG_STORAGE_MULTI
#define NUM_DRIVES 1
#define CONFIG_STORAGE STORAGE_MMC
//#define CONFIG_NAND NAND_TCC

/* c100's with direct-to-NAND access are FAT16 */
#define HAVE_FAT16SUPPORT

/* LCD dimensions */
#define LCD_WIDTH  128
#define LCD_HEIGHT 64
#define LCD_DEPTH  1

#define LCD_PIXEL_ASPECT_WIDTH 4
#define LCD_PIXEL_ASPECT_HEIGHT 5

#define LCD_PIXELFORMAT VERTICAL_PACKING

/* Display colours, for screenshots and sim (0xRRGGBB) */
// FIXME taken from Archos, need to customize
#define LCD_DARKCOLOR       0x000000
#define LCD_BRIGHTCOLOR     0x5a915a
#define LCD_BL_DARKCOLOR    0x000000
#define LCD_BL_BRIGHTCOLOR  0x7ee57e

/* define this to indicate your device's keypad */
#define CONFIG_KEYPAD MPIO_HD300_PAD // reasonably close match for RCA_RC3000A_PAD

/* The RC3000A has a PCF8563 RTC,
   but it's register compatible with the E8564. */
#ifndef BOOTLOADER
/* This file is not read for use with another CPU */
#define CONFIG_RTC RTC_E8564
#define HAVE_RTC_ALARM
#endif

/* define this if you have RTC RAM available for settings */
//#define HAVE_RTC_RAM

/* Define this if you have a software controlled poweroff */
#define HAVE_SW_POWEROFF

/* The number of bytes reserved for loadable codecs */
#define CODEC_SIZE 0x50000

/* The number of bytes reserved for loadable plugins */
#define PLUGIN_BUFFER_SIZE 0x50000

#define AB_REPEAT_ENABLE

/* Define this if you do software codec */
#define CONFIG_CODEC SWCODEC

/* Define this if you have the CS42L55 audio codec */
#define HAVE_CS42L55

/*#define CONFIG_TUNER TEA5767*/

/* TLV320 has no tone controls, so we use the software ones */
#define HAVE_SW_TONE_CONTROLS

/* Define this for LCD backlight available */
#define HAVE_BACKLIGHT

/* Software I2C is used */
#define HAVE_SOFTWARE_I2C

#define BATTERY_CAPACITY_DEFAULT 540 /* default battery capacity */
#define BATTERY_CAPACITY_MIN 540 /* min. capacity selectable */
#define BATTERY_CAPACITY_MAX 540 /* max. capacity selectable */
#define BATTERY_CAPACITY_INC 50   /* capacity increment */
#define BATTERY_TYPES_COUNT  1    /* only one type */

#define CONFIG_BATTERY_MEASURE VOLTAGE_MEASURE

/* define this if the unit should not shut down on low battery. */
#define NO_LOW_BATTERY_SHUTDOWN

/* Define this if you have a TCC770 */
#define CONFIG_CPU TCC760

/* Define this if you have ATA power-off control */
#define HAVE_ATA_POWER_OFF

/* Define this to the CPU frequency */
#define CPU_FREQ      120000000

/* Offset ( in the firmware file's header ) to the file CRC */
#define FIRMWARE_OFFSET_FILE_CRC 0

/* Offset ( in the firmware file's header ) to the real data */
#define FIRMWARE_OFFSET_FILE_DATA 8

/* The start address index for ROM builds */
/* #define ROM_START 0x11010 for behind original Archos */
// #define ROM_START 0x7010 /* for behind BootBox */

#define CONFIG_LCD LCD_S6B33B2 // FIXME integrate LCD driver LCD_SSD1815

#define BOOTFILE_EXT "c100"
#define BOOTFILE "rockbox." BOOTFILE_EXT
#define BOOTDIR "/.rockbox"

#ifdef BOOTLOADER
//#define TCCBOOT FIXME not appending to OF, so no need for this
#endif

#define IRAM_LCDFRAMEBUFFER IBSS_ATTR /* put the lcd frame buffer in IRAM */

/* Define this if a programmable hotkey is mapped */
#define HAVE_HOTKEY