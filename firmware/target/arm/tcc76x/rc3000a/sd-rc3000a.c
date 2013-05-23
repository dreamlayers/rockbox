/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2004 by Jens Arnold
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include <stdbool.h>
#include "sd.h"
#include "sdmmc.h"
#include "ata_mmc.h"
#include "ata_idle_notify.h"
#include "kernel.h"
#include "thread.h"
#include "led.h"
#include "system.h"
#include "debug.h"
#include "panic.h"
#include "usb.h"
#include "power.h"
#include "string.h"
#include "hwcompat.h"
#include "adc.h"
#include "bitswap.h"
#include "disk.h" /* for mount/unmount */
#include "storage.h"

#undef CONFIG_STORAGE
#define CONFIG_STORAGE STORAGE_MMC
#include "sdmmc.h"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define BLOCK_SIZE  512   /* fixed */


/* GSCR setting used after initialization
 * With GCLKmode = 0x100, this gives
 * 192 / (2 * 1 + 2) / 2 = 24 MHz
 *
 * OF instead uses 0xDC00400A, which violates requirement for DIV > 0.
 * With GCLKmode = 0x120, that's presumably 192 / 2 / (2 * 0 + 2) / 2 = 24 MHz
 * OF sometimes uses GCLKmode = 0x11A based on CSD
 *
 * FIXME: Can all SD cards handle 24 MHz?
 */
#define SD_GSCR (GSCR_EN | GSCR_MS | GSCR_WORD(7) | GSCR_DIV(1) | \
                 GSCR_DELAY(1) | GSCR_FRM2(10))

/* DMA would be nice here, but it does not seem possible on TCC760.
   A transfer can be triggered using CHCTRL_TYPE_HARDWARE, but it
   does not seem to synchronize with GSIO and runs too fast.
 */

/* Response formats:
   R1  = single byte, msb=0, various error flags
   R1b = R1 + busy token(s)
   R2  = 2 bytes (1st byte identical to R1), additional flags
   R3  = 5 bytes (R1 + OCR register)
*/

#define R2_OUT_OF_RANGE  0x80
#define R2_ERASE_PARAM   0x40
#define R2_WP_VIOLATION  0x20
#define R2_CARD_ECC_FAIL 0x10
#define R2_CC_ERROR      0x08
#define R2_ERROR         0x04
#define R2_ERASE_SKIP    0x02
#define R2_CARD_LOCKED   0x01

/* Data start tokens */

#define SD_SPI_START_BLOCK               0xfe
#define SD_SPI_START_WRITE_MULTIPLE      0xfc
#define SD_SPI_STOP_TRAN                 0xfd

/* Reponse types supported by sd_command() */
#define SD_SPI_RESP_R1 1
#define SD_SPI_RESP_R2 2
#define SD_SPI_RESP_R3 4
#define SD_SPI_RESP_R7 4 /* Also a big endian long, like R3 */

/* for compatibility */
static long last_disk_activity = -1;

/* private variables */

static struct mutex mmc_mutex SHAREDBSS_ATTR;

#ifdef HAVE_HOTSWAP
static long mmc_stack[((DEFAULT_STACK_SIZE*2) + 0x800)/sizeof(long)];
#else
static long mmc_stack[(DEFAULT_STACK_SIZE*2)/sizeof(long)];
#endif
static const char mmc_thread_name[] = "mmc";
static struct event_queue mmc_queue SHAREDBSS_ATTR;
static bool initialized = false;
static bool new_mmc_circuit;

static enum {
    SER_POLL_WRITE,
    SER_POLL_READ,
    SER_DISABLED
} serial_mode;

static const unsigned char dummy[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* 2 buffers used alternatively for writing, including start token,
 * dummy CRC and an extra byte to keep word alignment. */
static unsigned char write_buffer[2][BLOCK_SIZE+4];
static int current_buffer = 0;

static tCardInfo card_info[2];
#ifndef HAVE_MULTIDRIVE
static int current_card = 0;
#endif
static bool last_mmc_status = false;
static int countdown = -1; /* for mmc switch debouncing. -1 because the
                              countdown should not happen if the card
                              is inserted at boot */
static bool usb_activity;    /* monitoring the USB bridge */
static long last_usb_activity;

/* private function declarations */

static int select_card(int card_no);
static void deselect_card(void);
//static void setup_sci1(int bitrate_register);
//static void set_sci1_poll_read(void);
static void write_transfer(const unsigned char *buf, int len)
            __attribute__ ((section(".icode")));
static void read_transfer(unsigned char *buf, int len)
            __attribute__ ((section(".icode")));
static unsigned char poll_byte(void);
static unsigned char poll_busy(void);
static int receive_cxd(unsigned char *buf);
static int initialize_card(int card_no);
static int ICODE_ATTR receive_block(unsigned char *inbuf);
static int ICODE_ATTR send_block(const unsigned char *buf,
                                 unsigned char start_token);
//static void mmc_tick(void);

/* TCC76x implementation */

//#include <stdio.h> // For printf

/* GPIOA pins used for SD card in SPI mode */
#define SD_DI 1 /* Serial data sent to card, GSIO0 Data Out */
#define SD_CLK 2 /* Clock for communication with card, GSIO0 Clock */
#define SD_CS 4 /* Chip select for card */
#define SD_DO 8 /* Serial data received from card, GSIO0 Data In */

static inline void sd_gpio_init(void) {
    /* DI, CLK and DO use GSIO, and CS is a GPIO output */
    GSEL_A = (GSEL_A & ~SD_CS) | (SD_DI | SD_CLK | SD_DO);
    GTSEL_A &= ~(SD_DI | SD_CLK | SD_CS | SD_DO);
    GPIOA_DIR = (GPIOA_DIR & ~SD_DO) | (SD_DI|SD_CLK|SD_CS);
    GPIOA |= SD_DI | SD_CLK | SD_CS;
}

static inline void sd_write_byte(unsigned char byte) {
    GSDO0 = byte;
    while (GSGCR & GSGCR_Busy0);
#if 0
    unsigned char mask;

    for (mask = 0x80; mask > 0; mask >>= 1) {
        if (byte & mask) {
            GPIOA |= SD_DI;
        } else {
            GPIOA &= ~SD_DI;
        }
        GPIOA &= ~SD_CLK;
        GPIOA |= SD_CLK;
    }
#endif
}

static inline unsigned char sd_read_byte(void) {
    GSDO0 = 0xFF;
    while (GSGCR & GSGCR_Busy0);
    return GSDI0;
#if 0
    unsigned char byte = 0, mask;
    GPIOA |= SD_DI;
    for (mask = 0x80; mask > 0; mask >>= 1) {
        GPIOA &= ~SD_CLK;
        GPIOA |= SD_CLK;
        //asm("nop; nop; nop; nop; nop;");
        if (GPIOA & SD_DO) {
            byte |= mask;
        }
    }
    return byte;
#endif
}

/* implementation */

void mmc_enable_int_flash_clock(bool on)
{
#if 0
    /* Internal flash clock is enabled by setting PA12 high with the new
     * clock circuit, and by setting it low with the old clock circuit */
    if (on ^ new_mmc_circuit)
        and_b(~0x10, &PADRH);     /* clear clock gate PA12 */
    else
        or_b(0x10, &PADRH);       /* set clock gate PA12 */
#endif
}

static int select_card(int card_no)
{
    mutex_lock(&mmc_mutex);
    led(true);
    last_disk_activity = current_tick;

    //mmc_enable_int_flash_clock(card_no == 0);

    if (!card_info[card_no].initialized)
    {
        /* Initial rate <= 400 kHz for init */

        /* OF uses GCLKmode = 0x120 and GSCR0 = 0xDD3C0008
         * giving SCLK speed 192 / 2 / (2 * 79 + 2) / 2 : 300 kHz
         * It violates "DELAY should not be set to 0." */

        /* Use PLL frequency for GSCLK (192 MHz) */
        DIVMODE &= ~DIVMODE_DVMGSIO;
        GCLKmode |= 0x100; /* OF uses 0x120 for PLL/2 = 96 MHz */
        CKCTRL &= ~CKCTRL_GCK;

        /* 192 / (2 * 127 + 2) / 2 : 375 kHz */
        GSCR0 = GSCR_EN | GSCR_MS | GSCR_WORD(7) | \
                GSCR_DIV(127) | GSCR_DELAY(1) | GSCR_FRM2(7);

        /* At least 74 clock cycles with CS high for synchronization */
        GPIOA |= SD_CS;
        write_transfer(dummy, 10);
        //while (!(SSR1 & SCI_TEND));
    }

    GPIOA &= ~SD_CS; /* assert CS */

    if (card_info[card_no].initialized)
    {
        //setup_sci1(card_info[card_no].bitrate_register);
        return 0;
    }
    else
    {
        return initialize_card(card_no);
    }
}

static void deselect_card(void)
{
    //while (!(SSR1 & SCI_TEND));   /* wait for end of transfer */
    GPIOA |= SD_CS;           /* deassert CS */

    led(false);
    mutex_unlock(&mmc_mutex);
    last_disk_activity = current_tick;
}

static void write_transfer(const unsigned char *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) sd_write_byte(buf[i]);
#if 0
    const unsigned char *buf_end = buf + len;
    register unsigned char data;

    if (serial_mode != SER_POLL_WRITE)
    {
        while (!(SSR1 & SCI_TEND)); /* wait for end of transfer */
        SCR1 = 0;                   /* disable transmitter & receiver */
        SSR1 = 0;                   /* clear all flags */
        SCR1 = SCI_TE;              /* enable transmitter only */
        serial_mode = SER_POLL_WRITE;
    }

    while (buf < buf_end)
    {
        data = fliptable[(signed char)(*buf++)]; /* bitswap */
        while (!(SSR1 & SCI_TDRE));              /* wait for end of transfer */
        TDR1 = data;                             /* write byte */
        SSR1 = 0;                                /* start transmitting */
    }
#endif
//#warning write_transfer not implemented
}

/* don't call this with len == 0 */
static void read_transfer(unsigned char *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) buf[i] = sd_read_byte();
#if 0
    unsigned char *buf_end = buf + len - 1;
    register signed char data;

    if (serial_mode != SER_POLL_READ)
        set_sci1_poll_read();

    SSR1 = 0;                     /* start receiving first byte */
    while (buf < buf_end)
    {
        while (!(SSR1 & SCI_RDRF)); /* wait for data */
        data = RDR1;                /* read byte */
        SSR1 = 0;                   /* start receiving */
        *buf++ = fliptable[data];   /* bitswap */
    }
    while (!(SSR1 & SCI_RDRF));     /* wait for last byte */
    *buf = fliptable[(signed char)(RDR1)]; /* read & bitswap */
#endif
//#warning read_transfer not implemented
}

/* returns 0xFF on timeout */
static unsigned char poll_byte(void)
{
    long timeout = current_tick + HZ/10;

    unsigned char data;
    do {
        data = sd_read_byte();
    } while ((data == 0xFF) && !TIME_AFTER(current_tick, timeout));
    return data;
#if 0
    long i;
    unsigned char data = 0;       /* stop the compiler complaining */

    if (serial_mode != SER_POLL_READ)
        set_sci1_poll_read();

    i = 0;
    do {
        SSR1 = 0;                   /* start receiving */
        while (!(SSR1 & SCI_RDRF)); /* wait for data */
        data = RDR1;                /* read byte */
    } while ((data == 0xFF) && (++i < timeout));

    return fliptable[(signed char)data];
#endif
//#warning poll_byte not implemented
}

/* returns 0 on timeout */
static unsigned char poll_busy(void)
{
    unsigned char data, dummy;

    data = sd_read_byte();

    long timeout = current_tick + HZ/2;
    do {
        dummy = sd_read_byte();
    } while ((dummy != 0xFF) && !TIME_AFTER(current_tick, timeout));

    //intf("%02x %02x %d", dummy, data, current_tick - timeout);

    return (dummy == 0xFF) ? data : 0;
#if 0
    long i;
    unsigned char data, dummy;

    if (serial_mode != SER_POLL_READ)
        set_sci1_poll_read();

    /* get data response */
    SSR1 = 0;                     /* start receiving */
    while (!(SSR1 & SCI_RDRF));   /* wait for data */
    data = fliptable[(signed char)(RDR1)];  /* read byte */

    /* wait until the card is ready again */
    i = 0;
    do {
        SSR1 = 0;                   /* start receiving */
        while (!(SSR1 & SCI_RDRF)); /* wait for data */
        dummy = RDR1;               /* read byte */
    } while ((dummy != 0xFF) && (++i < timeout));

    return (dummy == 0xFF) ? data : 0;
#endif
}

/* Send MMC command and get response. Returns R1 byte directly.
 * Returns further R2 or R3 bytes in *data (can be NULL for other commands) */
static unsigned char sd_command(int cmd, unsigned long parameter,
                                unsigned long resptype, void *data)
{
    static struct {
        unsigned char cmd;
        unsigned long parameter;
        const unsigned char crc7;    /* fixed, valid for CMD0 only */
        const unsigned char trailer;
    } __attribute__((packed)) command = {0x40, 0, 0x95, 0xFF};

    unsigned char ret;

    command.cmd = cmd + 0x40;
    //printf("cmd: %d", command.cmd);
    command.parameter = htobe32(parameter);

    write_transfer((unsigned char *)&command, sizeof(command));

    ret = poll_byte();

    switch (resptype) {
    case SD_SPI_RESP_R1:
        break;
    case SD_SPI_RESP_R2:
        read_transfer(data, 1);
        break;
    case SD_SPI_RESP_R3:
    /* duplicate case SD_SPI_RESP_R7: */
        read_transfer(data, 4);
        *(unsigned long *)data = betoh32(*(unsigned long *)data);
        break;
    default:
        return ret;
    }

    write_transfer(dummy, 1);
    return ret;
}

/* Receive CID/ CSD data (16 bytes) */
static int receive_cxd(unsigned char *buf)
{
    if (poll_byte() != SD_SPI_START_BLOCK)
    {
        write_transfer(dummy, 1);
        return -1;                /* not start of data */
    }

    read_transfer(buf, 16);
    write_transfer(dummy, 3);     /* 2 bytes dontcare crc + 1 byte trailer */
    return 0;
}

static int initialize_card(int card_no)
{
    int rc, i;
    tCardInfo *card = &card_info[card_no];
    int r;
    unsigned long response;
    bool sd_v2 = false;
    long init_timeout;

    memset(card, sizeof(*card), 0);

    /* Switch to SPI mode (assuming CS already 0) */
    if (sd_command(SD_GO_IDLE_STATE, 0, SD_SPI_RESP_R1, NULL)
        != SD_SPI_R1_IDLE_STATE)
        return -1;                /* error or no response */

    /* Mandatory command for v2 hosts */
    /* Non v2 cards will not respond to this command */
    if(sd_command(SD_SEND_IF_COND, 0x1AA, SD_SPI_RESP_R7, &response)
       != SD_SPI_R1_IDLE_STATE)
        if((response & 0xFFF) == 0x1AA)
            sd_v2 = true;

    /* Optional command to get voltage range */
    if (sd_command(SD_SPI_READ_OCR, 0, SD_SPI_RESP_R3, &card->ocr)
        != SD_SPI_R1_IDLE_STATE)
        return -2;

    if (!(card->ocr & 0x00100000)) /* 3.2 .. 3.3 V */
        return -3;

    /* Timeout for initialization is 1sec, from SD Specification 2.00 */
    init_timeout = current_tick + HZ;

    while (1)
    {
        if(TIME_AFTER(current_tick, init_timeout))
            return -4;

        if (sd_command(SD_APP_CMD, card->rca, SD_SPI_RESP_R1, NULL)
            & SD_SPI_R1_CARD_ERROR)
            return -5;

        response = sd_command(SD_APP_OP_COND,
                              (1 << 20) /* 3.2-3.3V */ |
                              (1 << 21) /* 3.3-3.4V */ |
                              (sd_v2 ? (1 << 30) : 0),
                              SD_SPI_RESP_R1, NULL);

        if (response == 0)
            break; /* Finished idle/initialization mode */
        else if (response & SD_SPI_R1_CARD_ERROR)
            return -6;
    }

    /* Mandatory command to get CCS */
    if (sd_command(SD_SPI_READ_OCR, 0, SD_SPI_RESP_R3, &card->ocr))
        return -7;

    /* End of card identification mode. Entering data transfer mode. */

    /* switch to full speed */
// FIXME 24 MHz for now, FIXME where to put this
    GSCR0 = SD_GSCR;

    /* get CID register */
    if (sd_command(SD_SEND_CID, 0, SD_SPI_RESP_R1, NULL))
        return -8;
    rc = receive_cxd((unsigned char*)card->cid);
    if (rc)
        return rc * 10 - 8;

    /* get CSD register */
    if (sd_command(SD_SEND_CSD, 0, SD_SPI_RESP_R1, NULL))
        return -9;
    rc = receive_cxd((unsigned char*)card->csd);
    if (rc)
        return rc * 10 - 5;

    sd_parse_csd(card);

    /* always use 512 byte blocks (probably unnecessary) */
    if (sd_command(SD_SET_BLOCKLEN, BLOCK_SIZE, SD_SPI_RESP_R1, NULL))
        return -7;

    card->initialized = true;
    return 0;
}

/* Receive one block */
static int ICODE_ATTR receive_block(unsigned char *inbuf)
{
    if (poll_byte() != SD_SPI_START_BLOCK)
    {
        write_transfer(dummy, 1);
        return -1;                /* not start of data */
    }

    /* Temporarily use 16 bit transfers */
    GSCR0 = SD_GSCR | GSCR_WORD(15);

    register unsigned char *p = inbuf;
    register unsigned char *pend = inbuf + BLOCK_SIZE - 2;
    register unsigned long v;

    /* First part of loop unrolled for optimization */
    GSDO0 = 0xFFFF;
    goto recv_blk_loop_entry;

    /* Loop structured so write and test happen while waiting for busy */
    do {
        *p++ = v >> 8;
        *p++ = v;
        /* Last iteration will discard CRC */
recv_blk_loop_entry:
        while (GSGCR & GSGCR_Busy0);
        v = GSDI0;
        GSDO0 = 0xFFFF;
    } while (p < pend);

    /* End of unrolled loop */
    *p++ = v >> 8;
    *p++ = v;

    while (GSGCR & GSGCR_Busy0); /* Waiting for CRC */
    GSCR0 = SD_GSCR;
    GSDO0 = 0xFF;
    while (GSGCR & GSGCR_Busy0);

    return 0;
}

/* Send one block with DMA from the current write buffer, possibly preparing
 * the next block within the next write buffer in the background. */
static int ICODE_ATTR send_block(const unsigned char *buf,
                                 unsigned char start_token)
{
    int rc = 0;

    /* Send start token */
    GSDO0 = start_token;

#if 0
    register const unsigned char *p = buf;
    register const unsigned char *pend = buf + BLOCK_SIZE;
    register unsigned long v;

    /* First part of loop unrolled for optimization, reading first word */
    v = *(p++) << 8;
    v |= *(p++);

    /* Wait for busy after start token */
    while (GSGCR & GSGCR_Busy0);

    /* Temporarily use 16 bit transfers */
    GSCR0 = SD_GSCR | GSCR_WORD(15);

    /* Loop structured so read and test happen while waiting for busy */
    do {
        while (GSGCR & GSGCR_Busy0);
        //FIXME 16 bit transfers don't work without this delay
        //printf("%04x", GSDI0);
        GSDO0 = v;

        v = *(p++) << 8;
        v |= *(p++);
    } while (p < pend);

    /* End of unrolled loop, sending last word */
    while (GSGCR & GSGCR_Busy0);
    GSDO0 = v;

    /* Send dummy CRC */
    GSDO0 = 0xFFFF;
    while (GSGCR & GSGCR_Busy0); /* Waiting for CRC */

    /* Reset GSCR to normal value */
    GSCR0 = SD_GSCR;
#else
    register const unsigned char *pend = buf + BLOCK_SIZE;
    register const unsigned char *p = buf;

    /* Wait for busy after start token */
    while (GSGCR & GSGCR_Busy0);
    GSDO0 = *(p++);
    do {
        while (GSGCR & GSGCR_Busy0);
        GSDO0 = *(p++);
    } while (p < pend);

    /* Wait for busy after last byte */
    while (GSGCR & GSGCR_Busy0);

    /* Send CRC */
    sd_write_byte(0xFF);
    sd_write_byte(0xFF);
#endif
    if ((poll_busy() & 0x1F) != 0x05) /* something went wrong */
        rc = -1;

    GSDO0 = 0xFF;
    while (GSGCR & GSGCR_Busy0);

    last_disk_activity = current_tick;

    return rc;
#if 0
    int rc = 0;
    unsigned char *curbuf = write_buffer[current_buffer];

    curbuf[1] = fliptable[(signed char)start_token];
    *(unsigned short *)(curbuf + BLOCK_SIZE + 2) = 0xFFFF;

    while (!(SSR1 & SCI_TEND));   /* wait for end of transfer */

    SCR1 = 0;                     /* disable serial */
    SSR1 = 0;                     /* clear all flags */

    /* setup DMA channel 0 */
    CHCR0 = 0;                    /* disable */
    SAR0 = (unsigned long)(curbuf + 1);
    DAR0 = TDR1_ADDR;
    DTCR0 = BLOCK_SIZE + 3;       /* start token + block + dummy crc */
    CHCR0 = 0x1701;               /* fixed dest. address, TXI1, enable */
    DMAOR = 0x0001;
    SCR1 = (SCI_TE|SCI_TIE);      /* kick off DMA */

    if (prepare_next)
        send_block_prepare();
    yield();                      /* be nice */

    while (!(CHCR0 & 0x0002));    /* wait for end of DMA */
    while (!(SSR1 & SCI_TEND));   /* wait for end of transfer */
    SCR1 = 0;
    serial_mode = SER_DISABLED;

    if ((poll_busy() & 0x1F) != 0x05) /* something went wrong */
        rc = -1;

    write_transfer(dummy, 1);
    last_disk_activity = current_tick;

    return rc;
#endif
}

int sd_read_sectors(IF_MD(int drive,)
                    unsigned long start,
                    int incount,
                    void* inbuf)
{
    int rc = 0;
    int lastblock = 0;
    unsigned long end_block;
    tCardInfo *card;
#ifndef HAVE_MULTIDRIVE
    int drive = current_card;
#endif

    card = &card_info[drive];

    rc = select_card(drive);
    if (rc)
    {
        rc = rc * 10 - 1;
        goto error;
    }

    end_block = start + incount;
    if (end_block > card->numblocks)
    {
        rc = -2;
        goto error;
    }

    /* Some cards don't like reading the very last block with
     * CMD_READ_MULTIPLE_BLOCK, so make sure this block is always
     * read with CMD_READ_SINGLE_BLOCK. */
    if (end_block == card->numblocks)
        lastblock = 1;

    if (incount > 1)
    {
        /* MMC4.2: make multiplication conditional */
        if (sd_command(SD_READ_MULTIPLE_BLOCK, start * BLOCK_SIZE, 0, NULL))
        {
            rc =  -3;
            goto error;
        }
        while (--incount >= lastblock)
        {
            rc = receive_block(inbuf);
            if (rc)
            {
                /* If an error occurs during multiple block reading, the
                 * host still needs to send CMD_STOP_TRANSMISSION */
                /* FIXME actually R1b */
                sd_command(SD_STOP_TRANSMISSION, 0, SD_SPI_RESP_R1, NULL);
                rc = rc * 10 - 4;
                goto error;
            }
            inbuf += BLOCK_SIZE;
            start++;
            /* ^^ necessary for the abovementioned last block special case */
        }
        if (sd_command(SD_STOP_TRANSMISSION, 0, SD_SPI_RESP_R1, NULL))
        {
            rc = -5;
            goto error;
        }
    }
    if (incount > 0)
    {
        /* MMC4.2: make multiplication conditional */
        if (sd_command(SD_READ_SINGLE_BLOCK, start * BLOCK_SIZE, 0, NULL))
        {
            rc = -6;
            goto error;
        }
        rc = receive_block(inbuf);
        if (rc)
        {
            rc = rc * 10 - 7;
            goto error;
        }
    }

  error:

    deselect_card();
#if 0
    if (rc != 0) printf("X %d, %d, %d", rc, start, incount);
    else printf("Y %d, %d, %d", rc, start, incount);
#endif
    return rc;
}

int sd_write_sectors(IF_MD(int drive,)
                     unsigned long start,
                     int count,
                     const void* buf)
{
    int rc = 0;
    int write_cmd;
    unsigned char start_token;
    tCardInfo *card;
#ifndef HAVE_MULTIDRIVE
    int drive = current_card;
#endif

    card = &card_info[drive];
    rc = select_card(drive);
    if (rc)
    {
        rc = rc * 10 - 1;
        goto error;
    }

    if (start + count  > card->numblocks)
        panicf("Writing past end of card");

    if (count > 1)
    {
        write_cmd   = SD_WRITE_MULTIPLE_BLOCK;
        start_token = SD_SPI_START_WRITE_MULTIPLE;
    }
    else
    {
        write_cmd   = SD_WRITE_BLOCK;
        start_token = SD_SPI_START_BLOCK;
    }
                 /* MMC4.2: make multiplication conditional */
    if (sd_command(write_cmd, start * BLOCK_SIZE, SD_SPI_RESP_R1, NULL))
    {
        rc = -2;
        goto error;
    }
    while (--count >= 0)
    {
        rc = send_block(buf, start_token);
        if (rc)
        {
            rc = rc * 10 - 3;
            break;
            /* If an error occurs during multiple block writing,
             * the STOP_TRAN token still needs to be sent. */
        }
        buf += BLOCK_SIZE;
    }
    if (write_cmd == SD_WRITE_MULTIPLE_BLOCK)
    {
        static const unsigned char stop_tran = SD_SPI_STOP_TRAN;
        write_transfer(&stop_tran, 1);
        poll_busy();
    }

  error:

    deselect_card();

    return rc;
}

bool mmc_disk_is_active(void)
{
#if 0
    /* this is correct unless early return from write gets implemented */
    return mutex_test(&mmc_mutex);
#endif //FIXME
    return true;
}

#if 0
static void mmc_thread(void)
{
    struct queue_event ev;
    bool idle_notified = false;

    while (1) {
        queue_wait_w_tmo(&mmc_queue, &ev, HZ);
        switch ( ev.id )
        {
            case SYS_USB_CONNECTED:
                usb_acknowledge(SYS_USB_CONNECTED_ACK);
                /* Wait until the USB cable is extracted again */
                usb_wait_for_disconnect(&mmc_queue);
                break;

#ifdef HAVE_HOTSWAP
            case SYS_HOTSWAP_INSERTED:
                disk_mount(1); /* mount MMC */
                queue_broadcast(SYS_FS_CHANGED, 0);
                break;

            case SYS_HOTSWAP_EXTRACTED:
                disk_unmount(1); /* release "by force" */
                queue_broadcast(SYS_FS_CHANGED, 0);
                break;
#endif

            default:
                if (TIME_BEFORE(current_tick, last_disk_activity+(3*HZ)))
                {
                    idle_notified = false;
                }
                else
                {
                    if (!idle_notified)
                    {
                        call_storage_idle_notifys(false);
                        idle_notified = true;
                    }
                }
                break;
        }
    }
}
#endif

bool mmc_detect(void)
{
    return true; // FIXME (adc_read(ADC_MMC_SWITCH) < 0x200);
}

bool mmc_usb_active(int delayticks)
{
    /* reading "inactive" is delayed by user-supplied monoflop value */
    return (usb_activity ||
            TIME_BEFORE(current_tick, last_usb_activity + delayticks));
}

#if 0
static void mmc_tick(void)
{
    bool current_status;


    if (new_mmc_circuit)
        /* USB bridge activity is 0 on idle, ~527 on active */
        current_status = adc_read(ADC_USB_ACTIVE) > 0x100;
    else
        current_status = adc_read(ADC_USB_ACTIVE) < 0x190;

    if (!current_status && usb_activity)
        last_usb_activity = current_tick;
    usb_activity = current_status;

    current_status = mmc_detect();
    /* Only report when the status has changed */
    if (current_status != last_mmc_status)
    {
        last_mmc_status = current_status;
        countdown = HZ/3;
    }
    else
    {
        /* Count down until it gets negative */
        if (countdown >= 0)
            countdown--;

        if (countdown == 0)
        {
            if (current_status)
            {
                queue_broadcast(SYS_HOTSWAP_INSERTED, 0);
            }
            else
            {
                queue_broadcast(SYS_HOTSWAP_EXTRACTED, 0);
                card_info[1].initialized = false;
            }
        }
    }
}
#endif

void sd_enable(bool on)
{
#if 0
    PBCR1 &= ~0x0CF0;      /* PB13, PB11 and PB10 become GPIO,
                            * if not modified below */
    if (on)
        PBCR1 |= 0x08A0;   /* as SCK1, TxD1, RxD1 */

    and_b(~0x80, &PADRL);  /* assert flash reset */
    sleep(HZ/100);
    or_b(0x80, &PADRL);    /* de-assert flash reset */
    sleep(HZ/100);
    card_info[0].initialized = false;
    card_info[1].initialized = false;
#endif // FIXME
//#warning mmc_enable not implemented
}

int sd_init(void)
{
    int rc = 0;

    if (!initialized)
    {
        mutex_init(&mmc_mutex);
        queue_init(&mmc_queue, true);
    }
    mutex_lock(&mmc_mutex);
    led(false);

    sd_gpio_init();

#if 0
    last_mmc_status = mmc_detect();
#ifndef HAVE_MULTIDRIVE
    /* Use MMC if inserted, internal flash otherwise */
    current_card = last_mmc_status ? 1 : 0;
#endif

    if (!initialized)
    {
        /* Port setup */
        PACR1 &= ~0x0F3C;  /* GPIO function for PA13 (flash busy), PA12
                            * (clk gate), PA10 (flash CS), PA9 (MMC CS) */
        PACR2 &= ~0x4000;  /* GPIO for PA7 (flash reset) */
        PADR  |=  0x0680;  /* set all the selects + reset high (=inactive) */
        PAIOR |=  0x1680;  /* make outputs for them and the PA12 clock gate */

        PBCR1 &= ~0x0CF0;  /* GPIO function for PB13, PB11 and PB10 */
        PBDR  |=  0x2C00;  /* SCK1, TxD1 and RxD1 high in GPIO */
        PBIOR |=  0x2000;  /* SCK1 output */
        PBIOR &= ~0x0C00;  /* TxD1, RxD1 input */

        IPRE  &=  0x0FFF;  /* disable SCI1 interrupts for the CPU */

        new_mmc_circuit = ((HW_MASK & MMC_CLOCK_POLARITY) != 0);

        create_thread(mmc_thread, mmc_stack,
                      sizeof(mmc_stack), 0, mmc_thread_name
                      IF_PRIO(, PRIORITY_SYSTEM)
                      IF_COP(, CPU));
        tick_add_task(mmc_tick);
        initialized = true;
    }
    mmc_enable(true);
#endif

    mutex_unlock(&mmc_mutex);
    return rc;
}

long sd_last_disk_activity(void)
{
    return last_disk_activity;
}

tCardInfo *card_get_info_target(int card_no)
{
    return &card_info[card_no];
}

#ifdef HAVE_HOTSWAP
bool sd_removable(IF_MD_NONVOID(int drive))
{
#ifndef HAVE_MULTIDRIVE
    const int drive=0;
#endif
    return (drive==1);
}

bool sd_present(IF_MD_NONVOID(int drive))
{
#ifndef HAVE_MULTIDRIVE
    const int drive=0;
#endif
    if(drive==0)
    {
        return true;
    }
    else
    {
        return mmc_detect();
    }
}
#endif

#ifdef CONFIG_STORAGE_MULTI
int sd_num_drives(int first_drive)
{
    /* We don't care which logical drive number(s) we have been assigned */
    (void)first_drive;

#ifdef HAVE_MULTIDRIVE
    return 2;
#else
    return 1;
#endif
}
#endif
