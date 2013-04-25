/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 by Dave Chapman
 *
 * Based on Rockbox iriver bootloader by Linus Nielsen Feltzing
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
#include "config.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "cpu.h"
#include "system.h"
#include "lcd.h"
#include "../kernel-internal.h"
#include "storage.h"
#include "file_internal.h"
#include "fat.h"
#include "disk.h"
#include "font.h"
#include "button.h"
#include "adc.h"
#include "adc-target.h"
#include "backlight.h"
#include "backlight-target.h"
#include "panic.h"
#include "power.h"
#include "file.h"
#include "common.h"
#include "rb-loader.h"
#include "loader_strerror.h"
#include "version.h"

// TEMP FIXME
#include "i2c.h"

/* Show the Rockbox logo - in show_logo.c */
extern void show_logo(void);

/* Address to load main Rockbox image to */
#define LOAD_ADDRESS 0x20000000 /* DRAM_START */

extern int line;

#define MAX_LOAD_SIZE (8*1024*1024) /* Arbitrary, but plenty. */

unsigned long data[] = { 0, 0x00001111, 0x00002222, 0x00003333 };

/* The following function is just test/development code */
void show_debug_screen(void)
{
    int button;
    int power_count = 0;
    int count = 0;
    bool do_power_off = false;

        unsigned char b;
        int r;

        unsigned char *buf = 0x8000;
        buf[0] = 0xAA;

        buf[1] = 0xFF;

        printf("DMA: %02x %02x", buf[0], buf[1]);

        ST_SADR0 = &buf[0];
        ST_DADR0 = &buf[1];
        HCOUNT0 = 1;
        CHCTRL0 = CHCTRL_TYPE_SOFTWARE | CHCTRL_WSIZE_8;
        CHCTRL0 |= CHCTRL_EN;

        while ((CHCTRL0 & CHCTRL_FLAG) == 0);

        printf("DMA: %02x %02x", buf[0], buf[1]);
        while(true) core_sleep();
#if 0
#include "cscodec.h"
    audiohw_preinit();
    pcm_play_dma_init();
#endif
            DAMR |= DAMR_TE;
    while (!do_power_off) {
        line = 1;
        button = button_get(false);
        
        /* Power-off if POWER button has been held for a time
           This loop is currently running at about 100 iterations/second
         */
        if (button & POWEROFF_BUTTON) {
            power_count++;
            if (power_count > 100)
               do_power_off = true;
        } else {
            power_count = 0;
        }
#if 0
        if (button & BUTTON_SELECT){
            backlight_hw_off();
        }
        else{
            backlight_hw_on();
        }
#endif
        printf("Btn: 0x%08x",button);
#if 0

        DADO_L(0) = data[0];
        DADO_R(0) = data[0];
        DADO_L(1) = data[1];
        DADO_R(1) = data[1];
        DADO_L(2) = data[2];
        DADO_R(2) = data[2];
        DADO_L(3) = data[3];
        DADO_R(3) = data[3];
#endif
#if 0
    cscodec_write(9,0x61);
    cscodec_write(9,0x97);
    cscodec_write(0x12,0x60);
    cscodec_write(0x13,/*xx*/0x60);
    cscodec_write(0x14,0xC0);
    cscodec_write(9,0x41);
#endif

        b=255;
        r = i2c_readmem(0x94, 0x20, &b, 1);
        printf("S: %d: %d", r, b);
#if 0
        printf("Tick: %d",current_tick);
        printf("GPIOA: 0x%08x",GPIOA);
        printf("GPIOB: 0x%08x",GPIOB);
        printf("GPIOC: 0x%08x",GPIOC);
        printf("GPIOD: 0x%08x",GPIOD);
#ifndef CPU_TCC76X
        printf("GPIOE: 0x%08x",GPIOE);
#endif
#endif

#if 0
        int i;
        for (i = 0; i<4; i++)
        {
            printf("ADC%d: 0x%04x",i,adc_read(i));
        }
#endif
        count++;
        printf("Count: %d",count);
        lcd_update();
        sleep(HZ/10);
        GPIOA &= ~0x800;

    }

    lcd_clear_display();
    line = 0;
    printf("POWER-OFF");

    /* Power-off */
    power_off();

    printf("(NOT) POWERED OFF");
    while (true);
}

void* main(void)
{
#ifdef TCCBOOT
    int rc;
    unsigned char* loadbuffer = (unsigned char*)LOAD_ADDRESS;
#endif

    system_init();
    power_init();
    
    kernel_init();
    enable_irq();
    
    lcd_init();

    adc_init();
    button_init();
    backlight_init();

    font_init();
    lcd_setfont(FONT_SYSFIXED);
    
    show_logo();

    backlight_hw_on();

/* Only load the firmware if TCCBOOT is defined - this ensures SDRAM_START is
   available for loading the firmware. Otherwise display the debug screen. */
#ifdef TCCBOOT
    printf("Rockbox boot loader");
    printf("Version %s", rbversion);

    printf("ATA");
    rc = storage_init();
    if(rc)
    {
        reset_screen();
        error(EATA, rc, true);
    }

    filesystem_init();

    printf("mount");
    rc = disk_mount_all();
    if (rc<=0)
    {
        error(EDISK,rc, true);
    }

    rc = load_firmware(loadbuffer, BOOTFILE, MAX_LOAD_SIZE);

    if (rc <= EFILE_EMPTY)
    {
        error(EBOOTFILE,rc, true);
    }
    else
    {
        int(*kernel_entry)(void) = (void *) loadbuffer;

        disable_irq();
        rc = kernel_entry();
    }

    panicf("Boot failed!");
#else
    show_debug_screen();
#endif

    return 0;
}
