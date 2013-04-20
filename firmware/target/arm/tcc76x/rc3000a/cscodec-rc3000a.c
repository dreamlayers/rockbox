/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id: wmcodec-s5l8700.c 22025 2009-07-25 00:49:13Z dave $
 *
 * S5L8702-specific code for Cirrus codecs
 *
 * Copyright (c) 2010 Michael Sparmann
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

#include "system.h"
#include "audiohw.h"
#include "i2c.h"
#include "cscodec.h"

void audiohw_init(void)
{
    audiohw_preinit();
}

unsigned char cscodec_read(int reg)
{
    unsigned char data;
    i2c_readmem(0x94, reg, &data, 1);
    return data;
}

void cscodec_write(int reg, unsigned char data)
{
    i2c_writemem(0x94, reg, &data, 1);
    //sleep(1);
}

void cscodec_power(bool state)
{
    // FIXME: might be reset, not power
    if (state) {

        GPIOD |= 0x100000; // For CODEC FIXME: might be reset, not power
        GPIOA |= 0x400; // This enables power to speaker amplifier
        //GPIOD &= ~0x40000; // This messes with the CODEC and makes register writes unreliable!
    } else {
        GPIOD &= ~0x100000;
    }
}

void cscodec_reset(bool state)
{
#if 0
    if (state) PDAT(3) &= ~8;
    else PDAT(3) |= 8;
#endif
}

void cscodec_clock(bool state)
{
#if 0
    if (state) CLKCON3 &= ~0xffff;
    else CLKCON3 |= 0x8000;
#endif
}
