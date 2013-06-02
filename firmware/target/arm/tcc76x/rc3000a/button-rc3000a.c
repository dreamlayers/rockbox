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
#include "cpu.h"
#include "adc.h"
#include "button.h"
#include "kernel.h"

void button_init_device(void)
{
    CFGPUA |= 0x8000; // Is this needed?
}

int button_read_device(void)
{
    int btn = BUTTON_NONE;
    int btnadc = adc_read(0);

    if ((GPIOA & 0x8000) == 0) btn |= BUTTON_PLAY; // BUTTON_POWER;

    if (btnadc <= 0x82) btn |= BUTTON_UP; /* 0x5A */
    else if (btnadc <= 0xBE) btn |= BUTTON_DOWN; /* 0x9B */
    else if (btnadc <= 0xFA); /* nothing */
    else if (btnadc <= 0x140) btn |= BUTTON_ENTER; // BUTTON_SOURCE; /* 0x11F */
    else if (btnadc <= 0x190); /* nothing */
    else if (btnadc <= 0x208) btn |= BUTTON_MENU; /* 0x1BC */
    else if (btnadc <= 0x28A) btn |= BUTTON_REC; // ORD; /* 0x244 */
    else if (btnadc <= 0x30C) btn |= BUTTON_RIGHT; /* 0x2C4 */
    else if (btnadc <= 0x384) btn |= BUTTON_LEFT; /* 0x340 */
    /* else if (btnadc <= 0x3CF); nothing */
    return btn;
}

bool button_hold(void)
{
    return 0;
}

#ifdef HAVE_HEADPHONE_DETECTION
bool headphones_inserted(void)
{
    /* Starts off true because speaker amp is off */
    static bool last_detect = true;
    static bool updated = true;
    static long debounce_timeout;
    bool detect;

    detect = (GPIOD & 0x80000) == 0;

    /* Debouncing for speaker amp power switching.
     * TODO: Implement this as Rockbox feature */
    if (detect != last_detect) {
        debounce_timeout = current_tick + HZ/2;
        updated = false;
        last_detect = detect;
    } else if (!updated) {
        if (TIME_AFTER(current_tick, debounce_timeout)) {
            if (detect) {
                /* Headphones inserted, shutdown speaker amp */
                GPIOA &= ~0x400;
            } else {
                /* Headphones removed, power speaker amp */
                GPIOA |= 0x400;
            }
            updated = true;
        }
    }

    return detect;
}
#endif /* HAVE_HEADPHONE_DETECTION */
