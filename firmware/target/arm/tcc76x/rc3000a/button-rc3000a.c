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

void button_init_device(void)
{
    CFGPUA |= 0x8000; // Is this needed?
}

int button_read_device(void)
{
    int btn = BUTTON_NONE;
    int btnadc = adc_read(0);

    if ((GPIOA & 0x8000) == 0) btn |= BUTTON_POWER;

    if (btnadc <= 0x82) btn |= BUTTON_UP; /* 0x5A */
    else if (btnadc <= 0xBE) btn |= BUTTON_DOWN; /* 0x9B */
    else if (btnadc <= 0xFA); /* nothing */
    else if (btnadc <= 0x140) btn |= BUTTON_SOURCE; /* 0x11F */
    else if (btnadc <= 0x190); /* nothing */
    else if (btnadc <= 0x208) btn |= BUTTON_MENU; /* 0x1BC */
    else if (btnadc <= 0x28A) btn |= BUTTON_RECORD; /* 0x244 */
    else if (btnadc <= 0x30C) btn |= BUTTON_RIGHT; /* 0x2C4 */
    else if (btnadc <= 0x384) btn |= BUTTON_LEFT; /* 0x340 */
    /* else if (btnadc <= 0x3CF); nothing */
    return btn;
}

bool button_hold(void)
{
    return 0;
}
