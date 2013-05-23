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
#ifndef BACKLIGHT_TARGET_H
#define BACKLIGHT_TARGET_H

#include "tcc76x.h"

static inline bool backlight_hw_init(void)
{
    /* Set GPIO pin as output */
    GPIOD_DIR |= 0x8000;
#ifndef BOOTLOADER
    /* With PWM fading, it seems init should turn on backlight */
    GPIOD |= 0x8000;
#endif
    return true;
}

static inline void _backlight_led_on(void)
{
    /* Enable backlight */
    GPIOD |= 0x8000;
}

static inline void _backlight_led_off(void)
{
    /* Disable backlight */
    GPIOD &= ~0x8000;
}

#ifdef BOOTLOADER
/* No PWM fading, and normal backlight switching functions are used. */
#define _backlight_on() _backlight_led_on()
#define _backlight_off() _backlight_led_off()
#else /* !BOOTLOADER */
/* PWM fading functions */
#define _backlight_on_normal() _backlight_led_on()
#define _backlight_on_isr() _backlight_led_on()
#define _backlight_off_normal() _backlight_led_off()
#define _backlight_off_isr() _backlight_led_off()
#endif

#endif
