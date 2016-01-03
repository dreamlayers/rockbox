/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2015 by Boris Gjenero
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
#include "usb.h"
#include "tcc76x.h"
#include "system.h"
#include "panic.h"

#ifdef HAVE_USBSTACK
#include "usb_core.h"

void usb_init_device(void)
{
}

void usb_enable(bool on)
{
    if (on)
        usb_core_init();
    else
        usb_core_exit();
}

void usb_attach(void)
{
   usb_enable(true);
}

int usb_detect(void)
{
    /* TODO: not correct for all targets, we should poll VBUS
       signal on USB bus. */
    //if (charger_inserted())
        return USB_INSERTED;
    //return USB_EXTRACTED;
}
#else /* !HAVE_USBSTACK */
void usb_init_device(void)
{
#if 0
    /* simply switch USB off for now */
    BCLKCTR |= DEV_USBD;
    TCC7xx_USB_PHY_CFG = 0x3e4c;
    BCLKCTR &= ~DEV_USBD;
#endif
}

void usb_enable(bool on)
{
    (void)on;
}

/* Always return false for now */
int usb_detect(void)
{
    /* GPIOA & 0x4000 */
    return USB_EXTRACTED;
}
#endif
