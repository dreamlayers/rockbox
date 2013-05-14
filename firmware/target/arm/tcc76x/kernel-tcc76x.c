/***************************************************************************
*             __________               __   ___.
*   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
*   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
*   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
*   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
*                     \/            \/     \/    \/            \/
* $Id$
*
* Copyright (C) 2008 by Rob Purchase
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
#include "system.h"
#include "kernel.h"
#include "timer.h"
#include "thread.h"

void tick_start(unsigned int interval_in_ms)
{
    /* disable Timer0 */
    TCFG0 &= ~TCFGn_EN;

    /* Counter counts from 0 to TREF0 (TREF0 + 1 steps */
    TREF0 = (interval_in_ms * 1000) - 1;

    /* Timer0 = reset to 0, TCLK/2, IRQ enable, enable, continuous */
    TCFG0 = TCFGn_CC | TCFGn_TCKSEL(0) | TCFGn_IEN | TCFGn_EN;

    /* Unmask timer IRQ */
    IEN |= TC_IRQ_MASK;
}

/* NB: Since we are using a single timer IRQ, tick tasks are dispatched as
       part of the central timer IRQ processing in timer-tcc76x.c */
