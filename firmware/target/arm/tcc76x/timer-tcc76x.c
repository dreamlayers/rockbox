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
#include "cpu.h"
#include "system.h"
#include "timer.h"
#include "logf.h"

bool timer_set(long cycles, bool start)
{
    /* Table of prescale increases achieved by
     * incrementing TCKSEL starting at 0 */
    static const short prescale_tab[] = {
        1, 1, 1, 1, 5, 2
    };
    int prescale = 0;

    /* Timer 4 is 20 bit, so prescaler may be needed.
     * Maximum supported value is 1 << 31
     * TC32 is 32 bit, but it doesn't seem to work with XIN */
    while (cycles > (1 << 20)) {
        cycles >>= prescale_tab[prescale];
        prescale++;
    }

    if (start && pfn_unregister != NULL) {
        pfn_unregister();
        pfn_unregister = NULL;
    }

    /* disable Timer */
    TCFG4 &= ~TCFGn_EN;

    /* Set prescaler */
    TCFG4 = TCFGn_TCKSEL(prescale);

    /* Counter counts from 0 to cycles */
    TREF4 = cycles - 1;

    if (!start) timer_start(); // FIXME is it possible to not restart?

    return true;
}

bool timer_start(void)
{
    /* Clear timer, enable interrupts and start timer */
    TCFG4 |= TCFGn_CC | TCFGn_IEN | TCFGn_EN;
    return true;
}

void timer_stop(void)
{
    /* Disable interrupts and stop timer */
    TCFG4 &= ~(TCFGn_IEN | TCFGn_EN);
}

/* Timer interrupt processing - all timers (inc. tick) have a single IRQ */
void ICODE_ATTR TIMER(void)
{
    if (TIREQ & TIREQ_TF0) /* Tick interrupt */
    {
        /* Run through the list of tick tasks */
        call_tick_tasks();

        /* reset Timer 0 IRQ & ref flags */
        TIREQ |= TIREQ_TI0 | TIREQ_TF0;
    }

    if (TIREQ & TIREQ_TF4) /* Timer interrupt */
    {
        if (pfn_timer != NULL)
            pfn_timer();

        /* reset Timer 0 IRQ & ref flags */
        TIREQ |= TIREQ_TI4 | TIREQ_TF4;
    }
}
