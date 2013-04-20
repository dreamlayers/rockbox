/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 by Rob Purchase
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
#include "tick.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "button.h"
#include "lcd.h"
#include "font.h"
#include "adc.h"

bool dbg_ports(void)
{
    return false;
}

bool dbg_hw_info(void)
{
    int line = 0, i, button, oldline;
    bool done=false;

    lcd_setfont(FONT_SYSFIXED);
    lcd_clear_display();

    /* Put all the static text before the while loop */
    lcd_puts(0, line++, "[Hardware info]");

    oldline=line;
    while(!done)
    {
        line = oldline;
        button = button_get(false);

        button &= ~BUTTON_REPEAT;
#ifdef BUTTON_SELECT
        if (button == BUTTON_SELECT)
#else
        if (button == BUTTON_PLAY)  // BUTTON_STOP fixme
#endif
            done=true;

        lcd_putsf(0, line++, "tick: %d", current_tick);
        lcd_putsf(0, line++, "seconds: %d", current_tick/HZ);

        lcd_putsf(0, line++, "A:%08lx B:%08lx", GPIOA, GPIOB);
        lcd_putsf(0, line++, "C:%08lx D:%08lx", GPIOC, GPIOD);
        lcd_putsf(0, line++, "ADC0: %03x ADC1: %03x", adc_read(0), adc_read(1));
        lcd_putsf(0, line++, "ADC2: %03x ADC3: %03x", adc_read(2), adc_read(3));

        lcd_update();
    }
    return false;
}
