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
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "button.h"
#include "lcd.h"
#include "font.h"
#include "adc.h"

bool dbg_ports(void)
{
    int line = 0, button, oldline;
    bool done=false;

    lcd_setfont(FONT_SYSFIXED);
    lcd_clear_display();

    /* Put all the static text before the while loop */
    lcd_puts(0, line++, "[Ports]");

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

/* Tool function to read the flash manufacturer and type, if available.
   Only chips which could be reprogrammed in system will return values.
   (The mode switch addresses vary between flash manufacturers, hence addr1/2) */
   /* In IRAM to avoid problems when running directly from Flash */
static bool dbg_flash_id(unsigned* p_manufacturer, unsigned* p_device,
                         unsigned addr1, unsigned addr2)
                         ICODE_ATTR __attribute__((noinline));
static bool dbg_flash_id(unsigned* p_manufacturer, unsigned* p_device,
                         unsigned addr1, unsigned addr2)
{
    unsigned not_manu, not_id; /* read values before switching to ID mode */
    unsigned manu, id; /* read values when in ID mode */

    volatile unsigned short* flash = (unsigned short*)0x70000000; /* flash mapping */
    int old_level; /* saved interrupt level */

    not_manu = flash[0]; /* read the normal content */
    not_id   = flash[1]; /* should be 'A' (0x41) and 'R' (0x52) from the "ARCH" marker */

    /* disable interrupts, prevent any stray flash access */
    old_level = disable_irq_save();

    flash[addr1] = 0xAA; /* enter command mode */
    flash[addr2] = 0x55;
    flash[addr1] = 0x90; /* ID command */
    /* Atmel wants 20ms pause here */
    /* sleep(HZ/50); no sleeping possible while interrupts are disabled */

    manu = flash[0]; /* read the IDs */
    id   = flash[1];

    flash[0] = 0xF0; /* reset flash (back to normal read mode) */
    /* Atmel wants 20ms pause here */
    /* sleep(HZ/50); no sleeping possible while interrupts are disabled */

    restore_irq(old_level); /* enable interrupts again */
    /* I assume success if the obtained values are different from
        the normal flash content. This is not perfectly bulletproof, they
        could theoretically be the same by chance, causing us to fail. */
    if (not_manu != manu || not_id != id) /* a value has changed */
    {
        *p_manufacturer = manu; /* return the results */
        *p_device = id;
        return true; /* success */
    }
    return false; /* fail */
}

bool dbg_hw_info(void)
{
    int line = 0, button;
    unsigned manu, id;
    bool got_id;

    lcd_setfont(FONT_SYSFIXED);
    lcd_clear_display();

    /* Put all the static text before the while loop */
    lcd_puts(0, line++, "[Hardware info]");

    got_id = dbg_flash_id(&manu, &id, 0x5555, 0x2AAA); /* try SST, Atmel, NexFlash */
    if (!got_id)
        got_id = dbg_flash_id(&manu, &id, 0x555, 0x2AA); /* try AMD, Macronix */

    if (got_id)
        lcd_putsf(0, line++, "Flash: M=%02x D=%04x", manu, id);
    else
        lcd_puts(0, line++, "Flash: M=?? D=????"); /* unknown, sorry */

    lcd_update();

    while(1)
    {
        button = button_get(false);

        button &= ~BUTTON_REPEAT;
#ifdef BUTTON_SELECT
        if (button == BUTTON_SELECT)
#else
        if (button == BUTTON_PLAY)  // BUTTON_STOP fixme
#endif
            break;
    }

    return false;
}
