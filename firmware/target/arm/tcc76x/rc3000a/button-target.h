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

#ifndef _BUTTON_TARGET_H_
#define _BUTTON_TARGET_H_

/* Main unit's buttons */
/* Updated to match MPIOHD300Port to use keymap */
#define BUTTON_PLAY         0x00000001 /* Also PAUSE, POWER */
#define BUTTON_ENTER        0x00000002 /* SOURCE */
#define BUTTON_MENU         0x00000004
#define BUTTON_REC          0x00000008
#define BUTTON_REW          0x00000010 /* also LEFT */
#define BUTTON_UP           0x00000020 /* also VOLUME */
#define BUTTON_DOWN         0x00000040
#define BUTTON_FF           0x00000080 /* also RIGHT */

#if 0
#define BUTTON_MAIN (BUTTON_POWER|BUTTON_SOURCE|BUTTON_MENU\
                     |BUTTON_RECORD|BUTTON_LEFT|BUTTON_UP\
                     |BUTTON_DOWN|BUTTON_RIGHT)

/* Software power-off */
#define POWEROFF_BUTTON BUTTON_POWER
#define POWEROFF_COUNT 40
#endif

#define BUTTON_LEFT BUTTON_REW
#define BUTTON_RIGHT BUTTON_FF
#define BUTTON_ON BUTTON_PLAY

#define BUTTON_MAIN (BUTTON_PLAY|BUTTON_REW|BUTTON_FF|\
        BUTTON_UP|BUTTON_DOWN|BUTTON_REC|BUTTON_MENU|\
        BUTTON_ENTER)

#define POWEROFF_BUTTON BUTTON_PLAY
#define POWEROFF_COUNT 30

#endif /* _BUTTON_TARGET_H_ */
