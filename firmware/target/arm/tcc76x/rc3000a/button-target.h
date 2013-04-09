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
#define BUTTON_POWER        0x00000001 /* Also PLAY/PAUSE */
#define BUTTON_SOURCE       0x00000002
#define BUTTON_MENU         0x00000004
#define BUTTON_RECORD       0x00000008
#define BUTTON_LEFT         0x00000010 /* also REWIND */
#define BUTTON_UP           0x00000020 /* also VOLUME */
#define BUTTON_DOWN         0x00000040
#define BUTTON_RIGHT        0x00000080 /* also F.FWD */

#define BUTTON_MAIN (BUTTON_POWER|BUTTON_SOURCE|BUTTON_MENU\
                     |BUTTON_RECORD|BUTTON_LEFT|BUTTON_UP\
                     |BUTTON_DOWN|BUTTON_RIGHT)

/* Software power-off */
#define POWEROFF_BUTTON BUTTON_POWER
#define POWEROFF_COUNT 40

#endif /* _BUTTON_TARGET_H_ */
