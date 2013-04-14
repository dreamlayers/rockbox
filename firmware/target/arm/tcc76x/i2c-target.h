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
#ifndef I2C_TARGET_H
#define I2C_TARGET_H

/* Definitions for the RC3000A I2C bus */

#define SDA_BIT 0x100
#define SCL_BIT 0x200

#define SCL    (GPIOB & SCL_BIT)
#define SCL_HI GPIOB_DIR &= ~SCL_BIT;
#define SCL_LO GPIOB_DIR |= SCL_BIT
#define SCL_OUT_LO { CFGPUB |= SCL_BIT; GSEL_B &= ~SCL_BIT; GTSEL_B &= ~SCL_BIT; GPIOB_DIR |= SCL_BIT; GPIOB &= ~SCL_BIT; }

#define SDA        (GPIOB & SDA_BIT)
#define SDA_HI     GPIOB_DIR &= ~SDA_BIT
#define SDA_LO     GPIOB_DIR |= SDA_BIT
//#define SDA_INPUT  GPIOB_DIR &= ~SDA_BIT
//#define SDA_OUTPUT GPIOB_DIR |= SDA_BIT
#define SDA_OUT_LO { CFGPUB |= SDA_BIT; GSEL_B &= ~SDA_BIT; GTSEL_B &= ~SDA_BIT; GPIOB_DIR |= SDA_BIT; GPIOB &= ~SDA_BIT; }

//FIXME!!!
#define DELAY { volatile int i; for (i=0;i<100;i++); }

#endif /* I2C_TARGET_H */
