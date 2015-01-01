/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * This file was automatically generated by headergen, DO NOT EDIT it.
 * headergen version: 2.1.8
 * XML versions: stmp3600:2.3.0
 *
 * Copyright (C) 2013 by Amaury Pouly
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
#ifndef __HEADERGEN__STMP3600__LCDIF__H__
#define __HEADERGEN__STMP3600__LCDIF__H__

#define REGS_LCDIF_BASE (0x80060000)

#define REGS_LCDIF_VERSION "2.3.0"

/**
 * Register: HW_LCDIF_CTRL
 * Address: 0
 * SCT: yes
*/
#define HW_LCDIF_CTRL                               (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x0 + 0x0))
#define HW_LCDIF_CTRL_SET                           (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x0 + 0x4))
#define HW_LCDIF_CTRL_CLR                           (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x0 + 0x8))
#define HW_LCDIF_CTRL_TOG                           (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x0 + 0xc))
#define BP_LCDIF_CTRL_SFTRST                        31
#define BM_LCDIF_CTRL_SFTRST                        0x80000000
#define BF_LCDIF_CTRL_SFTRST(v)                     (((v) << 31) & 0x80000000)
#define BP_LCDIF_CTRL_CLKGATE                       30
#define BM_LCDIF_CTRL_CLKGATE                       0x40000000
#define BF_LCDIF_CTRL_CLKGATE(v)                    (((v) << 30) & 0x40000000)
#define BP_LCDIF_CTRL_PRESENT                       29
#define BM_LCDIF_CTRL_PRESENT                       0x20000000
#define BF_LCDIF_CTRL_PRESENT(v)                    (((v) << 29) & 0x20000000)
#define BP_LCDIF_CTRL_BUSY_ENABLE                   25
#define BM_LCDIF_CTRL_BUSY_ENABLE                   0x2000000
#define BV_LCDIF_CTRL_BUSY_ENABLE__BUSY_DISABLED    0x0
#define BV_LCDIF_CTRL_BUSY_ENABLE__BUSY_ENABLED     0x1
#define BF_LCDIF_CTRL_BUSY_ENABLE(v)                (((v) << 25) & 0x2000000)
#define BF_LCDIF_CTRL_BUSY_ENABLE_V(v)              ((BV_LCDIF_CTRL_BUSY_ENABLE__##v << 25) & 0x2000000)
#define BP_LCDIF_CTRL_FIFO_STATUS                   24
#define BM_LCDIF_CTRL_FIFO_STATUS                   0x1000000
#define BV_LCDIF_CTRL_FIFO_STATUS__FIFO_FULL        0x0
#define BV_LCDIF_CTRL_FIFO_STATUS__FIFO_OK          0x1
#define BF_LCDIF_CTRL_FIFO_STATUS(v)                (((v) << 24) & 0x1000000)
#define BF_LCDIF_CTRL_FIFO_STATUS_V(v)              ((BV_LCDIF_CTRL_FIFO_STATUS__##v << 24) & 0x1000000)
#define BP_LCDIF_CTRL_DMA_REQ                       23
#define BM_LCDIF_CTRL_DMA_REQ                       0x800000
#define BF_LCDIF_CTRL_DMA_REQ(v)                    (((v) << 23) & 0x800000)
#define BP_LCDIF_CTRL_DATA_SWIZZLE                  21
#define BM_LCDIF_CTRL_DATA_SWIZZLE                  0x600000
#define BV_LCDIF_CTRL_DATA_SWIZZLE__NO_SWAP         0x0
#define BV_LCDIF_CTRL_DATA_SWIZZLE__LITTLE_ENDIAN   0x0
#define BV_LCDIF_CTRL_DATA_SWIZZLE__BIG_ENDIAN_SWAP 0x1
#define BV_LCDIF_CTRL_DATA_SWIZZLE__SWAP_ALL_BYTES  0x1
#define BV_LCDIF_CTRL_DATA_SWIZZLE__HWD_SWAP        0x2
#define BV_LCDIF_CTRL_DATA_SWIZZLE__HWD_BYTE_SWAP   0x3
#define BF_LCDIF_CTRL_DATA_SWIZZLE(v)               (((v) << 21) & 0x600000)
#define BF_LCDIF_CTRL_DATA_SWIZZLE_V(v)             ((BV_LCDIF_CTRL_DATA_SWIZZLE__##v << 21) & 0x600000)
#define BP_LCDIF_CTRL_RESET                         20
#define BM_LCDIF_CTRL_RESET                         0x100000
#define BV_LCDIF_CTRL_RESET__LCDRESET_LOW           0x0
#define BV_LCDIF_CTRL_RESET__LCDRESET_HIGH          0x1
#define BF_LCDIF_CTRL_RESET(v)                      (((v) << 20) & 0x100000)
#define BF_LCDIF_CTRL_RESET_V(v)                    ((BV_LCDIF_CTRL_RESET__##v << 20) & 0x100000)
#define BP_LCDIF_CTRL_MODE86                        19
#define BM_LCDIF_CTRL_MODE86                        0x80000
#define BV_LCDIF_CTRL_MODE86__8080_MODE             0x0
#define BV_LCDIF_CTRL_MODE86__6800_MODE             0x1
#define BF_LCDIF_CTRL_MODE86(v)                     (((v) << 19) & 0x80000)
#define BF_LCDIF_CTRL_MODE86_V(v)                   ((BV_LCDIF_CTRL_MODE86__##v << 19) & 0x80000)
#define BP_LCDIF_CTRL_DATA_SELECT                   18
#define BM_LCDIF_CTRL_DATA_SELECT                   0x40000
#define BV_LCDIF_CTRL_DATA_SELECT__CMD_MODE         0x0
#define BV_LCDIF_CTRL_DATA_SELECT__DATA_MODE        0x1
#define BF_LCDIF_CTRL_DATA_SELECT(v)                (((v) << 18) & 0x40000)
#define BF_LCDIF_CTRL_DATA_SELECT_V(v)              ((BV_LCDIF_CTRL_DATA_SELECT__##v << 18) & 0x40000)
#define BP_LCDIF_CTRL_WORD_LENGTH                   17
#define BM_LCDIF_CTRL_WORD_LENGTH                   0x20000
#define BV_LCDIF_CTRL_WORD_LENGTH__16_BIT           0x0
#define BV_LCDIF_CTRL_WORD_LENGTH__8_BIT            0x1
#define BF_LCDIF_CTRL_WORD_LENGTH(v)                (((v) << 17) & 0x20000)
#define BF_LCDIF_CTRL_WORD_LENGTH_V(v)              ((BV_LCDIF_CTRL_WORD_LENGTH__##v << 17) & 0x20000)
#define BP_LCDIF_CTRL_RUN                           16
#define BM_LCDIF_CTRL_RUN                           0x10000
#define BF_LCDIF_CTRL_RUN(v)                        (((v) << 16) & 0x10000)
#define BP_LCDIF_CTRL_COUNT                         0
#define BM_LCDIF_CTRL_COUNT                         0xffff
#define BF_LCDIF_CTRL_COUNT(v)                      (((v) << 0) & 0xffff)

/**
 * Register: HW_LCDIF_TIMING
 * Address: 0x10
 * SCT: no
*/
#define HW_LCDIF_TIMING                 (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x10))
#define BP_LCDIF_TIMING_CMD_HOLD        24
#define BM_LCDIF_TIMING_CMD_HOLD        0xff000000
#define BF_LCDIF_TIMING_CMD_HOLD(v)     (((v) << 24) & 0xff000000)
#define BP_LCDIF_TIMING_CMD_SETUP       16
#define BM_LCDIF_TIMING_CMD_SETUP       0xff0000
#define BF_LCDIF_TIMING_CMD_SETUP(v)    (((v) << 16) & 0xff0000)
#define BP_LCDIF_TIMING_DATA_HOLD       8
#define BM_LCDIF_TIMING_DATA_HOLD       0xff00
#define BF_LCDIF_TIMING_DATA_HOLD(v)    (((v) << 8) & 0xff00)
#define BP_LCDIF_TIMING_DATA_SETUP      0
#define BM_LCDIF_TIMING_DATA_SETUP      0xff
#define BF_LCDIF_TIMING_DATA_SETUP(v)   (((v) << 0) & 0xff)

/**
 * Register: HW_LCDIF_DATA
 * Address: 0x20
 * SCT: no
*/
#define HW_LCDIF_DATA               (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x20))
#define BP_LCDIF_DATA_DATA_THREE    24
#define BM_LCDIF_DATA_DATA_THREE    0xff000000
#define BF_LCDIF_DATA_DATA_THREE(v) (((v) << 24) & 0xff000000)
#define BP_LCDIF_DATA_DATA_TWO      16
#define BM_LCDIF_DATA_DATA_TWO      0xff0000
#define BF_LCDIF_DATA_DATA_TWO(v)   (((v) << 16) & 0xff0000)
#define BP_LCDIF_DATA_DATA_ONE      8
#define BM_LCDIF_DATA_DATA_ONE      0xff00
#define BF_LCDIF_DATA_DATA_ONE(v)   (((v) << 8) & 0xff00)
#define BP_LCDIF_DATA_DATA_ZERO     0
#define BM_LCDIF_DATA_DATA_ZERO     0xff
#define BF_LCDIF_DATA_DATA_ZERO(v)  (((v) << 0) & 0xff)

/**
 * Register: HW_LCDIF_DEBUG
 * Address: 0x30
 * SCT: no
*/
#define HW_LCDIF_DEBUG                      (*(volatile unsigned long *)(REGS_LCDIF_BASE + 0x30))
#define BP_LCDIF_DEBUG_BUSY                 27
#define BM_LCDIF_DEBUG_BUSY                 0x8000000
#define BF_LCDIF_DEBUG_BUSY(v)              (((v) << 27) & 0x8000000)
#define BP_LCDIF_DEBUG_LAST_SUBWORD         26
#define BM_LCDIF_DEBUG_LAST_SUBWORD         0x4000000
#define BF_LCDIF_DEBUG_LAST_SUBWORD(v)      (((v) << 26) & 0x4000000)
#define BP_LCDIF_DEBUG_SUBWORD_POSITION     24
#define BM_LCDIF_DEBUG_SUBWORD_POSITION     0x3000000
#define BF_LCDIF_DEBUG_SUBWORD_POSITION(v)  (((v) << 24) & 0x3000000)
#define BP_LCDIF_DEBUG_EMPTY_WORD           23
#define BM_LCDIF_DEBUG_EMPTY_WORD           0x800000
#define BF_LCDIF_DEBUG_EMPTY_WORD(v)        (((v) << 23) & 0x800000)
#define BP_LCDIF_DEBUG_STATE                16
#define BM_LCDIF_DEBUG_STATE                0x7f0000
#define BF_LCDIF_DEBUG_STATE(v)             (((v) << 16) & 0x7f0000)
#define BP_LCDIF_DEBUG_DATA_COUNT           0
#define BM_LCDIF_DEBUG_DATA_COUNT           0xffff
#define BF_LCDIF_DEBUG_DATA_COUNT(v)        (((v) << 0) & 0xffff)

#endif /* __HEADERGEN__STMP3600__LCDIF__H__ */