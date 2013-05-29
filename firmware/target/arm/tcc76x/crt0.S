/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2002 by Linus Nielsen Feltzing
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

/* Arm bootloader and startup code based on startup.s from the iPodLinux loader
 *
 * Copyright (c) 2003, Daniel Palffy (dpalffy (at) rainstorm.org)
 * Copyright (c) 2005, Bernard Leach <leachbj@bouncycastle.org>
 *
 */

#include "config.h"
#include "cpu.h"

    .section .init.text,"ax",%progbits

    .extern irq
    .extern fiq
    .extern UIE
    .extern main

    .global    start

/* Telechips firmware files start with a header, as part of the code. */

start:
#ifdef TCCBOOT
    /* Add -DTCCBOOT to EXTRA_DEFINES in the bootloader Makefile to
       enable building the bootloader to be appended to the end of the
       original firmware, dual-booting based on a key-press.

       The following two values are filled in by mktccboot.
     */
of_entrypoint:  .word  0   /* Saved entrypoint of original firmware*/
bl_entrypoint:  .word  0   /* Location in RAM of the start of our bootloader */

#else
    ldr    pc, =start_loc    /* jump to the main entry point  */

    .word  0xffff0106        /* Unknown magic */
    .word  0x3a726556        /* "Ver:" */
    .word  1234              /* Version? */
    .word  0                 /* 32 bit LE checksum, set so file sums to 0 */
#endif

start_loc:

/*
  If we are appended to the OF (i.e. dual-booting), do a button check,
  and branch to the OF's entry point (saved by mktccboot) if not active.
 */

#ifdef TCCBOOT
/* Both RC3000A firmware and TCC760 boot ROM have this seemingly
   pointless code after the initial jump. Maybe it is important?
 */
    mov    r0, #0
    subs   r0, r0, r0

    mov    r0, #0x80000000
#if defined(RC3000A)
    /* Only key accessible via GPIO is the power/play/pause key, which
       cannot be used because it is pressed to turn on the device. All other
       keys use ADC channel 0, and the pin isn't shared with GPIO.
     */

    mov    r1, #0x8000
    str    r1, [r0,#0x410]      /* EACLKmode = 0x8000 : ADC clock = XIN */

    ldr    r1, [r0,#0x400]
    bic    r1, r1, #0x2000      /* CKCTRL : enable ADC clock */
    str    r1, [r0,#0x400]

    ldr    r1, [r0,#0xA88]
    orr    r1, r1, #3           /* ADCCFG : auto powerdown, single mode */
    str    r1, [r0,#0xA88]      /* TODO: More precise init */

    mov    r1, #0               /* ADCCON : operate, channel 0 */
    str    r1, [r0,#0xA00]

1:
    ldr    r1, [r0,#0xA04]      /* Load ADCDATA */
    tst    r1, #1               /* Conversion finished and data stable? */
    beq    1b                   /* No, keep polling */

    ldr    r2, [r0,#0x400]
    orr    r2, r2, #0x2000      /* CKCTRL : disable ADC clock */
    str    r2, [r0,#0x400]

    cmp    r1, #0x200
    ldrhi  pc, of_entrypoint    /* Jump to OF if button not pressed */
#else
    #error No bootup key detection implemented for this target
#endif

/* We are now definitely executing the bootloader, so we relocate to the
   linked address (see boot.lds) - 1MB from the end of DRAM.
 */

// FIXME FIXME FIXME copied from boot ROM

    ldr     r0, =0xc0000078
    mcr     p15, 0, r0,c1,c0

    mov     r12, #0x80000000

    ldr     r0, =0x3036be
    str     r0, [r12,#0x400] // ckctrl
    ldr     r1, =0xffffffff
    str     r1, [r12,#0x43c] // sw_nrst : clear all resets

// FIXME FIXME FIXME copied from boot ROM

/* Configure memory controller. Datasheet recommends simply clearing SDEN,
   setting SDCFG and setting SDEN. Code here is like the RC3000A firmware.
   Currently executing from NOR flash. This must not be run from SDRAM!
 */
    mov    r0, #0xf0000000
    /* SDCFG value, depends on particular player */
#ifdef RC3000A
    ldr    r1, =0xc268a420
#else
    #error No SDCFG value defined for this target
#endif
    str    r1, [r0]             /* Set SDCFG */
    ldr    r1, [r0,#8]          /* MCFG */
    bic    r1, r1, #0x4000      /* Clear XDM: idle bus input mode */
    bic    r1, r1, #0x10        /* Clear SDEN: disable sdram */
    /* Release area 0 from boot ROM and use it for SDRAM.
       RCA3000A OF also enables JTAG here.
     */
    orr    r1, r1, #0x1
    str    r1, [r0,#8]
    eor    r1, r1, #0x10        /* Set SDEN */
    str    r1, [r0,#8]
    eor    r1, r1, #0x10        /* Clear SDEN */
    str    r1, [r0,#8]
    eor    r1, r1, #0x10        /* Set SDEN */
    str    r1, [r0,#8]

 /* Copy code and data for which the final destination is DRAM.
    Other sections are copied later, directly from original
    to final locations.
  */
    ldr    r0, bl_entrypoint

    mov    r1, #0x20000000    /* Destination: 1MB from end of DRAM */
    add    r1, r1, #((MEMORYSIZE - 1) * 0x100000)

    ldr    r2, =_dataend
1:
    cmp    r2, r1
    ldrhi  r3, [r0], #4
    strhi  r3, [r1], #4
    bhi    1b

    sub    r1, r0, r1          /* save copy distance for later copies */

    ldr    pc, =copied_start   /* jump to the relocated start_loc:  */
copied_start:
#endif /* TCCBOOT */

    /* Set up stack for IRQ mode */
    mov    r0,#0xd2
    msr    cpsr, r0
    ldr    sp, =irq_stack

    /* Set up stack for FIQ mode */
    mov    r0,#0xd1
    msr    cpsr, r0
    ldr    sp, =fiq_stack

#ifndef BOOTLOADER
    /* Load the banked FIQ mode registers with useful values here.
       These values will be used in the FIQ handler in pcm-telechips.c */
    .equ   DADO_BASE,  0x80000020

    ldr    r10, =DADO_BASE
    ldr    r11, =dma_play_data
#endif

    /* Let svc, abort and undefined modes use irq stack */
    msr    cpsr, #0xd3
    ldr    sp, =irq_stack
    msr    cpsr, #0xd7
    ldr    sp, =irq_stack
    msr    cpsr, #0xdb
    ldr    sp, =irq_stack

    /* Switch to sys mode */
    mov    r0,#0xdf
    msr    cpsr, r0

    /* Vectors and IRAM copy is done first since they are reclaimed for
     * other uninitialized sections */

    /* Copy exception handler code to address 0 */
    ldr    r2, =_vectorsstart
    ldr    r3, =_vectorsend
    ldr    r4, =_vectorscopy
#ifdef TCCBOOT
    add    r4, r4, r1
#endif
1:
    cmp    r3, r2
    ldrhi  r5, [r4], #4
    strhi  r5, [r2], #4
    bhi    1b

    /* Copy the IRAM */
    ldr    r2, =_iramstart
    ldr    r3, =_iramend
    ldr    r4, =_iramcopy
#ifdef TCCBOOT
    add    r4, r4, r1
#endif
1:
    cmp    r3, r2
    ldrhi  r5, [r4], #4
    strhi  r5, [r2], #4
    bhi    1b

#ifdef HAVE_INIT_ATTR
    /* copy init code to codec buffer */
    ldr    r2, =_initstart
    ldr    r3, =_initend
    ldr    r4, =_initcopy
#ifdef TCCBOOT
    add    r4, r4, r1
#endif
1:
    cmp     r3, r2
    ldrhi   r5, [r4], #4
    strhi   r5, [r2], #4
    bhi     1b
#endif

    /* Zero out IBSS */
    ldr    r2, =_iedata
    ldr    r3, =_iend
    mov    r4, #0
1:
    cmp    r3, r2
    strhi  r4, [r2], #4
    bhi    1b

    /* Initialise bss section to zero */
    ldr    r2, =_edata
    ldr    r3, =_end
    mov    r4, #0
1:
    cmp    r3, r2
    strhi  r4, [r2], #4
    bhi    1b

    /* Set up some stack and munge it with 0xdeadbeef */
    ldr    sp, =stackend
    mov    r3, sp
    ldr    r2, =stackbegin
    ldr    r4, =0xdeadbeef
1:
    cmp    r3, r2
    strhi  r4, [r2], #4
    bhi    1b

    bl     main
    /* main() should never return */

/* Exception handlers. Will be copied to address 0 after memory remapping */
    .section .vectors,"aw"
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]
    ldr    pc, [pc, #24]

    /* Exception vectors */
    .global vectors
vectors:
    .word  start
    .word  undef_instr_handler
    .word  software_int_handler
    .word  prefetch_abort_handler
    .word  data_abort_handler
    .word  reserved_handler
    .word  irq
    .word  fiq_handler

    .text

/* Align stacks to cache line boundary */
    .balign 16

/* 256 words of IRQ stack */
    .space 256*4
irq_stack:

/* 256 words of FIQ stack */
    .space 256*4
fiq_stack:
