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

#include "config.h"
#include "kernel.h"
#include "system.h"
#include "panic.h"

/* Externally defined interrupt handlers */
extern void ICODE_ATTR TIMER(void);
extern void ICODE_ATTR TC32(void);
extern void USB_DEVICE(void);
extern void ADC(void);

void irq(void) __attribute__((interrupt ("IRQ"), section(".icode")));
void irq(void)
{
    int irq = IREQ & ALL_IRQ_MASK;
    /* This function is responsible for clearing interrupt request flags
     * in the interrupt controller. Some modules have their own interrupt
     * request flags, and called functions need to clear those flags first. */
    // FIXME which to use? Could this miss interrupts?
    if (irq & TC_IRQ_MASK) {
        CREQ = TC_IRQ_MASK;
        TIMER();
    }
#ifdef HAVE_USBSTACK
      else if (irq & UB_IRQ_MASK) {
        CREQ = UB_IRQ_MASK;
        USB_DEVICE();
    }
#endif
      else if (irq & ADC_IRQ_MASK) {
        CREQ = ADC_IRQ_MASK;
        ADC();
    }
    else
        panicf("Unhandled IRQ 0x%08X", irq);
}

void fiq_handler(void) __attribute__((interrupt ("FIQ"), naked));

#ifdef BOOTLOADER
void fiq_handler(void)
{
    panicf("Unhandled FIQ 0x%08X", IREQ);
}
#endif

void system_reboot(void)
{
}

void system_exception_wait(void)

{
    while (1) core_sleep();
}

/* TODO - these should live in the target-specific directories and
   once we understand what all the GPIO pins do, move the init to the
   specific driver for that hardware.   For now, we just perform the
   same GPIO init as the original firmware - this makes it easier to
   investigate what the GPIO pins do.
*/

#if defined(RC3000A)
static void INIT_ATTR gpio_init(void)
{
    /* Set GPIO buffer drive strength */
    CFGDRVXL = 0;
    CFGDRVXH = 0;
    CFGDRVAL = 0;
    CFGDRVAH = 0;
    CFGDRVBL = 0;
    CFGDRVBH = 0;
    CFGDRVCL = 0;
    CFGDRVCH = 0;
    CFGDRVDL = 0;
    CFGDRVDH = 0;

    /* Set GPIO pullups */
    CFGPUD |=  0x80000000;

    /* Set GPIO direction */
    GPIOA_DIR = 0xFFFF0FFF;
    GPIOB_DIR = 0x3DFFFFFF;
    GPIOC_DIR = 0xFFFFFFFF;
    GPIOD_DIR = 0x0035F3FF;

    /* Set GPIO special function */
    GSEL_A =  0;
    GTSEL_A = 0;
    GSEL_B =  0x33E000FB;        TC32PCNT = 0; TC32MCNT = 0;

    GTSEL_B = 0x33E000FB;

    /* Initialize GPIO output data */
    GPIOA = 0x00001BFF;
    GPIOB = 0xFFFFFBFF;
    GPIOC = 0xFFFFFFFF;
    GPIOD = 0x000077FF;
}
#endif

static void INIT_ATTR pll_init(void)
{
    /* Set ARM core to FastBus mode */
    asm volatile (
        "mrc     p15, 0, r0, c1, c0  \n\t"
        "bic     r0, r0, #0xc0000000 \n\t"
        "mcr     p15, 0, r0, c1, c0  \n\t"
    );

    /* Clear XTFCLK and XTHCLK: ensure XTIN not selected for FCLK and HCLK */
    PWDCTL &= 0x30000;

    /* Use XIN, not PLL for DIVCLK0 and DIVCLK1 */
    PLLMODE &= ~PLLMODE_DIV1;

    /* Disable PLL clock and peripherals using it */
    CKCTRL |= CKCTRL_PLL;

    /* PLL frequency is:
     * fXin * 8 * (M + 2) / ((P + 2) * 2**S
     * 12 MHz * 8 * (14 + 2) / ((2 + 2) * 2**1) = 192 MHz */
    PLLMODE = PLLMODE_M(14) | PLLMODE_P(2) | PLLMODE_S(1);

    /* Re-enable PLL clock */
    CKCTRL &= ~CKCTRL_PLL;

    /* Wait for PLL lock */
    while ((PLLMODE & PLLMODE_LOCK) == 0);

    /* Zero out SCLKmode dividers for FCLK and HCLK. Not sure why. */
    SCLKmode &= ~0x3FFF;

    /* Use clock dividers for USB, ADC and DAI */
    DIVMODE |= 0x128;

    /* FIXME init CSCFG3 */

    /* FCLK = PLL/2, HCLK = PLL/4 */
    SCLKmode = SCLKmode_H_PHASE(16) | SCLKmode_F_PHASE(32);

    /* Use PLL output for DIVCLK0 and DIVCLK1. This switches CPU to PLL. */
    PLLMODE |= PLLMODE_DIV1;

    asm volatile (
        "nop                         \n\t"
        "nop                         \n\t"
    /* Set ARM core to ASYNC mode */
        "mrc     p15, 0, r0, c1, c0  \n\t"
        "orr     r0, r0, #0xc0000000 \n\t"
        "mcr     p15, 0, r0, c1, c0  \n\t"
    );
}

/* Second function called in the original firmware's startup code - we just
   set up the clocks in the same way as the original firmware for now. */
static void INIT_ATTR clock_init(void)
{
    /* Stop clocks for unused modules: CIF, LCD, I2C, UART */
    HCLKSTOP = 0x60A0;

    /* Set various clocks XTIN, assuming 32KHz, which is surprisingly slow.
     * OF uses orr, but this seems to be the intent. */
    EACLKmode = 0x8000;
    EX1CLKmode = 0x8000;
    UTCLKmode = 0x8000;
    LCLKmode = 0x2000;
    GCLKmode = 0x2000;
    CIFCLKmode = 0x2000;

    /* 2 Mhz, from 12Mhz / (5+1) */
    DIVMODE |= DIVMODE_DVMTC;
    TCLKmode = (0 << 7) | 5;

    pll_init();

 #if 0
    unsigned int i;

    /* STP = 0x1, PW = 0x04 , HLD = 0x0 */
    CSCFG3 = (CSCFG3 &~ 0x3fff) | 0x820;

    /* XIN=External main, Fcpu=Fsys, BCKDIV=1 (Fbus = Fsys / 2) */
    CLKCTRL = (CLKCTRL & ~0xff) | 0x14;

    if (BMI & 0x20)
      PCLKCFG0 = 0xc82d7000; /* EN1 = 1, XIN=Ext. main, DIV1 = 0x2d, P1 = 1 */
    else
      PCLKCFG0 = 0xc8ba7000; /* EN1 = 1, XIN=Ext. main, DIV1 = 0xba, P1 = 1 */

    MCFG |= 0x2000;

#ifdef LOGIK_DAX
    /* Only seen in the Logik DAX original firmware */
    SDCFG = (SDCFG & ~0x7000) | 0x2000;
#endif

    /* Disable PLL */
    PLL0CFG |= 0x80000000;

    /* Enable PLL, M=0xcf, P=0x13. m=M+8, p=P+2, S = 0
       Fout = (215/21)*12MHz = 122857142Hz */
    PLL0CFG = 0x0000cf13;

    i = 8000;
    while (--i) {};

    /* Enable PLL0 */
    CLKDIVC = 0x81000000;

    /* Fsys = PLL0, Fcpu = Fsys, Fbus=Fsys / 2 */
    CLKCTRL = 0x80000010;

    asm volatile (
        "nop      \n\t"
        "nop      \n\t"
    );

    /* Enable Z-Clock */
    PCLKCFG5 |= (1<<31) | (4<<28); /* Timer Z-Clock enable, XIN direct*/

    /* Set TC32 timer to be our USEC_TIMER (Xin divided by 12 = 1MHz) */
    TC32MCNT = 0;
    TC32LDV = 0;
    TC32EN = (1<<24) | 11;
#endif
}

static void INIT_ATTR cpu_init(void)
{
 /* Memory protection - see page 48 of ARM946 TRM
http://infocenter.arm.com/help/topic/com.arm.doc.ddi0201d/DDI0201D_arm946es_r1p1_trm.pdf
 */
    asm volatile (
        /* Region 0 (everything) addr 0, size=4GB, enabled */
        "mov     r0, #0x3f              \n\t"
        "mcr     p15, 0, r0, c6, c0, 0  \n\t"
        "mcr     p15, 0, r0, c6, c0, 1  \n\t"

        /* Region 1 (SDRAM) addr 0x20000000, size=256MB, enabled*/
        "mov     r0, #0x37              \n\t"
        "add     r0, r0, #0x20000000    \n\t"
        "mcr     p15, 0, r0, c6, c1, 0  \n\t"
        "mcr     p15, 0, r0, c6, c1, 1  \n\t"

        /* Region 2 (SRAM) addr 0x30000000, size=256MB, enabled*/
        "add     r0, r0, #0x10000000    \n\t"
        "mcr     p15, 0, r0, c6, c2, 0  \n\t"
        "mcr     p15, 0, r0, c6, c2, 1  \n\t"

        /* Address region 2 - addr 0x40000000, size=512MB, enabled*/
//        "mov     r0, #0x39              \n\t"
//        "add     r0, r0, #0x40000000    \n\t"
        "mov     r0, #0                 \n\t"
        "mcr     p15, 0, r0, c6, c3, 0  \n\t"
        "mcr     p15, 0, r0, c6, c3, 1  \n\t"

        /* Address region 4 - addr 0x60000000, size=256MB, enabled*/
//        "mov     r0, #0x37              \n\t"
//        "add     r0, r0, #0x60000000    \n\t"
        "mov     r0, #0                 \n\t"
        "mcr     p15, 0, r0, c6, c4, 0  \n\t"
        "mcr     p15, 0, r0, c6, c4, 1  \n\t"

        /* Address region 5 - addr 0x10000000, size=256MB, enabled*/
//        "mov     r0, #0x37              \n\t"
//        "add     r0, r0, #0x10000000    \n\t"
        "mov     r0, #0                 \n\t"
        "mcr     p15, 0, r0, c6, c5, 0  \n\t"
        "mcr     p15, 0, r0, c6, c5, 1  \n\t"

        /* Region 6 (perhipherals, etc.) addr 0x80000000, size=2GB, enabled*/
        "mov     r0, #0x3D              \n\t"
        "add     r0, r0, #0x80000000    \n\t"
        "mcr     p15, 0, r0, c6, c6, 0  \n\t"
        "mcr     p15, 0, r0, c6, c6, 1  \n\t"

        /* Address region 7 - addr 0x3000f000, size=4KB, enabled*/
//        "ldr     r0, =0x3000f017        \n\t"
        "mov     r0, #0                 \n\t"
        "mcr     p15, 0, r0, c6, c7, 0  \n\t"
        "mcr     p15, 0, r0, c6, c7, 1  \n\t"

        /* Set access permissions */
        "ldr     r0, =0xffff            \n\t"
        "mcr     p15, 0, r0, c5, c0, 0  \n\t"  /* data access */
        "mcr     p15, 0, r0, c5, c0, 1  \n\t"  /* instruction access */

        /* Set up caches */
        "mov     r0, #0x6               \n\t"
        "mcr     p15, 0, r0, c3, c0, 0  \n\t"  /* write buffer control */
        "mcr     p15, 0, r0, c2, c0, 0  \n\t"  /* data cacheable */
        "mcr     p15, 0, r0, c2, c0, 1  \n\t"  /* instruction cacheable */

        "mov     r0, #0x0               \n\t"
        "mcr     p15, 0, r0, c7, c5, 0  \n\t" /* Flush instruction cache */
        "mcr     p15, 0, r0, c7, c6, 0  \n\t" /* Flush data cache */

        /* Enable caches */
        "ldr     r0, =0xc000107d        \n\t"
        "mcr     p15, 0, r0, c1, c0, 0  \n\t" /* CPU control bits */

        : : : "r0"
    );
}



void INIT_ATTR system_init(void)
{
    /* mask all interrupts */
    IEN &= ~ALL_IRQ_MASK;

    /* Set all interrupts as IRQ for now - some may need to be FIQ in future */
    IRQSEL |= ALL_IRQ_MASK;

    /* Clear all interrupts */
    CREQ |= ALL_IRQ_MASK;

    /* Set master enable bit, but leave all interrupts disabled */
    IEN = (IEN & ~ALL_IRQ_MASK) | MEN_IRQ_MASK;

    cpu_init();
    clock_init();
    gpio_init();

    enable_irq();
}

int system_memory_guard(int newmode)
{
    (void)newmode;
    return 0;
}

#ifdef HAVE_ADJUSTABLE_CPU_FREQ

void set_cpu_frequency(long frequency)
{
}

#endif
