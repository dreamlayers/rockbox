/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 Dave Chapman
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
#ifndef __TCC77X_H__
#define __TCC77X_H__

#define CACHEALIGN_BITS (5) // FIXME is this right?

/* Generated from datasheet using this plus some manual attention:
 * sed "s/^\([^ ]*\) 0x\([^ ]*\) .* \(0x[0-9A-Fa-f]*\|Unknown\|-\) \(.*\)$/\/* \4 *\/\n#define \1 (*(volatile unsigned long *)0x80000A\2)/" new\ \ 3.txt  > foo
 */

/* Address Allocations for Internal Peripherals (Base = 0x80000000) */

/* 0x000 ~ 0x0FF DAI & CDIF */

/*
 * 0x100 ~ 0x1FF Interrupt Controller
 */

/* Interrupt Enable Register */
#define IEN (*(volatile unsigned long *)0x80000100)
/* Clear Interrupt Request Register */
#define CREQ (*(volatile unsigned long *)0x80000104)
/* Interrupt Request Flag Register */
#define IREQ (*(volatile unsigned long *)0x80000108)
/* IRQ/FIQ Select Register */
#define IRQSEL (*(volatile unsigned long *)0x8000010C)
/* External interrupt configuration register */
#define ICFG (*(volatile unsigned long *)0x80000110)
/* Masked interrupt request flag register */
#define MREQ (*(volatile unsigned long *)0x80000114)
/* Test Mode Register (must be remained zero) */
#define TSTREQ (*(volatile unsigned long *)0x80000118)
/* IRQ Raw Status (IREQ & IRQSEL) */
#define IRQ (*(volatile unsigned long *)0x80000120)
/* FIQ Raw Status (~IREQ & IRQSEL) */
#define FIQ (*(volatile unsigned long *)0x80000124)
/* Masked IRQ Status (IRQ & IEN) */
#define MIRQ (*(volatile unsigned long *)0x80000128)
/* Masked FIQ Status (FIQ & IEN) */
#define MFIQ (*(volatile unsigned long *)0x8000012C)
/* Trigger Mode (0: edge, 1:level) */
#define TMODE (*(volatile unsigned long *)0x80000130)
/* Synchronizer Control */
#define SYNC (*(volatile unsigned long *)0x80000134)
/* Wakeup Control */
#define WKUP (*(volatile unsigned long *)0x80000138)

/*
 * 0x200 ~ 0x2FF Timer Counter
 */

/* Timer/Counter 0 Configuration Register */
#define TCFG0 (*(volatile unsigned short *)0x80000200)
/* Timer/Counter 0 Counter Register */
#define TCNT0 (*(volatile unsigned long *)0x80000204)
/* Timer/Counter 0 Reference Register */
#define TREF0 (*(volatile unsigned long *)0x80000208)
/* Timer/Counter 0 Middle Reference Register */
#define TMREF0 (*(volatile unsigned long *)0x8000020C)
/* Timer/Counter 1 Configuration Register */
#define TCFG1 (*(volatile unsigned short *)0x80000210)
/* Timer/Counter 1 Counter Register */
#define TCNT1 (*(volatile unsigned long *)0x80000214)
/* Timer/Counter 1 Reference Register */
#define TREF1 (*(volatile unsigned long *)0x80000218)
/* Timer/Counter 1 Middle Reference Register */
#define TMREF1 (*(volatile unsigned long *)0x8000021C)
/* Timer/Counter 2 Configuration Register */
#define TCFG2 (*(volatile unsigned short *)0x80000220)
/* Timer/Counter 2 Counter Register */
#define TCNT2 (*(volatile unsigned long *)0x80000224)
/* Timer/Counter 2 Reference Register */
#define TREF2 (*(volatile unsigned long *)0x80000228)
/* Timer/Counter 2 Middle Reference Register */
#define TMREF2 (*(volatile unsigned long *)0x8000022C)
/* Timer/Counter 3 Configuration Register */
#define TCFG3 (*(volatile unsigned short *)0x80000230)
/* Timer/Counter 3 Counter Register */
#define TCNT3 (*(volatile unsigned long *)0x80000234)
/* Timer/Counter 3 Reference Register */
#define TREF3 (*(volatile unsigned long *)0x80000238)
/* Timer/Counter 3 Middle Reference Register */
#define TMREF3 (*(volatile unsigned long *)0x8000023C)
/* Timer/Counter 4 Configuration Register */
#define TCFG4 (*(volatile unsigned short *)0x80000240)
/* Timer/Counter 4 Counter Register */
#define TCNT4 (*(volatile unsigned long *)0x80000244)
/* Timer/Counter 4 Reference Register */
#define TREF4 (*(volatile unsigned long *)0x80000248)
/* Timer/Counter 5 Configuration Register */
#define TCFG5 (*(volatile unsigned short *)0x80000250)
/* Timer/Counter 5 Counter Register */
#define TCNT5 (*(volatile unsigned long *)0x80000254)
/* Timer/Counter 5 Reference Register */
#define TREF5 (*(volatile unsigned long *)0x80000258)
/* Timer/Counter n Interrupt Request Register */
#define TIREQ (*(volatile unsigned short *)0x80000260)
/* Watchdog Timer Configuration Register */
#define TWDCFG (*(volatile unsigned short *)0x80000270)
/* Watchdog Timer Clear Register */
#define TWDCLR (*(volatile unsigned short *)0x80000274)
/* 32-bit Counter Enable / Pre-scale Value */
#define TC32EN (*(volatile unsigned long *)0x80000280)
/* 32-bit Counter Load Value */
#define TC32LDV (*(volatile unsigned long *)0x80000284)
/* 32-bit Counter Match Value 0 */
#define TC32CMP0 (*(volatile unsigned long *)0x80000288)
/* 32-bit Counter Match Value 1 */
#define TC32CMP1 (*(volatile unsigned long *)0x8000028C)
/* 32-bit Counter Current Value (pre-scale counter) */
#define TC32PCNT (*(volatile unsigned long *)0x80000290)
/* 32-bit Counter Current Value (main counter) */
#define TC32MCNT (*(volatile unsigned long *)0x80000294)

/*
 * 0x300 ~ 0x3FF GPIO
 */

/* GPIO_A Data Register */
#define GDATA_A (*(volatile unsigned long *)0x80000300)
/* GPIO_A Direction Control Register */
#define GIOCON_A (*(volatile unsigned long *)0x80000304)
/* GPIO_A Function Select Register 1 */
#define GSEL_A (*(volatile unsigned long *)0x80000308)
/* GPIO_A Function Select Register 2 */
#define GTSEL_A (*(volatile unsigned long *)0x8000030C)
/* GPIO_B Data Register */
#define GDATA_B (*(volatile unsigned long *)0x80000310)
/* GPIO_B Direction Control Register */
#define GIOCON_B (*(volatile unsigned long *)0x80000314)
/* GPIO_B Function Select Register 1 */
#define GSEL_B (*(volatile unsigned long *)0x80000318)
/* GPIO_B Function Select Register 2 */
#define GTSEL_B (*(volatile unsigned long *)0x8000031C)
/* GPIO_C Data Register */
#define GDATA_C (*(volatile unsigned long *)0x80000320)
/* GPIO_C Direction Control Register */
#define GIOCON_C (*(volatile unsigned long *)0x80000324)
/* GPIO_D Data Register */
#define GDATA_D (*(volatile unsigned long *)0x80000330)
/* GPIO_D Direction Control Register */
#define GIOCON_D (*(volatile unsigned long *)0x80000334)

/*
 * 0x400 ~ 0x4FF Clock Generator & Power Management
 */

/* Clock Control Register */
#define CKCTRL (*(volatile unsigned long *)0x80000400)
/* PLL Control Register */
#define PLLMODE (*(volatile unsigned long *)0x80000404)
/* System Clock Control Register */
#define SCLKmode (*(volatile unsigned long *)0x80000408)
/* DCLK (DAI/CODEC) Control Register */
#define DCLKmode (*(volatile unsigned long *)0x8000040C)
/* ADCLK and EX2CLK Control Register */
#define EACLKmode (*(volatile unsigned long *)0x80000410)
/* EX1CLK Control Register */
#define EX1CLKmode (*(volatile unsigned long *)0x80000414)
/* UTCLK (UART) Control Register */
#define UTCLKmode (*(volatile unsigned long *)0x80000418)
/* UBCLK (USB) Control Register */
#define UBCLKmode (*(volatile unsigned long *)0x8000041C)
/* LCLK (LCD) Control Register */
#define LCLKmode (*(volatile unsigned long *)0x80000420)
/* TCLK (Timer) Control Register */
#define TCLKmode (*(volatile unsigned long *)0x80000424)
/* GCLK (GSIO) Control Register */
#define GCLKmode (*(volatile unsigned long *)0x80000428)
/* CIFCLK Control Register */
#define CIFCLKmode (*(volatile unsigned long *)0x8000042C)
/* Software Reset for each peripherals */
#define SW_nRST (*(volatile unsigned long *)0x8000043C)
/* Power Down Control */
#define PWDCTL (*(volatile unsigned long *)0x80000440)
/* Divider Mode Enable (DCO Disable) */
#define DIVMODE (*(volatile unsigned long *)0x80000444)
/* HCLK Stop Control */
#define HCLKSTOP (*(volatile unsigned long *)0x80000448)

/*
 * 0x500 ~ 0x5FF USB1.1 Device
 */

/* Non Indexed Registers */

/* Function Address Register */
#define UBFADR (*(volatile unsigned short *)0x80000500)
/* Power Management Register */
#define UBPWR (*(volatile unsigned short *)0x80000504)
/* Endpoint Interrupt Flag Register */
#define UBEIR (*(volatile unsigned short *)0x80000508)
/* USB Interrupt Flag Register */
#define UBIR (*(volatile unsigned short *)0x80000518)
/* Endpoint Interrupt Enable Register */
#define UBEIEN (*(volatile unsigned short *)0x8000051C)
/* Interrupt Enable Register */
#define UBIEN (*(volatile unsigned short *)0x8000052C)
/* Frame Number 1 Register */
#define UBFRM1 (*(volatile unsigned short *)0x80000530)
/* Frame Number 2 Register */
#define UBFRM2 (*(volatile unsigned short *)0x80000534)
/* Index Register */
#define UBIDX (*(volatile unsigned short *)0x80000538)

/* Common Indexed Register */

/* IN Max Packet Register */
#define MAXP (*(volatile unsigned short *)0x80000540)

/* In Indexed Registers */

/* IN CSR1 Register (EP0 CSR Register) */
#define INCSR1 (*(volatile unsigned short *)0x80000544)
/* IN CSR2 Register */
#define INCSR2 (*(volatile unsigned short *)0x80000548)

/* Out Indexed Registers */

/* OUT CSR1 Register */
#define OCSR1 (*(volatile unsigned short *)0x80000550)
/* OUT CSR2 Register */
#define OCSR2 (*(volatile unsigned short *)0x80000554)
/* OUT FIFO Write Count 1 Register */
#define OFIFO1 (*(volatile unsigned short *)0x80000558)
/* OUT FIFO Write Count 2 Register */
#define OFIFO2 (*(volatile unsigned short *)0x8000055C)

/* FIFO Registers */

/* EP0 FIFO Register */
#define EP0FIFO (*(volatile unsigned short *)0x80000580)
/* EP1 FIFO Register */
#define EP1FIFO (*(volatile unsigned short *)0x80000584)
/* EP2 FIFO Register */
#define EP2FIFO (*(volatile unsigned short *)0x80000588)

/* DMA Registers */

/* DMA Control Register */
#define DMACON (*(volatile unsigned short *)0x800005C0)
/* EP1 FIFO Access Register for DMA */
#define DMAEP1 (*(volatile unsigned short *)0x800005C4)
/* EP2 FIFO Access Register for DMA */
#define DMAEP2 (*(volatile unsigned short *)0x800005C8)

/* 0x600 ~ 0x6FF UART/IrDA */

/* 0x700 ~ 0x7FF GSIO (General Purpose Serial Input/Output) */

/* 0x800 ~ 0x8FF I2C */

/* 0x900 ~ 0x9FF ECC */

/*
 * 0xA00 ~ 0xAFF ADC Control & Etc.
 */

/* ADC Control Register */
#define ADCCON (*(volatile unsigned long *)0x80000A00)
/* ADC Data Register */
#define ADCDATA (*(volatile unsigned long *)0x80000A04)
/* ADC Control Register A */
#define ADCCONA (*(volatile unsigned long *)0x80000A80)
/* ADC Status Register */
#define ADCSTATUS (*(volatile unsigned long *)0x80000A84)
/* ADC Configuration Register */
#define ADCCFG (*(volatile unsigned long *)0x80000A88)

/* USB Port Control Register */
#define USBCTR (*(volatile unsigned long *)0x80000A14)
/* Test Mode Control Register */
#define TSTSEL (*(volatile unsigned long *)0x80000A18)
/* Miscellaneous Configuration Register */
#define MISCCFG (*(volatile unsigned long *)0x80000A1C)
/* Pull-up Enable for GPIO_A */
#define CFGPUA (*(volatile unsigned long *)0x80000A20)
/* Pull-up Enable for GPIO_B */
#define CFGPUB (*(volatile unsigned long *)0x80000A24)
/* Pull-up Enable for GPIO_C */
#define CFGPUC (*(volatile unsigned long *)0x80000A28)
/* Pull-up Enable for GPIO_D */
#define CFGPUD (*(volatile unsigned long *)0x80000A2C)
/* Buffer Drive Strength Select AL */
#define CFGDRVAL (*(volatile unsigned long *)0x80000A30)
/* Buffer Drive Strength Select AH */
#define CFGDRVAH (*(volatile unsigned long *)0x80000A34)
/* Buffer Drive Strength Select BL */
#define CFGDRVBL (*(volatile unsigned long *)0x80000A38)
/* Buffer Drive Strength Select BH */
#define CFGDRVBH (*(volatile unsigned long *)0x80000A3C)
/* Buffer Drive Strength Select CL */
#define CFGDRVCL (*(volatile unsigned long *)0x80000A40)
/* Buffer Drive Strength Select CH */
#define CFGDRVCH (*(volatile unsigned long *)0x80000A44)
/* Buffer Drive Strength Select DL */
#define CFGDRVDL (*(volatile unsigned long *)0x80000A48)
/* Buffer Drive Strength Select DH */
#define CFGDRVDH (*(volatile unsigned long *)0x80000A4C)
/* Buffer Drive Strength Select XL */
#define CFGDRVXL (*(volatile unsigned long *)0x80000A50)
/* Buffer Drive Strength Select XH */
#define CFGDRVXH (*(volatile unsigned long *)0x80000A54)
/* System Configuration */
#define CFGSYS (*(volatile unsigned long *)0x80000A60)

/* 0xB00 ~ 0xBFF Camera Interface */

/* 0xC00 ~ 0xCFF Reserved */

/* 0xD00 ~ 0xDFF USB1.1 Host */

/* 0xE00 ~ 0xEFF DMA Controller */

/* 0xF00 ~ 0xFFF LCD controller (TCC761 only) */

/*
 * Memory controller registers at 0xF0000000
 */

/* SDRAM Configuration Register */
#define SDCFG (*(volatile unsigned long *)0xF0000000)
/* SDRAM FSM Status Register */
#define SDFSM (*(volatile unsigned long *)0xF0000004)
/* Miscellaneous Configuration Register */
#define MCFG (*(volatile unsigned long *)0xF0000008)
/* Test mode register (must be remained zero) */
#define TST (*(volatile unsigned long *)0xF000000C)
/* External Chip Select 0 Configuration Register */
#define CSCFG0 (*(volatile unsigned long *)0xF0000010)
/* External Chip Select 1 Configuration Register */
#define CSCFG1 (*(volatile unsigned long *)0xF0000014)
/* External Chip Select 2 Configuration Register */
#define CSCFG2 (*(volatile unsigned long *)0xF0000018)
/* External Chip Select 3 Configuration Register */
#define CSCFG3 (*(volatile unsigned long *)0xF000001C)
/* Memory Controller Clock Count Register */
#define CLKCFG (*(volatile unsigned long *)0xF0000020)
/* SDRAM Command Register */
#define SDCMD (*(volatile unsigned long *)0xF0000024)

#endif
