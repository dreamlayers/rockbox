/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2008 by Vitja Makarov
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
#include "usb.h"

#include "tcc76x.h"

#include "cpu.h"
#include "system.h"
#include "kernel.h"
#include "panic.h"

#ifdef HAVE_USBSTACK
#include "usb_ch9.h"
#include "usb_core.h"

#define USB_PACKET_SIZE 64

static bool setup_phase = true;
int fucked = 2;

static int dbg_level = 0x00;
#define DEBUG(level, fmt, args...) do { if (dbg_level & (level)) printf(fmt, ## args); } while (0)

#include <inttypes.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "power.h"

#ifndef BOOTLOADER
#define printf(...) do {} while (0)
#define panicf_my panicf
#else
int printf(const char *fmt, ...);
#define panicf_my(fmt, args...) {               \
    int flags = disable_irq_save();             \
    printf("*** PANIC ***");                    \
    printf(fmt, ## args);                       \
    printf("*** PANIC ***");                    \
    while (usb_detect() == USB_INSERTED)        \
        ;                                       \
    power_off();                                \
    while(1);                                   \
    restore_irq(flags);                         \
}
#endif

struct tcc_ep {
    bool allocated; /* Endpoint has been allocated */
    unsigned char id; /* Combination of direction and number */
    int count;     /* Size of entire transfer, for later completion callback. */

    /* Data to send/receive. Updated during transfer. */
    char *buf;     /* Pointer to data. NULL is ok for 0 length transfer. */
    int bytesleft; /* Bytes left to transfer, or -1 if no transfer. */

    struct semaphore completion_sem; /* For waiting in usb_drv_send */
    bool wait; /* If true, completion signals semaphore */

    /* Hardware registers associated with endpoint */
    volatile unsigned short *fifo; /* FIFO register for PIO */
};

// FIXME remove
void die(char *s) {
        printf(s);
        disable_irq_save();
        while (1);
}

static struct tcc_ep tcc_endpoints[USB_NUM_ENDPOINTS] = {
    { /* control */
        .allocated = true,
        .id = EP_CONTROL,
        .bytesleft = -1,
        .fifo = &EP0FIFO,
    }, {  /* bulk / iso */
        .allocated = false,
        .bytesleft = -1,
        .fifo = &EP1FIFO,
    }, { /* bulk / iso */
        .allocated = false,
        .bytesleft = -1,
        .fifo = &EP2FIFO,
    },
} ;

static void usb_pio_write(struct tcc_ep *ep) ICODE_ATTR;
static int usb_pio_read(struct tcc_ep *ep);

int usb_drv_request_endpoint(int type, int dir)
{
    int flags;
    size_t ep;
    int ret = -1;
    int ep_dir = dir & USB_ENDPOINT_DIR_MASK;

    /* TODO handle isochronous also */
    if (type != USB_ENDPOINT_XFER_BULK)
        return -1;

    flags = disable_irq_save();

    /* Find free endpoint. Control endpoint is not allocated this way. */
    for (ep = 1; ep < USB_NUM_ENDPOINTS; ep++) {
        if (!tcc_endpoints[ep].allocated) {
            tcc_endpoints[ep].allocated = true;
            tcc_endpoints[ep].id = ep | ep_dir;
            ret = ep | ep_dir;

            /* Initialize endpoint registers */
            UBIDX = ep;
            MAXP = MAXP_64_BYTES;
            if (ep_dir == USB_DIR_IN) {
                INCSR1n = (INCSR1n & ~0xFF) | (INCSR1n_CTGL|INCSR1n_FLFF);
                /* ASET makes writes a tiny bit faster */
                INCSR2n = (INCSR2n & ~(INCSR2n_ISO|INCSR2n_DMA)) |
                          (INCSR2n_ASET|INCSR2n_MDIN);
            } else {
                INCSR2n &= ~(INCSR2n_ASET|INCSR2n_ISO|INCSR2n_DMA|INCSR2n_MDIN);
                OCSR1n = (OCSR1n & ~0xFF) | (OCSR1n_CTGL|OCSR1n_FLFF);
                OCSR2n &= ~(OCSR2n_ACLR|OCSR2n_ISO);
            }
            break;
        }
    }

    restore_irq(flags);

    return ret;
}

void usb_drv_release_endpoint(int ep)
{
    int flags;
    ep = ep & 0x7f;

    if (ep < 1 || ep >= USB_NUM_ENDPOINTS)
        return ;

    flags = disable_irq_save();

    tcc_endpoints[ep].allocated = false;

    restore_irq(flags);
}

#if 0
static
char *dump_data(char *data, int count)
{
    static char buf[1024];
    char *dump = buf;
    int i;

    for (i = 0; i < count; i++)
        dump += snprintf(dump, sizeof(buf) - (dump - buf), "%02x", data[i]);
    return buf;
}
#endif

/* Reset TCC76x usb device */
static void usb_reset(void)
{
#if 0
    pullup_on();

    TCC7xx_USB_DELAY_CTRL |= 0x81;

    TCC7xx_USB_SYS_CTRL = 0xa000 |
        TCC7xx_USB_SYS_CTRL_RESET |
        TCC7xx_USB_SYS_CTRL_RFRE  |
        TCC7xx_USB_SYS_CTRL_SPDEN |
        TCC7xx_USB_SYS_CTRL_VBONE |
        TCC7xx_USB_SYS_CTRL_VBOFE;

    usb_set_speed(1);
    pullup_on();

    TCC7xx_USB_EPIF = TCC7xx_USB_EPIF_IRQ_MASK;
    global_ep_irq_mask = 0x1;
    TCC7xx_USB_EPIE = global_ep_irq_mask;
#endif
#if 1
    int ep;
    for (ep = 1; ep < USB_NUM_ENDPOINTS; ep++) {
        tcc_endpoints[ep].allocated = false;
        tcc_endpoints[ep].bytesleft = -1;
    }
#endif

    /* Initialize control endpoint */
    UBIDX = 0;
    MAXP = MAXP_64_BYTES;

    usb_core_bus_reset();
}

static inline void usb_ep0_interrupt(void) {
    unsigned long ep0csr_val;

    UBIDX = 0;
    ep0csr_val = EP0CSR;
    //printf("E0C: %x", ep0csr_val);

    if (ep0csr_val & EP0CSR_STST) {
        /* Stall, meaning protocol violation */
        //FIXME
    }

    if (ep0csr_val & EP0CSR_CEND) {
        /* Host signalled end of control transfer. */
        /* Next token would be another setup. */
        setup_phase = true;
        /* Clear CEND */
        EP0CSR = ep0csr_val & EP0CSR_CLSE;
    }

    if (ep0csr_val & EP0CSR_ORDY) {
        /* New token received. */
        if (setup_phase) {
            /* This should be setup token */
            static unsigned char ctrldata[8];
            int i;
            unsigned char *p = (unsigned char *)ctrldata;
            int l = OFIFO1n;
            //printf ("CPR: %d", l);
            if (l != 8) die("LEN");
            for (i = 0; i < 8; i++) {
                p[i] = EP0FIFO;
            }
#if 1
            if (ctrldata[1] == 5) {
                // FIXME all this delay is needed for set address
                printf("0: %02x %02x %02x %02x", p[0], p[1], p[2], p[3]);
                printf("4: %02x %02x %02x %02x", p[4], p[5], p[6], p[7]);
                //EP0CSR = EP0CSR_CLOR;
                lcd_update();
            }
#endif
            /* For setup tokens, usb_core_control_request is called */
            usb_core_control_request((struct usb_ctrlrequest*)ctrldata);
            setup_phase = false;
        }
        //} else {
            /* This should be data phase data from host */
        //}
    }

    if (tcc_endpoints[0].bytesleft >= 0 &&
        (ep0csr_val & EP0CSR_IRDY) == 0) {
        usb_pio_write(&tcc_endpoints[0]);
    }
}

/* Handler for data endpoint interrupt, called from USB_DEVICE() */
static inline void usb_ep12_interrupt(struct tcc_ep *ep) {
    unsigned long epcsr_val;

    if ((ep->id & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
        epcsr_val = INCSR1n;
        if (ep->bytesleft >= 0 && (epcsr_val & INCSR1n_IRDY) == 0) {
            usb_pio_write(ep);
        }
    } else {
        epcsr_val = OCSR1n;
        if ((epcsr_val & OCSR1n_ORDY) && ep->bytesleft != 0) {
            usb_pio_read(ep);
        }
    }
}

/* IRQ handler */
void USB_DEVICE(void)
{
    unsigned long ubir_val = UBIR & (UBIR_RST | UBIR_RSM | UBIR_SP);
    unsigned long ubeir_val = UBEIR & (UBEIR_EP2 | UBEIR_EP1 | UBEIR_EP0);

    /* Clear interrupts that will be handled here */
    UBIR = ubir_val;
    UBEIR = ubeir_val;

    if (ubeir_val & UBEIR_EP1) {
        UBIDX = 1;
        usb_ep12_interrupt(&tcc_endpoints[1]);
    }

    if (ubeir_val & UBEIR_EP2) {
        UBIDX = 2;
        usb_ep12_interrupt(&tcc_endpoints[2]);
    }
    //printf("USBD: %d %d", ubir_val, ubeir_val);

    /* FIXME this wait is necessary and horrible for performance */
    if (fucked) {//(ubeir_val & (UBEIR_EP1|UBEIR_EP2)) == 0)
        printf("FUCKED: %d %d", ubir_val, ubeir_val);
        lcd_update();
    }

    if (ubir_val & UBIR_RST) {
        usb_reset();
    }

    if (ubir_val & UBIR_RSM) {
        /* Re-enable suspend interrupt */
        UBIEN |= UBIEN_SP;
    }

    if (ubir_val & UBIR_SP) {
        /* Enter suspend mode. Interrupt would be generated continuously,
         * so it must be disabled. */
        UBIEN &= ~UBIEN_SP;
        UBPWR |= UBPWR_ENSP;
    }

    if (ubeir_val & UBEIR_EP0) {
        usb_ep0_interrupt();
    }
}

void usb_drv_set_address(int address)
{
    UBFADR = UBFADR_UP | address;
    // DEBUG(2, "setting address %d %d", address, TCC7xx_USB_FUNC);
    if (fucked == 2) fucked = 0;
}

int usb_drv_port_speed(void)
{
    /* TCC76x is USB 1.0 full speed */
    return 0;
}

/* Write data to host using PIO.
 * This is a separate function because it is called both from
 * usb_drv_send and from the interrupt handler.
 *
 * Requirements: UBIDX already selects endpoint
 *               IRDY is initially clear
 *               Exclusive access to USB hardware (eg. IRQ disabled)
 *               ASET enabled for data endpoints
 *
 * Note: This should be able to send zero length packets with
 *       buf == NULL and bytesleft == 0
 */
static void ICODE_ATTR usb_pio_write(struct tcc_ep *ep)
{
    register volatile unsigned short *fifo = ep->fifo;
    register unsigned char *p = ep->buf;

    /* The FIFO is 128 bytes, and multiple packets can fit. */
    while (1) {
        /* Write packet to the FIFO */
        unsigned int l = MIN(ep->bytesleft, USB_PACKET_SIZE);
        unsigned char *pend = p + l;
        do {
            *fifo = *(p++);
        } while (p < pend);
        ep->bytesleft -= l;

        if (ep->id == EP_CONTROL) {
            if (ep->bytesleft == 0) {
                /* No more left to send. Assume that end of
                 * transmission means end of EP0 data phase */
                EP0CSR |= EP0CSR_IRDY | EP0CSR_DEND;
                setup_phase = true;
                break;
            } else {
                /* One packet sent, more left to send */
                EP0CSR |= EP0CSR_IRDY;
                if (EP0CSR & EP0CSR_IRDY) {
                    /* FIFO full for now, more to send later */
                    ep->buf = p;
                    return;
                }
            }
        } else {
            /* Data endpoint */
            if (ep->bytesleft == 0) {
                /* Only set IRDY if less than a full packet was sent.
                 * Otherwise, it was set because of INCSR1n_ASET. */
                if (l < USB_PACKET_SIZE) INCSR1n |= INCSR1n_IRDY;
                break;
            } else {
                if (INCSR1n & INCSR1n_IRDY) {
                    /* FIFO full for now, more to send later */
                    ep->buf = p;
                    return;
                }
            }
        }
    }

    /* If more is left to send, a return left the loop. This is
     * only executed when the transfer is finished */
    if (ep->wait) semaphore_release(&ep->completion_sem);
    usb_core_transfer_complete(ep->id & USB_ENDPOINT_NUMBER_MASK,
                               USB_DIR_IN, 0, ep->count);
    ep->bytesleft = -1;

    return;
}

/* This function is used for both usb_drv_send() and
 * usb_drv_send_nonblocking() */
static int usb_drv_send_internal(int endpoint, void *ptr, int length, bool wait)
{
    int flags;
    //int rc = 0;
    unsigned short epidx = endpoint & USB_ENDPOINT_NUMBER_MASK;
    struct tcc_ep *ep;

    //printf("send: %d, %d, %d", endpoint, length, wait);

    ep = &tcc_endpoints[epidx];

    if (endpoint == EP_CONTROL) {
        /* Assume 0 length send means no data phase. This does not
         * send a 0 byte packet. It just ends the data phase. */
        if (length == 0) {
            flags = disable_irq_save();
            UBIDX = 0;
            setup_phase = true;
            EP0CSR |= EP0CSR_CLOR | EP0CSR_DEND;
            restore_irq(flags);
            //printf("CLOR, DEND");
            return 0;
        }
    } else if ((ep->id & USB_ENDPOINT_DIR_MASK) != USB_DIR_IN) {
        panicf_my("%s(%d,%d): Not supported", __func__, endpoint, length);
    }

    DEBUG(2, "%s(%d,%d):", __func__, endpoint, length);

    flags = disable_irq_save();

    if(ep->bytesleft >= 0) {
        panicf_my("%s: ep is already busy", __func__);
    }

    /* Save parameters for use by writing function */
    ep->buf = (unsigned char *)ptr;
    ep->bytesleft = length;
    ep->count = length;
    ep->wait = wait;

    UBIDX = epidx;

    /* If IRDY is low, the IRDY interrupt was probably already handled.
     * A packet must be written now to start the transfer. After it is
     * sent, hardware clears IRDY, generating an interrupt. Such
     * interrupts will then send the rest of the data, if any remains. */
    if ((endpoint == EP_CONTROL) ?
        ((EP0CSR & EP0CSR_IRDY) == 0) :
        ((INCSR1n & INCSR1n_IRDY) == 0)) {
        usb_pio_write(ep);
    }

    restore_irq(flags);

    if (wait) {
        semaphore_wait(&ep->completion_sem, TIMEOUT_BLOCK);
    }

    return 0;
}

int usb_drv_send_nonblocking(int endpoint, void *ptr, int length, bool wait)
{
    return usb_drv_send_internal(endpoint, ptr, length, false);
}

int usb_drv_send(int endpoint, void *ptr, int length, bool wait)
{
    return usb_drv_send_internal(endpoint, ptr, length, true);
}

/* This is a separate function because it is called both from
 * usb_drv_recv and from the interrupt handler. */
static int usb_pio_read(struct tcc_ep *ep)
{
    register volatile unsigned short *fifo = ep->fifo;
    unsigned int l;
    unsigned char *p = ep->buf;
    unsigned int i;

    l = MIN(ep->bytesleft, OFIFO1n & 0xFF);
    l = MIN(l, USB_PACKET_SIZE);
    for (i = 0; i < l; i++) {
        *(p++) = *fifo;
    }

    ep->bytesleft -= l;

    if (ep->id == EP_CONTROL) {
        if (ep->bytesleft == 0) {
            EP0CSR |= EP0CSR_CLOR | EP0CSR_DEND;
            die("NOTHERE");
            usb_core_transfer_complete(EP_CONTROL, USB_DIR_OUT, 0, ep->count);
        } else {
            EP0CSR |= EP0CSR_CLOR;
            ep->buf = p;
        }
    } else {
        OCSR1n &= ~OCSR1n_ORDY;
        if (ep->bytesleft == 0 || l < USB_PACKET_SIZE) {
            usb_core_transfer_complete(ep->id & USB_ENDPOINT_NUMBER_MASK,
                                       USB_DIR_OUT, 0, ep->count);
            ep->bytesleft = -1;
        } else {
            ep->buf = p;
        }
    }

    return 0; // FIXME can it fail?
}


int usb_drv_recv(int endpoint, void* ptr, int length)
{
    //printf("recv: %d, %d", endpoint, length);
    int flags;
    //int rc = 0;
    unsigned short epidx = endpoint & USB_ENDPOINT_NUMBER_MASK;
    struct tcc_ep *ep = &tcc_endpoints[endpoint & 0x7f];

    if (endpoint == EP_CONTROL && length != 0) {
        die("FUCK");
    }
    if (endpoint == EP_CONTROL && length == 0) {
        /* Use zero length send as end of setup packet */
        //printf("CLOR");
        flags = disable_irq_save();
        UBIDX = 0;
        EP0CSR |= EP0CSR_CLOR;
        restore_irq(flags);
        return 0;
    } else if ((ep->id & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN || length == 0) {
        panicf_my("%s(%d,%d): Not supported", __func__, endpoint, length);
    }

    DEBUG(2, "%s(%d,%d):", __func__, endpoint, length);

    flags = disable_irq_save();

    if(ep->bytesleft >= 0) {
        panicf_my("%s: ep is already busy", __func__);
    }

    ep->buf = (unsigned char *)ptr;
    ep->bytesleft = length;
    ep->count = length;

    UBIDX = epidx;

    /* Data that has already arrived is read here. Any further reads happen
     * via interrupts. */
    if ((endpoint == EP_CONTROL) ?
        (EP0CSR & EP0CSR_ORDY) : (OCSR1n & OCSR1n_ORDY)) {
        usb_pio_read(ep);
    }

    restore_irq(flags);

    return 0;
}

void usb_drv_cancel_all_transfers(void)
{
#if 0
    int endpoint;
    int flags;

    DEBUG(2, "%s", __func__);

    flags = disable_irq_save();
    for (endpoint = 0; endpoint < 4; endpoint++) {
        if (tcc_endpoints[endpoint].buf) {
/*            usb_core_transfer_complete(tcc_endpoints[endpoint].id,
                                        tcc_endpoints[endpoint].dir, -1, 0); */
            tcc_endpoints[endpoint].buf = NULL;
        }
    }

    global_ep_irq_mask = 1;
    TCC7xx_USB_EPIE = global_ep_irq_mask;
    TCC7xx_USB_EPIF = TCC7xx_USB_EPIF_IRQ_MASK;
    restore_irq(flags);
#endif
}

void usb_drv_set_test_mode(int mode)
{
    panicf_my("%s(%d)", __func__, mode);
}

bool usb_drv_stalled(int endpoint, bool in)
{
    panicf_my("%s(%d,%d)", __func__, endpoint, in);
    return false;
}

void usb_drv_stall(int endpoint, bool stall,bool in)
{
    (void) endpoint;
    (void) stall;
    (void) in;
    printf("%s(%d,%d,%d)", __func__, endpoint, stall, in);
}

void usb_drv_init(void)
{
    size_t i;

    DEBUG(2, "%s", __func__);

    for (i = 0; i < sizeof(tcc_endpoints)/sizeof(struct tcc_ep); i++) {
        tcc_endpoints[i].allocated = false;
        semaphore_init(&tcc_endpoints[i].completion_sem, 1, 0);
    }

    /* Driving USB GPIO low will cause re-enumeration */
    GSEL_B &= ~0xC000000;
    GTSEL_B &= ~0xC000000;
    GPIOB_DIR |= 0xC000000;
    GPIOB &= ~0xC000000;

    /* Enable USB clock and reset USB device */
    SW_nRST &= ~SW_nRST_UB;
    HCLKSTOP &= ~HCLKSTOP_USBD;
    UBCLKmode = 0x103; /* 192 / 4 = 48 MHz */
    CKCTRL &= ~CKCTRL_USB;
    sleep(HZ/4); // FIXME how long?
    SW_nRST |= SW_nRST_UB;

    /* USB address 0 */
    UBFADR = UBFADR_UP;

    /* Clear all interrupts */
    UBEIR |= UBEIR_EP2 | UBEIR_EP1 | UBEIR_EP0;
    UBIR |= UBIR_RST | UBIR_RSM | UBIR_SP;

    /* Enable interrupts */
    UBEIEN |= UBEIEN_EP2 | UBEIEN_EP1 | UBEIEN_EP0;;
    UBIEN |= UBIEN_RST | UBIEN_SP;

    /* Enable USB interrupts */
    CREQ = UB_IRQ_MASK;
    IRQSEL |= UB_IRQ_MASK;
    IEN |= UB_IRQ_MASK;

    /* Give pins to USB controller. This should let host see pullup,
     * and issue a reset. */
    GSEL_B |= 0xC000000;
}

void usb_drv_exit(void)
{
#if 0
    TCC7xx_USB_EPIE = 0;
    BCLKCTR &= ~DEV_USBD;

    SWRESET |= DEV_USBD;
    udelay(50);
    SWRESET &= ~DEV_USBD;

    pullup_off();
#endif
}

void usb_init_device(void)
{
}

void usb_enable(bool on)
{
    if (on)
        usb_core_init();
    else
        usb_core_exit();
}

void usb_attach(void)
{
   usb_enable(true);
}

int usb_detect(void)
{
    /* TODO: not correct for all targets, we should poll VBUS
       signal on USB bus. */
    //if (charger_inserted())
        return USB_INSERTED;
    //return USB_EXTRACTED;
}

#ifdef BOOTLOADER
#include "sd.h"
void usb_test(void)
{
    int rc;

    rc = sd_init();

    if(rc) {
        panicf("sd_init failed");
    }

    usb_init();
    usb_start_monitoring();
    usb_acknowledge(SYS_USB_CONNECTED_ACK);

    while (1) {
        sleep(HZ);
//        usb_serial_send("Hello\r\n", 7);
    }
}
#endif
#else
void usb_init_device(void)
{
#if 0
    /* simply switch USB off for now */
    BCLKCTR |= DEV_USBD;
    TCC7xx_USB_PHY_CFG = 0x3e4c;
    BCLKCTR &= ~DEV_USBD;
#endif
}

void usb_enable(bool on)
{
    (void)on;
}

/* Always return false for now */
int usb_detect(void)
{
    return USB_EXTRACTED;
}
#endif
