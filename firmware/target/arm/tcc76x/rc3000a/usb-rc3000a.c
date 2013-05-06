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

#define TCC7xx_USB_EPIF_IRQ_MASK 0xf

static bool setup_phase = true;
static int dbg_level = 0x00;
static int global_ep_irq_mask = 0x1;
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
    //int mask;      /* Endpoint bit mask */
    int count;     /* Size of entire transfer, for later completion callback. */
    /* Data to send/recv. Updated during transfer. */
    char *buf;     /* Pointer to data or NULL if none. */
    int bytesleft; /* Bytes left to transfer, starting at buf. */
    volatile unsigned short *fifo; /* FIFO register to read/write bytes via PIO */
    unsigned short packet_size;
    unsigned short irdy;
    struct semaphore completion_sem;
};

static struct tcc_ep tcc_endpoints[] = {
    /* control */
    {
        .allocated = false,
        .id = EP_CONTROL,
        .buf = NULL,
        .fifo = &EP0FIFO,
        .packet_size = 64,
        .irdy = EP0CSR_IRDY,
    }, {  /* bulk */
        .allocated = false,
        .buf = NULL,
        .fifo = &EP1FIFO,
        .packet_size = 64,
        .irdy = INCSR1n_IRDY,
    }, { /* bulk */
        .allocated = false,
        .buf = NULL,
        .fifo = &EP2FIFO,
        .packet_size = 64,
        .irdy = INCSR1n_IRDY,
    },
} ;

static bool usb_drv_write_ep(struct tcc_ep *ep);
static void usb_set_speed(int);
static int usb_pio_write_packet(struct tcc_ep *ep);

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
    for (ep = 1; ep <= USB_NUM_ENDPOINTS; ep++) {
        if (!tcc_endpoints[ep].allocated) {
            tcc_endpoints[ep].allocated = true;
            tcc_endpoints[ep].id = ep | ep_dir;
            ret = ep | ep_dir;
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

    if (ep < 1 || ep > USB_NUM_ENDPOINTS)
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

#if 0
static
void handle_control(void)
{
    /* control are always 8 bytes len */
    static unsigned char ep_control[8];
    struct usb_ctrlrequest *req =
        (struct usb_ctrlrequest *) ep_control;
    unsigned short stat;
    unsigned short count = 0;
    int i;
    int type;

    /* select control endpoint */
    TCC7xx_USB_INDEX = 0x00;
    stat = TCC7xx_USB_EP0_STAT;

    if (stat & 0x10) {
        DEBUG(2, "stall");
        TCC7xx_USB_EP0_STAT = 0x10;
    }

    if (TCC7xx_USB_EP0_STAT & 0x01) { /* RX */
        uint16_t *ptr = (uint16_t *) ep_control;

        count = TCC7xx_USB_EP_BRCR;

        if (TCC7xx_USB_EP0_STAT & 0x2)
            TCC7xx_USB_EP0_STAT = 0x02;

        if (count != 4) { /* bad control? */
            unsigned short dummy;

            while (count--)
                dummy = TCC7xx_USB_EP0_BUF;
                DEBUG(1, "WTF: count = %d", count);
            } else {
                /* simply read control packet */
                for (i = 0; i < count; i++)
                    ptr[i] = TCC7xx_USB_EP0_BUF;
            }

            count *= 2;
            TCC7xx_USB_EP0_STAT = 0x01;
            DEBUG(1, "CTRL: len = %d %04x", count, stat);
    } else if (TCC7xx_USB_EP0_STAT & 0x02) { /* TX */
        TCC7xx_USB_EP0_STAT = 0x02;
        DEBUG(2, "TX Done\n");
    } else {
        DEBUG(1, "stat: %04x", stat);
    }

    TCC7xx_USB_EPIF = 1;

    if (0 == (stat & 0x1) || count != 8)
        return ;
#if 1 /* TODO: remove me someday */
    {
        int i;
        uint16_t *ptr = (uint16_t *) ep_control;
        for (i = 1; i < (count>>1); i++) {
            if (ptr[i] != ptr[0])
                break;
        }
        if (i == (count>>1)) {
            /*DEBUG(2, */panicf_my("sanity failed");
            return ;
        }
    }
#endif
    type = req->bRequestType;

    /* TODO: don't pass some kinds of requests to upper level */
    switch (req->bRequest) {
    case USB_REQ_CLEAR_FEATURE:
        DEBUG(2, "USB_REQ_CLEAR_FEATURE");
        DEBUG(2, "...%04x %04x", req->wValue, req->wIndex);
        break;
    case USB_REQ_SET_ADDRESS:
        //DEBUG(2, "USB_REQ_SET_ADDRESS, %d %d", req->wValue, TCC7xx_USB_FUNC);
        /* seems we don't have to set it manually
           TCC7xx_USB_FUNC = req->wValue; */
        break;
    case USB_REQ_GET_DESCRIPTOR:
        DEBUG(2, "gd, %02x %02x", req->wValue, req->wIndex);
        break;
    case USB_REQ_GET_CONFIGURATION:
        DEBUG(2, "USB_REQ_GET_CONFIGURATION");
        break;
    default:
        DEBUG(2, "req: %02x %02d", req->bRequestType, req->bRequest);
    }

    usb_core_control_request(req);
}

static
void handle_ep_in(struct tcc_ep *tcc_ep, uint16_t stat)
{
    uint8_t *buf = tcc_ep->buf;
    uint16_t *wbuf = (uint16_t *) buf;
    int wcount;
    int count;
    int i;

    if (tcc_ep->dir != USB_DIR_OUT) {
        panicf_my("ep%d: is input only", tcc_ep->id);
    }

    wcount = TCC7xx_USB_EP_BRCR;

    DEBUG(2, "ep%d: %04x %04x", tcc_ep->id, stat, wcount);

    /* read data */
    count = wcount * 2;
    if (stat & TCC7xx_USP_EP_STAT_LWO) {
        count--;
        wcount--;
    }

    if (buf == NULL)
        panicf_my("ep%d: Unexpected packet! %d %x", tcc_ep->id, count, TCC7xx_USB_EP_CTRL);
    if (tcc_ep->max_len < count)
        panicf_my("Too big packet: %d excepted %d %x", count, tcc_ep->max_len, TCC7xx_USB_EP_CTRL);

    for (i = 0; i < wcount; i++)
        wbuf[i] = *tcc_ep->ep;

    if (count & 1) { /* lwo */
        uint16_t tmp = *tcc_ep->ep;
        buf[count - 1] = tmp & 0xff;
    }

    tcc_ep->buf = NULL;

    TCC7xx_USB_EP_STAT = TCC7xx_USB_EP_STAT;
    TCC7xx_USB_EPIF = tcc_ep->mask;
    TCC7xx_USB_EPIE &= ~tcc_ep->mask; /* TODO: use INGLD? */
    global_ep_irq_mask &= ~tcc_ep->mask;

    if (TCC7xx_USB_EP_STAT & 0x1)
        panicf_my("One more packet?");

    TCC7xx_USB_EP_CTRL |= TCC7xx_USB_EP_CTRL_OUTHD;

    usb_core_transfer_complete(tcc_ep->id, USB_DIR_OUT, 0, count);
}

static
void handle_ep_out(struct tcc_ep *tcc_ep, uint16_t stat)
{
    bool done;
    (void) stat;

    if (tcc_ep->dir != USB_DIR_IN) {
        panicf_my("ep%d: is out only", tcc_ep->id);
    }

//    if (tcc_ep->buf == NULL) {
//        panicf_my("%s:%d", __FILE__, __LINE__);
//    }

    done = usb_drv_write_ep(tcc_ep);

//    TCC7xx_USB_EP_STAT = 0x2; /* Clear TX stat */
    TCC7xx_USB_EPIF = tcc_ep->mask;

    if (done) { // tcc_ep->buf == NULL) {
        TCC7xx_USB_EPIE &= ~tcc_ep->mask;
        global_ep_irq_mask &= ~tcc_ep->mask;

//        usb_core_transfer_complete(tcc_ep->id, USB_DIR_IN, 0, tcc_ep->count);
    }
}

static
void handle_ep(unsigned short ep_irq)
{
    if (ep_irq & 0x1) {
        handle_control();
    }

    if (ep_irq & 0xe) {
        int endpoint;

        for (endpoint = 1; endpoint < 4; endpoint++) {
            struct tcc_ep *tcc_ep = &tcc_endpoints[endpoint];
            uint16_t stat;

            if (0 == (ep_irq & (1 << endpoint)))
                continue;
            if (!tcc_ep->allocated)
                panicf_my("ep%d: wasn't requested", endpoint);

            TCC7xx_USB_INDEX = endpoint;
            stat = TCC7xx_USB_EP_STAT;

            DEBUG(1, "ep%d: %04x", endpoint, stat);

            if (stat & 0x1)
                handle_ep_in(tcc_ep, stat);
            else if (stat & 0x2)
                handle_ep_out(tcc_ep, stat);
            else /* TODO: remove me? */
                panicf_my("Unhandled ep%d state: %x, %d", endpoint, TCC7xx_USB_EP_STAT, TCC7xx_USB_INDEX);
        }
    }
}

static void usb_set_speed(int high_speed)
{
    TCC7xx_USB_EP_DIR = 0x0000;

    /* control endpoint */
    TCC7xx_USB_INDEX = 0;
    TCC7xx_USB_EP0_CTRL = 0x0000;
    TCC7xx_USB_EP_MAXP = 64;
    TCC7xx_USB_EP_CTRL = TCC7xx_USB_EP_CTRL_CDP | TCC7xx_USB_EP_CTRL_FLUSH;

    /* ep1: bulk-in, to host */
    TCC7xx_USB_INDEX = 1;
    TCC7xx_USB_EP_DIR |= (1 << 1);
    TCC7xx_USB_EP_CTRL = TCC7xx_USB_EP_CTRL_CDP;

    if (high_speed)
        TCC7xx_USB_EP_MAXP = 512;
    else
        TCC7xx_USB_EP_MAXP = 64;

    TCC7xx_USB_EP_DMA_CTRL = 0x0;

    /* ep2: bulk-out, from host */
    TCC7xx_USB_INDEX = 2;
    TCC7xx_USB_EP_DIR &= ~(1 << 2);
    TCC7xx_USB_EP_CTRL = TCC7xx_USB_EP_CTRL_CDP;

    if (high_speed)
        TCC7xx_USB_EP_MAXP = 512;
    else
        TCC7xx_USB_EP_MAXP = 64;

    TCC7xx_USB_EP_DMA_CTRL = 0x0;

    /* ep3: interrupt in */
    TCC7xx_USB_INDEX = 3;
    TCC7xx_USB_EP_DIR &= ~(1 << 3);
    TCC7xx_USB_EP_CTRL = TCC7xx_USB_EP_CTRL_CDP;
    TCC7xx_USB_EP_MAXP = 64;

    TCC7xx_USB_EP_DMA_CTRL = 0x0;
}
#endif

/*
  Reset TCC7xx usb device
 */
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
    /* Initialize control endpoint */
    UBIDX = 0;
    MAXP = MAXP_64_BYTES;

    usb_core_bus_reset();
}

static inline void handle_control_interrupt(void) {
    unsigned long ep0csr_val;

    UBIDX = 0;
    ep0csr_val = EP0CSR;
    //printf("E0C: %x", ep0csr_val);

#if 0
#define EP0CSR_CLSE (1 << 7) /* Clear Setup End Bit */
#define EP0CSR_CLOR (1 << 6) /* Clear Output Packet Ready Bit */
#define EP0CSR_ISST (1 << 5) /* Issue STALL Handshake */
#define EP0CSR_CEND (1 << 4) /* Control Transfer End */
#define EP0CSR_DEND (1 << 3) /* Data End */
#define EP0CSR_STST (1 << 2) /* STALL Handshake Issued */
#define EP0CSR_IRDY (1 << 1) /* IN Packet Ready */
#define EP0CSR_ORDY (1 << 0) /* OUT Packet Ready */
#endif

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
        //if (setup_phase) {
            /* This should be setup token */
            static unsigned char ctrldata[8];
            int i;
            unsigned char *p = (unsigned char *)ctrldata;
            printf ("CPR: %d", OFIFO1);
            for (i = 0; i < 8; i++) {
                p[i] = EP0FIFO;
            }
            printf("0: %02x %02x %02x %02x", p[0], p[1], p[2], p[3]);
            printf("4: %02x %02x %02x %02x", p[4], p[5], p[6], p[7]);
            //EP0CSR = EP0CSR_CLOR;
            lcd_update();
            /* For setup tokens, usb_core_control_request is called */
            usb_core_control_request((struct usb_ctrlrequest*)ctrldata);
            //setup_phase = false;
        //} else {
            /* This should be data phase data from host */
        //}
    } else if (ep0csr_val & EP0CSR_IRDY) {
        /* Ready to send packet */
        if (tcc_endpoints[0].buf != NULL) {
            /* Data available to send */
            usb_pio_write_packet(&tcc_endpoints[0]);
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

    //printf("USBD: %d %d", ubir_val, ubeir_val);
    lcd_update();

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
        handle_control_interrupt();
    }
#if 0
    unsigned short sys_stat;
    unsigned short ep_irq;
    unsigned short index_save;

    sys_stat = TCC7xx_USB_SYS_STAT;

    if (sys_stat & TCC7xx_USB_SYS_STAT_RESET) {
        TCC7xx_USB_SYS_STAT = TCC7xx_USB_SYS_STAT_RESET;
        usb_reset();
        TCC7xx_USB_SYS_CTRL |= TCC7xx_USB_SYS_CTRL_SUSPEND;
        DEBUG(2, "reset");
    }

    if (sys_stat & TCC7xx_USB_SYS_STAT_RESUME) {
        TCC7xx_USB_SYS_STAT = TCC7xx_USB_SYS_STAT_RESUME;
        usb_reset();
        TCC7xx_USB_SYS_CTRL |= TCC7xx_USB_SYS_CTRL_SUSPEND;
        DEBUG(2, "resume");
    }

    if (sys_stat & TCC7xx_USB_SYS_STAT_SPD_END) {
        usb_set_speed(1);
        TCC7xx_USB_SYS_STAT = TCC7xx_USB_SYS_STAT_SPD_END;
        DEBUG(2, "spd end");
    }

    if (sys_stat & TCC7xx_USB_SYS_STAT_ERRORS) {
        DEBUG(2, "errors: %4x", sys_stat & TCC7xx_USB_SYS_STAT_ERRORS);
        TCC7xx_USB_SYS_STAT = sys_stat & TCC7xx_USB_SYS_STAT_ERRORS;
    }

//    TCC7xx_USB_SYS_STAT = sys_stat;

    index_save = TCC7xx_USB_INDEX;

    ep_irq = TCC7xx_USB_EPIF & global_ep_irq_mask;

    while (ep_irq & TCC7xx_USB_EPIF_IRQ_MASK) {
        handle_ep(ep_irq);

        /* is that really needed, btw not a problem for rockbox */
        udelay(50);
        ep_irq = TCC7xx_USB_EPIF & global_ep_irq_mask;
    }

    TCC7xx_USB_INDEX = index_save;
#endif
}

void usb_drv_set_address(int address)
{
    UBFADR = UBFADR_UP | address;
    // DEBUG(2, "setting address %d %d", address, TCC7xx_USB_FUNC);
}

int usb_drv_port_speed(void)
{
    //return (TCC7xx_USB_SYS_STAT & 0x10) ? 1 : 0;
}

#if 0
static int usb_drv_write_packet(volatile unsigned short *buf, unsigned char *data, int len, int max)
{
    uint16_t  *wbuf = (uint16_t *) data;
    int count, i;

    len = MIN(len, max);
    count = (len + 1) / 2;

    TCC7xx_USB_EP_BWCR = len;

    for (i = 0; i < count; i++)
        *buf = *wbuf++;

    return len;
}

static bool usb_drv_write_ep(struct tcc_ep *ep)
{
    int count;

    if (ep->max_len == 0)
        return true;

    count = usb_drv_write_packet(ep->ep, ep->buf, ep->max_len, 512);
    TCC7xx_USB_EP_STAT = 0x2; /* Clear TX stat */

    ep->buf += count;
    ep->count += count;
    ep->max_len -= count;

    if (ep->max_len == 0) {
        usb_core_transfer_complete(ep->id, USB_DIR_IN, 0, ep->count);
        ep->buf = NULL;
//        return true;
    }

    return false;
}
#endif

#if 0
int usb_drv_send(int endpoint, void *ptr, int length)
{
    //printf("send: %d, %d", endpoint, length);
    if (endpoint == EP_CONTROL) {
        if (setup_phase && length == 0) {
            printf("CLOR, DEND");
            EP0CSR |= EP0CSR_CLOR | EP0CSR_DEND;
            return 0;
        } //else {
        setup_phase = false;
        int i;
        while (EP0CSR & EP0CSR_IRDY);
        for (i = 0; i < length; i++) {
            EP0FIFO = ((unsigned char *)ptr)[i];
        }
        /* FIXME is this right way to detect end of control transfer? */
            printf("IRDY, DEND");
            EP0CSR |= EP0CSR_IRDY | EP0CSR_DEND;
//        }
        //usb_drv_recv(EP_CONTROL, NULL, 0);
            setup_phase = true;
        return 0;
    }
#if 0
    int flags = disable_irq_save();
    int rc = 0;
    char *data = (unsigned char*) ptr;

    DEBUG(2, "%s(%d,%d)" , __func__, endpoint, length);

    if (endpoint != 0)
        panicf_my("%s(%d,%d)", __func__, endpoint, length);

    TCC7xx_USB_INDEX = 0;
    while (length > 0) {
        int ret;

        ret = usb_drv_write_packet(&TCC7xx_USB_EP0_BUF, data, length, 64);
        length -= ret;
        data += ret;

        while (0 == (TCC7xx_USB_EP0_STAT & 0x2))
            ;
        TCC7xx_USB_EP0_STAT = 0x2;
    }

    restore_irq(flags);
    return rc;
#endif
}
#endif

/* This is a separate function because it is called both from
 * usb_drv_send and from the interrupt handler. */
static int usb_pio_write_packet(struct tcc_ep *ep)
{
    register volatile unsigned short *fifo = ep->fifo;
    unsigned int l = MIN(ep->bytesleft, ep->packet_size);
    unsigned char *p = ep->buf;
    unsigned int i;
    for (i = 0; i < l; i++) {
        *fifo = *(p++);
    }

    ep->bytesleft -= l;

    if (ep->bytesleft == 0) {
        ep->buf = NULL;
        INCSR1n = ep->irdy |
                  ((ep->id == EP_CONTROL) ? EP0CSR_DEND : 0);
        semaphore_release(&ep->completion_sem);
        usb_core_transfer_complete(ep->id, USB_DIR_OUT, 0, ep->count);
    } else {
        ep->buf = p;
        INCSR1n |= ep->irdy;
    }

    return 0; // FIXME can it fail?
}

static int usb_drv_send_internal(int endpoint, void *ptr, int length, bool wait)
{
    int flags;
    int rc = 0, count = length;
    char *data = (unsigned char*) ptr;
    unsigned short epidx = endpoint & 0x7f;
    struct tcc_ep *ep;
    unsigned short irdy;

    ep = &tcc_endpoints[epidx];

    if (endpoint == EP_CONTROL) {
        /* Assume 0 length send means no data phase */
        if (length == 0) {
            printf("CLOR, DEND");
            EP0CSR |= EP0CSR_CLOR | EP0CSR_DEND;
            return 0;
        }
    } else if ((ep->id & USB_ENDPOINT_DIR_MASK) != USB_DIR_IN || length == 0) {
        panicf_my("%s(%d,%d): Not supported", __func__, endpoint, length);
    }

    DEBUG(2, "%s(%d,%d):", __func__, endpoint, length);

    flags = disable_irq_save();

    if(ep->buf != NULL) {
        panicf_my("%s: ep is already busy", __func__);
    }

    ep->buf = data;
    ep->bytesleft = length;
    ep->count = count;

    UBIDX = epidx;

    /* If IRDY is low, the IRDY interrupt was probably already handled.
       A packet must be written now to start the transfer. After it is sent,
       hardware clears IRDY, generating an interrupt. Such interrupts
       will then send the rest of the data, if any remains. */
//#if &EP0CSR != &INCSR1n
//#error Code assumes EP0CSR == INCSR1n
//#endif
    if ((INCSR1n & ep->irdy) == 0) {
        usb_pio_write_packet(ep);
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

int usb_drv_recv(int endpoint, void* ptr, int length)
{
    //printf("recv: %d, %d", endpoint, length);
    if (endpoint == EP_CONTROL) {
        int i;
        while ((EP0CSR & EP0CSR_ORDY) == 0);
        for (i = 0; i < length; i++) {
            ((unsigned char *)ptr)[i] = EP0FIFO;
        }

        if (length == 0) {
            printf("CLOR");
            EP0CSR |= EP0CSR_CLOR;
        } else {
            printf("CLOR, DEND");
            EP0CSR |= EP0CSR_CLOR | EP0CSR_DEND;
        }
        //EP0CSR |= EP0CSR_CLOR | ((length == 0) ? 0 : EP0CSR_DEND);
        setup_phase = false;

        return 0;
    }
#if 0
    volatile struct tcc_ep *tcc_ep = &tcc_endpoints[endpoint & 0x7f];
    int flags;

    if (length == 0) {
        if (endpoint != 0)
            panicf_my("%s(%d,%d) zero length?", __func__, endpoint, length);
        return 0;
    }
    // TODO: check ep
    if (tcc_ep->dir != USB_DIR_OUT)
        panicf_my("%s(%d,%d)", __func__, endpoint, length);

    DEBUG(2, "%s(%d,%d)", __func__, endpoint, length);

    flags = disable_irq_save();

    if (tcc_ep->buf) {
        panicf_my("%s: overrun: %x %x", __func__,
                  (unsigned int)tcc_ep->buf, (unsigned int)tcc_ep);
    }

    tcc_ep->buf = ptr;
    tcc_ep->max_len = length;
    tcc_ep->count = 0;

    TCC7xx_USB_INDEX = tcc_ep->id;

    TCC7xx_USB_EP_CTRL &= ~TCC7xx_USB_EP_CTRL_OUTHD;
    TCC7xx_USB_EPIE |= tcc_ep->mask;
    global_ep_irq_mask |= tcc_ep->mask;

    restore_irq(flags);

    return 0;
#endif
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
        tcc_endpoints[i].buf = NULL;
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
    sleep(HZ/2); // FIXME how long?
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
//#include "ata.h"
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
