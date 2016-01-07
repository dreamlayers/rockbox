/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2015 by Boris Gjenero
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
#ifndef __ATA_SAT_H__
#define __ATA_SAT_H__

#define SCSI_SAT_12               0xa1
#define SCSI_SAT_16               0x85

/* Second byte of CDB */

#define SAT_MULTCOUNT(x)            ((x) >> 5)

#define SAT_PROTO(x)                (((x) >> 1) & 15)
#define SAT_PROTO_HARD_RESET        0
#define SAT_PROTO_SRST              1
/* 2 reserved */
#define SAT_PROTO_NON_DATA          3
#define SAT_PROTO_PIO_IN            4
#define SAT_PROTO_PIO_OUT           5
#define SAT_PROTO_DMA               6
#define SAT_PROTO_DMA_QUEUED        7
#define SAT_PROTO_DEV_DIAG          8
#define SAT_PROTO_DEV_RESET         9
#define SAT_PROTO_UDMA_IN          10
#define SAT_PROTO_UDMA_OUT         11
#define SAT_PROTO_FPDMA            12
/* 13, 14 reserved */
#define SAT_PROTO_RETURN_RESPONSE  15

/* Reserved in SAT 12 and only meaningful in SAT 16.
   Also valid for byte 3 (extend) of response. */
#define SAT_EXTEND                 1

/* Third byte of CDB */

#define SAT_OFFLINE(x)             (((x) >> 5) & 3)
#define SAT_OFFLINE_0              0
#define SAT_OFFLINE_2              1
#define SAT_OFFLINE_6              2
#define SAT_OFFLINE_14             3

#define SAT_CKCOND                 (1 << 5)
/* Bit 4 is reserved in both SAT 12 and SAT 16 */
#define SAT_T_DIR                  (1 << 3)
#define SAT_BYT_BLOK               (1 << 2)

#define SAT_T_LENGTH(x)            ((x) & 3)
#define SAT_T_LENGTH_0             0
#define SAT_T_LENGTH_FEATURE       1
#define SAT_T_LENGTH_SECTOR_COUNT  2
#define SAT_T_LENGTH_STPSIU        3

struct sat_12 {
    uint8_t opcode; /* == 0xA1 */
    uint8_t mult_proto;
    uint8_t flags;
    uint8_t features;
    uint8_t sector_count;
    uint8_t lba_low;
    uint8_t lba_mid;
    uint8_t lba_high;
    uint8_t device;
    uint8_t command;
    uint8_t reserved;
    uint8_t control;
};

struct sat_16 {
    uint8_t opcode; /* == 0x85 */
    uint8_t mult_proto_extend;
    uint8_t flags;
    uint8_t hi_features;
    uint8_t lo_features;
    uint8_t hi_sector_count;
    uint8_t lo_sector_count;
    uint8_t hi_lba_low;
    uint8_t lo_lba_low;
    uint8_t hi_lba_mid;
    uint8_t lo_lba_mid;
    uint8_t hi_lba_high;
    uint8_t lo_lba_high;
    uint8_t device;
    uint8_t command;
    uint8_t control;
};

struct sat_response {
    uint8_t response_code;   /* 0x72, current error */
    uint8_t sense_key;       /* 0x01, recovered error */
    uint8_t asc;             /* 0x00 */
    uint8_t ascq;            /* 0x1d ATA pass through information available */
    uint8_t reserved1;       /* 3 reserved bytes, all zero */
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t additional_len;  /* 0x0e additional sense length */

    uint8_t descriptor_code; /* == 0x09 */
    uint8_t descriptor_len;  /* == 12  */

    uint8_t extend;
    uint8_t error;
    /* If the extend bit is 0, all hi_ members are reserved. */
    uint8_t hi_sector_count;
    uint8_t lo_sector_count;
    uint8_t hi_lba_low;
    uint8_t lo_lba_low;
    uint8_t hi_lba_mid;
    uint8_t lo_lba_mid;
    uint8_t hi_lba_high;
    uint8_t lo_lba_high;
    uint8_t device;
    uint8_t status;
};

size_t ata_sat(const uint8_t *cdb, uint8_t *buf, void **sense);
size_t ata_sat_readsize(const uint8_t *cdb);
size_t ata_sat_writesize(const uint8_t *cdb);
#endif
