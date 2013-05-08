/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id: wm8975.c 28572 2010-11-13 11:38:38Z theseven $
 *
 * Driver for Cirrus Logic CS42L55 audio codec
 *
 * Copyright (c) 2010 Michael Sparmann
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
#include "logf.h"
#include "system.h"
#include "kernel.h"
#include "string.h"
#include "audio.h"
#include "sound.h"
#include "audiohw.h"
#include "cscodec.h"
#include "cs42l51.h"

const struct sound_settings_info audiohw_settings[] = {
    [SOUND_VOLUME]        = {"dB", 0,  1,-102,  12, -25},
    [SOUND_BASS]          = {"dB", 1, 15,-105, 120,   0},
    [SOUND_TREBLE]        = {"dB", 1, 15,-105, 120,   0},
    [SOUND_BALANCE]       = {"%",  0,  1,-100, 100,   0},
    [SOUND_CHANNELS]      = {"",   0,  1,   0,   5,   0},
    [SOUND_STEREO_WIDTH]  = {"%",  0,  5,   0, 250, 100},
    [SOUND_BASS_CUTOFF]   = {"",   0,  1,   1,   4,   2},
    [SOUND_TREBLE_CUTOFF] = {"",   0,  1,   1,   4,   1},
};

static int bass, treble;

static void cscodec_freeze(bool freeze)
{
    cscodec_write(CS42L51_DAC_CTL,
                  /* Route data via Signal Processing Engine to DAC */
                  /* FIXME is this needed when not using treble or bass */
                  CS42L51_DAC_CTL_DATA_SEL(1) | \
                  /* Make modifications take effect after unfreeze */
                  (freeze ? CS42L51_DAC_CTL_FREEZE : 0) | \
                  /* Change volume on zero crossings */
                  CS42L51_DAC_CTL_DACSZ(1));
}


/* convert tenth of dB volume (-600..120) to master volume register value */
int tenthdb2master(int db)
{
    // FIXME if (db < VOLUME_MIN) return HPACTL_HPAMUTE;
    return (db / 5);
}

#if 0
static void cscodec_setbits(int reg, unsigned char off, unsigned char on)
{
    unsigned char data = (cscodec_read(reg) & ~off) | on;
    cscodec_write(reg, data);
}
#endif

static void audiohw_mute(bool mute)
{
    cscodec_write(CS42L51_DAC_OUT_CTL,
                  /* Maximum headphone gain multiplier */
                  CS42L51_DAC_OUT_CTL_HP_GAIN(7) | \
                  /* Mute DAC */
                  (mute ? (CS42L51_DAC_OUT_CTL_DACB_MUTE | \
                           CS42L51_DAC_OUT_CTL_DACA_MUTE)
                        : 0));
}

void audiohw_preinit(void)
{
    cscodec_power(true);
    cscodec_clock(true);
    cscodec_reset(true);
    sleep(HZ / 100);
    cscodec_reset(false);

    bass = 0;
    treble = 0;
    // FIXME preserve reserved values
    cscodec_write(CS42L51_POWER_CTL1, CS42L51_POWER_CTL1_PDN);
    cscodec_write(CS42L51_MIC_POWER_CTL,
                  /* Single speed mode, for 4 to 50 kHz sample rates */
                  CS42L51_MIC_POWER_CTL_SPEED(CS42L51_SSM_MODE) | \
                  /* RC3000A has no microphone */
                  CS42L51_MIC_POWER_CTL_PDN_MICB | \
                  CS42L51_MIC_POWER_CTL_PDN_MICA | \
                  CS42L51_MIC_POWER_CTL_PDN_BIAS | \
                  /* MCLK is 256fs, but need 128fs */
                  CS42L51_MIC_POWER_CTL_MCLK_DIV2);
    cscodec_write(CS42L51_INTF_CTL,
                  /* Use I2S */
                  CS42L51_INTF_CTL_DAC_FORMAT(CS42L51_DAC_DIF_I2S) | \
                  CS42L51_INTF_CTL_ADC_I2S);
    cscodec_freeze(true);
    cscodec_write(CS42L51_PCMA_VOL, 0);
    cscodec_write(CS42L51_PCMB_VOL, 0);
    cscodec_write(CS42L51_ADCA_VOL, 0);
    cscodec_write(CS42L51_ADCB_VOL, 0);
    cscodec_write(CS42L51_ADC_INPUT,
                  /* Mute ADC */
                  CS42L51_ADC_INPUT_ADCB_MUTE | CS42L51_ADC_INPUT_ADCA_MUTE);
    audiohw_mute(true);
    cscodec_freeze(false);
    cscodec_write(CS42L51_POWER_CTL1,
                  /* Remove overall powerdown, only powering down PGA */
                  CS42L51_POWER_CTL1_PDN_PGAB | \
                  CS42L51_POWER_CTL1_PDN_PGAA);


    // FIXME sequence for selecting radio input
#if 0
    cscodec_freeze(true);

    /* OF uses 18 for stereo and 24 for mono */
    // FIXME why init PGA when PGA is PDN
    cscodec_write(CS42L51_ALC_PGA_CTL, 18);
    cscodec_write(CS42L51_ALC_PGB_CTL, 18);

    /* ADC volume +3dB, not muted */
    cscodec_write(CS42L51_ADCA_VOL,6);
    cscodec_write(CS42L51_ADCB_VOL,6);

    /* PCM volume 0dB, muted */
    cscodec_write(CS42L51_PCMA_VOL, 1);// FIXME 0x80);
    cscodec_write(CS42L51_PCMB_VOL, 1); //0x80);

    /* Input AIN1 to ADC */
    cscodec_write(CS42L51_ADC_INPUT,0);

    cscodec_freeze(false);

    /* FIXME WTF */
    cscodec_write(CS42L51_POWER_CTL1,
              /* Remove overall powerdown, only powering down PGA */
              CS42L51_POWER_CTL1_PDN_PGAA | \
              CS42L51_POWER_CTL1_PDN_PGAB);

    /* FIXME worse hack */
    cscodec_write(CS42L51_INTF_CTL,
    CS42L51_INTF_CTL_LOOPBACK /* not | CS42L51_INTF_CTL_MASTER */ | \
              /* Use I2S */
              CS42L51_INTF_CTL_DAC_FORMAT(CS42L51_DAC_DIF_I2S) | \
              CS42L51_INTF_CTL_ADC_I2S);
#else
    /* PCM volume 0dB, muted */
    cscodec_write(CS42L51_PCMA_VOL, 0);// FIXME 0x80);
    cscodec_write(CS42L51_PCMB_VOL, 0); //0x80);
#endif
              audiohw_mute(false);

}

void audiohw_postinit(void)
{
#if 0
    cscodec_write(HPACTL, 0);
    cscodec_write(HPBCTL, 0);
    cscodec_write(LINEACTL, 0);
    cscodec_write(LINEBCTL, 0);
    cscodec_write(CLSHCTL, CLSHCTL_ADPTPWR_SIGNAL);
    audiohw_mute(false);
#endif
}

void audiohw_set_master_vol(int vol_l, int vol_r)
{
    /* -102dB to +12dB in 0.5dB steps */
    /* 00011000 == +12dB  (0x18) */
    /* 00000000 == 0dB    (0x0)  */
    /* 11111111 == 0.5dB  (0xFF) */
    /* 00011001 == -102dB (0x19) */

    cscodec_freeze(1);
    cscodec_write(CS42L51_AOUTA_VOL, vol_l & 0xFF);
    cscodec_write(CS42L51_AOUTB_VOL, vol_r & 0xFF);
    cscodec_freeze(0);
}

static void handle_dsp_power(void)
{
#if 0
    if (bass || treble)
    {
        cscodec_setbits(PLAYCTL, PLAYCTL_PDN_DSP, 0);
        cscodec_setbits(BTCTL, 0, BTCTL_TCEN);
    }
    else
    {
        cscodec_setbits(BTCTL, BTCTL_TCEN, 0);
        cscodec_setbits(PLAYCTL, 0, PLAYCTL_PDN_DSP);
    }
#endif
}

void audiohw_set_bass(int value)
{
#if 0
    bass = value;
    handle_dsp_power();
    if (value >= -105 && value <= 120)
        cscodec_setbits(TONECTL, TONECTL_BASS_MASK,
                        (8 - value / 15) << TONECTL_BASS_SHIFT);
#endif
}

void audiohw_set_treble(int value)
{
#if 0
    treble = value;
    handle_dsp_power();
    if (value >= -105 && value <= 120)
        cscodec_setbits(TONECTL, TONECTL_TREB_MASK,
                        (8 - value / 15) << TONECTL_TREB_SHIFT);
#endif
}

void audiohw_set_bass_cutoff(int value)
{
#if 0
    cscodec_setbits(BTCTL, BTCTL_BASSCF_MASK,
                    (value - 1) << BTCTL_BASSCF_SHIFT);
#endif
}

void audiohw_set_treble_cutoff(int value)
{
#if 0
    cscodec_setbits(BTCTL, BTCTL_TREBCF_MASK,
                    (value - 1) << BTCTL_TREBCF_SHIFT);
#endif
}

void audiohw_set_prescaler(int value)
{
#if 0
    cscodec_setbits(MSTAVOL, MSTAVOL_VOLUME_MASK,
                    (-value / 5) << MSTAVOL_VOLUME_SHIFT);
    cscodec_setbits(MSTBVOL, MSTBVOL_VOLUME_MASK,
                    (-value / 5) << MSTBVOL_VOLUME_SHIFT);
#endif
}

/* Nice shutdown of CS42L55 codec */
void audiohw_close(void)
{
    audiohw_mute(true);
#if 0
    cscodec_write(HPACTL, HPACTL_HPAMUTE);
    cscodec_write(HPBCTL, HPBCTL_HPBMUTE);
    cscodec_write(LINEACTL, LINEACTL_LINEAMUTE);
    cscodec_write(LINEBCTL, LINEBCTL_LINEBMUTE);
    cscodec_write(PWRCTL1, PWRCTL1_PDN_CHRG | PWRCTL1_PDN_ADCA
                         | PWRCTL1_PDN_ADCB | PWRCTL1_PDN_CODEC);
#endif
    cscodec_reset(true);
    cscodec_clock(false);
    cscodec_power(false);
}

/* Note: Disable output before calling this function */
void audiohw_set_frequency(int fsel)
{
#if 0
    if (fsel == HW_FREQ_8) cscodec_write(CLKCTL2, CLKCTL2_8000HZ);
    else if (fsel == HW_FREQ_11) cscodec_write(CLKCTL2, CLKCTL2_11025HZ);
    else if (fsel == HW_FREQ_12) cscodec_write(CLKCTL2, CLKCTL2_12000HZ);
    else if (fsel == HW_FREQ_16) cscodec_write(CLKCTL2, CLKCTL2_16000HZ);
    else if (fsel == HW_FREQ_22) cscodec_write(CLKCTL2, CLKCTL2_22050HZ);
    else if (fsel == HW_FREQ_24) cscodec_write(CLKCTL2, CLKCTL2_24000HZ);
    else if (fsel == HW_FREQ_32) cscodec_write(CLKCTL2, CLKCTL2_32000HZ);
    else if (fsel == HW_FREQ_44) cscodec_write(CLKCTL2, CLKCTL2_44100HZ);
    else if (fsel == HW_FREQ_48) cscodec_write(CLKCTL2, CLKCTL2_48000HZ);
#endif
}

#ifdef HAVE_RECORDING
//TODO: Implement
#endif /* HAVE_RECORDING */
