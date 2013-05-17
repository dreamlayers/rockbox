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

static int bass = 0, treble = 0;

static void cscodec_freeze(bool freeze)
{
    cscodec_write(CS42L51_DAC_CTL,
                  /* Route data via Signal Processing Engine to DAC.
                   * That isn't needed when not using its features, but
                   * switching back and forth causes nasty clicks */
                  CS42L51_DAC_CTL_DATA_SEL(1) | \
                  /* Make modifications take effect after unfreeze */
                  (freeze ? CS42L51_DAC_CTL_FREEZE : 0) | \
                  /* Change volume on zero crossings */
                  CS42L51_DAC_CTL_DACSZ(1));
}

#define AINPOWER_BITS (CS42L51_POWER_CTL1_PDN_PGAB | \
                       CS42L51_POWER_CTL1_PDN_PGAA | \
                       CS42L51_POWER_CTL1_PDN_ADCB | \
                       CS42L51_POWER_CTL1_PDN_ADCA)

static void cscodec_setbits(int reg, unsigned char off, unsigned char on)
{
    unsigned char data = (cscodec_read(reg) & ~off) | on;
    cscodec_write(reg, data);
}

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

    /* PDN must be set quickly to ensure CODEC goes into
     * software controlled mode. */
    cscodec_setbits(CS42L51_POWER_CTL1, 0, CS42L51_POWER_CTL1_PDN);
    /* Keep analog input off until needed */
    cscodec_setbits(CS42L51_POWER_CTL1, 0,  AINPOWER_BITS);
    cscodec_write(CS42L51_MIC_POWER_CTL,
                  /* Single speed mode, for 4 to 50 kHz sample rates */
                  CS42L51_MIC_POWER_CTL_SPEED(CS42L51_SSM_MODE) | \
                  /* Power down microphone hardware */
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
    // FIXME examine these
    cscodec_write(CS42L51_PCMA_VOL, 0);
    cscodec_write(CS42L51_PCMB_VOL, 0);
    cscodec_write(CS42L51_ADCA_VOL, 0);
    cscodec_write(CS42L51_ADCB_VOL, 0);
    cscodec_write(CS42L51_ADC_INPUT,
                  /* Mute ADC */
                  CS42L51_ADC_INPUT_ADCB_MUTE | CS42L51_ADC_INPUT_ADCA_MUTE);
    audiohw_mute(true);
    cscodec_freeze(false);
    /* Remove PDN */
    cscodec_setbits(CS42L51_POWER_CTL1, CS42L51_POWER_CTL1_PDN, 0);

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

void audiohw_set_volume(int vol_l, int vol_r)
{
    /* -102dB to +12dB in 0.5dB steps */
    /* 00011000 == +12dB  (0x18) */
    /* 00000000 == 0dB    (0x0)  */
    /* 11111111 == 0.5dB  (0xFF) */
    /* 00011001 == -102dB (0x19) */

    cscodec_freeze(1);
    cscodec_write(CS42L51_AOUTA_VOL, (vol_l / 5) & 0xFF);
    cscodec_write(CS42L51_AOUTB_VOL, (vol_r / 5) & 0xFF);
    cscodec_freeze(0);
}

static void handle_dsp_power(void)
{
    if (bass || treble)
    {
        /* Enable tone controls */
        cscodec_setbits(CS42L51_BEEP_CONF, 0, CS42L51_BEEP_CONF_TC_EN);
    }
    else
    {
        /* Disable tone controls */
        cscodec_setbits(CS42L51_BEEP_CONF, CS42L51_BEEP_CONF_TC_EN, 0);
    }
}

#ifdef AUDIOHW_HAVE_BASS_CUTOFF
void audiohw_set_bass(int value)
{
    bass = value;
    handle_dsp_power();
    if (value >= -105 && value <= 120)
        cscodec_setbits(CS42L51_TONE_CTL, CS42L51_TONE_CTL_BASS(15),
                        CS42L51_TONE_CTL_BASS(8 - value / 15));
}
#endif

#ifdef AUDIOHW_HAVE_TREBLE
void audiohw_set_treble(int value)
{
    treble = value;
    handle_dsp_power();
    if (value >= -105 && value <= 120)
        cscodec_setbits(CS42L51_TONE_CTL, CS42L51_TONE_CTL_TREB(15),
                        CS42L51_TONE_CTL_TREB(8 - value / 15));
}
#endif

#ifdef AUDIOHW_HAVE_BASS_CUTOFF
void audiohw_set_bass_cutoff(int value)
{
    cscodec_setbits(CS42L51_BEEP_CONF, CS42L51_BEEP_CONF_BASS_CF(3),
                    CS42L51_BEEP_CONF_BASS_CF(value - 1));
}
#endif

#ifdef AUDIOHW_HAVE_TREBLE_CUTOFF
void audiohw_set_treble_cutoff(int value)
{
    cscodec_setbits(CS42L51_BEEP_CONF, CS42L51_BEEP_CONF_TREB_CF(3),
                    CS42L51_BEEP_CONF_TREB_CF(value - 1));
}
#endif

#ifdef AUDIOHW_HAVE_PRESCALER
void audiohw_set_prescaler(int value)
{
    /* Hardware supports -51.5dB to +12dB in 0.5dB steps. Values are
     * inverted because the argument is a gain decrease. */
    if (value > 120) value = -120;
    if (value < -515) value = 515;
    int vol = CS42L51_MIX_VOLUME(-value / 5);

    /* Set PCM volume for playback */
    cscodec_write(CS42L51_PCMA_VOL, vol);
    cscodec_write(CS42L51_PCMB_VOL, vol);
    /* Set ADC volume for radio. Preserve mute because it is
     * set via audiohw_set_monitor(). */
    cscodec_setbits(CS42L51_ADCA_VOL, CS42L51_MIX_VOLUME(0x7F), vol);
    cscodec_setbits(CS42L51_ADCA_VOL, CS42L51_MIX_VOLUME(0x7F), vol);
}
#endif

void audiohw_set_monitor(bool x)
{
    /* Monitoring is done via the ADC mixer in the signal processing engine.
     * It allows for tone controls and mixing with PCM output for voice.
     * The volume in these registers is used as a prescaler and preserved. */
    unsigned char val = x ? 0 : CS42L51_MIX_MUTE_ADCMIX;
    cscodec_setbits(CS42L51_ADCA_VOL, CS42L51_MIX_MUTE_ADCMIX, val);
    cscodec_setbits(CS42L51_ADCB_VOL, CS42L51_MIX_MUTE_ADCMIX, val);
}

void cscodec_select_ain(int ain, int gain)
{
    cscodec_freeze(true);

    /* Datasheet recommends this for powering down channel */
    cscodec_setbits(CS42L51_POWER_CTL1, 0, CS42L51_POWER_CTL1_PDN);
    cscodec_setbits(CS42L51_POWER_CTL1, AINPOWER_BITS, ain ? 0 : AINPOWER_BITS);
    cscodec_setbits(CS42L51_POWER_CTL1, CS42L51_POWER_CTL1_PDN, 0);

    /* OF sets PGA to 18 for stereo and 24 for mono, but powers it down.
     * It uses 6, meaning +3dB in ADCx_VOL. Here, ADCx_VOL is the prescaler
     * and the PGA is used for +3dB gain. */
    cscodec_write(CS42L51_ALC_PGA_CTL, 6);
    cscodec_write(CS42L51_ALC_PGB_CTL, 6);

    /* Input AIN1 to ADC */
    cscodec_write(CS42L51_ADC_INPUT,0);

    cscodec_freeze(false);
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
