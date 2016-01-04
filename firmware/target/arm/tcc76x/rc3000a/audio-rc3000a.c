/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id: audio-nano2g.c 23095 2009-10-11 09:17:12Z dave $
 *
 * Copyright (C) 2006 by Michael Sevakis
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
#include "system.h"
#include "cpu.h"
#include "audio.h"
#include "sound.h"
#include "cscodec.h"

#if INPUT_SRC_CAPS != 0
void audio_set_output_source(int source)
{
    /* There's nothing to do here, because audio output always comes from
     * the DAC and all input is routed via the ADC. Only switching available
     * is monitor enabling and ADC input source switching. */
    (void)source;
} /* audio_set_output_source */

void audio_input_mux(int source, unsigned flags)
{
    (void)flags;
    /* Prevent pops from unneeded switching */
    static int last_source = AUDIO_SRC_PLAYBACK;

    switch (source)
    {
        default:                        /* playback - no recording */
            source = AUDIO_SRC_PLAYBACK;
        case AUDIO_SRC_PLAYBACK:
            if (source != last_source)
            {
                audiohw_set_monitor(false);
                cscodec_select_ain(0, 0);
#ifdef HAVE_RECORDING
                audiohw_disable_recording();
#endif
            }
        break;
#ifdef HAVE_LINE_REC
        case AUDIO_SRC_LINEIN:          /* recording only */
            if (source != last_source)
            {
                audiohw_set_monitor(false);
                audiohw_enable_recording(false); /* source line */
            }
        break;
#endif
        case AUDIO_SRC_FMRADIO:         /* recording and playback */
            if (source != last_source)
            {
                /* OF sets PGA to 18 for stereo and 24 for mono, but powers
                 * it down. It uses 6, meaning +3dB in ADCx_VOL. Here, ADCx_VOL
                 * is the prescaler and the PGA is used for +3dB gain. */
                cscodec_select_ain(1, 30);
                audiohw_set_monitor(true);
            }
    } /* end switch */

    last_source = source;
} /* audio_input_mux */
#endif /* INPUT_SRC_CAPS != 0 */
