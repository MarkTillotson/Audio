#include <dspinst.h>
#include <math.h>
#include "synth_coupled_resonator.h"

/* Coupled-form resonator sinusoid generate
 *
 * Copyright (c) 2020, Mark Tillotson
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/*
 * Coupled-form resonator using the iteration
 *  x -= e * y ;
 *  y += e * x ;
 *  where e = 2 * sin (w/2),  w = 2*pi * f / Fs  (f = frequency, Fs = sample rate)
 *
 *  https://ccrma.stanford.edu/~jos/pdf/GordonAndSmith86.pdf
 *
 *  if x is defined to have phase 0,
 *  y has phase -(pi/2 - w/2)
 *
 *  A quadrature pair can be obtained thus:
 *  y2 =  (y - x * sin(w/2)) / cos(w/2)
 *  (x, y2) are in quadrature so can measure amplitude and phase from them, useful for phase-continuous frequency changes
 */

#define AMP 0x10000000   // amplitude of x, y 
#define ONE 0x80000000   // amplitude of e/2


void AudioSynthCoupledSine::frequency (float f)
{
  if (f < 0)     f = 0 ;
  if (f > 20000) f = 20000 ;

  float new_hw = M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  int32_t new_e = int (round (ONE * sin (new_hw))) ;  // should be 2 * sin (new_hw), but scaled in the update loop

  __disable_irq() ;
  // make phase-continuous (assuming amplitude unchanged)
  float y2 = (y - x * sin(halfomega)) / cos(halfomega) ;  // x,y2 are in quadrature, derive y2 from x,y here
  float phase = atan2 (y2, x) ;
  
  halfomega = new_hw ;
  e = new_e ;
  set_phase (phase) ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::amplitude (float amp)
{
  if (amp < 0.0) amp = 0.0 ;
  if (amp > 0.97) amp = 0.97 ;

  float scale = AMP * amp ;

  __disable_irq() ;
  if (amplitud == 0)
  {
    x = 0 ;
    y = int (round (scale * cos (halfomega))) ;
  }
  else
  {
    x = int (round (x * amp / amplitud)) ;
    y = int (round (y * amp / amplitud)) ;
    // TODO normalize to prevent drift on many successive calls
  }
  amplitud = amp ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  __disable_irq() ;
  set_phase (phase) ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::set_phase (float phase)
{
  float scale = AMP * amplitud ;
  x = int (round (scale * cos (phase))) ;
  y = int (round (scale * sin (phase + halfomega))) ;
}

void AudioSynthCoupledSine::update ()
{
  audio_block_t * block = allocate() ;
  if (block == NULL)
    return ;

  int16_t * data = block->data ;

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    *data++ = (int16_t) ((x+0x1000) >> 13) ;
    x -= multiply_32x32_rshift32 (e, y) << 2 ;  // shift is 2 as we represent 2 sin(w/2) as sin(w/2).
    y += multiply_32x32_rshift32 (e, x) << 2 ;  // shift would be 1 for FIX1.31 airthmetic otherwise
  }

  transmit (block) ;
  release (block) ;
}
