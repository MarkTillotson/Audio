#include <dspinst.h>
#include <math.h>
#include "synth_quadrature.h"

/* Quadrature generator
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
 * Complex resonator using the iteration
 *  x -= k1 * y ;
 *  y += k2 * x ;
 *  x -= k1 * y ;
 *  where k1 = tan(w/2), k2 = sin(w),  w = 2*pi * f / Fs  (f = frequency, Fs = sample rate)
 *
 *  https://vicanek.de/articles/QuadOsc.pdf
 *
 *  (x, y) are in quadrature
 */


#define AMP 0x1000000   // amplitude of x, y 
#define ONE 0x10000000   // amplitude of k1, k2


void AudioSynthQuadrature::frequency (float f)
{
  if (f < 0)     f = 0 ;
  if (f > 20000) f = 20000 ;

  float new_hw = M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  int32_t new_k1 = int (round (ONE * tan (new_hw))) ;
  int32_t new_k2 = int (round (ONE * sin (2*new_hw))) ;

  __disable_irq() ;
  // automatically phase-continuous
  k1 = new_k1 ;
  k2 = new_k2 ;
  d  = multiply_32x32_rshift32 (k1, y<<4) ;
  __enable_irq() ;
}

void AudioSynthQuadrature::agc (void)
{
  if (amplitud == 0.0)
    return ;
  int64_t xx = x ;
  int64_t yy = y ;
  int64_t en = xx*xx + yy*yy ;
  int64_t target = fabs (amplitud) * AMP ;
  int32_t amp_error = (en - target*target) / (2 * target) ;

  x -= (xx * amp_error / AMP) ;
  y -= (yy * amp_error / AMP) ;
  d  = multiply_32x32_rshift32 (k1, y<<4) ;
}
  

void AudioSynthQuadrature::amplitude (float amp)
{
  if (amp < -0.9999)    amp = -0.9999 ;
  if (amp > 0.9999) amp = 0.9999 ;

  float scale = AMP * amp ;

  __disable_irq() ;
  if (amplitud == 0)
  {
    x = int (round (scale)) ;
    y = 0 ;
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

void AudioSynthQuadrature::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  __disable_irq() ;
  set_phase (phase) ;
  __enable_irq() ;
}

void AudioSynthQuadrature::set_phase (float phase)
{
  float scale = AMP * amplitud ;
  x = int (round (scale * cos (phase))) ;
  y = int (round (scale * sin (phase))) ;
}

void AudioSynthQuadrature::update ()
{
  audio_block_t * xblock = allocate() ;
  if (xblock == NULL)
    return ;
  audio_block_t * yblock = allocate() ;
  if (yblock == NULL)
  {
    release (xblock) ;
    return ;
  }

  int16_t * xdata = xblock->data ;
  int16_t * ydata = yblock->data ;
  agc() ;  // keep amplitude from drifting

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    *xdata++ = (int16_t) ((x+0x100) >> 9) ;
    *ydata++ = (int16_t) ((y+0x100) >> 9) ;
    x -= d ;
    y += multiply_32x32_rshift32 (k2, x<<4) ;
    d  = multiply_32x32_rshift32 (k1, y<<4) ;
    x -= d ;
  }

  transmit (xblock, 0) ;
  transmit (yblock, 1) ;
  release (xblock) ;
  release (yblock) ;
}
