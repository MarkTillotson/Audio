/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
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

#include <Arduino.h>
#include "synth_ofdm.h"

#define SQRTH 0.70710678
static int   graycode   [4]  = { 0, 1, 3, 2 } ;
// cosines of the 8th roots of unity, plus first two repeated
static float phasetable [10] = { 1.0, SQRTH, 0.0, -SQRTH, -1.0, -SQRTH, 0.0, SQRTH, 1.0, SQRTH } ;


void AudioAnalyzeOFDM::demodulate (void)
{
  // FFT
  arm_cfft_radix4_f32 (&cfft_inst, (float *) samples);
  
  int vecind = 0 ;
  int vecshft = 0 ;

  if (first_block)
  {
    for (int i = 0 ; i < OFDM_CHANNELS ; i++)
    {
      float real = samples [i+1].real ;
      float imag = samples [i+1].imag ;
      float phase = atan2 (imag, real) ;
      phases [i] = phase ;
    }
    first_block = false ;
  }

  for (int i = 0 ; i < OFDM_CHANNELS ; i++)
  {
    float real = samples [i+1].real ;
    float imag = samples [i+1].imag ;
    float phase = atan2 (imag, real) ;
    float phase_diff = phase - phases [i] ;
    phase_diff = - phase_diff ;
    phases [i] = phase ;
    while (phase_diff < -M_PI) phase_diff += 2*M_PI ;
    while (phase_diff >= M_PI) phase_diff -= 2*M_PI ;
    int bits ;
    if (phase_diff >= 0)
      bits = phase_diff > M_PI/2 ? 2 : 3 ;
    else
      bits = phase_diff < -M_PI/2 ? 0 : 1 ;

    if (vecshft == 0)
      data_vector[vecind] = bits ;
    else
      data_vector[vecind] |= bits << vecshft ;

    // step to next pair of databits for next time
    vecshft = (vecshft + 2) & 6 ;
    if (vecshft == 0)
      vecind ++ ;
  }
}


void AudioAnalyzeOFDM::update(void)
{
  audio_block_t * block = receiveReadOnly (0) ;
  if (block == NULL)
    return;

  int16_t * p = block->data ;
  int16_t * end = p + AUDIO_BLOCK_SAMPLES;
  unsigned int index = AUDIO_BLOCK_SAMPLES * ((segment + 7) & 7) ;  // index into the ifft output

  switch (segment)
  {
  case 0: // fade in period
    segment ++ ;
    break ;

  case 9: // fade out period
    demodulate () ;
    listener (data_vector) ;  // in time for next set of samples
    segment = 0 ;
    break ;

  default:  // body of block
    do
    {
      samples[index].real = (*p++) / 32767.0 ;
      samples[index].imag = 0.0 ;
      index ++ ;
    } while (p < end) ;
    segment ++ ;
    break ;
  }
  
  release (block) ;
}
