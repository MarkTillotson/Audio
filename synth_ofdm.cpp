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
static byte graycode [4] = { 0, 1, 3, 2 } ;
// cosines of the 8th roots of unity, plus first two repeated
static float phasetable [10] = { 1.0, SQRTH, 0.0, -SQRTH, -1.0, -SQRTH, 0.0, SQRTH, 1.0, SQRTH } ;


void AudioSynthOFDM::get_samples (void)
{
  // setup complex freq entries
  samples[0].real = 0 ;
  samples[0].imag = 0 ;
  samples[512].real = 0 ;
  samples[512].imag = 0 ;
  int vecind = 0 ;
  int vecshft = 0 ;
#if OFDM_TEST_MODE
  float prescale = 40.0 ;
  for (unsigned int i = 0 ; i < 1 ; i++)
#else
  float prescale = 8.0 ;
  for (unsigned int i = 0 ; i < OFDM_CHANNELS ; i++)
#endif
  {
    // get next two data bits for this channel
    byte bits = graycode [(data_vector[vecind] >> vecshft) & 3] ;
    // step to next pair of databits for next time
    vecshft = (vecshft + 2) & 6 ;
    if (vecshft == 0)
      vecind ++ ;

    // delta the phasecode by +3, +1, -1, -3 depending on the gray coded bits - "pi/4 DQPSK"
    byte phasecode = (phasecodes[i] + 2*bits + 5) & 7 ;
    phasecodes [i] = phasecode ;
    
    float real = prescale * (phasetable [phasecode]) ;   // cos
    float imag = prescale * (phasetable [phasecode+2]) ; // sin
    
    if ((i & 62) == 62)
    {
      real = prescale ;
      imag = 0 ;
    }
    //else if (i < 80 || i > 150)
    //{
    //  real = imag = 0 ;
    //}

    samples [i+1].real = real ;       // positive freq entries
    samples [i+1].imag = imag ;
    samples [1024-(i+1)].real = real ;  // negative freq entries, conjugates.
    samples [1024-(i+1)].imag = -imag ;
  }
#if OFDM_TEST_MODE
  for (unsigned int i = 2 ; i < 512 ; i++)
#else
  for (unsigned int i = OFDM_CHANNELS+1 ; i < 512 ; i++)
#endif
  {
    samples [i].real = 0 ;
    samples [i].imag = 0 ;
    samples [1024-i].real = 0 ;
    samples [1024-i].imag = 0 ;
  }
  
  // perform inverse FFT to get sample data
  arm_cfft_radix4_f32 (&cfft_inst, (float *) samples);
}


void AudioSynthOFDM::update(void)
{
  audio_block_t * block = allocate() ;
  if (block == NULL)
    return;

  int16_t * p = block->data ;
  int16_t * end = p + AUDIO_BLOCK_SAMPLES;
  unsigned int index = AUDIO_BLOCK_SAMPLES * segment ;  // index into the ifft output

  if (segment == 0)
    get_samples () ;

  do
  {
    *p++ = int (round (32767 * samples[index++].real)) ;
  } while (p < end) ;

  if (segment == 7)
    datasource (data_vector) ;  // in time for next set of samples
  
  segment = (segment+1) & 7 ;

  transmit (block) ;
  release (block) ;
}
