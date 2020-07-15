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
#include "analyze_fft.h"
#include "sqrt_integer.h"
#include "utility/dspinst.h"


void AudioAnalyzeFFT::copy_to_fft_buffer (int blk, const int16_t * src)
{
  int32_t * dest = &r_buffer[blk << 7] ;
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    *dest++ = (*src++) << 16 ;
}

void AudioAnalyzeFFT::zero_fft_buffer (int blk)
{
  int32_t * dest = &r_buffer[blk << 7] ;
  for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++)
    *dest++ = 0 ;
}

void AudioAnalyzeFFT::apply_window_to_fft_buffer (void)
{
  for (unsigned int i=0; i < N/2; i++)
  {
    int64_t product = r_buffer[i] ;
    product *= window [i] ;
    int32_t val = (int32_t) ((product + 0x4000) >> 16) ;
    r_buffer[i] = val ;
    r_buffer[N-i-1] = val ;
  }
}

void AudioAnalyzeFFT::update(void)
{
  audio_block_t * block = receiveReadOnly();
  if (block == NULL)
    return;
  if (!valid)
  {
    release (block) ;
    return ;
  }

#if defined(__ARM_ARCH_7EM__)
  copy_to_fft_buffer (state, block->data) ;
  if (state < total_blocks - overlap_blocks)
    release (block) ;
  else
    blocklist [state] = block ;
  state ++ ;
  if (state >= total_blocks)
  {
    apply_window_to_fft_buffer ();

    arm_rfft_q31 (&fft_inst, r_buffer, c_buffer);

    for (unsigned int i=0; i <= N/2; i++)
    {
      float re = c_buffer [2*i] ;
      float im = c_buffer [2*i+1] ;
      float mag = sqrt (re * re + im * im) ;
      output[i] = int (round (mag)) ;
    }
    outputflag = true;

    // setup for overlapping with previous:
    for (int i = 0 ; i < overlap_blocks ; i++)
    {
      int ind = i + total_blocks - overlap_blocks ;
      if (blocklist[ind] == NULL)
	zero_fft_buffer (i) ;
      else
	copy_to_fft_buffer (i, blocklist[ind]->data);
      blocklist[i] = blocklist[ind];
    }
    for (int i = 0 ; i < total_blocks - overlap_blocks ; i++)
      if (blocklist [i] != NULL)
	release (blocklist [i]) ;
		
    state = overlap_blocks;
  }
#else
  release(block);
#endif
}


