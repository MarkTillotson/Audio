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
#include "analyze_fft1024_hires.h"
#include "sqrt_integer.h"
#include "utility/dspinst.h"


// 140312 - PAH - slightly faster copy
static void copy_to_fft_buffer(void *destination, const void *source)
{
	const uint16_t *src = (const uint16_t *)source;
	uint32_t *dst = (uint32_t *)destination;

	for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++)
	  *dst++ = (*src++) << 16 ;
}

static void zero_fft_buffer(int32_t * dest)
{
  for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++)
    *dest++ = 0 ;
}

static void apply_window_to_fft_buffer(void * r_buffer, const void *window)
{
  int32_t *buf = (int32_t *) r_buffer;
  const int16_t *win = (int16_t *) window;

  for (int i=0; i < 1024; i++)
  {
    int16_t coeff = win[i] ;
    int64_t val = buf[i] ;
    val = (coeff * val + 0x4000) >> 15 ;
    buf[i] = val ;
  }
}

void AudioAnalyzeFFT1024HiRes::update(void)
{
	audio_block_t *block;

	block = receiveReadOnly();
	if (!block) return;

#if defined(__ARM_ARCH_7EM__)
	copy_to_fft_buffer(r_buffer+(state << 7), block->data);
	if (state < total_blocks - overlap_blocks)
	  release (block) ;
	else
	  blocklist[state] = block;

	state += 1;
	if (state == total_blocks)
	{
		if (window)
		  apply_window_to_fft_buffer (r_buffer, window);

		arm_rfft_q31 (&fft_inst, r_buffer, c_buffer);

		c_buffer[1] = 0 ;  // zero out the Fs/2 entry as its stuck in with the zero bin


		// TODO: support averaging multiple copies
		for (int i=0; i < 512; i++)
		{
		  float real = c_buffer [2*i] ;
		  float imag = c_buffer [2*i+1] ;
		  float mag = sqrt (real*real + imag*imag) ;
		  output[i] = int (round (mag)) ;
		}
		outputflag = true;
		
		// setup for overlapping with previous:
		for (int i = 0 ; i < overlap_blocks ; i++)
		{
		  int ind = i + total_blocks - overlap_blocks ;
		  if (blocklist[ind] == NULL)
		    zero_fft_buffer (r_buffer + (i << 7)) ;
		  else
		    copy_to_fft_buffer (r_buffer + (i << 7), blocklist[ind]->data);
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
