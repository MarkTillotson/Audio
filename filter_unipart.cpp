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

#include "filter_unipart.h"
#include "utility/dspinst.h"

#if defined(__ARM_ARCH_7EM__)

void AudioFilterUnipart::update (void)
{
  if (state == -1)  // not active
  {
    audio_block_t * block = receiveWritable();
    if (block != NULL)
      release (block) ;
    return ;
  }

  audio_block_t * block = receiveReadOnly();
  audio_block_t * out_blk = NULL ;

  if (prev_blk == NULL)
  {
    arm_fill_f32 (0.0, in_array, AUDIO_BLOCK_SAMPLES) ;
    out_blk = allocate () ;
  }
  else
  {
    arm_q15_to_float (prev_blk->data, in_array, AUDIO_BLOCK_SAMPLES) ;
    out_blk = prev_blk ;
  }
  
  if (block == NULL)
  {
    arm_fill_f32 (0.0, in_array + AUDIO_BLOCK_SAMPLES, AUDIO_BLOCK_SAMPLES) ;
    prev_blk = NULL ;
  }
  else
  {
    arm_q15_to_float (block->data, in_array + AUDIO_BLOCK_SAMPLES, AUDIO_BLOCK_SAMPLES) ;
    prev_blk = block ;
  }

  arm_rfft_fast_f32 (&fft_instance, in_array, delay_line + state * 2*AUDIO_BLOCK_SAMPLES, 0) ;

  for (int i = 0 ; i < Npart ; i++)
  {
    int del_index = (state + Npart - i) % Npart ;
    arm_cmplx_mult_cmplx_f32 (filt_spectra + i * 2*AUDIO_BLOCK_SAMPLES,  // needs to mult accumulate
			      delay_line + del_index * 2*AUDIO_BLOCK_SAMPLES,
			      out_temp,
			      AUDIO_BLOCK_SAMPLES) ;
    arm_add_f32 (out_temp, out_spectra, out_spectra, 2*AUDIO_BLOCK_SAMPLES) ;
    
  }
  state = (state + 1) % Npart ;

  arm_rfft_fast_f32 (&fft_instance, out_spectra, out_array, 1) ;  // inverse fft

  if (out_blk != NULL)
  {
    arm_float_to_q15 (out_array + AUDIO_BLOCK_SAMPLES, out_blk->data, AUDIO_BLOCK_SAMPLES) ;
    transmit (out_blk) ;
    release (out_blk) ;
  }
  
}


void AudioFilterUnipart::setFIRCoefficients (int size, float * coeffs)
{
  Npart = (size + AUDIO_BLOCK_SAMPLES-1) / AUDIO_BLOCK_SAMPLES ;

  filt_spectra = new float [(Npart+1) * 2*AUDIO_BLOCK_SAMPLES] ;
  if (filt_spectra == NULL)
    return ;
  delay_line   = new float [(Npart+2) * 2*AUDIO_BLOCK_SAMPLES] ;
  if (delay_line == NULL)
  {
    delete[] filt_spectra ;
    return ;
  }
  
  int index = 0 ;
  for (int i = 0 ; i < Npart ; i++)
  {
    arm_copy_f32 (coeffs + index, in_array, AUDIO_BLOCK_SAMPLES) ;
    arm_fill_f32 (0.0, in_array + AUDIO_BLOCK_SAMPLES, AUDIO_BLOCK_SAMPLES) ;
    arm_rfft_fast_f32 (&fft_instance, in_array, filt_spectra + 2*index, 0) ;
    index += AUDIO_BLOCK_SAMPLES ;
  }
  arm_fill_f32 (0.0, delay_line, (Npart+1) * 2*AUDIO_BLOCK_SAMPLES) ;

  state = 0 ;
}

#elif defined(KINETISL)

void AudioFilterUnipart::update (void)
{
  audio_block_t * block = receiveReadOnly();
  if (block != NULL)
    release(block);
}

#endif
