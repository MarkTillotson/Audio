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
#include "filter_oaconvolve.h"
#include "utility/dspinst.h"

#if defined(__ARM_ARCH_7EM__)

/*
static void arm_q31_to_q16 (q31_t * src, int16_t * dst, int count)  // assume multiple of 4
{
  while (count > 0)
  {
    *dst++ = (int16_t) ((*src++ + 0x4000) >> 15) ;   // convert q31 to q16 unbiased
    *dst++ = (int16_t) ((*src++ + 0x4000) >> 15) ;
    *dst++ = (int16_t) ((*src++ + 0x4000) >> 15) ;
    *dst++ = (int16_t) ((*src++ + 0x4000) >> 15) ;
    count -= 4 ;
  }
}

static void arm_q16_to_q31 (int16_t * src, q31_t * dst, int count)  // assume multiple of 4
{
  while (count > 0)
  {
    *dst++ = *src++ << 15 ; // convert q16 to q31
    *dst++ = *src++ << 15 ;
    *dst++ = *src++ << 15 ;
    *dst++ = *src++ << 15 ;
    count -= 4 ;
  }
}
*/

void AudioFilterOAConvolve::update (void)
{
  if (state == -1)  // not active
  {
    audio_block_t * block = receiveReadOnly();
    if (block != NULL)
      release (block) ;
    return ;
  }

  // if active:
  //Serial.print ('.') ;
  
  // input block handling
  audio_block_t * block = receiveWritable();  // reuse it for the output
  q31_t * q = &in_arr [iptr * AUDIO_BLOCK_SAMPLES] ;
  if (block == NULL)  // treat missing input as equivalent to all zeroes
  {
    block = allocate () ;
    arm_fill_q31 (0, q, AUDIO_BLOCK_SAMPLES) ;
  }
  else
  {
    int16_t * p = block->data ;
    arm_q15_to_q31 (p, q, AUDIO_BLOCK_SAMPLES) ;
  }
  iptr = (iptr + 1) % 6 ;
  // input handling done
  

  // output block handling
  if (block != NULL)   // check this just in case allocate failed
  {
    q31_t * p = &out_arr [optr * AUDIO_BLOCK_SAMPLES] ;
    int16_t * q = block->data ;
    if (optr < 2)   // if the first two segments, need to add the overlap
    {
      arm_add_q31 (p, save_arr, p, AUDIO_BLOCK_SAMPLES) ;
    }
    arm_q31_to_q15 (p, q, AUDIO_BLOCK_SAMPLES) ;  // working
    transmit (block, 0) ;
    release (block) ;
  }
  optr = (optr + 1) % 6 ;
  // output handling done
  

  // now we can do actual operations
  switch (state)
  {
  case 0:  // forward fft 
    arm_rfft_q31 (&sample_fft_instance, in_arr, spect_arr) ;
    break ;
  case 1:  // multiply spectra
    arm_cmplx_mult_cmplx_q31 (spect_arr, filt_spect, spect_arr, 8 * AUDIO_BLOCK_SAMPLES) ;
    arm_scale_q31 (spect_arr, 0x7fffffff, 3, spect_arr, 2*8 * AUDIO_BLOCK_SAMPLES) ; // compensate for convolution gain?
    break ;
  case 2:  // inverse fft
    arm_rfft_q31 (&sample_ifft_instance, spect_arr, out_arr) ;
    arm_scale_q31 (out_arr, 0x7fffffff, 10, out_arr, 8 * AUDIO_BLOCK_SAMPLES) ; // convert 10.22 to 1.31, factor of 2 as well?
    break ;
  case 3: case 4: // this is when adds are happening in the outputing.
    break ;
  case 5:  // do the save copy for next time round
    arm_copy_q31 (&out_arr [6 * AUDIO_BLOCK_SAMPLES], save_arr, 2 * AUDIO_BLOCK_SAMPLES) ;
    break ;
  }

  state = (state + 1) % 6 ;
}


void AudioFilterOAConvolve::setFIRCoefficients (int size, float * coeffs)
{
  if (size > 8 * AUDIO_BLOCK_SAMPLES)
    size = 8 * AUDIO_BLOCK_SAMPLES ;

  /*
  float sum = 0.0 ;
  for (int i = 0 ; i < size ; i++)
      sum += coeffs[i] ;
  Serial.print ("coeffs sum ") ; Serial.println (sum) ;
  */
  
  arm_fill_q31 (0, in_arr, 8 * AUDIO_BLOCK_SAMPLES) ;
  // place impulse response in in_arr
  arm_float_to_q31 (coeffs, in_arr, size) ;

  // calculate its spectrum
  arm_rfft_q31 (&sample_fft_instance, in_arr, filt_spect) ;
  arm_scale_q31 (filt_spect, 0x7fffffff, 9, filt_spect, 2*8*AUDIO_BLOCK_SAMPLES) ; // 10.22 format back to 1.31
  for (int i = 0 ; i < 20 ; i++)
  {
    Serial.print (filt_spect [2*i]) ;
    Serial.print (" + j ") ;
    Serial.println (filt_spect [2*i+1]) ;
  }

  // reset in_array to zeros
  arm_fill_q31 (0, in_arr, 8 * AUDIO_BLOCK_SAMPLES) ;
  arm_fill_q31 (0, &out_arr[3 * AUDIO_BLOCK_SAMPLES], 3 * AUDIO_BLOCK_SAMPLES) ;
  arm_fill_q31 (0, save_arr, 2 * AUDIO_BLOCK_SAMPLES) ;

  __disable_irq () ;
  iptr = 5 ;  // correct starting points
  optr = 3 ;
  state = 0 ;  // make active
  __enable_irq () ;
}


#elif defined(KINETISL)

void AudioFilterOAConvolve::update (void)
{
  audio_block_t * block = receiveReadOnly();
  if (block != NULL)
    release(block);
}

#endif
