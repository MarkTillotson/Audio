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

#ifndef filter_oaconvolve_h_
#define filter_oaconvolve_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"


class AudioFilterOAConvolve : public AudioStream
{
public:
  AudioFilterOAConvolve(void) : AudioStream(1, inputQueueArray)
  {
    state = -1 ; iptr = -1 ; optr = -1 ; // not active
    arm_rfft_init_q31 (&sample_fft_instance,  8 * AUDIO_BLOCK_SAMPLES, 0, 1) ;
    arm_rfft_init_q31 (&sample_ifft_instance, 8 * AUDIO_BLOCK_SAMPLES, 1, 1) ;
  }

  virtual void update(void);

  // Set the filter from FIR coefficients
  void setFIRCoefficients (int size, float * coeffs);

private:
  q31_t filt_spect [2 * (8 * AUDIO_BLOCK_SAMPLES)] ; // filter spectrum
  q31_t in_arr [8 * AUDIO_BLOCK_SAMPLES] ;         // samples input
  q31_t spect_arr [2 * 8 * AUDIO_BLOCK_SAMPLES] ;  // working spectrum that gets multiplied by filter
  q31_t out_arr [8 * AUDIO_BLOCK_SAMPLES] ;        // samples output
  q31_t save_arr [2 * AUDIO_BLOCK_SAMPLES] ;       // saves for adding
  volatile int state, iptr, optr ;
  audio_block_t * inputQueueArray[1];
  arm_rfft_instance_q31 sample_fft_instance ;
  arm_rfft_instance_q31 sample_ifft_instance ;
};

#endif
