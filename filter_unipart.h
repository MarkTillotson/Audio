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

#ifndef filter_unipart_h_
#define filter_unipart_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"


class AudioFilterUnipart : public AudioStream
{
public:
  AudioFilterUnipart(void) : AudioStream(1, inputQueueArray)
  {
    arm_rfft_fast_init_f32 (&fft_instance, 2 * AUDIO_BLOCK_SAMPLES) ;
    prev_blk = NULL ;
    state = -1 ; // not active
  }

  virtual void update (void);

  // Set the filter from FIR coefficients
  void setFIRCoefficients (int size, float * coeffs);
  
private:
  audio_block_t * inputQueueArray[1];
  arm_rfft_fast_instance_f32 fft_instance ;
  float in_array    [2 * AUDIO_BLOCK_SAMPLES] ;    // input samples
  float out_temp    [2 * 2*AUDIO_BLOCK_SAMPLES] ;  // complex destination for multiplies
  float out_spectra [2 * 2*AUDIO_BLOCK_SAMPLES] ;  // accumulator for spectra
  float out_array   [2 * AUDIO_BLOCK_SAMPLES] ;    // output samples
  float * filt_spectra ;                           // filter spectra
  float * delay_line ;                             // successive FFTs
  audio_block_t * prev_blk ;
  volatile int state, Npart ;
} ;

#endif
