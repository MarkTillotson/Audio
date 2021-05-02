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

#ifndef analyze_ofdm_h_
#define analyze_ofdm_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"


struct ofdm_complex
{
  float real ;
  float imag ;
} ;

#define OFDM_BW       11050.0
#define OFDM_CHANNELS (int (1024 * OFDM_BW / AUDIO_SAMPLE_RATE_EXACT))

static void ofdm_null_listener (byte * vec)
{}

class AudioAnalyzeOFDM : public AudioStream
{
 public:
  AudioAnalyzeOFDM(void) : AudioStream (1, inputQueueArray)
  {
    segment = 0 ;
    listener = ofdm_null_listener ;
    arm_cfft_radix4_init_f32 (&cfft_inst, 1024, 0, 1) ;  // initialize for forward FFT with bit reverse
    first_block = true ;
  }

  void set_listener (void (*_listener) (byte *))
  {
    listener = _listener ;
  }

  virtual void update (void);
  
 private:
  void demodulate (void);
  
  audio_block_t * inputQueueArray[1];
  unsigned int segment ;
  struct ofdm_complex samples [1024] ;
  byte data_vector [(OFDM_CHANNELS+3)/4] ;
  float phases [OFDM_CHANNELS] ;   // can pack this better but its small compared to the complex vector
  void (*listener) (byte *) ;
  arm_cfft_radix4_instance_f32 cfft_inst;
  bool first_block ;
};

#endif

