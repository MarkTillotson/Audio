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

#ifndef synth_ofdm_h_
#define synth_ofdm_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "analyze_ofdm.h"
#include "arm_math.h"

#define OFDM_TEST_MODE 0


#define OFDM_BW       11050.0
#define OFDM_CHANNELS (int (1024 * OFDM_BW / AUDIO_SAMPLE_RATE_EXACT))

static void ofdm_random_source (byte * vec)
{
  for (int i = 0 ; i < (OFDM_CHANNELS+3)/4 ; i++)
    vec[i] = random (0x100) ;
}

static void ofdm_test_source (byte * vec)
{
  vec [0] = 0x55 ;
}

class AudioSynthOFDM : public AudioStream
{
 public:
  AudioSynthOFDM() : AudioStream(0, NULL)
  {
    segment = 0 ;
#if OFDM_TEST_MODE
    datasource = ofdm_test_source ;
#else
    datasource = ofdm_random_source ;
#endif
    arm_cfft_radix4_init_f32 (&cfft_inst, 1024, 1, 1) ;  // initialize for inverse FFT with bitrev
    // taper function table
    for (int i = 0 ; i <= AUDIO_BLOCK_SAMPLES ; i++)
    {
      raised_cosine [i] = 0.5 * (1 - cos (M_PI * i / AUDIO_BLOCK_SAMPLES)) ;
    }
    // initialize phase codes to random initial state (data is phase differences between blocks)
    for (int i = 0 ; i < OFDM_CHANNELS ; i++)
    {
      phasecodes [i] = random (8) ;
    }
  }

  virtual void update(void);

  void set_datasource (void (*fn) (byte *))
  {
    if (fn != NULL)
      datasource = fn ;
  }

 private:
  void get_samples (void);
  
  unsigned int segment ;
  float raised_cosine [AUDIO_BLOCK_SAMPLES+1] ;
  struct ofdm_complex samples [1024] ;
  byte data_vector [(OFDM_CHANNELS+3)/4] ;
  byte phasecodes [OFDM_CHANNELS] ;   // can pack this better but its small compared to the complex vector
  void (*datasource) (byte *) ;
  arm_cfft_radix4_instance_f32 cfft_inst;
};

#endif
