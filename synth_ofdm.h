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

/*
#define OFDM_FFTN    1024
#define OFDM_FFTMSK  0x3FF
#define GOLBITS_IN  12
#define GOLBITS_OUT 23
#define CHUNK_SIZE  16
#define OFDM_BYTES_PER_BLOCK (GOLBITS_IN * CHUNK_SIZE / 8)

// two channels for pilot tones, and one channel per pair of data bits
#define CHANNELS_PER_CHUNK (CHUNK_SIZE/2 + 2)

#define OFDM_CHANNELS      (CHANNELS_PER_CHUNK * GOLBITS_OUT)
#define OFDM_CHANNEL_MIN   8
#define OFDM_CHANNEL_MAX   (OFDM_CHANNEL_MIN + OFDM_CHANNELS - 1)
*/
//#define OFDM_BW       11050.0
//#define OFDM_CHANNELS (int (OFDM_FFTN * OFDM_BW / AUDIO_SAMPLE_RATE_EXACT))


static void ofdm_random_source (byte * vec)
{
  // NEED to change size of vec, split into golay blocks of 12 bits x CHUNK_SIZE
  //for (unsigned int i = 0 ; i < (OFDM_CHANNELS+3)/4 ; i++)
  for (unsigned int i = 0 ; i < OFDM_BYTES_PER_BLOCK ; i++)
    vec[i] = random (0x100) ;
}

class AudioSynthOFDM : public AudioStream
{
 public:
  AudioSynthOFDM() : AudioStream(0, NULL)
  {
    segment = 0 ;
    datasource = ofdm_random_source ;
    arm_cfft_radix4_init_f32 (&cfft_inst, OFDM_FFTN, 1, 1) ;  // initialize for inverse FFT with bitrev
    // taper function table
    /*
    for (unsigned int i = 0 ; i <= AUDIO_BLOCK_SAMPLES ; i++)
    {
      raised_cosine [i] = 0.5 * (1 - cos (M_PI * i / AUDIO_BLOCK_SAMPLES)) ;
    }
    */
    // initialize phase codes to random initial state (data is phase differences between blocks)
    for (unsigned int i = 0 ; i < OFDM_CHANNELS ; i++)
    {
      phasecodes [i] = (i + random (8)) & 7 ;
    }
    debug = 1.0 ;
  }

  virtual void update(void);

  void set_datasource (void (*fn) (byte *))
  {
    if (fn != NULL)
      datasource = fn ;
  }

 private:
  void get_samples (void);
  void set_channel (uint32_t chan, float real, float imag) ;
  
  unsigned int segment ;
  //float raised_cosine [AUDIO_BLOCK_SAMPLES+1] ;
  struct ofdm_complex samples [OFDM_FFTN] ;
  // NEED to change size of vec, split into golay blocks of 12 bits x CHUNK_SIZE
  byte data_vector [OFDM_BYTES_PER_BLOCK] ;
  byte phasecodes [OFDM_CHANNEL_MAX+1] ;   // can pack this better but its small compared to the complex vector
  void (*datasource) (byte *) ;
  arm_cfft_radix4_instance_f32 cfft_inst;

  float debug = 1.0 ;
};

#endif
