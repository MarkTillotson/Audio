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

#define OFDM_FFTN     1024
#define OFDM_FFTMSK   0x3FF
#define GOLBITS_IN  12
#define GOLBITS_OUT 23
#define CHUNK_SIZE  16
#define OFDM_BYTES_PER_BLOCK (GOLBITS_IN * CHUNK_SIZE / 8)

// two channels for pilot tones, and one channel per pair of data bits
#define CHANNELS_PER_CHUNK (CHUNK_SIZE/2 + 2)

#define OFDM_CHANNELS      (CHANNELS_PER_CHUNK * GOLBITS_OUT)
#define OFDM_CHANNEL_MIN   8
#define OFDM_CHANNEL_MAX   (OFDM_CHANNEL_MIN + OFDM_CHANNELS - 1)
//#define OFDM_BW       11050.0
//#define OFDM_CHANNELS (int (OFDM_FFTN * OFDM_BW / AUDIO_SAMPLE_RATE_EXACT))

static void ofdm_null_listener (byte * vec)
{}

class AudioAnalyzeOFDM : public AudioStream
{
 public:
  AudioAnalyzeOFDM(void) : AudioStream (1, inputQueueArray)
  {
    shifted = 0 ;
    index = 0 ;
    part_index = 0 ;
    listener = ofdm_null_listener ;
    arm_cfft_radix4_init_f32 (&cfft_inst, OFDM_FFTN, 0, 1) ;  // initialize for forward FFT with bit reverse
    first_block = true ;
    for (unsigned int i = 0 ; i <= OFDM_CHANNEL_MAX / 64 ; i++)
    {
      pilot_pdiff [i] = 0 ;
      pilot_pdiffavg [i] = 0 ;
    }
  }

  void set_listener (void (*_listener) (byte *))
  {
    listener = _listener ;
  }

  float get_pilot_phasediff (int n)
  {
    return pilot_pdiffavg[n] ;
  }

  int get_squonk (void) { return total_count ; }
  int get_shifted (void) { return shifted ; }

  virtual void update (void);
  
 private:
  void demodulate (void);
  
  audio_block_t * inputQueueArray[1];
  unsigned int index ;
  unsigned int part_index ;
  struct ofdm_complex samples [OFDM_FFTN] ;
  byte data_vector [(OFDM_CHANNEL_MAX+1)/4] ;  // need to change for golay demod
  float phases [OFDM_CHANNEL_MAX+1] ;   // can pack this better but its small compared to the complex vector
  void (*listener) (byte *) ;
  arm_cfft_radix4_instance_f32 cfft_inst;
  bool first_block ;
  uint32_t sq_sums [OFDM_CHANNEL_MAX+1] ;
  float pilot_pdiff [OFDM_CHANNEL_MAX / 64 + 1] ;
  float pilot_pdiffavg [OFDM_CHANNEL_MAX / 64 + 1] ;
  int total_count ;
  struct ofdm_complex qam_maps [OFDM_FFTN/2] ;
  unsigned int shifted ;
};

#endif

