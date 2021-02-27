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

#ifndef analyze_fft_h_
#define analyze_fft_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "fft_windows.h"


#if AUDIO_BLOCK_SAMPLES != 128
#error "Code assumes 2^7 samples per block"
#endif

class AudioAnalyzeFFT : public AudioStream
{
public:
  AudioAnalyzeFFT (void) : AudioStream (1, inputQueueArray)
  {
    valid = false ;
    full_scale = 1.0 ;
  }
  
  int Npoints (unsigned int N_points) ;
  
  bool available(void) ;
  float read (unsigned int binNumber) ;
  float dB (unsigned int binNumber) ;
  float readNoise (unsigned int binNumber) ;
  float dBNoise (unsigned int binNumber) ;
  float read (unsigned int binFirst, unsigned int binLast) ;
  void fftWindow (FFTWindow * window_desc) ;
  void overlapBlocks (unsigned int blocks) ;
  void fullScaleVolts (float amplitude) ;

  virtual void update(void);
  uint32_t * output ;

protected:
  void copy_to_fft_buffer (int blk, const int16_t * src) ;
  void zero_fft_buffer (int blk) ;
  void apply_window_to_fft_buffer (void) ;
  
private:
  void init(void);
  unsigned int N;
  int16_t * window ;
  audio_block_t ** blocklist ;
  int32_t * r_buffer ;
  int32_t * c_buffer ;
  volatile uint8_t state;
  volatile bool outputflag;
  audio_block_t * inputQueueArray [1];
  arm_rfft_instance_q31 fft_inst;
  uint8_t total_blocks ;
  uint8_t overlap_blocks ;
  volatile bool valid ;
  volatile float processing_gain ;
  volatile float noise_gain ;
  volatile float full_scale ;
};

#endif
