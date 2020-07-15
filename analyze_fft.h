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
#include "arm_math.h"
#include "fft_windows.h"


#if AUDIO_BLOCK_SAMPLES != 128
#error "Code assumes 2^7 samples per block"
#endif

class AudioAnalyzeFFT : public AudioStream
{
public:
  AudioAnalyzeFFT (unsigned int N) : AudioStream (1, inputQueueArray)
  {
    state = 0 ;
    outputflag = false ;
    
    total_blocks = N / AUDIO_BLOCK_SAMPLES ;
    overlap_blocks = total_blocks >> 1 ;
    if (arm_rfft_init_q31 (&fft_inst, N, 0, 1) != ARM_MATH_SUCCESS)
    {
      Serial.println ("Bad size for AudioAnalyzeFFT") ;
      valid = false ;
      return ;
    }
    output = (uint16_t *) malloc ((N / 2 + 1) * sizeof (int16_t)) ;
    blocklist = (audio_block_t **) malloc (total_blocks * sizeof (audio_block_t*)) ;
    r_buffer = (int32_t *) malloc (N * sizeof (int16_t)) ;
    c_buffer = (int32_t *) malloc (2 * N * sizeof (int16_t)) ;
    window = (int16_t *) malloc (N/2 * sizeof (int16_t)) ;
    if (output == NULL || blocklist == NULL || r_buffer == NULL || c_buffer == NULL || window == NULL)
    {
      Serial.println ("malloc failed for AudioAnalyzeFFT") ;
      valid = false ;
      return ;
    }
    fftWindow (&hann_window) ;  // default window
    valid = true ;
  }
  
  bool available()
  {
    bool avail = outputflag ;
    outputflag = false ;
    return avail ;
  }

  float read (unsigned int binNumber)
  {
    if (binNumber > N/2)
      return 0.0;
    return (float) output[binNumber] / 0x40000000 ;
  }
  
  float read (unsigned int binFirst, unsigned int binLast)
  {
    if (binFirst > binLast)
    {
      unsigned int tmp = binLast;
      binLast = binFirst;
      binFirst = tmp;
    }
    if (binFirst > N/2)
      return 0.0 ;
    if (binLast > N/2)
      binLast = N/2 ;
    uint64_t sum = 0L ;
    do
    {
      sum += output [binFirst++] ;
    } while (binFirst <= binLast) ;
    
    return (float) sum / 0x40000000 ;
  }
  
  void fftWindow (FFTWindow * window_desc)
  {
    window_desc->expand_q15 (window, N) ;
  }

  void overlapBlocks (unsigned int blocks)
  {
    blocks %= (N / AUDIO_BLOCK_SAMPLES) ;
    overlap_blocks = blocks ;
  }

  virtual void update(void);
  uint16_t * output ;

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
  uint8_t state;
  volatile bool outputflag;
  audio_block_t * inputQueueArray [1];
  arm_rfft_instance_q31 fft_inst;
  int total_blocks ;
  int overlap_blocks ;
  bool valid ;
};

#endif
