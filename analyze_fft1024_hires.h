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

#ifndef analyze_fft1024_hires_h_
#define analyze_fft1024_hires_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"

// windows.c
extern "C" {
extern const int16_t AudioWindowHanning1024[];
extern const int16_t AudioWindowBartlett1024[];
extern const int16_t AudioWindowBlackman1024[];
extern const int16_t AudioWindowFlattop1024[];
extern const int16_t AudioWindowFHT144D1024[];
extern const int16_t AudioWindowBlackmanHarris1024[];
extern const int16_t AudioWindowNuttall1024[];
extern const int16_t AudioWindowBlackmanNuttall1024[];
extern const int16_t AudioWindowWelch1024[];
extern const int16_t AudioWindowHamming1024[];
extern const int16_t AudioWindowCosine1024[];
extern const int16_t AudioWindowTukey1024[];
}

class AudioAnalyzeFFT1024HiRes : public AudioStream
{
public:
  AudioAnalyzeFFT1024HiRes() : AudioStream(1, inputQueueArray),
			       window(AudioWindowHanning1024), state(total_blocks/2), outputflag(false) {
    arm_rfft_init_q31 (&fft_inst, 1024, 0, 1);  // use 32 bit fixpoint real fft
  }
  
  bool available()
  {
    if (outputflag == true)
    {
      outputflag = false;
      return true;
    }
    return false;
  }

  float read(unsigned int binNumber)
  {
    if (binNumber > 511) return 0.0;
    return (float)(output[binNumber]) / 0x40000000 ;
  }

  float read(unsigned int binFirst, unsigned int binLast)
  {
    if (binFirst > binLast)
    {
      unsigned int tmp = binLast;
      binLast = binFirst;
      binFirst = tmp;
    }
    if (binFirst > 511)
      return 0.0;
    if (binLast > 511)
      binLast = 511;
    float sum = 0.0;
    do {
      sum += output[binFirst++];
    } while (binFirst <= binLast);
    return sum / 0x40000000 ;
  }

  void averageTogether(uint8_t n)
  {
    // not implemented yet (may never be, 86 Hz output rate is ok)
  }

  void windowFunction(const int16_t *w)
  {
    window = w;
  }

  // Select how many audio blocks to overlap, 0..7 are valid
  void overlapBlocks(int b)
  {
    overlap_blocks = b & 7 ;
  }

  virtual void update(void);
  uint32_t output[512] __attribute__ ((aligned (4)));

protected:
  static const int total_blocks = 8 ;

private:
  void init(void);
  const int16_t *window;
  audio_block_t *blocklist[total_blocks];
  int32_t r_buffer[1024] __attribute__ ((aligned (4))); // input to real fft
  int32_t c_buffer[2048] __attribute__ ((aligned (8))); // output from real fft is complex
  uint8_t state;
  volatile bool outputflag;
  audio_block_t *inputQueueArray[1];
  arm_rfft_instance_q31 fft_inst;
  int overlap_blocks = 4 ;
};

#endif
