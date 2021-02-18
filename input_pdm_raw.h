/* Audio Library for Teensy 3.X
 * Copyright (c) 2018, Paul Stoffregen, paul@pjrc.com
 * enhancement 2021, Mark Tillotson
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

#ifndef _input_pdm_raw_h_
#define _input_pdm_raw_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "DMAChannel.h"

class AudioInputPDMRaw : public AudioStream
{
public:
  AudioInputPDMRaw(void) : AudioStream(0, NULL)
  {
    begin();
  }

  virtual void update(void);
  void begin(void);

protected:	
  static bool update_responsibility;
  static DMAChannel dma;
  static void isr(void);

private:
  static audio_block_t *block0;
  static audio_block_t *block1;
  static audio_block_t *block2;
  static audio_block_t *block3;
};



class AudioConvertFromPDMOld : public AudioStream
{
public:
  AudioConvertFromPDMOld (void): AudioStream (4, inputQueueArray)
  {
  }
  
  virtual void update (void) ;

protected:


private:
  audio_block_t * inputQueueArray [4] ;
};


class AudioConvertFromPDM : public AudioStream
{
public:
  AudioConvertFromPDM (void): AudioStream (4, inputQueueArray)
  {
    sum = 0 ;
    del1 = 0 ;
    del2 = 0 ;
    gain_factor = 0 ;
    dc_offset = 0 ;
  }
  
  virtual void update (void) ;

  void set_gain_factor (int gf)
  {
    gain_factor = gf < 0 ? 0 : gf > 7 ? 7 : gf ;
  }

protected:
  int16_t cicfilt (uint16_t b);

private:
  audio_block_t * inputQueueArray [4] ;
  int32_t sum ;
  uint32_t a1, a2, a3, a4, a5 ;  // integrators
  uint32_t d1, d2, d3, d4, d5 ;  // differentiators
  int32_t del1, del2 ;
  int gain_factor ;
  int32_t dc_offset ;
};



class AudioConvertToPDM : public AudioStream
{
public:
  AudioConvertToPDM (void): AudioStream (1, inputQueueArray)
  {
    accum = 0 ;
  }

  virtual void update (void) ;

private:
  audio_block_t * inputQueueArray [1] ;
  int32_t accum ;
};

#endif
