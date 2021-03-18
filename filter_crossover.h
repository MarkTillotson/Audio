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

#ifndef filter_crossover_h_
#define filter_crossover_h_

#include "Arduino.h"
#include "AudioStream.h"



class FilterSpec
{
public:
  FilterSpec () {}

  virtual void apply (int n, float * samples) = 0 ;
};


#define FILTERSPEC_MAX_SOS_SECTIONS 4  // for 8 pole

#define CROSSOVER_TYPE_BUTTERWORTH 0
#define CROSSOVER_TYPE_LINKWITZ_RILEY 1  // should be enum
#define LAST_CROSSOVER_TYPE 1

class FilterSpecIIR : public FilterSpec  // maybe have a FIR option too?
{
public:
  FilterSpecIIR (unsigned int _sos_count, float * _sos_coeffs) : FilterSpec()
  {
    initialize (_sos_count, _sos_coeffs) ;
  }

  virtual void apply (int n, float * samples) ;

private:
  unsigned int sos_count ;
  float sos_coeffs [6 * FILTERSPEC_MAX_SOS_SECTIONS] ;
  float sos_state  [2 * FILTERSPEC_MAX_SOS_SECTIONS] ;

  void initialize (unsigned int _sos_count, float * _sos_coeffs) ;
  void sos_apply (float * state, float * coeffs, int n, float * samples) ;
};



class AudioFilterCrossover : public AudioStream
{
public:
  AudioFilterCrossover(void) : AudioStream(2, inputQueueArray)
  {
    stages = 0 ;
    for (int i = 0 ; i < 6 ; i++)
      specs[i] = NULL ;
  }

  
  virtual void update(void);

  bool setStages (int ways)
  {
    if (ways < 2) return false ;
    if (ways > 4) return false ;
    stages = ways - 1 ;
    return true ;
  }

  bool setLowpass (unsigned int stage, FilterSpec * lowpass)
  {
    if (stage >= stages) return false ;
    specs [stage * 2 - 2] = lowpass ;
    return true ;
  }
	
  bool setHighpass (unsigned int stage, FilterSpec * highpass)
  {
    if (stage >= stages) return false ;
    specs [stage * 2 - 1] = highpass ;
    return true ;
  }

  bool setFilters (unsigned int stage, FilterSpec * lowpass, FilterSpec * highpass)
  {
    if (stage >= stages) return false ;
    specs [stage * 2 - 2] = lowpass ;
    specs [stage * 2 - 1] = highpass ;
    return true ;
  }

  bool setCrossoverFreq (unsigned int stage, float freq, int crossover_type)
  {
    if (stage >= stages || crossover_type > LAST_CROSSOVER_TYPE)
      return false ;
    specs [stage * 2 - 2] = crossover_lowpass_for (freq, crossover_type) ;
    specs [stage * 2 - 1] = crossover_highpass_for (freq, crossover_type) ;
    return true ;
  }

private:
  audio_block_t * inputQueueArray [2];
  unsigned int stages ;
  FilterSpec * specs[6] ;
  float saved_samples [AUDIO_BLOCK_SAMPLES] ;
  float left_samples [AUDIO_BLOCK_SAMPLES] ;
  float right_samples [AUDIO_BLOCK_SAMPLES] ;
  
  void output_to_block (int chan, float * samples) ;
  FilterSpec * crossover_lowpass_for (float freq, int crossover_type) ;
  FilterSpec * crossover_highpass_for (float freq, int crossover_type) ;
};

#endif
