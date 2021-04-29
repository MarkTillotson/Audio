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

#ifndef synth_qam_h_
#define synth_qam_h_

// source types
#define QAM_SOURCE_LFSR 0
#define QAM_SOURCE_RAMP 1
#define QAM_SOURCE_FN   2
#define QAM_SOURCE_RAND 3

#include <Arduino.h>
#include "AudioStream.h"
#include "arm_math.h"

typedef struct QAM_wavelet_state
{
  int offset ;
  float i_scale, q_scale ;
} QAM_wavelet_state ;


class AudioSynthQAM : public AudioStream
{
public:
  AudioSynthQAM(void) : AudioStream(0,NULL),
    order (16),
    symbol_freq (11050.0),
    beta (0.5),
    source (QAM_SOURCE_LFSR)
  {
    sample_number = 0 ;
    active = false ;
    root_raised = true ;
    setup (16, 11050, 0.5, QAM_SOURCE_LFSR) ;
  }

  void setup (int _order, float _symbol_freq, float _beta, int _source);

  void set_loopback (bool lb);
  
  virtual void update(void);

private:
  void process_active_states (float & i_sum, float & q_sum) ;
  void add_state (float i_scale, float q_scale, int offset) ;
  inline bool state_dead (struct QAM_wavelet_state * state) ;
  inline void process_active_state (float & i_sum, float & q_sum, struct QAM_wavelet_state * state) ;
  void gen_impulse_response (float * arr, float Ts, float t, int n) ;
  float gen_root_raised_cosine (float t, float Ts);
  float gen_raised_cosine (float t, float Ts);

  
  volatile bool active;
  int order, N ;
  bool root_raised ;
  long sample_number ;
  float symbol_freq;
  float beta;
  int source;
  float phase, phase_step ;
  float points_per_symbol ;
  int oversampling;
  int wavl_size;

  int max_states ;
  int ins, del ;
  struct QAM_wavelet_state * states;

  float * table ;
};


#endif
