/* Audio Library for Teensy 3.X
 * Copyright (c) 2018, Paul Stoffregen, paul@pjrc.com
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

#include <Arduino.h>
#include "synth_qam.h"
#include "arm_math.h"
#include "utility/dspinst.h"


void AudioSynthQAM::setup (int _order, float _symbol_freq, float _beta, int _source)
{
  if (active)
  {
    active = false ;
    delete [] table ;
    delete [] I_states ;
    delete [] Q_states ;
  }
  order = _order ;
  symbol_freq = _symbol_freq ;
  beta = _beta ;
  source = _source ;

  // calculate oversampling, points per symbol, etc
  points_per_symbol = AUDIO_SAMPLE_RATE_EXACT / symbol_freq ;
  int wanted_points = 4 * isqrt (order) ;
  if (wanted_points > points_per_symbol)
  {
    oversampling = int (wanted_points / points_per_symbol) + 1 ;
    points_per_symbol *= oversampling ;
  }
  else
    oversampling = 1 ;
  phase_step = 1.0 / points_per_symbol ;
  phase = 1.0 ;  // set up to trigger immediately on first sample

  int waves = int (sqrt (order) + 1.0) ;
  int half_points = int (waves * points_per_symbol) ;
    
  wavl_size = 2 * half_points - 1 ;
  table = new float [wavl_size] ;

  gen_root_raised_impulse (table, 1.0, 1.0 / points_per_symbol, wavl_size, beta) ;

  max_states = int (wavl_size / (AUDIO_SAMPLE_RATE_EXACT / symbol_freq) / oversampling) + 1 ;
  I_states = new struct PAM_wavelet_state [max_states] ;
  Q_states = new struct PAM_wavelet_state [max_states] ;
  i_ins = i_del = 0 ;
  q_ins = q_del = 0 ;

  active = true;
}

float AudioSynthQAM::process_active_states (struct PAM_wavelet_state * array, int ins, int del)
{
  float value = 0.0 ;
  int p = del ;
  while (p != ins)
  {
    value += process_active_state (array + p) ;
    if (state_dead (array + p) && p == del)
      del += 1 ;
      
    if (++p >= max_states)
      p = 0 ;
  }

  return value ;
}

inline bool AudioSynthQAM::state_dead (struct PAM_wavelet_state * state)
{
  return state->offset >= wavl_size ;
}
  

inline float AudioSynthQAM::process_active_state (struct PAM_wavelet_state * state)
{
  int off = state->offset ;
  float val = table [off] ;
  state->offset = off + oversampling ;
  return state->scale * val ;
}

void AudioSynthQAM::gen_root_raised_impulse (float * arr, float Ts, float t, int n)
{
  int half = n/2 ;
  for (int i = 0 ; i < n ; i++)
  {
    float x = i + 0.5 - half ;
    arr [i] = root_raised_cosine (x*t, Ts) ;
  }
}

float AudioSynthQAM::root_raised_cosine (float t, float Ts)
{
  float b = beta ;
  if (t == 0.0)
    return (1.0 + b * (4 / M_PI - 1.0)) / Ts ;
  float x = t / Ts ;
  float disc = 4 * b * x ;
  if (abs (disc) == 1.0)
  {
    a = M_PI / 4 / b ;
    return b / (Ts * sqrt(2)) * ((1 + 2/M_PI) * sin (a) + (1 - 2/M_PI) * cos (a)) ;
  }
  return 1.0 / Ts * (sin (M_PI * x * (1-b)) + disc * cos (M_PI * x * (1+b))) / (M_PI * x * (1 - disc*disc)) ;
}


void AudioSynthQAM::update (void)
{
  if (!active)
    return ;
  audio_block_t * I_blk = allocate () ;
  audio_block_t * Q_blk = allocate () ;
  if (I_blk == NULL && Q_blk == NULL)    return ;
  if (I_blk == NULL) { release (Q_blk) ; return ; }
  if (Q_blk == NULL) { release (I_blk) ; return ; }
  

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    phase += phase_step ;
    if (phase >= 1.0)
    {
      phase -= 1.0 ;
      // generate new symbols here
    }

    float i_sample = process_active_states (I_states, i_ins, i_del) ;
    float q_sample = process_active_states (Q_states, q_ins, q_del) ;
    I_blk->data [i] = int (round (32767.0 * i_sample)) ;
    Q_blk->data [i] = int (round (32767.0 * i_sample)) ;
  }

  transmit (I_blk, 0) ; release (I_blk) ;
  transmit (Q_blk, 1) ; release (Q_blk) ;
}

