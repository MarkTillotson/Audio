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
    delete [] states ;
  }
  order = _order ;
  if (order == 4)
    bits = 1 ;
  else if (order == 16)
    bits = 2 ;
  else if (order == 64)
    bits = 3 ;
  else if (order == 256)
    bits = 4 ;
  symbol_freq = _symbol_freq ;
  beta = _beta ;
  source = _source ;

  // calculate oversampling, points per symbol, etc
  points_per_symbol = AUDIO_SAMPLE_RATE_EXACT / symbol_freq ;
  int wanted_points = 4 * int (round (sqrt (order))) ;
  if (wanted_points > points_per_symbol)
  {
    oversampling = int (wanted_points / points_per_symbol) + 1 ;
    points_per_symbol *= oversampling ;
  }
  else
    oversampling = 1 ;
  phase_step = 1.0 / (points_per_symbol/oversampling) ;
  phase = 1.0 ;  // set up to trigger immediately on first sample

  int waves = int (sqrt (order) + 1.0) ;
  int half_points = int (waves * points_per_symbol) ;
    
  wavl_size = 2 * half_points - 1 ;
  table = new float [wavl_size] ;
  Serial.printf ("table size %i\n", wavl_size) ;

  gen_impulse_response (table, 1.0, 1.0 / points_per_symbol, wavl_size) ;

  /*
  for (int i = 0 ; i < wavl_size ; i++)
    Serial.println (table[i]) ;
  */
  
  max_states = int (wavl_size / oversampling) + 1 ;
  states = new struct QAM_wavelet_state [max_states] ;
  Serial.printf ("states %i, overs %i\n", max_states, oversampling) ;
  ins = del = 0 ;

  active = true;
}

void AudioSynthQAM::process_active_states (float & i_sum, float & q_sum)
{
  i_sum = 0.0 ;
  q_sum = 0.0 ;
  int p = del ;
  while (p != ins)
  {
    process_active_state (i_sum, q_sum, states + p) ;
    if (state_dead (states + p) && p == del)
    {
      del += 1 ;
      if (del >= max_states)
	del = 0 ;
    }
      
    if (++p >= max_states)
      p = 0 ;
  }
}

inline bool AudioSynthQAM::state_dead (struct QAM_wavelet_state * state)
{
  return state->offset >= wavl_size ;
}
  

inline void AudioSynthQAM::process_active_state (float & i_sum, float & q_sum, struct QAM_wavelet_state * state)
{
  int off = state->offset ;
  float val = table [off] ;
  state->offset = off + oversampling ;
  i_sum += state->i_scale * val ;
  q_sum += state->q_scale * val ;
}

void AudioSynthQAM::gen_impulse_response (float * arr, float Ts, float t, int n)
{
  int half = n/2 ;
  for (int i = 0 ; i < n ; i++)
  {
    float x = i + 0.5 - half ;
    if (root_raised)
      arr [i] = gen_root_raised_cosine (x*t, Ts) ;
    else
      arr [i] = gen_raised_cosine (x*t, Ts) ;  // for loopback tests
  }
}

void AudioSynthQAM::set_loopback (bool lb)
{
  root_raised = !lb ;
  if (active)
    gen_impulse_response (table, 1.0, 1.0 / points_per_symbol, wavl_size) ;
}


float AudioSynthQAM::gen_root_raised_cosine (float t, float Ts)
{
  if (t == 0.0)
    return (1.0 + beta * (4 / M_PI - 1.0)) / Ts ;
  float x = t / Ts ;
  float disc = 4 * beta * x ;
  if (abs (disc) == 1.0)
  {
    float a = M_PI / 4 / beta ;
    return beta / (Ts * sqrt(2)) * ((1 + 2/M_PI) * sin (a) + (1 - 2/M_PI) * cos (a)) ;
  }
  return 1.0 / Ts * (sin (M_PI * x * (1 - beta)) + disc * cos (M_PI * x * (1 + beta))) / (M_PI * x * (1 - disc*disc)) ;
}


inline static float sinc (float x)
{
  if (x == 0.0)
    return 1.0 ;
  x *= M_PI ;
  return sin (x) / x ;
}

float AudioSynthQAM::gen_raised_cosine (float t, float Ts)
{
  float x = t / Ts ;
  float disc = 2 * beta * x ;
  if (abs (disc) == 1.0)
    return M_PI / (4 * Ts) * sinc (0.5 / beta) ;
  return (1 / Ts) * sinc (x) * cos (M_PI * beta * x) / (1 - disc*disc) ;
}

void AudioSynthQAM::add_state (float i_scale, float q_scale, int offset)
{
  states [ins].i_scale = i_scale ;
  states [ins].q_scale = q_scale ;
  states [ins].offset = offset ;
  ins ++ ;
  if (ins >= max_states)
    ins = 0 ;
}


void AudioSynthQAM::update (void)
{
  if (!active)
    return ;
  audio_block_t * I_blk = allocate () ;
  if (I_blk == NULL)
    return ;
  audio_block_t * Q_blk = allocate () ;
  if (Q_blk == NULL)
  {
    release (I_blk) ;
    return ;
  }

  int N = 1<<bits ;

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    phase += phase_step ;
    if (phase >= 1.0)
    {
      phase -= 1.0 ;
      float fraction = phase * points_per_symbol / oversampling ;
      int index = int (round (fraction)) ;

      // generate new symbols here
      int binary ;
      if (source == QAM_SOURCE_RAMP)
	binary = sample_number & (N*N - 1) ;
      else if (source == QAM_SOURCE_RAND)
	binary = random (N*N) ;

      int i_val = binary >> bits ;
      int q_val = binary - (i_val << bits) ;
      float i_value = i_val;
      float q_value = q_val;
      i_value -= (N - 1.0) / 2 ;
      q_value -= (N - 1.0) / 2 ;
      i_value /= N ;
      q_value /= N ;
      add_state (i_value, q_value, index) ;

      sample_number ++ ;
    }

    float i_sample, q_sample ;
    process_active_states (i_sample, q_sample) ;
    I_blk->data [i] = int (round (32767.0 * i_sample)) ;
    Q_blk->data [i] = int (round (32767.0 * q_sample)) ;
  }

  transmit (I_blk, 0) ; release (I_blk) ;
  transmit (Q_blk, 1) ; release (Q_blk) ;
}

