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

#include "filter_crossover.h"
#include <arm_math.h>

#if defined(__ARM_ARCH_7EM__)



void FilterSpecIIR::initialize (unsigned int _sos_count, float * _sos_coeffs)
{
  arm_fill_f32 (0.0, sos_state, 2 * FILTERSPEC_MAX_SOS_SECTIONS) ;
  if (1 <= _sos_count && _sos_count <= FILTERSPEC_MAX_SOS_SECTIONS)
  {
    sos_count = _sos_count ;
    arm_copy_f32 (_sos_coeffs, sos_coeffs, 6 * sos_count) ;
    for (unsigned int i = 0 ; i < sos_count ; i++)
      sos_coeffs [6*i + 3] = 1.0 / sos_coeffs [6*i + 3] ;  // reciprocate b0 in advance
  }
  else
    sos_count = 0 ;
}


void FilterSpecIIR::apply (int n, float * samples)
{
  for (unsigned int i = 0 ; i < sos_count ; i++)
    sos_apply (sos_state + 2*i, sos_coeffs + 6*i, n, samples) ;
}


void FilterSpecIIR::sos_apply (float * state, float * coeffs, int n, float * samples)
{
  float a0 =     coeffs[0], a1 = coeffs[1], a2 = coeffs[2] ;
  float b0 = 1.0/coeffs[3], b1 = coeffs[4], b2 = coeffs[5] ;
  float s1 = state[0], s2 = state[1] ;

  if (a0 == 1.0 && a1 == 2.0 && a2 == 1.0)  // quicker case for most low pass stages (zeroes both at Nyquist)
  {
    for (int i = 0 ; i < n ; i++)
    {
      float val = samples[i] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val += 2 * s1 ;
      val += s2 ;
      samples[i] = val ;
      s2 = s1 ;
      s1 = s0 ;
    }
  }
  else if (a0 == 1.0 && a1 == -2.0 && a2 == 1.0)  // quicker case for most high pass stages (zeroes both at DC)
  {
    for (int i = 0 ; i < n ; i++)
    {
      float val = samples[i] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val -= 2 * s1 ;
      val += s2 ;
      samples[i] = val ;
      s2 = s1 ;
      s1 = s0 ;
    }
  }
  else
  {
    for (int i = 0 ; i < n ; i++)
    {
      float val = samples[i] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val *= a0 ;
      val += a1 * s1 ;
      val += a2 * s2 ;
      samples[i] = val ;
      s2 = s1 ;
      s1 = s0 ;
    }
  }
  state[0] = s1 ;
  state[1] = s2 ;
}


static void input_from_block (audio_block_t * block, float * samples)
{
  if (block == NULL)
    arm_fill_f32 (0.0, samples, AUDIO_BLOCK_SAMPLES) ;
  else
    arm_q15_to_float (block->data, samples, AUDIO_BLOCK_SAMPLES) ;
}

void AudioFilterCrossover::output_to_block (int chan, float * samples)
{
  audio_block_t * block = allocate () ;
  if (block == NULL) 
    return ;
  arm_float_to_q15 (samples, block->data, AUDIO_BLOCK_SAMPLES) ;
  transmit (block, chan) ;
  release (block) ;
}

static void apply_filter (FilterSpec * filter, float * samples)
{
  if (filter != NULL)
    filter->apply (AUDIO_BLOCK_SAMPLES, samples) ;
}


void AudioFilterCrossover::update (void)
{
  if (stages == 0)
    return ;

  input_from_block (receiveReadOnly (0), left_samples) ;
  input_from_block (receiveReadOnly (1), right_samples) ;

  // each stage splits off a high pass stream, output immediately, and a low pass which is handed to next stage
  for (unsigned int i = 0 ; i < stages ; i++)
  {
    arm_copy_f32 (left_samples, saved_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i+1], left_samples) ;         // high pass for stage
    output_to_block (2*i, left_samples) ;               // output it
    arm_copy_f32 (saved_samples, left_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i], left_samples) ;           // low pass for stage - retain for next stage

    arm_copy_f32 (right_samples, saved_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i+1], right_samples) ;        // high pass for stage
    output_to_block (2*i+1, right_samples) ;            // output it
    arm_copy_f32 (saved_samples, right_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i], right_samples) ;          // low pass for stage - retain for next stage
  }

  output_to_block (2*stages,   left_samples) ;          // at end output the last, lowest frequency
  output_to_block (2*stages+1, right_samples) ;
}
    

#elif defined(KINETISL)

void AudioFilterCrossover::update (void)
{
  audio_block_t * block = receiveReadOnly();
  if (block)
    release (block);  // surely you don't have to release a read only.
}

#endif
