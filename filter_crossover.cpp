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



static void sos_apply (float * state, float * coeffs, int n, float * samples)
{
  const float gain =   coeffs[0], a1 = coeffs[1], a2 = coeffs[2] ;
  const float b0 = 1.0/coeffs[3], b1 = coeffs[4], b2 = coeffs[5] ;
  float s1 = state[0], s2 = state[1] ;

  if (a1 == 2.0 && a2 == 1.0)
  {
    for (int i = 0 ; i < n ; i+=2)
    {
      float val = samples[i] ;
      float val2 = samples[i+1] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      //val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val2 -= b1 * s0 ;
      val2 -= b2 * s1 ;
      float s00 = val2 ;
      val += s1 ;
      val += s1 ;
      val += s2 ;
      val2 += s0 ;
      val2 += s0 ;
      val2 += s1 ;
      val *= gain ;
      samples[i] = val ;
      val2 *= gain ;
      samples[i+1] = val2 ;
      s2 = s0 ;
      s1 = s00 ;
    }
  }
  else if (a1 == -2.0 && a2 == 1.0)
  {
    for (int i = 0 ; i < n ; i+=2)
    {
      float val = samples[i] ;
      float val2 = samples[i+1] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      //val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val2 -= b1 * s0 ;
      val2 -= b2 * s1 ;
      float s00 = val2 ;
      val -= s1 ;
      val -= s1 ;
      val += s2 ;
      val2 -= s0 ;
      val2 -= s0 ;
      val2 += s1 ;
      val *= gain ;
      samples[i] = val ;
      val2 *= gain ;
      samples[i+1] = val2 ;
      s2 = s0 ;
      s1 = s00 ;
    }
  }
  else
  {
    for (int i = 0 ; i < n ; i+=2)
    {
      float val = samples[i] ;
      float val2 = samples[i+1] ;
      val -= b1 * s1 ;
      val -= b2 * s2 ;
      //val *= b0 ;  // this coefficient we reciprocated already to avoid a division here.
      float s0 = val ;
      val2 -= b1 * s0 ;
      val2 -= b2 * s1 ;
      float s00 = val2 ;
      val += a1 * s1 ;
      val += a2 * s2 ;
      val2 += a1 * s0 ;
      val2 += a2 * s1 ;
      val *= gain ;
      samples[i] = val ;
      val2 *= gain ;
      samples[i+1] = val2 ;
      s2 = s0 ;
      s1 = s00 ;
    }
  }
  state[0] = s1 ;
  state[1] = s2 ;
}


void FilterSpecIIR::apply (int n, float * samples, bool alt_state)
{
  for (unsigned int i = 0 ; i < sos_count ; i++)
    sos_apply ((alt_state ? sos_state2 : sos_state) + 2*i, sos_coeffs + 6*i, n, samples) ;
}

void AudioFilterCrossover::input_from_block (audio_block_t * block, float * samples)
{
  if (block == NULL)
    arm_fill_f32 (0.0, samples, AUDIO_BLOCK_SAMPLES) ;
  else
  {
    arm_q15_to_float (block->data, samples, AUDIO_BLOCK_SAMPLES) ;
    release (block) ;
  }
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

static void apply_filter (FilterSpec * filter, float * samples, bool alternate_state)
{
  if (filter != NULL)
    filter->apply (AUDIO_BLOCK_SAMPLES, samples, alternate_state) ;
}


static void bilinear_z_transform (float * s)
{
  float u = 1 + s[0] ;
  float v =   + s[1] ;
  float x = 1 - s[0] ;
  float y =   - s[1] ;
  float scale = 1.0 / (x*x + y*y) ;
  float xnum = u*x + v*y ;
  float ynum = v*x - u*y ;
  s[0] = xnum * scale ;
  s[1] = ynum * scale ;
}


static void get_butterworth_poles (float f, int order, float * poles)
{
  float omega = tan (M_PI * f) ;
  if (order & 1)
  {
    *poles++ = -omega ;
    *poles++ = 0.0 ;
    bilinear_z_transform (poles-2) ;
    for (int i = 2 ; i < order ; i+=2)
    {
      float angle = i * 0.5 * M_PI / order ;
      *poles++ = -omega * cos(angle) ;
      *poles++ =  omega * sin(angle) ;
      bilinear_z_transform (poles-2) ;
    }
  }
  else
  {
    for (int i = 1 ; i < order ; i+=2)
    {
      float angle = i * 0.5 * M_PI / order ;
      *poles++ = -omega * cos(angle) ;
      *poles++ =  omega * sin(angle) ;
      bilinear_z_transform (poles-2) ;
    }
  }
}

static void sos_coeffs_for_1st_order (float * sos_coeffs, float x, bool low_pass)
{
  float gain = low_pass ? 2.0 / (1-x) : 2.0 / (1+x) ;
  sos_coeffs[0] = 1.0 / gain ;
  sos_coeffs[1] = low_pass ? 1.0 : -1.0 ;
  sos_coeffs[2] = 0.0 ;
  sos_coeffs[3] = 1.0 ;
  sos_coeffs[4] = -x ;
  sos_coeffs[5] = 0.0 ;
  //Serial.printf ("1st order %f %f %f %f %f %f\n", sos_coeffs[0], sos_coeffs[1], sos_coeffs[2], sos_coeffs[3], sos_coeffs[4], sos_coeffs[5]) ;
}

static void sos_coeffs_for_squared_1st_order (float * sos_coeffs, float x, bool low_pass)
{
  float gain = low_pass ? 2.0 * 2.0 / ((1-x)*(1-x)) : 2.0 * 2.0 / ((1+x)*(1+x)) ;
  sos_coeffs[0] = 1.0 / gain ;
  sos_coeffs[1] = low_pass ? 2.0 : -2.0 ;
  sos_coeffs[2] = 1.0 ;
  sos_coeffs[3] = 1.0 ;
  sos_coeffs[4] = -2.0 * x ;
  sos_coeffs[5] = x*x ;
  //Serial.printf ("1st order sq %f %f %f %f %f %f\n", sos_coeffs[0], sos_coeffs[1], sos_coeffs[2], sos_coeffs[3], sos_coeffs[4], sos_coeffs[5]) ;
}

static void sos_coeffs_for_2nd_order (float * sos_coeffs, float x, float y, bool low_pass)
{
  float gain = low_pass ? 2.0 * 2.0 / ((1-x)*(1-x) + y*y) : 2.0 * 2.0 / ((1+x)*(1+x) + y*y) ;
  sos_coeffs[0] = 1.0 / gain ;
  sos_coeffs[1] = low_pass ? 2.0 : -2.0 ;
  sos_coeffs[2] = 1.0 ;
  sos_coeffs[3] = 1.0 ;
  sos_coeffs[4] = -2.0 * x ;
  sos_coeffs[5] = x*x + y*y ;
  //Serial.printf ("2nd order %f %f %f %f %f %f\n", sos_coeffs[0], sos_coeffs[1], sos_coeffs[2], sos_coeffs[3], sos_coeffs[4], sos_coeffs[5]) ;
}


FilterSpec * AudioFilterCrossover::crossover_lowpass_for (float freq, int crossover_type, int order)
{
  float poles [2 * FILTERSPEC_MAX_SOS_SECTIONS] ;
  switch (crossover_type)
  {
  case CROSSOVER_TYPE_BUTTERWORTH:
    {
      if (order < 1 || order > 2 * FILTERSPEC_MAX_SOS_SECTIONS)
	return NULL ;
      get_butterworth_poles (freq / AUDIO_SAMPLE_RATE_EXACT, order, poles) ;
      float sos_coeffs [6 * FILTERSPEC_MAX_SOS_SECTIONS] ;
      int index = 0 ;
      if (order & 1)
      {
	sos_coeffs_for_1st_order (sos_coeffs, poles[0], true) ;
	index += 6 ;
	for (int i = 2 ; i < order ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], true) ;
	  index += 6 ;
	}
      }
      else
      {
	for (int i = 1 ; i < order ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], true) ;
	  index += 6 ;
	}
      }
      return new FilterSpecIIR ((order+1) / 2, sos_coeffs) ;
    }
    break ;
    
  case CROSSOVER_TYPE_LINKWITZ_RILEY:
    {
      if (order < 2 || order > 2 * FILTERSPEC_MAX_SOS_SECTIONS || (order & 1) == 1)
	return NULL ;
      int border = order/2 ;
      get_butterworth_poles (freq / AUDIO_SAMPLE_RATE_EXACT, border, poles) ;
      float sos_coeffs [6 * FILTERSPEC_MAX_SOS_SECTIONS] ;
      int index = 0 ;
      if (border & 1)
      {
	sos_coeffs_for_squared_1st_order (sos_coeffs+index, poles[0], true) ;
	index += 6 ;
	for (int i = 2 ; i < border ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], true) ;
	  index += 6 ;
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], true) ;
	  index += 6 ;
	}
      }
      else
      {
	for (int i = 1 ; i < border ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], true) ;
	  index += 6 ;
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], true) ;
	  index += 6 ;
	}
      }
      Serial.printf ("LINK low %i\n", index) ;
      return new FilterSpecIIR (order / 2, sos_coeffs) ;
    }
    break ;
  }
  return NULL ;
}

FilterSpec * AudioFilterCrossover::crossover_highpass_for (float freq, int crossover_type, int order)
{
  float poles [2 * FILTERSPEC_MAX_SOS_SECTIONS] ;
  switch (crossover_type)
  {
  case CROSSOVER_TYPE_BUTTERWORTH:
    {
      if (order < 1 || order > 2 * FILTERSPEC_MAX_SOS_SECTIONS)
	return NULL ;
      get_butterworth_poles (freq / AUDIO_SAMPLE_RATE_EXACT, order, poles) ;
      float sos_coeffs [6 * FILTERSPEC_MAX_SOS_SECTIONS] ;
      int index = 0 ;
      if (order & 1)
      {
	sos_coeffs_for_1st_order (sos_coeffs, poles[0], false) ;
	index += 6 ;
	for (int i = 2 ; i < order ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], false) ;
	  index += 6 ;
	}
      }
      else
      {
	for (int i = 1 ; i < order ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], false) ;
	  index += 6 ;
	}
      }
      return new FilterSpecIIR ((order+1) / 2, sos_coeffs) ;
    }
    break ;
    
  case CROSSOVER_TYPE_LINKWITZ_RILEY:
    {
      if (order < 2 || order > 2 * FILTERSPEC_MAX_SOS_SECTIONS || (order & 1) == 1)
	return NULL ;
      int border = order/2 ;
      get_butterworth_poles (freq / AUDIO_SAMPLE_RATE_EXACT, border, poles) ;
      float sos_coeffs [6 * FILTERSPEC_MAX_SOS_SECTIONS] ;
      int index = 0 ;
      if (border & 1)
      {
	sos_coeffs_for_squared_1st_order (sos_coeffs+index, poles[0], false) ;
	index += 6 ;
	for (int i = 2 ; i < border ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], false) ;
	  index += 6 ;
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i], poles[i+1], false) ;
	  index += 6 ;
	}
      }
      else
      {
	for (int i = 1 ; i < border ; i+=2)
	{
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], false) ;
	  index += 6 ;
	  sos_coeffs_for_2nd_order (sos_coeffs + index, poles[i-1], poles[i], false) ;
	  index += 6 ;
	}
      }
      Serial.printf ("LINK high %i\n", index) ;
      return new FilterSpecIIR (order / 2, sos_coeffs) ;
    }
    break ;
  }
  return NULL ;
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
    apply_filter (specs[2*i+1], left_samples, false) ;         // high pass for stage
    output_to_block (2*i, left_samples) ;               // output it
    arm_copy_f32 (saved_samples, left_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i], left_samples, false) ;           // low pass for stage - retain for next stage

    arm_copy_f32 (right_samples, saved_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i+1], right_samples, true) ;        // high pass for stage
    output_to_block (2*i+1, right_samples) ;            // output it
    arm_copy_f32 (saved_samples, right_samples, AUDIO_BLOCK_SAMPLES) ;
    apply_filter (specs[2*i], right_samples, true) ;          // low pass for stage - retain for next stage
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
