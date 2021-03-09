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

#include "filter_unipart.h"
#include "utility/dspinst.h"

#if defined(__ARM_ARCH_7EM__)




inline void reconstruct_neg_freqs (float * spect, int Npoints)
{
  int i = 2 ;
  int j = 2 * Npoints - 2 ;
  while (j > i)
  {
    spect[j]   =  spect[i++];
    spect[j+1] = -spect[i++];
    j -= 2 ;
  }
  spect[j]   = 0.0 ;
  spect[j+1] = 0.0 ;
}

inline void mul_acc_spectrum (float * filt_spectrum,
			      float * in_spectrum,
			      float * out_spectrum,
			      int Nbins)
{
  float out_temp [2*Nbins] ;
  arm_cmplx_mult_cmplx_f32 (filt_spectrum, in_spectrum, out_temp, Nbins) ; // multiply...
  arm_add_f32 (out_temp, out_spectrum, out_spectrum, 2*Nbins) ; // ...accumulate
}


// allows inc to be negative without breaking due to C semantics for % operator being wrong.
inline void inc_mod (int & a, int inc, int b)
{
  a = (a + inc + b) % b ;
}


void AudioFilterUnipart::update (void)
{
  if (!active)
  {
    audio_block_t * block = receiveWritable();
    if (block != NULL)
      release (block) ;
    return ;
  }

  const int L = AUDIO_BLOCK_SAMPLES ;  // less cumbersome
  const int N = 2*L ;

  audio_block_t * block = receiveWritable();  // since passed down via prev_blk to out_blk
  audio_block_t * out_blk = NULL ;

  // the in_array gets the previous and curret blocks samples, and we have
  // to handle lack of blocks as if were zero:
  if (prev_blk == NULL)
  {
    arm_fill_f32 (0.0, in_array, L) ;
    out_blk = allocate () ;
  }
  else
  {
    arm_q15_to_float (prev_blk->data, in_array, L) ;
    out_blk = prev_blk ;   // reuse the previous block for output
  }
  
  if (block == NULL)
    arm_fill_f32 (0.0, in_array + L, L) ;
  else
    arm_q15_to_float (block->data, in_array + L, L) ;

  prev_blk = block ;


  // write latest spectrum to relevant slot in the delay_line, end of input processing
  arm_rfft_fast_f32 (&fft_instance, in_array, delay_line + state * 2*L, 0) ;
  inc_mod (state, 1, Npart+1) ;  // state indexes delay_line, 

  if (out_blk != NULL)  // if starved of blocks, don't bother doing the output stuff
  {
    arm_fill_f32 (0.0, out_spectra, 2*L) ; // clear the output accumulator
    int del_index = state ;
    for (int i = 0 ; i < Npart ; i++)      // loop through all the filter partitions
    {
      inc_mod (del_index, -1, Npart+1) ;  // delayline index goes backwards (it is a convolution, not correlation!)
      // multiply each delayline spectrum by relevant filter spectrum, accumulate in out_spectra
      mul_acc_spectrum (filt_spectra + i * 2*L,
			delay_line + del_index * 2*L,
			out_spectra,
			L) ;
    }
    reconstruct_neg_freqs (out_spectra, N) ;  // we only bothered above with positive frequency bins...

    arm_rfft_fast_f32 (&fft_instance, out_spectra, out_array, 1) ;  // 1 means inverse fft

    arm_float_to_q15 (out_array + L, out_blk->data, L) ;  // just needs second half of samples
    transmit (out_blk) ;
    release (out_blk) ;
  }
  
}


void AudioFilterUnipart::setFIRCoefficients (int size, float * coeffs)
{
  const int L = AUDIO_BLOCK_SAMPLES ;  // less cumbersome

  if (coeffs == NULL)  // shutdown and release resources:
  {
    Npart = 0 ;
    state = 0 ;
    if (filt_spectra != NULL)
      delete[] filt_spectra ;
    if (delay_line != NULL)
      delete[] delay_line ;
    active = false ;
  }

  Npart = (size + L-1) / L ;  // setup state for update() loop
  state = 0 ;

  filt_spectra = new float [(Npart + 1) * 2*L] ;  // entries overlap by L complex values, need extra L complex at end
  if (filt_spectra == NULL)
    return ;
  delay_line = new float [(Npart + 2) * 2*L] ;    // entries overlap by L complex values, and we also wrap, need two extra L at end
  if (delay_line == NULL)
  {
    delete[] filt_spectra ;
    return ;
  }
  
  int index = 0 ;
  for (int i = 0 ; i < Npart ; i++)
  {
    arm_copy_f32 (coeffs + index, in_array, L) ;
    if (index+L > size)
      arm_fill_f32 (0.0, in_array + (size-index), index+2*L-size) ; // last partition may be shorter
    else
      arm_fill_f32 (0.0, in_array + L, L) ;
    
    arm_rfft_fast_f32 (&fft_instance, in_array, filt_spectra + 2*index, 0) ;  // store filt spectrum
    index += L ;
  }
  arm_fill_f32 (0.0, delay_line, (Npart+1) * 2*L) ;  // zero out delay line for clean startup

  active = true ;  // enable the update() method to start processing
}


void AudioFilterUnipart::pause (void)
{
  active = false ;
}

void AudioFilterUnipart::resume (void)
{
  active = true ;
}


#elif defined(KINETISL)

void AudioFilterUnipart::update (void)
{
  audio_block_t * block = receiveReadOnly();
  if (block != NULL)
    release (block) ;
}

#endif
