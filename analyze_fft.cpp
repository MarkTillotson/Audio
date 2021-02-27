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

#include "utility/dspinst.h"
#include "analyze_fft.h"

bool AudioAnalyzeFFT::available(void)
  {
    if (!valid)
      return false ;
    bool avail = outputflag ;
    outputflag = false ;
    return avail ;
  }

float AudioAnalyzeFFT::read (unsigned int binNumber)
{
  if (!valid || binNumber > N/2)
    return 0.0;
  return (float) output[binNumber] * processing_gain ;
}

float AudioAnalyzeFFT::dB (unsigned int binNumber)
{
  return 20 * log10 (read (binNumber)) ;
}

float AudioAnalyzeFFT::readNoise (unsigned int binNumber)
{
  if (!valid || binNumber > N/2)
    return 0.0;
  return (float) output[binNumber] * noise_gain ;
}

float AudioAnalyzeFFT::dBNoise (unsigned int binNumber)
{
  return 20 * log10 (readNoise (binNumber)) ;
}
  
float AudioAnalyzeFFT::read (unsigned int binFirst, unsigned int binLast)
{
  if (binFirst > binLast)
  {
    unsigned int tmp = binLast;
    binLast = binFirst;
    binFirst = tmp;
  }
  if (!valid || binFirst > N/2)
    return 0.0 ;
  if (binLast > N/2)
    binLast = N/2 ;
  uint64_t sum = 0L ;
  do
  {
    sum += output [binFirst++] ;
  } while (binFirst <= binLast) ;
    
  return (float) sum * processing_gain ;
}
  
void AudioAnalyzeFFT::fftWindow (FFTWindow * window_desc)
{
  __disable_irq() ;
  window_desc->expand_q15 (window, N) ;
  processing_gain = window_desc->processingGain() ;
  noise_gain = processing_gain * sqrt (window_desc->noiseBandwidth()) ;
  noise_gain *= sqrt (AUDIO_SAMPLE_RATE_EXACT / N) ;
  // scale for read() methods
  processing_gain = full_scale / 0x40000000 / processing_gain ;
  noise_gain = full_scale / 0x40000000 / noise_gain ;
  __enable_irq() ;
}

void AudioAnalyzeFFT::overlapBlocks (unsigned int blocks)
{
  blocks %= (N / AUDIO_BLOCK_SAMPLES) ;
  __disable_irq() ;
  if (overlap_blocks != blocks)
  {
    overlap_blocks = blocks ;
    for (int i = 0 ; i < total_blocks ; i++)
      if (blocklist[i] != NULL)
      {
        release (blocklist[i]) ;
        blocklist[i] = NULL ;
      }
  }
  __enable_irq() ;
}

void AudioAnalyzeFFT::fullScaleVolts (float amplitude)
{
  if (amplitude > 0)
    full_scale = amplitude ;
}


int AudioAnalyzeFFT::Npoints (unsigned int N_points)
{
  if (valid)
  {
    if (N == N_points)
      return N ;  // already the right size
    delete (output) ;
    delete (blocklist) ;
    delete (r_buffer) ;
    delete (c_buffer) ;
    delete (window) ;
  }
  N = N_points ;
  state = 0 ;
  outputflag = false ;
    
  total_blocks = N / AUDIO_BLOCK_SAMPLES ;
  overlap_blocks = total_blocks >> 1 ;
  if (arm_rfft_init_q31 (&fft_inst, N, 0, 1) != ARM_MATH_SUCCESS)
  {
    //Serial.println ("Bad size for AudioAnalyzeFFT") ;
    valid = false ;
    return 0 ;
  }

  output = new uint32_t [(N/2+1)] ;
  blocklist = new audio_block_t * [total_blocks] ;
  r_buffer = new int32_t [N] ;
  c_buffer = new int32_t [2 * N] ;
  window = new int16_t [N/2+1] ;
  if (output == NULL || blocklist == NULL || r_buffer == NULL || c_buffer == NULL || window == NULL)
  {
    //Serial.println ("malloc failed for AudioAnalyzeFFT") ;
    valid = false ;
    return 0 ;
  }
  
  for (int i = 0 ; i < total_blocks ; i++)
    blocklist[i] = NULL ;
  fftWindow (&hann_window) ;  // default window
  //Serial.printf ("Valid fft, N=%i, overlap=%i, total=%i, state=%i\n", N, overlap_blocks, total_blocks, state) ;
  __disable_irq() ;
  valid = true ;
  __enable_irq() ;
  return N ;
}


void AudioAnalyzeFFT::copy_to_fft_buffer (int blk, const int16_t * src)
{
  int32_t * dest = &r_buffer[blk << 7] ;
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    *dest++ = (*src++) << 16 ;
}

void AudioAnalyzeFFT::zero_fft_buffer (int blk)
{
  int32_t * dest = &r_buffer[blk << 7] ;
  for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++)
    *dest++ = 0 ;
}

void AudioAnalyzeFFT::apply_window_to_fft_buffer (void)
{
  r_buffer[0] = 0 ;
  for (unsigned int i=1; i < N/2; i++)
  {
    int64_t product = r_buffer[i] ;  // 32 bit signed
    product *= window [i] ;   // time 16 bit signed
    int32_t val = (int32_t) ((product + 0x4000) >> 15) ;  // scale back to 32 bits
    r_buffer[i] = val ;
    
    product = r_buffer[N-i] ;
    product *= window [i] ;
    val = (int32_t) ((product + 0x4000) >> 15) ;
    r_buffer[N-i] = val ;
  }
}

void AudioAnalyzeFFT::update(void)
{
  if (N == 0)  // Npoints() hasn't been called yet...
    return ;
  
  audio_block_t * block = receiveReadOnly();
  if (block == NULL)
    return;
  if (!valid)
  {
    release (block) ;
    return ;
  }

#if defined(__ARM_ARCH_7EM__)
  copy_to_fft_buffer (state, block->data) ;
  if (state < total_blocks - overlap_blocks)
    release (block) ;
  else
    blocklist [state] = block ;
  state ++ ;
  if (state >= total_blocks)
  {
    apply_window_to_fft_buffer ();

    arm_rfft_q31 (&fft_inst, r_buffer, c_buffer);

    for (unsigned int i=0; i <= N/2; i++)
    {
      float real = c_buffer [2*i] ;
      float imag = c_buffer [2*i+1] ;
      float mag = sqrt (real*real + imag*imag) ;
      output[i] = int (round (mag)) ;
    }
    outputflag = true;

    // setup for overlapping with previous:
    for (int i = 0 ; i < overlap_blocks ; i++)
    {
      int ind = i + total_blocks - overlap_blocks ;
      if (blocklist[ind] == NULL)
        zero_fft_buffer (i) ;
      else
        copy_to_fft_buffer (i, blocklist[ind]->data);
      blocklist[i] = blocklist[ind];
    }
    for (int i = 0 ; i < total_blocks - overlap_blocks ; i++)
      if (blocklist [i] != NULL)
        release (blocklist [i]) ;
                
    state = overlap_blocks;
  }
#else
  release(block);
#endif
}
