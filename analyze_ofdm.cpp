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

#include <Arduino.h>
#include "synth_ofdm.h"

#define SQRTH 0.70710678
// static int   graycode   [4]  = { 0, 1, 3, 2 } ;
// cosines of the 8th roots of unity, plus first two repeated
//static float phasetable [10] = { 1.0, SQRTH, 0.0, -SQRTH, -1.0, -SQRTH, 0.0, SQRTH, 1.0, SQRTH } ;


static uint16_t golay_decode (uint32_t codeword)
{
  return (codeword >> 11) & 0xFFF ; /// no error correction!
}


void AudioAnalyzeOFDM::demodulate (void)
{
  // FFT
  arm_cfft_radix4_f32 (&cfft_inst, (float *) samples);
  

  float pilot_ph ;
  if (first_block)
  {
    for (unsigned int i = OFDM_CHANNEL_MIN ; i <= OFDM_CHANNEL_MAX ; i++)
    {
      float real = samples [i].real ;
      float imag = samples [i].imag ;

      qam_maps [i].real = real ;
      qam_maps [i].imag = imag ;

      float phase = atan2 (imag, real) ;
      phases [i] = phase ;
    }
    first_block = false ;
  }
  else
  {
    int vecind = 0 ;
    
    int base_chan = OFDM_CHANNEL_MIN ;
    for (unsigned int i = 0 ; i < CHUNK_SIZE ; i += 2)
    {
      uint32_t gol0 = 0;
      uint32_t gol1 = 0;
      
      int chan = base_chan ;
      for (unsigned int j = 0 ; j < GOLBITS_OUT ; j++)
      {
	float real = samples [chan].real ;
	float imag = samples [chan].imag ;
	qam_maps [chan].real = real ;
	qam_maps [chan].imag = imag ;
	float phase = atan2 (imag, real) ;

	float phase_diff = phase - phases [chan] ;
	phase_diff = - phase_diff ;
	phases [chan] = phase ;
	while (phase_diff < -M_PI) phase_diff += 2*M_PI ;
	while (phase_diff >= M_PI) phase_diff -= 2*M_PI ;

	byte bits ;
	if (phase_diff >= 0)
	  bits = phase_diff > M_PI/2 ? 2 : 3 ;
	else
	  bits = phase_diff < -M_PI/2 ? 0 : 1 ;

	gol0 |= (bits & 1) << 23 ;
	gol1 |= (bits & 2) << 22 ;
	gol0 >>= 1 ;
	gol1 >>= 1 ;
	chan += CHANNELS_PER_CHUNK ;  // 2 for pilot tones
      }

      uint16_t plain0 = golay_decode (gol0) ;
      uint16_t plain1 = golay_decode (gol1) ;

      data_vector [vecind++] = plain0 & 0xFF ;
      data_vector [vecind++] = ((plain0 >> 8) & 0xF) | ((plain1 << 4) & 0xF0) ;
      data_vector [vecind++] = (plain1 >> 4) & 0xFF ;

      base_chan ++ ;
    }
  }
  
  int pilot_ind = 0 ;
  // deal with pilot tones
  for (unsigned int pilot = OFDM_CHANNEL_MIN + CHUNK_SIZE/2 ;
       pilot <= OFDM_CHANNEL_MAX ;
       pilot += CHANNELS_PER_CHUNK)
  {
    float real = samples [pilot].real ;
    float imag = samples [pilot].imag ;
    qam_maps [pilot].real = real ;
    qam_maps [pilot].imag = imag ;
    float phase = atan2 (imag, real) ;
    pilot_ph = phase ;

    real = samples [pilot+1].real ;
    imag = samples [pilot+1].imag ;
    qam_maps [pilot+1].real = real ;
    qam_maps [pilot+1].imag = imag ;
    phase = atan2 (imag, real) ;

    float diff = phase - pilot_ph ;
    if (diff >= M_PI)  diff -= 2*M_PI ;	
    if (diff < -M_PI)  diff += 2*M_PI ;	
    pilot_pdiff[pilot_ind++] = diff ;
  }
  part_index = 0 ;
}


void AudioAnalyzeOFDM::update(void)
{
  total_count ++ ;
  audio_block_t * block = receiveReadOnly (0) ;
  if (block == NULL)
    return;

  int16_t * p = block->data ;
  int16_t * end = p + AUDIO_BLOCK_SAMPLES;
  //unsigned int index = AUDIO_BLOCK_SAMPLES * segment ;  // index into the ifft output

  /*
  // fist cut of synchronization, do rms calcs over 16 sample blocks, sum over 32 sample blocks, take log
  // and stick out on the test stream
  audio_block_t * rms = allocate() ;
  if (rms != NULL)
  {
    int16_t * rmsdata = rms->data ;
    for (unsigned int i = 0 ; i < AUDIO_BLOCK_SAMPLES>>4 ; i++)
    {
      uint32_t sum = 0 ;
      for (unsigned int j = 0 ; j < 16 ; j++)
      {
	int32_t sqr = *p++ ;
	sqr *= sqr ;
	sum += (sqr >> 4) ;
      }
      sq_sums [i] = sum ;
      sum = 0 ;
      for (unsigned int j = 0 ; j < 2 ; j++)
	sum += sq_sums [(64 + i - j) % (AUDIO_BLOCK_SAMPLES>>4)] ;
      int16_t sqmag = (int16_t) (sum >> 15) ;
      sqmag = int (log (sqmag+1) * 3000) ;

      for (unsigned int j = 0 ; j < 16 ; j+=2)
      {
	*rmsdata++ = sqmag ;
	*rmsdata++ = -sqmag ;
      }
    }
    transmit (rms, 0) ;
    release (rms) ;
    p = block->data ;
  }
  */


  // output quadrature of frequencies to two channels for debug
  audio_block_t * ipart = allocate() ;
  audio_block_t * qpart = allocate() ;
  if (ipart != NULL && qpart != NULL)
  {
    int16_t * pi = ipart->data ;
    int16_t * pq = qpart->data ;
    int16_t * end2 = pi + AUDIO_BLOCK_SAMPLES ;
    do
    {
      int16_t re = int (round (2700 * qam_maps[part_index].real)) ;
      int16_t im = int (round (3000 * qam_maps[part_index].imag)) ;
      *pi++ = re ;
      *pq++ = im ;
      *pi++ = re ;
      *pq++ = im ;

      part_index ++ ;
    } while (pi < end2) ;
    transmit (ipart, 0) ;
    transmit (qpart, 1) ;
  }
  if (ipart != NULL) release (ipart) ;
  if (qpart != NULL) release (qpart) ;


  if (index >= OFDM_FFTN)
  {
    demodulate () ;
    listener (data_vector) ;
    index = 0 ;
  }

  do
  {
    samples[index].real = (*p++) / 32767.0 ;
    samples[index].imag = 0.0 ;

    index ++ ;
    if (index >= OFDM_FFTN)
    {
      demodulate () ;
      listener (data_vector) ;
      index = 0 ;
    }
  } while (p < end) ;


  for (unsigned int i = 0 ; i <= OFDM_CHANNELS/64 ; i++)
    pilot_pdiffavg[i] += 0.02 * (pilot_pdiff[i] - pilot_pdiffavg[i]) ;

  /*
  if ((total_count & 0x7f) == 0x7f)
  {
    shifted += 4 ; shifted &= OFDM_FFTMSK ;
    //Serial.printf ("- %x -\n", shifted) ;
    index += 4 ; index &= OFDM_FFTMSK ;
  }

  if ((total_count & 0x7ff) == 0)
  {
    shifted += AUDIO_BLOCK_SAMPLES ;
    Serial.printf ("- %x -\n", shifted) ;
    index = index+AUDIO_BLOCK_SAMPLES ;
  }
  */

  release (block) ;
}
