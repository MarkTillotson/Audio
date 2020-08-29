#include <dspinst.h>
#include <math.h>
#include "synth_waveguide.h"

#define ONE 0x7F000000

void AudioSynthWaveguideSine::frequency (float f)
{
  float new_angle = 2 * M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  float new_Cn = cos (new_angle) ;
  float new_k = 2 / (1 - new_Cn) - 1 ;

  // make phase-continuous:
  float temp = d1 / ONE ;
  temp = sqrt ((1 - temp*temp) / new_k) ;

  int32_t nd0 ;
  if (d0 < 0)
    nd0 = - int (round (ONE * temp)) ;
  else
    nd0 = int (round (ONE * temp)) ;
  int32_t nCn = (int32_t) round (ONE * new_Cn) ;
  __disable_irq() ;
  freq = f ;
  d0 = nd0 ;
  Cn = nCn ;
  __enable_irq() ;
}

void AudioSynthWaveguideSine::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  bool cos_positive = cos (phase) >= 0 ;
  float sine = sin (phase) ;
  float f_Cn = cos (2 * M_PI * freq / AUDIO_SAMPLE_RATE_EXACT) ;
  float k = 2 / (1 - f_Cn) - 1 ;

  float temp = sqrt ((1.0 - sine*sine) / k) ;

  int32_t nd1 = int (round (ONE * sine)) ;
  int32_t nd0 ;
  if (cos_positive)
    nd0 = int (round (ONE * temp)) ;
  else
    nd0 = -int (round (ONE * temp)) ;
  __disable_irq() ;
  d1 = nd1 ;
  d0 = nd0 ;
  __enable_irq() ;
}


void AudioSynthWaveguideSine::update ()
{
  audio_block_t * block = allocate() ;
  if (block == NULL)
    return ;

  int16_t * data = block->data ;
  
  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i += 2)
  {
    *data++ = (int16_t) (d1 >> 15) ;
    int32_t c = multiply_32x32_rshift32 (d0+d1, Cn) << 1 ;  // don't need the accumulate, want the rounding
    int32_t nd0 = c - d1 ;
    int32_t nd1 = c + d0 ;

    *data++ = (int16_t) (nd1 >> 15) ;
    c = multiply_32x32_rshift32 (nd0+nd1, Cn) << 1 ;  // don't need the accumulate, want the rounding
    d0 = c - nd1 ;
    d1 = c + nd0 ;
  }

  transmit (block) ;
  release (block) ;
}
