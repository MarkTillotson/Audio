#include <dspinst.h>
#include <math.h>
#include "synth_waveguide.h"

#define AMP 0x10000000
#define ONE 0x80000000

void AudioSynthWaveguideSine::frequency (float f)
{
  if (f < 0)     f = 0 ;
  if (f > 20000) f = 20000 ;

  float scale = AMP * amplitud ;
  float new_angle = 2 * M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  float new_Cn = cos (new_angle) ;
  float new_k = 2 / (1 - new_Cn) - 1 ;
  int32_t nCn = (int32_t) round (ONE * new_Cn) ;

  // make phase-continuous (assuming amplitude unchanged)
  __disable_irq() ;
  float temp = (float)d1 / scale ;
  temp = 1.0 - temp * temp ;
  if (temp < 0.0)
    temp = 0.0 ;
  temp = sqrt (temp / new_k) ;

  if (d0 < 0)
    d0 = - int (round (scale * temp)) ;
  else
    d0 = int (round (scale * temp)) ;
  freq = f ;
  Cn = nCn ;
  __enable_irq() ;
}

void AudioSynthWaveguideSine::amplitude (float amp)
{
  if (amp < 0.0) amp = 0.0 ;
  if (amp > 0.995) amp = 0.995 ;

  float f_Cn = cos (2 * M_PI * freq / AUDIO_SAMPLE_RATE_EXACT) ;
  float k = 2 / (1 - f_Cn) - 1 ;
  float scale = AMP * amp ;

  __disable_irq() ;
  if (amplitud == 0)
  {
    d0 = 0 ;
    d1 = int (round (scale)) ;
  }
  else
  {
    d0 = int (round (d0 * amp / amplitud)) ;
    d1 = int (round (d1 * amp / amplitud)) ;
    float energy = (float)d1*d1 + k*d0*d0 ;
    energy /= scale*scale ;
    float correction = 1.0 / energy ;
    d0 = int (round (d0 * correction)) ;
    d1 = int (round (d1 * correction)) ;
  }
  amplitud = amp ;
  __enable_irq() ;
}

void AudioSynthWaveguideSine::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  bool cos_positive = cos (phase) >= 0 ;
  float sine = sin (phase) ;
  float f_Cn = cos (2 * M_PI * freq / AUDIO_SAMPLE_RATE_EXACT) ;
  float k = 2 / (1 - f_Cn) - 1 ;
  float scale = AMP * amplitud ;

  float temp = sqrt ((1.0 - sine*sine) / k) ;

  int32_t nd1 = int (round (scale * sine)) ;
  int32_t nd0 ;
  if (cos_positive)
    nd0 = int (round (scale * temp)) ;
  else
    nd0 = -int (round (scale * temp)) ;
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

  if (Cn < 0)
  {
    for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i += 2)
    {
      *data++ = (int16_t) ((d1+0x1000) >> 13) ;
      int32_t c = multiply_32x32_rshift32 (d0+d1, -Cn) << 1 ;  // don't need the accumulate, want the rounding
      int32_t nd0 = -c - d1 ;
      int32_t nd1 = -c + d0 ;

      *data++ = (int16_t) ((nd1+0x1000) >> 13) ;
      c = multiply_32x32_rshift32 (nd0+nd1, -Cn) << 1 ;  // don't need the accumulate, want the rounding
      d0 = -c - nd1 ;
      d1 = -c + nd0 ;
    }
  }
  else
  {
    for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i += 2)
    {
      *data++ = (int16_t) ((d1+0x1000) >> 13) ;
      int32_t c = multiply_32x32_rshift32 (d0+d1, Cn) << 1 ;  // don't need the accumulate, want the rounding
      int32_t nd0 = c - d1 ;
      int32_t nd1 = c + d0 ;

      *data++ = (int16_t) ((nd1+0x1000) >> 13) ;
      c = multiply_32x32_rshift32 (nd0+nd1, Cn) << 1 ;  // don't need the accumulate, want the rounding
      d0 = c - nd1 ;
      d1 = c + nd0 ;
    }
  }

  transmit (block) ;
  release (block) ;
}

///////////////////////////////////////////////

void AudioSynthWaveguideSineFloat::frequency (float f)
{
  if (f < 0)     f = 0 ;
  if (f > 20000) f = 20000 ;

  float new_angle = 2 * M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  float new_Cn = cos (new_angle) ;
  float new_k = 2 / (1 - new_Cn) - 1 ;

  // make phase-continuous (assuming amplitude unchanged)
  __disable_irq() ;
  float temp = 1.0 - d1 * d1 ;
  if (temp < 0.0)
    temp = 0.0 ;
  temp = sqrt (temp / new_k) ;

  d0 = d0 < 0 ? - temp : temp ;
  freq = f ;
  Cn = new_Cn ;
  __enable_irq() ;
}

void AudioSynthWaveguideSineFloat::amplitude (float amp)
{
  if (amp < 0.0) amp = 0.0 ;
  if (amp > 0.995) amp = 0.995 ;
  __disable_irq() ;
  amplitud = 32768 * amp ;
  __enable_irq() ;
}

void AudioSynthWaveguideSineFloat::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  bool cos_positive = cos (phase) >= 0 ;
  float sine = sin (phase) ;
  float f_Cn = cos (2 * M_PI * freq / AUDIO_SAMPLE_RATE_EXACT) ;
  float k = 2 / (1 - f_Cn) - 1 ;

  float temp = sqrt ((1.0 - sine*sine) / k) ;

  __disable_irq() ;
  d1 = sine ;
  d0 = cos_positive ? temp : -temp ;
  __enable_irq() ;
}


void AudioSynthWaveguideSineFloat::update ()
{
  audio_block_t * block = allocate() ;
  if (block == NULL)
    return ;

  int16_t * data = block->data ;

  float pd0 = d0 ;
  float pd1 = d1 ;
  float amp = amplitud ;
  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i += 2)
  {
    *data++ = (pd1 * amp) ;
    float c = (pd0+pd1) * Cn ;
    float nd0 = c - pd1 ;
    float nd1 = c + pd0 ;

    *data++ = (nd1 * amp) ;
    c = (nd0+nd1) * Cn ;
    pd0 = c - nd1 ;
    pd1 = c + nd0 ;
  }
  d0 = pd0 ;
  d1 = pd1 ;

  transmit (block) ;
  release (block) ;
}

////////////////////////////////////////////////////////////////

void AudioSynthCoupledSine::frequency (float f)
{
  if (f < 0)     f = 0 ;
  if (f > 20000) f = 20000 ;

  float new_hw = M_PI * f / AUDIO_SAMPLE_RATE_EXACT ;
  int32_t new_e = int (float (ONE * sin (new_hw))) ;
  // make phase-continuous (assuming amplitude unchanged)
  __disable_irq() ;
  freq = f ;
  halfomega = new_hw ;
  e = new_e ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::amplitude (float amp)
{
  if (amp < 0.0) amp = 0.0 ;
  if (amp > 0.999) amp = 0.999 ;

  float scale = AMP * amp ;

  __disable_irq() ;
  if (amplitud == 0)
  {
    x = 0 ;
    y = cos (halfomega) ;
  }
  else
  {
    x = int (round (x * amp / amplitud)) ;
    y = int (round (y * amp / amplitud)) ;
    float energy = (float)x*x + (float)y*y ;
    energy /= scale*scale ;
    float correction = 1.0 / energy ;
    x = int (round (x * correction)) ;
    y = int (round (y * correction)) ;
  }
  amplitud = amp ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::phase (float degrees)
{
  float phase = degrees * M_PI / 180 ;
  float xx = cos (phase) ;
  float yy = sin (phase + halfomega) ;

  float scale = AMP * amplitud ;

  __disable_irq() ;
  x = int (round (scale * xx)) ;
  y = int (round (scale * yy)) ;
  __enable_irq() ;
}

void AudioSynthCoupledSine::update ()
{
  audio_block_t * block = allocate() ;
  if (block == NULL)
    return ;

  int16_t * data = block->data ;

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    *data++ = (int16_t) ((x+0x1000) >> 13) ;
    x -= multiply_32x32_rshift32 (e, y) << 2 ;
    y += multiply_32x32_rshift32 (e, x) << 2 ;
  }

  transmit (block) ;
  release (block) ;
}
