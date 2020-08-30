#ifndef __SYNTH_COUPLED_RESONATOR_H__
#define __SYNTH_COUPLED_RESONATOR_H__

#include <Arduino.h>
#include <AudioStream.h>


/*
 * Coupled-form resonator using the iteration
 *  x -= e * y ;
 *  y += e * x ;
 *  where e = 2 * sin (w/2),  w = 2*pi * f / Fs  (f = frequency, Fs = sample rate)
 *
 *  https://ccrma.stanford.edu/~jos/pdf/GordonAndSmith86.pdf
 *
 *  if x is defined to have phase 0,
 *  y has phase -(pi/2 - w/2)
 *
 *  A quadrature pair can be obtained thus:
 *  y2 =  (y - x * sin(w/2)) / cos(w/2)
 *  (x, y2) are in quadrature so can measure amplitude and phase from them, useful for phase-continuous frequency changes
 */

class AudioSynthCoupledSine : public AudioStream
{
public:
  AudioSynthCoupledSine (void) : AudioStream(0, NULL)
  {
    amplitud = 0 ;
    frequency (1000) ;
  }

  void frequency (float Hz) ;
  void amplitude (float amp) ;
  void phase (float degrees) ;

  virtual void update (void) ;

private:
  void set_phase (float phase) ;
  
  float amplitud ;
  float halfomega ;
  int32_t x, y, e ;
};

#endif
