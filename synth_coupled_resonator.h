#ifndef __SYNTH_COUPLED_RESONATOR_H__
#define __SYNTH_COUPLED_RESONATOR_H__

#include <Arduino.h>
#include <AudioStream.h>


/* Coupled-form resonator sinusoid generate
 *
 * Copyright (c) 2020, Mark Tillotson
 *
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
