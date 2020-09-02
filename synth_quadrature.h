#ifndef __SYNTH_COUPLED_RESONATOR_H__
#define __SYNTH_COUPLED_RESONATOR_H__

#include <Arduino.h>
#include <AudioStream.h>


/* Full quadrature generator
 *
 * Outputs quadrature on two output streams, cosine and sine respectively
 * using method described below.
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
 * Complex resonator using the iteration
 *  x -= k1 * y ;
 *  y += k2 * x ;
 *  x -= k1 * y ;
 *  where k1 = tan(w/2), k2 = sin(w),  w = 2*pi * f / Fs  (f = frequency, Fs = sample rate)
 *
 *  https://vicanek.de/articles/QuadOsc.pdf
 *
 *  (x, y) are in exact quadrature.
 *  an agc is implemented to stabilize against drift in amplitude.  The agc is applied once per
 *  block to reduce overhead.
 */


class AudioSynthQuadrature : public AudioStream
{
public:
  AudioSynthQuadrature (void) : AudioStream(0, NULL)
  {
    amplitud = 0 ;
    frequency (1000) ;
  }

  void frequency (float Hz) ;
  void amplitude (float amp) ;
  void phase (float degrees) ;

  virtual void update (void) ;

private:
  void agc (void) ;
  void set_phase (float phase) ;
  
  float amplitud ;
  int32_t x, y, d, k1, k2 ;
};

#endif
