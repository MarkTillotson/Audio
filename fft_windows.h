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

#ifndef data_windows_h_
#define data_windows_h_

#include <stdint.h>
#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"


enum WindowType
{
  RECTANGULAR_WINDOW = 0,
  COSINE_WINDOW = 1,
  FUNCTION_WINDOW = 2
};


class FFTWindow
{
 public:
  FFTWindow (const char * name) ;
  FFTWindow (const char * name, float32_t (*fn) (float32_t)) ;
  FFTWindow (const char * name, int cosine_order, float64_t * cosine_coeffs) ;

  void expand_float (float32_t * buffer, int N) ;
  void expand_q31 (int32_t * buffer, int N) ;
  void expand_q15 (int16_t * buffer, int N) ;
  float32_t processingGain ();
  float32_t noiseBandwidth () ;
  static void register_fft_window (FFTWindow * window) ;
  static FFTWindow * fft_window (const char * name) ;

 private:
  const char * name ;
  WindowType type ;
  int cosine_order = 0 ;
  float64_t * cosine_coeffs = NULL ; // table of cosine poly coefficients for cosine-based window
  float32_t (*fn) (float32_t) ;  // function for calculating arb window, domain is 0..1
  float32_t processing_gain ;
  float32_t noise_bandwidth ;
  FFTWindow * next = NULL ;
  static FFTWindow * list_fft_windows ;

  float32_t calc (float32_t x) ;
};

extern FFTWindow rect_window ;
extern FFTWindow hann_window ;
extern FFTWindow hamming_window ;
extern FFTWindow blackman_window ;
extern FFTWindow flattop_window ;
extern FFTWindow blackman_harris_window ;
extern FFTWindow nuttall_window ;
extern FFTWindow blackman_nuttall_window ;
extern FFTWindow hft144d_window ;
extern FFTWindow bartlett_window ;
extern FFTWindow welch_window ;
extern FFTWindow cosine_window ;
extern FFTWindow tukey_window ;


#endif
