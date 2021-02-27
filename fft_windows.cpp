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

#include "fft_windows.h"


FFTWindow::FFTWindow (const char * _name)
{
  name = _name ;
  type = RECTANGULAR_WINDOW ;
}

FFTWindow::FFTWindow (const char * _name, float32_t (*_fn) (float32_t))
{
  name = _name ;
  fn = _fn ;
  type = FUNCTION_WINDOW ;
}

FFTWindow::FFTWindow (const char * _name, int _cosine_order, float64_t * _cosine_coeffs)
{
  name = _name ;
  cosine_order = _cosine_order ;
  cosine_coeffs = _cosine_coeffs ;
  type = COSINE_WINDOW ;
}


float32_t FFTWindow::calc (float32_t x)
{
  switch (type)
  {
  case RECTANGULAR_WINDOW:
    return 1.0 ;

  case COSINE_WINDOW:
    {
      float64_t val = cosine_coeffs[0] ;
      for (int i = 1 ; i < cosine_order ; i++)
        val += cosine_coeffs[i] * cos (2*M_PI * i*x) ;
      return (float32_t) val ;
    }

  case FUNCTION_WINDOW:
    return (*fn) (x) ;

  default:
    return 0.0 ;
  }
}

float32_t FFTWindow::processingGain ()
{
  return processing_gain ;
}

float32_t FFTWindow::noiseBandwidth ()
{
  return noise_bandwidth ;
}

// expand as single floats into buffer - note buffer is half N in size as symmetry is exploited
void FFTWindow::expand_float (float32_t * buffer, int N)
{
  float32_t val = calc (0.0) ;
  float32_t sum = val ;
  float32_t sumsq = val*val ;
  buffer[0] = val ;

  for (int i = 1 ; i < N/2 ; i++)
  {
    val = calc ((float32_t)i / N) ;
    sum += 2*val ;
    sumsq += 2*val*val ;
    buffer[i] = val ;
  }

  val = calc (0.5) ;
  sum += val ;
  sumsq += val*val ;
  buffer[N/2] = val ;

  processing_gain = sum / N ;
  noise_bandwidth = sumsq * N / (sum * sum) ;
}

void FFTWindow::expand_q31 (int32_t * buffer, int N)
{
  float32_t max = calc (0.5) ;
  float32_t sum = 1.0 ;
  float32_t sumsq = 1.0 ;

  float32_t val = calc (0.0) / max ;
  sum += val ;
  sumsq += val*val ;
  buffer [0] = (int32_t) int (round (val * 0x7fffffff)) ;

  for (int i = 1 ; i < N/2 ; i++)
  {
    float32_t val = calc ((float32_t)i / N) / max ;
    sum += 2*val ;
    sumsq += 2*val*val ;
    int32_t ival = (int32_t) int (round (val * 0x7fffffff)) ;
    buffer[i] = ival ;
  }

  buffer[N/2] = 0x7fffffff ;

  processing_gain = sum / N ;
  noise_bandwidth = sumsq * N / (sum * sum) ;
}

void FFTWindow::expand_q15 (int16_t * buffer, int N)
{
  float32_t max = calc (0.5) ;
  float32_t sum = 1.0 ;
  float32_t sumsq = 1.0 ;
  
  float32_t val = calc (0.0) / max ;
  sum += val ;
  sumsq += val*val ;
  buffer [0] = (int16_t) int (round (val * 0x7fff)) ;
  
  for (int i = 1 ; i < N/2 ; i++)
  {
    float32_t val = calc ((float32_t)i / N) / max ;
    sum += 2*val ;
    sumsq += 2*val*val ;
    int16_t ival = (int16_t) int (round (val * 0x7fff)) ;
    buffer[i] = ival ;
  }

  buffer[N/2] = 0x7fff ;

  processing_gain = sum / N ;
  noise_bandwidth = sumsq * N / (sum * sum) ;
}

float64_t window_coeffs[] =
{
  // 0
  1.0, -1.0,    // Hann = 1 - cos(a)
  // 2
  0.54, -0.46,  // Hamming
  // 4
  0.42, -0.5, 0.08, // Blackman
  // 7
  1.00, -1.93, 1.29, -0.388, 0.028,  // Flattop
  // 12
  0.35875, -0.48829, 0.14128, -0.01168, // BlackmanHarris
  // 16
  0.355768, -0.487396, 0.144232, -0.012604, // Nuttall
  // 20
  0.3635819, -0.4891775, 0.1365995, -0.0106411,  // BlackmanNuttal
  // 24
  1.00000000, -1.96760033, 1.57983607, -0.81123644, 0.22583558, -0.02773848, 0.00090360, // HFT144D flattop
  // 31
} ;

FFTWindow rect_window ("Rect") ;
FFTWindow hann_window ("Hann", 2, window_coeffs+0) ;
FFTWindow hamming_window ("Hamming", 2, window_coeffs+2) ;
FFTWindow blackman_window ("Blackman", 3, window_coeffs+4) ;
FFTWindow flattop_window ("Flattop", 5, window_coeffs+7) ;
FFTWindow blackman_harris_window ("BlackmanHarris", 4, window_coeffs+12) ;
FFTWindow nuttall_window ("Nuttall", 4, window_coeffs+16) ;
FFTWindow blackman_nuttall_window ("BlackmanNuttall", 4, window_coeffs+20) ;
FFTWindow hft144d_window ("HFT144D", 7, window_coeffs+24) ;

float32_t bartlett_fn (float32_t x) { return 1.0 - abs (2*x-1) ; }

float32_t welch_fn (float32_t x)
{
  float32_t p = (x-0.5) / 0.5 ;
  return 1.0 - p*p ;
}

float32_t cosine_fn (float32_t x) { return cos (M_PI * (x - 0.5)) ; }

float32_t tukey_fn (float32_t x)
{
  if (x < 0.25 || x > 0.75)
    return 0.5 * (1.0 - cos (4*M_PI * x)) ;
  else
    return 1.0 ;
}


FFTWindow bartlett_window ("Bartlett", bartlett_fn) ;
FFTWindow welch_window ("Welch", welch_fn) ;
FFTWindow cosine_window ("Cosine", cosine_fn) ;
FFTWindow tukey_window ("Tukey", tukey_fn) ;


FFTWindow * FFTWindow::list_fft_windows = NULL ;

void FFTWindow::register_fft_window (FFTWindow * window)
{
  window->next = list_fft_windows ;
  list_fft_windows = window ;
}

void register_all_windows ()
{
  FFTWindow::register_fft_window (&rect_window) ;
  FFTWindow::register_fft_window (&hann_window) ;
  FFTWindow::register_fft_window (&hamming_window) ;
  FFTWindow::register_fft_window (&blackman_window) ;
  FFTWindow::register_fft_window (&flattop_window) ;
  FFTWindow::register_fft_window (&blackman_harris_window) ;
  FFTWindow::register_fft_window (&nuttall_window) ;
  FFTWindow::register_fft_window (&blackman_nuttall_window) ;
  FFTWindow::register_fft_window (&bartlett_window) ;
  FFTWindow::register_fft_window (&welch_window) ;
  FFTWindow::register_fft_window (&cosine_window) ;
  FFTWindow::register_fft_window (&tukey_window) ;
  FFTWindow::register_fft_window (&hft144d_window) ;
}

FFTWindow * FFTWindow::fft_window (const char * name)
{
  if (list_fft_windows == NULL)
    register_all_windows () ;
  for (FFTWindow * entry = list_fft_windows ; entry != NULL ; entry = entry->next)
  {
    Serial.printf ("compare %s to %s\n", name, entry->name) ;
    if (strcmp (entry->name, name) == 0)
      return entry ;
  }
  return NULL ;
}
