#include <Audio.h>

//#DEFINE PLOT   // enable this for ASCII graph of dB

AudioSynthWaveformSine tone1 ;
AudioSynthWaveformSine tone2 ;
AudioSynthWaveformSine tone3 ;
AudioSynthWaveformSine tone4 ;
AudioSynthWaveformSine tone5 ;

AudioMixer4            mixer, mixer2 ;
AudioConnection        c1 (tone1, 0, mixer, 0) ;
AudioConnection        c2 (tone2, 0, mixer, 1) ;
AudioConnection        c3 (tone3, 0, mixer, 2) ;
AudioConnection        c4 (tone4, 0, mixer, 3) ;
AudioConnection        c5 (tone5, 0, mixer2, 0) ;

AudioConnection        c6 (mixer, 0, mixer2, 1) ;

AudioAnalyzeFFT1024    exist_fft ;
AudioConnection        c7 (mixer2, exist_fft) ;
AudioAnalyzeFFT        gen_fft ;
AudioConnection        c8 (mixer2, gen_fft) ;

AudioOutputI2S         out ;
AudioConnection        c9 (mixer2, 0, out, 0) ;
AudioConnection        c10 (mixer2, 0, out, 1) ;

AudioControlSGTL5000   shield ;


void setup() 
{
  AudioMemory (40) ;  // plenty for 2 FFT's each using 8 buffers
  
  // test tones - ensure not simple ratios with sampling frequency, and not harmonically related
  tone1.frequency (501) ; tone1.amplitude (1.0) ;  mixer.gain (0, 0.89125) ; // -1dB
  tone2.frequency (1234) ; tone2.amplitude (1.0) ;  mixer.gain (1, 0.1) ;     // -20dB
  tone3.frequency (2203) ; tone3.amplitude (1.0) ;  mixer.gain (2, 0.01) ;    // -40dB
  tone4.frequency (3555) ; tone4.amplitude (1.0) ;  mixer.gain (3, 0.001) ;   // -60dB
  tone5.frequency (4009) ; tone5.amplitude (1.0) ;  mixer2.gain (0, 0.0001) ;  // -80dB  (about 3 LSBs amplitude!)

  mixer2.gain (1, 1) ;  // mixer to mixer2 unity gain

  // setup same number of points for FFTs and same flattop window.
  gen_fft.Npoints (1024) ;
  gen_fft.fftWindow (&flattop_window) ;
  exist_fft.windowFunction (AudioWindowFlattop1024) ;

  //SGTL5000:
  shield.enable ();
  shield.volume (.35);  // not too loud for headphones
  shield.lineOutLevel (13) ; // max line level for 'scope
}

float dB (float val)
{
  if (val == 0.0)
    return -140.0 ;  // avoid NaNs/Infs
  return 20 * log10 (val) ;
}

int freq (int index)
{
  float f = 44100.0 / 1024 * index ;
  return int (round (f)) ;
}

void plot (float val)  // for ASCII plotting
{
  int deebee = 2 - int(round(dB (val))) ;
  for (int i = 0 ; i < deebee ; i++)
    Serial.print (' ') ;
  Serial.println ('*') ;
}

void loop() 
{
  delay (1000) ; // avoid bombing the serial port too heavily
  
  if (gen_fft.available ())
  {
    Serial.println ("Generic AudioAnalyzeFFT with 1024 points:") ;
#ifndef PLOT
    for (int i = 9 ; i < 16 ; i ++)  // around tone1
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (gen_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 26 ; i < 33 ; i ++)  // around tone2
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (gen_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 48 ; i < 55 ; i ++)  // around tone3
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (gen_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 79 ; i < 86 ; i ++)  // around tone4
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (gen_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 90 ; i < 97 ; i ++)  // around tone5
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (gen_fft.read (i)), 7) ;
    }

#else
    for (int i = 0 ; i < 100 ; i++)
      plot (gen_fft.read (i)) ;
#endif
    Serial.println () ;
    Serial.println () ;
  }
  
  if (exist_fft.available ())
  {
    Serial.println ("AudioAnalyzeFFT1024:") ;
#ifndef PLOT
    for (int i = 9 ; i < 16 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 7) ;
    }  
    Serial.println () ;
    for (int i = 26 ; i < 33 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 7) ;
    }  
    Serial.println () ;
    for (int i = 48 ; i < 55 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 79 ; i < 86 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 7) ;
    }
    Serial.println () ;
    for (int i = 90 ; i < 97 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 7) ;
    }

#else
    for (int i = 0 ; i < 100 ; i++)
      plot (exist_fft.read (i)) ;
#endif
    Serial.println () ;
    Serial.println () ;
  }
}
