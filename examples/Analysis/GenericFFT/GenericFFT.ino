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
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dB (i), 3) ;
    }
    Serial.println () ;
    for (int i = 26 ; i < 33 ; i ++)  // around tone2
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dB (i), 3) ;
    }
    Serial.println () ;
    for (int i = 48 ; i < 55 ; i ++)  // around tone3
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dB (i), 3) ;
    }
    Serial.println () ;
    for (int i = 79 ; i < 86 ; i ++)  // around tone4
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dB (i), 3) ;
    }
    Serial.println () ;
    for (int i = 90 ; i < 97 ; i ++)  // around tone5
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dB (i), 3) ;
    }
    Serial.println ("noise floor:") ;
    for (int i = 150 ; i < 165 ; i ++)  // quiet part of spectrum to see noise floor
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (gen_fft.dBNoise (i), 3) ;
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
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }  
    Serial.println () ;
    for (int i = 26 ; i < 33 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }  
    Serial.println () ;
    for (int i = 48 ; i < 55 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }
    Serial.println () ;
    for (int i = 79 ; i < 86 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }
    Serial.println () ;
    for (int i = 90 ; i < 97 ; i ++)
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }
    Serial.println ("noise floor, as power spectrum:") ;
    for (int i = 150 ; i < 165 ; i ++)  // quiet part of spectrum to see noise floor
    {
      Serial.print (freq(i)) ; Serial.print ("Hz ") ; Serial.println (dB (exist_fft.read (i)), 3) ;
    }

#else
    for (int i = 0 ; i < 100 ; i++)
      plot (exist_fft.read (i)) ;
#endif
    Serial.println () ;
    Serial.println () ;
  }
}

/* example output from T4.0

AudioAnalyzeFFT1024:
388Hz -23.709
431Hz -16.219
474Hz -14.389
517Hz -14.325    // -1dB -13.3dB due to processing gain of unnormalized flattop window
560Hz -15.345
603Hz -20.942
646Hz -35.011

1120Hz -43.150
1163Hz -35.253
1206Hz -33.457
1249Hz -33.308   // -20dB -13.3dB due to processing gain of unnormalized flattop window
1292Hz -34.378
1335Hz -39.628
1378Hz -54.185

2067Hz -78.268
2110Hz -57.440
2153Hz -54.746
2196Hz -52.924   // -39.6dB -13.3dB due to processing gain of unnormalized flattop window
2239Hz -54.185
2283Hz -55.345
2326Hz -68.725

3402Hz -84.288
3445Hz -78.268
3488Hz -74.746
3531Hz -72.247   // -59dB -13.3dB due to processing gain of unnormalized flattop window
3575Hz -74.746
3618Hz -72.247
3661Hz -78.268

3876Hz -84.288
3919Hz -84.288
3962Hz inf
4005Hz -84.288   // lost in noise floor
4048Hz -84.288
4091Hz -84.288
4134Hz -84.288
noise floor, as power spectrum:
6460Hz -84.288
6503Hz -84.288
6546Hz -84.288   // numerical error leads to way higher noise floor than reality, should be -141dB about
6589Hz -84.288
6632Hz inf
6675Hz -84.288
6718Hz -84.288
6761Hz inf
6804Hz inf
6848Hz -84.288
6891Hz -84.288
6934Hz inf
6977Hz -84.288
7020Hz -78.268
7063Hz -84.288


Generic AudioAnalyzeFFT with 1024 points:
388Hz -10.383
431Hz -2.902
474Hz -1.048
517Hz -1.004   // -1dB
560Hz -2.007
603Hz -7.646
646Hz -21.647

1120Hz -29.619
1163Hz -21.985
1206Hz -20.056
1249Hz -20.003   // -20dB
1292Hz -20.954
1335Hz -26.462
1378Hz -40.248

2067Hz -56.707
2110Hz -44.880
2153Hz -40.539
2196Hz -40.003   // -40dB
2239Hz -40.163
2283Hz -42.910
2326Hz -52.042

3402Hz -84.396
3445Hz -68.562
3488Hz -61.689
3531Hz -60.171   // -60.2 dB (amplitude of signal only 32LSBs, so slight amplitude error expected)
3575Hz -60.157
3618Hz -61.401
3661Hz -67.560

3876Hz -97.379
3919Hz -86.388
3962Hz -82.056
4005Hz -81.569   // -81.5 dB (amplitude of signal only 3LSBs, so significant amplitude error expected)
4048Hz -81.624
4091Hz -85.164
4134Hz -95.263
noise floor:
6460Hz -134.117
6503Hz -127.824  // ideally noise floor would be about -141dB for 16 bits and 22.050kHz bandwidth.
6546Hz -123.631  // still this is about 45dB better than AudioAnalyzeFFT1024 for same data...
6589Hz -127.652
6632Hz -129.584
6675Hz -137.534
6718Hz -132.302
6761Hz -143.825
6804Hz -139.573
6848Hz -132.996
6891Hz -136.070
6934Hz -131.536
6977Hz -131.404
7020Hz -131.876
7063Hz -132.146

*/
