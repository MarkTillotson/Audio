#ifndef __SYNTH_WAVEGUIDE_H__
#define __SYNTH_WAVEGUIDE_H__

#include <Arduino.h>
#include <AudioStream.h>

class AudioSynthWaveguideSine : public AudioStream
{
public:
  AudioSynthWaveguideSine (void) : AudioStream(0, NULL)
  {
    amplitud = 1.0 ;
    frequency (1000) ;
    phase (0) ;
  }

  void frequency (float Hz) ;
  void amplitude (float amp) ;
  void phase (float degrees) ;

  virtual void update (void) ;

private:
  float freq ;
  float amplitud ;
  int32_t Cn ;
  int32_t d0, d1 ;
};


class AudioSynthWaveguideSineFloat : public AudioStream
{
public:
  AudioSynthWaveguideSineFloat (void) : AudioStream(0, NULL)
  {
    amplitud = 0.995 * 32768 ;
    frequency (1000) ;
    phase (0) ;
  }

  void frequency (float Hz) ;
  void amplitude (float amp) ;
  void phase (float degrees) ;

  virtual void update (void) ;

private:
  float freq ;
  float amplitud ;
  float Cn ;
  float d0, d1 ;
};

#endif
