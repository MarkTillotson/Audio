#ifndef __SYNTH_WAVEGUIDE_H__
#define __SYNTH_WAVEGUIDE_H__

#include <Arduino.h>
#include <AudioStream.h>

class AudioSynthWaveguideSine : public AudioStream
{
public:
  AudioSynthWaveguideSine (void) : AudioStream(0, NULL)
  {
    frequency (1000) ;
    phase (0) ;
  }

  void frequency (float Hz) ;
  void phase (float degrees) ;

  virtual void update (void) ;

private:
  float freq ;
  int32_t Cn ;
  int32_t d0, d1 ;
};

#endif
