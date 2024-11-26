#ifndef PLAY_AUDIO_H
#define PLAY_AUDIO_H

#include <Arduino.h>
#include "driver/ledc.h"

class PlayAudio {
private:
    const int audioPin;
    const int sampleRate;

public:
    PlayAudio(int pin = 14, int rate = 8000);
    void begin();
    void playWaveform();
};

#endif 