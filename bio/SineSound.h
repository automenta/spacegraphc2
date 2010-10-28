/* 
 * File:   SineSound.h
 * Author: seh
 *
 * Created on February 16, 2010, 1:36 AM
 */

#ifndef _SINESOUND_H
#define	_SINESOUND_H

#include <stdio.h>

//#include "../audio/Audio.h"
//#include "../audio/SoundSource.h"
//#include "../neural/NOutput.h"
//
//static double FREQ_RANGE = 1024.0;
//
//class SineSound : public NOutput, SoundSource {
//    Audio* audio;
//
//public:
//    double zeroFreq;
//
//    double freq, amp;
//    unsigned sample;
//    double zeroAmp;
//
//
//    SineSound(Brain* b, Audio *_a, double _baseFreq) : NOutput(b, 2), audio(_a), zeroFreq(_baseFreq) {
//        audio->sources->push_back(this);
//        sample = 0;
//        zeroAmp = 0.25;
//    }
//
//    virtual void process(double dt) {
//        float v = outs[0]->getOutput();
//        float a = outs[1]->getOutput();
//
//        //freq = zeroFreq + FREQ_RANGE * (v + 1.0)*0.5;
//        freq = zeroFreq;
//        amp = zeroAmp + fabs(a)/0.5;
//
//        //printf("frequency: %f %f\n", v, pg->frequency);
//    }
//
//    virtual double nextSample(int sampleRate) {
//        return amp * sin(freq/double(sampleRate) * ((double)sample++));
//    }
//
//    virtual ~SineSound() {
//        audio->sources->remove(this);
//    }
//
//private:
//
//};

#endif	/* _SINESOUND_H */

