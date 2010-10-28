/* 
 * File:   NColor.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:26 PM
 */

#ifndef _NCOLOR_H
#define	_NCOLOR_H

#include <btBulletDynamicsCommon.h>
#include <NOutput.h>

/** neural color output: takes 3 outputs and produces R,G,B or H,S,B color that can be used for anything */
class NColor : public NOutput {

    //TODO add brightness/contrast/hue amplifiers
    
public:
    double rgb[3];
    
    NColor(Brain* b) : NOutput(b, 3) {

    }

    virtual void process(double dt) {
        //neurons -> RGB
        rgb[0] = 0.5 * (1.0 + outs[0]->getOutput());
        rgb[1] = 0.5 * (1.0 + outs[1]->getOutput());
        rgb[2] = 0.5 * (1.0 + outs[2]->getOutput());
    }

    btVector3 getColor() {
        return btVector3(rgb[0], rgb[1], rgb[2]);
    }

    virtual ~NColor() {

    }
    
private:

};

#endif	/* _NCOLOR_H */

