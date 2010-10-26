/* 
 * File:   BodyState.h
 * Author: me
 *
 * Created on October 24, 2010, 8:22 PM
 */

#ifndef BODYSTATE_H
#define	BODYSTATE_H

#include "btBulletDynamicsCommon.h"

class BodyProcess {
public:
    btVector3 color;
    
    BodyProcess(float r, float g, float b) {
        color.setX(r);
        color.setY(g);
        color.setZ(b);
    }

    virtual void update(double dt) {    }
    virtual void draw() {    }
};

#endif	/* BODYSTATE_H */

