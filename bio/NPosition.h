/* 
 * File:   NPosition.h
 * Author: seh
 *
 * Created on February 15, 2010, 10:06 PM
 */

#ifndef _NPOSITION_H
#define	_NPOSITION_H

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <NInput.h>

/** adapts a 3D vector to an NInput */
class NPosition : public NInput {
    btVector3 p, v;
    int derivatives;

public:

    //numDerivatives is hardwired to 0 or 1
    NPosition(Brain* b, int numDerivatives) : NInput(b, 3*(numDerivatives+1)) {
        derivatives = numDerivatives;
        if (derivatives > 1) derivatives = 1;

    }
    void set(float f[]) {
        btVector3 nextP(f[0], f[1], f[2]);
        v = nextP - p;
        p = nextP;
    }
    void set(float f[], float r[]) {
        btVector3 nextP(f[0]-r[0], f[1]-r[1], f[2]-r[2]);
        v = nextP - p;
        p = nextP;
    }

    void setSin(btVector3 nextP, double distScale) {
        nextP = btVector3(sin(nextP[0] * distScale), sin(nextP[1] * distScale), sin(nextP[2] * distScale));
        v = nextP - p;
        p = nextP;
    }

    virtual void process(double dt) {        
        ins[0]->setInput(p[0]);
        ins[1]->setInput(p[1]);
        ins[2]->setInput(p[2]);

        if (derivatives > 0) {
            ins[3]->setInput(v[0]);
            ins[4]->setInput(v[1]);
            ins[5]->setInput(v[2]);
        }
        //...
    }

    virtual ~NPosition() {

    }
    
private:

};

#endif	/* _NPOSITION_H */

