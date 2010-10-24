/* 
 * File:   Bench.h
 * Author: me
 *
 * Created on October 24, 2010, 2:18 AM
 */

#ifndef BENCH_H
#define	BENCH_H

#include "Cell.h"

class Bench : public Cell {
public:
    
    Bench (Spacegraph* s, const btVector3& positionOffset) : Cell(s)
    {
        btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.35, 0.25, 0.85));
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(0,2,0));

        btRigidBody* base = addNewBody(btScalar(15.), offset/**transform*/, baseShape);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);
        
    }

};

class DentistChair {

};

/** Bed... or operating table */
class Bed {

};

#endif	/* BENCH_H */

