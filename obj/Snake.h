/* 
 * File:   Snake.h
 * Author: me
 *
 * Created on October 24, 2010, 6:21 PM
 */

#ifndef SNAKE_H
#define	SNAKE_H

#include <vector>
using namespace std;

#include "Cell.h"

/** Snake, or a limb */
class Snake : public Cell {
public:

    vector<btRigidBody*> parts;
    vector<btGeneric6DofConstraint*> joints;
    unsigned numParts;

    Snake(Spacegraph* s, const btVector3& positionOffset, unsigned _numParts, float partLength, float radius /* length, width, height, ... */) : Cell(s) {
        numParts = _numParts;
        parts.resize(numParts);
        joints.resize(numParts-1);

        // Setup all the rigid bodies
        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(positionOffset);


        float partMass = 10.0;
        float x = 0;
        for (unsigned i = 0; i < numParts; i++) {
            btTransform transform;
            transform.setIdentity();

            btCapsuleShape* shape = new btCapsuleShape(btScalar(radius), btScalar(partLength));
            transform.setIdentity();
            transform.setOrigin(btVector3(btScalar(x), btScalar(0), btScalar(0.)));
            transform.getBasis().setEulerZYX(0, 0, M_PI_2);
            parts[i] = addNewBody(btScalar(partMass), offset*transform, shape, 1.0, 0.75, 0.25);
            parts[i]->setDamping(0.05, 0.85);
            parts[i]->setDeactivationTime(0.8);
            parts[i]->setSleepingThresholds(1.6, 2.5);

            x += partLength;
        }

        float gap = partLength/10.0+radius/2.0;
        
        for (unsigned i = 0; i < numParts - 1; i++) {
            btGeneric6DofConstraint* con;

            btTransform localA, localB;

            //connect i to i+1
            localA.setIdentity();
            localB.setIdentity();
            //localA.getBasis().setEulerZYX(0, 0, M_PI_2);
            localA.setOrigin(btVector3(0, btScalar(-partLength/2.0 - gap/2.0), btScalar(0.)));
            //localB.getBasis().setEulerZYX(0, 0, M_PI_2);
            localB.setOrigin(btVector3(0, btScalar(partLength/2.0 + gap/2.0), btScalar(0.)));
            con = new btGeneric6DofConstraint(*parts[i], *parts[i+1], localA, localB, false);
            //con.setLimit(0, 0, 0.1);
            //con->setLimit(M_PI_4, M_PI_4, M_PI_2);
            joints[i] = con;
            //coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
            
            m_ownerWorld->addConstraint(joints[i], true);
        }


    }

};

#endif	/* SNAKE_H */

