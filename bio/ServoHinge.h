/* 
 * File:   ServoHinge.h
 * Author: seh
 *
 * Created on April 25, 2010, 8:34 PM
 */

#ifndef _SERVOHINGE_H
#define	_SERVOHINGE_H

#include <math.h>

//#include <space/DefaultSpace.h>
//#include <space/AbstractBody.h>
//
//#include <neural/Brain.h>
//#include <bio/Retina.h>
//#include <bio/NPosition.h>
//#include <bio/SixDoFMotor.h>
//#include <bio/SineSound.h>
//
//
//class ServoHinge : public AbstractBody {
//public:
//    btHingeConstraint* c;
//    float angle;
//    float t;
//
//    btRigidBody* fingerA;
//    btRigidBody* fingerB;
//
//    btVector3* aShape;
//    btVector3* bShape;
//
//
//    ServoHinge(btVector3* _aShape, btVector3* _bShape) : aShape(_aShape), bShape(_bShape) {
//        c = NULL;
//    }
//
//    virtual ~ServoHinge();
//
//     //btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA = false);
//    virtual void init() {
//
//        float fingerMass = 0.5;
//
//        fingerA = createRigidShape(fingerMass, btVector3(0,0,-1), new btBoxShape(*aShape));
//        fingerB = createRigidShape(fingerMass, btVector3(0,0,1), new btBoxShape(*bShape));
//
//        float u = 0.5 * ((*aShape).x() + (*bShape).x()) / 10.0;
//
//        angle = 0.0;
//        t = 0;
//
//        btVector3 pa(-(*aShape).x()-u, 0, 0);
//        btVector3 pb(-(*bShape).x()+u, 0, 0);
//        btVector3 aa(0, 1, 0);
//        btVector3 ab(0, 1, 0);
//        c = new btHingeConstraint(*fingerA, *fingerB, pa, pb, aa, ab, false);
//        c->enableMotor(true);
//        c->enableAngularMotor(true, 10.0, 2.0);
//        c->enableFeedback(true);
//        addJoint(c);
//    }
//
//    virtual void process(btScalar dt) {
//        if (c!=NULL) {
//            c->setMotorTarget(angle, 1.0);
//            c->setLimit(angle-0.1, angle+0.1, 0.5, 0.5, 0.5);
//            t += dt;
//            angle = 1.5 + 0.45 * sin(t);
//        }
//    }
//
//
//};

#endif	/* _SERVOHINGE_H */

