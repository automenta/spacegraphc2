/* 
 * File:   SixDoFMotor.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:41 PM
 */

#ifndef _SIXDOFMOTOR_H
#define	_SIXDOFMOTOR_H

#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <NOutput.h>

class SixDoFMotor : public NOutput {
    btGeneric6DofConstraint* constraint;

    float normalLength;
    float /*linearScale,*/ angularScale;
    float /*linearStimulation,*/ angularStimulation;

    //float lastAngle1, lastAngle2, lastLength;
    
public:

    SixDoFMotor(Brain* b, btGeneric6DofConstraint* _constraint, float _angularScale, float _angularStimulation) :
    NOutput(b, 2), constraint(_constraint), normalLength(0), angularScale(_angularScale),
     angularStimulation(_angularStimulation) {

    }

    virtual void process(double dt) {
        outs[0]->setStimulationFactor(angularStimulation);
        outs[0]->setDecay(0.99);
        outs[1]->setStimulationFactor(angularStimulation);
        outs[1]->setDecay(0.99);
//        outs[2]->setStimulationFactor(linearStimulation);
//        outs[2]->setDecay(0.99);

        //float smoothing = 0.02;

        double a1 = outs[0]->getOutput();
        double a2 = outs[1]->getOutput();

        printf("%f %f\n", a1, a2);
        
        //double l = outs[2]->getOutput();

        //double angle1 = smoothing * a1 + (1.0 - smoothing) * lastAngle1;
        //double angle2 = smoothing * a2 + (1.0 - smoothing) * lastAngle2;
        //double lengthMod = smoothing * l + (1.0 - smoothing) * lastLength;

        //lastAngle1 = angle1;
        //lastAngle2 = angle2;
        //lastLength = lengthMod;

        //TODO expose this parameter
        //double lengthVariation = 0.001;

        float xmax = 0; //normalLength + (lengthMod) * lengthVariation;
        float xmin = 0;

        //cout << xmin << " " << xmax << "\n";

        //constraint->setAngularLowerLimit(btVector3(-angularScale/2, -angularScale/2, -angularScale/2));
        //constraint->setAngularUpperLimit(btVector3(angularScale/2, angularScale/2, angularScale/2));

        constraint->getTranslationalLimitMotor()->m_lowerLimit.setX(xmax);
        constraint->getTranslationalLimitMotor()->m_upperLimit.setX(xmax);
        constraint->getTranslationalLimitMotor()->m_currentLimit[0] = xmax;
        constraint->getTranslationalLimitMotor()->m_enableMotor[0] = true;
        constraint->getTranslationalLimitMotor()->m_currentLimit[1] = xmax;
        constraint->getTranslationalLimitMotor()->m_enableMotor[1] = true;
        constraint->getTranslationalLimitMotor()->m_currentLimit[2] = xmax;
        constraint->getTranslationalLimitMotor()->m_enableMotor[2] = true;

        //setLimit: 0..2 are linear limits, 3..5 are angular limits
        constraint->setLimit(0, 0, 0);
        constraint->setLimit(1, 0, 0);
        constraint->setLimit(2, 0, 0);

        float currentAngle1 = a1 * angularScale;
        constraint->getRotationalLimitMotor(0)->m_currentPosition = currentAngle1;
        constraint->setLimit(3+0, currentAngle1, currentAngle1);
        constraint->getRotationalLimitMotor(0)->m_enableMotor = true;

        float currentAngle2 = a2 * angularScale;
        constraint->getRotationalLimitMotor(1)->m_currentPosition = currentAngle2;
        constraint->setLimit(3+1, currentAngle2, currentAngle2);
        constraint->getRotationalLimitMotor(1)->m_enableMotor = true;

        constraint->setLimit(3+2, 0, 0);
        
        //
        //			btVector3 v(xmin, 0, 0);
        //			c->setLinearLowerLimit(v);
        //
        //			btVector3 w(xmax, 0, 0);
        //			c->setLinearUpperLimit(w);

        //			btScalar fCurAngle = hingeC->getHingeAngle();
        //
        //			btScalar fTargetAngle = (1.0 + getLegTargetAngle(i)) * 2.0; //0.5 * (1 + sin(2 * M_PI * fTargetPercent));
        //
        //			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle
        //					* (hingeC->getUpperLimit() - hingeC->getLowerLimit());
        //			btScalar fAngleError = fTargetLimitAngle - fCurAngle;
        //			btScalar fDesiredAngularVel = 1000000.f * fAngleError / dt
        //					* 1000000.;
        //			hingeC->enableAngularMotor(true, fDesiredAngularVel,
        //					m_fMuscleStrength);

    }

    virtual ~SixDoFMotor() {

    }
private:

};


class BalancedSixDoFRotator : public NOutput {
    btGeneric6DofConstraint* constraint;

    float normalLength;
    float angularScale;
    float angularStimulation;
    float decayRate;

public:

    BalancedSixDoFRotator(Brain* b, btGeneric6DofConstraint* _constraint, float _angularScale, float _angularStimulation, float _decayRate) :
    NOutput(b, 6), constraint(_constraint), normalLength(0), angularScale(_angularScale),
     angularStimulation(_angularStimulation), decayRate(_decayRate) {


    }

    virtual void process(double dt) {
        for (unsigned o = 0; o < 6; o++) {
            outs[o]->setStimulationFactor(angularStimulation);
            outs[o]->setDecay(decayRate);
        }

        double a1Plus = outs[0]->getOutput();
        double a1Neg = outs[1]->getOutput();
        double a2Plus = outs[2]->getOutput();
        double a2Neg = outs[3]->getOutput();
        double a3Plus = outs[4]->getOutput();
        double a3Neg = outs[5]->getOutput();


        constraint->getTranslationalLimitMotor()->m_lowerLimit.setX(0);
        constraint->getTranslationalLimitMotor()->m_upperLimit.setX(0);
        constraint->getTranslationalLimitMotor()->m_currentLimit[0] = 0;
        constraint->getTranslationalLimitMotor()->m_enableMotor[0] = true;
        constraint->getTranslationalLimitMotor()->m_currentLimit[1] = 0;
        constraint->getTranslationalLimitMotor()->m_enableMotor[1] = true;
        constraint->getTranslationalLimitMotor()->m_currentLimit[2] = 0;
        constraint->getTranslationalLimitMotor()->m_enableMotor[2] = true;

        //setLimit: 0..2 are linear limits, 3..5 are angular limits
        constraint->setLimit(0, 0, 0);
        constraint->setLimit(1, 0, 0);
        constraint->setLimit(2, 0, 0);

        float currentAngle1 = (a1Plus - a1Neg) * angularScale;
        constraint->getRotationalLimitMotor(0)->m_currentPosition = currentAngle1;
        constraint->setLimit(3+0, currentAngle1, currentAngle1);
        constraint->getRotationalLimitMotor(0)->m_enableMotor = true;

        float currentAngle2 = (a2Plus - a2Neg) * angularScale;
        constraint->getRotationalLimitMotor(1)->m_currentPosition = currentAngle2;
        constraint->setLimit(3+1, currentAngle2, currentAngle2);
        constraint->getRotationalLimitMotor(1)->m_enableMotor = true;

        float currentAngle3 = (a3Plus - a3Neg) * angularScale;
        constraint->getRotationalLimitMotor(2)->m_currentPosition = currentAngle3;
        constraint->setLimit(3+2, currentAngle3, currentAngle3);
        constraint->getRotationalLimitMotor(2)->m_enableMotor = true;

    }

    virtual ~BalancedSixDoFRotator() {

    }
private:

};


class ImpulseMotor : public NOutput {
    btRigidBody* body;

    float linearScale;
    float linearStimulation;

public:

    ImpulseMotor(Brain* b, btRigidBody* _body, float _linearScale, float _linearStimulation) :
    NOutput(b, 3), body(_body), linearScale(_linearScale), linearStimulation(_linearStimulation) {

    }

    virtual void process(double dt) {
        outs[0]->setStimulationFactor(linearStimulation);
        outs[0]->setDecay(0.5);
        outs[1]->setStimulationFactor(linearStimulation);
        outs[1]->setDecay(0.5);
        outs[2]->setStimulationFactor(linearStimulation);
        outs[2]->setDecay(0.5);

        double x = outs[0]->getOutput() * linearScale;
        double y = outs[1]->getOutput() * linearScale;
        double z = outs[2]->getOutput() * linearScale;

        double ax = body->getWorldTransform().getRotation().getAxis().getX();
        double ay = body->getWorldTransform().getRotation().getAxis().getY();
        double az = body->getWorldTransform().getRotation().getAxis().getZ();

        body->applyCentralForce(btVector3(ax * x, ay * y, az * z));
    }

    virtual ~ImpulseMotor() {

    }
private:

};

class BodyScaleMotor : public NOutput {
    btRigidBody* body;

    float normalLength;
    float linearScale, angularScale;
    float linearStimulation, angularStimulation;

    float lastLength;
    float minScale, maxScale;
    float normalMass;

public:

    BodyScaleMotor(Brain* b, btRigidBody* _body, float _linearScale, float _linearStimulation) :
    NOutput(b, 1), body(_body), normalLength(0), linearScale(_linearScale), linearStimulation(_linearStimulation) {

        lastLength = 0;
        minScale = 0.9;
        maxScale = 1.1;
        normalMass = 1.0f / body->getInvMass();
    }

    virtual void process(double dt) {
        outs[0]->setStimulationFactor(linearStimulation);
        //outs[1]->setDecay(0.9999);

        float smoothing = 0.05;

        double l = outs[0]->getOutput();

        double lengthMod = smoothing * l + (1.0 - smoothing) * lastLength;

        lastLength = lengthMod;

        //TODO expose this parameter

        float xmax = normalLength + lengthMod;

        float yScale = 1.0f + xmax;
        yScale = fmax(yScale, minScale);
        yScale = fmin(yScale, maxScale);

        //default to the Y coordinate
        body->getCollisionShape()->setLocalScaling(btVector3(yScale, yScale, yScale));


    }

    virtual ~BodyScaleMotor() {

    }
private:

};

#endif	/* _SIXDOFMOTOR_H */

