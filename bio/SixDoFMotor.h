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
    float linearScale, angularScale;
    float linearStimulation, angularStimulation;

    float lastAngle1, lastAngle2, lastLength;
public:

    SixDoFMotor(Brain* b, btGeneric6DofConstraint* _constraint, float _linearScale, float _angularScale, float _linearStimulation, float _angularStimulation) :
    NOutput(b, 3), constraint(_constraint), normalLength(0), linearScale(_linearScale), angularScale(_angularScale),
    linearStimulation(_linearStimulation), angularStimulation(_angularStimulation) {

        lastAngle1 = lastAngle2 = lastLength = 0;

    }

    virtual void process(double dt) {
        outs[0]->setStimulationFactor(angularStimulation);
        outs[0]->setDecay(0.99);
        outs[1]->setStimulationFactor(angularStimulation);
        outs[1]->setDecay(0.99);
        outs[2]->setStimulationFactor(linearStimulation);
        outs[2]->setDecay(0.99);

        float smoothing = 0.02;

        double a1 = outs[0]->getOutput();
        double a2 = outs[1]->getOutput();
        double l = outs[2]->getOutput();

        double angle1 = smoothing * a1 + (1.0 - smoothing) * lastAngle1;
        double angle2 = smoothing * a2 + (1.0 - smoothing) * lastAngle2;
        double lengthMod = smoothing * l + (1.0 - smoothing) * lastLength;

        lastAngle1 = angle1;
        lastAngle2 = angle2;
        lastLength = lengthMod;

        //TODO expose this parameter
        double lengthVariation = 0.001;

        float xmax = 0; //normalLength + (lengthMod) * lengthVariation;
        float xmin = 0;

        //cout << xmin << " " << xmax << "\n";

        constraint->setAngularLowerLimit(btVector3(-angularScale/2, -angularScale/2, 0));
        constraint->setAngularUpperLimit(btVector3(angularScale/2, angularScale/2, 0));

        constraint->getTranslationalLimitMotor()->m_currentLimit[0] = xmax;
        constraint->getTranslationalLimitMotor()->m_lowerLimit.setX(xmax);
        constraint->getTranslationalLimitMotor()->m_upperLimit.setX(xmax);
        constraint->getTranslationalLimitMotor()->m_enableMotor[0] = true;

        //setLimit: 0..2 are linear limits, 3..5 are angular limits
        
        float currentAngle1 = angle1 * angularScale;
        constraint->getRotationalLimitMotor(0)->m_currentPosition = currentAngle1;
        constraint->setLimit(3+0, currentAngle1, currentAngle1);
        constraint->getRotationalLimitMotor(0)->m_enableMotor = true;

        float currentAngle2 = angle2 * angularScale;
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

