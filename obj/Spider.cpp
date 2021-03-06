/* 
 * File:   SpiderBody.cpp
 * Author: seh
 * 
 * Created on February 15, 2010, 9:18 PM
 */

#include "Spider.h"

Spider::Spider(Spacegraph* s, unsigned numLegs, vector<btScalar>* _legLengths, vector<btScalar>* _legRadii, const btVector3& _positionOffset, unsigned _retinaSize, unsigned _initialNeurons, unsigned minSynapses, unsigned maxSynapses) : Cell(s) {
    NUM_LEGS = numLegs;
    legLengths = _legLengths;
    legRadii = _legRadii;
    PARTS_PER_LEG = legLengths->size();
    retinaSize = _retinaSize;

    headPhase = 0;

    positionOffset = _positionOffset;
    initialNeurons = _initialNeurons;
    //shapes.reserve(PART_COUNT);
    //joints.reserve(JOINT_COUNT);

    btVector3 vUp(0, 1, 0);

    // Setup geometry
    float headRadius = 0.15f;
    float headHeight = 0.015f;
    float headMass = 3.05;

    float fHeight = 0.5;

    float fLegDensity = 0.8;

    float motorStrength = 0.0005;  //amount that each spike actuates a motor
    float motorDecay = 0.998; //lower = returns to zero (stillness) quicker, higher = returns to zero slower... between [0,1.0]

    double partSeparationFactor = 1.05;

    int angleResolution = 12;

    brain = new Brain();

    // Setup rigid bodies
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(positionOffset);

    // root
    btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);

    headShape = new btCapsuleShape(btScalar(headRadius), btScalar(headHeight));

    btRigidBody* headBody = addNewBody(headMass, offset*transform, headShape, new BodyProcess(0.9, 0.9, 0.9));

    float fLegLength = (*legLengths)[0];

    //        BodyScaleMotor* hbm  = new BodyScaleMotor(brain, headBody, 15.5, 0.55);
    //        scaleControllers.push_back(hbm);
    //
    //        ImpulseMotor* hi = new ImpulseMotor(brain, headBody, 0.1, 0.1);
    //        impulseControllers.push_back(hi);

    // legs
    for (unsigned i = 0; i < NUM_LEGS; i++) {
        float fAngle = 2 * M_PI * i / NUM_LEGS + headPhase;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);

        transform.setIdentity();
        btVector3 vBoneOrigin = btVector3(btScalar(fCos * (headRadius + 0.5 * fLegLength)), btScalar(fHeight), btScalar(fSin * (headRadius + 0.5 * fLegLength)));
        transform.setOrigin(vBoneOrigin);

        for (unsigned j = 0; j < PARTS_PER_LEG; j++) {
            btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
            btVector3 vAxis = vToBone.cross(vUp);
            transform.setRotation(btQuaternion(vAxis, M_PI_2));
            float fLegLength = (*legLengths)[j];
            float fLegRadius = fLegLength * (*legRadii)[j];
            float fLegMass = fLegDensity * fLegLength * fLegRadius * fLegRadius; //approximate cylinder volume
            //btCapsuleShape* legPart = new btCapsuleShape(btScalar(fLegRadius), btScalar(fLegLength));
            btBoxShape* legPart = new btBoxShape(btVector3(fLegRadius, fLegLength * 0.5, fLegRadius));

            float g = 0.0;
            float r = j % 2 ? 0.5 : 0.75;
            float b = i % 2 ? 0.5 : 0.75;

            btRigidBody* legPartBody = addNewBody(btScalar(fLegMass), offset*transform, legPart, new BodyProcess(r, g, b));

            if (j == PARTS_PER_LEG-1) {

                Retina* r = new Retina(brain, s, legPartBody, retinaSize, retinaSize, 1.0, 8, M_PI/4.0);
                r->originOffset = btVector3(0, 0, 0);
                r->forwardSign = -1;
                retinas.push_back( r );
                addProcess(r);

                //ImpulseMotor* hl = new ImpulseMotor(brain, legPartBody, 0.001, 0.001);
                //impulseControllers.push_back(hl);
            }
            else /*if (j == 0)*/ {
                float touchDistance = 0.5;
                {
                    Retina* r = new Retina(brain, s, legPartBody, 2, 2, 1.0, touchDistance, M_PI/2.0);
                    r->basisForward = 0;
                    r->basisUp = 1;
                    r->originOffset = btVector3(0, 0, 0);
                    
                    retinas.push_back( r );
                    addProcess(r);
                }
                {
                    Retina* r = new Retina(brain, s, legPartBody, 2, 2, 1.0, touchDistance, M_PI/2.0);
                    r->basisForward = 0;
                    r->basisUp = 1;
                    r->forwardSign = -1;
                    r->originOffset = btVector3(0, 0, 0);

                    retinas.push_back( r );
                    addProcess(r);
                }
                {
                    Retina* r = new Retina(brain, s, legPartBody, 2, 2, 1.0, touchDistance, M_PI/2.0);
                    r->basisForward = 2;
                    r->basisUp = 1;
                    r->forwardSign = -1;
                    r->originOffset = btVector3(0, 0, 0);

                    retinas.push_back( r );
                    addProcess(r);
                }
                {
                    Retina* r = new Retina(brain, s, legPartBody, 2, 2, 1.0, touchDistance, M_PI/2.0);
                    r->basisForward = 2;
                    r->basisUp = 1;
                    r->originOffset = btVector3(0, 0, 0);

                    retinas.push_back( r );
                    addProcess(r);
                }

            }

            //                BodyScaleMotor* bm  = new BodyScaleMotor(brain, legPartBody, 15.5, 0.55);
            //                scaleControllers.push_back(bm);

        }

    }

    //        {
    //            headEye = new Retina(brain, s->getSpace(), headBody, retinaSize*2.0, retinaSize*2.0, 0.78, 90);
    //            headEye->originOffset = btVector3(0,0,0);
    //            legEye.push_back( headEye );
    //
    //        }

    kinestheticInputsStart = brain->getNumInputs();

    // Setup some damping on the m_bodies
    for (unsigned i = 0; i < bodies.size(); ++i) {
        //bodies[i]->setDamping(0.8, 0.85);
        //bodies[i]->setDeactivationTime(0.8);
        //bodies[i]->setSleepingThresholds(0.5f, 0.5f);

        partPos.push_back(new NAngle(brain, angleResolution));
    }

    kinestheticInputsStop = brain->getNumInputs();

    btTransform localA, localB;


    for (unsigned i = 0; i < NUM_LEGS; i++) {

        double la, lb;

        btGeneric6DofConstraint* c;

        localA.setIdentity();
        localB.setIdentity();
        la = headRadius;
        lb = fLegLength;

        float fAngle = 2 * M_PI * i / NUM_LEGS + headPhase;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);

        btRigidBody* head = bodies[0];
        btRigidBody* thigh = bodies[1 + i * PARTS_PER_LEG];
        localA.getBasis().setEulerZYX(0, -fAngle, 0);
        localA.setOrigin(btVector3(btScalar(fCos * headRadius), btScalar(0.), btScalar(fSin * headRadius)));
        localB = thigh->getWorldTransform().inverse() * head->getWorldTransform() * localA;

        c = new btGeneric6DofConstraint(*head, *thigh, localA, localB, false);

        joints.push_back(c);
        s->getSpace()->addConstraint(c);

        BalancedSixDoFRotator* sm = new BalancedSixDoFRotator(brain, c, M_PI_2 / 2.0, motorStrength, motorDecay);
        jointControllers.push_back(sm);

    }
    for (unsigned j = 1; j < PARTS_PER_LEG; j++) {
        for (unsigned i = 0; i < NUM_LEGS; i++) {

            double la, lb;

            btGeneric6DofConstraint* c;

            localA.setIdentity();
            localB.setIdentity();
            la = (*legLengths)[j - 1];
            lb = (*legLengths)[j];

            localA.setOrigin(btVector3(0, -(la / 2) * partSeparationFactor, 0));
            localB.setOrigin(btVector3(0, (lb / 2) * partSeparationFactor, 0));

            c = new btGeneric6DofConstraint(*bodies[1 + i * PARTS_PER_LEG + j - 1], *bodies[1 + i * PARTS_PER_LEG + j], localA, localB, false);

            joints.push_back(c);
            s->getSpace()->addConstraint(c);

            BalancedSixDoFRotator* sm = new BalancedSixDoFRotator(brain, c, M_PI_2 / 2.0, motorStrength, motorDecay);
            jointControllers.push_back(sm);

        }
    }

    //voice = new SineSound(brain, space->audio, 16);

    addNeurons(initialNeurons, minSynapses, maxSynapses);

}
