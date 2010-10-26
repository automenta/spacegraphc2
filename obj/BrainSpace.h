/* 
 * File:   BrainSpace.h
 * Author: me
 *
 * Created on October 25, 2010, 3:29 PM
 */

#ifndef BRAINSPACE_H
#define	BRAINSPACE_H

#include "Cell.h"
#include "Brain.h"
#include "BodyProcess.h"

class InNeuronProcess : public BodyProcess {
    InNeuron* neuron;
    
public:
    InNeuronProcess(InNeuron* i) : BodyProcess(0, 0, 0) {
        neuron = i;
    }

    virtual void update(double dt) {
        float i = neuron->getInput();
        color.setValue(i, i, i);
    }
};

class OutNeuronProcess : public BodyProcess {
    OutNeuron* neuron;

public:
    OutNeuronProcess(OutNeuron* i) : BodyProcess(0, 0, 0) {
        neuron = i;
    }

    virtual void update(double dt) {
        float i = neuron->getOutput();
        color.setValue(i, i, i);
    }
};

class BrainSpace : public Cell {

public:
    BrainSpace(Spacegraph* s, Brain* brain) : Cell(s) {

        btVector3 p(0,0,0);
        for (unsigned i = 0; i < brain->ins.size(); i++) {
            InNeuron* n = brain->ins[i];
            addNeuron(n, p);
            p.setX(p.getX() + 2.0);
        }

        btVector3 q(0,0,2);
        for (unsigned i = 0; i < brain->outs.size(); i++) {
            OutNeuron* n = brain->outs[i];
            addNeuron(n, q);
            q.setX(q.getX() + 2.0);
        }

    }
    
    
    void addNeuron(InNeuron* i, btVector3& position) {
        
        btTransform offset; offset.setIdentity();    offset.setOrigin(position);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

        InNeuronProcess* proc = new InNeuronProcess(i);
        btRigidBody* base = addNewBody(btScalar(15.), offset, baseShape, proc);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);    
    }

    void addNeuron(OutNeuron* o, btVector3& position) {

        btTransform offset; offset.setIdentity();    offset.setOrigin(position);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

        OutNeuronProcess* proc = new OutNeuronProcess(o);
        btRigidBody* base = addNewBody(btScalar(15.), offset, baseShape, proc);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);
    }

};

#endif	/* BRAINSPACE_H */

