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

class InterNeuronProcess : public BodyProcess {
    Neuron* neuron;

public:

    InterNeuronProcess(Neuron* i) : BodyProcess(0, 0, 0) {
        neuron = i;
    }

    virtual void update(double dt) {
        float i = neuron->getOutput();
        //float p = neuron->potential;
        if (i < 0) {
            color.setValue(-i, 0, 0);
        }
        else {
            color.setValue(0, i, 0);
        }
    }
};

class BrainSpace : public Cell {
    Brain* brain;
    map<AbstractNeuron*, btRigidBody*> neuronToBody;

public:

    BrainSpace(Spacegraph* s, Brain* _brain) : Cell(s) {
        brain = _brain;

        //INPUTS
        btVector3 p(0, 0, 0);
        for (unsigned i = 0; i < brain->ins.size(); i++) {
            InNeuron* n = brain->ins[i];
            addNeuron(n, p);
            p.setX(p.getX() + 2.0);
        }

        //OUTPUTS
        btVector3 q(0, 0, 4);
        for (unsigned i = 0; i < brain->outs.size(); i++) {
            OutNeuron* n = brain->outs[i];
            addNeuron(n, q);
            q.setX(q.getX() + 2.0);
        }

        //INTERS
        btVector3 r(0, 0, 2);
        for (map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); im != brain->neurons.end(); im++) {
            //cout << im->first << " " << im->second << endl;
            Neuron* n = im->first;
            addInterNeuron(n, r);
            r.setX(r.getX() + 2.0);
        }

    }

    void addNeuron(InNeuron* i, btVector3& position) {

        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(position);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

        InNeuronProcess* proc = new InNeuronProcess(i);
        btRigidBody* base = addNewBody(btScalar(15.), offset, baseShape, proc);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);
        neuronToBody[i] = base;
    }

    void addNeuron(OutNeuron* o, btVector3& position) {

        float neuronMass = 15;
        
        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(position);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

        OutNeuronProcess* proc = new OutNeuronProcess(o);
        btRigidBody* base = addNewBody(btScalar(neuronMass), offset, baseShape, proc);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);

        neuronToBody[o] = base;

    }

    void addInterNeuron(Neuron* o, btVector3& position) {

        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(position);

        btBoxShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

        InterNeuronProcess* proc = new InterNeuronProcess(o);
        btRigidBody* base = addNewBody(btScalar(15.), offset, baseShape, proc);
        base->setDamping(0.05, 0.85);
        base->setDeactivationTime(0.8);
        base->setSleepingThresholds(1.6, 2.5);

        neuronToBody[o] = base;
    }

    void drawNeuronsIncomingSynapses(Neuron *a) {
        //printf("drawing synapses of neuron: %p", a);

        btRigidBody* aBod = neuronToBody[a];

        btVector3 aPos = aBod->getWorldTransform().getOrigin();
        btVector3 axis = aBod->getWorldTransform().getRotation().getAxis().normalized();
        double aax = axis.getX();
        //double aay = axis.getY();

        if (a->target != NULL) {
            drawSynapse(a, aPos, aax, a->getOutput(), a->target);
        }

        list<Synapse*>* synapses = brain->neurons[a];
        for (std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
            Synapse* syn = *list_iter;
            AbstractNeuron *b = syn->inputNeuron;

            drawSynapse(a, aPos, aax, syn->getInput(), b);


        }

    }

    void drawSynapse(Neuron* a, btVector3& aPos, float aax, float w, AbstractNeuron* b) {
        float neuronSize = 0.5;


        //                if (b == NULL) {
        //                    cout << "synapse " << syn << " has NULL inputNeuron\n";
        //                    continue;
        //                }

        btRigidBody *bBod = neuronToBody[b];
        if (bBod == NULL) {
            return;
        }

        btVector3 bPos = bBod->getWorldTransform().getOrigin();
        //double bbx = bBod->getOrientation().getAxis().getX();
        //double bby = bBod->getOrientation().getAxis().getY();

        float input = neuronSize * (0.5 + 0.5 * fmin(fabs(a->potential), 1.0));
        float cr, cg, cb, ca;
        if (w < 0) {
            cr = fmin(0.5 + -w / 2.0, 1.0);
            cg = 0.25;
            cb = 0.25;
            ca = 0.5;
        } else {
            cb = fmin(0.5 + w / 2.0, 1.0);
            cr = 0.25;
            cg = 0.25;
            ca = 0.5;
        }

        //printf("drawing synapse: %p %f %f %f %f %f\n", syn, cr, cg, cb, aPos.getX(), bPos.getX());

        //glColor4f(cr, cg, cb, ca);
        glColor3f(cr, cg, cb);

        glVertex3f(aPos.getX() + aax*input, aPos.getY(), aPos.getZ());
        glVertex3f(aPos.getX() - aax*input, aPos.getY(), aPos.getZ());
        glVertex3f(bPos.getX(), bPos.getY(), bPos.getZ());

    }

    virtual void draw() {
        brain->addRandomInputs(-0.3, 0.3, 0.98);
        brain->update(0.01); //TODO move this to (non-draw) update method
        updateNeuronSizes(0.5, 0.5, 2.0);
        fdLayout();

        glBegin(GL_TRIANGLES);

        //        for (unsigned i = 0; i < brain->outs.size(); i++) {
        //            drawNeuronsOutgoingSynapses(brain->outs[i]);
        //        }


        for (map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); (im != brain->neurons.end()); im++) {
            Neuron* a = im->first;
            drawNeuronsIncomingSynapses(a);
        }
        
        glEnd();

    }

    void randomizeLocations(float wx, float wy, float wz) {
        for(map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); (im != brain->neurons.end()); im++) {
            Neuron* a = im->first;
            
            btRigidBody *aBod = neuronToBody[a];
            btVector3 aPos = aBod->getWorldTransform().getOrigin();
            aPos.setValue(frand(0, wx), frand(0, wy), frand(0, wz));
            aBod->getWorldTransform().setOrigin(aPos);
        }

    }

    void updateNeuronSizes(float neuronSize, float minSize, float maxSize) {
        //update shape sizes
        for (map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); (im != brain->neurons.end()); im++) {
            Neuron *n = im->first;
            btCollisionShape* bt = neuronToBody[n]->getRootCollisionShape();
            float s = neuronSize;
            //TODO
            //  Neuron* neuron = getNeuronFor(bt);

            float w = s * (1.0 + (fabs(n->getOutput())));
            float h = s * (1.0 + sqrt(fabs(n->potential)));
            w = fmax(minSize, fmin(w, maxSize));
            h = fmax(minSize, fmin(h, maxSize));

            bt->setLocalScaling(btVector3(w, h, (w + h) / 2.0));
        }
    }

    void fdLayout() {
        float neuronSize = 0.5;
        float naturalLength = 30.0;
        float tension = 0.1;
        float speed = 0.1;
        float repulsion = 0.3;

        btVector3 minXYZ(-20, -20, -20);
        btVector3 maxXYZ(20, 20, 20);


        //attract
        btVector3 center(0, 0, 0);

        unsigned i;
        //#pragma omp parallel for private(i)
        for(map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); (im != brain->neurons.end()); im++) {
            Neuron* a = im->first;
            btRigidBody *aBod = neuronToBody[a];
            btVector3 aPos = aBod->getWorldTransform().getOrigin();

            //TODO iterate synapses of a neuron
            unsigned numSynapses = brain->neurons[a]->size();// a->synapses.size();
            list<Synapse*>* synapses = brain->neurons[a];
            for (std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
                Synapse* syn = *list_iter;
                AbstractNeuron *b = syn->inputNeuron;
                if (b == NULL)
                    continue;

                btRigidBody *bBod = neuronToBody[b];
                if (bBod == NULL)
                    continue;

                btVector3 bPos = bBod->getWorldTransform().getOrigin();

                float currentLength = aPos.distance(bPos);
                float targetLength = neuronSize * naturalLength /** (2.0 - fabs(syn->weight))*/; // / (1.0 + fabs(syn->weight));
                float f = tension * (currentLength - targetLength)/* / ((float) numSynapses)*/;

                float sx = f * (bPos.getX() - aPos.getX());
                float sy = f * (bPos.getY() - aPos.getY());
                float sz = f * (bPos.getZ() - aPos.getZ());

                aPos += btVector3(sx / 2.0, sy / 2.0, sz / 2.0) * speed;
                bPos -= btVector3(sx / 2.0, sy / 2.0, sz / 2.0) * speed;

                aBod->getWorldTransform().setOrigin(aPos);
                bBod->getWorldTransform().setOrigin(bPos);

            }
        }

        //repel
        for(map< Neuron*, list<Synapse*>* >::iterator im = brain->neurons.begin(); (im != brain->neurons.end()); im++) {
            Neuron* a = im->first;
            btRigidBody *aBod = neuronToBody[a];
            btVector3 aPos = aBod->getWorldTransform().getOrigin();

            if (repulsion != 0.0) {
                btVector3 force(0, 0, 0);
            
                for(map< Neuron*, list<Synapse*>* >::iterator jm = brain->neurons.begin(); (jm != brain->neurons.end()); jm++) {
                    Neuron* b = jm->first;
                    if (a == b)
                        continue;

                    btRigidBody *bBod = neuronToBody[b];
                    if (bBod == NULL)
                        continue;

                    btVector3 bPos = bBod->getWorldTransform().getOrigin();

                    double dist = bPos.distance(aPos);

                    double f = repulsion * /*(nMass * mMass)*/1.0 / (dist * dist);

                    double sx = f * (aPos.getX() - bPos.getX());
                    double sy = f * (aPos.getY() - bPos.getY());
                    double sz = f * (aPos.getZ() - bPos.getZ());
                    force += btVector3(sx, sy, sz);
                }

                aPos += force * speed;
            }

            aPos[0] = fmax(fmin(aPos[0], maxXYZ[0]), minXYZ[0]);
            aPos[1] = fmax(fmin(aPos[1], maxXYZ[1]), minXYZ[1]);
            aPos[2] = fmax(fmin(aPos[2], maxXYZ[2]), minXYZ[2]);

            aBod->getWorldTransform().setOrigin(aPos);

            center += aPos;
        }

        center /= ((double) brain->numNeurons);

//        if (center) {
//            #pragma omp parallel for private(i)
//            for (i = 0; i < brain->numNeurons; i++) {
//                btRigidBody *aBod = neuronBody[i];
//                btVector3 pos = aBod->getWorldTransform().getOrigin();
//                aBod->getWorldTransform().setOrigin(pos - center);
//            }
//        }


        
    }
};

#endif	/* BRAINSPACE_H */

