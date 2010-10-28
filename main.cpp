/* 
 * File:   main.cpp
 * Author: me
 *
 * Created on October 23, 2010, 11:07 PM
 */

#include <cstdlib>

using namespace std;

#include "Spacegraph.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

#include "Humanoid.h"
#include "Bench.h"
#include "Snake.h"

#include "Brain.h"
#include "Neuron.h"
#include "BrainSpace.h"

void eden() {
    Spacegraph s;

    s.initPhysics();

    s.getSpace()->setGravity(btVector3(0, -10.0, 0));

    s.addGround();
    s.addCell(new Humanoid(&s, btVector3(1, 0.5, 0)));
    s.addCell(new Humanoid(&s, btVector3(-1, 0.5, 0)));
    s.addCell(new Bench(&s, btVector3(1.7, 1.9, 1.6)));
    s.addCell(new Snake(&s, btVector3(1.7, 1.9, -1.6), 15, 0.2, 0.1));

    //s.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    glutmain(0, NULL, 1920, 1080, "SpaceGraphC", &s);

}

void testBrain() {
    Brain* b = new Brain();

    vector<InNeuron*> ins;
    vector<OutNeuron*> outs;

    unsigned numIn = 32, numOut = 16, num = 256;

    for (unsigned i = 0; i < numIn; i++) {
        InNeuron* n = new InNeuron();
        n->setInput(0.5);
        ins.push_back(n);
    }
    b->addInputs(&ins);

    for (unsigned i = 0; i < numOut; i++) {
        outs.push_back(new OutNeuron(0.8, 0.999));
    }
    b->addOutputs(&outs);

    unsigned minSynapsesPerNeuron = 2;
    unsigned maxSynapsesPerNeuron = 6;
    float percentInhibitoryNeuron = 0.5;
    float percentInhibitorySynapse = 0.5;
    float percentInputSynapse = 0.5;
    float percentOutputNeuron = 0.5;
    float minSynapseWeight = 0.50;
    float maxSynapseWeight = 0.99;
    float neuronPotentialDecay = 0.99;

    b->wireRandomly(
            num,
            minSynapsesPerNeuron, maxSynapsesPerNeuron,
            percentInhibitoryNeuron,
            percentInputSynapse,
            percentInhibitorySynapse,
            percentOutputNeuron,
            minSynapseWeight,
            maxSynapseWeight,
            neuronPotentialDecay
            );

    b->printSummary();

    Spacegraph s;

    s.initPhysics();

    BrainSpace* bs = new BrainSpace(&s, b);
    //bs->fdLayout(100);
    bs->randomizeLocations(70.0, 70.0, 70.0);
    
    s.addCell(bs);

    glutmain(0, NULL, 1920, 1080, "SpaceGraphC", &s);

}

int main(int argc, char* argv[]) {
    //eden();
    testBrain();
}
