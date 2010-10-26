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

    unsigned numIn = 2, numOut = 3, num = 100;

    for (unsigned i = 0; i < numIn; i++) {
        ins.push_back(new InNeuron());
    }
    b->addInputs(&ins);

    for (unsigned i = 0; i < numOut; i++) {
        outs.push_back(new OutNeuron());
    }
    b->addOutputs(&outs);

    unsigned minSynapsesPerNeuron = 1;
    unsigned maxSynapsesPerNeuron = 5;
    float percentInhibitoryNeuron = 0.5;
    float percentInputSynapse = 0.25;
    float percentInhibitorySynapse = 0.5;
    float percentOutputNeuron = 0.25;
    float minSynapseWeight = 0.1;
    float maxSynapseWeight = 0.5;
    float neuronPotentialDecay = 0.1;

    for (unsigned i = 0; i < num; i++) {
        b->wireRandomly(
                minSynapsesPerNeuron, maxSynapsesPerNeuron,
                percentInhibitoryNeuron, percentInputSynapse,
                percentInhibitorySynapse,
                percentOutputNeuron,
                minSynapseWeight,
                maxSynapseWeight,
                neuronPotentialDecay
                );
    }

    b->update(0.1);

    b->printSummary();

    Spacegraph s;

    s.initPhysics();

    s.addCell(new BrainSpace(&s, b));

    glutmain(0, NULL, 1920, 1080, "SpaceGraphC", &s);

}

int main(int argc, char* argv[]) {
    //eden();
    testBrain();
}
