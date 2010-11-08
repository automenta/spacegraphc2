/* 
 * File:   main.cpp
 * Author: me
 *
 * Created on October 23, 2010, 11:07 PM
 */

#include <cstdlib>

#include <vector>
using namespace std;

#include <omp.h>

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
#include "Spider.h"

void addWall(Spacegraph* s, double w, double thick) {
    double z = thick*4.0;
    s->addCell(new ColorBox(s, thick, thick, w/2.0, btVector3(-w/2.0, z, 0), 0.5, 0.5, 0.5));
    s->addCell(new ColorBox(s, thick, thick, w/2.0, btVector3(w/2.0, z, 0), 0.5, 0.5, 0.5));
    s->addCell(new ColorBox(s, w/2.0-thick, thick, thick, btVector3(0, z, w/2.0), 0.5, 0.5, 0.5));
    s->addCell(new ColorBox(s, w/2.0-thick, thick, thick, btVector3(0, z, -w/2.0), 0.5, 0.5, 0.5));
}

void addRandomBlocks(Spacegraph* s, unsigned num, float minSize, float maxSize) {
    for (unsigned j = 0; j < num; j++) {
        float x = frand(-8.0, 8.0);
        float y = frand(-8.0, 8.0);
        float w = frand(minSize, maxSize);
        float h = frand(minSize, maxSize);
        float d = frand(minSize, maxSize);
        float r = frand(0, 1.0);
        float g = frand(0, 1.0);
        float b = frand(0, 1.0);
        s->addCell(new ColorBox(s, w, h, d, btVector3(x, 5.0, y), r, g, b));
    }

}

void eden() {
    Spacegraph s;

    s.initPhysics();

//    s.getSpace()->setGravity(btVector3(0, -4.0, 0));
//
//    s.addGround();

    //s.addCell(new Humanoid(&s, btVector3(1, 0.5, 0)));
    //s.addCell(new Humanoid(&s, btVector3(-1, 0.5, 0)));
    //s.addCell(new Snake(&s, btVector3(1.7, 1.9, -1.6), 15, 0.2, 0.1));

    addWall(&s, 12, 0.5);
    addRandomBlocks(&s, 8, 0.25, 1.0);
    addRandomBlocks(&s, 8, 0.05, 0.15);


    //for (unsigned i = 1; i < 2; i++)
    //{
        vector<btScalar> legLengths;
        vector<btScalar> legRadii;
        legLengths.push_back(0.2);  legRadii.push_back(0.35);
        legLengths.push_back(0.15);  legRadii.push_back(0.33);
        legLengths.push_back(0.12);  legRadii.push_back(0.31);
        legLengths.push_back(0.11);  legRadii.push_back(0.29);
        legLengths.push_back(0.10);  legRadii.push_back(0.27);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
        legLengths.push_back(0.09);  legRadii.push_back(0.25);
//        legLengths.push_back(0.09);  legRadii.push_back(0.25);
//        legLengths.push_back(0.09);  legRadii.push_back(0.25);
//        legLengths.push_back(0.09);  legRadii.push_back(0.2);


        Spider* spider = new Spider(&s, 1, &legLengths, &legRadii, btVector3(-1.7, 0.9, -1.6),48, 86000, 2, 4);
        s.addCell(spider);

    //}

    //s.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    glutmain(0, NULL, 1920, 1080, "SpaceGraphC", &s);

}

void testBrain() {
    Brain* b = new Brain();

    vector<InNeuron*> ins;
    vector<OutNeuron*> outs;

    unsigned numIn = 2, numOut = 4, num = 16;

    for (unsigned i = 0; i < numIn; i++) {
        InNeuron* n = new InNeuron();
        n->setInput(0.5);
        ins.push_back(n);
    }
    b->addInputs(&ins);

    for (unsigned i = 0; i < numOut; i++) {
        outs.push_back(new OutNeuron(0.1, 0.99));
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


    unsigned historyLength = 512;

    float y = -4;

    for (unsigned i = 0; i < b->ins.size(); i++) {
        NeuronBarGraph* bg = new NeuronBarGraph(&s, new btVector3(0,y,0), b->ins[i] , 12.0, historyLength);
        s.addCell(bg);
        y-=2;
    }

    for (unsigned o = 0; o < b->outs.size(); o++) {
        NeuronBarGraph* bg = new NeuronBarGraph(&s, new btVector3(0,y,0), b->outs[o] , 12.0, historyLength);
        s.addCell(bg);
        y-=2;
    }

    glutmain(0, NULL, 1920, 1080, "SpaceGraphC", &s);

}

void initMP() {

    omp_set_num_threads(4);
}

int main(int argc, char* argv[]) {

    initMP();


    eden();
    //testBrain();
}
