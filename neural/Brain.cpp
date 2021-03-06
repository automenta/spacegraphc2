/*
 * Brain.cpp
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#include <omp.h>

#include "Brain.h"

double Brain::update(float dt) {
    //TODO handle 'dt' appropriately

    // reset fired neurons counter
    neuronsFired = outNeuronsFired = 0;
    double learnedTotal = 0;

    resetOutputs();

    inValues.reserve(ins.size());
    for (unsigned i = 0; i < ins.size(); i++) {
        inValues[i] = ins[i]->getInput();
    }

//    vector<Neuron*> nv;
//    vector<list<Synapse*>* > sv;
//    nv.reserve(neurons.size());
//    sv.reserve(neurons.size());

    //for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {


    unsigned u;
    
    #pragma omp parallel for private(u)
    for (u = 0; u < neuronList.size(); u++)
    {
        //Neuron* n = im->first; //nv[j];
        Neuron* n = neuronList[u];

        //learnedTotal += n->forward(dt, im->second);
        learnedTotal += n->forward(dt, neurons[n]);

        // if neuron fires
        if (n->nextOutput != 0) {
            neuronsFired++;

            // motor neuron check & exec
            OutNeuron* mn = n->target;
            if (mn != NULL) {
                outNeuronsFired++;
                mn->stimulate(n->nextOutput);
            }
        }

        int th_id = omp_get_thread_num();
    }

    // commit outputs at the end
    for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
        Neuron* n = im->first;
        //printf("%p %f -> %f\n", n, n->output, n->nextOutput);
        n->output = n->nextOutput;
    }

    outValues.reserve(outs.size());
    for (unsigned i = 0; i < outs.size(); i++) {
        outValues[i] = outs[i]->getOutput();
    }

    //nv.clear();
    //sv.clear();

    return learnedTotal;
}
