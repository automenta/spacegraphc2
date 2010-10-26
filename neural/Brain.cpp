/*
 * Brain.cpp
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#include "Brain.h"

void Brain::update(float dt) {
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

    for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
        //nv.push_back(im->first);
        //sv.push_back(im->second);
    //}


    
    //#pragma omp parallel for
    //for (unsigned j = 0; j < nv.size(); j++) {
        //cout << omp_in_parallel() << " " << omp_get_dynamic() <<" " << omp_get_thread_num() << " of " << omp_get_num_threads() << "\n";

        Neuron* n = im->first; //nv[j];
        learnedTotal += n->forward(dt, im->second);

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
    }

    // commit outputs at the end
    for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
        Neuron* n = im->first;
        n->output = n->nextOutput;
    }

    outValues.reserve(outs.size());
    for (unsigned i = 0; i < outs.size(); i++) {
        outValues[i] = outs[i]->getOutput();
    }

    //nv.clear();
    //sv.clear();
}
