/*
 * Brain.h
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#ifndef BRAIN_H_
#define BRAIN_H_

#include <iostream>
#include <map>
#include <list>
#include <vector>
#include <stdio.h>
#include <math.h>

using namespace std;

#include "Math.h"
#include "Neuron.h"

class Brain {
public:
    int neuronsFired;
    int outNeuronsFired;

    int numNeurons;
    int totalSynapses;
    
    map<Neuron*, list<Synapse*>* > neurons;
    //vector<Neuron*> neurons;

    vector<InNeuron*> ins;
    vector<OutNeuron*> outs;

    vector<float> inValues;
    vector<float> outValues;

    Brain() {
        totalSynapses = 0;
    }


//    //DEPRECATED
//    Brain(unsigned numInputs, unsigned numOutputs, unsigned _numNeurons, unsigned _minSynapsesPerNeuron, unsigned _maxSynapsesPerNeuron, float _percentInhibitory) {
//        numNeurons = _numNeurons;
//        minSynapses = _minSynapsesPerNeuron;
//        maxSynapses = _maxSynapsesPerNeuron;
//
//        percentChanceInhibitoryNeuron = _percentInhibitory;
//        percentChanceInhibitorySynapses = _percentInhibitory;
//
//        for (unsigned i = 0; i < numInputs; i++)
//            newInput();
//
//        for (unsigned i = 0; i < numOutputs; i++)
//            newOutput();
//
//        initDefaults();
//        //init();
//
//    }


    
//    int getNeuronIndex(AbstractNeuron* a) {
//        for (unsigned i = 0; i < neurons.size(); i++) {
//            Neuron* n = neurons[i];
//            if (n == a)
//                return i;
//        }
//        return -1;
//    }


    InNeuron* newInput() {
        InNeuron* i = new InNeuron();
        ins.push_back(i);
        inValues.resize(ins.size(), 0);
        return i;
    }
    void addInputs(vector<InNeuron*>* addedIn) {
        for (unsigned i = 0; i < addedIn->size(); i++)
            ins.push_back((*addedIn)[i]);
        inValues.resize(ins.size(), 0);
    }
    void addOutputs(vector<OutNeuron*>* addedOut) {
        for (unsigned i = 0; i < addedOut->size(); i++)
            outs.push_back((*addedOut)[i]);
        outValues.resize(outs.size(), 0);
    }

    OutNeuron* newOutput() {
        OutNeuron* o = new OutNeuron();
        outs.push_back(o);
        outValues.resize(outs.size(), 0);
        return o;
    }

    InNeuron* getRandomInNeuron() {
        int i = irand(0, ins.size());
        return ins[i];
    }

    OutNeuron* getRandomOutNeuron() {
        int i = irand(0, outs.size());
        return outs[i];
    }

    Neuron* getRandomInterNeuron() {
        int i = irand(0, neurons.size());
        Neuron* n = NULL;
        for(map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (i >= 0) && (im != neurons.end()); im++, i--) {
            //cout << im->first << " " << im->second << endl;
            n = im->first;
        }
        return n;
    }

    void addRandomInputs(double minValue, double maxValue, double decay) {
        for (unsigned i = 0; i < ins.size(); i++) {
            InNeuron* n = ins[i];
            n->setInput((n->getInput() + frand(minValue, maxValue)) * decay);
        }

    }

    void resetInputs() {
        for (unsigned i = 0; i < ins.size(); i++) {
            InNeuron* n = ins[i];
            n->setInput(0.0);
        }
    }

    void resetOutputs() {
        for (unsigned i = 0; i < outs.size(); i++) {
            OutNeuron* n = outs[i];
            n->reset();
        }
    }

    void wireRandomly(unsigned minSynapsesPerNeuron, unsigned maxSynapsesPerNeuron, float percentInhibitoryNeuron, float percentInputSynapse, float percentInhibitorySynapse, float percentOutputNeuron, float minSynapseWeight, float maxSynapseWeight, float neuronPotentialDecay) {
        float minPlasticityStrengthen = 1.001;
        float maxPlasticityStrengthen = 1.015;
        float minPlasticityWeaken = 0.985;
        float maxPlasticityWeaken = 0.999;

        float percentChanceConsistentSynapses = 0.0;

        float percentChancePlasticNeuron = 1.0;

        float minFiringThreshold = 0.01;
        float maxFiringThreshold = 0.9999;

        // determine number of neurons this brain will start with
        //int numNeurons = (int) Math.round(Maths.random(minNeuronsAtBuildtime, maxNeuronsAtBuildtime));


        // create the architectural neurons
//        for (unsigned i = 0; i < numNeurons; i++) {
//            NeuronBuilder *nb = newRandomNeuronBuilder();
//            neuronBuilders.push_back(nb);
//
//        }

        // clear everything
        //neuron.clear();
        //sense.clear();
        //motor.clear();

        // we know the amount of neurons already, reset totalsynapses for the count later



        // create all runtime neurons
        /*for (unsigned i = 0; i < numNeurons; i++)*/
        {
            //Neuron* t = new CritterdingNeuron();
            Neuron* t = new IzhikevichNeuron();

            // is it inhibitory ?
            if (frand(0, 1) <= percentInhibitoryNeuron) {
                t->isInhibitory = true;
            }// if not, is it motor ?
            else {
                t->isInhibitory = false;
            }

            if (frand(0, 1) <= percentOutputNeuron) {
                OutNeuron* mn = getRandomOutNeuron();

                // check if motor already used
                //            bool proceed = true;
                //            for (NeuronBuilder nb : neuronBuilders) {
                //                if (nb.motor == mn) {
                //                    proceed = false;
                //                    break;
                //                }
                //            }

                //if (proceed) {

                //Allows a motor neuron to receive multiple inputs
                t->target = mn;
                //}
            }
            else {
                t->target = NULL;
            }

            // does it have synaptic plasticity ?
            if (frand(0, 1) <= percentChancePlasticNeuron) {
                t->isPlastic = true;
                t->plasticityStrengthen = frand(minPlasticityStrengthen, maxPlasticityStrengthen);
                t->plasticityWeaken = frand(minPlasticityWeaken, maxPlasticityWeaken);
            } else {
                t->isPlastic = false;
                t->plasticityStrengthen = (minPlasticityStrengthen + maxPlasticityStrengthen)/2.0;
                t->plasticityWeaken = (minPlasticityWeaken + maxPlasticityWeaken) / 2.0;
            }

            // does it have consistent synapses ?
            if (frand(0, 1) <= percentChanceConsistentSynapses) {
                t->hasConsistentSynapses = true;

                // if so, does it have inhibitory synapses ?
                if (frand(0, 1) <= percentInhibitorySynapse) {
                    t->hasInhibitorySynapses = true;
                }
            }

            // determine firing threshold
            if (t->target != NULL) {
                t->firingThreshold = maxFiringThreshold;
                //an->maxDendridicBranches = maxDendridicBranches;
            } else {
                t->firingThreshold = frand(minFiringThreshold, maxFiringThreshold);
                //an->maxDendridicBranches = irand(1, maxDendridicBranches);
            }

            //maximum that a synapse can multiply a signal. 1.0 = conserved
            t->maxSynapseWeight = maxSynapseWeight;    //ex: 1.0
            t->minSynapseWeight = minSynapseWeight;    //ex: 0.1

            t->potentialDecay = neuronPotentialDecay; //ex: 0.995;

            addNeuron(t);

            // determine amount of synapses this neuron will start with
            int numSynapses = irand(minSynapsesPerNeuron, maxSynapsesPerNeuron);

            bool isSensorNeuron;
            // create the architectural neurons
            for (int j = 0; j < numSynapses; j++) {
                AbstractNeuron* inNeuron;
                if (frand(0, 1) <= percentInputSynapse || neurons.size() < 2) {
                    isSensorNeuron = true;
                    inNeuron = getRandomInNeuron();
                }
                else {
                    isSensorNeuron = false;
                    inNeuron = getRandomInterNeuron();
                }
                
                float weight;
                if (t->hasConsistentSynapses) {
                    weight = (t->hasInhibitorySynapses) ? -1.0 : 1.0;
                } else {
                    float percentChanceInhibitorySynapses = 0.5f;
                    weight = (frand(0, 1) <= percentChanceInhibitorySynapses) ? -1.0 : 1.0;
                }


                Synapse* s = new Synapse(inNeuron, weight);
                addSynapse(s, t);
                totalSynapses++;
            }


        }

    }

    bool addNeuron(Neuron* n) {
        neurons[n] = new list<Synapse*>();
        return true;
    }

    bool removeNeuron(Neuron* n) {
        //TODO remove all synapses that have an input==n
        neurons.erase(n);
        return true;
    }

    void addSynapse(Synapse* s, Neuron* target) {
        neurons[target]->push_back(s);
    }

    void removeSynapse(Synapse* s, Neuron* target) {
        //iterate through all neurons synapse lists and remove existence of 's'
        for (std::list<Synapse*>::iterator list_iter = neurons[target]->begin(); list_iter != neurons[target]->end(); list_iter++) {
            Synapse* e = *list_iter;
            if (s == e) {
                neurons[target]->erase(list_iter);
            }
        }

    }

    void removeSynapses(Neuron* target) {
        neurons[target]->clear();
    }

    void update(float dt);

    //    void forwardParallel(float dt) {
    //        //TODO handle 'dt' appropriately
    //
    //        // reset fired neurons counter
    //        neuronsFired = outNeuronsFired = 0;
    //
    //        resetOutputs();
    //
    //        unsigned i;
    //
    //        //#pragma omp parallel for private(i)
    //
    //        for (i = 0; i < neurons.size(); i++) {
    //            Neuron* n = neurons[i];
    //            n->forward(dt);
    //
    //            // if neuron fires
    //            if (n->nextOutput != 0) {
    //                neuronsFired++;
    //
    //                // motor neuron check & exec
    //                OutNeuron* mn = n->target;
    //                if (mn != NULL) {
    //                    outNeuronsFired++;
    //                    mn->stimulate(n->nextOutput);
    //                }
    //            }
    //        }
    //
    //        // commit outputs at the end
    //#pragma omp parallel for private(i)
    //        for (i = 0; i < neurons.size(); i++) {
    //            Neuron* n = neurons[i];
    //            n->output = n->nextOutput;
    //        }
    //
    //    }

    void printSummary() {
        int numNeurons = neurons.size();
        cout << "Brain" << '\n';
        cout << "  Neurons: " << numNeurons << '\n';
        cout << "  ins/outs: " << ins.size() << "|" << outs.size() << '\n';
        cout << "  Synapses: " << totalSynapses << '\n';
        cout << "  Avg Synapses/Neuron: " << ((float) totalSynapses) / ((float) numNeurons) << '\n';
//        cout << "    Min Synapses: " << minSynapses << '\n';
//        cout << "    Max Synapses: " << maxSynapses << '\n';
    }

    void print() {
        printSummary();

        cout << '\n';

        for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
            Neuron* n = im->first;
            n->print();
        }

    }

    void setPotentialDecay(double pd) {
        for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
            Neuron* n = im->first;
            n->potentialDecay = pd;
        }
        cout << "All neurons have potentialDecay=" << pd << "\n";
    }

    void setFiringThreshold(double ft) {
        for (map< Neuron*, list<Synapse*>* >::iterator im = neurons.begin(); (im != neurons.end()); im++) {
            Neuron* n = im->first;
            n->firingThreshold = ft;
        }
        cout << "All neurons have firingThreshold=" << ft << "\n";
    }

    unsigned getNumInputs() { return ins.size(); }
    unsigned getNumOutputs() { return outs.size(); }

};

#endif /* BRAIN_H_ */
