/*
 * Neuron.h
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#ifndef NEURON_H_
#define NEURON_H_

using namespace std;
#include <list>

class AbstractNeuron {
public:

    virtual float getOutput() {
        return 0;
    }

};

class Synapse {
public:
    AbstractNeuron* inputNeuron; // InputNeuron's Output Pointer

    float weight; // its synaptic weight -1.0f <-> +1.0f

    void print() {
        cout << "       Synapse(input?=" << inputNeuron << ", weight=" << weight << ")\n";
    }

    Synapse(AbstractNeuron* _inputNeuron, float _synapticWeight) {
        inputNeuron = _inputNeuron;
        weight = _synapticWeight;
    }

//    void setInput(AbstractNeuron* input) {
//        inputNeuron = input;
//    }
    
    virtual float getInput() {
        return inputNeuron->getOutput();
    }

};

class InNeuron : public AbstractNeuron {
private:
    float input; //sensed input

public:

    InNeuron() {
        AbstractNeuron();
        input = 0;
    }

    float getInput() {
        return input;
    }

    void setInput(float i) {
        if (i > 1.0) i = 1.0;
        if (i < -1.0) i = -1.0;
        input = i;
    }

    virtual float getOutput() {
        //cout << "inNeuron gotOutput: " << input << "\n";
        return input;
    }
};

class OutNeuron : public AbstractNeuron {
public:

    float potential;
    float stimulationFactor; //sensitivity to input stimulus.  defines the discreteness or detail level of its output spectrum
    float potentialDecay;

    OutNeuron() {
        OutNeuron(0, 0);
    }

    OutNeuron(float _stimulationFactor, float _potentialDecay) {
        AbstractNeuron();
        potential = 0;
        stimulationFactor = _stimulationFactor;
        potentialDecay = _potentialDecay;
    }

    void setStimulationFactor(double newStimulationFactor) {
        stimulationFactor = newStimulationFactor;
    }

    void setDecay(double newDecay) {
        potentialDecay = newDecay;
    }

    void reset() {
        potential = 0;
    }

    void stimulate(float pulse) {
        potential = (potential*potentialDecay) + pulse * stimulationFactor;
        if (potential > 1.0) potential = 1.0;
        if (potential < -1.0) potential = -1.0;
    }

    virtual float getOutput() {
        return potential;
    }
};

class SynapseBuilder {
public:
    // determines if id referes to an interneuron or sensorneuron
    bool isSensorNeuron;

    // id of neuron which axon connects to this synapse
    //InterNeuron neuron;

    //int realneuronID;
    int neurontargetlayer;

    // its "weight"
    float weight;
    float percentChanceInhibitorySynapses;

    AbstractNeuron* inNeuron;

    SynapseBuilder() {
        isSensorNeuron = false;
        neurontargetlayer = 0;
        weight = 0.0F;
        percentChanceInhibitorySynapses = 0.5f;
    }



};

/** inter-neuron */
class Neuron : public OutNeuron {
public:
    OutNeuron* target;

    // Consistent Synapses flag
    bool hasConsistentSynapses;
    // inhibitory synapses flag
    bool hasInhibitorySynapses;


    float output, nextOutput;
    bool isInhibitory;
    bool active;
    float firingThreshold;
    bool isPlastic;
    float plasticityStrengthen;
    float plasticityWeaken;
    float maxSynapseWeight;
    float minSynapseWeight;

    void print() {
        cout << "  Neuron" << '\n';
        cout << "    output? " << (target != NULL) << '\n';
        cout << "    inhibitory? " << isInhibitory << '\n';
        cout << "    plastic? " << isPlastic << '\n';
        cout << "    maxSynapticWeightMagnitude? " << maxSynapseWeight << '\n';
        cout << "    plasticity strengthen=" << plasticityStrengthen << ", weaken=" << plasticityWeaken << '\n';
        cout << "    potential decay " << potentialDecay << '\n';
        cout << "    firing threshold " << firingThreshold << '\n';

//        for (unsigned i = 0; i < synapses.size(); i++) {
//            Synapse* s = synapses[i];
//            s->print();
//        }
    }

    Neuron() {
        isInhibitory = false;

        active = true;

        isPlastic = false; // plasticity up & down

        potential = 0.0;
        output = nextOutput = 0;

        target = NULL;
    }

    void setActive(bool nextActive) {
        active = nextActive;
    }

    virtual double forward(float dt, list<Synapse*>* synapses) = 0;

    virtual float getOutput() {
        return output;
    }
    
};

class CritterdingNeuron : public Neuron {
    //returns how much weight has been changed
    virtual double forward(float dt, list<Synapse*>* synapses) {
        double learnedTotal = 0;

        if (!active) {
            potential = 0.0f;
            nextOutput = 0.0f;
        }

        //TODO handle 'dt' appropriately

        // potential decay
        potential *= potentialDecay;

        // make every connection do it's influence on the neuron's total potential
        for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
            Synapse* s = *list_iter;

            // lower synaptic weights
            if (isPlastic) {
                double preWeight = s->weight;

                s->weight *= plasticityWeaken;

                clampWeight(s);

                learnedTotal += fabs(s->weight - preWeight);

            }

            potential += s->weight * s->getInput() /** s->dendriteBranch * */;
            //cout << "Synapse " << s->weight << " " << s->getInput() << " :: pot=" << potential << "\n";
        }

        if (isInhibitory) {
            learnedTotal += forwardInhibitory(synapses);
        } else {
            learnedTotal += forwardExhibitory(synapses);
        }

        return learnedTotal;
    }

    double forwardInhibitory(list<Synapse*>* synapses) {
        double learnedTotal = 0;
        // do we spike/fire
        if (potential <= -1.0f * firingThreshold) {
            // reset neural potential
            //potential = 0.0f;
            potential += firingThreshold;

            // fire the neuron
            nextOutput = -firingThreshold;

            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
            if (isPlastic) {
                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
                    Synapse* s = *list_iter;
                    double o = s->getInput();

                    double preWeight = s->weight;
                    // if synapse fired, strengthen the weight
                    if ((o < 0.0f && s->weight > 0.0f) || (o > 0.0f && s->weight < 0.0f)) {
                        s->weight *= plasticityStrengthen;
                    }

                    // clamp weight
                    clampWeight(s);

                    learnedTotal += fabs(preWeight - s->weight);
                }
            }
        }// don't fire the neuron
        else {
            nextOutput = 0;
            // reset potential if < 0
            if (potential > 0.0f) {
                potential = 0.0f;
            }
        }
        return learnedTotal;
    }

    double forwardExhibitory(list<Synapse*>* synapses) {
        double learnedTotal = 0;
        // do we spike/fire
        if (potential >= +1.0f * firingThreshold) {
            // reset neural potential
            //potential = 0.0f;
            potential -= firingThreshold;

            // fire the neuron
            nextOutput = firingThreshold;

            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
            if (isPlastic) {
                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
                    Synapse* s = *list_iter;
                    double o = s->getInput();

                    double preWeight = s->weight;
                    // if synapse fired, strengthen the weight
                    if ((o > 0.0f && s->weight > 0.0f) || (o < 0.0f && s->weight < 0.0f)) {
                        s->weight *= plasticityStrengthen;
                    }

                    // if weight > max back to max
                    clampWeight(s);

                    learnedTotal += fabs(preWeight - s->weight);
                }
            }
        }// don't fire the neuron
        else {
            nextOutput = 0;
            // reset potential if < 0
            if (potential < 0.0f) {
                potential = 0.0f;
            }
        }
        return learnedTotal;
    }

    void clampWeight(Synapse* s) {
        //WARNING this may be incorrect
        s->weight = fmax(fmin(s->weight, maxSynapseWeight), minSynapseWeight);
    }

};

class IzhikevichNeuron : public Neuron {
public:

    /** Recovery. */
    double recovery;

    /** A. */
    double a;

    /** B. */
    double b;

    /** C. */
    double c;

    /** D. */
    double d;

    //    /** Noise dialog. */
    //    private RandomSource noiseGenerator = new RandomSource();

    /** Add noise to the neuron. */
    bool addNoise, hasSpiked;

    /** Minimum value this neuron can take. */
    double lowerBound;

    /** Maximum value  this neuron can take. */
    double upperBound;

    /** Amount by which to increment or decrement neuron. */
    double increment;


    /** Value of any external inputs to neuron. */
    double inputValue;

    /** If true then do not update this neuron. */
    bool clamped;

    /** Target value. */
    double targetValue;

    IzhikevichNeuron() : Neuron() {

        recovery = 0;
        a = .02;
        b = .2;
        c = -65;
        d = 6;

        addNoise = false;
        hasSpiked = false;

        output = nextOutput = 0;

        lowerBound = -1;
        upperBound = 1;

        increment = .1;

        inputValue = 0;

        clamped = false;

        targetValue = 0;

    }

    /**
     * Default constructor needed for external calls which create neurons then
     * set their parameters.
     */

    virtual double forward(float dt, list<Synapse*>* fanIn) {
        double learnedTotal = 0;

        for(std::list<Synapse*>::iterator list_iter = fanIn->begin(); list_iter != fanIn->end(); list_iter++) {
            Synapse* s = *list_iter;
            // lower synaptic weights

            double preWeight = s->weight;
            if (isPlastic) {
                s->weight *= plasticityWeaken;
                clampWeight(s);
                learnedTotal += fabs(s->weight - preWeight);
            }
        }

        double inputs = getWeightedInputs(fanIn);


        if (addNoise) {
            //inputs += noiseGenerator.getRandom();
        }

        recovery += (dt * (a * ((b * output) - recovery)));

        double val = output
                + (dt * (((.04 * (output * output))
                + (5 * output) + 140)
                - recovery + inputs));

        if (val > 30) {
            val = c;
            recovery += d;
            hasSpiked = true;
        } else {
            hasSpiked = false;
        }

        if (hasSpiked) {
            if (isPlastic) {
                //adjust synaptic weights
                if (isInhibitory) {
                    for(std::list<Synapse*>::iterator list_iter = fanIn->begin(); list_iter != fanIn->end(); list_iter++) {
                        Synapse* s = *list_iter;
                        double o = s->getInput();

                        double preWeight = s->weight;
                        // if synapse fired, strengthen the weight
                        if ((o < 0.0f && s->weight > 0.0f) || (o > 0.0f && s->weight < 0.0f)) {
                            s->weight *= plasticityStrengthen;
                            learnedTotal += fabs(preWeight - s->weight);
                        }

                        // clamp weight
                        clampWeight(s);

                        //printf("-learned: (%f,%f) %f\n", plasticityWeaken, plasticityStrengthen, fabs(preWeight - s->weight));
                    }

                } else {
                    for (std::list<Synapse*>::iterator list_iter = fanIn->begin(); list_iter != fanIn->end(); list_iter++) {
                        Synapse* s = *list_iter;
                        double o = s->getInput();

                        double preWeight = s->weight;
                        // if synapse fired, strengthen the weight
                        if ((o > 0.0f && s->weight > 0.0f) || (o < 0.0f && s->weight < 0.0f)) {
                            s->weight *= plasticityStrengthen;
                            learnedTotal += fabs(preWeight - s->weight);
                        }

                        // if weight > max back to max
                        clampWeight(s);

                        //printf("+learned: (%f,%f) %f\n", plasticityWeaken, plasticityStrengthen, fabs(preWeight - s->weight));
                    }
                }


            }
        }


        nextOutput = val;

        return learnedTotal;
    }

    void clampWeight(Synapse* s) {
        //WARNING this may be incorrect
        s->weight = fmax(fmin(s->weight, maxSynapseWeight), minSynapseWeight);
    }


    //    //returns how much weight has been changed
    //    virtual double forward(float dt, list<Synapse*>* synapses) {
    //        double learnedTotal = 0;
    //
    //        if (!active) {
    //            potential = 0.0f;
    //            nextOutput = 0.0f;
    //        }
    //
    //        //TODO handle 'dt' appropriately
    //
    //        // potential decay
    //        potential *= potentialDecay;
    //
    //        // make every connection do it's influence on the neuron's total potential
    //        for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
    //            Synapse* s = *list_iter;
    //
    //            // lower synaptic weights
    //            if (isPlastic) {
    //                double preWeight = s->weight;
    //
    //                s->weight *= plasticityWeaken;
    //
    //                learnedTotal += fabs(s->weight - preWeight);
    //            }
    //
    //            potential += s->weight * s->getInput() /** s->dendriteBranch * */;
    //            //cout << "Synapse " << s << " " << s->getInput() << " :: pot=" << potential << "\n";
    //        }
    //
    //        if (isInhibitory) {
    //            learnedTotal += forwardInhibitory(synapses);
    //        } else {
    //            learnedTotal += forwardExhibitory(synapses);
    //        }
    //
    //        return learnedTotal;
    //    }
    //
    //    double forwardInhibitory(list<Synapse*>* synapses) {
    //        double learnedTotal = 0;
    //        // do we spike/fire
    //        if (potential <= -1.0f * firingThreshold) {
    //            // reset neural potential
    //            //potential = 0.0f;
    //            potential += firingThreshold;
    //
    //            // fire the neuron
    //            nextOutput = -firingThreshold;
    //
    //            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
    //            if (isPlastic) {
    //                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
    //                    Synapse* s = *list_iter;
    //                    double o = s->getInput();
    //
    //                    double preWeight = s->weight;
    //                    // if synapse fired, strengthen the weight
    //                    if ((o < 0.0f && s->weight > 0.0f) || (o > 0.0f && s->weight < 0.0f)) {
    //                        s->weight *= plasticityStrengthen;
    //                    }
    //
    //                    // clamp weight
    //                    clampWeight(s);
    //
    //                    learnedTotal += fabs(preWeight - s->weight);
    //                }
    //            }
    //        }// don't fire the neuron
    //        else {
    //            nextOutput = 0;
    //            // reset potential if < 0
    //            if (potential > 0.0f) {
    //                potential = 0.0f;
    //            }
    //        }
    //        return learnedTotal;
    //    }
    //
    //    double forwardExhibitory(list<Synapse*>* synapses) {
    //        double learnedTotal = 0;
    //        // do we spike/fire
    //        if (potential >= +1.0f * firingThreshold) {
    //            // reset neural potential
    //            //potential = 0.0f;
    //            potential -= firingThreshold;
    //
    //            // fire the neuron
    //            nextOutput = firingThreshold;
    //
    //            // PLASTICITY: if neuron & synapse fire together, the synapse strenghtens
    //            if (isPlastic) {
    //                for(std::list<Synapse*>::iterator list_iter = synapses->begin(); list_iter != synapses->end(); list_iter++) {
    //                    Synapse* s = *list_iter;
    //                    double o = s->getInput();
    //
    //                    double preWeight = s->weight;
    //                    // if synapse fired, strengthen the weight
    //                    if ((o > 0.0f && s->weight > 0.0f) || (o < 0.0f && s->weight < 0.0f)) {
    //                        s->weight *= plasticityStrengthen;
    //                    }
    //
    //                    // if weight > max back to max
    //                    clampWeight(s);
    //
    //                    learnedTotal += fabs(preWeight - s->weight);
    //                }
    //            }
    //        }// don't fire the neuron
    //        else {
    //            nextOutput = 0;
    //            // reset potential if < 0
    //            if (potential < 0.0f) {
    //                potential = 0.0f;
    //            }
    //        }
    //        return learnedTotal;
    //    }

    //    void clampWeight(Synapse* s) {
    //        s->weight = fmax(fmin(s->weight, maxSynapseWeight), minSynapseWeight);
    //    }



    //    /**
    //     * @return the fan in array list.
    //     */
    //    public List<Synapse> getFanIn() {
    //        return fanIn;
    //    }
    //
    //    /**
    //     * @return the fan out array list.
    //     */
    //    public List<Synapse> getFanOut() {
    //        return fanOut;
    //    }

    //    /**
    //     * Increment this neuron by increment.
    //     */
    //    void incrementActivation() {
    //        if (activation < upperBound) {
    //            activation += increment;
    //        }
    //    }
    //
    //    /**
    //     * Decrement this neuron by increment.
    //     */
    //    void decrementActivation() {
    //        if (activation > lowerBound) {
    //            activation -= increment;
    //        }
    //    }

    //    /**
    //     * Connect this neuron to target neuron via a weight.
    //     *
    //     * @param target the connnection between this neuron and a target neuron
    //     */
    //    void addTarget(final Synapse target) {
    //        fanOut.add(target);
    //    }
    //
    //    /**
    //     * Remove this neuron from target neuron via a weight.
    //     *
    //     * @param target the connnection between this neuron and a target neuron
    //     */
    //    void removeTarget(final Synapse target) {
    //        fanOut.remove(target);
    //    }
    //
    //    /**
    //     * Connect this neuron to source neuron via a weight.
    //     *
    //     * @param source the connnection between this neuron and a source neuron
    //     */
    //    void addSource(final Synapse source) {
    //        fanIn.add(source);
    //    }
    //
    //    /**
    //     * Remove this neuron from source neuron via a weight.
    //     *
    //     * @param source the connnection between this neuron and a source neuron
    //     */
    //    void removeSource(final Synapse source) {
    //        fanIn.remove(source);
    //    }

    // not used.  Consider deleting?
    //    /**
    //     * Add specified amount of activation to this neuron.
    //     *
    //     * @param amount amount to add to this neuron
    //     */
    //    public void addActivation(final double amount) {
    //        activation += amount;
    //    }

    /**
     * Sums the weighted signals that are sent to this node.
     *
     * @return weighted input to this node
     */
    double getWeightedInputs(list<Synapse*>* fanIn) {
        double wtdSum = inputValue;
        if (fanIn->size() > 0) {
            for (std::list<Synapse*>::iterator list_iter = fanIn->begin(); list_iter != fanIn->end(); list_iter++) {
                Synapse* w = *list_iter;
                wtdSum += w->weight * w->getInput();
            }
        }

        return wtdSum;
    }

    /**
     * Returns a random value between the upper and lower bounds of this neuron.
     * @return the random value.
     */
    double getRandomValue() {
        return (upperBound - lowerBound) * frand(0, 1) + lowerBound;
    }


    /*
     * Update all neurons n this neuron is connected to, by adding current activation
     * times the connection-weight  NOT CURRENTLY USED.
     */
    //    public void updateConnectedOutward() {
    //        // Update connected weights
    //        if (fanOut.size() > 0) {
    //            for (int j = 0; j < fanOut.size(); j++) {
    //                Synapse w = (Synapse) fanOut.get(j);
    //                Neuron target = w.getTarget();
    //                target.setActivation(w.getStrength() * activation);
    //                target.checkBounds();
    //            }
    //        }
    //    }

    /**
     * Check if this neuron is connected to a given weight.
     *
     * @param w weight to check
     *
     * @return true if this neuron has w in its fan_in or fan_out
     */
    //    public boolean connectedToWeight(final Synapse w) {
    //        if (fanOut.size() > 0) {
    //            for (int j = 0; j < fanOut.size(); j++) {
    //                Synapse outW = (Synapse) fanOut.get(j);
    //
    //                if (w.equals(outW)) {
    //                    return true;
    //                }
    //            }
    //        }
    //
    //        if (fanIn.size() > 0) {
    //            for (int j = 0; j < fanIn.size(); j++) {
    //                Synapse inW = (Synapse) fanIn.get(j);
    //
    //                if (w.equals(inW)) {
    //                    return true;
    //                }
    //            }
    //        }
    //
    //        return false;
    //    }

    //    /**
    //     * Round the activation level of this neuron off to a specified precision.
    //     *
    //     * @param precision precision to round this neuron's activation off to
    //     */
    //    public void round(final int precision) {
    //        setActivation(Network.round(getActivation(), precision));
    //    }

    /**
     * If activation is above or below its bounds set it to those bounds.
     */
    void checkBounds() {
        output = clip(output);
    }

    /**
     * If value is above or below its bounds set it to those bounds.
     * @param value Value to check
     * @return clip
     */
    double clip(double value) {
        double val = value;
        if (val > upperBound) {
            val = upperBound;
        }

        if (val < lowerBound) {
            val = lowerBound;
        }

        return val;
    }

    //    /**
    //     * Sends relevant information about the network to standard output. TODO: Change to toString()
    //     */
    //    public void debug() {
    //        System.out.println("neuron " + id);
    //        System.out.println("fan in");
    //
    //        for (int i = 0; i < fanIn.size(); i++) {
    //            Synapse tempRef = (Synapse) fanIn.get(i);
    //            System.out.println("fanIn [" + i + "]:" + tempRef);
    //        }
    //
    //        System.out.println("fan out");
    //
    //        for (int i = 0; i < fanOut.size(); i++) {
    //            Synapse tempRef = (Synapse) fanOut.get(i);
    //            System.out.println("fanOut [" + i + "]:" + tempRef);
    //        }
    //    }


    /**
     * Returns the sum of the strengths of the weights attaching to this neuron.
     *
     * @return the sum of the incoming weights to this neuron.
     */
    //    double getSummedIncomingWeights(List<RealtimeSynapse> fanIn) {
    //        double ret = 0;
    //
    //        for (int i = 0; i < fanIn.size(); i++) {
    //            RealtimeSynapse tempRef = (RealtimeSynapse) fanIn.get(i);
    //            ret += tempRef.getStrength();
    //        }
    //
    //        return ret;
    //    }

    //    /**
    //     * Returns the number of neurons attaching to this one which have activity above
    //     * a specified threshold.
    //     *
    //     * @param threshold value above which neurons are considered "active."
    //     * @return number of "active" neurons
    //     */
    //    int getNumberOfActiveInputs(List<RealtimeSynapse> fanIn, final int threshold) {
    //        int numActiveLines = 0;
    //        // Determine number of active (greater than 0) input lines
    //        for (RealtimeSynapse incoming : fanIn) {
    //            if (incoming.getSource().getActivation() > threshold) {
    //                numActiveLines++;
    //            }
    //        }
    //        return numActiveLines;
    //    }
    //
    //    /**
    //     * @return the average activation of neurons connecting to this neuron
    //     */
    //    double getAverageInput(List<RealtimeSynapse> fanIn) {
    //        return getTotalInput(fanIn) / ((double)fanIn.size());
    //    }
    //
    //    /**
    //     * @return the total activation of neurons connecting to this neuron
    //     */
    //    double getTotalInput(List<RealtimeSynapse> fanIn) {
    //        double ret = 0;
    //
    //        for (int i = 0; i < fanIn.size(); i++) {
    //            ret += ((RealtimeSynapse) fanIn.get(i)).getSource().getActivation();
    //        }
    //
    //        return ret;
    //    }

    //    /**
    //     * TODO:
    //     * Check if any couplings attach to this world and if there are no none, remove the listener.
    //     * @param world
    //     */
    //    private void removeWorldListener(World world) {
    //
    //    }

    /**
     * Return true if this neuron has a motor coupling attached.
     *
     * @return true if this neuron has a motor coupling attached
     */
    bool isOutput() {
        return false;
        //        return (motorCoupling != null);
    }

    /**
     * Return true if this neuron has a sensory coupling attached.
     *
     * @return true if this neuron has a sensory coupling attached
     */
    bool isInput() {
        return false;
        //  return (sensoryCoupling != null);
    }

    //    /**
    //     * True if the synapse is connected to this neuron, false otherwise.
    //     * @param s the synapse to check.
    //     * @return true if synapse is connected, false otherwise.
    //     */
    //    bool isConnected(List<RealtimeSynapse> fanIn, List<RealtimeSynapse> fanOut, final RealtimeSynapse s) {
    //        return (fanIn.contains(s) || fanOut.contains(s));
    //    }

    /**
     * Set activation to 0; override for other "clearing" behavior.
     */
    void clear() {
        output = nextOutput = 0;
    }


};



#endif /* NEURON_H_ */
