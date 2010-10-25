/* 
 * File:   NOutput.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:06 PM
 */

#ifndef _NOUTPUT_H
#define	_NOUTPUT_H

using namespace std;

#include <vector>

#include "Brain.h"

class NOutput {
public:

    vector<OutNeuron*> outs;

    NOutput(Brain* b, int size) {
        for (unsigned j = 0; j < size; j++) {
            OutNeuron* o = b->newOutput();
            outs.push_back(o);
        }
    }

    virtual void process(double dt) { }
    
    virtual ~NOutput() {

    }

};

#endif	/* _NOUTPUT_H */

