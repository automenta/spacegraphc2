/* 
 * File:   NInput.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:06 PM
 */

#ifndef _NINPUT_H
#define	_NINPUT_H

using namespace std;
#include <vector>

#include "Brain.h"

class NInput {
    Brain* brain;
    
public:
    vector<InNeuron*> ins;

    NInput(Brain* b, int size) : brain(b) {
        for (unsigned int j = 0; j < size; j++) {
            InNeuron* i = b->newInput();
            ins.push_back(i);
        }
    }
    
    //sets neural inputs.  dt = delta time since last call
    virtual void process(double dt) { }
    
    virtual ~NInput() {
        
    }
    
private:

};

#endif	/* _NINPUT_H */

