/* 
 * File:   BrainLink.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:31 PM
 */

#ifndef _BRAINLINK_H
#define	_BRAINLINK_H

#include "NInput.h"
#include "NOutput.h"

/** adapts two brains in bidirectional communication. the relative width of the i/o channels can be balanced -1..+1 between the two brain "edges" */
class BrainLink : public NInput, NOutput {
public:
    BrainLink(Brain* b) : NInput(b, 0), NOutput(b, 0) { }

    virtual ~BrainLink() { }
private:

};

#endif	/* _BRAINLINK_H */

