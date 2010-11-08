/* 
 * File:   CellProcess.h
 * Author: me
 *
 * Created on October 24, 2010, 8:25 PM
 */

#ifndef CELLPROCESS_H
#define	CELLPROCESS_H

class CellProcess {
public:
    CellProcess();

    virtual void update(double dt) {    }
    virtual void draw() {    }

    virtual ~CellProcess();
private:

};

#endif	/* CELLPROCESS_H */

