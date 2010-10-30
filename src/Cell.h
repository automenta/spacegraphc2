/* 
 * File:   Cell.h
 * Author: me
 *
 * Created on October 24, 2010, 1:20 AM
 */

#ifndef CELL_H
#define	CELL_H

#include <list>
using namespace std;

#include "Spacegraph.h"
#include "BodyProcess.h"
#include "btBulletDynamicsCommon.h"


class Cell
{

public:
        list<BodyProcess*>* processes;
	btDynamicsWorld* m_ownerWorld;
        Spacegraph* spacegraph;

        Cell (Spacegraph* s)		
	{
            spacegraph = s;
            m_ownerWorld = s->getSpace();
            processes = new list<BodyProcess*>();

	}
        
	virtual btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
            return addNewBody(mass, startTransform, shape, 0.5, 0.5, 0.5);
        }

	virtual btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape, float r, float g, float b) {
            return addNewBody(mass, startTransform, shape, new BodyProcess(r, g, b));
        }


	virtual btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape, BodyProcess* process, bool collidable=true)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

                if (collidable) {
                    m_ownerWorld->addRigidBody(body);
                }
                else {
                    ((btDiscreteDynamicsWorld*)m_ownerWorld)->addRigidBody(body, 0, 0);
                }

                spacegraph->setBodyProcess(body, process);

                processes->push_back(process);

		return body;
	}

        void removeBody(btRigidBody* body) {
            m_ownerWorld->removeRigidBody(body);

            BodyProcess* process = spacegraph->getProcess(body);
            if (process!=NULL) {
                processes->remove(process);
            }
            
            spacegraph->removeBodyProcess(body);
        }

        virtual void update(double dt) {
            list<BodyProcess*>::iterator p = processes->begin();
            while(p != processes->end()){
                (*p)->update(dt);
                p++;
            }
        }

        virtual void draw() { }
};


#endif	/* CELL_H */

