/* 
 * File:   Cell.h
 * Author: me
 *
 * Created on October 24, 2010, 1:20 AM
 */

#ifndef CELL_H
#define	CELL_H

#include "Spacegraph.h"
#include "BodyProcess.h"
#include "btBulletDynamicsCommon.h"


class Cell
{

public:
	btDynamicsWorld* m_ownerWorld;
        Spacegraph* spacegraph;



        Cell (Spacegraph* s)		
	{
            spacegraph = s;
            m_ownerWorld = s->getSpace();

	}
	btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
            return addNewBody(mass, startTransform, shape, 0.5, 0.5, 0.5);
        }

	btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape, float r, float g, float b) {
            return addNewBody(mass, startTransform, shape, new BodyProcess(r, g, b));
        }


	btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape, BodyProcess* process)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);
                spacegraph->setBodyProcess(body, process);

		return body;
	}

        void removeBody(btRigidBody* body) {
            m_ownerWorld->removeRigidBody(body);
            spacegraph->removeBodyProcess(body);
        }
};


#endif	/* CELL_H */

