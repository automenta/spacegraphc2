/* 
 * File:   Cell.h
 * Author: me
 *
 * Created on October 24, 2010, 1:20 AM
 */

#ifndef CELL_H
#define	CELL_H

#include "Spacegraph.h"
#include "btBulletDynamicsCommon.h"


class Cell
{

public:

	btRigidBody* addNewBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

	btDynamicsWorld* m_ownerWorld;

        Cell (Spacegraph* s)
		
	{
            m_ownerWorld = s->getSpace();

	}

};


#endif	/* CELL_H */

