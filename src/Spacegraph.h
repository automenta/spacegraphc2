/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include <map>
using namespace std;

#include "btBulletDynamicsCommon.h"

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "Renderer.h"

#include "BodyProcess.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class Spacegraph : public GlutDemoApplication
{

        Renderer* renderer;

	btAlignedObjectArray<class Cell*> cells;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

        map<btRigidBody*, BodyProcess*>* bodyProcess;
        
        float rotationMomentum, distMomentum;
        float nextEle;
        float nextAzi;
        float nextDist;
        float minDist;

public:

        Spacegraph() : GlutDemoApplication() {
            renderer = new Renderer();
            m_shapeDrawer = renderer;
            bodyProcess = new map<btRigidBody*, BodyProcess*>();

            m_frustumZNear = 0.5f;
            rotationMomentum = 0.95f;
            distMomentum = 0.95f;
            minDist = 0.1f;
        }

        void setBodyProcess(btRigidBody* body, BodyProcess* process) {
            (*bodyProcess)[body] = process;
        }        
        
        void removeBodyProcess(btRigidBody* body) {
            bodyProcess->erase(body);
        }

        btDynamicsWorld* getSpace() {
            return m_dynamicsWorld;
        }

        void addCell(Cell *c);

        void initPhysics();

	void exitPhysics();

	virtual ~Spacegraph()
	{
            delete renderer;
            exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

        virtual void mouseFunc(int button, int state, int x, int y);

	virtual void mouseMotionFunc(int x,int y);
        
        virtual void updateCamera();

        virtual void renderme();
        
        virtual void renderscene(int pass);
        
	static DemoApplication* Create()
	{
		Spacegraph* demo = new Spacegraph();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
