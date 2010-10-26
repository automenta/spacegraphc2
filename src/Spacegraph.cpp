/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
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



#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "Spacegraph.h"
#include "Cell.h"

#include "BodyProcess.h"

static bool use6Dof = false;

// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

void Spacegraph::initPhysics() {
    // Setup the basic world

    setTexturing(true);
    setShadows(true);

    setCameraDistance(btScalar(5.));

    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldAabbMin(-10000, -10000, -10000);
    btVector3 worldAabbMax(10000, 10000, 10000);
    m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    //m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
    //m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;

    getSpace()->setGravity(btVector3(0,0,0));


    clientResetScene();

    nextEle = m_ele;
    nextAzi = m_azi;
    nextDist = m_cameraDistance;

}

void Spacegraph::addGround() {
    // Setup a big ground box
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.), btScalar(10.), btScalar(200.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -10, 0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
        btCollisionObject* fixedGround = new btCollisionObject();
        fixedGround->setCollisionShape(groundShape);
        fixedGround->setWorldTransform(groundTransform);
        m_dynamicsWorld->addCollisionObject(fixedGround);
#else
        localCreateRigidBody(btScalar(0.), groundTransform, groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

    }

}

void Spacegraph::addCell(Cell *c) {
    cells.push_back(c);
}


void Spacegraph::clientMoveAndDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //simple dynamics world doesn't handle fixed-time-stepping
    float ms = getDeltaTimeMicroseconds();

    float minFPS = 1000000.f / 60.f;
    if (ms > minFPS)
        ms = minFPS;

    double dt = ms / 1000000.f;
    if (m_dynamicsWorld) {
        m_dynamicsWorld->stepSimulation(dt);

        //optional but useful: debug drawing
        m_dynamicsWorld->debugDrawWorld();

    }

    for (unsigned i = 0; i < cells.size(); i++) {
        cells[i]->update(dt);
    }

    renderme();

    glFlush();

    glutSwapBuffers();
}

float lerp(float current, float next, float momentum) {
    return (momentum) * current + (1.0 - momentum) * next;
}

void Spacegraph::updateCamera() {


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    m_ele = lerp(m_ele, nextEle, rotationMomentum);
    m_azi = lerp(m_azi, nextAzi, rotationMomentum);
    m_cameraDistance = lerp(m_cameraDistance, nextDist, distMomentum);

    btScalar rele = m_ele * btScalar(0.01745329251994329547); // rads per deg
    btScalar razi = m_azi * btScalar(0.01745329251994329547); // rads per deg


    btQuaternion rot(m_cameraUp, razi);


    btVector3 eyePos(0, 0, 0);
    eyePos[m_forwardAxis] = -m_cameraDistance;

    btVector3 forward(eyePos[0], eyePos[1], eyePos[2]);
    if (forward.length2() < SIMD_EPSILON) {
        forward.setValue(1.f, 0.f, 0.f);
    }
    btVector3 right = m_cameraUp.cross(forward);
    btQuaternion roll(right, -rele);

    eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

    m_cameraPosition[0] = eyePos.getX();
    m_cameraPosition[1] = eyePos.getY();
    m_cameraPosition[2] = eyePos.getZ();
    m_cameraPosition += m_cameraTargetPosition;

    if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
        return;

    btScalar aspect;
    btVector3 extents;

    if (m_glutScreenWidth > m_glutScreenHeight) {
        aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;
        extents.setValue(aspect * 1.0f, 1.0f, 0);
    } else {
        aspect = m_glutScreenHeight / (btScalar) m_glutScreenWidth;
        extents.setValue(1.0f, aspect * 1.f, 0);
    }


    if (m_ortho) {
        // reset matrix
        glLoadIdentity();


        extents *= m_cameraDistance;
        btVector3 lower = m_cameraTargetPosition - extents;
        btVector3 upper = m_cameraTargetPosition + extents;
        //gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
        glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(), -1000, 1000);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        //glTranslatef(100,210,0);
    } else {
        if (m_glutScreenWidth > m_glutScreenHeight) {
            //			glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
            glFrustum(-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
        } else {
            //			glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
            glFrustum(-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
        }
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
                m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2],
                m_cameraUp.getX(), m_cameraUp.getY(), m_cameraUp.getZ());
    }

}

void Spacegraph::displayCallback() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateCamera();
    renderme();

    //optional but useful: debug drawing
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    glFlush();
    glutSwapBuffers();
}

void Spacegraph::keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
//        case 'e':
//        {
//            btVector3 startOffset(0, 2, 0);
//            spawnRagdoll(startOffset);
//            break;
//        }
        default:
            DemoApplication::keyboardCallback(key, x, y);
    }


}

int xPickingConstraintId = 0;
btVector3 xOldPickingPos;
float xOldPickingDist = 0.f;
btVector3 touchPos(-1, -1, -1);
btRigidBody* touchedBody = 0; //for deactivation state

void Spacegraph::mouseFunc(int button, int state, int x, int y) {
    btScalar mousePickClampingX = 30.f;

    if (state == 0) {
        m_mouseButtons |= 1 << button;
    } else {
        m_mouseButtons = 0;
    }

    m_mouseOldX = x;
    m_mouseOldY = y;

    updateModifierKeys();
    if ((m_modifierKeys & BT_ACTIVE_ALT) && (state == 0)) {
        return;
    }

    //printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
    //button 0, state 0 means left mouse down

    btVector3 rayTo = getRayTo(x, y);

    switch (button) {
        case 2:
        {
            
//            if (state == 0) {
//
//                shootBox(rayTo);
//            }
            break;
        };
        case 1:
        {


            if (state == 0) {

#if 0
                //apply an impulse
                if (m_dynamicsWorld) {
                    btCollisionWorld::ClosestRayResultCallback rayCallback(m_cameraPosition, rayTo);
                    m_dynamicsWorld->rayTest(m_cameraPosition, rayTo, rayCallback);
                    if (rayCallback.hasHit()) {

                        btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
                        if (body) {
                            body->setActivationState(ACTIVE_TAG);
                            btVector3 impulse = rayTo;
                            impulse.normalize();
                            float impulseStrength = 10.f;
                            impulse *= impulseStrength;
                            btVector3 relPos = rayCallback.m_hitPointWorld - body->getCenterOfMassPosition();
                            body->applyImpulse(impulse, relPos);
                        }
                    }
                }
#endif



            } else {

            }
            break;
        }
        case 0:
        {
            if (state == 0) {


                //add a point to point constraint for picking
                if (m_dynamicsWorld) {

                    btVector3 rayFrom;
                    if (m_ortho) {
                        rayFrom = rayTo;
                        rayFrom.setZ(-100.f);
                    } else {
                        rayFrom = m_cameraPosition;
                    }

                    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
                    m_dynamicsWorld->rayTest(rayFrom, rayTo, rayCallback);
                    if (rayCallback.hasHit()) {


                        btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
                        if (body) {
                            //other exclusions?
                            if (!(body->isStaticObject() || body->isKinematicObject())) {
                                touchedBody = body;
                                touchedBody->setActivationState(DISABLE_DEACTIVATION);


                                btVector3 pickPos = rayCallback.m_hitPointWorld;
                                printf("pickPos=%f,%f,%f\n", pickPos.getX(), pickPos.getY(), pickPos.getZ());


                                btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;






                                if (use6Dof) {
                                    btTransform tr;
                                    tr.setIdentity();
                                    tr.setOrigin(localPivot);
                                    btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*body, tr, false);
                                    dof6->setLinearLowerLimit(btVector3(0, 0, 0));
                                    dof6->setLinearUpperLimit(btVector3(0, 0, 0));
                                    dof6->setAngularLowerLimit(btVector3(0, 0, 0));
                                    dof6->setAngularUpperLimit(btVector3(0, 0, 0));

                                    m_dynamicsWorld->addConstraint(dof6);
                                    m_pickConstraint = dof6;

                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 0);
                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 1);
                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 2);
                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 3);
                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 4);
                                    dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.8, 5);

                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 0);
                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 1);
                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 2);
                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 3);
                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 4);
                                    dof6->setParam(BT_CONSTRAINT_STOP_ERP, 0.1, 5);
                                } else {
                                    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                                    m_dynamicsWorld->addConstraint(p2p);
                                    m_pickConstraint = p2p;
                                    p2p->m_setting.m_impulseClamp = mousePickClampingX;
                                    //very weak constraint for picking
                                    p2p->m_setting.m_tau = 0.001f;
                                    /*
                                                                                                            p2p->setParam(BT_CONSTRAINT_CFM,0.8,0);
                                                                                                            p2p->setParam(BT_CONSTRAINT_CFM,0.8,1);
                                                                                                            p2p->setParam(BT_CONSTRAINT_CFM,0.8,2);
                                                                                                            p2p->setParam(BT_CONSTRAINT_ERP,0.1,0);
                                                                                                            p2p->setParam(BT_CONSTRAINT_ERP,0.1,1);
                                                                                                            p2p->setParam(BT_CONSTRAINT_ERP,0.1,2);
                                     */


                                }
                                use6Dof = !use6Dof;

                                //save mouse position for dragging
                                xOldPickingPos = rayTo;
                                touchPos = pickPos;

                                xOldPickingDist = (pickPos - rayFrom).length();
                            }
                        }
                    }
                }

            } else {

                if (m_pickConstraint && m_dynamicsWorld) {
                    m_dynamicsWorld->removeConstraint(m_pickConstraint);
                    delete m_pickConstraint;
                    //printf("removed constraint %i",xPickingConstraintId);
                    m_pickConstraint = 0;
                    touchedBody->forceActivationState(ACTIVE_TAG);
                    touchedBody->setDeactivationTime(0.f);
                    touchedBody = 0;
                }


            }

            break;

        }
        default:
        {
        }
    }

}

void Spacegraph::mouseMotionFunc(int x, int y) {

    if (m_pickConstraint) {
        //move the constraint pivot

        if (m_pickConstraint->getConstraintType() == D6_CONSTRAINT_TYPE) {
            btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*> (m_pickConstraint);
            if (pickCon) {
                //keep it at the same picking distance

                btVector3 newRayTo = getRayTo(x, y);
                btVector3 rayFrom;
                btVector3 oldPivotInB = pickCon->getFrameOffsetA().getOrigin();

                btVector3 newPivotB;
                if (m_ortho) {
                    newPivotB = oldPivotInB;
                    newPivotB.setX(newRayTo.getX());
                    newPivotB.setY(newRayTo.getY());
                } else {
                    rayFrom = m_cameraPosition;
                    btVector3 dir = newRayTo - rayFrom;
                    dir.normalize();
                    dir *= xOldPickingDist;

                    newPivotB = rayFrom + dir;
                }
                pickCon->getFrameOffsetA().setOrigin(newPivotB);
            }

        } else {
            btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*> (m_pickConstraint);
            if (pickCon) {
                //keep it at the same picking distance

                btVector3 newRayTo = getRayTo(x, y);
                btVector3 rayFrom;
                btVector3 oldPivotInB = pickCon->getPivotInB();
                btVector3 newPivotB;
                if (m_ortho) {
                    newPivotB = oldPivotInB;
                    newPivotB.setX(newRayTo.getX());
                    newPivotB.setY(newRayTo.getY());
                } else {
                    rayFrom = m_cameraPosition;
                    btVector3 dir = newRayTo - rayFrom;
                    dir.normalize();
                    dir *= xOldPickingDist;

                    newPivotB = rayFrom + dir;
                }
                pickCon->setPivotB(newPivotB);
            }
        }
    }

    float dx, dy;
    dx = btScalar(x) - m_mouseOldX;
    dy = btScalar(y) - m_mouseOldY;


    ///only if ALT key is pressed (Maya style)
    /**if (m_modifierKeys & BT_ACTIVE_ALT)*/
    {
//        if (m_mouseButtons & 2) {
//            btVector3 hor = getRayTo(0, 0) - getRayTo(1, 0);
//            btVector3 vert = getRayTo(0, 0) - getRayTo(0, 1);
//            btScalar multiplierX = btScalar(0.001);
//            btScalar multiplierY = btScalar(0.001);
//            if (m_ortho) {
//                multiplierX = 1;
//                multiplierY = 1;
//            }
//
//
//            m_cameraTargetPosition += hor * dx * multiplierX;
//            m_cameraTargetPosition += vert * dy * multiplierY;
//        }

        if (m_mouseButtons & (2 << 2) && m_mouseButtons & 1) {
        } else if (m_mouseButtons & 2) {
            nextAzi += dx * btScalar(0.2);
            nextAzi = fmodf(nextAzi, btScalar(360.f));
            nextEle += dy * btScalar(0.2);
            nextEle = fmodf(nextEle, btScalar(180.f));
        } else if (m_mouseButtons & 4) {
            nextDist -= dy * btScalar(0.01f);
            if (nextDist < minDist)
                nextDist = minDist;

        }
    }


    m_mouseOldX = x;
    m_mouseOldY = y;
    updateCamera();


}

void Spacegraph::exitPhysics() {

    int i;

    for (i = 0; i < cells.size(); i++) {
        Cell* doll = cells[i];
        delete doll;
    }

    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them

    for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++) {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }

    //delete dynamics world
    delete m_dynamicsWorld;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;


}

void Spacegraph::renderscene(int pass) {
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();
    const int numObjects = m_dynamicsWorld->getNumCollisionObjects();

    for (int i = 0; i < numObjects; i++) {
        btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(colObj);

        if (body && body->getMotionState()) {
            btDefaultMotionState* myMotionState = (btDefaultMotionState*) body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
            rot = myMotionState->m_graphicsWorldTrans.getBasis();
        } else {
            colObj->getWorldTransform().getOpenGLMatrix(m);
            rot = colObj->getWorldTransform().getBasis();
        }

        //		btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
        //		if(i&1) wireColor=btVector3(0.f,0.0f,1.f);
        //		///color differently for active, sleeping, wantsdeactivation states
        //		if (colObj->getActivationState() == 1) //active
        //		{
        //			if (i & 1)
        //			{
        //				wireColor += btVector3 (1.f,0.f,0.f);
        //			}
        //			else
        //			{
        //				wireColor += btVector3 (.5f,0.f,0.f);
        //			}
        //		}
        //		if(colObj->getActivationState()==2) //ISLAND_SLEEPING
        //		{
        //			if(i&1)
        //			{
        //				wireColor += btVector3 (0.f,1.f, 0.f);
        //			}
        //			else
        //			{
        //				wireColor += btVector3 (0.f,0.5f,0.f);
        //			}
        //		}

        btVector3 aabbMin, aabbMax;
        m_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin, aabbMax);

        aabbMin -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        aabbMax += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        //		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
        //		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
        //		m_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));


        const btVector3 defaultColor(0.5, 0.5, 0.5);

        BodyProcess *process = NULL;
        if (body!=NULL) {
            process = (*bodyProcess)[body];
        }
        
        if (!(getDebugMode() & btIDebugDraw::DBG_DrawWireframe)) {
            switch (pass) {
                case 0: renderer->drawOpenGLx(m, colObj->getCollisionShape(), defaultColor, getDebugMode(), aabbMin, aabbMax, process);
                    break;
                case 1: renderer->drawShadow(m, m_sundirection*rot, colObj->getCollisionShape(), aabbMin, aabbMax);
                    break;
                case 2: renderer->drawOpenGLx(m, colObj->getCollisionShape(), defaultColor * btScalar(0.3), 0, aabbMin, aabbMax, process);
                    break;
            }
        }
    }

}

void Spacegraph::renderme()
{
	myinit();

	updateCamera();

	if (m_dynamicsWorld)
	{
		if(m_enableshadows)
		{
			glClear(GL_STENCIL_BUFFER_BIT);
			glEnable(GL_CULL_FACE);
			renderscene(0);

			glDisable(GL_LIGHTING);
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_STENCIL_TEST);
			glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
			glStencilFunc(GL_ALWAYS,1,0xFFFFFFFFL);
			glFrontFace(GL_CCW);
			glStencilOp(GL_KEEP,GL_KEEP,GL_INCR);
			renderscene(1);
			glFrontFace(GL_CW);
			glStencilOp(GL_KEEP,GL_KEEP,GL_DECR);
			renderscene(1);
			glFrontFace(GL_CCW);

			glPolygonMode(GL_FRONT,GL_FILL);
			glPolygonMode(GL_BACK,GL_FILL);
			glShadeModel(GL_SMOOTH);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
			glEnable(GL_LIGHTING);
			glDepthMask(GL_TRUE);
			glCullFace(GL_BACK);
			glFrontFace(GL_CCW);
			glEnable(GL_CULL_FACE);
			glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

			glDepthFunc(GL_LEQUAL);
			glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
			glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
			glDisable(GL_LIGHTING);
			renderscene(2);
			glEnable(GL_LIGHTING);
			glDepthFunc(GL_LESS);
			glDisable(GL_STENCIL_TEST);
			glDisable(GL_CULL_FACE);
		}
		else
		{
			glDisable(GL_CULL_FACE);
			renderscene(0);
		}

//		int	xOffset = 10;
//		int yStart = 20;
//		int yIncr = 20;
//
//
//		glDisable(GL_LIGHTING);
//		glColor3f(0, 0, 0);

//		if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
//		{
//			setOrthographicProjection();
//
//			showProfileInfo(xOffset,yStart,yIncr);
//
//#ifdef USE_QUICKPROF
//
//
//			if ( getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
//			{
//				static int counter = 0;
//				counter++;
//				std::map<std::string, hidden::ProfileBlock*>::iterator iter;
//				for (iter = btProfiler::mProfileBlocks.begin(); iter != btProfiler::mProfileBlocks.end(); ++iter)
//				{
//					char blockTime[128];
//					sprintf(blockTime, "%s: %lf",&((*iter).first[0]),btProfiler::getBlockTime((*iter).first, btProfiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
//					glRasterPos3f(xOffset,yStart,0);
//					GLDebugDrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
//					yStart += yIncr;
//
//				}
//
//			}
//#endif //USE_QUICKPROF
//
//
//
//
//			resetPerspectiveProjection();
//		}

		glEnable(GL_LIGHTING);


	}

    for (unsigned i = 0; i < cells.size(); i++) {
        cells[i]->draw();
    }
	//updateCamera();

}


