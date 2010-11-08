/* 
 * File:   Retina.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:21 PM
 */

#ifndef _RETINA_H
#define	_RETINA_H

#include <math.h>

#include "Cell.h"
#include "BodyProcess.h"
#include "CellProcess.h"

#include <NInput.h>
#include <NOutput.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h>
#include <BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h>

#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btMultiSphereShape.h>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <LinearMath/btAabbUtil2.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>


//#include <BulletCollision/CollisionShapes/btTetrahedronShape.h"
//#include <BulletCollision/CollisionShapes/btConeShape.h"
//#include <BulletCollision/CollisionShapes/btCylinderShape.h"
//#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
//
#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>


#include <iostream>
using namespace std;

struct CastResult {
    bool hit;
    btCollisionObject* hitBody;
    btVector3 hitPosition;
};

class Raycast {
public:
    Raycast(btDynamicsWorld* btWorld);
    ~Raycast();

    CastResult cast(const btVector3& rayFrom, const btVector3& rayTo);
private:
    btDynamicsWorld* btDynWorld;
    CastResult result;
};

class Retina : public NInput, public NOutput, public CellProcess {
    btCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btAxisSweep3* m_overlappingPairCache;
    btCollisionWorld* m_collisionWorld;

    btVoronoiSimplexSolver simplexSolver;


    bool textureCreated;
    GLuint textureID;

    btRigidBody* eyePart;

    double retinaWidth, retinaHeight;
    double focusAngle;

    btDynamicsWorld *world;

    GLubyte *texture;

    Raycast* rayCast;

    double visionDistance;
    float proportionFired;

public:
    unsigned basisForward, basisUp;
    float forwardSign;

    btVector3 originOffset;

    unsigned width, height;
    btVector4** pixel; //color array

    //global "to" pyramid
    btVector3 from; //base
    btVector3 toUL, toBR, toUR, toBL; 
    
    unsigned timeScale;

    unsigned p; //vision frame count
    bool antialias;
    Spacegraph* space;

    Retina(Brain* b, Spacegraph* s, btRigidBody* _eyePart, unsigned pixelWidth, unsigned pixelHeight, float _proportionFired, float _visionDistance, float _focusAngle)
    : NInput(b, pixelWidth*pixelHeight * 4), NOutput(b, 0, 0.5, 0.99),
    width(pixelWidth), height(pixelHeight), eyePart(_eyePart), proportionFired(_proportionFired), visionDistance(_visionDistance) {

        space = s;
        world = s->getSpace();

        basisForward = 1;
        basisUp = 2;
        forwardSign = 1;

        //texture = [ImageHeight][ImageWidth][3];
        textureCreated = false;
        texture = new GLubyte[pixelHeight * pixelWidth * 3];

        antialias = false;
        p = 0;
        timeScale = 1;

        focusAngle = _focusAngle;
        
        rayCast = new Raycast(world);

        pixel = new btVector4*[pixelWidth];
        for (unsigned j = 0; j < pixelWidth; j++) {
            pixel[j] = new btVector4[pixelHeight];
        }
        //        dist = new btScalar*[pixelWidth];
        //        for (unsigned j = 0; j < pixelWidth; j++) {
        //            dist[j] = new btScalar[pixelHeight];
        //        }
    }
    virtual void draw();
    virtual void update(double dt);


    virtual btVector4 getColor(CastResult res, btScalar d, float vDistance);


    virtual ~Retina() {

        for (unsigned j = 0; j < width; j++) {
            delete pixel[j];
            //delete dist[j];
        }
        delete pixel;
        //delete dist;
        delete rayCast;

    }


    //    ///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    //    bool worldRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);
    //
    //    ///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    //    bool singleObjectRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);
    //
    //    ///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
    //    bool lowlevelRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);
    //
    //    btVector4 getPixel(int x, int y) {
    //        return pixel[x][y];
    //    }
    //
    //    void setPixel(int x, int y, btVector4 c) {
    //        pixel[x][y] = c;
    //    }
    //
    //    virtual void process(double dt) {
    //        int mode = 0;
    //
    //        btVector3 rayFrom = eyePart->getCenterOfMassPosition();
    //        btVector3 rayForward = eyePart->getWorldTransform().getRotation().getAxis();
    //        rayForward.normalize();
    //        float farPlane = 600.f;
    //        rayForward *= farPlane;
    //
    //        btVector3 vertical(0.f, 1.f, 0.f);
    //        btVector3 hor;
    //        hor = rayForward.cross(vertical);
    //        hor.normalize();
    //        vertical = hor.cross(rayForward);
    //        vertical.normalize();
    //
    //        float tanfov = tan(0.5f * fov);
    //
    //        hor *= 2.f * farPlane * tanfov;
    //        vertical *= 2.f * farPlane * tanfov;
    //
    //        btVector3 rayToCenter = rayFrom + rayForward;
    //
    //        btVector3 dHor = hor * 1.f / float(screenWidth);
    //        btVector3 dVert = vertical * 1.f / float(screenHeight);
    //
    //        btTransform rayFromTrans;
    //        rayFromTrans.setIdentity();
    //        rayFromTrans.setOrigin(rayFrom);
    //
    //        btTransform rayFromLocal;
    //        btTransform rayToLocal;
    //
    //        btVector3 rayTo;
    //
    //        btVector4 rgba;
    //
    //        unsigned x, y;
    //        for (x = 0; x < width; x++) {
    //
    //            for (y = 0; y < height; y++) {
    //
    //                rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
    //                rayTo += x * dHor;
    //                rayTo -= y * dVert;
    //                btVector3 worldNormal(0, 0, 0);
    //                btVector3 worldPoint(0, 0, 0);
    //
    //                bool hasHit = false;
    //                int mode = 0;
    //                switch (mode) {
    //                    case 0:
    //                        hasHit = lowlevelRaytest(rayFrom, rayTo, worldNormal, worldPoint);
    //                        break;
    //                    case 1:
    //                        hasHit = singleObjectRaytest(rayFrom, rayTo, worldNormal, worldPoint);
    //                        break;
    //                    case 2:
    //                        hasHit = worldRaytest(rayFrom, rayTo, worldNormal, worldPoint);
    //                        break;
    //                    default:
    //                    {
    //                    }
    //                }
    //
    //                if (hasHit) {
    //                    float lightVec0 = worldNormal.dot(btVector3(0, -1, -1)); //0.4f,-1.f,-0.4f));
    //                    float lightVec1 = worldNormal.dot(btVector3(-1, 0, -1)); //-0.4f,-1.f,-0.4f));
    //
    //
    //                    rgba = btVector4(lightVec0, lightVec1, 0, 1.f);
    //                    rgba.setMin(btVector3(1, 1, 1));
    //                    rgba.setMax(btVector3(0.2, 0.2, 0.2));
    //                    rgba[3] = 1.f;
    //                    setPixel(x, y, rgba);
    //                } else {
    //                    //rgba = getPixel(x, y);
    //                    rgba = btVector4(0, 0, 0, 1);
    //                }
    //                if (!rgba.length2()) {
    //                    rgba = btVector4(1, 1, 1, 1);
    //                }
    //
    //                setPixel(x, y, rgba);
    //            }
    //        }
    //
    //    }


private:

};


#endif	/* _RETINA_H */

