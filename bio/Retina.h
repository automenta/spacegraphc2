/* 
 * File:   Retina.h
 * Author: seh
 *
 * Created on February 15, 2010, 9:21 PM
 */

#ifndef _RETINA_H
#define	_RETINA_H

#include <math.h>

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

class Retina : public NInput, public NOutput {
    btCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btAxisSweep3* m_overlappingPairCache;
    btCollisionWorld* m_collisionWorld;

    btVoronoiSimplexSolver simplexSolver;

    btRigidBody* eyePart;

    double retinaWidth, retinaHeight;

    btDynamicsWorld *world;


    Raycast* rayCast;

    double visionDistance;
    float proportionFired;

public:
    btVector3 originOffset;

    unsigned width, height;
    btVector4** pixel; //color array
    unsigned timeScale;

    unsigned p; //vision frame count

    Retina(Brain* b, btDynamicsWorld *_world, btRigidBody* _eyePart, unsigned pixelWidth, unsigned pixelHeight, float _proportionFired, float _visionDistance /* double minframerate */)
    : NInput(b, pixelWidth*pixelHeight * 3),
    NOutput(b, 1, 0.5, 0.99),
    world(_world), width(pixelWidth), height(pixelHeight), eyePart(_eyePart), proportionFired(_proportionFired), visionDistance(_visionDistance) {

        p = 0;
        timeScale = 1;

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

    virtual void process(double dt) {
        btTransform tr = eyePart->getWorldTransform();

        unsigned input = 0, x;


        float focus = outs[0]->getOutput();
        float focusScale = 0.01;
        float vDistance = (visionDistance / 2.0) * (1.0 + focus * focusScale);

        if (width > height) {
            retinaWidth = vDistance;
            retinaHeight = vDistance * (float(height) / float(width));
        } else {
            retinaHeight = vDistance;
            retinaWidth = vDistance * (float(width) / float(height));
        }

        #pragma omp parallel for
        for (x = 0; x < width; x++) {
            unsigned ip = (p++) % (timeScale);
            for (unsigned y = 0; y < height; y++) {
                double dist;

                if (!(frand(0, 1.0) <= proportionFired)) {
                    if (x > 0)
                        if ( y > 0)
                            if (x < width-1)
                                if (y < height-1) {
                                    //anti-alias
                                    float r = pixel[x-1][y].getX() + pixel[x+1][y].getX() + pixel[x][y-1].getX() + pixel[x][y+1].getX() + pixel[x-1][y-1].getX() + pixel[x+1][y+1].getX() + pixel[x-1][y-1].getX() + pixel[x+1][y+1].getX();
                                    float g = pixel[x-1][y].getY() + pixel[x+1][y].getY() + pixel[x][y-1].getY() + pixel[x][y+1].getY() + pixel[x-1][y-1].getY() + pixel[x+1][y+1].getY() + pixel[x-1][y-1].getY() + pixel[x+1][y+1].getY();
                                    float b = pixel[x-1][y].getZ() + pixel[x+1][y].getZ() + pixel[x][y-1].getZ() + pixel[x][y+1].getZ() + pixel[x-1][y-1].getZ() + pixel[x+1][y+1].getZ() + pixel[x-1][y-1].getZ() + pixel[x+1][y+1].getZ();
                                    r/=8.0;
                                    g/=8.0;
                                    b/=8.0;
                                    pixel[x][y] = btVector4(r, g, b, 1.0);
                                }
                } else {
                    btVector3 from = eyePart->getWorldTransform().getOrigin() + originOffset;


                    btVector3 forwardRay(
                            tr.getBasis()[0][1],
                            tr.getBasis()[1][1],
                            tr.getBasis()[2][1]);

                    btVector3 upRay(
                            tr.getBasis()[0][2],
                            tr.getBasis()[1][2],
                            tr.getBasis()[2][2]);

                    forwardRay.normalize();
                    upRay.normalize();
                    
//                    cout << forwardRay.x() << "," << forwardRay.y() << "," << forwardRay.z() << "  ";
//                    cout << upRay.x() << "," << upRay.y() << "," << upRay.z() << "\n";

                    btVector3 hor = forwardRay.cross(upRay);
                    hor.normalize();
                    hor *= vDistance;

                    upRay = hor.cross(forwardRay);
                    upRay.normalize();
                    upRay *= vDistance;

                    btVector3 to = (tr.getOrigin() + forwardRay * visionDistance) - (0.5f * hor) + (0.5f * upRay);
                    to += x * (hor * (1.0f / (retinaWidth)));
                    to -= y * (upRay * (1.0f / (retinaHeight)));

                    CastResult r = rayCast->cast(from, to);
                    if (r.hit) {
                        btScalar d = r.hitPosition.distance(from);
                        //dist[x][y] = d;
                        dist = d;
                        pixel[x][y] = getColor(r, d, vDistance);
                    } else {
                        //dist[x][y] = vDistance;
                        dist = vDistance;
                        pixel[x][y] = btVector4(-0.5, -0.5, -0.5, 0);
                    }

                }



                double vd = dist / vDistance;

                btVector3* cp = &(pixel[x][y]);

                ins[input++]->setInput(-0.5 + 2.0 * cp->getX() * vd);
                ins[input++]->setInput(-0.5 + 2.0 * cp->getY() * vd);
                ins[input++]->setInput(-0.5 + 2.0 * cp->getZ() * vd);

            }
        }


    }

    virtual btVector4 getColor(CastResult res, btScalar d, float vDistance) {
        double pd = (1.0 - (d / vDistance));
        btCollisionObject* c = res.hitBody;
        btRigidBody* rb = (btRigidBody*)res.hitBody;

        //return btVector4(rb->color.getX(), rb->color.getY(), rb->color.getZ(), 1);
        return btVector4(1, 0, 0, 1);

//
//        float r, g, b;
//
//        long C = (long)c;
//        int hash = (C & 0x00000011) + ((C & 0x00001100) >> 2) + ((C & 0x00110000) >> 4) + ((C & 0x11000000) >> 6);
//
//        switch (hash % 9) {
//            case 0:
//                r = 1.0;  g = 0.5;   b = 0.25;
//                break;
//            case 1:
//                r = 0.25; g = 0.5;   b = 1.0;
//                break;
//            case 2:
//                r = 0.5; g = 1.0;   b = 0.25;
//                break;
//            case 3:
//                r = 0.25; g = 0.5;   b = 1.0;
//                break;
//            case 4:
//                r = 0.25;  g = 1.0;  b = 0.5;
//                break;
//            case 5:
//                r = 0.25; g = 1.0;   b = 0.5;
//                break;
//            case 6:
//                r = 1.0; g = 0;   b = 0;
//                break;
//            case 7:
//                r = 0.0; g = 1.0;   b = 0;
//                break;
//            case 8:
//                r = 0.0; g = 0;   b = 1.0;
//                break;
//        }
//
//        btVector4 v(r*pd, g*pd, b*pd, 1);
//        return v;
    }

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

