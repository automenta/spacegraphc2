/* 
 * File:   Retina.cpp
 * Author: seh
 * 
 * Created on February 15, 2010, 9:21 PM
 */

#include "Retina.h"

void Retina::draw() {
    if (!textureCreated) {
        glGenTextures(1, &textureID);

        glBindTexture(GL_TEXTURE_2D, textureID);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        textureCreated = true;
    }

    glPushMatrix();

    glColor3f(1, 1, 1);

    {
        glBegin(GL_LINES);
        //pyramid base
        glVertex3f(from.x(), from.y(), from.z());
        glVertex3f(toUL.x(), toUL.y(), toUL.z());

        glVertex3f(from.x(), from.y(), from.z());
        glVertex3f(toUR.x(), toUR.y(), toUR.z());

        glVertex3f(from.x(), from.y(), from.z());
        glVertex3f(toBR.x(), toBR.y(), toBR.z());

        glVertex3f(from.x(), from.y(), from.z());
        glVertex3f(toBL.x(), toBL.y(), toBL.z());


        //diagonals
        glVertex3f(toUL.x(), toUL.y(), toUL.z());
        glVertex3f(toUR.x(), toUR.y(), toUR.z());

        glVertex3f(toUR.x(), toUR.y(), toUR.z());
        glVertex3f(toBR.x(), toBR.y(), toBR.z());

        glVertex3f(toBR.x(), toBR.y(), toBR.z());
        glVertex3f(toBL.x(), toBL.y(), toBL.z());

        glVertex3f(toBL.x(), toBL.y(), toBL.z());
        glVertex3f(toUL.x(), toUL.y(), toUL.z());

        glEnd();

    }


    //texture pyramid base
    {

        glEnable(GL_TEXTURE_2D);

        glBindTexture(GL_TEXTURE_2D, textureID);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture);

        glBegin(GL_POLYGON);

        glTexCoord2f(1.0, 1.0);
        glVertex3f(toUR.x(), toUR.y(), toUR.z());

        glTexCoord2f(0.0, 1.0);
        glVertex3f(toUL.x(), toUL.y(), toUL.z());

        glTexCoord2f(0.0, 0.0);
        glVertex3f(toBL.x(), toBL.y(), toBL.z());

        glTexCoord2f(1.0, 0.0);
        glVertex3f(toBR.x(), toBR.y(), toBR.z());

        glEnd();
        glDisable(GL_TEXTURE_2D);

    }


    glPopMatrix();
}

btVector4 Retina::getColor(CastResult res, btScalar d, float vDistance) {
    double pd = (1.0 - (d / vDistance));
    btCollisionObject* c = res.hitBody;
    if (c!=NULL) {
        btRigidBody* rb = (btRigidBody*) res.hitBody;
        if (rb!=NULL) {
            BodyProcess* bp = space->getProcess(rb);
            if (bp != NULL) {
                return btVector4(bp->color.x(), bp->color.y(), bp->color.z(), pd);
            }
        }
    }
    
    return btVector4(0.5, 0.5, 0.5, pd);

}

void Retina::update(double dt) {

    btTransform tr = eyePart->getWorldTransform();

    unsigned input = 0, x;


//    float focus = outs[0]->getOutput();
//    float focusScale = 0.01;
//    float vDistance = (visionDistance / 2.0) * (1.0 + focus * focusScale);
    float vDistance = visionDistance;

    float retinaDimension = vDistance * sin(focusAngle);
    if (width > height) {
        retinaWidth = retinaDimension;
        retinaHeight = retinaDimension * (float(height) / float(width));
    } else {
        retinaHeight = retinaDimension;
        retinaWidth = retinaDimension * (float(width) / float(height));
    }

    unsigned textureIndex = 0;

    //#pragma omp parallel for
    for (x = 0; x < width; x++) {
        unsigned ip = (p++) % (timeScale);
        for (unsigned y = 0; y < height; y++) {
            double dist;

            if (!(frand(0, 1.0) <= proportionFired)) {
                if (antialias) {
                    if (x > 0)
                        if (y > 0)
                            if (x < width - 1)
                                if (y < height - 1) {
                                    //anti-alias
                                    float r = pixel[x - 1][y].getX() + pixel[x + 1][y].getX() + pixel[x][y - 1].getX() + pixel[x][y + 1].getX() + pixel[x - 1][y - 1].getX() + pixel[x + 1][y + 1].getX() + pixel[x - 1][y - 1].getX() + pixel[x + 1][y + 1].getX();
                                    float g = pixel[x - 1][y].getY() + pixel[x + 1][y].getY() + pixel[x][y - 1].getY() + pixel[x][y + 1].getY() + pixel[x - 1][y - 1].getY() + pixel[x + 1][y + 1].getY() + pixel[x - 1][y - 1].getY() + pixel[x + 1][y + 1].getY();
                                    float b = pixel[x - 1][y].getZ() + pixel[x + 1][y].getZ() + pixel[x][y - 1].getZ() + pixel[x][y + 1].getZ() + pixel[x - 1][y - 1].getZ() + pixel[x + 1][y + 1].getZ() + pixel[x - 1][y - 1].getZ() + pixel[x + 1][y + 1].getZ();
                                    r /= 8.0;
                                    g /= 8.0;
                                    b /= 8.0;
                                    pixel[x][y] = btVector4(r, g, b, 1.0);
                                }
                }
            } else {

                from = eyePart->getWorldTransform().getOrigin() + originOffset;

                btVector3 forwardRay(
                        tr.getBasis()[0][basisForward],
                        tr.getBasis()[1][basisForward],
                        tr.getBasis()[2][basisForward]);
                forwardRay *= forwardSign;

                btVector3 upRay(
                        tr.getBasis()[0][basisUp],
                        tr.getBasis()[1][basisUp],
                        tr.getBasis()[2][basisUp]);

                forwardRay.normalize();
                upRay.normalize();

                //                    cout << forwardRay.x() << "," << forwardRay.y() << "," << forwardRay.z() << "  ";
                //                    cout << upRay.x() << "," << upRay.y() << "," << upRay.z() << "\n";

                btVector3 hor = forwardRay.cross(upRay);
                hor.normalize();

                upRay = hor.cross(forwardRay);
                upRay.normalize();

                btVector3 to = (tr.getOrigin() + forwardRay * visionDistance) - (0.5f * hor * retinaWidth) + (0.5f * upRay * retinaHeight);
                btVector3 toOffset = retinaWidth * (((float) x) / ((float) width)) * hor;
                toOffset -= retinaHeight * (((float) y) / ((float) height)) * upRay;
                to += toOffset;

                //display the texture at half the visionDistance
                btVector3 toVisible = (tr.getOrigin() + forwardRay * visionDistance/2.0) - (0.5f * hor * retinaWidth) + (0.5f * upRay * retinaHeight);
                toVisible += toOffset;

                if ((x == 0) && (y == 0)) toBL = toVisible;
                if ((x == width - 1) && (y == 0)) toBR = toVisible;
                if ((y == height - 1) && (x == 0)) toUL = toVisible;
                if ((y == height - 1) && (x == width - 1)) toUR = toVisible;

                CastResult r = rayCast->cast(from, to);
                if (r.hit) {
                    btScalar d = r.hitPosition.distance(from);
                    dist = d;
                    pixel[x][y] = getColor(r, d, vDistance);
                } else {
                    dist = vDistance;
                    pixel[x][y] = btVector4(0, 0, 0, 0);
                }

            }

            btVector4* cp = &(pixel[x][y]);

            double zd = cp->w()*0.1 + 0.9;

            texture[textureIndex++] = (GLubyte) (255.0 * ((float)cp->x()) * zd );
            texture[textureIndex++] = (GLubyte) (255.0 * ((float)cp->y()) * zd );
            texture[textureIndex++] = (GLubyte) (255.0 * ((float)cp->z()) * zd );

            ins[input++]->setInput(cp->x());
            ins[input++]->setInput(cp->y());
            ins[input++]->setInput(cp->z());
            ins[input++]->setInput(cp->w());

        }
    }
    //printf("%d %d: %d\n", (int)width, (int)height, (int)textureIndex);
}

Raycast::Raycast(btDynamicsWorld* btWorld) {
    btDynWorld = btWorld;
}

CastResult Raycast::cast(const btVector3& rayFrom, const btVector3& rayTo) {
    result.hit = false;

    btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom, rayTo);
    btDynWorld->rayTest(rayFrom, rayTo, resultCallback);

    if (resultCallback.hasHit()) {
        // 		cerr << "1 true" << endl;
        result.hitBody = resultCallback.m_collisionObject;
        result.hit = true;
        result.hitPosition = resultCallback.m_hitPointWorld;

        /*		if ( result.hitBody )
                        {
                                cerr << "2 true" << endl;
                                result.hit = true;
                                result.hitPosition = resultCallback.m_hitPointWorld;
                        }*/
    }

    return result;
}

Raycast::~Raycast() {
}


//bool Retina::worldRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
//{
//
//	struct	AllRayResultCallback : public btCollisionWorld::RayResultCallback	{
//		AllRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld)
//		:m_rayFromWorld(rayFromWorld),
//		m_rayToWorld(rayToWorld)
//		{
//		}
//
//		btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
//		btVector3	m_rayToWorld;
//
//		btVector3	m_hitNormalWorld;
//		btVector3	m_hitPointWorld;
//
//		virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
//		{
//
////caller already does the filter on the m_closestHitFraction
//			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
//
//			m_closestHitFraction = rayResult.m_hitFraction;
//
//			m_collisionObject = rayResult.m_collisionObject;
//			if (normalInWorldSpace)
//			{
//				m_hitNormalWorld = rayResult.m_hitNormalLocal;
//			} else
//			{
//				///need to transform normal into worldspace
//				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
//			}
//			m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
//			return 1.f;
//		}
//	};
//
//
//	AllRayResultCallback	resultCallback(rayFrom,rayTo);
////	btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
//	m_collisionWorld->rayTest(rayFrom,rayTo,resultCallback);
//	if (resultCallback.hasHit())
//	{
//		worldNormal = resultCallback.m_hitNormalWorld;
//		return true;
//	}
//	return false;
//}
//
//
//bool Retina::singleObjectRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
//{
//
//	btScalar closestHitResults = 1.f;
//
//	btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
//
//	bool hasHit = false;
//	btConvexCast::CastResult rayResult;
//	btSphereShape pointShape(0.0f);
//	btTransform rayFromTrans;
//	btTransform rayToTrans;
//
//	rayFromTrans.setIdentity();
//	rayFromTrans.setOrigin(rayFrom);
//	rayToTrans.setIdentity();
//	rayToTrans.setOrigin(rayTo);
//
//	for (int s=0;s<numObjects;s++)
//	{
//		//comment-out next line to get all hits, instead of just the closest hit
//		//resultCallback.m_closestHitFraction = 1.f;
//
//		//do some culling, ray versus aabb
//		btVector3 aabbMin,aabbMax;
//		shapePtr[s]->getAabb(transforms[s],aabbMin,aabbMax);
//		btScalar hitLambda = 1.f;
//		btVector3 hitNormal;
//		btCollisionObject	tmpObj;
//		tmpObj.setWorldTransform(transforms[s]);
//
//
//		if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal))
//		{
//			//reset previous result
//
//			btCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans, &tmpObj, shapePtr[s], transforms[s], resultCallback);
//			if (resultCallback.hasHit())
//			{
//				//float fog = 1.f - 0.1f * rayResult.m_fraction;
//				resultCallback.m_hitNormalWorld.normalize();//.m_normal.normalize();
//				worldNormal = resultCallback.m_hitNormalWorld;
//				//worldNormal = transforms[s].getBasis() *rayResult.m_normal;
//				worldNormal.normalize();
//				hasHit = true;
//			}
//		}
//	}
//
//	return hasHit;
//}
//
//
//bool Retina::lowlevelRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
//{
//
//	btScalar closestHitResults = 1.f;
//
//	bool hasHit = false;
//	btConvexCast::CastResult rayResult;
//	btSphereShape pointShape(0.0f);
//	btTransform rayFromTrans;
//	btTransform rayToTrans;
//
//	rayFromTrans.setIdentity();
//	rayFromTrans.setOrigin(rayFrom);
//	rayToTrans.setIdentity();
//	rayToTrans.setOrigin(rayTo);
//
//        btConvexShape*	shapePtr = m_collisionWorld->getCollisionObjectArray()[0];
//        btTransform* transforms;
//
//	for (int s=0;s<numObjects;s++)
//	{
//
//		//do some culling, ray versus aabb
//		btVector3 aabbMin,aabbMax;
//		shapePtr[s]->getAabb(transforms[s],aabbMin,aabbMax);
//		btScalar hitLambda = 1.f;
//		btVector3 hitNormal;
//		btCollisionObject	tmpObj;
//		tmpObj.setWorldTransform(transforms[s]);
//
//
//		if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal))
//		{
//			//reset previous result
//
//			//choose the continuous collision detection method
//			btSubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
//			//btGjkConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
//			//btContinuousConvexCollision convexCaster(&pointShape,shapePtr[s],&simplexSolver,0);
//
//			if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
//			{
//				if (rayResult.m_fraction < closestHitResults)
//				{
//					closestHitResults = rayResult.m_fraction;
//
//					worldNormal = transforms[s].getBasis() *rayResult.m_normal;
//					worldNormal.normalize();
//					hasHit = true;
//				}
//			}
//		}
//	}
//
//	return hasHit;
//
//}
