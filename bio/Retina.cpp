/* 
 * File:   Retina.cpp
 * Author: seh
 * 
 * Created on February 15, 2010, 9:21 PM
 */

#include "Retina.h"

Raycast::Raycast(btDynamicsWorld* btWorld)
{
	btDynWorld = btWorld;
}

CastResult Raycast::cast(const btVector3& rayFrom, const btVector3& rayTo)
{
	result.hit = false;

	btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
	btDynWorld->rayTest(rayFrom,rayTo,resultCallback);

	if (resultCallback.hasHit())
	{
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

Raycast::~Raycast()
{
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
