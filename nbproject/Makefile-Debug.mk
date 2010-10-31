#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/neural/Neuron.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btUnionFind.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHinge2Constraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btEmptyShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btManifoldResult.o \
	${OBJECTDIR}/obj/Bench.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_tri_collision.o \
	${OBJECTDIR}/src/Cell.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btOptimizedBvh.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactShape.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidDynamicsWorld.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_memory.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionObject.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o \
	${OBJECTDIR}/src/Renderer.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBox2dShape.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SequentialThreadSupport.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btShapeHull.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTask/SpuSampleTask.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMeshShape.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactBvh.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionTaskProcess.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o \
	${OBJECTDIR}/bullet-gl/GL_Simplex1to4.o \
	${OBJECTDIR}/bullet-gl/RenderTexture.o \
	${OBJECTDIR}/bullet-gl/Win32DemoApplication.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionDispatcher.o \
	${OBJECTDIR}/neural/NOutput.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCapsuleShape.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o \
	${OBJECTDIR}/bullet-gl/GL_ShapeDrawer.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btContactProcessing.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStridingMeshInterface.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexHullShape.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btUniversalConstraint.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btContactConstraint.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btTypedConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o \
	${OBJECTDIR}/bullet-src/MiniCL/MiniCL.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexInternalShape.o \
	${OBJECTDIR}/bullet-src/MiniCL/MiniCLTaskScheduler.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSliderConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleBuffer.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStaticPlaneShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCollisionShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btSphereShape.o \
	${OBJECTDIR}/bullet-gl/GL_DialogDynamicsWorld.o \
	${OBJECTDIR}/bio/ServoHinge.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxDetector.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCompoundShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactQuantizedBvh.o \
	${OBJECTDIR}/src/CellProcess.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btAxisSweep3.o \
	${OBJECTDIR}/bullet-gl/GL_DialogWindow.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/LinearMath/btSerializer.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleCallback.o \
	${OBJECTDIR}/bullet-gl/GLDebugFont.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuLibspe2Support.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o \
	${OBJECTDIR}/bullet-src/LinearMath/btQuickprof.o \
	${OBJECTDIR}/bio/NPosition.o \
	${OBJECTDIR}/neural/BrainLink.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btRigidBody.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btRaycastVehicle.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/PosixThreadSupport.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvex2dShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/SphereTriangleDetector.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSimulationIslandManager.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o \
	${OBJECTDIR}/bullet-gl/DemoApplication.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btQuantizedBvh.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGenericPoolAllocator.o \
	${OBJECTDIR}/neural/Math.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btWheelInfo.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.o \
	${OBJECTDIR}/bio/BloodBrainInterface.o \
	${OBJECTDIR}/bio/NColor.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btTriangleShapeEx.o \
	${OBJECTDIR}/src/Spacegraph.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_contact.o \
	${OBJECTDIR}/bio/Retina.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btConvexCast.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/LinearMath/btAlignedAllocator.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexPointCloudShape.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/Win32ThreadSupport.o \
	${OBJECTDIR}/obj/BrainSpace.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuGatheringCollisionDispatcher.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultiSphereShape.o \
	${OBJECTDIR}/src/BodyProcess.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDispatcher.o \
	${OBJECTDIR}/neural/NInput.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBody.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/btThreadSupportInterface.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_box_set.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Character/btKinematicCharacterController.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBoxShape.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/btParallelConstraintSolver.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTaskProcess.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConcaveShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHingeConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCylinderShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvt.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMinkowskiSumShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o \
	${OBJECTDIR}/neural/Brain.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/btGpu3DGridBroadphase.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTetrahedronShape.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionWorld.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMesh.o \
	${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o \
	${OBJECTDIR}/bullet-src/MiniCL/MiniCLTask/MiniCLTask.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btGhostObject.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o \
	${OBJECTDIR}/bullet-src/LinearMath/btGeometryUtil.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o \
	${OBJECTDIR}/bio/SineSound.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyHelpers.o \
	${OBJECTDIR}/bullet-gl/Win32AppMain.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o \
	${OBJECTDIR}/bullet-gl/GLDebugDrawer.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btUniformScalingShape.o \
	${OBJECTDIR}/bullet-gl/GlutDemoApplication.o \
	${OBJECTDIR}/bullet-gl/GlutStuff.o \
	${OBJECTDIR}/obj/Spider.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o \
	${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o \
	${OBJECTDIR}/bullet-src/LinearMath/btConvexHull.o \
	${OBJECTDIR}/obj/Humanoid.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConeShape.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btDefaultSoftBodySolver.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o \
	${OBJECTDIR}/bio/SixDoFMotor.o \
	${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/Bullet-C-API.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionObjectWrapper.o \
	${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuFakeDma.o \
	${OBJECTDIR}/obj/Snake.o \
	${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftSoftCollisionAlgorithm.o \
	${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-fopenmp
CXXFLAGS=-fopenmp

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-Debug.mk dist/Debug/GNU-Linux-x86/spacegraphc

dist/Debug/GNU-Linux-x86/spacegraphc: ${OBJECTFILES}
	${MKDIR} -p dist/Debug/GNU-Linux-x86
	${LINK.cc} -lGL -lglut -lGLU -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/spacegraphc ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/neural/Neuron.o: neural/Neuron.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/Neuron.o neural/Neuron.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btUnionFind.o: bullet-src/BulletCollision/CollisionDispatch/btUnionFind.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btUnionFind.o bullet-src/BulletCollision/CollisionDispatch/btUnionFind.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o: bullet-src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o bullet-src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHinge2Constraint.o: bullet-src/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHinge2Constraint.o bullet-src/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btEmptyShape.o: bullet-src/BulletCollision/CollisionShapes/btEmptyShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btEmptyShape.o bullet-src/BulletCollision/CollisionShapes/btEmptyShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btManifoldResult.o: bullet-src/BulletCollision/CollisionDispatch/btManifoldResult.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btManifoldResult.o bullet-src/BulletCollision/CollisionDispatch/btManifoldResult.cpp

${OBJECTDIR}/obj/Bench.o: obj/Bench.cpp 
	${MKDIR} -p ${OBJECTDIR}/obj
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/obj/Bench.o obj/Bench.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o: bullet-src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o bullet-src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o: bullet-src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o bullet-src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_tri_collision.o: bullet-src/BulletCollision/Gimpact/gim_tri_collision.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_tri_collision.o bullet-src/BulletCollision/Gimpact/gim_tri_collision.cpp

${OBJECTDIR}/src/Cell.o: src/Cell.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/Cell.o src/Cell.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btOptimizedBvh.o: bullet-src/BulletCollision/CollisionShapes/btOptimizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btOptimizedBvh.o bullet-src/BulletCollision/CollisionShapes/btOptimizedBvh.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactShape.o: bullet-src/BulletCollision/Gimpact/btGImpactShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactShape.o bullet-src/BulletCollision/Gimpact/btGImpactShape.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidDynamicsWorld.o: bullet-src/BulletSoftBody/btSoftRigidDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidDynamicsWorld.o bullet-src/BulletSoftBody/btSoftRigidDynamicsWorld.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_memory.o: bullet-src/BulletCollision/Gimpact/gim_memory.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_memory.o bullet-src/BulletCollision/Gimpact/gim_memory.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionObject.o: bullet-src/BulletCollision/CollisionDispatch/btCollisionObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionObject.o bullet-src/BulletCollision/CollisionDispatch/btCollisionObject.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp

${OBJECTDIR}/src/Renderer.o: src/Renderer.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/Renderer.o src/Renderer.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o: bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o: bullet-src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o bullet-src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBox2dShape.o: bullet-src/BulletCollision/CollisionShapes/btBox2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBox2dShape.o bullet-src/BulletCollision/CollisionShapes/btBox2dShape.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SequentialThreadSupport.o: bullet-src/BulletMultiThreaded/SequentialThreadSupport.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SequentialThreadSupport.o bullet-src/BulletMultiThreaded/SequentialThreadSupport.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btShapeHull.o: bullet-src/BulletCollision/CollisionShapes/btShapeHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btShapeHull.o bullet-src/BulletCollision/CollisionShapes/btShapeHull.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTask/SpuSampleTask.o: bullet-src/BulletMultiThreaded/SpuSampleTask/SpuSampleTask.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTask/SpuSampleTask.o bullet-src/BulletMultiThreaded/SpuSampleTask/SpuSampleTask.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMeshShape.o: bullet-src/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMeshShape.o bullet-src/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactBvh.o: bullet-src/BulletCollision/Gimpact/btGImpactBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactBvh.o bullet-src/BulletCollision/Gimpact/btGImpactBvh.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionTaskProcess.o: bullet-src/BulletMultiThreaded/SpuCollisionTaskProcess.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionTaskProcess.o bullet-src/BulletMultiThreaded/SpuCollisionTaskProcess.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o: bullet-src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o bullet-src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp

${OBJECTDIR}/bullet-gl/GL_Simplex1to4.o: bullet-gl/GL_Simplex1to4.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GL_Simplex1to4.o bullet-gl/GL_Simplex1to4.cpp

${OBJECTDIR}/bullet-gl/RenderTexture.o: bullet-gl/RenderTexture.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/RenderTexture.o bullet-gl/RenderTexture.cpp

${OBJECTDIR}/bullet-gl/Win32DemoApplication.o: bullet-gl/Win32DemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/Win32DemoApplication.o bullet-gl/Win32DemoApplication.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionDispatcher.o: bullet-src/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionDispatcher.o bullet-src/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp

${OBJECTDIR}/neural/NOutput.o: neural/NOutput.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/NOutput.o neural/NOutput.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.o: bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.o bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCapsuleShape.o: bullet-src/BulletCollision/CollisionShapes/btCapsuleShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCapsuleShape.o bullet-src/BulletCollision/CollisionShapes/btCapsuleShape.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o: bullet-src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o bullet-src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp

${OBJECTDIR}/bullet-gl/GL_ShapeDrawer.o: bullet-gl/GL_ShapeDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GL_ShapeDrawer.o bullet-gl/GL_ShapeDrawer.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btContactProcessing.o: bullet-src/BulletCollision/Gimpact/btContactProcessing.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btContactProcessing.o bullet-src/BulletCollision/Gimpact/btContactProcessing.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o: bullet-src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o bullet-src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStridingMeshInterface.o: bullet-src/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStridingMeshInterface.o bullet-src/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexHullShape.o: bullet-src/BulletCollision/CollisionShapes/btConvexHullShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexHullShape.o bullet-src/BulletCollision/CollisionShapes/btConvexHullShape.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btUniversalConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btUniversalConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btContactConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btContactConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btTypedConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btTypedConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o: bullet-src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o bullet-src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o: bullet-src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o bullet-src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp

${OBJECTDIR}/bullet-src/MiniCL/MiniCL.o: bullet-src/MiniCL/MiniCL.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/MiniCL
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/MiniCL/MiniCL.o bullet-src/MiniCL/MiniCL.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexInternalShape.o: bullet-src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexInternalShape.o bullet-src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp

${OBJECTDIR}/bullet-src/MiniCL/MiniCLTaskScheduler.o: bullet-src/MiniCL/MiniCLTaskScheduler.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/MiniCL
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/MiniCL/MiniCLTaskScheduler.o bullet-src/MiniCL/MiniCLTaskScheduler.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSliderConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSliderConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleBuffer.o: bullet-src/BulletCollision/CollisionShapes/btTriangleBuffer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleBuffer.o bullet-src/BulletCollision/CollisionShapes/btTriangleBuffer.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStaticPlaneShape.o: bullet-src/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btStaticPlaneShape.o bullet-src/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCollisionShape.o: bullet-src/BulletCollision/CollisionShapes/btCollisionShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCollisionShape.o bullet-src/BulletCollision/CollisionShapes/btCollisionShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btSphereShape.o: bullet-src/BulletCollision/CollisionShapes/btSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btSphereShape.o bullet-src/BulletCollision/CollisionShapes/btSphereShape.cpp

${OBJECTDIR}/bullet-gl/GL_DialogDynamicsWorld.o: bullet-gl/GL_DialogDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GL_DialogDynamicsWorld.o bullet-gl/GL_DialogDynamicsWorld.cpp

${OBJECTDIR}/bio/ServoHinge.o: bio/ServoHinge.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/ServoHinge.o bio/ServoHinge.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o: bullet-src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o bullet-src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxDetector.o: bullet-src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxDetector.o bullet-src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCompoundShape.o: bullet-src/BulletCollision/CollisionShapes/btCompoundShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCompoundShape.o bullet-src/BulletCollision/CollisionShapes/btCompoundShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactQuantizedBvh.o: bullet-src/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactQuantizedBvh.o bullet-src/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp

${OBJECTDIR}/src/CellProcess.o: src/CellProcess.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/CellProcess.o src/CellProcess.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btAxisSweep3.o: bullet-src/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btAxisSweep3.o bullet-src/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp

${OBJECTDIR}/bullet-gl/GL_DialogWindow.o: bullet-gl/GL_DialogWindow.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GL_DialogWindow.o bullet-gl/GL_DialogWindow.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidCollisionAlgorithm.o: bullet-src/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftRigidCollisionAlgorithm.o bullet-src/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/LinearMath/btSerializer.o: bullet-src/LinearMath/btSerializer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/LinearMath/btSerializer.o bullet-src/LinearMath/btSerializer.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexShape.o: bullet-src/BulletCollision/CollisionShapes/btConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexShape.o bullet-src/BulletCollision/CollisionShapes/btConvexShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleCallback.o: bullet-src/BulletCollision/CollisionShapes/btTriangleCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleCallback.o bullet-src/BulletCollision/CollisionShapes/btTriangleCallback.cpp

${OBJECTDIR}/bullet-gl/GLDebugFont.o: bullet-gl/GLDebugFont.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GLDebugFont.o bullet-gl/GLDebugFont.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.o: bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.o bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuLibspe2Support.o: bullet-src/BulletMultiThreaded/SpuLibspe2Support.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuLibspe2Support.o bullet-src/BulletMultiThreaded/SpuLibspe2Support.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o: bullet-src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o bullet-src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o: bullet-src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o bullet-src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o: bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp

${OBJECTDIR}/bullet-src/LinearMath/btQuickprof.o: bullet-src/LinearMath/btQuickprof.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/LinearMath/btQuickprof.o bullet-src/LinearMath/btQuickprof.cpp

${OBJECTDIR}/bio/NPosition.o: bio/NPosition.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/NPosition.o bio/NPosition.cpp

${OBJECTDIR}/neural/BrainLink.o: neural/BrainLink.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/BrainLink.o neural/BrainLink.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o: bullet-src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o bullet-src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btRigidBody.o: bullet-src/BulletDynamics/Dynamics/btRigidBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btRigidBody.o bullet-src/BulletDynamics/Dynamics/btRigidBody.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.o: bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.o bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btRaycastVehicle.o: bullet-src/BulletDynamics/Vehicle/btRaycastVehicle.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btRaycastVehicle.o bullet-src/BulletDynamics/Vehicle/btRaycastVehicle.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/PosixThreadSupport.o: bullet-src/BulletMultiThreaded/PosixThreadSupport.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/PosixThreadSupport.o bullet-src/BulletMultiThreaded/PosixThreadSupport.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvex2dShape.o: bullet-src/BulletCollision/CollisionShapes/btConvex2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvex2dShape.o bullet-src/BulletCollision/CollisionShapes/btConvex2dShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/SphereTriangleDetector.o: bullet-src/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/SphereTriangleDetector.o bullet-src/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSimulationIslandManager.o: bullet-src/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSimulationIslandManager.o bullet-src/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o: bullet-src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o bullet-src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp

${OBJECTDIR}/bullet-gl/DemoApplication.o: bullet-gl/DemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/DemoApplication.o bullet-gl/DemoApplication.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btQuantizedBvh.o: bullet-src/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btQuantizedBvh.o bullet-src/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o: bullet-src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o bullet-src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGenericPoolAllocator.o: bullet-src/BulletCollision/Gimpact/btGenericPoolAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGenericPoolAllocator.o bullet-src/BulletCollision/Gimpact/btGenericPoolAllocator.cpp

${OBJECTDIR}/neural/Math.o: neural/Math.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/Math.o neural/Math.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o: bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o bullet-src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o: bullet-src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o bullet-src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btWheelInfo.o: bullet-src/BulletDynamics/Vehicle/btWheelInfo.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Vehicle/btWheelInfo.o bullet-src/BulletDynamics/Vehicle/btWheelInfo.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.o: bullet-src/BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.o bullet-src/BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.cpp

${OBJECTDIR}/bio/BloodBrainInterface.o: bio/BloodBrainInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/BloodBrainInterface.o bio/BloodBrainInterface.cpp

${OBJECTDIR}/bio/NColor.o: bio/NColor.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/NColor.o bio/NColor.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btTriangleShapeEx.o: bullet-src/BulletCollision/Gimpact/btTriangleShapeEx.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btTriangleShapeEx.o bullet-src/BulletCollision/Gimpact/btTriangleShapeEx.cpp

${OBJECTDIR}/src/Spacegraph.o: src/Spacegraph.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/Spacegraph.o src/Spacegraph.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_contact.o: bullet-src/BulletCollision/Gimpact/gim_contact.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_contact.o bullet-src/BulletCollision/Gimpact/gim_contact.cpp

${OBJECTDIR}/bio/Retina.o: bio/Retina.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/Retina.o bio/Retina.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btConvexCast.o: bullet-src/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btConvexCast.o bullet-src/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/LinearMath/btAlignedAllocator.o: bullet-src/LinearMath/btAlignedAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/LinearMath/btAlignedAllocator.o bullet-src/LinearMath/btAlignedAllocator.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexPointCloudShape.o: bullet-src/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexPointCloudShape.o bullet-src/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/Win32ThreadSupport.o: bullet-src/BulletMultiThreaded/Win32ThreadSupport.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/Win32ThreadSupport.o bullet-src/BulletMultiThreaded/Win32ThreadSupport.cpp

${OBJECTDIR}/obj/BrainSpace.o: obj/BrainSpace.cpp 
	${MKDIR} -p ${OBJECTDIR}/obj
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/obj/BrainSpace.o obj/BrainSpace.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuGatheringCollisionDispatcher.o: bullet-src/BulletMultiThreaded/SpuGatheringCollisionDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuGatheringCollisionDispatcher.o bullet-src/BulletMultiThreaded/SpuGatheringCollisionDispatcher.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultiSphereShape.o: bullet-src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMultiSphereShape.o bullet-src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp

${OBJECTDIR}/src/BodyProcess.o: src/BodyProcess.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/src/BodyProcess.o src/BodyProcess.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDispatcher.o: bullet-src/BulletCollision/BroadphaseCollision/btDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDispatcher.o bullet-src/BulletCollision/BroadphaseCollision/btDispatcher.cpp

${OBJECTDIR}/neural/NInput.o: neural/NInput.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/NInput.o neural/NInput.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBody.o: bullet-src/BulletSoftBody/btSoftBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBody.o bullet-src/BulletSoftBody/btSoftBody.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/btThreadSupportInterface.o: bullet-src/BulletMultiThreaded/btThreadSupportInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/btThreadSupportInterface.o bullet-src/BulletMultiThreaded/btThreadSupportInterface.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_box_set.o: bullet-src/BulletCollision/Gimpact/gim_box_set.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/gim_box_set.o bullet-src/BulletCollision/Gimpact/gim_box_set.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Character/btKinematicCharacterController.o: bullet-src/BulletDynamics/Character/btKinematicCharacterController.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Character
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Character/btKinematicCharacterController.o bullet-src/BulletDynamics/Character/btKinematicCharacterController.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBoxShape.o: bullet-src/BulletCollision/CollisionShapes/btBoxShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btBoxShape.o bullet-src/BulletCollision/CollisionShapes/btBoxShape.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/btParallelConstraintSolver.o: bullet-src/BulletMultiThreaded/btParallelConstraintSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/btParallelConstraintSolver.o bullet-src/BulletMultiThreaded/btParallelConstraintSolver.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTaskProcess.o: bullet-src/BulletMultiThreaded/SpuSampleTaskProcess.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuSampleTaskProcess.o bullet-src/BulletMultiThreaded/SpuSampleTaskProcess.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o: bullet-src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o bullet-src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConcaveShape.o: bullet-src/BulletCollision/CollisionShapes/btConcaveShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConcaveShape.o bullet-src/BulletCollision/CollisionShapes/btConcaveShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o: bullet-src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o bullet-src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o: bullet-src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o bullet-src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.o: bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.o bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.o: bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.o bullet-src/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/boxBoxDistance.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHingeConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btHingeConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCylinderShape.o: bullet-src/BulletCollision/CollisionShapes/btCylinderShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btCylinderShape.o bullet-src/BulletCollision/CollisionShapes/btCylinderShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvt.o: bullet-src/BulletCollision/BroadphaseCollision/btDbvt.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvt.o bullet-src/BulletCollision/BroadphaseCollision/btDbvt.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMinkowskiSumShape.o: bullet-src/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btMinkowskiSumShape.o bullet-src/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o: bullet-src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o bullet-src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp

${OBJECTDIR}/neural/Brain.o: neural/Brain.cpp 
	${MKDIR} -p ${OBJECTDIR}/neural
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/neural/Brain.o neural/Brain.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/btGpu3DGridBroadphase.o: bullet-src/BulletMultiThreaded/btGpu3DGridBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/btGpu3DGridBroadphase.o bullet-src/BulletMultiThreaded/btGpu3DGridBroadphase.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTetrahedronShape.o: bullet-src/BulletCollision/CollisionShapes/btTetrahedronShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTetrahedronShape.o bullet-src/BulletCollision/CollisionShapes/btTetrahedronShape.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionWorld.o: bullet-src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCollisionWorld.o bullet-src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMesh.o: bullet-src/BulletCollision/CollisionShapes/btTriangleMesh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleMesh.o bullet-src/BulletCollision/CollisionShapes/btTriangleMesh.cpp

${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o: bullet-src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o bullet-src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp

${OBJECTDIR}/bullet-src/MiniCL/MiniCLTask/MiniCLTask.o: bullet-src/MiniCL/MiniCLTask/MiniCLTask.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/MiniCL/MiniCLTask
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/MiniCL/MiniCLTask/MiniCLTask.o bullet-src/MiniCL/MiniCLTask/MiniCLTask.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btGhostObject.o: bullet-src/BulletCollision/CollisionDispatch/btGhostObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btGhostObject.o bullet-src/BulletCollision/CollisionDispatch/btGhostObject.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o: bullet-src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o bullet-src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp

${OBJECTDIR}/bullet-src/LinearMath/btGeometryUtil.o: bullet-src/LinearMath/btGeometryUtil.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/LinearMath/btGeometryUtil.o bullet-src/LinearMath/btGeometryUtil.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o: bullet-src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o bullet-src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp

${OBJECTDIR}/bio/SineSound.o: bio/SineSound.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/SineSound.o bio/SineSound.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o: bullet-src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o bullet-src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o: bullet-src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o bullet-src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o: bullet-src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o bullet-src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyHelpers.o: bullet-src/BulletSoftBody/btSoftBodyHelpers.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftBodyHelpers.o bullet-src/BulletSoftBody/btSoftBodyHelpers.cpp

${OBJECTDIR}/bullet-gl/Win32AppMain.o: bullet-gl/Win32AppMain.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/Win32AppMain.o bullet-gl/Win32AppMain.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o: bullet-src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o bullet-src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp

${OBJECTDIR}/bullet-gl/GLDebugDrawer.o: bullet-gl/GLDebugDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GLDebugDrawer.o bullet-gl/GLDebugDrawer.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btUniformScalingShape.o: bullet-src/BulletCollision/CollisionShapes/btUniformScalingShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btUniformScalingShape.o bullet-src/BulletCollision/CollisionShapes/btUniformScalingShape.cpp

${OBJECTDIR}/bullet-gl/GlutDemoApplication.o: bullet-gl/GlutDemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GlutDemoApplication.o bullet-gl/GlutDemoApplication.cpp

${OBJECTDIR}/bullet-gl/GlutStuff.o: bullet-gl/GlutStuff.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-gl
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-gl/GlutStuff.o bullet-gl/GlutStuff.cpp

${OBJECTDIR}/obj/Spider.o: obj/Spider.cpp 
	${MKDIR} -p ${OBJECTDIR}/obj
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/obj/Spider.o obj/Spider.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o: bullet-src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o bullet-src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp

${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o: bullet-src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o bullet-src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o: bullet-src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o bullet-src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp

${OBJECTDIR}/bullet-src/LinearMath/btConvexHull.o: bullet-src/LinearMath/btConvexHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/LinearMath/btConvexHull.o bullet-src/LinearMath/btConvexHull.cpp

${OBJECTDIR}/obj/Humanoid.o: obj/Humanoid.cpp 
	${MKDIR} -p ${OBJECTDIR}/obj
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/obj/Humanoid.o obj/Humanoid.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConeShape.o: bullet-src/BulletCollision/CollisionShapes/btConeShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btConeShape.o bullet-src/BulletCollision/CollisionShapes/btConeShape.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btDefaultSoftBodySolver.o: bullet-src/BulletSoftBody/btDefaultSoftBodySolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btDefaultSoftBodySolver.o bullet-src/BulletSoftBody/btDefaultSoftBodySolver.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o: bullet-src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o bullet-src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp

${OBJECTDIR}/bio/SixDoFMotor.o: bio/SixDoFMotor.cpp 
	${MKDIR} -p ${OBJECTDIR}/bio
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bio/SixDoFMotor.o bio/SixDoFMotor.cpp

${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/Bullet-C-API.o: bullet-src/BulletDynamics/Dynamics/Bullet-C-API.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletDynamics/Dynamics/Bullet-C-API.o bullet-src/BulletDynamics/Dynamics/Bullet-C-API.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionObjectWrapper.o: bullet-src/BulletMultiThreaded/SpuCollisionObjectWrapper.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuCollisionObjectWrapper.o bullet-src/BulletMultiThreaded/SpuCollisionObjectWrapper.cpp

${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuFakeDma.o: bullet-src/BulletMultiThreaded/SpuFakeDma.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletMultiThreaded
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletMultiThreaded/SpuFakeDma.o bullet-src/BulletMultiThreaded/SpuFakeDma.cpp

${OBJECTDIR}/obj/Snake.o: obj/Snake.cpp 
	${MKDIR} -p ${OBJECTDIR}/obj
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/obj/Snake.o obj/Snake.cpp

${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftSoftCollisionAlgorithm.o: bullet-src/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletSoftBody/btSoftSoftCollisionAlgorithm.o bullet-src/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp

${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o: bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -g -O -Ibullet-gl -Ibullet-src -Isrc -Iobj -Ineural -Ibio -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o bullet-src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Debug
	${RM} dist/Debug/GNU-Linux-x86/spacegraphc

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
