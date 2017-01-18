///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#pragma once

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#include "..\PINT_Common\PINT_Ice.h"

#include <vector>

#define HK_CONFIG_SIMD HK_CONFIG_SIMD_ENABLED

#include <Common/Base/hkBase.h>
#include <Common/Base/Container/String/hkStringBuf.h>
#include <Common/Base/Ext/hkBaseExt.h>

#include <Common/Base/KeyCode.h>
#include <Common/Base/keycode.cxx>
#undef HK_FEATURE_PRODUCT_AI
#undef HK_FEATURE_PRODUCT_ANIMATION
#undef HK_FEATURE_PRODUCT_BEHAVIOR
#undef HK_FEATURE_PRODUCT_CLOTH
#undef HK_FEATURE_PRODUCT_DESTRUCTION
#undef HK_FEATURE_PRODUCT_AI
#undef HK_FEATURE_PRODUCT_SCRIPT
#include <Common/Base/Config/hkProductFeatures.cxx>

#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/Memory/System/hkMemorySystem.h>
#include <Common/Base/System/Error/hkDefaultError.h>
#include <Common/Base/Monitor/hkMonitorStream.h>
#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>

#include <Physics2012/Collide/hkpCollide.h>										
#include <Physics2012/Collide/Agent/ConvexAgent/SphereBox/hkpSphereBoxAgent.h>	
#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>					
#include <Physics2012/Collide/Shape/Convex/Sphere/hkpSphereShape.h>		
#include <Physics2012/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>		
#include <Physics2012/Collide/Shape/Convex/Cylinder/hkpCylinderShape.h>		
#include <Physics2012/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics2012/Collide/Shape/Compound\Collection\ExtendedMeshShape/hkpExtendedMeshShape.h>
#include <Physics2012/Collide/Shape/Compound\Collection\StorageExtendedMesh/hkpStorageExtendedMeshShape.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>					
#include <Physics2012/Internal/Collide/BvCompressedMesh/hkpBvCompressedMeshShape.h>
#include <Physics2012/Internal/Collide/BvCompressedMesh/hkpBvCompressedMeshShapeCinfo.h>

#include <Physics2012/Collide/Query/Collector/PointCollector/hkpClosestCdPointCollector.h>
#include <Physics2012/Collide/Query/Collector/BodyPairCollector/hkpFirstCdBodyPairCollector.h>
#include <Physics2012/Collide/Query/Collector/BodyPairCollector/hkpAllCdBodyPairCollector.h>
#include <Physics2012/Collide/Query/Collector/BodyPairCollector/hkpFlagCdBodyPairCollector.h>
#include <Physics2012/Collide/Query/CastUtil/hkpLinearCastInput.h>
#include <Physics2012/Collide/Query/CastUtil/hkpWorldRayCastInput.h>			
#include <Physics2012/Collide/Query/CastUtil/hkpWorldRayCastOutput.h>			
#include <Physics2012\Collide\Filter\Group\hkpGroupFilter.h>

#include <Physics2012/Dynamics/World/hkpWorld.h>								
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>							
#include <Physics\Constraint\Data\BallAndSocket\hkpBallAndSocketConstraintData.h>
#include <Physics\Constraint\Data\Hinge\hkpHingeConstraintData.h>
#include <Physics\Constraint\Data\LimitedHinge\hkpLimitedHingeConstraintData.h>
#include <Physics/Constraint/Data/Prismatic/hkpPrismaticConstraintData.h>
#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>	

//#include <Common/Base/Thread/Job/ThreadPool/Cpu/hkCpuJobThreadPool.h>
#include <Common/Base/Thread/Pool/hkCpuThreadPool.h>
#include <Common/Base/Thread/JobQueue/hkJobQueue.h>

#include <Physics2012/Internal/Collide/Mopp/Code/hkpMoppCode.h>
#include <Physics2012/Collide/Shape/Compound/Tree/Mopp/hkpMoppUtility.h>
#include <Physics2012/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>

// Visual Debugger includes
#include <Common/Visualize/hkVisualDebugger.h>
#include <Physics2012/Utilities/VisualDebugger/hkpPhysicsContext.h>				

/*
// Classlists
#define INCLUDE_HAVOK_PHYSICS_CLASSES
#define HK_CLASSES_FILE <Common/Serialize/Classlist/hkClasses.h>
#include <Common/Serialize/Util/hkBuiltinTypeRegistry.cxx>

// Generate a custom list to trim memory requirements
#define HK_COMPAT_FILE <Common/Compat/hkCompatVersions.h>
//#include <Common/Compat/hkCompat_None.cxx>
*/

#include <Physics2012/Collide/Shape/Convex/ConvexTransform/hkpConvexTransformShape.h>		
#include <Physics2012/Collide/Shape/Compound/Collection/List/hkpListShape.h>		
#include <Common/Internal/ConvexHull/hkGeometryUtility.h>

#include <Physics2012/Dynamics/Phantom/hkpAabbPhantom.h>

#include <Physics2012/Utilities/Dynamics/KeyFrame/hkpKeyFrameUtility.h>

//#include <Physics/Collide/Agent/hkpCollisionAgentConfig.h>

///

#include <Physics2012/Dynamics/Constraint/Chain/hkpConstraintChainInstance.h>
#include <Physics2012/Dynamics/Constraint/Chain/BallSocket/hkpBallSocketChainData.h>
#include <Physics2012/Dynamics/Constraint/Chain/StiffSpring/hkpStiffSpringChainData.h>
#include <Physics2012/Dynamics/Constraint/Chain/Powered/hkpPoweredChainData.h>

// Put whatever is needed here to make PINT_CommonHavok.cpp compile
#include "PINT_Havok2014_1_0.h"
