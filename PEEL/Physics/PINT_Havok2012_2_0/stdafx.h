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

#include <Physics/Collide/hkpCollide.h>										
#include <Physics/Collide/Agent/ConvexAgent/SphereBox/hkpSphereBoxAgent.h>	
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>					
#include <Physics/Collide/Shape/Convex/Sphere/hkpSphereShape.h>		
#include <Physics/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>		
#include <Physics/Collide/Shape/Convex/Cylinder/hkpCylinderShape.h>		
#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics/Collide/Shape/Compound\Collection\ExtendedMeshShape/hkpExtendedMeshShape.h>
#include <Physics/Collide/Shape/Compound\Collection\StorageExtendedMesh/hkpStorageExtendedMeshShape.h>
#include <Physics/Collide/Dispatch/hkpAgentRegisterUtil.h>					
#include <Physics/Internal/Collide/BvCompressedMesh/hkpBvCompressedMeshShape.h>
#include <Physics/Internal/Collide/BvCompressedMesh/hkpBvCompressedMeshShapeCinfo.h>

#include <Physics/Collide/Query/Collector/PointCollector/hkpClosestCdPointCollector.h>
#include <Physics/Collide/Query/Collector/BodyPairCollector/hkpFirstCdBodyPairCollector.h>
#include <Physics/Collide/Query/Collector/BodyPairCollector/hkpAllCdBodyPairCollector.h>
#include <Physics/Collide/Query/Collector/BodyPairCollector/hkpFlagCdBodyPairCollector.h>
#include <Physics/Collide/Query/CastUtil/hkpLinearCastInput.h>
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>			
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastOutput.h>			
#include <Physics\Collide\Filter\Group\hkpGroupFilter.h>

#include <Physics/Dynamics/World/hkpWorld.h>								
#include <Physics/Dynamics/Entity/hkpRigidBody.h>							
#include <Physics\Constraint\Data\BallAndSocket\hkpBallAndSocketConstraintData.h>
#include <Physics\Constraint\Data\Hinge\hkpHingeConstraintData.h>
#include <Physics\Constraint\Data\LimitedHinge\hkpLimitedHingeConstraintData.h>
#include <Physics/Constraint/Data/Prismatic/hkpPrismaticConstraintData.h>
#include <Physics/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>	

#include <Common/Base/Thread/Job/ThreadPool/Cpu/hkCpuJobThreadPool.h>
#include <Common/Base/Thread/JobQueue/hkJobQueue.h>

#include <Physics/Internal/Collide/Mopp/Code/hkpMoppCode.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppUtility.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>

// Visual Debugger includes
#include <Common/Visualize/hkVisualDebugger.h>
#include <Physics/Utilities/VisualDebugger/hkpPhysicsContext.h>				

/*
// Classlists
#define INCLUDE_HAVOK_PHYSICS_CLASSES
#define HK_CLASSES_FILE <Common/Serialize/Classlist/hkClasses.h>
#include <Common/Serialize/Util/hkBuiltinTypeRegistry.cxx>

// Generate a custom list to trim memory requirements
#define HK_COMPAT_FILE <Common/Compat/hkCompatVersions.h>
//#include <Common/Compat/hkCompat_None.cxx>
*/

#include <Physics/Collide/Shape/Convex/ConvexTransform/hkpConvexTransformShape.h>		
#include <Physics/Collide/Shape/Compound/Collection/List/hkpListShape.h>		
#include <Common/Internal/ConvexHull/hkGeometryUtility.h>

#include <Physics/Dynamics/Phantom/hkpAabbPhantom.h>

#include <Physics/Utilities/Dynamics/KeyFrame/hkpKeyFrameUtility.h>

//#include <Physics/Collide/Agent/hkpCollisionAgentConfig.h>

///

#include <Physics/Dynamics/Constraint/Chain/hkpConstraintChainInstance.h>
#include <Physics/Dynamics/Constraint/Chain/BallSocket/hkpBallSocketChainData.h>
#include <Physics/Dynamics/Constraint/Chain/StiffSpring/hkpStiffSpringChainData.h>
#include <Physics/Dynamics/Constraint/Chain/Powered/hkpPoweredChainData.h>

// Put whatever is needed here to make PINT_CommonHavok.cpp compile
#include "PINT_Havok2012_2_0.h"
