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

#include "PxPhysicsAPI.h"
using namespace physx;

#define IS_PHYSX_3_2
#define PHYSX3_DISABLE_SHARED_APPLY_ACTION
#define PHYSX_SUPPORT_SSE_FLAG
#define PHYSX_SUPPORT_ARTICULATIONS
#define PHYSX_SUPPORT_SCRATCH_BUFFER
#define PHYSX_SUPPORT_SUBSTEPS

#define PxJointLinearLimitPair PxJointLimitPair
#define PxJointAngularLimitPair PxJointLimitPair

#define PxJointLinearLimitPair PxJointLimitPair
#define PxJointAngularLimitPair PxJointLimitPair

typedef PxPruningStructure		PxPruningStructureType;
typedef PxSceneQueryFilterData	PxQueryFilterData;
typedef PxSceneQueryFilterFlag	PxQueryFlag;

#define PxOverlapHit	PxShape*

PX_INLINE bool raycastAny(	PxScene* scene,
							const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
							PxSceneQueryHit& hit, const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
							PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	return scene->raycastAny(origin, unitDir, distance, hit, filterData, filterCall, cache, queryClient);
}

PX_INLINE bool overlapAny(	PxScene* scene,
							const PxGeometry& geometry, const PxTransform& pose,
							PxOverlapHit& hit,
							const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL,
							PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	return scene->overlapAny( geometry, pose, hit, filterData, filterCall, queryClient);
}

PX_INLINE PxI32 overlapMultiple(PxScene* scene,
								const PxGeometry& geometry, const PxTransform& pose,
								PxOverlapHit* hitBuffer, PxU32 hitBufferSize,
								const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
								PxSceneQueryFilterCallback* filterCall = NULL,
								PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	return scene->overlapMultiple( geometry, pose, hitBuffer, hitBufferSize, filterData, filterCall, queryClient);
}
