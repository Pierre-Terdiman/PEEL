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

#define UINT32_MAX MAX_UDWORD
#define UINT8_MAX MAX_UBYTE

#include "PxPhysicsAPI.h"
#include "PsFoundation.h"
#include "PsUtilities.h"

using namespace physx;

#define PHYSX_SUPPORT_GPU
#ifdef PHYSX_SUPPORT_GPU
	#define BUILD_GPU_DATA	buildGPUData
#endif
#define BUILD_GPU_DATA	buildGPUData
#define PHYSX_SUPPORT_SHARED_SHAPES
#define PHYSX_SUPPORT_ARTICULATIONS
#define PHYSX_SUPPORT_PX_MESH_MIDPHASE
#define PHYSX_SUPPORT_PX_MESH_MIDPHASE2
#define PHYSX_SUPPORT_PX_MESH_COOKING_HINT
#define PHYSX_SUPPORT_PX_BROADPHASE_TYPE
#define PHYSX_SUPPORT_SCRATCH_BUFFER
#define PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
#define PHYSX_SUPPORT_STABILIZATION_FLAG
#define PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
#define PHYSX_SUPPORT_INSERTION_CALLBACK
#define PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
#define PHYSX_SUPPORT_VEHICLE_SUSPENSION_SWEEPS
#define PHYSX_SUPPORT_DISABLE_PREPROCESSING
#define PHYSX_SUPPORT_RAYCAST_CCD
#define PHYSX_SUPPORT_ANGULAR_CCD
#define PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
#define PHYSX_REMOVE_JOINT_32_COMPATIBILITY
#define PHYSX_SUPPORT_SUBSTEPS

// Copy of deprecated 3.3 stuff
#define PxSceneQueryFlag PxHitFlag
#define PxSceneQueryFlags PxHitFlags
#define PxSceneQueryHit PxQueryHit
#define PxSceneQueryFilterData PxQueryFilterData
#define PxSceneQueryFilterFlag PxQueryFlag
#define PxSceneQueryFilterFlags PxQueryFlags
#define PxSceneQueryFilterCallback PxQueryFilterCallback
#define PxSceneQueryCache PxQueryCache

PX_INLINE bool raycastAny(PxScene* scene,
const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryHit& hit, const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	fdAny.clientId = queryClient;
	PxRaycastBuffer buf;
	scene->raycast(origin, unitDir, distance, buf, PxHitFlags(), fdAny, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

PX_INLINE bool raycastSingle(PxScene* scene,
const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryFlags outputFlags, PxRaycastHit& hit,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	PxRaycastBuffer buf;
	PxQueryFilterData fd1 = filterData; fd1.clientId = queryClient;
	scene->raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

PX_INLINE PxI32 raycastMultiple(PxScene* scene,
const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryFlags outputFlags,
PxRaycastHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	PxRaycastBuffer buf(hitBuffer, hitBufferSize);
	PxQueryFilterData fd1 = filterData; fd1.clientId = queryClient;
	scene->raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	blockingHit = buf.hasBlock;
	if (blockingHit)
	{
		if (buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return buf.nbTouches+1;
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return buf.nbTouches;
}

PX_INLINE bool sweepAny(PxScene* scene,
const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryFlags queryFlags,
PxSceneQueryHit& hit,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL,
const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT,
const PxReal inflation = 0.f)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	fdAny.clientId = queryClient;
	PxSweepBuffer buf;
	scene->sweep(geometry, pose, unitDir, distance,
		buf, queryFlags, fdAny, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

PX_INLINE bool sweepSingle(PxScene* scene,
const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryFlags outputFlags,
PxSweepHit& hit,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL,
const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT, const PxReal inflation=0.f)
{
	PxSweepBuffer buf;
	PxQueryFilterData fd1 = filterData; fd1.clientId = queryClient;
	scene->sweep(geometry, pose, unitDir, distance, buf,
		outputFlags, fd1, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

PX_INLINE PxI32 sweepMultiple(PxScene* scene,
const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
PxSceneQueryFlags outputFlags, PxSweepHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT, const PxReal inflation = 0.f)
{
	PxQueryFilterData fd1 = filterData; fd1.clientId = queryClient;
	PxSweepBuffer buf(hitBuffer, hitBufferSize);
	scene->sweep(
		geometry, pose, unitDir, distance, buf, outputFlags, fd1, filterCall,
		cache, inflation);
	blockingHit = buf.hasBlock;
	if (blockingHit)
	{
		if (buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return buf.nbTouches+1;
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return buf.nbTouches;
}

PX_INLINE PxI32 overlapMultiple(PxScene* scene,
const PxGeometry& geometry, const PxTransform& pose,
PxOverlapHit* hitBuffer, PxU32 hitBufferSize,
const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
PxSceneQueryFilterCallback* filterCall = NULL,
PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	PxQueryFilterData fd1 = filterData; fd1.clientId = queryClient;
	fd1.flags |= PxQueryFlag::eNO_BLOCK;
	PxOverlapBuffer buf(hitBuffer, hitBufferSize);
	scene->overlap(geometry, pose, buf, fd1, filterCall);
	if (buf.hasBlock)
	{
		if (buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return buf.nbTouches+1;
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return buf.nbTouches;
}

PX_INLINE bool overlapAny(PxScene* scene,
	const PxGeometry& geometry, const PxTransform& pose,
	PxOverlapHit& hit,
	const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
	PxSceneQueryFilterCallback* filterCall = NULL,
	PxClientID queryClient = PX_DEFAULT_CLIENT)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= (PxQueryFlag::eANY_HIT | PxQueryFlag::eNO_BLOCK);
	fdAny.clientId = queryClient;
	PxOverlapBuffer buf;
	scene->overlap(geometry, pose, buf, fdAny, filterCall);
	hit = buf.block;
	return buf.hasBlock;
}
//~Copy of deprecated 3.3 stuff


