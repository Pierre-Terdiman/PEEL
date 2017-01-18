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

#define PHYSX_SUPPORT_SHARED_SHAPES
#define PHYSX_SUPPORT_PX_BROADPHASE_TYPE

typedef PxPruningStructure	PhysxPruningStructure;
