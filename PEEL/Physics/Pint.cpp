///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Pint.h"
#include "Render.h"

///////////////////////////////////////////////////////////////////////////////

PintCaps::PintCaps() :
	mSupportRigidBodySimulation	(false),
	mSupportKinematics			(false),
	mSupportCollisionGroups		(false),
	mSupportCompounds			(false),
	mSupportConvexes			(false),
	mSupportMeshes				(false),
	mSupportSphericalJoints		(false),
	mSupportHingeJoints			(false),
	mSupportFixedJoints			(false),
	mSupportPrismaticJoints		(false),
	mSupportPhantoms			(false),
	mSupportRaycasts			(false),
	mSupportBoxSweeps			(false),
	mSupportSphereSweeps		(false),
	mSupportCapsuleSweeps		(false),
	mSupportConvexSweeps		(false),
	mSupportSphereOverlaps		(false),
	mSupportBoxOverlaps			(false),
	mSupportCapsuleOverlaps		(false),
	mSupportConvexOverlaps		(false)
{
}

///////////////////////////////////////////////////////////////////////////////
