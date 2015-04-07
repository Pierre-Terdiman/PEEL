///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PintSQ.h"
#include "Pint.h"

PintSQ::PintSQ() :
	mOwner				(null),
	mThreadContext		(null),
	//
	mRaycastData		(null),
	mBoxSweepData		(null),
	mSphereSweepData	(null),
	mCapsuleSweepData	(null),
	mConvexSweepData	(null),
	mSphereOverlapData	(null),
	mBoxOverlapData		(null),
	mCapsuleOverlapData	(null)
{
}

PintSQ::~PintSQ()
{
	ASSERT(!mCapsuleOverlapData);
	ASSERT(!mBoxOverlapData);
	ASSERT(!mSphereOverlapData);
	ASSERT(!mConvexSweepData);
	ASSERT(!mCapsuleSweepData);
	ASSERT(!mSphereSweepData);
	ASSERT(!mBoxSweepData);
	ASSERT(!mRaycastData);
}

void PintSQ::ResetAllDataPointers()
{
	mRaycastData		= null;
	mBoxSweepData		= null;
	mSphereSweepData	= null;
	mCapsuleSweepData	= null;
	mConvexSweepData	= null;
	mSphereOverlapData	= null;
	mBoxOverlapData		= null;
	mCapsuleOverlapData	= null;
}

void PintSQ::ResetHitData()
{
	mRaycasts.Reset();
	mRaycastsAny.Reset();
	mRaycastsAll.Reset();
	//
	mBoxSweeps.Reset();
	mSphereSweeps.Reset();
	mCapsuleSweeps.Reset();
	mConvexSweeps.Reset();
	//
	mSphereOverlapAny.Reset();
	mBoxOverlapAny.Reset();
	mCapsuleOverlapAny.Reset();
	mSphereOverlapObjects.Reset();
	mBoxOverlapObjects.Reset();
	mCapsuleOverlapObjects.Reset();
}

void PintSQ::Reset()
{
	if(mThreadContext)
	{
		mOwner->ReleaseSQThreadContext(mThreadContext);
		mThreadContext = null;
	}
	mOwner = null;

	ResetHitData();

	ResetAllDataPointers();
}

void PintSQ::Init(Pint* owner)
{
	Reset();
	mOwner	= owner;
	mThreadContext = owner->CreateSQThreadContext();
	owner->mSQHelper = this;
}

///////////////////////////////////////////////////////////////////////////////

PintRaycastHit* PintSQ::PrepareRaycastQuery(udword nb, const PintRaycastData* data)
{
//	ResetAllDataPointers();
	mRaycastData = data;
	return mRaycasts.PrepareQuery(nb);
}

PintBooleanHit* PintSQ::PrepareRaycastAnyQuery(udword nb, const PintRaycastData* data)
{
//	ResetAllDataPointers();
	mRaycastData = data;
	return mRaycastsAny.PrepareQuery(nb);
}

PintOverlapObjectHit* PintSQ::PrepareRaycastAllQuery(udword nb, const PintRaycastData* data)
{
//	ResetAllDataPointers();
	mRaycastData = data;
	return mRaycastsAll.PrepareQuery(nb);
}

///////////////////////////////////////////////////////////////////////////////

PintRaycastHit* PintSQ::PrepareBoxSweepQuery(udword nb, const PintBoxSweepData* data)
{
//	ResetAllDataPointers();
	mBoxSweepData = data;
	return mBoxSweeps.PrepareQuery(nb);
}

PintRaycastHit* PintSQ::PrepareSphereSweepQuery(udword nb, const PintSphereSweepData* data)
{
//	ResetAllDataPointers();
	mSphereSweepData = data;
	return mSphereSweeps.PrepareQuery(nb);
}

PintRaycastHit* PintSQ::PrepareCapsuleSweepQuery(udword nb, const PintCapsuleSweepData* data)
{
//	ResetAllDataPointers();
	mCapsuleSweepData = data;
	return mCapsuleSweeps.PrepareQuery(nb);
}

PintRaycastHit* PintSQ::PrepareConvexSweepQuery(udword nb, const PintConvexSweepData* data)
{
//	ResetAllDataPointers();
	mConvexSweepData = data;
	return mConvexSweeps.PrepareQuery(nb);
}

///////////////////////////////////////////////////////////////////////////////

PintBooleanHit* PintSQ::PrepareSphereOverlapAnyQuery(udword nb, const PintSphereOverlapData* data)
{
//	ResetAllDataPointers();
	mSphereOverlapData = data;
	return mSphereOverlapAny.PrepareQuery(nb);
}

PintOverlapObjectHit* PintSQ::PrepareSphereOverlapObjectsQuery(udword nb, const PintSphereOverlapData* data)
{
//	ResetAllDataPointers();
	mSphereOverlapData = data;
	return mSphereOverlapObjects.PrepareQuery(nb);
}

PintBooleanHit* PintSQ::PrepareBoxOverlapAnyQuery(udword nb, const PintBoxOverlapData* data)
{
//	ResetAllDataPointers();
	mBoxOverlapData = data;
	return mBoxOverlapAny.PrepareQuery(nb);
}

PintOverlapObjectHit* PintSQ::PrepareBoxOverlapObjectsQuery(udword nb, const PintBoxOverlapData* data)
{
//	ResetAllDataPointers();
	mBoxOverlapData = data;
	return mBoxOverlapObjects.PrepareQuery(nb);
}

PintBooleanHit* PintSQ::PrepareCapsuleOverlapAnyQuery(udword nb, const PintCapsuleOverlapData* data)
{
//	ResetAllDataPointers();
	mCapsuleOverlapData = data;
	return mCapsuleOverlapAny.PrepareQuery(nb);
}

PintOverlapObjectHit* PintSQ::PrepareCapsuleOverlapObjectsQuery(udword nb, const PintCapsuleOverlapData* data)
{
//	ResetAllDataPointers();
	mCapsuleOverlapData = data;
	return mCapsuleOverlapObjects.PrepareQuery(nb);
}

///////////////////////////////////////////////////////////////////////////////

void PintSQ::Render(PintRender& renderer, bool paused)
{
	if(!(mOwner->GetFlags() & PINT_IS_ACTIVE))
		return;
	const Point Color = mOwner->GetMainColor();
	renderer.DrawRaycastData				(mRaycasts.mNbHits,					mRaycastData,			mRaycasts.mHits,				Color);
	renderer.DrawRaycastAnyData				(mRaycastsAny.mNbHits,				mRaycastData,			mRaycastsAny.mHits,				Color);
	renderer.DrawBoxSweepData				(mBoxSweeps.mNbHits,				mBoxSweepData,			mBoxSweeps.mHits,				Color);
	renderer.DrawSphereSweepData			(mSphereSweeps.mNbHits,				mSphereSweepData,		mSphereSweeps.mHits,			Color);
	renderer.DrawCapsuleSweepData			(mCapsuleSweeps.mNbHits,			mCapsuleSweepData,		mCapsuleSweeps.mHits,			Color);
	renderer.DrawConvexSweepData			(mConvexSweeps.mNbHits,				mConvexSweepData,		mConvexSweeps.mHits,			Color);
	renderer.DrawSphereOverlapAnyData		(mSphereOverlapAny.mNbHits,			mSphereOverlapData,		mSphereOverlapAny.mHits,		Color);
	renderer.DrawSphereOverlapObjectsData	(mSphereOverlapObjects.mNbHits,		mSphereOverlapData,		mSphereOverlapObjects.mHits,	Color);
	renderer.DrawBoxOverlapAnyData			(mBoxOverlapAny.mNbHits,			mBoxOverlapData,		mBoxOverlapAny.mHits,			Color);
	renderer.DrawBoxOverlapObjectsData		(mBoxOverlapObjects.mNbHits,		mBoxOverlapData,		mBoxOverlapObjects.mHits,		Color);
	renderer.DrawCapsuleOverlapAnyData		(mCapsuleOverlapAny.mNbHits,		mCapsuleOverlapData,	mCapsuleOverlapAny.mHits,		Color);
	renderer.DrawCapsuleOverlapObjectsData	(mCapsuleOverlapObjects.mNbHits,	mCapsuleOverlapData,	mCapsuleOverlapObjects.mHits,	Color);
	//###TOFIX
//	if(!paused)
//		ResetAllDataPointers();
}

///////////////////////////////////////////////////////////////////////////////

