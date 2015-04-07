///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_SQ_H
#define PINT_SQ_H

#include "PintDef.h"

	class Pint;
	class PintRender;
	struct PintRaycastHit;
	struct PintRaycastData;
	struct PintBoxSweepData;
	struct PintSphereSweepData;
	struct PintCapsuleSweepData;
	struct PintConvexSweepData;
	struct PintSphereOverlapData;
	struct PintBoxOverlapData;
	struct PintCapsuleOverlapData;
	struct PintBooleanHit;
	struct PintOverlapObjectHit;

	template <typename Type>
	class Hits
	{
		public:
						Hits() :
							mNbHits		(0),
							mMaxNbHits	(0),
							mHits		(null)
						{
						}
						~Hits()
						{
							ASSERT(!mHits);
						}

			void		Reset()
						{
							mNbHits = 0;
							mMaxNbHits = 0;
							DELETEARRAY(mHits);
						}

			Type*		PrepareQuery(udword nb)
						{
							mNbHits = nb;
							if(nb>mMaxNbHits)
							{
								DELETEARRAY(mHits);
								mHits = ICE_NEW(Type)[nb];
								mMaxNbHits = nb;
							}
							return mHits;
						}

			udword		mNbHits;
			udword		mMaxNbHits;
			Type*		mHits;
	};

	// Manages Pint-dependent SQ-related data
	class PintSQ
	{
		public:
												PintSQ();
												~PintSQ();

				void							Init(Pint* owner);
		// Raycasts
				PintRaycastHit*					PrepareRaycastQuery					(udword nb, const PintRaycastData* data);
				PintBooleanHit*					PrepareRaycastAnyQuery				(udword nb, const PintRaycastData* data);
				PintOverlapObjectHit*			PrepareRaycastAllQuery				(udword nb, const PintRaycastData* data);
		// Sweeps
				PintRaycastHit*					PrepareBoxSweepQuery				(udword nb, const PintBoxSweepData* data);
				PintRaycastHit*					PrepareSphereSweepQuery				(udword nb, const PintSphereSweepData* data);
				PintRaycastHit*					PrepareCapsuleSweepQuery			(udword nb, const PintCapsuleSweepData* data);
				PintRaycastHit*					PrepareConvexSweepQuery				(udword nb, const PintConvexSweepData* data);
		// Overlaps
				PintBooleanHit*					PrepareSphereOverlapAnyQuery		(udword nb, const PintSphereOverlapData* data);
				PintOverlapObjectHit*			PrepareSphereOverlapObjectsQuery	(udword nb, const PintSphereOverlapData* data);
				PintBooleanHit*					PrepareBoxOverlapAnyQuery			(udword nb, const PintBoxOverlapData* data);
				PintOverlapObjectHit*			PrepareBoxOverlapObjectsQuery		(udword nb, const PintBoxOverlapData* data);
				PintBooleanHit*					PrepareCapsuleOverlapAnyQuery		(udword nb, const PintCapsuleOverlapData* data);
				PintOverlapObjectHit*			PrepareCapsuleOverlapObjectsQuery	(udword nb, const PintCapsuleOverlapData* data);

				void							Render(PintRender& renderer, bool paused);
				void							Reset();
				void							ResetHitData();

		inline_	PintSQThreadContext				GetThreadContext()	const	{ return mThreadContext;		}

		private:
				Pint*							mOwner;
				PintSQThreadContext				mThreadContext;
				//
				Hits<PintRaycastHit>			mRaycasts;
				Hits<PintBooleanHit>			mRaycastsAny;
				Hits<PintOverlapObjectHit>		mRaycastsAll;
				//
				Hits<PintRaycastHit>			mBoxSweeps;
				Hits<PintRaycastHit>			mSphereSweeps;
				Hits<PintRaycastHit>			mCapsuleSweeps;
				Hits<PintRaycastHit>			mConvexSweeps;
				//
				Hits<PintBooleanHit>			mSphereOverlapAny;
				Hits<PintBooleanHit>			mBoxOverlapAny;
				Hits<PintBooleanHit>			mCapsuleOverlapAny;
				Hits<PintOverlapObjectHit>		mSphereOverlapObjects;
				Hits<PintOverlapObjectHit>		mBoxOverlapObjects;
				Hits<PintOverlapObjectHit>		mCapsuleOverlapObjects;

		// Following test-related data is only used for rendering results
				const PintRaycastData*			mRaycastData;
				//
				const PintBoxSweepData*			mBoxSweepData;
				const PintSphereSweepData*		mSphereSweepData;
				const PintCapsuleSweepData*		mCapsuleSweepData;
				const PintConvexSweepData*		mConvexSweepData;
				//
				const PintSphereOverlapData*	mSphereOverlapData;
				const PintBoxOverlapData*		mBoxOverlapData;
				const PintCapsuleOverlapData*	mCapsuleOverlapData;

				void							ResetAllDataPointers();
	};

#endif