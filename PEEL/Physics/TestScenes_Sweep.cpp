///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "MyConvex.h"
#include "Loader_RepX.h"
#include "Loader_Bin.h"
#include "Random.h"

///////////////////////////////////////////////////////////////////////////////

	class SceneSweepVsShapes : public TestBase
	{
		public:
								SceneSweepVsShapes()			{ mType = PINT_SHAPE_UNDEFINED;	mTypeData = 4;	mDynamic = false;	}
		virtual					~SceneSweepVsShapes()			{																	}
		virtual	TestCategory	GetCategory()			const	{ return CATEGORY_SWEEP;											}
		virtual	bool			ProfileUpdate()					{ return true;														}

		virtual void			GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.55f, -0.59f));
			desc.mCamera[1] = CameraPose(Point(52.39f, 11.74f, 47.06f), Point(-0.69f, -0.17f, 0.70f));
		}

				PintShape		mType;
				udword			mTypeData;
				bool			mDynamic;
	};

	class SceneBoxSweepVsShapes : public SceneSweepVsShapes
	{
		public:
		virtual	udword			Update(Pint& pint)				{ return DoBatchBoxSweeps(*this, pint);				}

		virtual bool			CommonSetup()
		{
			TestBase::CommonSetup();
			return GenerateArrayOfVerticalBoxSweeps(*this, 50.0f, 32, 32, gSQMaxDist);
		}

		virtual bool			Setup(Pint& pint, const PintCaps& caps)
		{
			if(!caps.mSupportBoxSweeps)
				return false;

			const udword NbX = 32;
			const udword NbY = 32;
			const float Altitude = 10.0f;
			const float Scale = 50.0f;
			const float Mass = mDynamic ? 1.0f : 0.0f;

			return GenerateArrayOfObjects(pint, caps, mType, mTypeData, NbX, NbY, Altitude, Scale, Mass);
		}

		virtual void			CommonUpdate()
		{
			mCurrentTime += 1.0f/60.0f;
			UpdateBoxSweeps(*this, mCurrentTime);
		}
	};

#define START_SQ_BOX_SWEEP_VS_SHAPES(name, desc, type, is_dynamic)										\
	class name : public SceneBoxSweepVsShapes															\
	{																									\
		public:																							\
								name()						{ mType = type;	mDynamic = is_dynamic;	}	\
		virtual					~name()						{										}	\
		virtual	const char*		GetName()			const	{ return #name;							}	\
		virtual	const char*		GetDescription()	const	{ return desc;							}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepOverhead = "Empty scene used to measure the operating overhead of 32*32 box-sweeps.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepOverhead, gDesc_SceneBoxSweepOverhead, PINT_SHAPE_UNDEFINED, false)
END_TEST(SceneBoxSweepOverhead)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticSpheres = "32*32 box-sweeps against 32*32 static spheres.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsStaticSpheres, gDesc_SceneBoxSweepVsStaticSpheres, PINT_SHAPE_SPHERE, false)
END_TEST(SceneBoxSweepVsStaticSpheres)

static const char* gDesc_SceneBoxSweepVsDynamicSpheres = "32*32 box-sweeps against 32*32 dynamic spheres.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsDynamicSpheres, gDesc_SceneBoxSweepVsDynamicSpheres, PINT_SHAPE_SPHERE, true)
END_TEST(SceneBoxSweepVsDynamicSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticCapsules = "32*32 box-sweeps against 32*32 static capsules.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsStaticCapsules, gDesc_SceneBoxSweepVsStaticCapsules, PINT_SHAPE_CAPSULE, false)
END_TEST(SceneBoxSweepVsStaticCapsules)

static const char* gDesc_SceneBoxSweepVsDynamicCapsules = "32*32 box-sweeps against 32*32 dynamic capsules.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsDynamicCapsules, gDesc_SceneBoxSweepVsDynamicCapsules, PINT_SHAPE_CAPSULE, true)
END_TEST(SceneBoxSweepVsDynamicCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticBoxes = "32*32 box-sweeps against 32*32 static boxes.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsStaticBoxes, gDesc_SceneBoxSweepVsStaticBoxes, PINT_SHAPE_BOX, false)
END_TEST(SceneBoxSweepVsStaticBoxes)

static const char* gDesc_SceneBoxSweepVsDynamicBoxes = "32*32 box-sweeps against 32*32 dynamic boxes.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsDynamicBoxes, gDesc_SceneBoxSweepVsDynamicBoxes, PINT_SHAPE_BOX, true)
END_TEST(SceneBoxSweepVsDynamicBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticConvexes = "32*32 box-sweeps against 32*32 static convexes.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsStaticConvexes, gDesc_SceneBoxSweepVsStaticConvexes, PINT_SHAPE_CONVEX, false)
END_TEST(SceneBoxSweepVsStaticConvexes)

static const char* gDesc_SceneBoxSweepVsDynamicConvexes = "32*32 box-sweeps against 32*32 dynamic convexes.";
START_SQ_BOX_SWEEP_VS_SHAPES(SceneBoxSweepVsDynamicConvexes, gDesc_SceneBoxSweepVsDynamicConvexes, PINT_SHAPE_CONVEX, true)
END_TEST(SceneBoxSweepVsDynamicConvexes)

///////////////////////////////////////////////////////////////////////////////

	class SceneSphereSweepVsShapes : public SceneSweepVsShapes
	{
		public:
		virtual	udword			Update(Pint& pint)				{ return DoBatchSphereSweeps(*this, pint);			}

		virtual bool			CommonSetup()
		{
			TestBase::CommonSetup();
			return GenerateArrayOfVerticalSphereSweeps(*this, 50.0f, 32, 32, gSQMaxDist);
		}

		virtual bool			Setup(Pint& pint, const PintCaps& caps)
		{
			if(!caps.mSupportSphereSweeps)
				return false;

			const udword NbX = 32;
			const udword NbY = 32;
			const float Altitude = 10.0f;
			const float Scale = 50.0f;
			const float Mass = mDynamic ? 1.0f : 0.0f;

			return GenerateArrayOfObjects(pint, caps, mType, mTypeData, NbX, NbY, Altitude, Scale, Mass);
		}

		virtual void			CommonUpdate()
		{
			mCurrentTime += 1.0f/60.0f;
			//UpdateSphereSweeps(*this, mCurrentTime);
		}
	};

#define START_SQ_SPHERE_SWEEP_VS_SHAPES(name, desc, type, is_dynamic)									\
	class name : public SceneSphereSweepVsShapes														\
	{																									\
		public:																							\
								name()						{ mType = type;	mDynamic = is_dynamic;	}	\
		virtual					~name()						{										}	\
		virtual	const char*		GetName()			const	{ return #name;							}	\
		virtual	const char*		GetDescription()	const	{ return desc;							}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepOverhead = "Empty scene used to measure the operating overhead of 32*32 sphere-sweeps.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepOverhead, gDesc_SceneSphereSweepOverhead, PINT_SHAPE_UNDEFINED, false)
END_TEST(SceneSphereSweepOverhead)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticSpheres = "32*32 sphere-sweeps against 32*32 static spheres.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsStaticSpheres, gDesc_SceneSphereSweepVsStaticSpheres, PINT_SHAPE_SPHERE, false)
END_TEST(SceneSphereSweepVsStaticSpheres)

static const char* gDesc_SceneSphereSweepVsDynamicSpheres = "32*32 sphere-sweeps against 32*32 dynamic spheres.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsDynamicSpheres, gDesc_SceneSphereSweepVsDynamicSpheres, PINT_SHAPE_SPHERE, true)
END_TEST(SceneSphereSweepVsDynamicSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticCapsules = "32*32 sphere-sweeps against 32*32 static capsules.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsStaticCapsules, gDesc_SceneSphereSweepVsStaticCapsules, PINT_SHAPE_CAPSULE, false)
END_TEST(SceneSphereSweepVsStaticCapsules)

static const char* gDesc_SceneSphereSweepVsDynamicCapsules = "32*32 sphere-sweeps against 32*32 dynamic capsules.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsDynamicCapsules, gDesc_SceneSphereSweepVsDynamicCapsules, PINT_SHAPE_CAPSULE, true)
END_TEST(SceneSphereSweepVsDynamicCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticBoxes = "32*32 sphere-sweeps against 32*32 static boxes.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsStaticBoxes, gDesc_SceneSphereSweepVsStaticBoxes, PINT_SHAPE_BOX, false)
END_TEST(SceneSphereSweepVsStaticBoxes)

static const char* gDesc_SceneSphereSweepVsDynamicBoxes = "32*32 sphere-sweeps against 32*32 dynamic boxes.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsDynamicBoxes, gDesc_SceneSphereSweepVsDynamicBoxes, PINT_SHAPE_BOX, true)
END_TEST(SceneSphereSweepVsDynamicBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticConvexes = "32*32 sphere-sweeps against 32*32 static convexes.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsStaticConvexes, gDesc_SceneSphereSweepVsStaticConvexes, PINT_SHAPE_CONVEX, false)
END_TEST(SceneSphereSweepVsStaticConvexes)

static const char* gDesc_SceneSphereSweepVsDynamicConvexes = "32*32 sphere-sweeps against 32*32 dynamic convexes.";
START_SQ_SPHERE_SWEEP_VS_SHAPES(SceneSphereSweepVsDynamicConvexes, gDesc_SceneSphereSweepVsDynamicConvexes, PINT_SHAPE_CONVEX, true)
END_TEST(SceneSphereSweepVsDynamicConvexes)

///////////////////////////////////////////////////////////////////////////////

	class SceneCapsuleSweepVsShapes : public SceneSweepVsShapes
	{
		public:
		virtual	udword			Update(Pint& pint)				{ return DoBatchCapsuleSweeps(*this, pint);			}

		virtual bool			CommonSetup()
		{
			TestBase::CommonSetup();
			return GenerateArrayOfVerticalCapsuleSweeps(*this, 50.0f, 32, 32, gSQMaxDist);
		}

		virtual bool			Setup(Pint& pint, const PintCaps& caps)
		{
			if(!caps.mSupportCapsuleSweeps)
				return false;

			const udword NbX = 32;
			const udword NbY = 32;
			const float Altitude = 10.0f;
			const float Scale = 50.0f;
			const float Mass = mDynamic ? 1.0f : 0.0f;

			return GenerateArrayOfObjects(pint, caps, mType, mTypeData, NbX, NbY, Altitude, Scale, Mass);
		}

		virtual void			CommonUpdate()
		{
			mCurrentTime += 1.0f/60.0f;
			UpdateCapsuleSweeps(*this, mCurrentTime);
		}
	};

#define START_SQ_CAPSULE_SWEEP_VS_SHAPES(name, desc, type, is_dynamic)									\
	class name : public SceneCapsuleSweepVsShapes														\
	{																									\
		public:																							\
								name()						{ mType = type;	mDynamic = is_dynamic;	}	\
		virtual					~name()						{										}	\
		virtual	const char*		GetName()			const	{ return #name;							}	\
		virtual	const char*		GetDescription()	const	{ return desc;							}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepOverhead = "Empty scene used to measure the operating overhead of 32*32 capsule-sweeps.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepOverhead, gDesc_SceneCapsuleSweepOverhead, PINT_SHAPE_UNDEFINED, false)
END_TEST(SceneCapsuleSweepOverhead)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticSpheres = "32*32 capsule-sweeps against 32*32 static spheres.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsStaticSpheres, gDesc_SceneCapsuleSweepVsStaticSpheres, PINT_SHAPE_SPHERE, false)
END_TEST(SceneCapsuleSweepVsStaticSpheres)

static const char* gDesc_SceneCapsuleSweepVsDynamicSpheres = "32*32 capsule-sweeps against 32*32 dynamic spheres.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsDynamicSpheres, gDesc_SceneCapsuleSweepVsDynamicSpheres, PINT_SHAPE_SPHERE, true)
END_TEST(SceneCapsuleSweepVsDynamicSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticCapsules = "32*32 capsule-sweeps against 32*32 static capsules.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsStaticCapsules, gDesc_SceneCapsuleSweepVsStaticCapsules, PINT_SHAPE_CAPSULE, false)
END_TEST(SceneCapsuleSweepVsStaticCapsules)

static const char* gDesc_SceneCapsuleSweepVsDynamicCapsules = "32*32 capsule-sweeps against 32*32 dynamic capsules.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsDynamicCapsules, gDesc_SceneCapsuleSweepVsDynamicCapsules, PINT_SHAPE_CAPSULE, true)
END_TEST(SceneCapsuleSweepVsDynamicCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticBoxes = "32*32 capsule-sweeps against 32*32 static boxes.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsStaticBoxes, gDesc_SceneCapsuleSweepVsStaticBoxes, PINT_SHAPE_BOX, false)
END_TEST(SceneCapsuleSweepVsStaticBoxes)

static const char* gDesc_SceneCapsuleSweepVsDynamicBoxes = "32*32 capsule-sweeps against 32*32 dynamic boxes.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsDynamicBoxes, gDesc_SceneCapsuleSweepVsDynamicBoxes, PINT_SHAPE_BOX, true)
END_TEST(SceneCapsuleSweepVsDynamicBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticConvexes = "32*32 capsule-sweeps against 32*32 static convexes.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsStaticConvexes, gDesc_SceneCapsuleSweepVsStaticConvexes, PINT_SHAPE_CONVEX, false)
END_TEST(SceneCapsuleSweepVsStaticConvexes)

static const char* gDesc_SceneCapsuleSweepVsDynamicConvexes = "32*32 capsule-sweeps against 32*32 dynamic convexes.";
START_SQ_CAPSULE_SWEEP_VS_SHAPES(SceneCapsuleSweepVsDynamicConvexes, gDesc_SceneCapsuleSweepVsDynamicConvexes, PINT_SHAPE_CONVEX, true)
END_TEST(SceneCapsuleSweepVsDynamicConvexes)

///////////////////////////////////////////////////////////////////////////////

static PintShapeRenderer* CommonSetup_SceneConvexSweep(MyConvex& convex, TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, const Point& offset, float max_dist, udword convex_id)
{
	convex.LoadFile(convex_id);

	PintShapeRenderer* renderer = CreateConvexRenderer(convex.mNbVerts, convex.mVerts);

	Matrix3x3 Rot;
	Rot.Identity();

	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(offset.x + CoeffX * scale_x, offset.y + altitude, offset.z + CoeffY * scale_y);

			test.RegisterConvexSweep(0, renderer, PR(Origin, Rot), dir, max_dist);
		}
	}

	test.mCreateDefaultEnvironment = false;
	return renderer;
}

static bool Setup_SceneConvexSweep(PintShapeRenderer* renderer, MyConvex& convex, TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportConvexSweeps)
		return false;

	PINT_CONVEX_DATA_CREATE Desc;
	Desc.mNbVerts	= convex.mNbVerts;
	Desc.mVerts		= convex.mVerts;
	Desc.mRenderer	= renderer;
	const udword h = pint.CreateConvexObject(Desc);
	ASSERT(h==0);

	return true;
}

	class SceneConvexSweepVsShapes : public SceneSweepVsShapes
	{
		public:

				MyConvex			mConvex;
				PintShapeRenderer*	mRenderer;

		virtual void				CommonRelease()
		{
			mConvex.Release();
			SceneSweepVsShapes::CommonRelease();
		}

		virtual bool				CommonSetup()
		{
			SceneSweepVsShapes::CommonSetup();

			const float Altitude = 30.0f;
			const Point Dir(0.0f, -1.0f, 0.0f);
			const Point Offset(0.0f, 0.0f, 0.0f);
			mRenderer = CommonSetup_SceneConvexSweep(mConvex, *this, 32, 32, Altitude, 50.0f, 50.0f, Dir, Offset, gSQMaxDist, mTypeData);
			return mRenderer!=null;
		}

		virtual bool				Setup(Pint& pint, const PintCaps& caps)
		{
			if(!Setup_SceneConvexSweep(mRenderer, mConvex, *this, pint, caps))
				return false;

			return GenerateArrayOfConvexes(pint, caps, true, 50.0f, 32, 32, mTypeData);
		}

		virtual void				CommonUpdate()
		{
			mCurrentTime += 1.0f/60.0f;
			UpdateConvexSweeps(*this, mCurrentTime);
		}

		virtual udword				Update(Pint& pint)
		{
			return DoBatchConvexSweeps(*this, pint);
		}
	};

#define START_SQ_CONVEX_SWEEP_VS_SHAPES(name, desc, type, convex_id)														\
	class name : public SceneConvexSweepVsShapes																			\
	{																														\
		public:																												\
								name()						{ mType = type;	mDynamic = false; mTypeData = convex_id;	}	\
		virtual					~name()						{															}	\
		virtual	const char*		GetName()			const	{ return #name;												}	\
		virtual	const char*		GetDescription()	const	{ return desc;												}

static const char* gDesc_SceneConvexSweepVsStaticConvexes = "32*32 convex-sweeps against 32*32 static convexes.";
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes1, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 1)
END_TEST(SceneConvexSweepVsStaticConvexes1)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes2, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 2)
END_TEST(SceneConvexSweepVsStaticConvexes2)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes3, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 3)
END_TEST(SceneConvexSweepVsStaticConvexes3)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes4, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 4)
END_TEST(SceneConvexSweepVsStaticConvexes4)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes5, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 5)
END_TEST(SceneConvexSweepVsStaticConvexes5)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes6, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 6)
END_TEST(SceneConvexSweepVsStaticConvexes6)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes7, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 7)
END_TEST(SceneConvexSweepVsStaticConvexes7)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes8, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 8)
END_TEST(SceneConvexSweepVsStaticConvexes8)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes9, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 9)
END_TEST(SceneConvexSweepVsStaticConvexes9)
START_SQ_CONVEX_SWEEP_VS_SHAPES(SceneConvexSweepVsStaticConvexes10, gDesc_SceneConvexSweepVsStaticConvexes, PINT_SHAPE_CONVEX, 10)
END_TEST(SceneConvexSweepVsStaticConvexes10)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SweepAccuracy = "Tests the accuracy of linear sweep tests. Ideally all engines should return the same impact distance.";

START_SQ_TEST(SweepAccuracy, CATEGORY_SWEEP, gDesc_SweepAccuracy)

	virtual void SweepAccuracy::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(8.55f, -0.99f, 0.74f), Point(-0.93f, 0.37f, -0.07f));
	}

	virtual bool SweepAccuracy::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

			const float Radius = 0.5f;
			const float HalfHeight_ = 1.0f;
	/*
			PINT_CAPSULE_CREATE ShapeDesc;
			ShapeDesc.mRadius		= Radius;
			ShapeDesc.mHalfHeight	= HalfHeight_;
			ShapeDesc.mRenderer		= CreateCapsuleRenderer(Radius, HalfHeight_*2.0f);
	*/

	/*
			PINT_SPHERE_CREATE ShapeDesc;
			ShapeDesc.mRadius		= Radius;
			ShapeDesc.mRenderer		= CreateSphereRenderer(Radius);
	*/
			PINT_BOX_CREATE ShapeDesc;
			ShapeDesc.mExtents		= Point(Radius, Radius, Radius);
			ShapeDesc.mRenderer		= CreateBoxRenderer(ShapeDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &ShapeDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition.Zero();
			ObjectDesc.mRotation = Quat(1.1f, 2.2f, 3.3f, 4.4f);
			ObjectDesc.mRotation.Normalize();
			CreatePintObject(pint, ObjectDesc);

		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 1.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const Point P0(HalfHeight, 10.0f, HalfHeight);
		const Point P1(-HalfHeight, 10.0f, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void SweepAccuracy::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		const float Offset = sinf(mCurrentTime)*50.0f;

		const float Altitude = 100.0f;
		const float HalfHeight = 1.8f;
		udword Nb = GetNbRegisteredCapsuleSweeps();
		PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
		while(Nb--)
		{
			const Point P0(1.3f + HalfHeight, Altitude+Offset, HalfHeight);
			const Point P1(1.3f + -HalfHeight, Altitude+Offset, -HalfHeight);

			Data->mCapsule.mP0 = P0;
			Data->mCapsule.mP1 = P1;
			Data++;
		}
	}

	virtual udword SweepAccuracy::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SweepAccuracy)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SweepAccuracy2 = "Tests the accuracy of linear sweep tests. Ideally all engines should return the same impact distance.";

START_SQ_TEST(SweepAccuracy2, CATEGORY_SWEEP, gDesc_SweepAccuracy2)

	virtual void SweepAccuracy2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.42f, 0.02f, 5.22f), Point(-0.82f, 0.15f, -0.55f));
	}

	virtual bool SweepAccuracy2::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 5000.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SweepAccuracy2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

	/*		const float Radius = 0.5f;
			const float HalfHeight_ = 1.0f;
			PINT_BOX_CREATE ShapeDesc;
			ShapeDesc.mExtents		= Point(Radius, Radius, Radius);
			ShapeDesc.mRenderer		= CreateBoxRenderer(ShapeDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &ShapeDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition.Zero();
			ObjectDesc.mRotation = Quat(1.1f, 2.2f, 3.3f, 4.4f);
			ObjectDesc.mRotation.Normalize();
			CreatePintObject(pint, ObjectDesc);
	*/

		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 1.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const Point P0(HalfHeight, 10.0f, HalfHeight);
		const Point P1(-HalfHeight, 10.0f, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
		return true;
	}

	virtual void SweepAccuracy2::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		const float Offset = sinf(mCurrentTime)*50.0f;

		const float Altitude = 100.0f;
		const float HalfHeight = 1.8f;
		udword Nb = GetNbRegisteredCapsuleSweeps();
		PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
		while(Nb--)
		{
			const Point P0(1.3f + HalfHeight, Altitude+Offset, HalfHeight);
			const Point P1(1.3f + -HalfHeight, Altitude+Offset, -HalfHeight);

			Data->mCapsule.mP0 = P0;
			Data->mCapsule.mP1 = P1;
			Data++;
		}
	}

	virtual udword SweepAccuracy2::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SweepAccuracy2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneLongBoxSweepVsSeaOfStatics = "Long diagonal box sweep against a sea of static boxes.";

START_SQ_TEST(SceneLongBoxSweepVsSeaOfStatics, CATEGORY_SWEEP, gDesc_SceneLongBoxSweepVsSeaOfStatics)

	virtual void SceneLongBoxSweepVsSeaOfStatics::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(40.53f, 3.74f, 42.89f), Point(0.15f, -0.89f, -0.43f));
		desc.mCamera[1] = CameraPose(Point(54.50f, 10.80f, 72.83f), Point(-0.59f, -0.11f, -0.80f));
	}

	virtual bool SceneLongBoxSweepVsSeaOfStatics::CommonSetup()
	{
		TestBase::CommonSetup();

		Point Dir(1.f, 0.0f, 1.0f);
		Dir.Normalize();

		const Point Origin(50.0f, 0.0f, 50.0f);

		OBB Box;
		Box.mCenter		= Origin;
		Box.mExtents	= Point(1.0f, 1.0f, 1.0f);
		Box.mRot.Identity();

		RegisterBoxSweep(Box, -Dir, 2000.0f);

		Box.mCenter		= Origin + Point(0.0f, 10.0f, 0.0f);
		RegisterBoxSweep(Box, -Dir, 2000.0f);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneLongBoxSweepVsSeaOfStatics::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 40.0f, 128, 128, 0.0f);
	}

	virtual void SceneLongBoxSweepVsSeaOfStatics::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword SceneLongBoxSweepVsSeaOfStatics::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneLongBoxSweepVsSeaOfStatics)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneLongSphereSweepVsSeaOfStatics = "Long diagonal sphere sweep against a sea of static boxes.";

START_SQ_TEST(SceneLongSphereSweepVsSeaOfStatics, CATEGORY_SWEEP, gDesc_SceneLongSphereSweepVsSeaOfStatics)

	virtual void SceneLongSphereSweepVsSeaOfStatics::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(40.53f, 3.74f, 42.89f), Point(0.15f, -0.89f, -0.43f));
		desc.mCamera[1] = CameraPose(Point(54.50f, 10.80f, 72.83f), Point(-0.59f, -0.11f, -0.80f));
	}

	virtual bool SceneLongSphereSweepVsSeaOfStatics::CommonSetup()
	{
		TestBase::CommonSetup();

		Point Dir(1.f, 0.0f, 1.0f);
		Dir.Normalize();

		const Point Origin(50.0f, 0.0f, 50.0f);

		RegisterSphereSweep(Sphere(Origin, 1.0f), -Dir, 2000.0f);
		RegisterSphereSweep(Sphere(Origin + Point(0.0f, 10.0f, 0.0f), 1.0f), -Dir, 2000.0f);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneLongSphereSweepVsSeaOfStatics::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 40.0f, 128, 128, 0.0f);
	}

	virtual udword SceneLongSphereSweepVsSeaOfStatics::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneLongSphereSweepVsSeaOfStatics)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneLongCapsuleSweepVsSeaOfStatics = "Long diagonal capsule sweep against a sea of static boxes.";

START_SQ_TEST(SceneLongCapsuleSweepVsSeaOfStatics, CATEGORY_SWEEP, gDesc_SceneLongCapsuleSweepVsSeaOfStatics)

	virtual void SceneLongCapsuleSweepVsSeaOfStatics::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(40.53f, 3.74f, 42.89f), Point(0.15f, -0.89f, -0.43f));
		desc.mCamera[1] = CameraPose(Point(54.50f, 10.80f, 72.83f), Point(-0.59f, -0.11f, -0.80f));
	}

	virtual bool SceneLongCapsuleSweepVsSeaOfStatics::CommonSetup()
	{
		TestBase::CommonSetup();

		Point Dir(1.f, 0.0f, 1.0f);
		Dir.Normalize();

		const Point Origin(50.0f, 0.0f, 50.0f);

		const Point Ext(2.0f, 0.0f, 0.0f);

		RegisterCapsuleSweep(LSS(Segment(Origin-Ext, Origin+Ext), 1.0f), -Dir, 2000.0f);

		const Point C = Origin + Point(0.0f, 10.0f, 0.0f);
		RegisterCapsuleSweep(LSS(Segment(C-Ext, C+Ext), 1.0f), -Dir, 2000.0f);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneLongCapsuleSweepVsSeaOfStatics::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 40.0f, 128, 128, 0.0f);
	}

	virtual void SceneLongCapsuleSweepVsSeaOfStatics::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword SceneLongCapsuleSweepVsSeaOfStatics::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneLongCapsuleSweepVsSeaOfStatics)

///////////////////////////////////////////////////////////////////////////////

	// TODO: there's a design issue for convex sweeps. The sweeps now need data from each Pint (the convex mesh), which
	// was not the case before. Creating things in CommonSetup() doesn't work since there's no Pint pointer. Creating
	// things in Setup() doesn't work since we UnregisterAllConvexSweeps() and the previous Pint-provided objects are
	// lost. We end up calling all Pint plugins with the convex data from the last Pint. Which happens to work just
	// fine between PhysX 3.3 and 3.4, which is mental (we send a PxConvexMesh created in the 3.3 plugin to the 3.4 SDK
	// and things just work. Madness!)

	class SceneSweepVsSingleTriangle : public TestBase
	{
		public:
								SceneSweepVsSingleTriangle()	{ mType = PINT_SHAPE_UNDEFINED;	mDoubleSided = false;	}
		virtual					~SceneSweepVsSingleTriangle()	{														}
		virtual	TestCategory	GetCategory()			const	{ return CATEGORY_SWEEP;								}
		virtual	bool			ProfileUpdate()					{ return true;											}

		virtual void SceneSweepVsSingleTriangle::GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
		}

		virtual bool SceneSweepVsSingleTriangle::CommonSetup()
		{
			CreateSingleTriangleMesh(*this, 4.0f, &mTriangle, mDoubleSided);
			mCreateDefaultEnvironment = false;
			return TestBase::CommonSetup();
		}

		virtual bool SceneSweepVsSingleTriangle::Setup(Pint& pint, const PintCaps& caps)
		{
			if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
				return false;

			if(mType==PINT_SHAPE_SPHERE && !caps.mSupportSphereSweeps)
				return false;
			if(mType==PINT_SHAPE_CAPSULE && !caps.mSupportCapsuleSweeps)
				return false;
			if(mType==PINT_SHAPE_BOX && !caps.mSupportBoxSweeps)
				return false;
			if(mType==PINT_SHAPE_CONVEX && !caps.mSupportConvexSweeps)
				return false;

			const Point Dir(0.0f, -1.0f, 0.0f);
			if(mType==PINT_SHAPE_SPHERE)
			{
				const float Radius = 0.4f;

				UnregisterAllSphereSweeps();
				const Point P(0.0f, 10.0f, 0.0f);
				RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);
			}
			else if(mType==PINT_SHAPE_CAPSULE)
			{
		//		const float CapsuleRadius = 1.4f;
				const float CapsuleRadius = 0.4f;
				const float HalfHeight = 1.8f;

				UnregisterAllCapsuleSweeps();
				const Point P0(HalfHeight, 10.0f, HalfHeight);
				const Point P1(-HalfHeight, 10.0f, -HalfHeight);
				RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
			}
			else if(mType==PINT_SHAPE_BOX)
			{
				UnregisterAllBoxSweeps();
				const Point P(0.0f, 10.0f, 0.0f);
				Matrix3x3 Rot;
				Rot.Identity();
				RegisterBoxSweep(OBB(P, Point(0.4f, 2.0f, 0.4f), Rot), Dir, gSQMaxDist);
			}
			else if(mType==PINT_SHAPE_CONVEX)
			{
				UnregisterAllConvexSweeps();

				MyConvex C;
			//	udword i=2;	// Small convexes
				udword i=4;	// 'Big' convexes
			//	udword i=7;
			//	udword i=13;
				C.LoadFile(i);

				PINT_CONVEX_DATA_CREATE Desc;
				Desc.mNbVerts	= C.mNbVerts;
				Desc.mVerts		= C.mVerts;
				PintShapeRenderer* renderer = CreateConvexRenderer(C.mNbVerts, C.mVerts);
				Desc.mRenderer	= renderer;
				const udword h = pint.CreateConvexObject(Desc);

				const Point P(0.0f, 10.0f, 0.0f);
				Matrix3x3 Rot;
				Rot.Identity();

				RegisterConvexSweep(h, renderer, PR(P, Rot), Dir, gSQMaxDist);
			}
			return true;
		}

		virtual void SceneSweepVsSingleTriangle::CommonUpdate()
		{
			if(mType==PINT_SHAPE_SPHERE)
			{
				mCurrentTime += 0.5f/60.0f;
		//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		//		UpdateSphereSweeps(*this, mCurrentTime);
			}
			else if(mType==PINT_SHAPE_CAPSULE)
			{
				mCurrentTime += 0.5f/60.0f;
//				mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
				UpdateCapsuleSweeps(*this, mCurrentTime);
			}
			else if(mType==PINT_SHAPE_BOX)
			{
				mCurrentTime += 0.5f/60.0f;
		//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		//mCurrentTime = 357.0f * 0.5f/60.0f;
				UpdateBoxSweeps(*this, mCurrentTime);
			}
			else if(mType==PINT_SHAPE_CONVEX)
			{
				mCurrentTime += 0.5f/60.0f;
		//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		//mCurrentTime = 357.0f * 0.5f/60.0f;
				UpdateConvexSweeps(*this, mCurrentTime);
			}
		}

		virtual udword SceneSweepVsSingleTriangle::Update(Pint& pint)
		{
			udword Ret = 0;
			if(mType==PINT_SHAPE_SPHERE)
				Ret = DoBatchSphereSweeps(*this, pint);
			else if(mType==PINT_SHAPE_CAPSULE)
				Ret = DoBatchCapsuleSweeps(*this, pint);
			else if(mType==PINT_SHAPE_BOX)
				Ret = DoBatchBoxSweeps(*this, pint);
			else if(mType==PINT_SHAPE_CONVEX)
				Ret = DoBatchConvexSweeps(*this, pint);
			return Ret;
		}

		virtual	void SceneSweepVsSingleTriangle::CommonRender(PintRender& renderer)
		{
			const Point Color(0.0f, 0.0f, 0.0f);
			renderer.DrawLine(mTriangle.mVerts[0], mTriangle.mVerts[1], Color);
			renderer.DrawLine(mTriangle.mVerts[1], mTriangle.mVerts[2], Color);
			renderer.DrawLine(mTriangle.mVerts[2], mTriangle.mVerts[0], Color);
			renderer.DrawTriangle(mTriangle.mVerts[0], mTriangle.mVerts[2], mTriangle.mVerts[1], Color);
			if(mDoubleSided)
				renderer.DrawTriangle(mTriangle.mVerts[0], mTriangle.mVerts[1], mTriangle.mVerts[2], Color);
		}

			PintShape	mType;
			Triangle	mTriangle;
			bool		mDoubleSided;
	};

#define START_SINGLE_TRIANGLE_SWEEP_TEST(name, desc, type, double_sided)										\
	class name : public SceneSweepVsSingleTriangle																\
	{																											\
		public:																									\
								name()						{ mType = type;	mDoubleSided = double_sided;	}	\
		virtual					~name()						{												}	\
		virtual	const char*		GetName()			const	{ return #name;									}	\
		virtual	const char*		GetDescription()	const	{ return desc;									}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsSingleTriangle = "1 sphere-sweep against a single triangle.";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneSphereSweepVsSingleTriangle, gDesc_SceneSphereSweepVsSingleTriangle, PINT_SHAPE_SPHERE, false)
END_TEST(SceneSphereSweepVsSingleTriangle)

static const char* gDesc_SceneSphereSweepVsSingleTriangleDoubleSided = "1 sphere-sweep against a single triangle (double-sided).";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneSphereSweepVsSingleTriangleDoubleSided, gDesc_SceneSphereSweepVsSingleTriangleDoubleSided, PINT_SHAPE_SPHERE, true)
END_TEST(SceneSphereSweepVsSingleTriangleDoubleSided)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsSingleTriangle = "1 capsule-sweep against a single triangle.";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneCapsuleSweepVsSingleTriangle, gDesc_SceneCapsuleSweepVsSingleTriangle, PINT_SHAPE_CAPSULE, false)

	virtual	void SceneCapsuleSweepVsSingleTriangle::CommonRender(PintRender& renderer)
	{
		SceneSweepVsSingleTriangle::CommonRender(renderer);

		if(0)	// Debug render code drawing extruded triangle
		{
			const Point v0(-2.666666f, 0.0f, -1.333333f);
			const Point v1(1.333333f, 0.0f, 2.666666f);
			const Point v2(1.333333f, 0.0f, -1.333333f);

			const Point ExtrusionDir(2.4054673f, 0.83290195f, 0.0f);

			const Point p0 = v0 - ExtrusionDir;
			const Point p1 = v1 - ExtrusionDir;
			const Point p2 = v2 - ExtrusionDir;

			const Point p0b = v0 + ExtrusionDir;
			const Point p1b = v1 + ExtrusionDir;
			const Point p2b = v2 + ExtrusionDir;

			renderer.DrawTriangle(v0, v1, v2, Point(0.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p1, p2, Point(1.0f, 0.0f, 0.0f));
			renderer.DrawTriangle(p0b, p1b, p2b, Point(0.0f, 0.0f, 1.0f));

			renderer.DrawTriangle(p1, p1b, p2b, Point(0.0f, 1.0f, 1.0f));
			renderer.DrawTriangle(p1, p2b, p2, Point(0.0f, 1.0f, 1.0f));

			renderer.DrawTriangle(p0, p2, p2b, Point(1.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p2b, p0b, Point(1.0f, 1.0f, 0.0f));

			renderer.DrawTriangle(p0b, p1b, p1, Point(1.0f, 0.0f, 1.0f));
			renderer.DrawTriangle(p0b, p1, p0, Point(1.0f, 0.0f, 1.0f));
		}
	}

END_TEST(SceneCapsuleSweepVsSingleTriangle)

static const char* gDesc_SceneCapsuleSweepVsSingleTriangleDoubleSided = "1 capsule-sweep against a single triangle (double sided).";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneCapsuleSweepVsSingleTriangleDoubleSided, gDesc_SceneCapsuleSweepVsSingleTriangleDoubleSided, PINT_SHAPE_CAPSULE, true)
END_TEST(SceneCapsuleSweepVsSingleTriangleDoubleSided)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsSingleTriangle = "1 box-sweep against a single triangle.";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneBoxSweepVsSingleTriangle, gDesc_SceneBoxSweepVsSingleTriangle, PINT_SHAPE_BOX, false)
END_TEST(SceneBoxSweepVsSingleTriangle)

static const char* gDesc_SceneBoxSweepVsSingleTriangleDoubleSided = "1 box-sweep against a single triangle (double sided).";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneBoxSweepVsSingleTriangleDoubleSided, gDesc_SceneBoxSweepVsSingleTriangleDoubleSided, PINT_SHAPE_BOX, true)
END_TEST(SceneBoxSweepVsSingleTriangleDoubleSided)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneConvexSweepVsSingleTriangle = "1 convex-sweep against a single triangle.";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneConvexSweepVsSingleTriangle, gDesc_SceneConvexSweepVsSingleTriangle, PINT_SHAPE_CONVEX, false)
END_TEST(SceneConvexSweepVsSingleTriangle)

static const char* gDesc_SceneConvexSweepVsSingleTriangleDoubleSided = "1 convex-sweep against a single triangle (double sided).";
START_SINGLE_TRIANGLE_SWEEP_TEST(SceneConvexSweepVsSingleTriangleDoubleSided, gDesc_SceneConvexSweepVsSingleTriangleDoubleSided, PINT_SHAPE_CONVEX, true)
END_TEST(SceneConvexSweepVsSingleTriangleDoubleSided)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsSingleTriangle_InitialOverlap = "1 sphere-sweep against a single triangle. (Initial overlap)";

START_SQ_TEST(SceneSphereSweepVsSingleTriangle_InitialOverlap, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsSingleTriangle_InitialOverlap)

	Triangle mTriangle;

	virtual void SceneSphereSweepVsSingleTriangle_InitialOverlap::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneSphereSweepVsSingleTriangle_InitialOverlap::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f, &mTriangle);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneSphereSweepVsSingleTriangle_InitialOverlap::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Triangle& T = mTriangle;

		Point TriCenter;
		T.Center(TriCenter);

		const Point D0 = (T.mVerts[0] - TriCenter).Normalize();
		const Point D1 = (T.mVerts[1] - TriCenter).Normalize();
		const Point D2 = (T.mVerts[2] - TriCenter).Normalize();

		const Point E01 = (T.mVerts[0] + T.mVerts[1])*0.5f;
		const Point E12 = (T.mVerts[1] + T.mVerts[2])*0.5f;
		const Point E20 = (T.mVerts[2] + T.mVerts[0])*0.5f;

		const Point D01 = (E01 - TriCenter).Normalize();
		const Point D12 = (E12 - TriCenter).Normalize();
		const Point D20 = (E20 - TriCenter).Normalize();

//		const Point Dir(0.0f, -1.0f, 0.0f);
		Point Dir(-1.0f, -1.0f, -1.0f);
		Dir.Normalize();
		const float Radius = 0.4f;

		UnregisterAllSphereSweeps();
		const Point P(0.0f, 0.1f, 0.0f);
//		const Point P(0.0f, -0.1f, 0.0f);
//		const Point P(0.0f, -Radius+0.01f, 0.0f);
//		const Point P(0.0f, -Radius-0.1f, 0.0f);
//		const Point P = (T.mVerts[0] + D0*0.2f) + Point(0.0f, 0.1f, 0.0f);
//		const Point P = (T.mVerts[1] + D1*0.2f) + Point(0.0f, 0.1f, 0.0f);
//		const Point P = (E01 + D01*0.2f) + Point(0.0f, 0.1f, 0.0f);
		RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);

		BasicRandom Rnd(42);
		for(udword i=0;i<4095;i++)
		{
			UnitRandomPt(Dir, Rnd);
			Dir.y = -fabsf(Dir.y);
			RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);
		}
		return true;
	}

	virtual void SceneSphereSweepVsSingleTriangle_InitialOverlap::CommonUpdate()
	{
		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
//		UpdateSphereSweeps(*this, mCurrentTime);
	}

	virtual udword SceneSphereSweepVsSingleTriangle_InitialOverlap::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsSingleTriangle_InitialOverlap)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsSingleTriangle_InitialOverlap = "1 capsule-sweep against a single triangle. (Initial overlap)";

START_SQ_TEST(SceneCapsuleSweepVsSingleTriangle_InitialOverlap, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsSingleTriangle_InitialOverlap)

	virtual void SceneCapsuleSweepVsSingleTriangle_InitialOverlap::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneCapsuleSweepVsSingleTriangle_InitialOverlap::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneCapsuleSweepVsSingleTriangle_InitialOverlap::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Point Dir(0.0f, -1.0f, 0.0f);
//		const float CapsuleRadius = 1.4f;
		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const float CapsuleAltitude = 0.1f;
//		const float CapsuleAltitude = 1.5f;
		const Point P0(HalfHeight, CapsuleAltitude, HalfHeight);
		const Point P1(-HalfHeight, CapsuleAltitude, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
		return true;
	}

	virtual void SceneCapsuleSweepVsSingleTriangle_InitialOverlap::CommonUpdate()
	{
/*		static bool init=false;
		if(!init)
			init = true;
		else
			return;*/

		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
//		mCurrentTime = 40.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword SceneCapsuleSweepVsSingleTriangle_InitialOverlap::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

	virtual	void SceneCapsuleSweepVsSingleTriangle_InitialOverlap::CommonRender(PintRender& renderer)
	{
		if(0)	// Debug render code drawing extruded triangle
		{
			const Point v0(-2.666666f, 0.0f, -1.333333f);
			const Point v1(1.333333f, 0.0f, 2.666666f);
			const Point v2(1.333333f, 0.0f, -1.333333f);

//			const Point ExtrusionDir(2.4054673f, 0.83290195f, 0.0f);

			PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
			const Point ExtrusionDir = (Data->mCapsule.mP0 - Data->mCapsule.mP1)*0.5f;
			const Point Center = (Data->mCapsule.mP0 + Data->mCapsule.mP1)*0.5f;
			renderer.DrawLine(Center, Center+Point(10.0f, 0.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 10.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 0.0f, 10.0f), Point(0.5f, 0.5f, 0.5f));

			const Point p0 = v0 - ExtrusionDir;
			const Point p1 = v1 - ExtrusionDir;
			const Point p2 = v2 - ExtrusionDir;

			const Point p0b = v0 + ExtrusionDir;
			const Point p1b = v1 + ExtrusionDir;
			const Point p2b = v2 + ExtrusionDir;

			renderer.DrawTriangle(v0, v1, v2, Point(0.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p1, p2, Point(1.0f, 0.0f, 0.0f));
			renderer.DrawTriangle(p0b, p1b, p2b, Point(0.0f, 0.0f, 1.0f));

			renderer.DrawTriangle(p1, p1b, p2b, Point(0.0f, 1.0f, 1.0f));
			renderer.DrawTriangle(p1, p2b, p2, Point(0.0f, 1.0f, 1.0f));

			renderer.DrawTriangle(p0, p2, p2b, Point(1.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p2b, p0b, Point(1.0f, 1.0f, 0.0f));

			renderer.DrawTriangle(p0b, p1b, p1, Point(1.0f, 0.0f, 1.0f));
			renderer.DrawTriangle(p0b, p1, p0, Point(1.0f, 0.0f, 1.0f));
		}
	}

END_TEST(SceneCapsuleSweepVsSingleTriangle_InitialOverlap)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsSingleTriangle_InitialOverlap = "1 box-sweep against a single triangle. (Initial overlap)";

START_SQ_TEST(SceneBoxSweepVsSingleTriangle_InitialOverlap, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsSingleTriangle_InitialOverlap)

	virtual void SceneBoxSweepVsSingleTriangle_InitialOverlap::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneBoxSweepVsSingleTriangle_InitialOverlap::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneBoxSweepVsSingleTriangle_InitialOverlap::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Point Dir(0.0f, -1.0f, 0.0f);

		UnregisterAllBoxSweeps();
		const Point P(0.0f, 0.1f, 0.0f);
		Matrix3x3 Rot;
		Rot.Identity();
		RegisterBoxSweep(OBB(P, Point(0.4f, 2.0f, 0.4f), Rot), Dir, gSQMaxDist);
		return true;
	}

	virtual void SceneBoxSweepVsSingleTriangle_InitialOverlap::CommonUpdate()
	{
		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword SceneBoxSweepVsSingleTriangle_InitialOverlap::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsSingleTriangle_InitialOverlap)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsSingleTriangle_ParallelSweep = "1 sphere-sweep against a single triangle. (Parallel sweep)";

START_SQ_TEST(SceneSphereSweepVsSingleTriangle_ParallelSweep, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsSingleTriangle_ParallelSweep)

	Triangle mTriangle;

	virtual void SceneSphereSweepVsSingleTriangle_ParallelSweep::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneSphereSweepVsSingleTriangle_ParallelSweep::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f, &mTriangle);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneSphereSweepVsSingleTriangle_ParallelSweep::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Triangle& T = mTriangle;

		Point TriCenter;
		T.Center(TriCenter);

		const Point D0 = (T.mVerts[0] - TriCenter).Normalize();
		const Point D1 = (T.mVerts[1] - TriCenter).Normalize();
		const Point D2 = (T.mVerts[2] - TriCenter).Normalize();

		const Point E01 = (T.mVerts[0] + T.mVerts[1])*0.5f;
		const Point E12 = (T.mVerts[1] + T.mVerts[2])*0.5f;
		const Point E20 = (T.mVerts[2] + T.mVerts[0])*0.5f;

		const Point D01 = (E01 - TriCenter).Normalize();
		const Point D12 = (E12 - TriCenter).Normalize();
		const Point D20 = (E20 - TriCenter).Normalize();

		Point Dir(1.0f, 0.0f, 0.0f);
		const float Radius = 0.4f;

		UnregisterAllSphereSweeps();
		const Point P(-10.0f, 0.1f, 0.0f);
		RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);

		return true;
	}

	virtual void SceneSphereSweepVsSingleTriangle_ParallelSweep::CommonUpdate()
	{
		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
//		UpdateSphereSweeps(*this, mCurrentTime);
	}

	virtual udword SceneSphereSweepVsSingleTriangle_ParallelSweep::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsSingleTriangle_ParallelSweep)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsSingleTriangle_ParallelSweep = "1 capsule-sweep against a single triangle. (Parallel sweep)";

START_SQ_TEST(SceneCapsuleSweepVsSingleTriangle_ParallelSweep, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsSingleTriangle_ParallelSweep)

	virtual void SceneCapsuleSweepVsSingleTriangle_ParallelSweep::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneCapsuleSweepVsSingleTriangle_ParallelSweep::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneCapsuleSweepVsSingleTriangle_ParallelSweep::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Point Dir(1.0f, 0.0f, 0.0f);
//		const float CapsuleRadius = 1.4f;
		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const float CapsuleAltitude = 0.1f;
//		const float CapsuleAltitude = 1.5f;
		const Point P0(HalfHeight-10.0f, CapsuleAltitude, HalfHeight);
		const Point P1(-HalfHeight-10.0f, CapsuleAltitude, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
		return true;
	}

	virtual void SceneCapsuleSweepVsSingleTriangle_ParallelSweep::CommonUpdate()
	{
		mCurrentTime += 0.5f/60.0f;
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword SceneCapsuleSweepVsSingleTriangle_ParallelSweep::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

	virtual	void SceneCapsuleSweepVsSingleTriangle_ParallelSweep::CommonRender(PintRender& renderer)
	{
		if(0)	// Debug render code drawing extruded triangle
		{
			const Point v0(-2.666666f, 0.0f, -1.333333f);
			const Point v1(1.333333f, 0.0f, 2.666666f);
			const Point v2(1.333333f, 0.0f, -1.333333f);

//			const Point ExtrusionDir(2.4054673f, 0.83290195f, 0.0f);

			PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
			const Point ExtrusionDir = (Data->mCapsule.mP0 - Data->mCapsule.mP1)*0.5f;
			const Point Center = (Data->mCapsule.mP0 + Data->mCapsule.mP1)*0.5f;
			renderer.DrawLine(Center, Center+Point(10.0f, 0.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 10.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 0.0f, 10.0f), Point(0.5f, 0.5f, 0.5f));

			const Point p0 = v0 - ExtrusionDir;
			const Point p1 = v1 - ExtrusionDir;
			const Point p2 = v2 - ExtrusionDir;

			const Point p0b = v0 + ExtrusionDir;
			const Point p1b = v1 + ExtrusionDir;
			const Point p2b = v2 + ExtrusionDir;

			renderer.DrawTriangle(v0, v1, v2, Point(0.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p1, p2, Point(1.0f, 0.0f, 0.0f));
			renderer.DrawTriangle(p0b, p1b, p2b, Point(0.0f, 0.0f, 1.0f));

			renderer.DrawTriangle(p1, p1b, p2b, Point(0.0f, 1.0f, 1.0f));
			renderer.DrawTriangle(p1, p2b, p2, Point(0.0f, 1.0f, 1.0f));

			renderer.DrawTriangle(p0, p2, p2b, Point(1.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p2b, p0b, Point(1.0f, 1.0f, 0.0f));

			renderer.DrawTriangle(p0b, p1b, p1, Point(1.0f, 0.0f, 1.0f));
			renderer.DrawTriangle(p0b, p1, p0, Point(1.0f, 0.0f, 1.0f));
		}
	}

END_TEST(SceneCapsuleSweepVsSingleTriangle_ParallelSweep)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsSingleTriangle_ParallelSweep = "1 box-sweep against a single triangle. (Parallel sweep)";

START_SQ_TEST(SceneBoxSweepVsSingleTriangle_ParallelSweep, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsSingleTriangle_ParallelSweep)

	virtual void SceneBoxSweepVsSingleTriangle_ParallelSweep::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual bool SceneBoxSweepVsSingleTriangle_ParallelSweep::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 4.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SceneBoxSweepVsSingleTriangle_ParallelSweep::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Point Dir(1.0f, 0.0f, 0.0f);

		UnregisterAllBoxSweeps();
		const Point P(-10.0f, 0.1f, 0.0f);
		Matrix3x3 Rot;
		Rot.Identity();
		RegisterBoxSweep(OBB(P, Point(0.4f, 2.0f, 0.4f), Rot), Dir, gSQMaxDist);
		return true;
	}

	virtual void SceneBoxSweepVsSingleTriangle_ParallelSweep::CommonUpdate()
	{
		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword SceneBoxSweepVsSingleTriangle_ParallelSweep::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsSingleTriangle_ParallelSweep)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticMeshes_Archipelago = "32*32 box-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_Archipelago)

	virtual bool SceneBoxSweepVsStaticMeshes_Archipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneBoxSweepVsStaticMeshes_Archipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const Point BoxExtents(1.2f, 0.5f, 0.5f);
		RegisterArrayOfBoxSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, BoxExtents, Offset, gSQMaxDist);
		return true;
	}

	virtual void SceneBoxSweepVsStaticMeshes_Archipelago::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_Archipelago::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticMeshes_KP = "32*32 box-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_KP)

	virtual bool SceneBoxSweepVsStaticMeshes_KP::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const Point Extents(1.2f, 0.5f, 0.5f);

		UnregisterAllBoxSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			OBB Box;
			Box.mCenter		= Point(50.0f, 50.0f, 50.0f);
			Box.mExtents	= Extents;
			Box.mRot.Identity();

			RegisterBoxSweep(Box, Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool SceneBoxSweepVsStaticMeshes_KP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual void SceneBoxSweepVsStaticMeshes_KP::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_KP::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static bool CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(TestBase& test, const Point& extents, udword nb_sweeps, const Matrix3x3& rot)
{
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	for(udword i=0;i<nb_sweeps;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;
		const OBB Box(Origin, extents, rot);
		test.RegisterBoxSweep(Box, -Dir, 200.0f);
//		if(i>=12 && i<15)
//		if(i==13)
//		test.RegisterBoxSweep(Box, -Dir, 20.0f);	//###MEGADEBUG
	}

	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool Setup_SceneBoxSweepVsStaticMeshes_TessBunny(TestBase& test, Pint& pint, const PintCaps& caps/*, const Point& extents, udword nb_sweeps, const Matrix3x3& rot*/)
{
	if(!caps.mSupportBoxSweeps)
		return false;

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1 = "64 radial box-sweeps against the tesselated bunny. Each box is a (1, 1, 1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1)

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test1::CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 64, Rot);
	}

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test1::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_TessBunny_Test1::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1b = "4096 radial box-sweeps against the tesselated bunny. Each box is a (1, 1, 1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1b, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1b)

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test1b::CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 4096, Rot);
	}

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test1b::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_TessBunny_Test1b::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1b)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test2 = "256 radial box-sweeps against the tesselated bunny. Each box is a (1, 1, 1) cube. Box orientation is non-identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test2, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test2)

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test2::CommonSetup()
	{
		Matrix3x3 RotX;
		RotX.RotX(degToRad(45.0f));

		Matrix3x3 RotY;
		RotY.RotY(degToRad(45.0f));

		RotX *= RotY;

		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 256, RotX);
	}

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test2::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_TessBunny_Test2::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test2)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test3 = "A single radial box-sweep against the tesselated bunny. Box is a (1, 1, 1) cube. Box orientation is non-identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test3, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test3)

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test3::CommonSetup()
	{
		Matrix3x3 RotX;
		RotX.RotX(degToRad(45.0f));

		Matrix3x3 RotY;
		RotY.RotY(degToRad(45.0f));

		RotX *= RotY;

		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 1, RotX);
	}

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test3::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_TessBunny_Test3::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test3)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny2_Test4 = "256 radial box-sweeps against the tesselated bunny. Each box is a (0.1, 0.1, 0.1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test4, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny2_Test4)

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test4::CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(0.1f, 0.1f, 0.1f), 256, Rot);
	}

	virtual bool SceneBoxSweepVsStaticMeshes_TessBunny_Test4::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneBoxSweepVsStaticMeshes_TessBunny_Test4::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test4)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticMeshes_Archipelago = "32*32 sphere-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_Archipelago)

	virtual bool SceneSphereSweepVsStaticMeshes_Archipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneSphereSweepVsStaticMeshes_Archipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const float Radius = 0.75f;
		RegisterArrayOfSphereSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, Radius, Offset, gSQMaxDist);
		return true;
	}

	virtual udword SceneSphereSweepVsStaticMeshes_Archipelago::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticMeshes_KP = "32*32 sphere-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_KP)

	virtual bool SceneSphereSweepVsStaticMeshes_KP::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const float Radius = 0.75f;

		UnregisterAllSphereSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			RegisterSphereSweep(Sphere(Point(50.0f, 50.0f, 50.0f), Radius), Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool SceneSphereSweepVsStaticMeshes_KP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword SceneSphereSweepVsStaticMeshes_KP::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static bool CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(TestBase& test, float radius)
{
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	for(udword i=0;i<64;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;

		test.RegisterSphereSweep(Sphere(Origin, radius), -Dir, 20.0f);
	}

	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool Setup_SceneSphereSweepVsStaticMeshes_TessBunny(TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportSphereSweeps)
		return false;

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneSphereSweepVsStaticMeshes_TessBunny = "64 radial sphere-sweeps against the tesselated bunny. Radius of the spheres is 1.0.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_TessBunny)

	virtual bool SceneSphereSweepVsStaticMeshes_TessBunny::CommonSetup()
	{
		return CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, 1.0f);
	}

	virtual bool SceneSphereSweepVsStaticMeshes_TessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneSphereSweepVsStaticMeshes_TessBunny::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_TessBunny)

static const char* gDesc_SceneSphereSweepVsStaticMeshes_TessBunny2 = "64 radial sphere-sweeps against the tesselated bunny. Radius of the spheres is 0.1.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_TessBunny2, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_TessBunny2)

	virtual bool SceneSphereSweepVsStaticMeshes_TessBunny2::CommonSetup()
	{
		return CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, 0.1f);
	}

	virtual bool SceneSphereSweepVsStaticMeshes_TessBunny2::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword SceneSphereSweepVsStaticMeshes_TessBunny2::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_TessBunny2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_Archipelago = "32*32 capsule-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_Archipelago)

	virtual void SceneCapsuleSweepVsStaticMeshes_Archipelago::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(71.21f, 4.04f, 83.04f), Point(0.55f, -0.46f, 0.70f));
		desc.mCamera[1] = CameraPose(Point(414.28f, 2.60f, 228.18f), Point(0.46f, -0.12f, 0.88f));
	}

	virtual bool SceneCapsuleSweepVsStaticMeshes_Archipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneCapsuleSweepVsStaticMeshes_Archipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;
		RegisterArrayOfCapsuleSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, CapsuleRadius, HalfHeight, Offset, gSQMaxDist);
		return true;
	}

	virtual void SceneCapsuleSweepVsStaticMeshes_Archipelago::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword SceneCapsuleSweepVsStaticMeshes_Archipelago::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_KP = "32*32 capsule-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_KP)

	virtual bool SceneCapsuleSweepVsStaticMeshes_KP::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			LSS Capsule;
			Capsule.mP0 = Point(50.0f-HalfHeight, 50.0f, 50.0f);
			Capsule.mP1 = Point(50.0f+HalfHeight, 50.0f, 50.0f);
			Capsule.mRadius	= CapsuleRadius;

			RegisterCapsuleSweep(Capsule, Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool SceneCapsuleSweepVsStaticMeshes_KP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual void SceneCapsuleSweepVsStaticMeshes_KP::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword SceneCapsuleSweepVsStaticMeshes_KP::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_TessBunny = "64 radial capsule-sweeps against the tesselated bunny.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_TessBunny)

	virtual bool SceneCapsuleSweepVsStaticMeshes_TessBunny::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;

		UnregisterAllCapsuleSweeps();
		BasicRandom Rnd(42);

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);

		for(udword i=0;i<64;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Center + Dir * 20.0f;

			LSS Capsule;
			Capsule.mP0		= Origin + Point(1.0f, 1.0f, 1.0f);
			Capsule.mP1		= Origin - Point(1.0f, 1.0f, 1.0f);
			Capsule.mRadius	= 0.1f;

			RegisterCapsuleSweep(Capsule, -Dir, 20.0f);
		}
/*
//		for(udword i=0;i<1024;i++)
		for(udword i=0;i<16;i++)
		{
//			UnitRandomPt(mRays[i].mDir, Rnd);
//			mRays[i].mOrig = mRays[i].mDir * 5000.0f;
//			mRays[i].mDir = -mRays[i].mDir;

			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Dir * 5000.0f;

			LSS Capsule;
			Capsule.mP0		= Origin + Point(0.5f, 0.0f, 0.0f);
			Capsule.mP1		= Origin - Point(0.5f, 0.0f, 0.0f);
			Capsule.mRadius	= 1.0f;

//			if(i==0)	// Very interesting one, gives very different hit results for each engine
//			if(i==1)	// Clear issue with bv4, impact is way too early
//			if(i==2)	// Gives a hit in bv4 (probably wrong), no hit for other engines... clear bug => now fixed
			RegisterCapsuleSweep(Capsule, -Dir, 5000.0f);
		}*/

		return true;
	}

	virtual bool SceneCapsuleSweepVsStaticMeshes_TessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword SceneCapsuleSweepVsStaticMeshes_TessBunny::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_TessBunny)

///////////////////////////////////////////////////////////////////////////////

static PintShapeRenderer* CommonSetup_SceneConvexSweepVsStaticMeshes_TessBunny(MyConvex& convex, TestBase& test)
{
//	LoadMeshesFromFile_(test, "Venus.bin", null, false, 0);
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	udword i=2;	// Small convexes
//	udword i=4;	// 'Big' convexes
//	udword i=7;
//	udword i=13;
	convex.LoadFile(i);

	PintShapeRenderer* renderer = CreateConvexRenderer(convex.mNbVerts, convex.mVerts);

	const Point P(0.0f, 10.0f, 0.0f);
	Matrix3x3 Rot;
	Rot.Identity();

	for(udword i=0;i<64;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;

		// Beware: we rely on the indices returned by pint.CreateConvexObject() here,
		// even though the functions haven't been called yet.
		test.RegisterConvexSweep(0, renderer, PR(Origin, Rot), -Dir, 20.0f);
	}

	test.mCreateDefaultEnvironment = false;
	return renderer;
}

static bool Setup_SceneConvexSweepVsStaticMeshes_TessBunny(PintShapeRenderer* renderer, MyConvex& convex, TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportMeshes || !caps.mSupportConvexSweeps)
		return false;

	PINT_CONVEX_DATA_CREATE Desc;
	Desc.mNbVerts	= convex.mNbVerts;
	Desc.mVerts		= convex.mVerts;
	Desc.mRenderer	= renderer;
	const udword h = pint.CreateConvexObject(Desc);
	ASSERT(h==0);

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneConvexSweepVsStaticMeshes_TessBunny = "64 radial convex-sweeps against the tesselated bunny.";

START_SQ_TEST(SceneConvexSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneConvexSweepVsStaticMeshes_TessBunny)

	MyConvex			mConvex;
	PintShapeRenderer*	mRenderer;

	virtual void SceneConvexSweepVsStaticMeshes_TessBunny::CommonRelease()
	{
		mConvex.Release();
		TestBase::CommonRelease();
	}

	virtual bool SceneConvexSweepVsStaticMeshes_TessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		mRenderer = CommonSetup_SceneConvexSweepVsStaticMeshes_TessBunny(mConvex, *this);
		return mRenderer!=null;
	}

	virtual bool SceneConvexSweepVsStaticMeshes_TessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneConvexSweepVsStaticMeshes_TessBunny(mRenderer, mConvex, *this, pint, caps);
	}

	virtual void SceneConvexSweepVsStaticMeshes_TessBunny::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		UpdateConvexSweeps(*this, mCurrentTime);
	}

	virtual udword SceneConvexSweepVsStaticMeshes_TessBunny::Update(Pint& pint)
	{
		return DoBatchConvexSweeps(*this, pint);
	}

END_TEST(SceneConvexSweepVsStaticMeshes_TessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxSweep_TestZone = "TestZone. Box sweep.";

START_SQ_TEST(BoxSweep_TestZone, CATEGORY_SWEEP, gDesc_BoxSweep_TestZone)

	virtual bool BoxSweep_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllBoxSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
/*			for(udword i=0;i<1024;i++)
			{
				const float x = Rnd.randomFloat()*100.0f;
				const float z = Rnd.randomFloat()*100.0f;

				Point D(x, -100.0f, z);
				D.Normalize();

				Quat Q;
				//UnitRandomQuat(Q, Rnd);
				Q.Identity();
				RegisterBoxSweep(OBB(Point(x, 50.0f, z), Point(1.0f, 1.0f, 1.0f), Q), D, 1000.0f);
//				RegisterBoxSweep(OBB(Point(x, 50.0f, z), Point(1.0f, 1.0f, 1.0f), Q), Point(0.0f, -1.0f, 0.0f), 1000.0f);
			}*/

			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
//desc.mCamera[0] = CameraPose(Point(8.36f, -4.22f, -28.18f), Point(0.14f, 0.21f, -0.97f));
			const udword Nb = 2048;
			for(udword i=0;i<Nb;i++)
			{
				const float Coeff = TWOPI*float(i)/float(Nb);
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				Quat Q;
//				UnitRandomQuat(Q, Rnd);
				Q.Identity();
//				RegisterBoxSweep(OBB(Point(Center.x, 2.0f, Center.z), Point(0.1f, 0.1f, 0.1f), Q), D, 1000.0f);
//				RegisterBoxSweep(OBB(Point(8.36f, -4.22f, -28.18f), Point(0.1f, 0.1f, 0.1f), Q), D, 1000.0f);
				RegisterBoxSweep(OBB(Point(8.36f, -4.22f, -28.18f), Point(0.5f, 0.5f, 0.5f), Q), D, 1000.0f);
			}

		}
		else
		{
			IndexedSurface* IS = GetFirstSurface();
			const Point* V = IS->GetVerts();
			const float Length = 0.1f;
			BasicRandom Rnd(42);
			for(udword i=0;i<IS->GetNbFaces();i++)
			{
				const IndexedTriangle* T = IS->GetFace(i);
				Point Center, Normal;
				T->Center(V, Center);
				T->Normal(V, Normal);

				Quat Q;
				UnitRandomQuat(Q, Rnd);
				Q.Identity();

				RegisterBoxSweep(OBB(Center+Normal*2.0f, Point(1.0f, 1.0f, 1.0f), Q), -Normal, 10000.0f);
	//			RegisterBoxSweep(OBB(Center+Normal*2.0f, Point(0.1f, 0.1f, 0.1f), Q), -Normal, 10.0f);
			}
		}
		return true;
	}

	virtual bool BoxSweep_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword BoxSweep_TestZone::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(BoxSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxSweep_TestZone2 = "TestZone. Box sweep 2.";

START_SQ_TEST(BoxSweep2_TestZone, CATEGORY_SWEEP, gDesc_BoxSweep_TestZone2)

	virtual bool BoxSweep2_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllBoxSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
			const udword Nb=2048;
			for(udword i=0;i<Nb;i++)
			{
				const float Coeff = TWOPI*float(i)/float(Nb);
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				Quat Q;
//				UnitRandomQuat(Q, Rnd);
				Q.Identity();
				RegisterBoxSweep(OBB(Point(-15.08f, 5.20f, -58.01f), Point(0.5f, 0.5f, 0.5f), Q), D, 1000.0f);
			}

		}
		return true;
	}

	virtual bool BoxSweep2_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword BoxSweep2_TestZone::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(BoxSweep2_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereSweep_TestZone = "TestZone. Sphere sweep.";

START_SQ_TEST(SphereSweep_TestZone, CATEGORY_SWEEP, gDesc_SphereSweep_TestZone)

	virtual bool SphereSweep_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllSphereSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
			for(udword i=0;i<1024;i++)
			{
				const float Coeff = TWOPI*float(i)/1024.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterSphereSweep(Sphere(Point(-15.08f, 5.20f, -58.01f), 0.5f), D, 1000.0f);
			}
		}
		return true;
	}

	virtual bool SphereSweep_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword SphereSweep_TestZone::Update(Pint& pint)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SphereSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleSweep_TestZone = "TestZone. Capsule sweep.";

START_SQ_TEST(CapsuleSweep_TestZone, CATEGORY_SWEEP, gDesc_CapsuleSweep_TestZone)

	virtual bool CapsuleSweep_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllCapsuleSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
//			for(udword i=0;i<1024;i++)
			for(udword i=0;i<32;i++)
			{
//				const float Coeff = TWOPI*float(i)/1024.0f;
				const float Coeff = TWOPI*float(i)/32.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterCapsuleSweep(LSS(Segment(Point(-15.08f, 5.20f, -58.01f), Point(-15.08f, 7.20f, -58.01f)), 0.5f), D, 1000.0f);
			}
		}
		return true;
	}

	virtual bool CapsuleSweep_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword CapsuleSweep_TestZone::Update(Pint& pint)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(CapsuleSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ValveRaysAndSweeps = "Valve level, raycasts & sweeps.";

START_SQ_TEST(ValveRaysAndSweeps, CATEGORY_SWEEP, gDesc_ValveRaysAndSweeps)

	virtual bool ValveRaysAndSweeps::IsPrivate()	const
	{
		return true;
	}

	virtual bool ValveRaysAndSweeps::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "rays(lotsof boxes).bin", false);
		mRepX = CreateRepXContext("c5m4_quarter2_Statics.repx", gValveScale, true);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool ValveRaysAndSweeps::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts || !caps.mSupportMeshes || !caps.mSupportBoxSweeps)
			return false;
		return AddToPint(pint, mRepX);
	}

	virtual udword ValveRaysAndSweeps::Update(Pint& pint)
	{
		udword Nb1 = DoBatchRaycasts(*this, pint);
		udword Nb2 = DoBatchBoxSweeps(*this, pint);
		return Nb1+Nb2;
	}

END_TEST(ValveRaysAndSweeps)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PlanetsideBoxSweeps = "512*4 box sweeps against the Planetside level.";

START_SQ_TEST(PlanetsideBoxSweeps, CATEGORY_SWEEP, gDesc_PlanetsideBoxSweeps)

	virtual bool PlanetsideBoxSweeps::IsPrivate()	const
	{
		return true;
	}

	virtual void PlanetsideBoxSweeps::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(157.59f, 21.66f, 770.35f), Point(0.66f, -0.54f, 0.52f));
		desc.mCamera[1] = CameraPose(Point(248.93f, 12.60f, 676.81f), Point(-0.04f, -0.23f, 0.97f));
		desc.mCamera[2] = CameraPose(Point(275.19f, 32.54f, 703.23f), Point(-1.00f, -0.08f, 0.05f));
		desc.mCamera[3] = CameraPose(Point(295.76f, 4.08f, 845.31f), Point(-0.32f, 0.11f, -0.94f));

		SRand(42);
		const Point Orig(241.09f, 6.29f, 730.26f);
		const Point Orig2(164.74f, 1.67f, 783.85f);
		const Point Orig3(275.19f, 32.54f, 703.23f);
		const Point Orig4(295.76f, 4.08f, 845.31f);
		Matrix3x3 Rot;
		Rot.Identity();
		for(udword j=0;j<512;j++)
		{
			const float angle = (float(j)/512.0f)*3.14159f*2.0f;
			const float s = sinf(angle);
			const float c = cosf(angle);
			Point Dir(s, 0.1f*(UnitRandomFloat()-0.5f), c);
			Dir.Normalize();
			RegisterBoxSweep(OBB(Orig, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig2, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig3, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig4, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
		}
	}

	virtual bool PlanetsideBoxSweeps::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool PlanetsideBoxSweeps::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportBoxSweeps)
			return false;
		return AddToPint(pint, mRepX);
	}

	virtual udword PlanetsideBoxSweeps::Update(Pint& pint)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(PlanetsideBoxSweeps)

