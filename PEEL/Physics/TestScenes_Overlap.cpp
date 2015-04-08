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
#include "Loader_RepX.h"
#include "Loader_Bin.h"
#include "Random.h"
#include "PintSQ.h"

///////////////////////////////////////////////////////////////////////////////

static bool SetupSphereOverlaps(TestBase& test, udword nb_x, udword nb_y, float scale_x, float scale_y, float altitude, float sphere_radius)
{
	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			test.RegisterSphereOverlap(Sphere(Origin, sphere_radius));
		}
	}
	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool SetupBoxOverlaps(TestBase& test, udword nb_x, udword nb_y, float scale_x, float scale_y, float altitude, float box_radius)
{
	BasicRandom Rnd(42);

	const Point Extents(box_radius, box_radius, box_radius*2.0f);

	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			Quat Q;
			UnitRandomQuat(Q, Rnd);
			const Matrix3x3 Rot = Q;

			test.RegisterBoxOverlap(OBB(Origin, Extents, Rot));
		}
	}
	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool SetupCapsuleOverlaps(TestBase& test, udword nb_x, udword nb_y, float scale_x, float scale_y, float altitude, float capsule_radius)
{
	BasicRandom Rnd(42);

	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			Quat Q;
			UnitRandomQuat(Q, Rnd);
			const Matrix3x3 Rot = Q;

//			const Point D = Rot[1] * capsule_radius;
			const Point D = Rot[1] * 10.0f;

			test.RegisterCapsuleOverlap(LSS(Segment(Origin-D, Origin+D), capsule_radius));
		}
	}
	test.mCreateDefaultEnvironment = false;
	return true;
}

///////////////////////////////////////////////////////////////////////////////

#define IMPLEMENT_OVERLAP_TEST(name, category, desc, query_type,											\
		altitude, sphere_radius, static_shape, query_shape)													\
	class name : public TestBase																			\
	{																										\
		public:																								\
								name(PintShape ss, PintShape qs) :											\
									mStaticShape(ss), mQueryShape(qs)	{						}			\
		virtual					~name()									{						}			\
		virtual	const char*		GetName()						const	{ return #name;			}			\
		virtual	const char*		GetDescription()				const	{ return desc;			}			\
		virtual	TestCategory	GetCategory()					const	{ return category;		}			\
		virtual	bool			ProfileUpdate()							{ return true;			}			\
		virtual bool			CommonSetup()																\
		{																									\
			TestBase::CommonSetup();																		\
			if(mQueryShape==PINT_SHAPE_SPHERE)																\
				return SetupSphereOverlaps(*this, 32, 32, 200.0f, 200.0f, altitude, sphere_radius);			\
			else if(mQueryShape==PINT_SHAPE_BOX)															\
				return SetupBoxOverlaps(*this, 32, 32, 200.0f, 200.0f, altitude, sphere_radius);			\
			else																							\
				return false;																				\
		}																									\
		virtual bool			Setup(Pint& pint, const PintCaps& caps)										\
		{																									\
			if(mQueryShape==PINT_SHAPE_SPHERE && !caps.mSupportSphereOverlaps)								\
				return false;																				\
			if(mQueryShape==PINT_SHAPE_BOX && !caps.mSupportBoxOverlaps)									\
				return false;																				\
			if(mStaticShape==PINT_SHAPE_SPHERE)																\
				return CreateSeaOfStaticSpheres(pint, 200.0f, 64, 64, 0.0f);								\
			if(mStaticShape==PINT_SHAPE_CAPSULE)															\
				return CreateSeaOfStaticCapsules(pint, 200.0f, 64, 64, 0.0f);								\
			if(mStaticShape==PINT_SHAPE_BOX)																\
				return CreateSeaOfStaticBoxes(pint, 200.0f, 64, 64, 0.0f);									\
			if(mStaticShape==PINT_SHAPE_CONVEX)																\
				return CreateSeaOfStaticConvexes(pint, caps, 64, 64, 0.0f);									\
			return false;																					\
		}																									\
		virtual udword			Update(Pint& pint)															\
		{																									\
			if(mQueryShape==PINT_SHAPE_SPHERE)																\
				return DoBatchSphereOverlaps(*this, pint, query_type);										\
			else if(mQueryShape==PINT_SHAPE_BOX)															\
				return DoBatchBoxOverlaps(*this, pint, query_type);											\
			return 0;																						\
		}																									\
				PintShape		mStaticShape;																\
				PintShape		mQueryShape;																\
	}name(static_shape, query_shape);

///////////////////////////////////////////////////////////////////////////////

namespace OverlapAny_Sphere
{
	static const char* gDesc_OverlapAny_SmallSpheres_VS_Spheres = "OVERLAP_ANY: 32*32 (small) sphere overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallSpheres_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallSpheres_VS_Spheres, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_SmallSpheres_VS_Capsules = "OVERLAP_ANY: 32*32 (small) sphere overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallSpheres_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallSpheres_VS_Capsules, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_SmallSpheres_VS_Boxes = "OVERLAP_ANY: 32*32 (small) sphere overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallSpheres_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallSpheres_VS_Boxes, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_BOX, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_SmallSpheres_VS_Convexes = "OVERLAP_ANY: 32*32 (small) sphere overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallSpheres_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallSpheres_VS_Convexes, OVERLAP_ANY, 0.0f, 2.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_SPHERE)

	///////////////////////////////////////////////////////////////////////////////

	static const char* gDesc_OverlapAny_LargeSpheres_VS_Spheres = "OVERLAP_ANY: 32*32 (large) sphere overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeSpheres_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeSpheres_VS_Spheres, OVERLAP_ANY, 0.0f, 20.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_LargeSpheres_VS_Capsules = "OVERLAP_ANY: 32*32 (large) sphere overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeSpheres_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeSpheres_VS_Capsules, OVERLAP_ANY, 0.0f, 20.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_LargeSpheres_VS_Boxes = "OVERLAP_ANY: 32*32 (large) sphere overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeSpheres_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeSpheres_VS_Boxes, OVERLAP_ANY, 0.0f, 20.0f, PINT_SHAPE_BOX, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapAny_LargeSpheres_VS_Convexes = "OVERLAP_ANY: 32*32 (large) sphere overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeSpheres_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeSpheres_VS_Convexes, OVERLAP_ANY, 0.0f, 20.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_SPHERE)
}

///////////////////////////////////////////////////////////////////////////////

namespace OverlapObjects_Sphere
{
	static const char* gDesc_OverlapObjects_SmallSpheres_VS_Spheres = "OVERLAP_OBJECTS: 32*32 (small) sphere overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallSpheres_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallSpheres_VS_Spheres, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_SmallSpheres_VS_Capsules = "OVERLAP_OBJECTS: 32*32 (small) sphere overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallSpheres_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallSpheres_VS_Capsules, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_SmallSpheres_VS_Boxes = "OVERLAP_OBJECTS: 32*32 (small) sphere overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallSpheres_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallSpheres_VS_Boxes, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_BOX, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_SmallSpheres_VS_Convexes = "OVERLAP_OBJECTS: 32*32 (small) sphere overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallSpheres_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallSpheres_VS_Convexes, OVERLAP_OBJECTS, 0.0f, 2.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_SPHERE)

	///////////////////////////////////////////////////////////////////////////////

	static const char* gDesc_OverlapObjects_LargeSpheres_VS_Spheres = "OVERLAP_OBJECTS: 32*32 (large) sphere overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeSpheres_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeSpheres_VS_Spheres, OVERLAP_OBJECTS, 0.0f, 20.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_LargeSpheres_VS_Capsules = "OVERLAP_OBJECTS: 32*32 (large) sphere overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeSpheres_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeSpheres_VS_Capsules, OVERLAP_OBJECTS, 0.0f, 20.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_LargeSpheres_VS_Boxes = "OVERLAP_OBJECTS: 32*32 (large) sphere overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeSpheres_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeSpheres_VS_Boxes, OVERLAP_OBJECTS, 0.0f, 20.0f, PINT_SHAPE_BOX, PINT_SHAPE_SPHERE)

	static const char* gDesc_OverlapObjects_LargeSpheres_VS_Convexes = "OVERLAP_OBJECTS: 32*32 (large) sphere overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeSpheres_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeSpheres_VS_Convexes, OVERLAP_OBJECTS, 0.0f, 20.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_SPHERE)
}

///////////////////////////////////////////////////////////////////////////////

namespace OverlapAny_Box
{
	static const char* gDesc_OverlapAny_SmallBoxes_VS_Spheres = "OVERLAP_ANY: 32*32 (small) box overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallBoxes_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallBoxes_VS_Spheres, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_SmallBoxes_VS_Capsules = "OVERLAP_ANY: 32*32 (small) box overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallBoxes_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallBoxes_VS_Capsules, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_SmallBoxes_VS_Boxes = "OVERLAP_ANY: 32*32 (small) box overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallBoxes_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallBoxes_VS_Boxes, OVERLAP_ANY, 0.0f, 1.0f, PINT_SHAPE_BOX, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_SmallBoxes_VS_Convexes = "OVERLAP_ANY: 32*32 (small) box overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_SmallBoxes_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapAny_SmallBoxes_VS_Convexes, OVERLAP_ANY, 0.0f, 2.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_BOX)

	///////////////////////////////////////////////////////////////////////////////

	static const char* gDesc_OverlapAny_LargeBoxes_VS_Spheres = "OVERLAP_ANY: 32*32 (large) box overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeBoxes_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeBoxes_VS_Spheres, OVERLAP_ANY, 0.0f, 10.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_LargeBoxes_VS_Capsules = "OVERLAP_ANY: 32*32 (large) box overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeBoxes_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeBoxes_VS_Capsules, OVERLAP_ANY, 0.0f, 10.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_LargeBoxes_VS_Boxes = "OVERLAP_ANY: 32*32 (large) box overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeBoxes_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeBoxes_VS_Boxes, OVERLAP_ANY, 0.0f, 10.0f, PINT_SHAPE_BOX, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapAny_LargeBoxes_VS_Convexes = "OVERLAP_ANY: 32*32 (large) box overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapAny_LargeBoxes_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapAny_LargeBoxes_VS_Convexes, OVERLAP_ANY, 0.0f, 20.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_BOX)
}

///////////////////////////////////////////////////////////////////////////////

namespace OverlapObjects_Box
{
	static const char* gDesc_OverlapObjects_SmallBoxes_VS_Spheres = "OVERLAP_OBJECTS: 32*32 (small) box overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallBoxes_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallBoxes_VS_Spheres, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_SmallBoxes_VS_Capsules = "OVERLAP_OBJECTS: 32*32 (small) box overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallBoxes_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallBoxes_VS_Capsules, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_SmallBoxes_VS_Boxes = "OVERLAP_OBJECTS: 32*32 (small) box overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallBoxes_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallBoxes_VS_Boxes, OVERLAP_OBJECTS, 0.0f, 1.0f, PINT_SHAPE_BOX, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_SmallBoxes_VS_Convexes = "OVERLAP_OBJECTS: 32*32 (small) box overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_SmallBoxes_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapObjects_SmallBoxes_VS_Convexes, OVERLAP_OBJECTS, 0.0f, 2.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_BOX)

	///////////////////////////////////////////////////////////////////////////////

	static const char* gDesc_OverlapObjects_LargeBoxes_VS_Spheres = "OVERLAP_OBJECTS: 32*32 (large) box overlaps against 64*64 static spheres.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeBoxes_VS_Spheres, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeBoxes_VS_Spheres, OVERLAP_OBJECTS, 0.0f, 10.0f, PINT_SHAPE_SPHERE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_LargeBoxes_VS_Capsules = "OVERLAP_OBJECTS: 32*32 (large) box overlaps against 64*64 static capsules.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeBoxes_VS_Capsules, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeBoxes_VS_Capsules, OVERLAP_OBJECTS, 0.0f, 10.0f, PINT_SHAPE_CAPSULE, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_LargeBoxes_VS_Boxes = "OVERLAP_OBJECTS: 32*32 (large) box overlaps against 64*64 static boxes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeBoxes_VS_Boxes, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeBoxes_VS_Boxes, OVERLAP_OBJECTS, 0.0f, 10.0f, PINT_SHAPE_BOX, PINT_SHAPE_BOX)

	static const char* gDesc_OverlapObjects_LargeBoxes_VS_Convexes = "OVERLAP_OBJECTS: 32*32 (large) box overlaps against 64*64 static convexes.";
	IMPLEMENT_OVERLAP_TEST(OverlapObjects_LargeBoxes_VS_Convexes, CATEGORY_OVERLAP, gDesc_OverlapObjects_LargeBoxes_VS_Convexes, OVERLAP_OBJECTS, 0.0f, 20.0f, PINT_SHAPE_CONVEX, PINT_SHAPE_BOX)
}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_DynamicSphere = "OVERLAP_OBJECTS: single dynamic sphere overlap against 128*128 static boxes.";

START_SQ_TEST(OverlapObjects_DynamicSphere, CATEGORY_OVERLAP, gDesc_OverlapObjects_DynamicSphere)

	virtual bool OverlapObjects_DynamicSphere::CommonSetup()
	{
		TestBase::CommonSetup();
		RegisterSphereOverlap(Sphere(Point(0.0f, 0.0f, 0.0f), 1.0f));
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapObjects_DynamicSphere::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 200.0f, 128, 128, 0.0f);
	}

	virtual void OverlapObjects_DynamicSphere::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		Data->mSphere.mRadius = (2.0f+sinf(mCurrentTime*1.0f))*10.0f;
	}

	virtual udword OverlapObjects_DynamicSphere::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_DynamicSphere)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_DynamicSpheres = "OVERLAP_OBJECTS: 32*32 (dynamic) spheres against 128*128 static boxes.";

START_SQ_TEST(OverlapObjects_DynamicSpheres, CATEGORY_OVERLAP, gDesc_OverlapObjects_DynamicSpheres)

	virtual bool OverlapObjects_DynamicSpheres::CommonSetup()
	{
		TestBase::CommonSetup();
		return SetupSphereOverlaps(*this, 32, 32, 200.0f, 200.0f, 0.0f, 20.0f);
	}

	virtual bool OverlapObjects_DynamicSpheres::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 200.0f, 128, 128, 0.0f);
	}

	virtual void OverlapObjects_DynamicSpheres::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredSphereOverlaps();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		const float Radius = (2.0f+sinf(mCurrentTime*1.0f))*5.0f;
		while(Nb--)
		{
			Data->mSphere.mRadius = Radius;
			Data++;
		}
	}

	virtual udword OverlapObjects_DynamicSpheres::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_DynamicSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_SphereVsTessBunny = "OVERLAP_ANY: 1 sphere against the tesselated bunny. This is really just to investigate an issue in PhysX.";

START_SQ_TEST(OverlapAny_SphereVsTessBunny, CATEGORY_OVERLAP, gDesc_OverlapAny_SphereVsTessBunny)

	virtual bool OverlapAny_SphereVsTessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "bunny.bin", null, false, 3);
		RegisterSphereOverlap(Sphere(Point(0.0f, 0.0f, 0.0f), 100.0f));
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapAny_SphereVsTessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapAny_SphereVsTessBunny::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_SphereVsTessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_SphereVsTessBunny = "OVERLAP_OBJECTS: 1 sphere against the tesselated bunny. This is really just to investigate an issue in Havok.";

START_SQ_TEST(OverlapObjects_SphereVsTessBunny, CATEGORY_OVERLAP, gDesc_OverlapObjects_SphereVsTessBunny)

	virtual bool OverlapObjects_SphereVsTessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "bunny.bin", null, false, 3);
		RegisterSphereOverlap(Sphere(Point(0.0f, 0.0f, 0.0f), 100.0f));
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapObjects_SphereVsTessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapObjects_SphereVsTessBunny::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_SphereVsTessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_SpheresVsTessBunny = "OVERLAP_ANY: 16*16*16 spheres against the tesselated bunny";

START_SQ_TEST(OverlapAny_SpheresVsTessBunny, CATEGORY_OVERLAP, gDesc_OverlapAny_SpheresVsTessBunny)

	virtual bool OverlapAny_SpheresVsTessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "bunny.bin", null, false, 3);
		for(udword i=0;i<16;i++)
			SetupSphereOverlaps(*this, 16, 16, 10.0f, 10.0f, float(i), 0.5f);
		return true;
	}

	virtual bool OverlapAny_SpheresVsTessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual void OverlapAny_SpheresVsTessBunny::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredSphereOverlaps();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		const float Radius = 0.1f+(1.0f + sinf(mCurrentTime*1.0f))*1.0f;
		while(Nb--)
		{
			Data->mSphere.mRadius = Radius;
			Data++;
		}
	}

	virtual udword OverlapAny_SpheresVsTessBunny::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_SpheresVsTessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_BoxesVsTessBunny = "OVERLAP_ANY: 16*16*16 boxes against the tesselated bunny";

START_SQ_TEST(OverlapAny_BoxesVsTessBunny, CATEGORY_OVERLAP, gDesc_OverlapAny_BoxesVsTessBunny)

	virtual bool OverlapAny_BoxesVsTessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "bunny.bin", null, false, 3);
		for(udword i=0;i<16;i++)
			SetupBoxOverlaps(*this, 16, 16, 10.0f, 10.0f, float(i), 0.5f);
		return true;
	}

	virtual bool OverlapAny_BoxesVsTessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapAny_BoxesVsTessBunny::Update(Pint& pint)
	{
		return DoBatchBoxOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_BoxesVsTessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_CapsulesVsTessBunny = "OVERLAP_ANY: 16*16*16 capsules against the tesselated bunny";

START_SQ_TEST(OverlapAny_CapsulesVsTessBunny, CATEGORY_OVERLAP, gDesc_OverlapAny_CapsulesVsTessBunny)

	virtual bool OverlapAny_CapsulesVsTessBunny::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "bunny.bin", null, false, 3);
		for(udword i=0;i<16;i++)
			SetupCapsuleOverlaps(*this, 16, 16, 10.0f, 10.0f, float(i), 0.5f);
		return true;
	}

	virtual bool OverlapAny_CapsulesVsTessBunny::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapAny_CapsulesVsTessBunny::Update(Pint& pint)
	{
		return DoBatchCapsuleOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_CapsulesVsTessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_SpheresVsTerrain = "OVERLAP_ANY: 4096 spheres against the terrain mesh";

START_SQ_TEST(OverlapAny_SpheresVsTerrain, CATEGORY_OVERLAP, gDesc_OverlapAny_SpheresVsTerrain)

	virtual void OverlapAny_SpheresVsTerrain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-480.68f, 1639.54f, -443.50f), Point(0.59f, -0.60f, 0.54f));
	}

	virtual bool OverlapAny_SpheresVsTerrain::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "terrain.bin");

		const Point Min(-128.0f, -432.117371f, -128.0f);
		const Point Max(3968.0f, 710.848145f, 3968.0f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		BasicRandom Rnd(42);
		for(udword i=0;i<4096;i++)
		{
			Point p;
			UnitRandomPt(p, Rnd);
			Point Pos = Center + Extents*p;
			Pos.y = 0.0f;
			RegisterSphereOverlap(Sphere(Pos, 20.0f));
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapAny_SpheresVsTerrain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapAny_SpheresVsTerrain::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_SpheresVsTerrain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_BoxesVsTerrain = "OVERLAP_ANY: 4096 boxes against the terrain mesh";

START_SQ_TEST(OverlapAny_BoxesVsTerrain, CATEGORY_OVERLAP, gDesc_OverlapAny_BoxesVsTerrain)

	virtual void OverlapAny_BoxesVsTerrain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-480.68f, 1639.54f, -443.50f), Point(0.59f, -0.60f, 0.54f));
	}

	virtual bool OverlapAny_BoxesVsTerrain::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "terrain.bin");

		const Point Min(-128.0f, -432.117371f, -128.0f);
		const Point Max(3968.0f, 710.848145f, 3968.0f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		BasicRandom Rnd(42);
		Matrix3x3 Rot;
		Rot.RotX(45.0f * DEGTORAD);
		for(udword i=0;i<4096;i++)
		{
			Point p;
			UnitRandomPt(p, Rnd);
			Point Pos = Center + Extents*p;
			Pos.y = 0.0f;
			RegisterBoxOverlap(OBB(Pos, Point(20.0f, 20.0f, 20.0f), Rot));
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapAny_BoxesVsTerrain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapAny_BoxesVsTerrain::Update(Pint& pint)
	{
//		return DoBatchBoxOverlaps(*this, pint, OVERLAP_OBJECTS);
		return DoBatchBoxOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_BoxesVsTerrain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_SpheresVsPlanetSide = "OVERLAP_ANY: 4096 spheres against the PlanetSide level";

START_SQ_TEST(OverlapAny_SpheresVsPlanetSide, CATEGORY_OVERLAP, gDesc_OverlapAny_SpheresVsPlanetSide)

	virtual bool OverlapAny_SpheresVsPlanetSide::IsPrivate()	const
	{
		return true;
	}

	virtual void OverlapAny_SpheresVsPlanetSide::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-480.68f, 1639.54f, -443.50f), Point(0.59f, -0.60f, 0.54f));
	}

	virtual bool OverlapAny_SpheresVsPlanetSide::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);

		const Point Min(0.0f, -20.f, 0.0f);
		const Point Max(1000.0f, 20.0f, 1000.0f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		BasicRandom Rnd(42);
		for(udword i=0;i<4096;i++)
		{
			Point p;
			UnitRandomPt(p, Rnd);
			Point Pos = Center + Extents*p;
//			Pos.y = 0.0f;
//			RegisterSphereOverlap(Sphere(Pos, 2.0f));
//			RegisterSphereOverlap(Sphere(Pos, 20.0f));
			RegisterSphereOverlap(Sphere(Pos, 10.0f));
//			RegisterSphereOverlap(Sphere(Pos, 1.0f));
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapAny_SpheresVsPlanetSide::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps || !caps.mSupportMeshes)
			return false;

		return AddToPint(pint, mRepX);
	}

	virtual void OverlapAny_SpheresVsPlanetSide::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredSphereOverlaps();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		const float Radius = 1.0f+(1.0f + sinf(mCurrentTime*1.0f))*5.0f;
		while(Nb--)
		{
			Data->mSphere.mRadius = Radius;
			Data++;
		}
	}

	virtual udword OverlapAny_SpheresVsPlanetSide::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_SpheresVsPlanetSide)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapAny_BoxesVsPlanetSide = "OVERLAP_ANY: 4096 boxes against the PlanetSide level";

START_SQ_TEST(OverlapAny_BoxesVsPlanetSide, CATEGORY_OVERLAP, gDesc_OverlapAny_BoxesVsPlanetSide)

	virtual bool OverlapAny_BoxesVsPlanetSide::IsPrivate()	const
	{
		return true;
	}

	virtual void OverlapAny_BoxesVsPlanetSide::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-480.68f, 1639.54f, -443.50f), Point(0.59f, -0.60f, 0.54f));
	}

	virtual bool OverlapAny_BoxesVsPlanetSide::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);

		const Point Min(0.0f, -20.f, 0.0f);
		const Point Max(1000.0f, 20.0f, 1000.0f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		BasicRandom Rnd(42);
		Matrix3x3 Rot;
		Rot.RotX(45.0f * DEGTORAD);
		for(udword i=0;i<4096;i++)
		{
			Point p;
			UnitRandomPt(p, Rnd);
			Point Pos = Center + Extents*p;
//			Pos.y = 0.0f;
			RegisterBoxOverlap(OBB(Pos, Point(10.0f, 10.0f, 10.0f), Rot));
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapAny_BoxesVsPlanetSide::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps || !caps.mSupportMeshes)
			return false;

		return AddToPint(pint, mRepX);
	}

/*	virtual void OverlapAny_BoxesVsPlanetSide::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredSphereOverlaps();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		const float Radius = 1.0f+(1.0f + sinf(mCurrentTime*1.0f))*5.0f;
		while(Nb--)
		{
			Data->mSphere.mRadius = Radius;
			Data++;
		}
	}*/

	virtual udword OverlapAny_BoxesVsPlanetSide::Update(Pint& pint)
	{
		return DoBatchBoxOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(OverlapAny_BoxesVsPlanetSide)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_SpheresVsKP = "OVERLAP_OBJECTS: 4096 spheres against the KP level";

START_SQ_TEST(OverlapObjects_SpheresVsKP, CATEGORY_OVERLAP, gDesc_OverlapObjects_SpheresVsKP)

	virtual void OverlapObjects_SpheresVsKP::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1085.22f, 3963.36f, -9239.41f), Point(-0.59f, -0.59f, 0.56f));
	}

	virtual bool OverlapObjects_SpheresVsKP::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "kp.bin");

		const Point Min(-9560.175781f, -1093.885132f, -9461.288086f);
		const Point Max(9538.423828f, 4906.125488f, 9637.304688f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		BasicRandom Rnd(42);
		for(udword i=0;i<4096;i++)
		{
			Point p;
			UnitRandomPt(p, Rnd);
			Point Pos = Center + Extents*p;
//			Pos.y = 0.0f;
			Pos.y *= 0.1f;
			RegisterSphereOverlap(Sphere(Pos, 200.0f));
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool OverlapObjects_SpheresVsKP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapObjects_SpheresVsKP::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_SpheresVsKP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_GiantSphereVsKP = "OVERLAP_OBJECTS: one single giant sphere against the KP level. Goal is to retrieve all meshes touched by the sphere. It's one of these rare tests where Opcode 1.3 still beats everybody, thanks to its 'containment tests' during traversal.";

START_SQ_TEST(OverlapObjects_GiantSphereVsKP, CATEGORY_OVERLAP, gDesc_OverlapObjects_GiantSphereVsKP)

	virtual void OverlapObjects_GiantSphereVsKP::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(926.05f, 933.78f, 523.28f), Point(-0.03f, -0.00f, -1.00f));
	}

	virtual bool OverlapObjects_GiantSphereVsKP::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "kp.bin");

		const Point Min(-9560.175781f, -1093.885132f, -9461.288086f);
		const Point Max(9538.423828f, 4906.125488f, 9637.304688f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

//		RegisterSphereOverlap(Sphere(Center, Extents.Magnitude()*0.25f));
		RegisterSphereOverlap(Sphere(Center, 2500.0f));
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void OverlapObjects_GiantSphereVsKP::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredSphereOverlaps();
		PintSphereOverlapData* Data = GetRegisteredSphereOverlaps();
		const float Radius = 0.1f+(1.0f + sinf(mCurrentTime*1.0f))*1250.0f;
		while(Nb--)
		{
			Data->mSphere.mRadius = Radius;
			Data++;
		}
	}

	virtual bool OverlapObjects_GiantSphereVsKP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapObjects_GiantSphereVsKP::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_GiantSphereVsKP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OverlapObjects_GiantBoxVsKP = "OVERLAP_OBJECTS: one single giant box against the KP level. Goal is to retrieve all meshes touched by the box.";

START_SQ_TEST(OverlapObjects_GiantBoxVsKP, CATEGORY_OVERLAP, gDesc_OverlapObjects_GiantBoxVsKP)

	virtual void OverlapObjects_GiantBoxVsKP::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(926.05f, 933.78f, 523.28f), Point(-0.03f, -0.00f, -1.00f));
	}

	virtual bool OverlapObjects_GiantBoxVsKP::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "kp.bin");

		const Point Min(-9560.175781f, -1093.885132f, -9461.288086f);
		const Point Max(9538.423828f, 4906.125488f, 9637.304688f);
		const Point Center = (Max + Min)*0.5f;
		const Point Extents = (Max - Min)*0.5f;

		Matrix3x3 Rot;
		Rot.Identity();
//		Rot.RotYX(0.1f, 0.2f);
		RegisterBoxOverlap(OBB(Center, Point(2500.0f, 2500.0f, 2500.0f), Rot));
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void OverlapObjects_GiantBoxVsKP::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredBoxOverlaps();
		PintBoxOverlapData* Data = GetRegisteredBoxOverlaps();
		const float Radius = 0.1f+(1.0f + sinf(mCurrentTime*1.0f))*1250.0f;
		while(Nb--)
		{
			Data->mBox.mExtents = Point(Radius*0.75f, Radius*0.75f, Radius*0.75f);
			Data++;
		}
	}

	virtual bool OverlapObjects_GiantBoxVsKP::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword OverlapObjects_GiantBoxVsKP::Update(Pint& pint)
	{
		return DoBatchBoxOverlaps(*this, pint, OVERLAP_OBJECTS);
	}

END_TEST(OverlapObjects_GiantBoxVsKP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereOverlapTriangles_TestZone = "MIDPHASE: TestZone. Sphere overlap triangles.";

START_SQ_TEST(SphereOverlapTriangles_TestZone, CATEGORY_OVERLAP, gDesc_SphereOverlapTriangles_TestZone)

	virtual bool SphereOverlapTriangles_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SphereOverlapTriangles_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword SphereOverlapTriangles_TestZone::Update(Pint& pint)
	{
		PintObjectHandle MeshHandle = pint.mOMHelper->GetObject(0);

		PintSphereOverlapData Data;
		Data.mSphere.mCenter = Point(0.0f, 0.0f, 0.0f);
//		Data.mSphere.mRadius = 60.0f;
		Data.mSphere.mRadius = 40.0f;
//		Data.mSphere.mRadius = 30.0f;
//		Data.mSphere.mRadius = 10.0f;
		return pint.FindTriangles_MeshSphereOverlap(pint.mSQHelper->GetThreadContext(), MeshHandle, 1, &Data);
	}

END_TEST(SphereOverlapTriangles_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxOverlapTriangles_TestZone = "MIDPHASE: TestZone. Box overlap triangles.";

START_SQ_TEST(BoxOverlapTriangles_TestZone, CATEGORY_OVERLAP, gDesc_BoxOverlapTriangles_TestZone)

	virtual bool BoxOverlapTriangles_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool BoxOverlapTriangles_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword BoxOverlapTriangles_TestZone::Update(Pint& pint)
	{
		PintObjectHandle MeshHandle = pint.mOMHelper->GetObject(0);

		PintBoxOverlapData Data;
		Data.mBox.mCenter = Point(0.0f, 0.0f, 0.0f);
		Data.mBox.mExtents = Point(30.0f, 40.0f, 30.0f);
		Data.mBox.mRot.RotYX(0.42f, 0.27f);
		return pint.FindTriangles_MeshBoxOverlap(pint.mSQHelper->GetThreadContext(), MeshHandle, 1, &Data);
	}

END_TEST(BoxOverlapTriangles_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleOverlapTriangles_TestZone = "MIDPHASE: TestZone. Capsule overlap triangles.";

START_SQ_TEST(CapsuleOverlapTriangles_TestZone, CATEGORY_OVERLAP, gDesc_CapsuleOverlapTriangles_TestZone)

	virtual bool CapsuleOverlapTriangles_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool CapsuleOverlapTriangles_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword CapsuleOverlapTriangles_TestZone::Update(Pint& pint)
	{
		PintObjectHandle MeshHandle = pint.mOMHelper->GetObject(0);

		PintCapsuleOverlapData Data;
		Data.mCapsule.mP0 = Point(0.0f, 0.0f, 0.0f);
		Data.mCapsule.mP1 = Point(10.0f, 10.0f, 10.0f);
		Data.mCapsule.mRadius = 40.0f;
		return pint.FindTriangles_MeshCapsuleOverlap(pint.mSQHelper->GetThreadContext(), MeshHandle, 1, &Data);
	}

END_TEST(CapsuleOverlapTriangles_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereOverlapAny_TestZone = "TestZone. Sphere overlap any.";

START_SQ_TEST(SphereOverlapAny_TestZone, CATEGORY_OVERLAP, gDesc_SphereOverlapAny_TestZone)

	virtual bool SphereOverlapAny_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllSphereOverlaps();

		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		const float Length = 0.1f;
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);
			Point Center;
			T->Center(V, Center);
			RegisterSphereOverlap(Sphere(Center, 10.0f));
//			RegisterSphereOverlap(Sphere(Center, 1.0f));
//			RegisterSphereOverlap(Sphere(Center, 0.1f));
		}
		return true;
	}

	virtual bool SphereOverlapAny_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword SphereOverlapAny_TestZone::Update(Pint& pint)
	{
		return DoBatchSphereOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(SphereOverlapAny_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxOverlapAny_TestZone = "TestZone. Box overlap any.";

START_SQ_TEST(BoxOverlapAny_TestZone, CATEGORY_OVERLAP, gDesc_BoxOverlapAny_TestZone)

	virtual bool BoxOverlapAny_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllBoxOverlaps();

		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		BasicRandom Rnd(42);
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);
			Point Center;
			T->Center(V, Center);

//			Center.y += 10.0f;

			if(0)
			{
				Point N;
				T->Normal(V, N);
				Center += N*0.4f;
//				Center += N*10.0f;
			}

			Quat Q;
			UnitRandomQuat(Q, Rnd);

//			RegisterBoxOverlap(OBB(Center, Point(0.1f, 0.1f, 0.1f), Q));
//			RegisterBoxOverlap(OBB(Center, Point(10.1f, 10.1f, 10.1f), Q));
			RegisterBoxOverlap(OBB(Center, Point(1.0f, 1.0f, 1.0f), Q));
//				RegisterBoxOverlap(OBB(Center, Point(0.01f, 0.01f, 0.01f), Q));
//			RegisterBoxOverlap(OBB(Center, Point(0.1f, 1.0f, 0.1f), Q));
//			RegisterBoxOverlap(OBB(Center, Point(10.0f, 100.0f, 10.0f), Q));
		}
		return true;
	}

	virtual bool BoxOverlapAny_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword BoxOverlapAny_TestZone::Update(Pint& pint)
	{
		return DoBatchBoxOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(BoxOverlapAny_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleOverlapAny_TestZone = "TestZone. Capsule overlap any.";

START_SQ_TEST(CapsuleOverlapAny_TestZone, CATEGORY_OVERLAP, gDesc_CapsuleOverlapAny_TestZone)

	virtual bool CapsuleOverlapAny_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllCapsuleOverlaps();

		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		BasicRandom Rnd(42);
		const float R = 1.0f;
//		const float R = 0.1f;
//		const float R = 10.0f;
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);
			Point Center;
			T->Center(V, Center);

//			Center.y += 10.0f;

			Quat Q;
			UnitRandomQuat(Q, Rnd);

			Matrix3x3 M = Q;
			const Point D = M[1]*4.0f;
//			const Point D = M[1]*2.0f;

			RegisterCapsuleOverlap(LSS(Segment(Center-D, Center+D), R));	// 1.95
		}
		return true;
	}

	virtual bool CapsuleOverlapAny_TestZone::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleOverlaps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword CapsuleOverlapAny_TestZone::Update(Pint& pint)
	{
		return DoBatchCapsuleOverlaps(*this, pint, OVERLAP_ANY);
	}

END_TEST(CapsuleOverlapAny_TestZone)

///////////////////////////////////////////////////////////////////////////////
