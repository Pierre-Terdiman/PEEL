///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Random.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintSQ.h"
#include "SourceRay.h"
#include "MyConvex.h"
#include "Loader_Bin.h"
#include "Loader_RepX.h"
#include "PintObjectsManager.h"

//bool gRaycastClosest = true;
udword gRaycastMode = 0;

///////////////////////////////////////////////////////////////////////////////

#define MAX_NB_TESTS		512
static udword gNbTests = 0;
static PhysicsTest*	gTests[MAX_NB_TESTS];

udword GetNbTests()				{ return gNbTests;						}
PhysicsTest* GetTest(udword i)	{ return i<gNbTests ? gTests[i] : null;	}

void InitTests()
{
	if(!gNbTests)
		return;

	udword* Keys = (udword*)ICE_ALLOC_TMP(sizeof(void*)*gNbTests);
	for(udword i=0;i<gNbTests;i++)
		Keys[i] = gTests[i]->GetCategory();

	RadixSort RS;
	const udword* Ranks = RS.Sort(Keys, gNbTests, RADIX_UNSIGNED).GetRanks();

	for(udword i=0;i<gNbTests;i++)
		Keys[i] = (udword)gTests[i];

	for(udword i=0;i<gNbTests;i++)
		gTests[i] = (PhysicsTest*)Keys[Ranks[i]];

	ICE_FREE(Keys);
}

static void RegisterTest(PhysicsTest& test)
{
	ASSERT(gNbTests<MAX_NB_TESTS);
	if(gNbTests==MAX_NB_TESTS)
	{
		printf("Skipping test: %s\n", test.GetName());
		return;
	}
//	printf("Registering test: %s\n", test.GetName());
	gTests[gNbTests++] = &test;
}

static bool CreateDefaultEnvironment(Pint& pint)
{
//return true;
	PINT_BOX_CREATE BoxDesc;
	BoxDesc.mExtents	= Point(400.0f, 10.0f, 400.0f);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
	BoxDesc.mRenderer->SetShadows(false);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes		= &BoxDesc;
	ObjectDesc.mPosition.x	= 0.0f;
	ObjectDesc.mPosition.y	= -10.0f;
	ObjectDesc.mPosition.z	= 0.0f;
	ObjectDesc.mMass		= 0.0f;
	CreatePintObject(pint, ObjectDesc);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

TestBase::TestBase() : mRepX(null), mCurrentTime(0.0f), mCreateDefaultEnvironment(true)
{
	RegisterTest(*this);
}

TestBase::~TestBase()
{
}

void TestBase::GetSceneParams(PINT_WORLD_CREATE& params)
{
	params.mGravity	= Point(0.0f, -9.81f, 0.0f);
}

bool TestBase::CommonSetup()
{
	return true;
}

bool TestBase::Init(Pint& pint)
{
	PintCaps Caps;
	pint.GetCaps(Caps);

	// We need to call the test setup before creating the default environment, to give
	// them a chance to properly create the collision filters before anything gets added
	// to the physics world (which is a requirement in some physics engines).
	if(!Setup(pint, Caps))
		return false;

	return mCreateDefaultEnvironment ? CreateDefaultEnvironment(pint) : true;
}

void TestBase::RegisterAABB(const AABB& aabb)
{
	AABB* Box = (AABB*)mAABBs.Reserve(sizeof(AABB)/sizeof(udword));
	*Box = aabb;
}

void TestBase::RenderAllAABBs(PintRender& renderer)
{
	const Point Color(0.0f, 1.0f, 0.0f);

	udword NbBoxes = GetNbAABBs();
	const AABB* Boxes = GetAABBs();
	for(udword i=0;i<NbBoxes;i++)
		renderer.DrawWirefameAABB(Boxes[i], Color);
}

/////

void TestBase::RegisterRaycast(const Point& origin, const Point& dir, float max_dist, void* phantom_handle)
{
	PintRaycastData* Data = (PintRaycastData*)mRaycastData.Reserve(sizeof(PintRaycastData)/sizeof(udword));
	Data->mOrigin	= origin;
	Data->mDir		= dir;
	Data->mMaxDist	= max_dist;

	if(phantom_handle)
		mPhantomData.Add(udword(phantom_handle));
}

udword TestBase::GetNbRegisteredRaycasts() const
{
	return mRaycastData.GetNbEntries()/(sizeof(PintRaycastData)/sizeof(udword));
}

PintRaycastData* TestBase::GetRegisteredRaycasts() const
{
	return (PintRaycastData*)mRaycastData.GetEntries();
}

udword TestBase::GetNbRegisteredPhantoms() const
{
	return mPhantomData.GetNbEntries();
}

void** TestBase::GetRegisteredPhantoms() const
{
	return (void**)mPhantomData.GetEntries();
}

void TestBase::UnregisterAllRaycasts()
{
	mRaycastData.Reset();
	mPhantomData.Reset();
}

/////

void TestBase::RegisterBoxSweep(const OBB& box, const Point& dir, float max_dist)
{
	PintBoxSweepData* Data = (PintBoxSweepData*)mBoxSweepData.Reserve(sizeof(PintBoxSweepData)/sizeof(udword));
	Data->mBox		= box;
	Data->mDir		= dir;
	Data->mMaxDist	= max_dist;
}

udword TestBase::GetNbRegisteredBoxSweeps() const
{
	return mBoxSweepData.GetNbEntries()/(sizeof(PintBoxSweepData)/sizeof(udword));
}

PintBoxSweepData* TestBase::GetRegisteredBoxSweeps() const
{
	return (PintBoxSweepData*)mBoxSweepData.GetEntries();
}

void TestBase::UnregisterAllBoxSweeps()
{
	mBoxSweepData.Reset();
}

/////

void TestBase::RegisterSphereSweep(const Sphere& sphere, const Point& dir, float max_dist)
{
	PintSphereSweepData* Data = (PintSphereSweepData*)mSphereSweepData.Reserve(sizeof(PintSphereSweepData)/sizeof(udword));
	Data->mSphere	= sphere;
	Data->mDir		= dir;
	Data->mMaxDist	= max_dist;
}

udword TestBase::GetNbRegisteredSphereSweeps() const
{
	return mSphereSweepData.GetNbEntries()/(sizeof(PintSphereSweepData)/sizeof(udword));
}

PintSphereSweepData* TestBase::GetRegisteredSphereSweeps() const
{
	return (PintSphereSweepData*)mSphereSweepData.GetEntries();
}

void TestBase::UnregisterAllSphereSweeps()
{
	mSphereSweepData.Reset();
}

/////

void TestBase::RegisterCapsuleSweep(const LSS& capsule, const Point& dir, float max_dist)
{
	PintCapsuleSweepData* Data = (PintCapsuleSweepData*)mCapsuleSweepData.Reserve(sizeof(PintCapsuleSweepData)/sizeof(udword));
	Data->mCapsule	= capsule;
	Data->mDir		= dir;
	Data->mMaxDist	= max_dist;
}

udword TestBase::GetNbRegisteredCapsuleSweeps() const
{
	return mCapsuleSweepData.GetNbEntries()/(sizeof(PintCapsuleSweepData)/sizeof(udword));
}

PintCapsuleSweepData* TestBase::GetRegisteredCapsuleSweeps() const
{
	return (PintCapsuleSweepData*)mCapsuleSweepData.GetEntries();
}

void TestBase::UnregisterAllCapsuleSweeps()
{
	mCapsuleSweepData.Reset();
}

/////

void TestBase::RegisterConvexSweep(const udword convex_object_index, PintShapeRenderer* renderer, const PR& pr, const Point& dir, float max_dist)
{
	PintConvexSweepData* Data = (PintConvexSweepData*)mConvexSweepData.Reserve(sizeof(PintConvexSweepData)/sizeof(udword));
	Data->mRenderer				= renderer;
	Data->mConvexObjectIndex	= convex_object_index;
	Data->mTransform			= pr;
	Data->mDir					= dir;
	Data->mMaxDist				= max_dist;
}

udword TestBase::GetNbRegisteredConvexSweeps() const
{
	return mConvexSweepData.GetNbEntries()/(sizeof(PintConvexSweepData)/sizeof(udword));
}

PintConvexSweepData* TestBase::GetRegisteredConvexSweeps() const
{
	return (PintConvexSweepData*)mConvexSweepData.GetEntries();
}

void TestBase::UnregisterAllConvexSweeps()
{
	mConvexSweepData.Reset();
}

/////

void TestBase::RegisterSphereOverlap(const Sphere& sphere)
{
	PintSphereOverlapData* Data = (PintSphereOverlapData*)mSphereOverlapData.Reserve(sizeof(PintSphereOverlapData)/sizeof(udword));
	Data->mSphere	= sphere;
}

udword TestBase::GetNbRegisteredSphereOverlaps() const
{
	return mSphereOverlapData.GetNbEntries()/(sizeof(PintSphereOverlapData)/sizeof(udword));
}

PintSphereOverlapData* TestBase::GetRegisteredSphereOverlaps() const
{
	return (PintSphereOverlapData*)mSphereOverlapData.GetEntries();
}

void TestBase::UnregisterAllSphereOverlaps()
{
	mSphereOverlapData.Reset();
}

/////

void TestBase::RegisterBoxOverlap(const OBB& box)
{
	PintBoxOverlapData* Data = (PintBoxOverlapData*)mBoxOverlapData.Reserve(sizeof(PintBoxOverlapData)/sizeof(udword));
	Data->mBox	= box;
}

udword TestBase::GetNbRegisteredBoxOverlaps() const
{
	return mBoxOverlapData.GetNbEntries()/(sizeof(PintBoxOverlapData)/sizeof(udword));
}

PintBoxOverlapData* TestBase::GetRegisteredBoxOverlaps() const
{
	return (PintBoxOverlapData*)mBoxOverlapData.GetEntries();
}

void TestBase::UnregisterAllBoxOverlaps()
{
	mBoxOverlapData.Reset();
}

/////

void TestBase::RegisterCapsuleOverlap(const LSS& capsule)
{
	PintCapsuleOverlapData* Data = (PintCapsuleOverlapData*)mCapsuleOverlapData.Reserve(sizeof(PintCapsuleOverlapData)/sizeof(udword));
	Data->mCapsule	= capsule;
}

udword TestBase::GetNbRegisteredCapsuleOverlaps() const
{
	return mCapsuleOverlapData.GetNbEntries()/(sizeof(PintCapsuleOverlapData)/sizeof(udword));
}

PintCapsuleOverlapData* TestBase::GetRegisteredCapsuleOverlaps() const
{
	return (PintCapsuleOverlapData*)mCapsuleOverlapData.GetEntries();
}

void TestBase::UnregisterAllCapsuleOverlaps()
{
	mCapsuleOverlapData.Reset();
}

/////

void TestBase::CommonRelease()
{
//	ObjectsManager::Release();

	if(mRepX)
	{
		ReleaseRepXContext(mRepX);
		mRepX = null;
	}

	mCurrentTime = 0.0f;
	mAABBs.Empty();
	ReleaseManagedSurfaces();
	mRaycastData.Empty();
	mPhantomData.Empty();
	mBoxSweepData.Empty();
	mSphereSweepData.Empty();
	mCapsuleSweepData.Empty();
	mConvexSweepData.Empty();
	mSphereOverlapData.Empty();
	mBoxOverlapData.Empty();
	mCapsuleOverlapData.Empty();
	mCameraManager.Release();
}

void TestBase::CommonUpdate()
{
}

void TestBase::CommonRender(PintRender&)
{
}

udword TestBase::Update(Pint& pint)
{
	return 0;
}

bool TestBase::ProfileUpdate()
{
	return false;
}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_EmptyScene = "Empty scene. Use this to measure the operating overhead of each engine.";

START_TEST(EmptyScene, CATEGORY_UNDEFINED, gDesc_EmptyScene)

	virtual bool EmptyScene::Setup(Pint& pint, const PintCaps& caps)
	{
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(EmptyScene)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_TestNewFeature = "For dev only. Experimental place to test new exposed features. Ignore it.";

START_TEST(TestNewFeature, CATEGORY_UNDEFINED, gDesc_TestNewFeature)

	virtual bool TestNewFeature::Setup(Pint& pint, const PintCaps& caps)
	{
		pint.TestNewFeature();
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(TestNewFeature)

///////////////////////////////////////////////////////////////////////////////

//PHYSICS_TEST_DYNAMIC(DynamicCompound, CATEGORY_UNDEFINED, null)
#ifdef REMOVED
	class DynamicCompound : public TestBase
	{
		public:
									DynamicCompound()		{								}
		virtual						~DynamicCompound()		{								}
		virtual	const char*			GetName()				{ return "DynamicCompound";		}
		virtual	TestCategory		GetCategory()	const	{ return CATEGORY_UNDEFINED;	}
		virtual	bool				Setup(Pint& pint, const PintCaps& caps);
		virtual	void				CommonUpdate();
		virtual	udword				Update(Pint& pint);
		virtual	bool				ProfileUpdate()			{ return false;					}

				PintObjectHandle	mShapes[3];
	}DynamicCompound;

bool DynamicCompound::Setup(Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportCompounds)
		return false;

//	const Point MainBodyExtents(3.0f, 1.0f, 3.0f);
	const Point MainBodyExtents(3.0f, 1.0f, 1.0f);
	const Point WingExtents(1.0f, 0.1f, 4.0f);

	PINT_SPHERE_CREATE SphereDesc;
	SphereDesc.mRadius		= 1.0f;
	SphereDesc.mRenderer	= CreateSphereRenderer(1.0f);

	PINT_BOX_CREATE BoxDesc[3];
	BoxDesc[0].mExtents		= MainBodyExtents;
	BoxDesc[1].mExtents		= WingExtents;
	BoxDesc[1].mLocalPos	= Point(MainBodyExtents.x + WingExtents.x, 0.0f, 0.0f);
	BoxDesc[2].mExtents		= WingExtents;
	BoxDesc[2].mLocalPos	= Point(-MainBodyExtents.x - WingExtents.x, 0.0f, 0.0f);
	BoxDesc[0].mRenderer	= CreateBoxRenderer(BoxDesc[0].mExtents);
	BoxDesc[1].mRenderer	= CreateBoxRenderer(BoxDesc[1].mExtents);
	BoxDesc[2].mRenderer	= CreateBoxRenderer(BoxDesc[2].mExtents);
	BoxDesc[0].mNext = &BoxDesc[1];
	BoxDesc[1].mNext = &BoxDesc[2];

	SphereDesc.mNext = &BoxDesc[1];

	PINT_OBJECT_CREATE ObjectDesc;
//	ObjectDesc.mShapes		= BoxDesc;
	ObjectDesc.mShapes		= &SphereDesc;
	ObjectDesc.mMass		= 1.0f;
	ObjectDesc.mPosition	= Point(0.0f, MainBodyExtents.y, 0.0f);
	PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
	ASSERT(Handle);

	udword NbShapes = pint.GetShapes(mShapes, Handle);
	ASSERT(NbShapes==3);
	return true;
}

void DynamicCompound::CommonUpdate()
{
	mCurrentTime += 1.0f/60.0f;
}

udword DynamicCompound::Update(Pint& pint)
{
	Matrix3x3 Rot;
	Rot.RotX(mCurrentTime*1.0f);
	const Quat Q = Rot;

/*	for(udword i=1;i<3;i++)
	{
		pint.SetLocalRot(mShapes[i], Q);
	}*/
	pint.SetLocalRot(mShapes[0], Q);
	return 0;
}
#endif

///////////////////////////////////////////////////////////////////////////////

void RegisterArrayOfRaycasts(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float max_dist, const Point& offset)
{
	test.UnregisterAllRaycasts();
	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			test.RegisterRaycast(Origin + offset, dir, max_dist);
		}
	}
}

bool GenerateArrayOfVerticalRaycasts(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist)
{
	const float Altitude = 30.0f;
	const Point Dir(0.0f, -1.0f, 0.0f);
	const Point Offset(0.0f, 0.0f, 0.0f);
	RegisterArrayOfRaycasts(test, nb_x, nb_y, Altitude, scale, scale, Dir, max_dist, Offset);

	test.mCreateDefaultEnvironment = false;
	return true;
}

void RegisterArrayOfBoxSweeps(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, const Point& extents, const Point& offset, float max_dist)
{
	test.UnregisterAllBoxSweeps();
	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			OBB Box;
			Box.mCenter		= Origin + offset;
			Box.mExtents	= extents;
			Box.mRot.Identity();

			test.RegisterBoxSweep(Box, dir, max_dist);
		}
	}
}

bool GenerateArrayOfVerticalBoxSweeps(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist)
{
	const float Altitude = 30.0f;
	const Point Dir(0.0f, -1.0f, 0.0f);
	const Point Offset(0.0f, 0.0f, 0.0f);
	const Point Extents(1.2f, 0.5f, 0.5f);
	RegisterArrayOfBoxSweeps(test, nb_x, nb_y, Altitude, scale, scale, Dir, Extents, Offset, max_dist);

	test.mCreateDefaultEnvironment = false;
	return true;
}

void RegisterArrayOfSphereSweeps(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float radius, const Point& offset, float max_dist)
{
	test.UnregisterAllSphereSweeps();
	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);

			test.RegisterSphereSweep(Sphere(Origin + offset, radius), dir, max_dist);
		}
	}
}

bool GenerateArrayOfVerticalSphereSweeps(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist)
{
	const float Altitude = 30.0f;
	const Point Dir(0.0f, -1.0f, 0.0f);
	const Point Offset(0.0f, 0.0f, 0.0f);
	const float SphereRadius = 0.75f;
	RegisterArrayOfSphereSweeps(test, nb_x, nb_y, Altitude, scale, scale, Dir, SphereRadius, Offset, max_dist);

	test.mCreateDefaultEnvironment = false;
	return true;
}

void RegisterArrayOfCapsuleSweeps(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float radius, float half_height, const Point& offset, float max_dist)
{
	test.UnregisterAllCapsuleSweeps();
	const float OneOverNbX = 1.0f / float(nb_x-1);
	const float OneOverNbY = 1.0f / float(nb_y-1);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(offset.x + CoeffX * scale_x, offset.y + altitude, offset.z + CoeffY * scale_y);
			const Point P0 = Origin + Point(half_height, 0.0f, 0.0f);
			const Point P1 = Origin - Point(half_height, 0.0f, 0.0f);

			test.RegisterCapsuleSweep(LSS(Segment(P0, P1), radius), dir, max_dist);
		}
	}
}

bool GenerateArrayOfVerticalCapsuleSweeps(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist)
{
	const float Altitude = 30.0f;
	const Point Dir(0.0f, -1.0f, 0.0f);
	const Point Offset(0.0f, 0.0f, 0.0f);
	const float CapsuleRadius = 0.4f;
	const float HalfHeight = 1.8f;
	RegisterArrayOfCapsuleSweeps(test, nb_x, nb_y, Altitude, scale, scale, Dir, CapsuleRadius, HalfHeight, Offset, max_dist);

	test.mCreateDefaultEnvironment = false;
	return true;
}

void UpdateBoxSweeps(TestBase& test, float angle)
{
	//###TOFIX
//	Matrix3x3 Rot;
//	Rot.RotZ(angle);

		Matrix3x3 RotZ;
		RotZ.RotZ(0.3f);
	//	RotZ.RotZ(angle*0.1f);
		RotZ.RotZ(angle);

		Matrix3x3 RotX;
	//	RotX.RotX(angle);
		RotX.RotX(angle*0.77f);

	//	Matrix3x3 RotY;
	//	RotY.RotY(angle);

		Matrix3x3 Rot = RotZ * RotX;
	//	Matrix3x3 Rot = RotX * RotZ;
	//	Matrix3x3 Rot = RotY * RotZ;


	udword Nb = test.GetNbRegisteredBoxSweeps();
	PintBoxSweepData* Data = test.GetRegisteredBoxSweeps();
	while(Nb--)
	{
		Data->mBox.mRot = Rot;
		Data++;
	}
}

void UpdateCapsuleSweeps(TestBase& test, float angle)
{
	Matrix3x3 Rot;
	Rot.RotZ(angle);

	udword Nb = test.GetNbRegisteredCapsuleSweeps();
	PintCapsuleSweepData* Data = test.GetRegisteredCapsuleSweeps();
	while(Nb--)
	{
		const Point Center = (Data->mCapsule.mP0 + Data->mCapsule.mP1)*0.5f;
		const float HalfHeight = Data->mCapsule.mP0.Distance(Data->mCapsule.mP1)*0.5f;
//		const Point Local0 = Data->mCapsule.mP0 - Center;
//		const Point Local1 = Data->mCapsule.mP1 - Center;
		const Point Local0(HalfHeight, 0.0f, 0.0f);
		const Point Local1(-HalfHeight, 0.0f, 0.0f);
		const Point NewLocal0 = Local0 * Rot;
		const Point NewLocal1 = Local1 * Rot;
		Data->mCapsule.mP0 = NewLocal0 + Center;
		Data->mCapsule.mP1 = NewLocal1 + Center;
		Data++;
	}
}

void UpdateConvexSweeps(TestBase& test, float angle)
{
		Matrix3x3 RotZ;
		RotZ.RotZ(0.3f);
	//	RotZ.RotZ(angle*0.1f);
		RotZ.RotZ(angle);

		Matrix3x3 RotX;
	//	RotX.RotX(angle);
		RotX.RotX(angle*0.77f);

	//	Matrix3x3 RotY;
	//	RotY.RotY(angle);

		Matrix3x3 Rot = RotZ * RotX;
	//	Matrix3x3 Rot = RotX * RotZ;
	//	Matrix3x3 Rot = RotY * RotZ;


	udword Nb = test.GetNbRegisteredConvexSweeps();
	PintConvexSweepData* Data = test.GetRegisteredConvexSweeps();
	while(Nb--)
	{
		Data->mTransform.mRot = Rot;
		Data++;
	}
}

udword DoBatchRaycasts(TestBase& test, Pint& pint, bool use_phantoms)
{
	const udword Nb = test.GetNbRegisteredRaycasts();
	const PintRaycastData* Data = test.GetRegisteredRaycasts();

	udword NbHits;
//	if(gRaycastClosest)
	if(gRaycastMode==0)
	{
		// Raycast closest
		PintRaycastHit* Dest = pint.mSQHelper->PrepareRaycastQuery(Nb, Data);

		if(use_phantoms)
		{
			const udword NbPhantoms = test.GetNbRegisteredPhantoms();
			ASSERT(NbPhantoms==Nb);
			void** PhantomData = test.GetRegisteredPhantoms();
			NbHits = pint.BatchRaycastsPhantom(Nb, Dest, Data, PhantomData);
		}
		else
		{
			NbHits = pint.BatchRaycasts(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
		}

		if(0)	// Save results
		{
			static bool firstTime = true;
			if(firstTime)
			{
				firstTime = false;
				FILE* fp = fopen("c:\\results.bin", "wb");
				if(fp)
				{
					fwrite(Dest, sizeof(PintRaycastHit), Nb, fp);
					fclose(fp);
				}
			}
		}
	}
	else if(gRaycastMode==1)
	{
		// Raycast any
		PintBooleanHit* Dest = pint.mSQHelper->PrepareRaycastAnyQuery(Nb, Data);

		ASSERT(!use_phantoms);
		NbHits = pint.BatchRaycastAny(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	else if(gRaycastMode==2)
	{
		// Raycast all
		PintOverlapObjectHit* Dest = pint.mSQHelper->PrepareRaycastAllQuery(Nb, Data);

		ASSERT(!use_phantoms);
		NbHits = pint.BatchRaycastAll(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	return NbHits;
}

udword DoBatchBoxSweeps(TestBase& test, Pint& pint)
{
	const udword Nb = test.GetNbRegisteredBoxSweeps();
	const PintBoxSweepData* Data = test.GetRegisteredBoxSweeps();

	PintRaycastHit* Dest = pint.mSQHelper->PrepareBoxSweepQuery(Nb, Data);

	return pint.BatchBoxSweeps(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
}

udword DoBatchSphereSweeps(TestBase& test, Pint& pint)
{
	const udword Nb = test.GetNbRegisteredSphereSweeps();
	const PintSphereSweepData* Data = test.GetRegisteredSphereSweeps();

	PintRaycastHit* Dest = pint.mSQHelper->PrepareSphereSweepQuery(Nb, Data);

	return pint.BatchSphereSweeps(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
}

udword DoBatchCapsuleSweeps(TestBase& test, Pint& pint)
{
	const udword Nb = test.GetNbRegisteredCapsuleSweeps();
	const PintCapsuleSweepData* Data = test.GetRegisteredCapsuleSweeps();

	PintRaycastHit* Dest = pint.mSQHelper->PrepareCapsuleSweepQuery(Nb, Data);

	return pint.BatchCapsuleSweeps(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
}

udword DoBatchConvexSweeps(TestBase& test, Pint& pint)
{
	const udword Nb = test.GetNbRegisteredConvexSweeps();
	const PintConvexSweepData* Data = test.GetRegisteredConvexSweeps();

	PintRaycastHit* Dest = pint.mSQHelper->PrepareConvexSweepQuery(Nb, Data);

	return pint.BatchConvexSweeps(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
}

udword DoBatchSphereOverlaps(TestBase& test, Pint& pint, BatchOverlapMode mode)
{
	const udword Nb = test.GetNbRegisteredSphereOverlaps();
	const PintSphereOverlapData* Data = test.GetRegisteredSphereOverlaps();

	if(mode==OVERLAP_ANY)
	{
		PintBooleanHit* Dest = pint.mSQHelper->PrepareSphereOverlapAnyQuery(Nb, Data);
		return pint.BatchSphereOverlapAny(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	else if(mode==OVERLAP_OBJECTS)
	{
		PintOverlapObjectHit* Dest = pint.mSQHelper->PrepareSphereOverlapObjectsQuery(Nb, Data);
		return pint.BatchSphereOverlapObjects(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	return 0;
}

udword DoBatchBoxOverlaps(TestBase& test, Pint& pint, BatchOverlapMode mode)
{
	const udword Nb = test.GetNbRegisteredBoxOverlaps();
	const PintBoxOverlapData* Data = test.GetRegisteredBoxOverlaps();

	if(mode==OVERLAP_ANY)
	{
		PintBooleanHit* Dest = pint.mSQHelper->PrepareBoxOverlapAnyQuery(Nb, Data);
		return pint.BatchBoxOverlapAny(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	else if(mode==OVERLAP_OBJECTS)
	{
		PintOverlapObjectHit* Dest = pint.mSQHelper->PrepareBoxOverlapObjectsQuery(Nb, Data);
		return pint.BatchBoxOverlapObjects(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	return 0;
}

udword DoBatchCapsuleOverlaps(TestBase& test, Pint& pint, BatchOverlapMode mode)
{
	const udword Nb = test.GetNbRegisteredCapsuleOverlaps();
	const PintCapsuleOverlapData* Data = test.GetRegisteredCapsuleOverlaps();

	if(mode==OVERLAP_ANY)
	{
		PintBooleanHit* Dest = pint.mSQHelper->PrepareCapsuleOverlapAnyQuery(Nb, Data);
		return pint.BatchCapsuleOverlapAny(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	else if(mode==OVERLAP_OBJECTS)
	{
		PintOverlapObjectHit* Dest = pint.mSQHelper->PrepareCapsuleOverlapObjectsQuery(Nb, Data);
		return pint.BatchCapsuleOverlapObjects(pint.mSQHelper->GetThreadContext(), Nb, Dest, Data);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////

/*
START_TEST(ProxyShapes, CATEGORY_UNDEFINED, null)

bool ProxyShapes::Setup(Pint& pint, const PintCaps& caps)
{
	const Point Extents(10.0f, 10.0f, 10.0f);
	const udword NbX = 32;
	const udword NbY = 32;
	const float Altitude = 10.0f;
	const float Scale = 20.0f;

	PINT_BOX_CREATE BoxDesc;
	BoxDesc.mExtents	= Extents;
	BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

	for(udword y=0;y<NbY;y++)
	{
		const float CoeffY = float(y) - float(NbY-1)*0.5f;
		for(udword x=0;x<NbX;x++)
		{
			const float CoeffX = float(x) - float(NbX-1)*0.5f;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition.x		= CoeffX * Scale;
			ObjectDesc.mPosition.y		= Altitude;
			ObjectDesc.mPosition.z		= CoeffY * Scale;
			CreatePintObject(pint, ObjectDesc);
		}
	}

	mCreateDefaultEnvironment = false;

	return true;
}
*/

///////////////////////////////////////////////////////////////////////////////

static bool LoadRays(const char* filename, TestBase& test, bool only_rays, bool no_processing)
{
	IceFile BinFile(filename);
	if(!BinFile.IsValid())
		return false;

	const udword NbRays = BinFile.LoadDword();

	const float Scale = gValveScale;
	udword NbIsRays = 0;
	udword NbIsSwept = 0;
	udword NbAligned = 0;
	udword NbAlignedUp = 0;
	udword NbAlignedDown = 0;
//	FILE* fp = fopen("d:\\rays.txt", "w");

	Matrix3x3 Idt;
	Idt.Identity();

#ifdef VALVE_ROTATE45
	Matrix3x3 Rot;
	Rot.RotX(45.0f * DEGTORAD);
#endif

	if(no_processing)
	{
		for(udword i=0;i<NbRays;i++)
		{
			Source1_Ray_t RayData;
			BinFile.LoadBuffer(&RayData, sizeof(Source1_Ray_t));

			const Point Origin(	RayData.m_Start.x+RayData.m_StartOffset.x,
								RayData.m_Start.y+RayData.m_StartOffset.y,
								RayData.m_Start.z+RayData.m_StartOffset.z);
			Point Dir(RayData.m_Delta.x, RayData.m_Delta.y, RayData.m_Delta.z);

			const float MaxDist = Dir.Magnitude();
			if(MaxDist!=0.0f)
			{
	//			fprintf(fp, "%f\n", MaxDist*Scale);
				Dir/=MaxDist;

				if(only_rays || RayData.m_IsRay)
				{
					test.RegisterRaycast(Origin, Dir, MaxDist);
				}
				else
				{
					const Point Extents(RayData.m_Extents.x, RayData.m_Extents.y, RayData.m_Extents.z);
					const OBB Box(Origin, Extents, Idt);
					test.RegisterBoxSweep(Box, Dir, MaxDist*Scale);
				}
			}
		}
	}
	else
	{
		for(udword i=0;i<NbRays;i++)
		{
			Source1_Ray_t RayData;
			BinFile.LoadBuffer(&RayData, sizeof(Source1_Ray_t));

			if(RayData.m_IsRay)
				NbIsRays++;
			if(RayData.m_IsSwept)
				NbIsSwept++;

			const Point Origin(	(RayData.m_Start.x+RayData.m_StartOffset.x)*Scale,
								(RayData.m_Start.z+RayData.m_StartOffset.z)*Scale,
								(RayData.m_Start.y+RayData.m_StartOffset.y)*Scale);
			Point Dir(RayData.m_Delta.x, RayData.m_Delta.z, RayData.m_Delta.y);

			const float MaxDist = Dir.Magnitude();
			if(MaxDist!=0.0f)
			{
	//			fprintf(fp, "%f\n", MaxDist*Scale);
				Dir/=MaxDist;

				if(Dir.x==0.0f && Dir.z==0.0f)
				{
					NbAligned++;
					if(Dir.y>0.0f)
						NbAlignedUp++;
					else
						NbAlignedDown++;
				}

				if(only_rays || RayData.m_IsRay)
				{
#ifdef VALVE_ROTATE45
					test.RegisterRaycast(Origin*Rot, Dir*Rot, MaxDist*Scale);
#else
					test.RegisterRaycast(Origin, Dir, MaxDist*Scale);
#endif
				}
				else
				{
					const Point Extents(RayData.m_Extents.x*Scale, RayData.m_Extents.z*Scale, RayData.m_Extents.y*Scale);
					const OBB Box(Origin, Extents, Idt);
					test.RegisterBoxSweep(Box, Dir, MaxDist*Scale);
				}
			}
		}
	}
//	fclose(fp);
	return true;
}

void LoadRaysFile(TestBase& test, const char* filename, bool only_rays, bool no_processing)
{
	ASSERT(filename);

	const char* File = FindPEELFile(filename);
	if(!File || !LoadRays(File, test, only_rays, no_processing))
		printf(_F("Failed to load '%s'\n", filename));

//	if(!LoadRays(_F("../build/%s", filename), test, only_rays, no_processing))
//		if(!LoadRays(_F("./%s", filename), test, only_rays, no_processing))
//			printf(_F("Failed to load '%s'\n", filename));
}

///////////////////////////////////////////////////////////////////////////////
