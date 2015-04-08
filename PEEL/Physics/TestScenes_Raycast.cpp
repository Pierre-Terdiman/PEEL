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

///////////////////////////////////////////////////////////////////////////////

#define START_SQ_RAYCAST_TEST_VS_MESH(name, category, desc)												\
	class name : public TestBase																		\
	{																									\
		public:																							\
								name()						{						}					\
		virtual					~name()						{						}					\
		virtual	const char*		GetName()			const	{ return #name;			}					\
		virtual	const char*		GetDescription()	const	{ return desc;			}					\
		virtual	TestCategory	GetCategory()		const	{ return category;		}					\
		virtual	bool			ProfileUpdate()				{ return true;			}					\
		virtual	udword			Update(Pint& pint)			{ return DoBatchRaycasts(*this, pint);	}	\
		virtual	bool			Setup(Pint& pint, const PintCaps& caps)									\
		{																								\
			if(!caps.mSupportRaycasts)																	\
				return false;																			\
			return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);								\
		}

///////////////////////////////////////////////////////////////////////////////

	class SceneRaycastVsShapes : public TestBase
	{
		public:
								SceneRaycastVsShapes()			{ mType = PINT_SHAPE_UNDEFINED;	mTypeData = 4;	mDynamic = false;	}
		virtual					~SceneRaycastVsShapes()			{																	}
		virtual	TestCategory	GetCategory()			const	{ return CATEGORY_RAYCAST;											}
		virtual	bool			ProfileUpdate()					{ return true;														}
		virtual	udword			Update(Pint& pint)				{ return DoBatchRaycasts(*this, pint);								}

		virtual void			GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.55f, -0.59f));
			desc.mCamera[1] = CameraPose(Point(37.45f, 11.64f, 32.89f), Point(-0.81f, -0.40f, 0.44f));
		}

		virtual bool			CommonSetup()
		{
			TestBase::CommonSetup();
			return GenerateArrayOfVerticalRaycasts(*this, 35.0f, 32, 32, gSQMaxDist);
		}

		virtual bool			Setup(Pint& pint, const PintCaps& caps)
		{
			if(!caps.mSupportRaycasts)
				return false;

			const udword NbX = 32;
			const udword NbY = 32;
			const float Altitude = 10.0f;
			const float Scale = 35.0f;
			const float Mass = mDynamic ? 1.0f : 0.0f;

			return GenerateArrayOfObjects(pint, caps, mType, mTypeData, NbX, NbY, Altitude, Scale, Mass);
		}

				PintShape		mType;
				udword			mTypeData;
				bool			mDynamic;
	};

#define START_SQ_RAYCAST_TEST_VS_SHAPES(name, desc, type, is_dynamic)									\
	class name : public SceneRaycastVsShapes															\
	{																									\
		public:																							\
								name()						{ mType = type;	mDynamic = is_dynamic;	}	\
		virtual					~name()						{										}	\
		virtual	const char*		GetName()			const	{ return #name;							}	\
		virtual	const char*		GetDescription()	const	{ return desc;							}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastOverhead = "Empty scene used to measure the operating overhead of 32*32 raycasts.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastOverhead, gDesc_SceneRaycastOverhead, PINT_SHAPE_UNDEFINED, false)
END_TEST(SceneRaycastOverhead)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticSpheres = "32*32 raycasts against 32*32 static spheres.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsStaticSpheres, gDesc_SceneRaycastVsStaticSpheres, PINT_SHAPE_SPHERE, false)
END_TEST(SceneRaycastVsStaticSpheres)

static const char* gDesc_SceneRaycastVsDynamicSpheres = "32*32 raycasts against 32*32 dynamic spheres.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsDynamicSpheres, gDesc_SceneRaycastVsDynamicSpheres, PINT_SHAPE_SPHERE, true)
END_TEST(SceneRaycastVsDynamicSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticCapsules = "32*32 raycasts against 32*32 static capsules.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsStaticCapsules, gDesc_SceneRaycastVsStaticCapsules, PINT_SHAPE_CAPSULE, false)
END_TEST(SceneRaycastVsStaticCapsules)

static const char* gDesc_SceneRaycastVsDynamicCapsules = "32*32 raycasts against 32*32 dynamic capsules.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsDynamicCapsules, gDesc_SceneRaycastVsDynamicCapsules, PINT_SHAPE_CAPSULE, true)
END_TEST(SceneRaycastVsDynamicCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticBoxes = "32*32 raycasts against 32*32 static boxes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsStaticBoxes, gDesc_SceneRaycastVsStaticBoxes, PINT_SHAPE_BOX, false)
END_TEST(SceneRaycastVsStaticBoxes)

static const char* gDesc_SceneRaycastVsDynamicBoxes = "32*32 raycasts against 32*32 dynamic boxes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsDynamicBoxes, gDesc_SceneRaycastVsDynamicBoxes, PINT_SHAPE_BOX, true)
END_TEST(SceneRaycastVsDynamicBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsBigStaticConvexes = "32*32 raycasts against 32*32 (big) static convexes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsBigStaticConvexes, gDesc_SceneRaycastVsBigStaticConvexes, PINT_SHAPE_CONVEX, false)
END_TEST(SceneRaycastVsBigStaticConvexes)

static const char* gDesc_SceneRaycastVsBigDynamicConvexes = "32*32 raycasts against 32*32 (big) dynamic convexes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsBigDynamicConvexes, gDesc_SceneRaycastVsBigDynamicConvexes, PINT_SHAPE_CONVEX, true)
	virtual bool	CommonSetup()
	{
		mTypeData = 1;
		return SceneRaycastVsShapes::CommonSetup();
	}
END_TEST(SceneRaycastVsBigDynamicConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsSmallStaticConvexes = "32*32 raycasts against 32*32 (small) static convexes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsSmallStaticConvexes, gDesc_SceneRaycastVsSmallStaticConvexes, PINT_SHAPE_CONVEX, false)
	virtual bool	CommonSetup()
	{
		mTypeData = 2;
		return SceneRaycastVsShapes::CommonSetup();
	}
END_TEST(SceneRaycastVsSmallStaticConvexes)

static const char* gDesc_SceneRaycastVsSmallDynamicConvexes = "32*32 raycasts against 32*32 (small) dynamic convexes.";
START_SQ_RAYCAST_TEST_VS_SHAPES(SceneRaycastVsSmallDynamicConvexes, gDesc_SceneRaycastVsSmallDynamicConvexes, PINT_SHAPE_CONVEX, true)
	virtual bool	CommonSetup()
	{
		mTypeData = 2;
		return SceneRaycastVsShapes::CommonSetup();
	}
END_TEST(SceneRaycastVsSmallDynamicConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PotPourri_StaticRaycasts = "4096 random raycasts against 4096 random static objects (sphere, box or capsule).";

START_SQ_RAYCAST_TEST(PotPourri_StaticRaycasts, CATEGORY_RAYCAST, gDesc_PotPourri_StaticRaycasts)

	virtual bool PotPourri_StaticRaycasts::CommonSetup()
	{
		TestBase::CommonSetup();
		return Setup_PotPourri_Raycasts(*this, 4096, 100.0f);
	}

	virtual bool PotPourri_StaticRaycasts::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 0.0f, 16, 16, 16);
	}

END_TEST(PotPourri_StaticRaycasts)

static const char* gDesc_PotPourri_StaticRaycasts2 = "20000 random raycasts against 4096 random static objects (sphere, box or capsule).";

START_SQ_RAYCAST_TEST(PotPourri_StaticRaycasts2, CATEGORY_RAYCAST, gDesc_PotPourri_StaticRaycasts2)

	virtual bool PotPourri_StaticRaycasts2::CommonSetup()
	{
		TestBase::CommonSetup();
		return Setup_PotPourri_Raycasts(*this, 20000, 100.0f);
	}

	virtual bool PotPourri_StaticRaycasts2::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 0.0f, 16, 16, 16);
	}

END_TEST(PotPourri_StaticRaycasts2)

static const char* gDesc_PotPourri_DynamicRaycasts = "4096 random raycasts against 4096 random dynamic objects (sphere, box or capsule).";

START_SQ_RAYCAST_TEST(PotPourri_DynamicRaycasts, CATEGORY_RAYCAST, gDesc_PotPourri_DynamicRaycasts)

	virtual bool PotPourri_DynamicRaycasts::CommonSetup()
	{
		TestBase::CommonSetup();
		return Setup_PotPourri_Raycasts(*this, 4096, 1.0f);
	}

	virtual bool PotPourri_DynamicRaycasts::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 1.0f, 16, 16, 16);
	}

END_TEST(PotPourri_DynamicRaycasts)

static const char* gDesc_PotPourri_DynamicRaycasts2 = "20000 random raycasts against 4096 random dynamic objects (sphere, box or capsule).";

START_SQ_RAYCAST_TEST(PotPourri_DynamicRaycasts2, CATEGORY_RAYCAST, gDesc_PotPourri_DynamicRaycasts2)

	virtual bool PotPourri_DynamicRaycasts2::CommonSetup()
	{
		TestBase::CommonSetup();
		return Setup_PotPourri_Raycasts(*this, 20000, 1.0f);
	}

	virtual bool PotPourri_DynamicRaycasts2::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 1.0f, 16, 16, 16);
	}

END_TEST(PotPourri_DynamicRaycasts2)

///////////////////////////////////////////////////////////////////////////////

static bool LoadMeshSurfaceRays(const char* filename, TestBase& test, Vertices& origins)
{
	IceFile BinFile(filename);
	if(!BinFile.IsValid())
		return false;

	udword Length;
	const Point* Data = (const Point*)BinFile.Load(Length);
	for(udword i=0;i<Length/sizeof(Point);i++)
	{
		origins.AddVertex(Data[i]);
	}
	return true;
}

static void LoadMeshSurfaceRays(TestBase& test, const char* filename, Vertices& origins)
{
	ASSERT(filename);

	const char* File = FindPEELFile(filename);
	if(!File || !LoadMeshSurfaceRays(File, test, origins))
		printf(_F("Failed to load '%s'\n", filename));

//	if(!LoadMeshSurfaceRays(_F("../build/%s", filename), test, origins))
//		if(!LoadMeshSurfaceRays(_F("./%s", filename), test, origins))
//			printf(_F("Failed to load '%s'\n", filename));
}

static const char* gDesc_SceneRaycastVsStaticMeshes_MeshSurface = "Mesh surface Visual Test.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_MeshSurface, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_MeshSurface)

	Vertices	mSurfaceRayOrigins;
	float		mAlpha;
	float		mPhi;
	float		mAlphaStep;
	float		mPhiStep;

	virtual	void SceneRaycastVsStaticMeshes_MeshSurface::CommonRelease()
	{
		mSurfaceRayOrigins.Empty();
		TestBase::CommonRelease();
	}

	virtual bool SceneRaycastVsStaticMeshes_MeshSurface::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");
//		LoadMeshesFromFile_(*this, "testzone.bin", null, false, 1, TESS_POLYHEDRAL);

		mCreateDefaultEnvironment = false;

		mAlpha = mPhi = 0.0f;
		mAlphaStep = 0.001f;
		mPhiStep = 0.002f;

		LoadMeshSurfaceRays(*this, "SurfaceOrigins.bin", mSurfaceRayOrigins);

		const udword nRays = mSurfaceRayOrigins.GetNbVertices();
		const Point* Origins = mSurfaceRayOrigins.GetVertices();
		BasicRandom Rnd(42);
		for(uword i=0;i<nRays;i++)
		{
			if(0)
			{
				RegisterRaycast(Origins[i], Point(0.0f, 0.0f, 0.0f), gSQMaxDist);
			}
			else
			{
				for(udword j=0;j<100;j++)
				{
					Point p; UnitRandomPt(p, Rnd); p*=0.1f;
					RegisterRaycast(Origins[i]+p, Point(0.0f, 0.0f, 0.0f), gSQMaxDist);
				}
			}
		}

		return true;
	}

	virtual void SceneRaycastVsStaticMeshes_MeshSurface::CommonUpdate()
	{
		TestBase::CommonUpdate();

		mAlpha += mAlphaStep;
		if (mAlpha > PI*2)
			mAlpha -= PI*2;
		mPhi += mPhiStep;
		if (mPhi > PI/4 || mPhi < -PI/4)
			mPhiStep = -mPhiStep;

		PintRaycastData* Rays = GetRegisteredRaycasts();
//		const udword nRays = mSurfaceRayOrigins.GetNbVertices();
		const udword nRays = GetNbRegisteredRaycasts();
		for(udword i=0; i<nRays; i++)
		{
			Point dir = Point(sinf(mAlpha), sinf(mPhi), cosf(mAlpha));
			dir.Normalize();
			Rays[i].mDir = dir;
		}
	}

END_TEST(SceneRaycastVsStaticMeshes_MeshSurface)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TestZone_ShortRays = "TestZone. Short rays.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TestZone_ShortRays, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TestZone_ShortRays)

	virtual bool SceneRaycastVsStaticMeshes_TestZone_ShortRays::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllRaycasts();

		//### refactor this
		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		const float Length = 0.1f;
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);

			Point Center, Normal;
			T->Center(V, Center);
			T->Normal(V, Normal);
			RegisterRaycast(Center+Normal*Length, -Normal, Length*2.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TestZone_ShortRays)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TestZone_LongRays = "TestZone. Long rays.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TestZone_LongRays, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TestZone_LongRays)

	virtual bool SceneRaycastVsStaticMeshes_TestZone_LongRays::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllRaycasts();

		//### refactor this
		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		const float Length = 0.1f;
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);

			Point Center, Normal;
			T->Center(V, Center);
			T->Normal(V, Normal);
//			RegisterRaycast(Center+Normal*Length, -Normal, Length*2.0f);
//			RegisterRaycast(Center+Normal*Length, -Normal, 10000.0f);
			RegisterRaycast(Center+Normal*Length, -Normal, MAX_FLOAT);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TestZone_LongRays)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TestZone_VerticalRays = "TestZone. Vertical rays.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TestZone_VerticalRays, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TestZone_VerticalRays)

	virtual bool SceneRaycastVsStaticMeshes_TestZone_VerticalRays::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllRaycasts();

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

	//	const udword NbX = 32;
	//	const udword NbY = 32;
		const udword NbX = 128;
		const udword NbY = 128;
		const float Altitude = 30.0f;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		RegisterArrayOfRaycasts(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, gSQMaxDist, Offset);

		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TestZone_VerticalRays)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_Archipelago1024 = "32*32 raycasts against the Archipelago mesh level.";

START_SQ_RAYCAST_TEST(SceneRaycastVsStaticMeshes_Archipelago1024, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_Archipelago1024)

	virtual bool SceneRaycastVsStaticMeshes_Archipelago1024::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneRaycastVsStaticMeshes_Archipelago1024::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
	//	const udword NbX = 128;
	//	const udword NbY = 128;
		const float Altitude = 30.0f;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		RegisterArrayOfRaycasts(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, gSQMaxDist, Offset);
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_Archipelago1024)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_Archipelago16384 = "128*128 raycasts against the Archipelago mesh level.";

START_SQ_RAYCAST_TEST(SceneRaycastVsStaticMeshes_Archipelago16384, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_Archipelago16384)

	virtual bool SceneRaycastVsStaticMeshes_Archipelago16384::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneRaycastVsStaticMeshes_Archipelago16384::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 128;
		const udword NbY = 128;
		const float Altitude = 30.0f;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		RegisterArrayOfRaycasts(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, gSQMaxDist, Offset);
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_Archipelago16384)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP1024 = "1024 raycasts against the Konoko Payne mesh level.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP1024, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP1024)

	virtual bool SceneRaycastVsStaticMeshes_KP1024::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const Point Origin(50.0f, 50.0f, 50.0f);

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
			RegisterRaycast(Origin, Dir, gSQMaxDist);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP1024)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP16384 = "16384 raycasts against the Konoko Payne mesh level.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP16384, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP16384)

	virtual bool SceneRaycastVsStaticMeshes_KP16384::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const Point Origin(50.0f, 50.0f, 50.0f);

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		for(udword i=0;i<16384;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
			RegisterRaycast(Origin, Dir, gSQMaxDist);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP16384)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP16384_2 = "16384 raycasts against the Konoko Payne mesh level (2).";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP16384_2, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP16384_2)

	virtual bool SceneRaycastVsStaticMeshes_KP16384_2::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const Point Origin(0.0f, 1000.0f, 0.0f);

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		for(udword i=0;i<16384;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
			RegisterRaycast(Origin, Dir, gSQMaxDist);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP16384_2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP16384_NoHit = "16384 very short 'sensor' raycasts against the Konoko Payne mesh level. No hits.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP16384_NoHit, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP16384_NoHit)

	virtual bool SceneRaycastVsStaticMeshes_KP16384_NoHit::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const float MaxDist = 10.0f;

		const Point Origin(50.0f, 50.0f, 50.0f);

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		for(udword i=0;i<16384;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
			RegisterRaycast(Origin, Dir, MaxDist);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP16384_NoHit)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP_Single = "Single long raycasts against the Konoko Payne mesh level.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP_Single, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP_Single)

	virtual bool SceneRaycastVsStaticMeshes_KP_Single::CommonSetup()
	{
		TestBase::CommonSetup();

		const float Scale = 0.1f;	// Should be similar to BenchMark_RaycastKP in PhysX
		LoadMeshesFromFile_(*this, "kp.bin", &Scale);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		const Point Origin(34.7202f, 6.0501f, -2.9846f);
		for(udword i=0;i<1;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
	//		Dir.x = fabsf(Dir.x);
	//		Dir.y = fabsf(Dir.y);
	//		Dir.z = fabsf(Dir.z);
			RegisterRaycast(Origin, Dir, 5000.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP_Single)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP2_Long = "2048 long raycasts against the Konoko Payne mesh level.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP2_Long, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP2_Long)

	virtual bool SceneRaycastVsStaticMeshes_KP2_Long::CommonSetup()
	{
		TestBase::CommonSetup();

		const float Scale = 0.1f;	// Should be similar to BenchMark_RaycastKP in PhysX
		LoadMeshesFromFile_(*this, "kp.bin", &Scale);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);
		const Point Origin(34.7202f, 6.0501f, -2.9846f);
		for(udword i=0;i<2048;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);
	//		Dir.x = fabsf(Dir.x);
	//		Dir.y = fabsf(Dir.y);
	//		Dir.z = fabsf(Dir.z);
			RegisterRaycast(Origin, Dir, 5000.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP2_Long)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP2_Short = "2048 short raycasts against the Konoko Payne mesh level.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_KP2_Short, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP2_Short)

	virtual bool SceneRaycastVsStaticMeshes_KP2_Short::CommonSetup()
	{
		TestBase::CommonSetup();

		const float Scale = 0.1f;	// Should be similar to BenchMark_RaycastKP in PhysX
		LoadMeshesFromFile_(*this, "kp.bin", &Scale);

		mCreateDefaultEnvironment = false;

		BasicRandom Rnd(42);

		// This version uses very short raycasts from just before the impact point
		{
			udword FileSize;
			const char* Filename = GetFile("SceneRaycastVsStaticMeshes_KP2.bin", FileSize);
			if(FileSize)
			{
				ubyte* Buffer = (ubyte*)ICE_ALLOC_TMP(FileSize);

				FILE* fp = fopen(Filename, "rb");
				if(fp)
				{
					fread(Buffer, FileSize, 1, fp);
					fclose(fp);

					UnregisterAllRaycasts();
					udword NbRays = FileSize/sizeof(PintRaycastHit);
					const PintRaycastHit* Hits = (const PintRaycastHit*)Buffer;
					for(udword i=0;i<NbRays;i++)
					{
						Point Dir;
						UnitRandomPt(Dir, Rnd);

						if(Hits[i].mObject)
							RegisterRaycast(Hits[i].mImpact - Dir*0.1f, Dir, 0.2f);
					}
				}
				ICE_FREE(Buffer);
			}
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_KP2_Short)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_KP_Phantom = "2048 short raycasts against the Konoko Payne mesh level, using phantom objects.";

START_SQ_TEST(SceneRaycastVsStaticMeshes_KP_Phantom, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP_Phantom)

	virtual bool SceneRaycastVsStaticMeshes_KP_Phantom::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SceneRaycastVsStaticMeshes_KP_Phantom::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		if(0)
		{
			UnregisterAllRaycasts();
			BasicRandom Rnd(42);
			const Point Origin(347.202f, 60.501f, -29.846f);
			for(udword i=0;i<2048;i++)
			{
				Point Dir;
				UnitRandomPt(Dir, Rnd);
		//		Dir.x = fabsf(Dir.x);
		//		Dir.y = fabsf(Dir.y);
		//		Dir.z = fabsf(Dir.z);
				RegisterRaycast(Origin, Dir, 5000.0f);
			}
		}
		else
		{
			// This version uses very short raycasts from just before the impact point
			BasicRandom Rnd(42);
			udword FileSize;
			const char* Filename = GetFile("SceneRaycastVsStaticMeshes_KP_Phantom.bin", FileSize);
			if(FileSize)
			{
				ubyte* Buffer = (ubyte*)ICE_ALLOC_TMP(FileSize);

				FILE* fp = fopen(Filename, "rb");
				if(fp)
				{
					fread(Buffer, FileSize, 1, fp);
					fclose(fp);

					UnregisterAllRaycasts();
					udword NbRays = FileSize/sizeof(PintRaycastHit);
					const PintRaycastHit* Hits = (const PintRaycastHit*)Buffer;
					for(udword i=0;i<NbRays;i++)
					{
						if(Hits[i].mObject)
						{
							void* PhantomHandle = null;
							if(caps.mSupportPhantoms)
							{
								AABB PhantomBox;
								PhantomBox.SetCenterExtents(Hits[i].mImpact, Point(1.0f, 1.0f, 1.0f));

								PhantomHandle = pint.CreatePhantom(PhantomBox);
							}

							Point Dir;
							UnitRandomPt(Dir, Rnd);

							RegisterRaycast(Hits[i].mImpact - Dir*0.1f, Dir, 0.2f, PhantomHandle);
						}
					}
				}
				ICE_FREE(Buffer);
			}
		}
		return true;
	}

	virtual udword SceneRaycastVsStaticMeshes_KP_Phantom::Update(Pint& pint)
	{
		PintCaps Caps;
		pint.GetCaps(Caps);

		return DoBatchRaycasts(*this, pint, Caps.mSupportPhantoms);
	}

END_TEST(SceneRaycastVsStaticMeshes_KP_Phantom)

///////////////////////////////////////////////////////////////////////////////

/*static const char* gDesc_SceneRaycastVsStaticMeshes_KP2_Phantom = "2048 short raycasts against the Konoko Payne mesh level, using phantom objects.";

PHYSICS_TEST_DYNAMIC(SceneRaycastVsStaticMeshes_KP2_Phantom, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_KP2_Phantom)

bool SceneRaycastVsStaticMeshes_KP2_Phantom::Setup(Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportMeshes || !caps.mSupportRaycasts || !caps.mSupportPhantoms)
		return false;

	const float Scale = 0.1f;	// Should be similar to BenchMark_RaycastKP in PhysX
	LoadMeshesFromFile(pint, *this, "kp.bin", &Scale);

	BasicRandom Rnd(42);

	// This version uses very short raycasts from just before the impact point
	{
		udword FileSize;
		const char* Filename = GetFile("SceneRaycastVsStaticMeshes_KP2.bin", FileSize);
		if(FileSize)
		{
			ubyte* Buffer = (ubyte*)ICE_ALLOC_TMP(FileSize);

			FILE* fp = fopen(Filename, "rb");
			if(fp)
			{
				fread(Buffer, FileSize, 1, fp);
				fclose(fp);

				UnregisterAllRaycasts();
				udword NbRays = FileSize/sizeof(PintRaycastHit);
				const PintRaycastHit* Hits = (const PintRaycastHit*)Buffer;
				for(udword i=0;i<NbRays;i++)
				{
					if(Hits[i].mObject)
					{
						AABB PhantomBox;
						PhantomBox.SetCenterExtents(Hits[i].mImpact, Point(1.0f, 1.0f, 1.0f));

						void* PhantomHandle = pint.CreatePhantom(PhantomBox);

						Point Dir;
						UnitRandomPt(Dir, Rnd);

						RegisterRaycast(Hits[i].mImpact - Dir*0.1f, Dir, PhantomHandle);
					}
				}
			}
			ICE_FREE(Buffer);
		}
	}

	mCreateDefaultEnvironment = false;
	return true;
}

void SceneRaycastVsStaticMeshes_KP2_Phantom::CommonUpdate()
{
}

udword SceneRaycastVsStaticMeshes_KP2_Phantom::Update(Pint& pint)
{
	return DoBatchRaycasts(*this, pint, 0.2f, true);
}*/

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_Terrain_Long = "128*128 long raycasts against the tesselated terrain.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_Terrain_Long, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_Terrain_Long)

	virtual bool SceneRaycastVsStaticMeshes_Terrain_Long::CommonSetup()
	{
		TestBase::CommonSetup();

	//	LoadMeshesFromFile_(*this, "Terrain.bin");
		LoadMeshesFromFile_(*this, "Terrain.bin", null, false, 1);

		mCreateDefaultEnvironment = false;

		const Point Dir(0.0f, -1.0f, 0.0f);
		// This version uses long raycasts from high above
		{
			Point Offset, Extents;
			GetGlobalBounds(Offset, Extents);

			const udword NbX = 128;
			const udword NbY = 128;
			const float Altitude = 500.0f;
			const float ScaleX = Extents.x - 1.0f;
			const float ScaleY = Extents.z - 1.0f;
			RegisterArrayOfRaycasts(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, gSQMaxDist, Offset);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_Terrain_Long)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_Terrain_Short = "32*32 short raycasts against the tesselated terrain.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_Terrain_Short, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_Terrain_Short)

	virtual bool SceneRaycastVsStaticMeshes_Terrain_Short::CommonSetup()
	{
		TestBase::CommonSetup();

	//	LoadMeshesFromFile_(*this, "Terrain.bin");
		LoadMeshesFromFile_(*this, "Terrain.bin", null, false, 1);

		mCreateDefaultEnvironment = false;

		const Point Dir(0.0f, -1.0f, 0.0f);
		// This version uses very short raycasts from just above the impact point
		{
			udword FileSize;
			const char* Filename = GetFile("SceneRaycastVsStaticMeshes_Terrain.bin", FileSize);
			if(FileSize)
			{
				ubyte* Buffer = (ubyte*)ICE_ALLOC_TMP(FileSize);

				FILE* fp = fopen(Filename, "rb");
				if(fp)
				{
					fread(Buffer, FileSize, 1, fp);
					fclose(fp);

					UnregisterAllRaycasts();
					udword NbRays = FileSize/sizeof(PintRaycastHit);
					const PintRaycastHit* Hits = (const PintRaycastHit*)Buffer;
					for(udword i=0;i<NbRays;i++)
					{
						if(Hits[i].mObject)
							RegisterRaycast(Hits[i].mImpact - Dir, Dir, 2.0f);
					}
				}
				ICE_FREE(Buffer);
			}
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_Terrain_Short)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TessBunny_NodeSorting = "Node sorting test. 2048 raycasts are fired from one of the 8 canonical directions in sequence. In theory the \
cost for each of the 8 directions should be roughly the same. In practice, engines without node sorting show a very high variation here. The direction changes every 32 frames.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TessBunny_NodeSorting, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TessBunny_NodeSorting)

	udword	mNbFrames;
	udword	mIndex;

	virtual bool SceneRaycastVsStaticMeshes_TessBunny_NodeSorting::CommonSetup()
	{
		mNbFrames = 0;
		mIndex = 0;

		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();

		for(udword i=0;i<2048;i++)
			RegisterRaycast(Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f), 2000.0f);
		return true;
	}

	virtual void SceneRaycastVsStaticMeshes_TessBunny_NodeSorting::CommonUpdate()
	{
		TestBase::CommonUpdate();
		if(mNbFrames==0)
		{
			const udword Index = mIndex;
			Point Dir;
			if(Index==0)
				Dir = Point(1.0f, 1.0f, 1.0f);
			else if(Index==1)
				Dir = Point(1.0f, 1.0f, -1.0f);
			else if(Index==2)
				Dir = Point(1.0f, -1.0f, 1.0f);
			else if(Index==3)
				Dir = Point(1.0f, -1.0f, -1.0f);
			else if(Index==4)
				Dir = Point(-1.0f, 1.0f, 1.0f);
			else if(Index==5)
				Dir = Point(-1.0f, 1.0f, -1.0f);
			else if(Index==6)
				Dir = Point(-1.0f, -1.0f, 1.0f);
			else if(Index==7)
				Dir = Point(-1.0f, -1.0f, -1.0f);

			Dir.Normalize();

			const float Distance = 20.0f;

			Point Center, Extents;
			GetGlobalBounds(Center, Extents);

			udword Nb = GetNbRegisteredRaycasts();
			PintRaycastData* Data = GetRegisteredRaycasts();
			while(Nb--)
			{
				Data->mOrigin = Center+Dir*Distance;
				Data->mDir = -Dir;
				Data++;
			}

			mIndex++;
			if(mIndex==8)
				mIndex=0;
		}
		mNbFrames++;
		if(mNbFrames==32)
			mNbFrames=0;
	}

END_TEST(SceneRaycastVsStaticMeshes_TessBunny_NodeSorting)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TessBunny16384 = "16384 radial raycasts against the tesselated bunny.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TessBunny16384, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TessBunny16384)

	virtual bool SceneRaycastVsStaticMeshes_TessBunny16384::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);

		for(udword i=0;i<16384;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Center + Dir * 20.0f;

			RegisterRaycast(Origin, -Dir, 20.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TessBunny16384)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TessBunny16384_2 = "16384 radial raycasts against the tesselated bunny. Same as before but with a larger ray length. In theory the \
ray length should not have an impact on performance. In practice however, some engines become much slower when the ray becomes larger.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TessBunny16384_2, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TessBunny16384_2)

	virtual bool SceneRaycastVsStaticMeshes_TessBunny16384_2::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();
		BasicRandom Rnd(42);

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);

		for(udword i=0;i<16384;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Center + Dir * 20.0f;

			RegisterRaycast(Origin, -Dir, 20000.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TessBunny16384_2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneRaycastVsStaticMeshes_TessBunnyShort = "Short raycasts against the tesselated bunny. There is one raycast per triangle. \
Each ray starts a little bit above the triangle. Ray direction is the opposite of the triangle normal.";

START_SQ_RAYCAST_TEST_VS_MESH(SceneRaycastVsStaticMeshes_TessBunnyShort, CATEGORY_RAYCAST, gDesc_SceneRaycastVsStaticMeshes_TessBunnyShort)

	virtual bool SceneRaycastVsStaticMeshes_TessBunnyShort::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 2);

		mCreateDefaultEnvironment = false;

		UnregisterAllRaycasts();

		IndexedSurface* IS = GetFirstSurface();
		const Point* V = IS->GetVerts();
		const float Length = 0.1f;
		for(udword i=0;i<IS->GetNbFaces();i++)
		{
			const IndexedTriangle* T = IS->GetFace(i);

			Point Center, Normal;
			T->Center(V, Center);
			T->Normal(V, Normal);
			RegisterRaycast(Center+Normal*Length, -Normal, Length*2.0f);
		}
		return true;
	}

END_TEST(SceneRaycastVsStaticMeshes_TessBunnyShort)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_KP_RT = "Konoko Payne mesh level. Raytracing test.";

START_SQ_RAYCAST_TEST_VS_MESH(KP_RT, CATEGORY_RAYCAST, gDesc_KP_RT)

	virtual bool KP_RT::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "default_peel_view_rays.bin", true, true);
		LoadMeshesFromFile_(*this, "kp.bin");
//		LoadMeshesFromFile_(*this, "kp.bin", null, false, 1, TESS_POLYHEDRAL);
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(KP_RT)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Bunny_RT = "Bunny. Raytracing test.";

START_SQ_RAYCAST_TEST_VS_MESH(Bunny_RT, CATEGORY_RAYCAST, gDesc_Bunny_RT)

	virtual bool Bunny_RT::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "bunny_rays.bin", true, true);
//		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 2);
		LoadMeshesFromFile_(*this, "Bunny.bin");
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(Bunny_RT)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Terrain_RT = "Terrain tiles. Raytracing test.";

START_SQ_RAYCAST_TEST_VS_MESH(Terrain_RT, CATEGORY_RAYCAST, gDesc_Terrain_RT)

	virtual bool Terrain_RT::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "default_peel_view_rays.bin", true, true);
		LoadMeshesFromFile_(*this, "Terrain.bin");
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(Terrain_RT)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_TestZone_RT = "TestZone (tess). Raytracing test.";

START_SQ_RAYCAST_TEST_VS_MESH(TestZone_RT, CATEGORY_RAYCAST, gDesc_TestZone_RT)

	virtual bool TestZone_RT::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "testzone_rays_5.bin", true, true);
//		LoadMeshesFromFile_(*this, "testzone.bin");
		LoadMeshesFromFile_(*this, "testzone.bin", null, false, 1, TESS_POLYHEDRAL);
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(TestZone_RT)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_TestZone_RT2 = "TestZone (tess). Raytracing test 2.";

START_SQ_RAYCAST_TEST_VS_MESH(TestZone_RT2, CATEGORY_RAYCAST, gDesc_TestZone_RT2)

	virtual bool TestZone_RT2::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "testzone_rays_4.bin", true, true);
//		LoadMeshesFromFile_(*this, "testzone.bin");
		LoadMeshesFromFile_(*this, "testzone.bin", null, false, 1, TESS_POLYHEDRAL);
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(TestZone_RT2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_TestZone_RT3 = "TestZone. Raytracing ride test.";

START_SQ_RAYCAST_TEST_VS_MESH(TestZone_RT3, CATEGORY_RAYCAST, gDesc_TestZone_RT3)

	virtual bool TestZone_RT3::CommonSetup()
	{
		TestBase::CommonSetup();

		mCameraManager.LoadCameraData("testzone_camera_data.bin");

//		LoadRaysFile(*this, "testzone_rays_2.bin", true, true);
		LoadMeshesFromFile_(*this, "testzone.bin");
//		LoadMeshesFromFile_(*this, "testzone.bin", null, false, 1, TESS_POLYHEDRAL);
		mCreateDefaultEnvironment = false;

		for(udword i=0;i<128*128;i++)
			RegisterRaycast(Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f), 0.0f);

		return true;
	}

	virtual void TestZone_RT3::CommonUpdate()
	{
		mCameraManager.UpdateCameraPose();
		mCameraManager.GenerateRays(GetRegisteredRaycasts(), 128, 1000.0f);
	}

END_TEST(TestZone_RT3)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Terrain_RT2 = "Terrain. Raytracing ride test.";

START_SQ_RAYCAST_TEST_VS_MESH(Terrain_RT2, CATEGORY_RAYCAST, gDesc_Terrain_RT2)

	virtual bool Terrain_RT2::CommonSetup()
	{
		TestBase::CommonSetup();

		mCameraManager.LoadCameraData("terrain_camera_data.bin");

		LoadMeshesFromFile_(*this, "terrain.bin");
		mCreateDefaultEnvironment = false;

		for(udword i=0;i<128*128;i++)
			RegisterRaycast(Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f), 0.0f);

		return true;
	}

	virtual void Terrain_RT2::CommonUpdate()
	{
		mCameraManager.UpdateCameraPose();
		mCameraManager.GenerateRays(GetRegisteredRaycasts(), 128, 10000.0f);
	}

END_TEST(Terrain_RT2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_KP_RT2 = "KP. Raytracing ride test.";

START_SQ_RAYCAST_TEST_VS_MESH(KP_RT2, CATEGORY_RAYCAST, gDesc_KP_RT2)

	virtual bool KP_RT2::CommonSetup()
	{
		TestBase::CommonSetup();

		mCameraManager.LoadCameraData("kp_camera_data.bin");

		LoadMeshesFromFile_(*this, "kp.bin");
		mCreateDefaultEnvironment = false;

		for(udword i=0;i<128*128;i++)
			RegisterRaycast(Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f), 0.0f);

		return true;
	}

	virtual void KP_RT2::CommonUpdate()
	{
		mCameraManager.UpdateCameraPose();
		mCameraManager.GenerateRays(GetRegisteredRaycasts(), 128, 10000.0f);
	}

END_TEST(KP_RT2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_InsideRays_TestZone = "TestZone. Inside rays.";

START_SQ_RAYCAST_TEST_VS_MESH(InsideRays_TestZone, CATEGORY_RAYCAST, gDesc_InsideRays_TestZone)

	virtual bool InsideRays_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllRaycasts();

		{
			// Shooting rays from inside the tower
			BasicRandom Rnd(42);
			for(udword i=0;i<4096;i++)
			{
				const float Coeff = TWOPI*float(i)/4096.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

//				Point D(x, 0.0f, z);
//				D.Normalize();
				Point D;
				UnitRandomPt(D, Rnd);

//				RegisterRaycast(Point(41.78f, 5.64f, -69.18f), D, 1000.0f);
				RegisterRaycast(Point(-15.08f, 5.20f, -58.01f), D, 1000.0f);
//				RegisterRaycast(Point(0.0f, 100.0f, 0.0f), Point(0.0f, 1.0f, 0.0f), 1000.0f);
			}
		}
		return true;
	}

END_TEST(InsideRays_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_OutsideRays_TestZone = "TestZone. Outside rays.";

START_SQ_RAYCAST_TEST_VS_MESH(OutsideRays_TestZone, CATEGORY_RAYCAST, gDesc_OutsideRays_TestZone)

	virtual bool OutsideRays_TestZone::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllRaycasts();

		{
			// Radial rays from the center, roughly
			for(udword i=0;i<4096;i++)
			{
				const float Coeff = TWOPI*float(i)/4096.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterRaycast(Point(8.36f, -4.22f, -28.18f), D, 1000.0f);
//				RegisterRaycast(Point(8.36f, -4.22f, -28.18f), D, 2.0f);
			}
		}
		return true;
	}

END_TEST(OutsideRays_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SpecularBulletRays = "Specular level. Bullet rays.";

START_SQ_RAYCAST_TEST(SpecularBulletRays, CATEGORY_RAYCAST, gDesc_SpecularBulletRays)

	virtual bool SpecularBulletRays::IsPrivate()	const
	{
		return true;
	}

	virtual bool SpecularBulletRays::CommonSetup()
	{
		TestBase::CommonSetup();
//		mRepX = CreateRepXContext("specular_raycast_meshes.repx", 1.0f, false);
		mRepX = CreateRepXContext("specular_raycast_meshes_statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;

		if(1)
		{
			UnregisterAllRaycasts();
			for(udword i=0;i<4096;i++)
			{
				const float Coeff = TWOPI*float(i)/4096.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterRaycast(Point(0.0f, 4.0f, 0.0f), D, 10000.0f);
			}
		}
		return true;
	}

	virtual bool SpecularBulletRays::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts || !caps.mSupportMeshes)
			return false;
		return AddToPint(pint, mRepX);
	}

END_TEST(SpecularBulletRays)

///////////////////////////////////////////////////////////////////////////////

#define IMPLEMENT_VALVE_RAYCAST_TEST(name, desc, filename)					\
START_SQ_RAYCAST_TEST(name, CATEGORY_RAYCAST, desc)							\
																			\
	virtual bool name::IsPrivate()	const									\
	{																		\
		return true;														\
	}																		\
																			\
	virtual bool name::CommonSetup()										\
	{																		\
		TestBase::CommonSetup();											\
		LoadRaysFile(*this, filename, true);								\
		mRepX = CreateRepXContext("c5m4_quarter2_Statics.repx", gValveScale, true);	\
		mCreateDefaultEnvironment = false;									\
		return true;														\
	}																		\
																			\
	virtual bool name::Setup(Pint& pint, const PintCaps& caps)				\
	{																		\
		if(!caps.mSupportMeshes || !caps.mSupportRaycasts)					\
			return false;													\
		return AddToPint(pint, mRepX);										\
	}																		\
																			\
END_TEST(name)

#define IMPLEMENT_VALVE_RAYCAST_TEST_OLD(name, desc, filename)				\
START_SQ_RAYCAST_TEST_VS_MESH(name, CATEGORY_RAYCAST, desc)					\
																			\
	virtual bool name::IsPrivate()	const									\
	{																		\
		return true;														\
	}																		\
																			\
	virtual bool name::CommonSetup()										\
	{																		\
		TestBase::CommonSetup();											\
		LoadRaysFile(*this, filename, true);								\
		LoadRepXFile_Obsolete(*this, "c5m4_quarter2_Statics.repx", gValveScale, true);\
		mCreateDefaultEnvironment = false;									\
		return true;														\
	}																		\
																			\
END_TEST(name)

static const char* gDesc_ValveRaycastsOnlyRays = "Valve level, raycasts 'only rays'.";
IMPLEMENT_VALVE_RAYCAST_TEST(ValveRaycastsOnlyRays, gDesc_ValveRaycastsOnlyRays, "rays(only rays).bin")
IMPLEMENT_VALVE_RAYCAST_TEST_OLD(ValveRaycastsOnlyRays_Old, gDesc_ValveRaycastsOnlyRays, "rays(only rays).bin")

static const char* gDesc_ValveRaycastsMidGame1 = "Valve level, raycasts 'MidGame1'.";
IMPLEMENT_VALVE_RAYCAST_TEST(ValveRaycastsMidGame1, gDesc_ValveRaycastsMidGame1, "l4d2_rays(midgame1).bin")
IMPLEMENT_VALVE_RAYCAST_TEST_OLD(ValveRaycastsMidGame1_Old, gDesc_ValveRaycastsMidGame1, "l4d2_rays(midgame1).bin")

static const char* gDesc_ValveRaycastsMidGame2 = "Valve level, raycasts 'MidGame2'.";
IMPLEMENT_VALVE_RAYCAST_TEST(ValveRaycastsMidGame2, gDesc_ValveRaycastsMidGame2, "l4d2_rays(midgame2).bin")
IMPLEMENT_VALVE_RAYCAST_TEST_OLD(ValveRaycastsMidGame2_Old, gDesc_ValveRaycastsMidGame2, "l4d2_rays(midgame2).bin")

static const char* gDesc_ValveRaycastsMidGame3 = "Valve level, raycasts 'MidGame3'.";
IMPLEMENT_VALVE_RAYCAST_TEST(ValveRaycastsMidGame3, gDesc_ValveRaycastsMidGame3, "l4d2_rays(midgame3).bin")
IMPLEMENT_VALVE_RAYCAST_TEST_OLD(ValveRaycastsMidGame3_Old, gDesc_ValveRaycastsMidGame3, "l4d2_rays(midgame3).bin")

static const char* gDesc_ValveRaycastsMidGame4 = "Valve level, raycasts 'MidGame4'.";
IMPLEMENT_VALVE_RAYCAST_TEST(ValveRaycastsMidGame4, gDesc_ValveRaycastsMidGame4, "l4d2_rays(midgame4).bin")
IMPLEMENT_VALVE_RAYCAST_TEST_OLD(ValveRaycastsMidGame4_Old, gDesc_ValveRaycastsMidGame4, "l4d2_rays(midgame4).bin")

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PlanetsideBulletRays = "512*4 bullet rays against the Planetside level.";

START_SQ_RAYCAST_TEST(PlanetsideBulletRays, CATEGORY_RAYCAST, gDesc_PlanetsideBulletRays)

	virtual bool PlanetsideBulletRays::IsPrivate()	const
	{
		return true;
	}

	virtual void PlanetsideBulletRays::GetSceneParams(PINT_WORLD_CREATE& desc)
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
		for(udword j=0;j<512;j++)
		{
			const float angle = (float(j)/512.0f)*3.14159f*2.0f;
			const float s = sinf(angle);
			const float c = cosf(angle);
			Point Dir(s, 0.1f*(UnitRandomFloat()-0.5f), c);
			Dir.Normalize();
			RegisterRaycast(Orig, Dir, 5000.0f);
			RegisterRaycast(Orig2, Dir, 5000.0f);
			RegisterRaycast(Orig3, Dir, 5000.0f);
			RegisterRaycast(Orig4, Dir, 5000.0f);
		}
	}

	virtual bool PlanetsideBulletRays::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool PlanetsideBulletRays::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportRaycasts)
			return false;
		return AddToPint(pint, mRepX);
	}

/*	virtual void PlanetsideBulletRays::CommonUpdate()
	{
		TestBase::CommonUpdate();
		udword Nb = GetNbRegisteredRaycasts();
		PintRaycastData* Data = GetRegisteredRaycasts();
		while(Nb--)
		{
			Ray R(Data->mOrigin, Data->mDir);
			R.Perturb(10.0f);
			Data->mDir = R.mDir;
			Data++;
		}
	}*/

END_TEST(PlanetsideBulletRays)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BlackRays = "Black rays (debug).";

START_SQ_RAYCAST_TEST_VS_MESH(BlackRays, CATEGORY_RAYCAST, gDesc_BlackRays)

	virtual bool BlackRays::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "black_rays.bin", true, true);
		LoadMeshesFromFile_(*this, "Venus.bin", null, false, 0);
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(BlackRays)

///////////////////////////////////////////////////////////////////////////////
