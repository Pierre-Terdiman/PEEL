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
#include "Loader_Bin.h"
#include "Loader_RepX.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SeaOfStaticConvexes = "64*64 static convexes. Used to measure memory usage. (Check or uncheck the per-engine options in the UI to share convexes). \
This also exposes issues in some engines that take a significant amount of time to simulate a scene where everything is static!";

START_TEST(SeaOfStaticConvexes, CATEGORY_STATIC_SCENE, gDesc_SeaOfStaticConvexes)

	virtual bool SeaOfStaticConvexes::CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SeaOfStaticConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticConvexes(pint, caps, 64, 64, 40.0f);
	}

END_TEST(SeaOfStaticConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SeaOfStaticBoxes = "128*128 static boxes";

START_TEST(SeaOfStaticBoxes, CATEGORY_STATIC_SCENE, gDesc_SeaOfStaticBoxes)

	virtual bool SeaOfStaticBoxes::CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SeaOfStaticBoxes::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticBoxes(pint, 40.0f, 128, 128, 0.0f);
	}

END_TEST(SeaOfStaticBoxes)

static const char* gDesc_SeaOfStaticBoxes2 = "128*128 static boxes";

START_TEST(SeaOfStaticBoxes2, CATEGORY_STATIC_SCENE, gDesc_SeaOfStaticBoxes2)

	virtual bool SeaOfStaticBoxes2::CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SeaOfStaticBoxes2::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticBoxes(pint, 100.0f, 128, 128, 0.0f);
	}

END_TEST(SeaOfStaticBoxes2)

static const char* gDesc_SeaOfStaticBoxes3 = "255*255 static boxes";

START_TEST(SeaOfStaticBoxes3, CATEGORY_STATIC_SCENE, gDesc_SeaOfStaticBoxes3)

	virtual bool SeaOfStaticBoxes3::CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool SeaOfStaticBoxes3::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticBoxes(pint, 200.0f, 255, 255, 0.0f);
	}

END_TEST(SeaOfStaticBoxes3)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SingleTriangle = "Single triangle. Used to check edge cases of static collision structures.";

START_TEST(SingleTriangle, CATEGORY_STATIC_SCENE, gDesc_SingleTriangle)

	virtual bool SingleTriangle::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 100.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SingleTriangle::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

END_TEST(SingleTriangle)

///////////////////////////////////////////////////////////////////////////////

#define IMPLEMENT_STATIC_MESH_SCENE(name, desc, filename, tesselation)	\
START_TEST(name, CATEGORY_STATIC_SCENE, desc)							\
																		\
	virtual bool name::CommonSetup()									\
	{																	\
		TestBase::CommonSetup();										\
		LoadMeshesFromFile_(*this, filename, null, false, tesselation);	\
		mCreateDefaultEnvironment = false;								\
		return true;													\
	}																	\
																		\
	virtual bool name::Setup(Pint& pint, const PintCaps& caps)			\
	{																	\
		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);	\
	}

static const char* gDesc_KP = "Konoko Payne mesh level. Used to check memory usage and for raytracing tests.";

IMPLEMENT_STATIC_MESH_SCENE(KP, gDesc_KP, "kp.bin", 0)

	virtual void KP::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1565.97f, 73.77f, -919.54f), Point(-0.65f, 0.41f, -0.63f));	// Zoom on a crack
		desc.mCamera[1] = CameraPose(Point(988.78f, 690.59f, -239.36f), Point(0.01f, -0.16f, -0.99f));
		desc.mCamera[2] = CameraPose(Point(-135.20f, 714.83f, 182.81f), Point(-0.96f, -0.10f, 0.24f));
		desc.mCamera[3] = CameraPose(Point(132.35f, 10.32f, 29.94f), Point(-0.45f, 0.70f, -0.56f));
		desc.mCamera[4] = CameraPose(Point(288.28f, 33.02f, 118.42f), Point(-0.78f, -0.33f, 0.54f));
		desc.mCamera[5] = CameraPose(Point(666.93f, 127.14f, 137.84f), Point(-0.77f, -0.29f, -0.57f));
		desc.mCamera[6] = CameraPose(Point(347.43f, 83.30f, 149.97f), Point(0.64f, -0.18f, -0.75f));
		desc.mCamera[7] = CameraPose(Point(367.75f, 91.14f, 119.23f), Point(-0.77f, -0.45f, 0.45f));
		desc.mCamera[8] = CameraPose(Point(318.61f, -88.88f, -995.75f), Point(-0.78f, 0.41f, 0.48f));
		desc.mCamera[9] = CameraPose(Point(1597.49f, 38.58f, -876.76f), Point(0.23f, 0.15f, -0.96f));
		desc.mCamera[10] = CameraPose(Point(561.67f, 190.55f, -1401.69f), Point(-0.41f, -0.12f, 0.90f));
		desc.mCamera[11] = CameraPose(Point(58.80f, 15.29f, 531.64f), Point(-0.81f, 0.11f, 0.57f));
		desc.mCamera[12] = CameraPose(Point(223.86f, 20.62f, 445.55f), Point(-0.42f, -0.40f, -0.81f));
		desc.mCamera[13] = CameraPose(Point(237.01f, 182.25f, 392.53f), Point(-0.79f, -0.26f, 0.55f));
	}

END_TEST(KP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Archipelago = "Archipelago mesh level. Used to check memory usage and for raytracing tests.";

IMPLEMENT_STATIC_MESH_SCENE(Archipelago, gDesc_Archipelago, "Archipelago.bin", 0)

	virtual void Archipelago::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-44.80f, 96.74f, -6.67f), Point(0.71f, -0.52f, 0.47f));
	}

END_TEST(Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Terrain = "Terrain tiles. Used to check memory usage and for raytracing tests.";

IMPLEMENT_STATIC_MESH_SCENE(Terrain, gDesc_Terrain, "Terrain.bin", 0)

	virtual void Terrain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.60f, -0.20f, -0.70f));
		desc.mCamera[1] = CameraPose(Point(-19.04f, 158.54f, -117.30f), Point(0.63f, -0.19f, 0.75f));
		desc.mCamera[2] = CameraPose(Point(3326.92f, 116.30f, 2212.80f), Point(-0.93f, -0.29f, -0.22f));
		desc.mCamera[3] = CameraPose(Point(3444.69f, 3034.93f, 117.91f), Point(-0.35f, -0.85f, 0.39f));
	}

END_TEST(Terrain)

IMPLEMENT_STATIC_MESH_SCENE(TesselatedTerrain, gDesc_Terrain, "Terrain.bin", 1)

	virtual void TesselatedTerrain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.60f, -0.20f, -0.70f));
		desc.mCamera[1] = CameraPose(Point(-19.04f, 158.54f, -117.30f), Point(0.63f, -0.19f, 0.75f));
		desc.mCamera[2] = CameraPose(Point(3326.92f, 116.30f, 2212.80f), Point(-0.93f, -0.29f, -0.22f));
		desc.mCamera[3] = CameraPose(Point(3444.69f, 3034.93f, 117.91f), Point(-0.35f, -0.85f, 0.39f));
	}

END_TEST(TesselatedTerrain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Venus = "Venus scene. Used for the raytracing test.";

IMPLEMENT_STATIC_MESH_SCENE(Venus, gDesc_Venus, "Venus.bin", 0)
	virtual void Venus::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(14.38f, 17.68f, -17.02f), Point(-0.73f, -0.36f, 0.58f));
	}
END_TEST(Venus)

static const char* gDesc_VenusMini100 = "Venus scene scaled by 1/100. Used for the raytracing test.";

	START_TEST(VenusMini100, CATEGORY_STATIC_SCENE, gDesc_VenusMini100)

		virtual bool VenusMini100::CommonSetup()
		{
			TestBase::CommonSetup();
			const float scale=1.0f/100.0f;
			LoadMeshesFromFile_(*this, "Venus.bin", &scale, false, 0);
			mCreateDefaultEnvironment = false;
			return true;
		}

		virtual bool VenusMini100::Setup(Pint& pint, const PintCaps& caps)
		{
			return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
		}

		virtual void VenusMini100::GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(0.1438f, 0.1768f, -0.1702f), Point(-0.73f, -0.36f, 0.58f));
		}
	END_TEST(VenusMini100)

static const char* gDesc_VenusMini1000 = "Venus scene scaled by 1/1000. Used for the raytracing test.";

	START_TEST(VenusMini1000, CATEGORY_STATIC_SCENE, gDesc_VenusMini1000)

		virtual bool VenusMini1000::CommonSetup()
		{
			TestBase::CommonSetup();
			const float scale=1.0f/1000.0f;
			LoadMeshesFromFile_(*this, "Venus.bin", &scale, false, 0);
			mCreateDefaultEnvironment = false;
			return true;
		}

		virtual bool VenusMini1000::Setup(Pint& pint, const PintCaps& caps)
		{
			return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
		}

		virtual void VenusMini1000::GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(0.01438f, 0.01768f, -0.01702f), Point(-0.73f, -0.36f, 0.58f));
		}
	END_TEST(VenusMini1000)

static const char* gDesc_VenusMaxi100 = "Venus scene scaled by 100. Used for the raytracing test.";

	START_TEST(VenusMaxi100, CATEGORY_STATIC_SCENE, gDesc_VenusMaxi100)

		virtual bool VenusMaxi100::CommonSetup()
		{
			TestBase::CommonSetup();
			const float scale=100.0f;
			LoadMeshesFromFile_(*this, "Venus.bin", &scale, false, 0);
			mCreateDefaultEnvironment = false;
			return true;
		}

		virtual bool VenusMaxi100::Setup(Pint& pint, const PintCaps& caps)
		{
			return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
		}

		virtual void VenusMaxi100::GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(1438.0f, 1768.0f, -1702.0f), Point(-0.73f, -0.36f, 0.58f));
		}
	END_TEST(VenusMaxi100)

static const char* gDesc_VenusMaxi1000 = "Venus scene scaled by 1000. Used for the raytracing test.";

	START_TEST(VenusMaxi1000, CATEGORY_STATIC_SCENE, gDesc_VenusMaxi1000)

		virtual bool VenusMaxi1000::CommonSetup()
		{
			TestBase::CommonSetup();
			const float scale=1000.0f;
			LoadMeshesFromFile_(*this, "Venus.bin", &scale, false, 0);
			mCreateDefaultEnvironment = false;
			return true;
		}

		virtual bool VenusMaxi1000::Setup(Pint& pint, const PintCaps& caps)
		{
			return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
		}

		virtual void VenusMaxi1000::GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mCamera[0] = CameraPose(Point(14380.0f, 17680.0f, -17020.0f), Point(-0.73f, -0.36f, 0.58f));
		}
	END_TEST(VenusMaxi1000)

IMPLEMENT_STATIC_MESH_SCENE(TesselatedVenus, gDesc_Venus, "Venus.bin", 1)
	virtual void TesselatedVenus::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(14.38f, 17.68f, -17.02f), Point(-0.73f, -0.36f, 0.58f));
	}
END_TEST(TesselatedVenus)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Mecha = "Mecha scene. Used for the raytracing test.";

IMPLEMENT_STATIC_MESH_SCENE(Mecha, gDesc_Mecha, "Mecha.bin", 0)
	virtual void Mecha::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(168.62f, 302.79f, -196.47f), Point(-0.40f, -0.74f, 0.55f));
	}
END_TEST(Mecha)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_RanRan = "RanRan scene (Poser mesh). Used for the raytracing test.";

IMPLEMENT_STATIC_MESH_SCENE(RanRan, gDesc_RanRan, "RanRan.bin", 0)
	virtual void RanRan::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-18.72f, 57.64f, -14.98f), Point(0.78f, 0.04f, 0.62f));
	}
END_TEST(RanRan)
IMPLEMENT_STATIC_MESH_SCENE(TesselatedRanRan, gDesc_RanRan, "RanRan.bin", 1)
	virtual void TesselatedRanRan::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-18.72f, 57.64f, -14.98f), Point(0.78f, 0.04f, 0.62f));
	}
END_TEST(TesselatedRanRan)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Bunny = "Bunny scene. Used for the raytracing test.";

IMPLEMENT_STATIC_MESH_SCENE(TesselatedBunny, gDesc_Bunny, "Bunny.bin", 3)
	virtual void TesselatedBunny::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.67f, 16.70f, 11.07f), Point(-0.67f, -0.53f, -0.52f));
	}
END_TEST(TesselatedBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_TestZone = "TestZone. Used for the raytracing test.";

IMPLEMENT_STATIC_MESH_SCENE(TestZone, gDesc_TestZone, "testzone.bin", 0)
	virtual void TestZone::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.43f, 5.68f, -31.10f), Point(0.90f, 0.05f, -0.43f));
		desc.mCamera[1] = CameraPose(Point(-39.54f, 13.74f, -92.63f), Point(0.76f, -0.16f, 0.64f));
		desc.mCamera[2] = CameraPose(Point(-22.40f, 10.29f, -39.07f), Point(0.42f, -0.28f, -0.86f));
		desc.mCamera[3] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.45f, -0.61f, -0.66f));
	}

END_TEST(TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Valve = "Valve level. Just the static scene for raytracing tests.";

START_TEST(Valve, CATEGORY_STATIC_SCENE, gDesc_Valve)

	virtual bool Valve::IsPrivate()	const
	{
		return true;
	}

	virtual bool Valve::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("c5m4_quarter2_Statics.repx", gValveScale, true);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool Valve::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes)
			return false;
		return AddToPint(pint, mRepX);
	}

END_TEST(Valve)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Planetside = "Planetside level. Just the static scene for raytracing tests.";

START_TEST(Planetside, CATEGORY_STATIC_SCENE, gDesc_Planetside)

	virtual bool Planetside::IsPrivate()	const
	{
		return true;
	}

	virtual void Planetside::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(93.89f, 19.72f, 689.68f), Point(0.38f, -0.37f, 0.85f));
		desc.mCamera[1] = CameraPose(Point(60.21f, 49.41f, 618.84f), Point(0.72f, -0.25f, 0.65f));
		desc.mCamera[2] = CameraPose(Point(229.98f, 5.80f, 704.46f), Point(0.51f, -0.12f, 0.85f));
		desc.mCamera[3] = CameraPose(Point(163.14f, 1.22f, 776.86f), Point(-0.05f, -0.06f, -1.00f));
		desc.mCamera[4] = CameraPose(Point(232.90f, 5.12f, 774.70f), Point(0.39f, -0.13f, -0.91f));
		desc.mCamera[5] = CameraPose(Point(409.69f, 23.02f, 712.23f), Point(-0.99f, -0.12f, 0.10f));
		desc.mCamera[6] = CameraPose(Point(131.35f, 69.80f, 700.85f), Point(-0.50f, -0.82f, 0.28f));
		desc.mCamera[7] = CameraPose(Point(227.48f, 24.30f, 662.48f), Point(-0.23f, -0.11f, 0.97f));
	}

	virtual bool Planetside::CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool Planetside::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes)
			return false;
		return AddToPint(pint, mRepX);
	}

END_TEST(Planetside)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Specular = "Specular level. Just the static scene for raytracing tests.";

START_TEST(Specular, CATEGORY_STATIC_SCENE, gDesc_Specular)

	virtual bool Specular::IsPrivate()	const
	{
		return true;
	}

	virtual bool Specular::CommonSetup()
	{
		TestBase::CommonSetup();
//		mRepX = CreateRepXContext("specular_raycast_meshes.repx", 1.0f, false);
		mRepX = CreateRepXContext("specular_raycast_meshes_statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool Specular::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes)
			return false;
		return AddToPint(pint, mRepX);
	}

END_TEST(Specular)

///////////////////////////////////////////////////////////////////////////////
