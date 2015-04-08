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
#include "Loader_Bin.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_01 = "CCD: 10 dynamic boxes (1.0; 1.0; 1.0) moving with linear velocity 400 against thin static boxes.";

START_TEST(CCDTest_DynamicsVsStatics_01, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_01)

	virtual void CCDTest_DynamicsVsStatics_01::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(22.80f, 27.34f, 23.90f), Point(-0.61f, -0.51f, -0.61f));
	}

	virtual bool CCDTest_DynamicsVsStatics_01::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 0.01f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const Point Extents(1.0f, 1.0f, 1.0f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		udword NbFastBoxes = 10;
		for(udword i=0;i<NbFastBoxes;i++)
		{
			const float Angle = float(i)*TWOPI/float(NbFastBoxes);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= Point(cosf(Angle)*5.0f, 5.0f, sinf(Angle)*5.0f);
			ObjectDesc.mLinearVelocity	= 400.0f * Point(ObjectDesc.mPosition.x, 0.0f, ObjectDesc.mPosition.z).Normalize();
			CreatePintObject(pint, ObjectDesc);
		}
		return true;
	}

END_TEST(CCDTest_DynamicsVsStatics_01)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_02 = "CCD: 30 dynamic boxes (0.4; 0.4; 0.4) moving with linear velocity 200 against thin static boxes.";

START_TEST(CCDTest_DynamicsVsStatics_02, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_02)

	virtual void CCDTest_DynamicsVsStatics_02::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(23.89f, 28.39f, 23.00f), Point(-0.58f, -0.60f, -0.55f));
	}

	virtual bool CCDTest_DynamicsVsStatics_02::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 0.01f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const Point Extents(0.4f, 0.4f, 0.4f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		udword NbFastBoxes = 30;
		for(udword i=0;i<NbFastBoxes;i++)
		{
			const float Angle = float(i)*TWOPI/float(NbFastBoxes);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= Point(cosf(Angle)*5.0f, 5.0f, sinf(Angle)*5.0f);
			ObjectDesc.mLinearVelocity	= 200.0f * Point(ObjectDesc.mPosition.x, 0.0f, ObjectDesc.mPosition.z).Normalize();
			CreatePintObject(pint, ObjectDesc);
		}
		return true;
	}

END_TEST(CCDTest_DynamicsVsStatics_02)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_03 = "CCD: 40 thin dynamic rods (0.04; 4.0; 0.04) falling with linear velocity 4 against a thin static planar mesh.";

START_TEST(CCDTest_DynamicsVsStatics_03, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_03)

	virtual void CCDTest_DynamicsVsStatics_03::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(20.96f, 11.22f, 22.69f), Point(-0.65f, -0.31f, -0.70f));
	}

	virtual bool CCDTest_DynamicsVsStatics_03::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportRigidBodySimulation)
			return false;

		CreatePlanarMesh(*this, pint, 0.0f);

	//	const Point Extents(0.01f, 4.0f, 0.01f);
		const Point Extents(0.04f, 4.0f, 0.04f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		udword NbFastBoxes = 40;
		for(udword i=0;i<NbFastBoxes;i++)
		{
			const float Angle = float(i)*TWOPI/float(NbFastBoxes);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= Point(cosf(Angle)*5.0f, 10.0f, sinf(Angle)*5.0f);
			ObjectDesc.mLinearVelocity	= 4.0f * Point(ObjectDesc.mPosition.x, 0.0f, ObjectDesc.mPosition.z).Normalize();
			CreatePintObject(pint, ObjectDesc);
		}

		mCreateDefaultEnvironment = false;

		return true;
	}

END_TEST(CCDTest_DynamicsVsStatics_03)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_04 = "CCD: 32*32 dynamic convexes falling with linear velocity 100 against a static mesh level.";

START_TEST(CCDTest_DynamicsVsStatics_04, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_04)

	virtual void CCDTest_DynamicsVsStatics_04::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(42.22f, 50.00f, 42.77f), Point(0.66f, -0.50f, 0.55f));
	}

	virtual bool CCDTest_DynamicsVsStatics_04::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool CCDTest_DynamicsVsStatics_04::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
	//	udword i=2;	// Small convexes
		udword i=4;	// 'Big' convexes
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 10.0f;
		const Point LinVel(0.0f, -100.0f, 0.0f);
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Extents.x-8.0f, Extents.z-8.0f, &Offset, &LinVel);
	}

END_TEST(CCDTest_DynamicsVsStatics_04)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_05 = "CCD: a single dynamic convex thrown with linear velocity 1500 against a complex static mesh.";

START_TEST(CCDTest_DynamicsVsStatics_05, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_05)

	virtual void CCDTest_DynamicsVsStatics_05::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(45.24f, 27.75f, 55.62f), Point(-0.57f, -0.10f, -0.82f));
	}

	virtual bool CCDTest_DynamicsVsStatics_05::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool CCDTest_DynamicsVsStatics_05::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const Point Extents(1.0f, 1.0f, 1.0f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= 1.0f;
		ObjectDesc.mPosition		= Point(0.0f, 10.0f, 50.0f);
		ObjectDesc.mLinearVelocity	= Point(0.0f, 0.0f, -1500.0f);
		CreatePintObject(pint, ObjectDesc);
		return true;
	}

END_TEST(CCDTest_DynamicsVsStatics_05)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_06 = "CCD: 32*32 dynamic convexes (of low complexity) moving down with linear velocity 100 against on a tesselated planar mesh.";

START_TEST(CCDTest_DynamicsVsStatics_06, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_06)

	virtual void CCDTest_DynamicsVsStatics_06::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(87.10f, 15.01f, 23.17f), Point(-0.93f, -0.29f, -0.24f));
	}

	virtual bool CCDTest_DynamicsVsStatics_06::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		CreatePlanarMesh(*this, pint, Altitude);

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
		udword i=2;	// Small convexes
	//	udword i=4;	// 'Big' convexes
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		for(int i=0;i<C.mNbVerts;i++)
			C.mVerts[i] *= 0.1f;

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		mCreateDefaultEnvironment = false;

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 20.0f;
		const float Scale = 35.0f;
		const Point LinVel(0.0f, -100.0f, 0.0f);
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Scale, Scale, null, &LinVel);
	}

END_TEST(CCDTest_DynamicsVsStatics_06)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicsVsStatics_BehaviorAfterImpact = "CCD: tests behavior of object after a CCD impact.";

START_TEST(CCDTest_DynamicsVsStatics_BehaviorAfterImpact, CATEGORY_CCD, gDesc_CCDTest_DynamicsVsStatics_BehaviorAfterImpact)

	virtual void CCDTest_DynamicsVsStatics_BehaviorAfterImpact::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-26.01f, 8.50f, 25.06f), Point(0.45f, -0.14f, -0.88f));
	}

	virtual bool CCDTest_DynamicsVsStatics_BehaviorAfterImpact::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		CreatePlanarMesh(*this, pint, Altitude);

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
		udword i=2;	// Small convexes
	//	udword i=4;	// 'Big' convexes
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		for(int i=0;i<C.mNbVerts;i++)
			C.mVerts[i] *= 0.1f;

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		mCreateDefaultEnvironment = false;

		const udword NbX = 2;
		const udword NbY = 2;
		const float AltitudeC = 20.0f;
		const float Scale = 3.0f;
		const Point LinVel(-100.0f, -200.0f, 0.0f);
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Scale, Scale, null, &LinVel);
	}

END_TEST(CCDTest_DynamicsVsStatics_BehaviorAfterImpact)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicDynamic_BoxVsStack = "CCD: dynamic vs dynamic. A fast moving dynamic box is thrown against a stack of dynamic boxes.";

START_TEST(CCDTest_DynamicDynamic_BoxVsStack, CATEGORY_CCD, gDesc_CCDTest_DynamicDynamic_BoxVsStack)

	virtual void CCDTest_DynamicDynamic_BoxVsStack::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(34.25f, 19.94f, 51.78f), Point(-0.48f, 0.00f, -0.88f));
		desc.mCamera[1] = CameraPose(Point(-0.50f, 11.52f, 14.95f), Point(-0.01f, -0.04f, -1.00f));
	}

	virtual bool CCDTest_DynamicDynamic_BoxVsStack::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		CreateBoxStack(pint, caps, 1, 20);

		const Point Extents(1.0f, 1.0f, 1.0f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= 1.0f;
		ObjectDesc.mPosition		= Point(0.0f, 10.0f, 110.0f);
		ObjectDesc.mLinearVelocity	= Point(0.0f, 0.0f, -1500.0f);
		CreatePintObject(pint, ObjectDesc);
		return true;
	}

END_TEST(CCDTest_DynamicDynamic_BoxVsStack)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicDynamic_CompoundCascade = "CCD: dynamic vs dynamic. Multiple compounds falling down from the sky.";

START_TEST(CCDTest_DynamicDynamic_CompoundCascade, CATEGORY_CCD, gDesc_CCDTest_DynamicDynamic_CompoundCascade)

	virtual bool CCDTest_DynamicDynamic_CompoundCascade::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportCompounds)
			return false;

		const Point Extents(1.5f, 0.5f, 0.5f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		const Point Extents2(0.5f, 0.5f, 1.5f);
		PINT_BOX_CREATE BoxDesc2;
		BoxDesc2.mExtents	= Extents2;
		BoxDesc2.mRenderer	= CreateBoxRenderer(Extents2);
		BoxDesc2.mNext		= &BoxDesc;

		for(udword i=0;i<100;i++)
		{
			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc2;
			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= Point(0.0f, float(i)*2.0f, 0.0f);
			CreatePintObject(pint, ObjectDesc);
		}
		return true;
	}

END_TEST(CCDTest_DynamicDynamic_CompoundCascade)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicDynamic_ConvexCascade = "CCD: dynamic vs dynamic stress test. Multiple convexes & thin boxes falling down from the sky. Try to pick up & manipulate pieces after the fall...";

START_TEST(CCDTest_DynamicDynamic_ConvexCascade, CATEGORY_CCD, gDesc_CCDTest_DynamicDynamic_ConvexCascade)

	virtual bool CCDTest_DynamicDynamic_ConvexCascade::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes)
			return false;

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
		C.LoadFile(1);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const Point Extents(10.0f, 0.1f, 10.0f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		for(udword i=0;i<100;i++)
		{
			PINT_OBJECT_CREATE ObjectDesc;
			if(i&1)
				ObjectDesc.mShapes	= &ConvexCreate;
			else
				ObjectDesc.mShapes	= &BoxDesc;
			ObjectDesc.mMass		= 1.0f;
			ObjectDesc.mPosition	= Point(0.0f, float(i)*2.0f, 0.0f);
	//		if(!(i&1))
	//			ObjectDesc.mLinearVelocity	= Point(0.0f, -1000.0f, 0.0f);
			CreatePintObject(pint, ObjectDesc);
		}
		return true;
	}

END_TEST(CCDTest_DynamicDynamic_ConvexCascade)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CCDTest_DynamicDynamic_PileOfThinBoxes = "CCD: dynamic vs dynamic. Pile of thin boxes. Try to pick up & manipulate pieces after the fall...";

START_TEST(CCDTest_DynamicDynamic_PileOfThinBoxes, CATEGORY_CCD, gDesc_CCDTest_DynamicDynamic_PileOfThinBoxes)

	virtual bool CCDTest_DynamicDynamic_PileOfThinBoxes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const Point Extents(10.0f, 0.1f, 10.0f);
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		for(udword i=0;i<64;i++)
		{
			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 1.0f;
			ObjectDesc.mPosition	= Point(0.0f, float(i)*0.2f, 0.0f);
			CreatePintObject(pint, ObjectDesc);
		}
		return true;
	}

END_TEST(CCDTest_DynamicDynamic_PileOfThinBoxes)

///////////////////////////////////////////////////////////////////////////////
