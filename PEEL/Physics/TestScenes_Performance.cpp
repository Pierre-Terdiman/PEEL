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
#include "Random.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LargeBoxStack50 = "Giga box stack (50-boxes-wide base). This one is just to show off with ICE Physics :)";

START_TEST(LargeBoxStack50, CATEGORY_PERFORMANCE, gDesc_LargeBoxStack50)

	virtual	void LargeBoxStack50::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(69.17f, 51.73f, 80.45f), Point(-0.57f, -0.08f, -0.82f));
		desc.mCamera[1] = CameraPose(Point(1.25f, 99.61f, 9.47f), Point(-0.28f, -0.16f, -0.95f));
	}

	virtual bool LargeBoxStack50::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 50);
	}

END_TEST(LargeBoxStack50)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LargeBoxStack30 = "Classical large box stack scene (30-boxes-wide base). Things to look for are the time it takes to simulate it, but also whether the stack eventually collapses or not.";

START_TEST(LargeBoxStack30, CATEGORY_PERFORMANCE, gDesc_LargeBoxStack30)

	virtual	void LargeBoxStack30::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(47.56f, 48.75f, 47.08f), Point(-0.61f, -0.31f, -0.73f));
	}

	virtual bool LargeBoxStack30::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 30);
	}

END_TEST(LargeBoxStack30)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_MediumBoxStack20 = "Similar to the previous test, medium-sized stack (20-boxes-wide base).";

START_TEST(MediumBoxStack20, CATEGORY_PERFORMANCE, gDesc_MediumBoxStack20)

	virtual	void MediumBoxStack20::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(26.29f, 36.32f, 35.39f), Point(-0.51f, -0.35f, -0.79f));
	}

	virtual bool MediumBoxStack20::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 20);
	//	return CreateBoxStack(pint, caps, 1, 14);
	}

END_TEST(MediumBoxStack20)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_MediumBoxStacks20 = "Multiple medium-sized box stacks (20-boxes-wide base). Similar to the medium box stack scene but with multiple simulation islands.";

START_TEST(MediumBoxStacks20, CATEGORY_PERFORMANCE, gDesc_MediumBoxStacks20)

	virtual	void MediumBoxStacks20::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(35.88f, 38.56f, 60.53f), Point(-0.58f, -0.41f, -0.70f));
	}

	virtual bool MediumBoxStacks20::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 10, 20);
	}

END_TEST(MediumBoxStacks20)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ManySmallBoxStacks10 = "Multiple small-sized box stacks (10-boxes-wide base).";

START_TEST(ManySmallBoxStacks10, CATEGORY_PERFORMANCE, gDesc_ManySmallBoxStacks10)

	virtual	void ManySmallBoxStacks10::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(18.79f, 20.00f, 130.56f), Point(-0.51f, -0.35f, -0.79f));
	}

	virtual bool ManySmallBoxStacks10::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 30, 10);
	}

END_TEST(ManySmallBoxStacks10)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxContainerAndSpheres = "A static box container filled with 256 dynamic spheres.";

START_TEST(BoxContainerAndSpheres, CATEGORY_PERFORMANCE, gDesc_BoxContainerAndSpheres)

	virtual	void BoxContainerAndSpheres::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool BoxContainerAndSpheres::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float Radius = 0.5f;
		const udword NbLayers = 4;
		const udword NbX = 16;
		const udword NbY = 16;
		float yy = Radius;
		BasicRandom Rnd(42);

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= Radius;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		for(udword k=0;k<NbLayers;k++)
		{
			for(udword y=0;y<NbY;y++)
			{
				const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

					const float RandomX = 0.1f * Rnd.RandomFloat();
					const float RandomY = 0.1f * Rnd.RandomFloat();

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &SphereDesc;
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - Radius - BoxSide*2.0f);
					ObjectDesc.mPosition.y	= yy;
					ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - Radius - BoxSide*2.0f);
					CreatePintObject(pint, ObjectDesc);
				}
			}
			yy += Radius*2.0f;
		}
		return true;
	}

END_TEST(BoxContainerAndSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxContainerAndBoxes = "A static box container filled with 256 dynamic boxes.";

START_TEST(BoxContainerAndBoxes, CATEGORY_PERFORMANCE, gDesc_BoxContainerAndBoxes)

	virtual	void BoxContainerAndBoxes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool BoxContainerAndBoxes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float Radius = 0.3f;
		const udword NbLayers = 4;
		const udword NbX = 16;
		const udword NbY = 16;
		float yy = Radius;
		BasicRandom Rnd(42);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(Radius, Radius, Radius);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		for(udword k=0;k<NbLayers;k++)
		{
			for(udword y=0;y<NbY;y++)
			{
				const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

					const float RandomX = 0.1f * Rnd.RandomFloat();
					const float RandomY = 0.1f * Rnd.RandomFloat();

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &BoxDesc;
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - Radius - BoxSide*2.0f);
					ObjectDesc.mPosition.y	= yy;
					ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - Radius - BoxSide*2.0f);

					UnitRandomQuat(ObjectDesc.mRotation, Rnd);

					CreatePintObject(pint, ObjectDesc);
				}
			}
			yy += Radius*2.0f;
		}
		return true;
	}

END_TEST(BoxContainerAndBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxContainerAndCapsules = "A static box container filled with 256 dynamic capsules.";

START_TEST(BoxContainerAndCapsules, CATEGORY_PERFORMANCE, gDesc_BoxContainerAndCapsules)

	virtual	void BoxContainerAndCapsules::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool BoxContainerAndCapsules::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float Radius = 0.3f;
		const float HalfHeight = 0.5f;
		const udword NbLayers = 4;
		const udword NbX = 16;
		const udword NbY = 16;
		float yy = Radius;
		BasicRandom Rnd(42);

		PINT_CAPSULE_CREATE CapsuleDesc;
		CapsuleDesc.mRadius		= Radius;
		CapsuleDesc.mHalfHeight	= HalfHeight;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(Radius, HalfHeight*2.0f);

		for(udword k=0;k<NbLayers;k++)
		{
			for(udword y=0;y<NbY;y++)
			{
				const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

					const float RandomX = 0.1f * Rnd.RandomFloat();
					const float RandomY = 0.1f * Rnd.RandomFloat();

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &CapsuleDesc;
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - Radius - BoxSide*2.0f);
					ObjectDesc.mPosition.y	= yy;
					ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - Radius - BoxSide*2.0f);

					UnitRandomQuat(ObjectDesc.mRotation, Rnd);

					CreatePintObject(pint, ObjectDesc);
				}
			}
			yy += HalfHeight*2.0f;
		}
		return true;
	}

END_TEST(BoxContainerAndCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PotPourri_Box = "A static box container filled with 256 random dynamic objects (sphere, box or capsule).";

START_TEST(PotPourri_Box, CATEGORY_PERFORMANCE, gDesc_PotPourri_Box)

	virtual	void PotPourri_Box::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool PotPourri_Box::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 4.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float SphereRadius = 0.5f;
		const float CapsuleRadius = 0.3f;
		const float HalfHeight = 0.5f;
		const udword NbLayers = 4;
		const udword NbX = 16;
		const udword NbY = 16;
		float yy = CapsuleRadius;
		BasicRandom Rnd(42);

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= SphereRadius;
		SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(CapsuleRadius, CapsuleRadius, CapsuleRadius);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_CAPSULE_CREATE CapsuleDesc;
		CapsuleDesc.mRadius		= CapsuleRadius;
		CapsuleDesc.mHalfHeight	= HalfHeight;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(CapsuleRadius, HalfHeight*2.0f);

		for(udword k=0;k<NbLayers;k++)
		{
			for(udword y=0;y<NbY;y++)
			{
				const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

					const float RandomX = 0.1f * Rnd.RandomFloat();
					const float RandomY = 0.1f * Rnd.RandomFloat();

					const udword Index = Rnd.Randomize() % 3;

					PINT_OBJECT_CREATE ObjectDesc;
					if(Index==0)
						ObjectDesc.mShapes	= &SphereDesc;
					else if(Index==1)
						ObjectDesc.mShapes	= &BoxDesc;
					else if(Index==2)
						ObjectDesc.mShapes	= &CapsuleDesc;
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - SphereRadius - BoxSide*2.0f);
					ObjectDesc.mPosition.y	= yy;
					ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - SphereRadius - BoxSide*2.0f);

					UnitRandomQuat(ObjectDesc.mRotation, Rnd);

					CreatePintObject(pint, ObjectDesc);
				}
			}
			yy += HalfHeight*2.0f;
		}
		return true;
	}

END_TEST(PotPourri_Box)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PotPourri_Box_WithConvexes = "A static box container filled with 256 random dynamic objects (sphere, box, capsule or convex).";

START_TEST(PotPourri_Box_WithConvexes, CATEGORY_PERFORMANCE, gDesc_PotPourri_Box_WithConvexes)

	virtual	void PotPourri_Box_WithConvexes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool PotPourri_Box_WithConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

	//	const udword NbInsideCirclePts = 4;
	//	const udword NbOutsideCirclePts = 4;
		const udword NbInsideCirclePts = 8;
		const udword NbOutsideCirclePts = 8;
	//	const udword NbInsideCirclePts = 32;
	//	const udword NbOutsideCirclePts = 32;

		const udword TotalNbVerts = NbInsideCirclePts + NbOutsideCirclePts;
		ASSERT(TotalNbVerts<1024);

		Point Pts[1024];
		const float ConvexScale = 2.0f;
		udword NbPts = GenerateConvex(Pts, NbInsideCirclePts, NbOutsideCirclePts, 0.2f*ConvexScale, 0.3f*ConvexScale, 0.2f*ConvexScale);
		ASSERT(NbPts==TotalNbVerts);

		PINT_CONVEX_CREATE ConvexCreate;
		ConvexCreate.mNbVerts	= TotalNbVerts;
		ConvexCreate.mVerts		= Pts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(TotalNbVerts, Pts);

		///

		const float BoxHeight = 4.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 10.0f;

		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float SphereRadius = 0.5f;
		const float CapsuleRadius = 0.3f;
		const float HalfHeight = 0.5f;
		const udword NbLayers = 4;
		const udword NbX = 16;
		const udword NbY = 16;
		float yy = CapsuleRadius;
		BasicRandom Rnd(42);

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= SphereRadius;
		SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(CapsuleRadius, CapsuleRadius, CapsuleRadius);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_CAPSULE_CREATE CapsuleDesc;
		CapsuleDesc.mRadius		= CapsuleRadius;
		CapsuleDesc.mHalfHeight	= HalfHeight;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(CapsuleRadius, HalfHeight*2.0f);

		for(udword k=0;k<NbLayers;k++)
		{
			for(udword y=0;y<NbY;y++)
			{
				const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

					const float RandomX = 0.1f * Rnd.RandomFloat();
					const float RandomY = 0.1f * Rnd.RandomFloat();

					const udword Index = Rnd.Randomize() % 4;

					PINT_OBJECT_CREATE ObjectDesc;
					if(Index==0)
						ObjectDesc.mShapes	= &SphereDesc;
					else if(Index==1)
						ObjectDesc.mShapes	= &BoxDesc;
					else if(Index==2)
						ObjectDesc.mShapes	= &CapsuleDesc;
					else if(Index==3)
						ObjectDesc.mShapes	= &ConvexCreate;
					
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - SphereRadius - BoxSide*2.0f);
					ObjectDesc.mPosition.y	= yy;
					ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - SphereRadius - BoxSide*2.0f);

					UnitRandomQuat(ObjectDesc.mRotation, Rnd);

					CreatePintObject(pint, ObjectDesc);
				}
			}
			yy += HalfHeight*2.0f;
		}
		return true;
	}

END_TEST(PotPourri_Box_WithConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SpheresOnLargeBox = "1024 dynamic spheres on a large static box.";

START_TEST(SpheresOnLargeBox, CATEGORY_PERFORMANCE, gDesc_SpheresOnLargeBox)

	virtual bool SpheresOnLargeBox::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 0.5f;
		const udword NbX = 32;
		const udword NbY = 32;
		return GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Radius, 30.0f, 30.0f);
	}

END_TEST(SpheresOnLargeBox)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Compounds320NotTouching = "The notorious compounds from the PhysX samples. Each compound is made of 320 boxes. In this test they do not touch, so \
we should not measure the impact of compound-vs-compound interaction. Engines using a single broadphase entry for each compound should perform a lot better than \
engines using a broadphase entry for each sub-shape.";

START_TEST(Compounds320NotTouching, CATEGORY_PERFORMANCE, gDesc_Compounds320NotTouching)

	virtual	void Compounds320NotTouching::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(104.05f, 43.87f, 100.62f), Point(-0.63f, -0.48f, -0.61f));
	}

	virtual bool Compounds320NotTouching::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCompounds || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbBoxes = 320;
		const float BoxExtent = 0.4f;
		const Point Extents(BoxExtent, BoxExtent, BoxExtent);
		PintShapeRenderer* RenderObject = CreateBoxRenderer(Extents);

	//	const udword NbBoxes = 32;
	//	const float BoxExtent = 2.0f;

		const float SphereRadius = 10.0f;
		const float BoxPosY = SphereRadius + BoxExtent;

		PINT_BOX_CREATE BoxDesc[NbBoxes];
		BasicRandom Rnd(42);
		for(udword i=0;i<NbBoxes;i++)
		{
			UnitRandomQuat(BoxDesc[i].mLocalRot, Rnd);
			UnitRandomPt(BoxDesc[i].mLocalPos, Rnd);
			BoxDesc[i].mLocalPos *= SphereRadius;

			BoxDesc[i].mExtents	= Extents;
			BoxDesc[i].mRenderer = RenderObject;
			if(i!=NbBoxes-1)
				BoxDesc[i].mNext = &BoxDesc[i+1];
		}

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= BoxDesc;
		ObjectDesc.mPosition.y	= BoxPosY;
		ObjectDesc.mMass		= 1.0f;

		for(udword y=0;y<4;y++)
		{
			const float YCoeff = float(y) - 1.0f;
			for(udword x=0;x<4;x++)
			{
				const float XCoeff = float(x) - 1.0f;

				ObjectDesc.mPosition.x	= XCoeff * SphereRadius * 4.0f;
				ObjectDesc.mPosition.z	= YCoeff * SphereRadius * 4.0f;

				PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
				ASSERT(Handle);
			}
		}
		return true;
	}

END_TEST(Compounds320NotTouching)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Compounds320Touching = "The notorious compounds from the PhysX samples. Each compound is made of 320 boxes. In this test they do touch, so \
we should measure the impact of compound-vs-compound interaction. Engines using a single broadphase entry for each compound may perform a lot worse than \
engines using a broadphase entry for each sub-shape, if the compound-compound code is O(n^2).";

START_TEST(Compounds320Touching, CATEGORY_PERFORMANCE, gDesc_Compounds320Touching)

	virtual	void Compounds320Touching::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(16.62f, 195.63f, 11.85f), Point(-0.33f, -0.93f, -0.17f));
	}

	virtual bool Compounds320Touching::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCompounds|| !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxHeight = 80.0f;
		const float BoxSide = 1.0f;
		const float BoxDepth = 15.0f;
		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);

		const float SphereRadius = 10.0f;
		const float BoxExtent = 0.4f;
		const float BoxPosY = SphereRadius + BoxExtent;
		const Point Extents(BoxExtent, BoxExtent, BoxExtent);
		PintShapeRenderer* RenderObject = CreateBoxRenderer(Extents);

		const udword NbBoxes = 320;
		PINT_BOX_CREATE BoxDesc[NbBoxes];
		BasicRandom Rnd(42);
		for(udword i=0;i<NbBoxes;i++)
		{
			UnitRandomQuat(BoxDesc[i].mLocalRot, Rnd);
			UnitRandomPt(BoxDesc[i].mLocalPos, Rnd);
			BoxDesc[i].mLocalPos *= SphereRadius;

			BoxDesc[i].mExtents		= Extents;
			BoxDesc[i].mRenderer	= RenderObject;
			if(i!=NbBoxes-1)
				BoxDesc[i].mNext = &BoxDesc[i+1];
		}

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= BoxDesc;
		ObjectDesc.mPosition.x	= 0.0f;
		ObjectDesc.mPosition.z	= 0.0f;
		ObjectDesc.mPosition.y	= BoxPosY;
		ObjectDesc.mMass		= 1.0f;

		for(udword n=0;n<8;n++)
		{
			ObjectDesc.mPosition.y	= BoxPosY + n * SphereRadius * 2.5f;

			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);
		}
		return true;
	}

END_TEST(Compounds320Touching)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_StackOfSmallCompounds = "8*8*8 compound objects stacked in a grid fashion. Each compound has 3 sub-shapes.";

START_TEST(StackOfSmallCompounds, CATEGORY_PERFORMANCE, gDesc_StackOfSmallCompounds)

	virtual	void StackOfSmallCompounds::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(111.02f, 69.80f, 121.30f), Point(-0.63f, -0.30f, -0.72f));
	}

	virtual bool StackOfSmallCompounds::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCompounds || !caps.mSupportRigidBodySimulation)
			return false;

		const float LargeBoxExtent = 5.0f;
		const float SmallBoxExtent = 1.0f;

		PINT_BOX_CREATE BoxDesc[3];
		BoxDesc[0].mExtents	= Point(LargeBoxExtent, SmallBoxExtent, SmallBoxExtent);
		BoxDesc[1].mExtents	= Point(SmallBoxExtent, LargeBoxExtent, SmallBoxExtent);
		BoxDesc[2].mExtents	= Point(SmallBoxExtent, SmallBoxExtent, LargeBoxExtent);
		BoxDesc[0].mRenderer	= CreateBoxRenderer(BoxDesc[0].mExtents);
		BoxDesc[1].mRenderer	= CreateBoxRenderer(BoxDesc[1].mExtents);
		BoxDesc[2].mRenderer	= CreateBoxRenderer(BoxDesc[2].mExtents);
		BoxDesc[0].mNext = &BoxDesc[1];
		BoxDesc[1].mNext = &BoxDesc[2];

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= BoxDesc;
		ObjectDesc.mMass		= 1.0f;

		const udword NbX = 8;
		const udword NbY = 8;
		const udword NbZ = 8;

		const float Spacing = LargeBoxExtent*2.0f;

		for(udword y=0;y<NbY;y++)
		{
			const float CoeffY = float(y);
			for(udword z=0;z<NbZ;z++)
			{
				const float CoeffZ = float(z);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = float(x);

					ObjectDesc.mPosition	= Point(CoeffX*Spacing, LargeBoxExtent+CoeffY*Spacing, CoeffZ*Spacing);
					PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
					ASSERT(Handle);
				}
			}
		}
		return true;
	}

END_TEST(StackOfSmallCompounds)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfSmallCompounds = "A pile of 8*8*8 compound objects. Each compound has 3 sub-shapes.";

START_TEST(PileOfSmallCompounds, CATEGORY_PERFORMANCE, gDesc_PileOfSmallCompounds)

	virtual	void PileOfSmallCompounds::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(112.68f, 74.16f, 113.66f), Point(-0.61f, -0.51f, -0.61f));
	}

	virtual bool PileOfSmallCompounds::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCompounds || !caps.mSupportRigidBodySimulation)
			return false;

		const float LargeBoxExtent = 5.0f;
		const float SmallBoxExtent = 1.0f;

		PINT_BOX_CREATE BoxDesc[3];
		BoxDesc[0].mExtents	= Point(LargeBoxExtent, SmallBoxExtent, SmallBoxExtent);
		BoxDesc[1].mExtents	= Point(SmallBoxExtent, LargeBoxExtent, SmallBoxExtent);
		BoxDesc[2].mExtents	= Point(SmallBoxExtent, SmallBoxExtent, LargeBoxExtent);
		BoxDesc[0].mRenderer	= CreateBoxRenderer(BoxDesc[0].mExtents);
		BoxDesc[1].mRenderer	= CreateBoxRenderer(BoxDesc[1].mExtents);
		BoxDesc[2].mRenderer	= CreateBoxRenderer(BoxDesc[2].mExtents);
		BoxDesc[0].mNext = &BoxDesc[1];
		BoxDesc[1].mNext = &BoxDesc[2];

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= BoxDesc;
		ObjectDesc.mMass		= 1.0f;

		const udword NbX = 8;
		const udword NbY = 8;
		const udword NbZ = 8;

		const float Spacing = LargeBoxExtent*2.1f;

		BasicRandom Rnd(42);

		for(udword y=0;y<NbY;y++)
		{
			const float CoeffY = float(y);
			for(udword z=0;z<NbZ;z++)
			{
				const float CoeffZ = float(z);
				for(udword x=0;x<NbX;x++)
				{
					const float CoeffX = float(x);

					UnitRandomQuat(ObjectDesc.mRotation, Rnd);

					ObjectDesc.mPosition	= Point(CoeffX*Spacing, LargeBoxExtent+CoeffY*Spacing, CoeffZ*Spacing);
					PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
					ASSERT(Handle);
				}
			}
		}
		return true;
	}

END_TEST(PileOfSmallCompounds)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexStack = "Convex stack scene from the PhysX SDK. There are 4 layers of 16*16 convexes, i.e. 1024 convexes in total. Each convex has 16 vertices.";

START_TEST(ConvexStack, CATEGORY_PERFORMANCE, gDesc_ConvexStack)

	virtual	void ConvexStack::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(81.67f, 27.75f, 68.78f), Point(-0.75f, -0.38f, -0.54f));
	}

	virtual bool ConvexStack::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 16;
		const udword NbY = 16;
		const udword NbLayers = 4;

	//	const udword NbInsideCirclePts = 4;
	//	const udword NbOutsideCirclePts = 4;
		const udword NbInsideCirclePts = 8;
		const udword NbOutsideCirclePts = 8;
	//	const udword NbInsideCirclePts = 32;
	//	const udword NbOutsideCirclePts = 32;

		return CreateTestScene_ConvexStack_Generic(pint, NbX, NbY, NbLayers, NbInsideCirclePts, NbOutsideCirclePts);
	}

END_TEST(ConvexStack)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexStack2 = "Convex stack scene from the PhysX SDK. Each convex has 16 vertices.";

START_TEST(ConvexStack2, CATEGORY_PERFORMANCE, gDesc_ConvexStack2)

	virtual	void ConvexStack2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(43.75f, 14.01f, 42.92f), Point(-0.70f, -0.24f, -0.67f));
	}

	virtual bool ConvexStack2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 12;
		const udword NbY = 12;

		Point Pts[16];
		udword NbPts = GenerateConvex(Pts, 8, 8, 2.0f, 3.0f, 2.0f);
		ASSERT(NbPts==16);

		PINT_CONVEX_CREATE ConvexCreate;
		ConvexCreate.mNbVerts	= 16;
		ConvexCreate.mVerts		= Pts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(16, Pts);

		const udword NbLayers = 12;
		for(udword j=0;j<NbLayers;j++)
		{
			const float Scale = 6.0f;
			for(udword y=0;y<NbY-j;y++)
			{
				for(udword x=0;x<NbX-j;x++)
				{
					const float xf = (float(j)*0.5f + float(x) - float(NbX)*0.5f)*Scale;
					const float yf = (float(j)*0.5f + float(y) - float(NbY)*0.5f)*Scale;

					const Point pos(xf, 0.0f + 2.0f * float(j), yf);

					PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
					ASSERT(Handle);
				}
			}
		}
		return true;
	}

END_TEST(ConvexStack2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexStack3 = "Convex stack scene from the PhysX SDK. There are 8 layers of 16*16 convexes, i.e. 2048 convexes in total. Each convex has 56 vertices.";

START_TEST(ConvexStack3, CATEGORY_PERFORMANCE, gDesc_ConvexStack3)

	virtual	void ConvexStack3::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(75.47f, 20.40f, 73.86f), Point(-0.69f, -0.25f, -0.68f));
	}

	virtual bool ConvexStack3::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 16;
		const udword NbY = 16;
		const udword NbLayers = 8;
		const udword NbInsideCirclePts = 28;
		const udword NbOutsideCirclePts = 28;

		return CreateTestScene_ConvexStack_Generic(pint, NbX, NbY, NbLayers, NbInsideCirclePts, NbOutsideCirclePts);
	}

END_TEST(ConvexStack3)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfLargeConvexes = "Pile of large convexes. The convex is the hull computed around 32 randomly generated vertices.";

START_TEST(PileOfLargeConvexes, CATEGORY_PERFORMANCE, gDesc_PileOfLargeConvexes)

	virtual	void PileOfLargeConvexes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(88.12f, 62.03f, 92.27f), Point(-0.66f, -0.26f, -0.71f));
	}

	virtual bool PileOfLargeConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 5;
		const udword NbY = 5;
		const udword NbLayers = 20;
		const float Amplitude = 4.0f;
		const udword NbRandomPts = 32;
		GenerateConvexPile(pint, NbX, NbY, NbLayers, Amplitude, NbRandomPts);
		return true;
	}

END_TEST(PileOfLargeConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HugePileOfLargeConvexes = "Huge pile of large convexes. The convex is the hull computed around 32 randomly generated vertices.";

START_TEST(HugePileOfLargeConvexes, CATEGORY_PERFORMANCE, gDesc_HugePileOfLargeConvexes)

	virtual	void HugePileOfLargeConvexes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(164.90f, 67.88f, 164.53f), Point(-0.70f, 0.07f, -0.71f));
	}

	virtual bool HugePileOfLargeConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 8;
		const udword NbY = 8;
		const udword NbLayers = 80;
		const float Amplitude = 4.0f;
		const udword NbRandomPts = 32;
		GenerateConvexPile(pint, NbX, NbY, NbLayers, Amplitude, NbRandomPts);
		return true;
	}

END_TEST(HugePileOfLargeConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfMediumConvexes = "Pile of medium convexes. The convex is the hull computed around 16 randomly generated vertices.";

START_TEST(PileOfMediumConvexes, CATEGORY_PERFORMANCE, gDesc_PileOfMediumConvexes)

	virtual	void PileOfMediumConvexes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.45f, -0.68f));
	}

	virtual bool PileOfMediumConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 5;
		const udword NbY = 5;
		const udword NbLayers = 20;
		const float Amplitude = 2.0f;
		const udword NbRandomPts = 16;
		GenerateConvexPile(pint, NbX, NbY, NbLayers, Amplitude, NbRandomPts);
		return true;
	}

END_TEST(PileOfMediumConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfSmallConvexes = "Pile of small convexes. The convex is the hull computed around 5 randomly generated vertices.";

START_TEST(PileOfSmallConvexes, CATEGORY_PERFORMANCE, gDesc_PileOfSmallConvexes)

	virtual	void PileOfSmallConvexes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(49.77f, 26.35f, 48.67f), Point(-0.68f, -0.35f, -0.64f));
	}

	virtual bool PileOfSmallConvexes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 8;
		const udword NbY = 8;
		const udword NbLayers = 20;
		const float Amplitude = 2.0f;
		const udword NbRandomPts = 5;
		GenerateConvexPile(pint, NbX, NbY, NbLayers, Amplitude, NbRandomPts);
		return true;
	}

END_TEST(PileOfSmallConvexes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointNet = "Stress test for spherical joints. The net is made of 40*40 spheres, all connected by spherical joints.";

START_TEST(SphericalJointNet, CATEGORY_PERFORMANCE, gDesc_SphericalJointNet)

	virtual void SphericalJointNet::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-50.0f, -50.0f, -50.0f), Point(50.0f, 50.0f, 50.0f));
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.47f, -0.66f));
	}

	virtual bool SphericalJointNet::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphericalJoints || !caps.mSupportRigidBodySimulation)
			return false;

		{
			PINT_SPHERE_CREATE ShapeDesc;
			ShapeDesc.mRadius	= 16.0f;
			ShapeDesc.mRenderer	= CreateSphereRenderer(16.0f);

			CreateStaticObject(pint, &ShapeDesc, Point(0.0f, 10.0f, 0.0f));
		}

		const udword NbX = 40;
		const udword NbY = 40;
		const float Scale = 40.0f;
		const float Altitude = 30.0f;

		PINT_SPHERE_CREATE ShapeDesc;
		ShapeDesc.mRadius	= 1.0f;
		ShapeDesc.mRenderer	= CreateSphereRenderer(1.0f);

		PintObjectHandle* Handles = (PintObjectHandle*)StackAlloc(NbX*NbY*sizeof(PintObjectHandle*));
		udword Index = 0;
		for(udword y=0;y<NbY;y++)
		{
			const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
			for(udword x=0;x<NbX;x++)
			{
				const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &ShapeDesc;
				ObjectDesc.mMass			= 1.0f;
				ObjectDesc.mPosition.x		= CoeffX * Scale;
				ObjectDesc.mPosition.y		= Altitude;
				ObjectDesc.mPosition.z		= CoeffY * Scale;
				Handles[Index++] = CreatePintObject(pint, ObjectDesc);
			}
		}

		const float CoeffX0 = 2.0f * ((0.0f/float(NbX-1)) - 0.5f);
		const float CoeffX1 = 2.0f * ((1.0f/float(NbX-1)) - 0.5f);
		const float CenterX = (CoeffX1 - CoeffX0)*0.5f*Scale;

		for(udword y=0;y<NbY;y++)
		{
			const udword Base = y*NbX;
			for(udword x=0;x<NbX-1;x++)
			{
				PINT_SPHERICAL_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[Base+x];
				Desc.mObject1		= Handles[Base+x+1];
				Desc.mLocalPivot0	= Point(CenterX, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(-CenterX, 0.0f, 0.0f);
				pint.CreateJoint(Desc);
			}
		}

		const float CoeffY0 = 2.0f * ((0.0f/float(NbY-1)) - 0.5f);
		const float CoeffY1 = 2.0f * ((1.0f/float(NbY-1)) - 0.5f);
		const float CenterY = (CoeffY1 - CoeffY0)*0.5f*Scale;

		for(udword x=0;x<NbX;x++)
		{
			const udword Base = x;
			for(udword y=0;y<NbY-1;y++)
			{
				PINT_SPHERICAL_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[x+(y*NbX)];
				Desc.mObject1		= Handles[x+(y+1)*NbX];
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, CenterY);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, -CenterY);
				pint.CreateJoint(Desc);
			}
		}

		mCreateDefaultEnvironment = false;

		return true;
	}

END_TEST(SphericalJointNet)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointNet2 = "Stress test for spherical joints. The net is made of 40*40 spheres, all connected by spherical joints.";

START_TEST(SphericalJointNet2, CATEGORY_PERFORMANCE, gDesc_SphericalJointNet2)

	virtual void SphericalJointNet2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-50.0f, -50.0f, -50.0f), Point(50.0f, 50.0f, 50.0f));
		desc.mCamera[0] = CameraPose(Point(60.07f, 35.87f, 60.52f), Point(-0.64f, -0.40f, -0.65f));
	}

	virtual bool SphericalJointNet2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphericalJoints || !caps.mSupportRigidBodySimulation)
			return false;

		if(0)
		{
			PINT_SPHERE_CREATE ShapeDesc;
			ShapeDesc.mRadius	= 16.0f;
			ShapeDesc.mRenderer	= CreateSphereRenderer(16.0f);
			CreateSimpleObject(pint, &ShapeDesc, 100.0f, Point(0.0f, 30.0f, 0.0f));
		}

		{
			const float BoxSide = 20.0f;
			const float BoxDepth = 20.0f;

			const float SphereRadius = 3.0f;
			const float CapsuleRadius = 3.0f;
			const float HalfHeight = 3.0f;
			const udword NbLayers = 2;
			const udword NbX = 4;
			const udword NbY = 4;
			float yy = 30.0f;
			BasicRandom Rnd(42);

			PINT_SPHERE_CREATE SphereDesc;
			SphereDesc.mRadius		= SphereRadius;
			SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mExtents	= Point(CapsuleRadius, CapsuleRadius, CapsuleRadius);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_CAPSULE_CREATE CapsuleDesc;
			CapsuleDesc.mRadius		= CapsuleRadius;
			CapsuleDesc.mHalfHeight	= HalfHeight;
			CapsuleDesc.mRenderer	= CreateCapsuleRenderer(CapsuleRadius, HalfHeight*2.0f);

			for(udword k=0;k<NbLayers;k++)
			{
				for(udword y=0;y<NbY;y++)
				{
					const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
					for(udword x=0;x<NbX;x++)
					{
						const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

						const float RandomX = 1.0f * Rnd.RandomFloat();
						const float RandomY = 1.0f * Rnd.RandomFloat();

						const udword Index = Rnd.Randomize() % 3;

						PINT_OBJECT_CREATE ObjectDesc;
						if(Index==0)
							ObjectDesc.mShapes	= &SphereDesc;
						else if(Index==1)
							ObjectDesc.mShapes	= &BoxDesc;
						else if(Index==2)
							ObjectDesc.mShapes	= &CapsuleDesc;
						ObjectDesc.mMass		= 1.0f;
						ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - SphereRadius - BoxSide*2.0f);
						ObjectDesc.mPosition.y	= yy;
						ObjectDesc.mPosition.z	= RandomY + CoeffY * (BoxDepth - SphereRadius - BoxSide*2.0f);

						UnitRandomQuat(ObjectDesc.mRotation, Rnd);

						CreatePintObject(pint, ObjectDesc);
					}
				}
				yy += HalfHeight*10.0f;
			}
		}


		const udword NbX = 40;
		const udword NbY = 40;
		const float Scale = 40.0f;
		const float Altitude = 0.0f;

		PINT_SPHERE_CREATE ShapeDesc;
		ShapeDesc.mRadius	= 1.0f;
		ShapeDesc.mRenderer	= CreateSphereRenderer(1.0f);

		PintObjectHandle* Handles = (PintObjectHandle*)StackAlloc(NbX*NbY*sizeof(PintObjectHandle*));
		udword Index = 0;
		for(udword y=0;y<NbY;y++)
		{
			const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
			for(udword x=0;x<NbX;x++)
			{
				const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &ShapeDesc;
				ObjectDesc.mMass			= 1.0f;
				if(x==0 && y==0)
					ObjectDesc.mMass		= 0.0f;
				if(x==0 && y==NbY-1)
					ObjectDesc.mMass		= 0.0f;
				if(x==NbX-1 && y==0)
					ObjectDesc.mMass		= 0.0f;
				if(x==NbX-1 && y==NbY-1)
					ObjectDesc.mMass		= 0.0f;
				ObjectDesc.mPosition.x		= CoeffX * Scale;
				ObjectDesc.mPosition.y		= Altitude;
				ObjectDesc.mPosition.z		= CoeffY * Scale;
				Handles[Index++] = CreatePintObject(pint, ObjectDesc);
			}
		}

		const float CoeffX0 = 2.0f * ((0.0f/float(NbX-1)) - 0.5f);
		const float CoeffX1 = 2.0f * ((1.0f/float(NbX-1)) - 0.5f);
		const float CenterX = (CoeffX1 - CoeffX0)*0.5f*Scale;

		for(udword y=0;y<NbY;y++)
		{
			const udword Base = y*NbX;
			for(udword x=0;x<NbX-1;x++)
			{
				PINT_SPHERICAL_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[Base+x];
				Desc.mObject1		= Handles[Base+x+1];
				Desc.mLocalPivot0	= Point(CenterX, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(-CenterX, 0.0f, 0.0f);
				pint.CreateJoint(Desc);
			}
		}

		const float CoeffY0 = 2.0f * ((0.0f/float(NbY-1)) - 0.5f);
		const float CoeffY1 = 2.0f * ((1.0f/float(NbY-1)) - 0.5f);
		const float CenterY = (CoeffY1 - CoeffY0)*0.5f*Scale;

		for(udword x=0;x<NbX;x++)
		{
			const udword Base = x;
			for(udword y=0;y<NbY-1;y++)
			{
				PINT_SPHERICAL_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[x+(y*NbX)];
				Desc.mObject1		= Handles[x+(y+1)*NbX];
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, CenterY);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, -CenterY);
				pint.CreateJoint(Desc);
			}
		}

		mCreateDefaultEnvironment = false;

		return true;
	}

END_TEST(SphericalJointNet2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexGalore = "12 layers of 12*12 convexes. Each convex is randomly chosen in a set of 14 predefined convex objects, of various complexities.";

START_TEST(ConvexGalore, CATEGORY_PERFORMANCE, gDesc_ConvexGalore)

	virtual	void ConvexGalore::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(45.18f, 21.90f, 43.90f), Point(-0.69f, -0.21f, -0.69f));
	}

	virtual bool ConvexGalore::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		PINT_CONVEX_CREATE ConvexCreate[14];
		MyConvex C[14];
		for(udword i=0;i<14;i++)
		{
			C[i].LoadFile(i);
			ConvexCreate[i].mNbVerts	= C[i].mNbVerts;
			ConvexCreate[i].mVerts		= C[i].mVerts;
			ConvexCreate[i].mRenderer	= CreateConvexRenderer(ConvexCreate[i].mNbVerts, ConvexCreate[i].mVerts);
		}

		const float Amplitude = 1.5f;
		const udword NbLayers = 12;
		const udword NbX = 12;
		const udword NbY = 12;
		BasicRandom Rnd(42);
		for(udword j=0;j<NbLayers;j++)
		{
			const float Scale = 4.0f;
			for(udword y=0;y<NbY;y++)
			{
				for(udword x=0;x<NbX;x++)
				{
					const float xf = (float(x)-float(NbX)*0.5f)*Scale;
					const float yf = (float(y)-float(NbY)*0.5f)*Scale;

					const Point pos = Point(xf, Amplitude + (Amplitude * 2.0f * float(j)), yf);

					const udword Index = Rnd.Randomize() % 14;

					PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate[Index], pos);
					ASSERT(Handle);
				}
			}
		}
		return true;
	}

END_TEST(ConvexGalore)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexGalore2 = "36 layers of 12*12 convexes. Each convex is randomly chosen in a set of 14 predefined convex objects, of various complexities.";

START_TEST(ConvexGalore2, CATEGORY_PERFORMANCE, gDesc_ConvexGalore2)

	virtual	void ConvexGalore2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(37.22f, 13.73f, 33.16f), Point(-0.76f, -0.15f, -0.63f));
	}

	virtual bool ConvexGalore2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		PINT_CONVEX_CREATE ConvexCreate[14];
		MyConvex C[14];
		for(udword i=0;i<14;i++)
		{
			C[i].LoadFile(i);
			ConvexCreate[i].mNbVerts	= C[i].mNbVerts;
			ConvexCreate[i].mVerts		= C[i].mVerts;
			ConvexCreate[i].mRenderer	= CreateConvexRenderer(ConvexCreate[i].mNbVerts, ConvexCreate[i].mVerts);
		}

		const float Amplitude = 1.5f;
		const udword NbLayers = 36;
		const udword NbX = 12;
		const udword NbY = 12;
		BasicRandom Rnd(42);
		for(udword j=0;j<NbLayers;j++)
		{
			const float Scale = 4.0f;
			for(udword y=0;y<NbY;y++)
			{
				for(udword x=0;x<NbX;x++)
				{
					const float xf = (float(x)-float(NbX)*0.5f)*Scale;
					const float yf = (float(y)-float(NbY)*0.5f)*Scale;

					const Point pos = Point(xf, Amplitude + (Amplitude * 2.0f * float(j)), yf);

					const udword Index = Rnd.Randomize() % 14;

					PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate[Index], pos);
					ASSERT(Handle);
				}
			}
		}
		return true;
	}

END_TEST(ConvexGalore2)

///////////////////////////////////////////////////////////////////////////////

static void SetupConvexClash(Pint& pint, udword i)
{
	const float Altitude = 10.0f;

	PINT_CONVEX_CREATE ConvexCreate;
	MyConvex C;
	C.LoadFile(i);

	ConvexCreate.mNbVerts	= C.mNbVerts;
	ConvexCreate.mVerts		= C.mVerts;
	ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

	const float Amplitude = 1.5f;
	const udword NbX = 32;
	const udword NbY = 32;

	const float Scale = 2.5f;
	for(udword y=0;y<NbY;y++)
	{
		for(udword x=0;x<NbX;x++)
		{
			const float xf = (float(x)-float(NbX)*0.5f)*Scale;
			const float yf = (float(y)-float(NbY)*0.5f)*Scale;

//			const Point pos = Point(xf, Altitude + 20.0f, yf);
			const Point pos = Point(xf, Altitude + 30.0f, yf);

			PintObjectHandle Handle = CreateStaticObject(pint, &ConvexCreate, pos);
			ASSERT(Handle);
		}
	}

	for(udword y=0;y<NbY;y++)
	{
		for(udword x=0;x<NbX;x++)
		{
			const float xf = (float(x)-float(NbX)*0.5f)*Scale;
			const float yf = (float(y)-float(NbY)*0.5f)*Scale;

//			const Point pos = Point(xf, Altitude + 20.0f, yf);
			const Point pos = Point(xf, Altitude + 50.0f, yf);

			PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
			ASSERT(Handle);
		}
	}
}

static const char* gDesc_ConvexClash = "32*32 dynamic convexes are falling towards 32*32 static convexes. This is a stress test for both convex contact generation \
and the broad phase.";

#define IMPLEMENT_CONVEX_CRASH(name, x)															\
	class name : public TestBase																\
	{																							\
		public:																					\
								name()						{								}	\
		virtual					~name()						{								}	\
		virtual	const char*		GetName()			const	{ return #name;					}	\
		virtual	const char*		GetDescription()	const	{ return gDesc_ConvexClash;		}	\
		virtual	TestCategory	GetCategory()		const	{ return CATEGORY_PERFORMANCE;	}	\
		virtual	bool			Setup(Pint& pint, const PintCaps& caps)							\
		{																						\
			if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)						\
				return false;																	\
																								\
			SetupConvexClash(pint, x);															\
			return true;																		\
		}																						\
	}name;

IMPLEMENT_CONVEX_CRASH(ConvexClash0, 0)	
IMPLEMENT_CONVEX_CRASH(ConvexClash1, 1)	
IMPLEMENT_CONVEX_CRASH(ConvexClash2, 2)	
IMPLEMENT_CONVEX_CRASH(ConvexClash3, 3)	
IMPLEMENT_CONVEX_CRASH(ConvexClash4, 4)	
IMPLEMENT_CONVEX_CRASH(ConvexClash5, 5)	
IMPLEMENT_CONVEX_CRASH(ConvexClash6, 6)	
IMPLEMENT_CONVEX_CRASH(ConvexClash7, 7)	
IMPLEMENT_CONVEX_CRASH(ConvexClash8, 8)	
IMPLEMENT_CONVEX_CRASH(ConvexClash9, 9)	
IMPLEMENT_CONVEX_CRASH(ConvexClash10, 10)	
IMPLEMENT_CONVEX_CRASH(ConvexClash11, 11)	
IMPLEMENT_CONVEX_CRASH(ConvexClash12, 12)	
IMPLEMENT_CONVEX_CRASH(ConvexClash13, 13)	

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SpheresOnPlanarMesh = "32*32 dynamic spheres on a tesselated planar mesh.";

START_TEST(SpheresOnPlanarMesh, CATEGORY_PERFORMANCE, gDesc_SpheresOnPlanarMesh)

	virtual bool SpheresOnPlanarMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		CreatePlanarMesh(*this, pint, Altitude);

		mCreateDefaultEnvironment = false;

		const float Radius = 0.5f;
		const udword NbX = 32;
		const udword NbY = 32;
		return GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Altitude + Radius*2.0f, 30.0f, 30.0f);
	}

END_TEST(SpheresOnPlanarMesh)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxesOnPlanarMesh = "32*32 dynamic boxes on a tesselated planar mesh.";

START_TEST(BoxesOnPlanarMesh, CATEGORY_PERFORMANCE, gDesc_BoxesOnPlanarMesh)

	virtual bool BoxesOnPlanarMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		CreatePlanarMesh(*this, pint, Altitude);

		mCreateDefaultEnvironment = false;

		const float Radius = 0.5f;
		const udword NbX = 32;
		const udword NbY = 32;
		return GenerateArrayOfBoxes(pint, Point(Radius, Radius, Radius), NbX, NbY, Altitude + Radius*2.0f, 30.0f);
	}

END_TEST(BoxesOnPlanarMesh)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SmallConvexesOnPlanarMesh = "32*32 dynamic convexes (of low complexity) on a tesselated planar mesh.";

START_TEST(SmallConvexesOnPlanarMesh, CATEGORY_PERFORMANCE, gDesc_SmallConvexesOnPlanarMesh)

	virtual bool SmallConvexesOnPlanarMesh::Setup(Pint& pint, const PintCaps& caps)
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

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		mCreateDefaultEnvironment = false;

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 3.0f;
		const float Scale = 35.0f;
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Scale, Scale);
	}

END_TEST(SmallConvexesOnPlanarMesh)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BigConvexesOnPlanarMesh = "32*32 dynamic convexes (of high complexity) on a tesselated planar mesh.";

START_TEST(BigConvexesOnPlanarMesh, CATEGORY_PERFORMANCE, gDesc_BigConvexesOnPlanarMesh)

	virtual bool BigConvexesOnPlanarMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		CreatePlanarMesh(*this, pint, Altitude);

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

		mCreateDefaultEnvironment = false;

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 3.0f;
		const float Scale = 35.0f;
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Scale, Scale);
	}

END_TEST(BigConvexesOnPlanarMesh)

///////////////////////////////////////////////////////////////////////////////

class BinReader
{
public:
	BinReader(const char* data) : mData(data)	{}

	int readDword()
	{
		const int* tmp = (const int*)mData;
		int Value = *tmp;
		mData += sizeof(int);
		return Value;
	}

	short readWord()
	{
		const short* tmp = (const short*)mData;
		short Value = *tmp;
		mData += sizeof(short);
		return Value;
	}

	float readFloat()
	{
		const float* tmp = (const float*)mData;
		float Value = *tmp;
		mData += sizeof(float);
		return Value;
	}
	const char*	mData;
};

struct Bone
{
	udword				mID;
	Point				mPivot;
//	PxRigidDynamic*		mBody;
	PintObjectHandle	mBody;
	PR					mPose;
};

class TestRagdoll
{
public:
	enum { NUM_BONES = 19 };

	TestRagdoll();
	~TestRagdoll();

	bool				Init(Pint& pint, udword tag, //PxPhysics& physics, const PxTolerancesScale& scale, PxScene* scene, PxMaterial* material,
							const Point& offset, PintCollisionGroup group, bool useCompound);

	Bone				mBones[NUM_BONES];
//	PxJoint*			mJoints[NUM_BONES-1];
	PintJointHandle*	mJoints[NUM_BONES-1];
//	PxAggregate*		mAggregate;

	const Bone*			FindBoneByName(int name)	const;
};

static /*const*/ float gRagdollScale = 0.1f;

static const unsigned int gRagdollData[]={
	0x00000013, 
	0x01c7d2f8, 0x41afcf8e, 0x40f61aee, 0xc3173a20, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41afcdbc, 0x41031a76, 0xc3171df7, 0x3c5c4318, 0x3f4df85f, 0xbf17fcea, 0x3f7aaa38, 0x3de0bf0b, 
	0x3e2efcb6, 0x3e4f816e, 0xbf1568a0, 0xbf494cd1, 0, 0x3f962963, 0x3f82a747, 0x3facfaae, 0x41200000, 0x01c211d0, 0x41b19fcc, 0x4115b5da, 0xc317093f, 0x3f000000, 0x3f000000, 0x41200000, 
	0x0000001e, 0x41b2e15d, 0x411ead21, 0xc31710d4, 0x3dc82982, 0x3f538bba, 0xbf0dface, 0x3f7e8ea1, 0xbd6deaec, 0x3db59f18, 0x3d283688, 0xbf0f6602, 0xbf53ce8b, 0, 0x3f85e78f, 0x3f3fbd05, 
	0x3f72ad19, 0x41200000, 0x00cb87d8, 0x41b2a164, 0x412a6224, 0xc316fb60, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b46871, 0x4132a8ec, 0xc3171ea4, 0x3de9e76c, 0x3f41b1eb, 0xbf24d011, 
	0x3f7da826, 0xbe08e43e, 0x3c98e5c0, 0xbd93578c, 0xbf23d9a2, 0xbf43d4a7, 0, 0x3fff4dd5, 0x3fa9a6c6, 0x3fd64ffc, 0x41200000, 0x02e2f720, 0x41b44d4e, 0x41477671, 0xc3171d0b, 0x3f000000, 
	0x3f000000, 0x41200000, 0x0000001e, 0x41b60d03, 0x414fb8e9, 0xc317482b, 0x3ebb5b9a, 0x3f2c083e, 0xbf24d17f, 0x3f62aa31, 0xbeedcb3d, 0x3c975790, 0xbe92bdb3, 0xbf13a99b, 0xbf43d3c3, 0, 
	0x3f47be8b, 0x3f0e353f, 0x3f0259e7, 0x41200000, 0x01c0ab78, 0x41b71239, 0x4154dd09, 0xc3176272, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b75e37, 0x415ff130, 0xc3174207, 0x3cb748e0, 
	0x3d863664, 0xbf7f6299, 0x3f7a33e7, 0x3e559708, 0x3d11e698, 0x3e57775c, 0xbf79ce60, 0xbd733a30, 0, 0x3fa45e84, 0x3fac9056, 0x3fa12de9, 0x41200000, 0x02da42c0, 0x41b08d5b, 0x4146748c, 
	0xc31790f4, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41acf2e7, 0x413f6bf5, 0xc317f575, 0xbf33bfaf, 0xbf32a077, 0xbe113d9a, 0xbea38ede, 0x3e05cffc, 0x3f7044b0, 0xbf22e7c5, 0x3f344d8b, 
	0xbea119dc, 0, 0x3f4ef6b5, 0x3f3d4831, 0x3f647526, 0x41200000, 0x02d7d660, 0x41aa6215, 0x4140d788, 0xc31843dd, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41ab042b, 0x413810f2, 
	0xc3198628, 0x3d835938, 0xbf297a1e, 0xbf3f2a06, 0xbec3b797, 0xbf351e95, 0x3f182af1, 0xbf6bfcb1, 0x3e7d431f, 0xbe98ccaf, 0, 0x3fe367b7, 0x3f1b7cb7, 0x3ee6f860, 0x41200000, 0x01bb7670, 
	0x41aba7c0, 0x4131ad4c, 0xc31a8cfe, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41aecc44, 0x4133f9f3, 0xc31b538e, 0x3f07a8b7, 0xbecdcdb6, 0xbf3f2a01, 0x3e856301, 0xbf42c337, 0x3f182aee, 
	0xbf4e9a2a, 0xbf02704b, 0xbe98cc99, 0, 0x3fa20510, 0x3f2bd0a1, 0x3f10131b, 0x41200000, 0x02e5a990, 0x41b3fe97, 0x4139e064, 0xc31c2367, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 
	0x41b89ee4, 0x413b9ea2, 0xc31c4fb8, 0x3f55e05b, 0x3ef57505, 0xbe898ec6, 0x3e4a0551, 0xbf37c9ba, 0xbf2ae648, 0xbf034f1b, 0x3f0135aa, 0xbf31c251, 0, 0x3f314730, 0x3edcf402, 0x3ef8df37, 
	0x41200000, 0x01c78ae0, 0x41b7377a, 0x414612a0, 0xc3169391, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b9e44c, 0x413ed265, 0xc31609a9, 0x3f008136, 0xbf4f596a, 0xbe9b49fe, 0xbeb0b327, 
	0x3e09301d, 0xbf6dcf74, 0x3f4b04d8, 0x3f122b98, 0xbe595fc4, 0, 0x3f4ef6ae, 0x3f3d482e, 0x3f64752b, 0x41200000, 0x02e7b820, 0x41bba084, 0x41400226, 0xc315b49a, 0x3f000000, 0x3f000000, 
	0x41200000, 0x0000001e, 0x41c2385f, 0x4131763f, 0xc31653e7, 0x3f14f045, 0x3e37f8c2, 0xbf4b122e, 0xbf29d990, 0xbeeb2794, 0xbf1733d8, 0xbef0ddec, 0x3f5eb382, 0xbe178f33, 0, 0x3fe367b4, 
	0x3f1b7cb8, 0x3ee6f85b, 0x41200000, 0x01b03a10, 0x41c72ad7, 0x4125afe2, 0xc316df37, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41c65566, 0x412c54a1, 0xc317a4bc, 0xbe7b440c, 0x3f0ea8f2, 
	0xbf4b1224, 0x3f085505, 0xbf1b32fd, 0xbf1733d2, 0xbf4f5f41, 0xbf113f4b, 0xbe178f06, 0, 0x3fa204a7, 0x3f2bcfe7, 0x3f101362, 0x41200000, 0x02dc4670, 0x41c34e4b, 0x413672c3, 0xc3187723, 
	0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41bfa089, 0x413b660c, 0xc318b84e, 0xbf1de048, 0xbf3f81d9, 0xbe7aee4c, 0x3f011ba6, 0xbf1d4f24, 0x3f1b4e90, 0xbf1abb03, 0x3e804849, 0x3f419924, 
	0, 0x3f314730, 0x3edcf40b, 0x3ef8df46, 0x41200000, 0x01c91298, 0x41ac2296, 0x40fa5649, 0xc317d5e8, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b022a8, 0x40db32fd, 0xc318fead, 
	0x3eb5712d, 0x3f011740, 0x3f4998b1, 0xbf1a161f, 0x3f447ab1, 0xbe61e495, 0xbf3733b2, 0xbecaa80b, 0x3f1353fd, 0, 0x4013a5a4, 0x3f909b1c, 0x3f654bde, 0x41200000, 0x01b72a48, 0x41b73cbe, 
	0x40aee909, 0xc31aa357, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b3ca9f, 0x406a6229, 0xc31ae39c, 0xbe3edbd8, 0x3f16647e, 0x3f4998a7, 0xbf79ab09, 0xbc60bb00, 0xbe61e49a, 0xbdf3496c, 
	0xbf4f22e9, 0x3f135402, 0, 0x40243acf, 0x3f930583, 0x3f64436a, 0x41200000, 0x01add448, 0x41b17700, 0x3fd86010, 0xc31b1912, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b2a2eb, 
	0x3f7a46de, 0xc31b9041, 0xbe14e0d0, 0x3f0325dd, 0x3f58ae48, 0xbf77220d, 0xbe858450, 0xbc02da80, 0x3e5dd530, 0xbf517935, 0x3f085002, 0, 0x3f797f50, 0x3fa81d25, 0x3efa2532, 0x41200000, 
	0x00c49c28, 0x41b37c86, 0x40f1df94, 0xc3169e57, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41bb0a32, 0x40c8d2bd, 0xc3168f52, 0x3f19a53a, 0x3f409f96, 0x3e8aed8f, 0xbf4cc351, 0x3f10fcb4, 
	0x3e4b712c, 0xbb8937e0, 0xbeac2c4c, 0x3f7116ce, 0, 0x4013a5a3, 0x3f909b1c, 0x3f654bd5, 0x41200000, 0x02dc3ad0, 0x41c64a04, 0x408da3d2, 0xc316a28a, 0x3f000000, 0x3f000000, 0x41200000, 
	0x0000001e, 0x41bc3f79, 0x404dfd9c, 0xc315f40f, 0xbf2bcdb6, 0x3f309e32, 0x3e8aed87, 0xbf298d7b, 0xbf38ef2e, 0x3e4b711c, 0x3eaa8a68, 0xbd3dfc4c, 0x3f7116d0, 0, 0x40243acf, 0x3f930585, 
	0x3f644368, 0x41200000, 0x00c06480, 0x41b1813a, 0x3fee5e20, 0xc315586f, 0x3f000000, 0x3f000000, 0x41200000, 0x0000001e, 0x41b5b2c6, 0x3f99035b, 0xc315303d, 0xbe4c3cf4, 0x3f74dbed, 0x3e5a161b, 
	0xbf711426, 0xbe7d1d44, 0x3e69aaba, 0x3e8ab3f9, 0xbe1ec503, 0x3f7335a2, 0, 0x3f797f52, 0x3fa81d27, 0x3efa2570, 0x41200000, 0x02e2f720, 0x01c0ab78, 0x41b71239, 0x4154dd09, 0xc3176272, 
	0xbf24d17f, 0x3c975790, 0xbf43d3c3, 0x00cb87d8, 0x02e2f720, 0x41b44d4e, 0x41477671, 0xc3171d0b, 0xbf24d011, 0x3c98e5c0, 0xbf43d4a7, 0x01c211d0, 0x00cb87d8, 0x41b2a164, 0x412a6224, 0xc316fb60, 
	0xbf0dface, 0x3db59f18, 0xbf53ce8b, 0x01c7d2f8, 0x01c211d0, 0x41b19fcc, 0x4115b5da, 0xc317093f, 0xbf17fcea, 0x3e2efcb6, 0xbf494cd1, 0x01c91298, 0x01c7d2f8, 0x41ac2296, 0x40fa5649, 0xc317d5e8, 
	0x3f4998b1, 0xbe61e495, 0x3f1353fd, 0x01b72a48, 0x01c91298, 0x41b73cbe, 0x40aee909, 0xc31aa357, 0x3f4998a7, 0xbe61e49a, 0x3f135402, 0x01add448, 0x01b72a48, 0x41b17700, 0x3fd86010, 0xc31b1912, 
	0x3f58ae48, 0xbc02da80, 0x3f085002, 0x00c49c28, 0x01c7d2f8, 0x41b37c86, 0x40f1df94, 0xc3169e57, 0x3e8aed8f, 0x3e4b712c, 0x3f7116ce, 0x02dc3ad0, 0x00c49c28, 0x41c64a04, 0x408da3d2, 0xc316a28a, 
	0x3e8aed87, 0x3e4b711c, 0x3f7116d0, 0x00c06480, 0x02dc3ad0, 0x41b1813a, 0x3fee5e20, 0xc315586f, 0x3e5a161b, 0x3e69aaba, 0x3f7335a2, 0x02da42c0, 0x00cb87d8, 0x41b08d5b, 0x4146748c, 0xc31790f4, 
	0xbe113d9a, 0x3f7044b0, 0xbea119dc, 0x02d7d660, 0x02da42c0, 0x41aa6215, 0x4140d788, 0xc31843dd, 0xbf3f2a06, 0x3f182af1, 0xbe98ccaf, 0x01bb7670, 0x02d7d660, 0x41aba7c0, 0x4131ad4c, 0xc31a8cfe, 
	0xbf3f2a01, 0x3f182aee, 0xbe98cc99, 0x02e5a990, 0x01bb7670, 0x41b3fe97, 0x4139e064, 0xc31c2367, 0xbe898ec6, 0xbf2ae648, 0xbf31c251, 0x01c78ae0, 0x00cb87d8, 0x41b7377a, 0x414612a0, 0xc3169391, 
	0xbe9b49fe, 0xbf6dcf74, 0xbe595fc4, 0x02e7b820, 0x01c78ae0, 0x41bba084, 0x41400226, 0xc315b49a, 0xbf4b122e, 0xbf1733d8, 0xbe178f33, 0x01b03a10, 0x02e7b820, 0x41c72ad7, 0x4125afe2, 0xc316df37, 
	0xbf4b1224, 0xbf1733d2, 0xbe178f06, 0x02dc4670, 0x01b03a10, 0x41c34e4b, 0x413672c3, 0xc3187723, 0xbe7aee4c, 0x3f1b4e90, 0x3f419924, 0xdeadbabe, 0x01c0ab78, 0x02e2f720, 0x01c0ab78, 0x00cb87d8, 
	0x01c0ab78, 0x01c211d0, 0x01c0ab78, 0x01c7d2f8, 0x01c0ab78, 0x01c78ae0, 0x01c0ab78, 0x02e7b820, 0x01c0ab78, 0x01b03a10, 0x01c0ab78, 0x02dc4670, 0x01c0ab78, 0x00c49c28, 0x01c0ab78, 0x02dc3ad0, 
	0x01c0ab78, 0x00c06480, 0x01c0ab78, 0x02da42c0, 0x01c0ab78, 0x02d7d660, 0x01c0ab78, 0x01bb7670, 0x01c0ab78, 0x02e5a990, 0x01c0ab78, 0x01c91298, 0x01c0ab78, 0x01b72a48, 0x01c0ab78, 0x01add448, 
	0x02e2f720, 0x00cb87d8, 0x02e2f720, 0x01c211d0, 0x02e2f720, 0x01c7d2f8, 0x02e2f720, 0x01c78ae0, 0x02e2f720, 0x02e7b820, 0x02e2f720, 0x01b03a10, 0x02e2f720, 0x02dc4670, 0x02e2f720, 0x00c49c28, 
	0x02e2f720, 0x02dc3ad0, 0x02e2f720, 0x00c06480, 0x02e2f720, 0x02da42c0, 0x02e2f720, 0x02d7d660, 0x02e2f720, 0x01bb7670, 0x02e2f720, 0x02e5a990, 0x02e2f720, 0x01c91298, 0x02e2f720, 0x01b72a48, 
	0x02e2f720, 0x01add448, 0x00cb87d8, 0x01c211d0, 0x00cb87d8, 0x01c7d2f8, 0x00cb87d8, 0x01c78ae0, 0x00cb87d8, 0x02e7b820, 0x00cb87d8, 0x01b03a10, 0x00cb87d8, 0x02dc4670, 0x00cb87d8, 0x00c49c28, 
	0x00cb87d8, 0x02dc3ad0, 0x00cb87d8, 0x00c06480, 0x00cb87d8, 0x02da42c0, 0x00cb87d8, 0x02d7d660, 0x00cb87d8, 0x01bb7670, 0x00cb87d8, 0x02e5a990, 0x00cb87d8, 0x01c91298, 0x00cb87d8, 0x01b72a48, 
	0x00cb87d8, 0x01add448, 0x01c211d0, 0x01c7d2f8, 0x01c211d0, 0x01c78ae0, 0x01c211d0, 0x02e7b820, 0x01c211d0, 0x01b03a10, 0x01c211d0, 0x02dc4670, 0x01c211d0, 0x00c49c28, 0x01c211d0, 0x02dc3ad0, 
	0x01c211d0, 0x00c06480, 0x01c211d0, 0x02da42c0, 0x01c211d0, 0x02d7d660, 0x01c211d0, 0x01bb7670, 0x01c211d0, 0x02e5a990, 0x01c211d0, 0x01c91298, 0x01c211d0, 0x01b72a48, 0x01c211d0, 0x01add448, 
	0x01c7d2f8, 0x01c78ae0, 0x01c7d2f8, 0x02e7b820, 0x01c7d2f8, 0x01b03a10, 0x01c7d2f8, 0x02dc4670, 0x01c7d2f8, 0x00c49c28, 0x01c7d2f8, 0x02dc3ad0, 0x01c7d2f8, 0x00c06480, 0x01c7d2f8, 0x02da42c0, 
	0x01c7d2f8, 0x02d7d660, 0x01c7d2f8, 0x01bb7670, 0x01c7d2f8, 0x02e5a990, 0x01c7d2f8, 0x01c91298, 0x01c7d2f8, 0x01b72a48, 0x01c7d2f8, 0x01add448, 0x01c78ae0, 0x02e7b820, 0x01c78ae0, 0x01b03a10, 
	0x01c78ae0, 0x02dc4670, 0x01c78ae0, 0x00c49c28, 0x01c78ae0, 0x02dc3ad0, 0x01c78ae0, 0x00c06480, 0x01c78ae0, 0x02da42c0, 0x01c78ae0, 0x02d7d660, 0x01c78ae0, 0x01bb7670, 0x01c78ae0, 0x02e5a990, 
	0x01c78ae0, 0x01c91298, 0x01c78ae0, 0x01b72a48, 0x01c78ae0, 0x01add448, 0x02e7b820, 0x01b03a10, 0x02e7b820, 0x02dc4670, 0x02e7b820, 0x00c49c28, 0x02e7b820, 0x02dc3ad0, 0x02e7b820, 0x00c06480, 
	0x02e7b820, 0x02da42c0, 0x02e7b820, 0x02d7d660, 0x02e7b820, 0x01bb7670, 0x02e7b820, 0x02e5a990, 0x02e7b820, 0x01c91298, 0x02e7b820, 0x01b72a48, 0x02e7b820, 0x01add448, 0x01b03a10, 0x02dc4670, 
	0x01b03a10, 0x00c49c28, 0x01b03a10, 0x02dc3ad0, 0x01b03a10, 0x00c06480, 0x01b03a10, 0x02da42c0, 0x01b03a10, 0x02d7d660, 0x01b03a10, 0x01bb7670, 0x01b03a10, 0x02e5a990, 0x01b03a10, 0x01c91298, 
	0x01b03a10, 0x01b72a48, 0x01b03a10, 0x01add448, 0x02dc4670, 0x00c49c28, 0x02dc4670, 0x02dc3ad0, 0x02dc4670, 0x00c06480, 0x02dc4670, 0x02da42c0, 0x02dc4670, 0x02d7d660, 0x02dc4670, 0x01bb7670, 
	0x02dc4670, 0x02e5a990, 0x02dc4670, 0x01c91298, 0x02dc4670, 0x01b72a48, 0x02dc4670, 0x01add448, 0x00c49c28, 0x02dc3ad0, 0x00c49c28, 0x00c06480, 0x00c49c28, 0x02da42c0, 0x00c49c28, 0x02d7d660, 
	0x00c49c28, 0x01bb7670, 0x00c49c28, 0x02e5a990, 0x00c49c28, 0x01c91298, 0x00c49c28, 0x01b72a48, 0x00c49c28, 0x01add448, 0x02dc3ad0, 0x00c06480, 0x02dc3ad0, 0x02da42c0, 0x02dc3ad0, 0x02d7d660, 
	0x02dc3ad0, 0x01bb7670, 0x02dc3ad0, 0x02e5a990, 0x02dc3ad0, 0x01c91298, 0x02dc3ad0, 0x01b72a48, 0x02dc3ad0, 0x01add448, 0x00c06480, 0x02da42c0, 0x00c06480, 0x02d7d660, 0x00c06480, 0x01bb7670, 
	0x00c06480, 0x02e5a990, 0x00c06480, 0x01c91298, 0x00c06480, 0x01b72a48, 0x00c06480, 0x01add448, 0x02da42c0, 0x02d7d660, 0x02da42c0, 0x01bb7670, 0x02da42c0, 0x02e5a990, 0x02da42c0, 0x01c91298, 
	0x02da42c0, 0x01b72a48, 0x02da42c0, 0x01add448, 0x02d7d660, 0x01bb7670, 0x02d7d660, 0x02e5a990, 0x02d7d660, 0x01c91298, 0x02d7d660, 0x01b72a48, 0x02d7d660, 0x01add448, 0x01bb7670, 0x02e5a990, 
	0x01bb7670, 0x01c91298, 0x01bb7670, 0x01b72a48, 0x01bb7670, 0x01add448, 0x02e5a990, 0x01c91298, 0x02e5a990, 0x01b72a48, 0x02e5a990, 0x01add448, 0x01c91298, 0x01b72a48, 0x01c91298, 0x01add448, 
	0x01b72a48, 0x01add448
};

TestRagdoll::TestRagdoll()// : mAggregate(NULL)
{
	for(udword i=0;i<18;i++)
		mJoints[i] = null;
}

TestRagdoll::~TestRagdoll()
{
/*	for(udword i=0;i<18;i++)
		if(mJoints[i])
			mJoints[i]->release();

	if(mAggregate)
		mAggregate->release();*/
}

const Bone* TestRagdoll::FindBoneByName(int name) const
{
	for(udword i=0;i<19;i++)
		if(mBones[i].mID==name)
			return &mBones[i];
	return NULL;
}

static PintObjectHandle createBodyPart(Pint& pint, const Point& extents, const PR& pose, float mass, PintCollisionGroup group)
{
	PINT_BOX_CREATE BoxDesc;
	BoxDesc.mExtents	= extents;
	BoxDesc.mRenderer	= CreateBoxRenderer(extents);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes			= &BoxDesc;
	ObjectDesc.mPosition		= pose.mPos;
	ObjectDesc.mRotation		= pose.mRot;
	ObjectDesc.mMass			= mass;
	ObjectDesc.mCollisionGroup	= group;
	PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
	ASSERT(Handle);
	return Handle;
}

static void ComputeLocalAnchor(Point& local, const Point& global, const PR* pose)
{
	if(pose)
	{
		Matrix4x4 M;
		InvertPRMatrix(M, Matrix4x4(*pose));

		local = global * M;
	}
	else local = global;
}

static void ComputeLocalAxis(Point& local, const Point& global, const PR* pose)
{
	if(pose)
	{
		Matrix4x4 M;
		InvertPRMatrix(M, Matrix4x4(*pose));

		local = global * Matrix3x3(M);
	}
	else local = global;
}

bool TestRagdoll::Init(Pint& pint, udword tag, const Point& offset, PintCollisionGroup group, bool /*useCompound*/)
{
	BinReader Data((const char*)gRagdollData);
	int nbBones = Data.readDword();
	ASSERT(nbBones==19);
	for(int i=0;i<nbBones;i++)
	{
		mBones[i].mID = Data.readDword();
		mBones[i].mPivot.x = Data.readFloat();
		mBones[i].mPivot.y = Data.readFloat();
		mBones[i].mPivot.z = Data.readFloat();
		mBones[i].mPivot *= gRagdollScale;

		const float linearDamping = Data.readFloat();
		const float angularDamping = Data.readFloat();
		const float maxAngularVelocity = Data.readFloat();
		const int solverIterationCount = Data.readDword();

		PR massLocalPose;
		massLocalPose.mPos.x = Data.readFloat();
		massLocalPose.mPos.y = Data.readFloat();
		massLocalPose.mPos.z = Data.readFloat();
		massLocalPose.mPos *= gRagdollScale;
		massLocalPose.mPos += offset;

		Matrix3x3 m;
		for(int x=0;x<3;x++)
		{
			for(int y=0;y<3;y++)
			{
				m.m[y][x] = Data.readFloat();
			}
		}
		massLocalPose.mRot = m;
		massLocalPose.mRot.Normalize();

		int collisionGroup = Data.readDword();
		Point dimensions;
		dimensions.x = Data.readFloat();
		dimensions.y = Data.readFloat();
		dimensions.z = Data.readFloat();
dimensions *= gRagdollScale;

		float density = Data.readFloat();
density *= gRagdollScale;
		mBones[i].mBody = createBodyPart(pint, dimensions, massLocalPose, density, group);
		mBones[i].mPose = massLocalPose;
	}

	for(int i=0;i<18;i++)
	{
		int bodyID0 = Data.readDword();
		int bodyID1 = Data.readDword();
		const Bone* b0 = FindBoneByName(bodyID0);
		const Bone* b1 = FindBoneByName(bodyID1);
		ASSERT(b0);
		ASSERT(b1);

		Point globalAnchor;
		globalAnchor.x = Data.readFloat();
		globalAnchor.y = Data.readFloat();
		globalAnchor.z = Data.readFloat();
globalAnchor *= gRagdollScale;
		globalAnchor += offset;

		Point globalAxis;
		globalAxis.x = Data.readFloat();
		globalAxis.y = Data.readFloat();
		globalAxis.z = Data.readFloat();

/*		PxRevoluteJoint* rv = PxRevoluteJointCreate(physics, b0->mBody, PxTransform::createIdentity(), b1->mBody, PxTransform::createIdentity());
		mJoints[i] = rv;
		rv->setConstraintFlags(PxConstraintFlag::ePROJECTION);
		rv->setProjectionLinearTolerance(0.1f);
		if(1)
		{
			PxJointLimitPair limit(-PxPi/2, PxPi/2, 0.05f);
			limit.restitution	= 0.0f;
			limit.lower			= -0.2f;
			limit.upper			= 0.2f;
			rv->setLimit(limit);

			rv->setRevoluteJointFlags(PxRevoluteJointFlag::eLIMIT_ENABLED);
		}

		PxSetJointGlobalFrame(*rv, &globalAnchor, &globalAxis);*/

		Matrix4x4 Mat0 = b0->mPose;
		Mat0.Invert();
		Matrix4x4 Mat1 = b1->mPose;
		Mat1.Invert();

		PINT_HINGE_JOINT_CREATE Desc;
		Desc.mObject0		= b0->mBody;
		Desc.mObject1		= b1->mBody;
/*		Desc.mLocalPivot0	= globalAnchor * Mat0;
		Desc.mLocalPivot1	= globalAnchor * Mat1;
		Desc.mLocalAxis0	= globalAxis * Matrix3x3(Mat0);
		Desc.mLocalAxis1	= globalAxis * Matrix3x3(Mat1);*/

// ### test
ComputeLocalAnchor(Desc.mLocalPivot0, globalAnchor, &b0->mPose);
ComputeLocalAnchor(Desc.mLocalPivot1, globalAnchor, &b1->mPose);
ComputeLocalAxis(Desc.mLocalAxis0, globalAxis, &b0->mPose);
ComputeLocalAxis(Desc.mLocalAxis1, globalAxis, &b1->mPose);

		Desc.mGlobalAnchor	= globalAnchor;
		Desc.mGlobalAxis	= globalAxis;
		Desc.mMinLimitAngle	= -0.2f;
		Desc.mMaxLimitAngle	= 0.2f;
//		Desc.mMinLimitAngle	= -0.02f;
//		Desc.mMaxLimitAngle	= 0.02f;
//		Desc.mMinLimitAngle	= degToRad(-45.0f);
//		Desc.mMaxLimitAngle	= degToRad(45.0f);
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool GenerateArrayOfRagdolls(Pint& pint, const PintCaps& caps, udword NbX, udword NbY, float Scale, Point* offset=null)
{
	if(!caps.mSupportHingeJoints || !caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
		return false;

	const PintDisabledGroups DG(1, 1);
	pint.SetDisabledGroups(1, &DG);

	for(udword i=0;i<NbX;i++)
	{
		const float CoeffX = NbX>1 ? (float(i)/float(NbX-1))-0.5f : 0.0f;
		for(udword j=0;j<NbY;j++)
		{
			const float CoeffY = NbY>1 ? (float(j)/float(NbY-1))-0.5f : 0.0f;
			TestRagdoll RD;

			Point Offset(CoeffX*Scale, 1.0f, CoeffY*Scale);
			if(offset)
				Offset += *offset;

			RD.Init(pint, 0, Offset, 1, false);
		}
	}
	return true;
}

static bool GenerateColumnOfRagdolls(Pint& pint, const PintCaps& caps, udword nb)
{
	if(!caps.mSupportHingeJoints || !caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
		return false;

	// Disable each ragdoll's inner collisions, but let ragdolls collide with each other.
	PintDisabledGroups* DisabledGroups = (PintDisabledGroups*)StackAlloc(nb*sizeof(PintDisabledGroups));
	for(udword i=0;i<nb;i++)
	{
		const PintCollisionGroup DisabledGroup = PintCollisionGroup(i+1);
		DisabledGroups[i] = PintDisabledGroups(DisabledGroup, DisabledGroup);
	}
	pint.SetDisabledGroups(nb, DisabledGroups);

	const float Inc = 2.0;
	Point Offset(0.0f, Inc, 0.0f);
	for(udword i=0;i<nb;i++)
	{
		TestRagdoll RD;
		RD.Init(pint, 0, Offset, i+1, false);
		Offset.y += Inc;
	}
	return true;
}

static const char* gDesc_Ragdoll = "Simple ragdoll made of 19 bones connected by hinge joints.";

START_TEST(Ragdoll, CATEGORY_PERFORMANCE, gDesc_Ragdoll)

	virtual	void Ragdoll::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.58f, 1.32f, -14.27f), Point(-0.98f, -0.02f, -0.21f));
	}

	virtual bool Ragdoll::Setup(Pint& pint, const PintCaps& caps)
	{
		return GenerateArrayOfRagdolls(pint, caps, 1, 1, 20.0f);
	}

END_TEST(Ragdoll)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Ragdoll100 = "100 ragdolls. Ragdoll-ragdoll collisions are disabled.";

START_TEST(Ragdolls_100, CATEGORY_PERFORMANCE, gDesc_Ragdoll100)

	virtual	void Ragdolls_100::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.68f, 2.85f, -4.21f), Point(-0.73f, -0.30f, -0.62f));
	}

	virtual bool Ragdolls_100::Setup(Pint& pint, const PintCaps& caps)
	{
	//	return GenerateArrayOfRagdolls(pint, caps, 10, 10, 10.0f);
		return GenerateArrayOfRagdolls(pint, caps, 10, 10, 20.0f);
	}

END_TEST(Ragdolls_100)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Ragdoll256 = "256 ragdolls. Ragdoll-ragdoll collisions are disabled.";

START_TEST(Ragdolls_256, CATEGORY_PERFORMANCE, gDesc_Ragdoll256)

	virtual	void Ragdolls_256::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(17.08f, 2.98f, -1.78f), Point(-0.75f, -0.15f, -0.65f));
	}

	virtual bool Ragdolls_256::Setup(Pint& pint, const PintCaps& caps)
	{
		return GenerateArrayOfRagdolls(pint, caps, 16, 16, 20.0f);
	}

END_TEST(Ragdolls_256)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Ragdoll256_OnTerrain = "256 ragdolls on a terrain. Ragdoll-ragdoll collisions are disabled.";

//static udword DoBatchRaycasts(TestBase& test, Pint& pint, bool use_phantoms);

START_TEST(Ragdolls_256_OnTerrain, CATEGORY_PERFORMANCE, gDesc_Ragdoll256_OnTerrain)

	virtual	void Ragdolls_256_OnTerrain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
//		desc.mCamera[0] = CameraPose(Point(17.08f, 2.98f, -1.78f), Point(-0.75f, -0.15f, -0.65f));
		desc.mCamera[0] = CameraPose(Point(2271.26f, 326.41f, 1416.32f), Point(-0.63f, -0.44f, 0.64f));
	}

	virtual bool Ragdolls_256_OnTerrain::CommonSetup()
	{
		TestBase::CommonSetup();
		LoadMeshesFromFile_(*this, "terrain.bin");
		mCreateDefaultEnvironment = false;

		if(0)
		{
			const float Scale = 2000.0f;
			const udword NbX = 16;
			const udword NbY = 16;
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
			Center.y += Extents.y;

			for(udword i=0;i<NbX;i++)
			{
				const float CoeffX = NbX>1 ? (float(i)/float(NbX-1))-0.5f : 0.0f;
				for(udword j=0;j<NbY;j++)
				{
					const float CoeffY = NbY>1 ? (float(j)/float(NbY-1))-0.5f : 0.0f;

					Point Offset(CoeffX*Scale, 1.0f, CoeffY*Scale);
					Offset += Center;

					RegisterRaycast(Offset, Point(0.0f, -1.0f, 0.0f), 5000.0f);
				}
			}
		}
		return true;
	}

	virtual bool Ragdolls_256_OnTerrain::Setup(Pint& pint, const PintCaps& caps)
	{
//		if(!caps.mSupportRaycasts)
//			return false;
		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);
		Center.y += Extents.y*0.2f;

		const float Saved = gRagdollScale;
		gRagdollScale = 1.0f;
//		bool Status = GenerateArrayOfRagdolls(pint, caps, 16, 16, 2000.0f, &Center);
//		static bool GenerateArrayOfRagdolls(Pint& pint, const PintCaps& caps, udword NbX, udword NbY, float Scale, Point* offset=null)
		{
			udword NbX = 16;
			udword NbY = 16;
			float Scale = 2000.0f;
			Point* offset = &Center;
			if(!caps.mSupportHingeJoints || !caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
				return false;

			const PintDisabledGroups DG(1, 1);
			pint.SetDisabledGroups(1, &DG);

			SRand(42);

			for(udword i=0;i<NbX;i++)
			{
				const float CoeffX = NbX>1 ? (float(i)/float(NbX-1))-0.5f : 0.0f;
				for(udword j=0;j<NbY;j++)
				{
					const float CoeffY = NbY>1 ? (float(j)/float(NbY-1))-0.5f : 0.0f;
					TestRagdoll RD;

					Point Offset(CoeffX*Scale, 1.0f, CoeffY*Scale);
					if(offset)
						Offset += *offset;

					Offset.y += UnitRandomFloat()*100.0f;

					RD.Init(pint, 0, Offset, 1, false);
				}
			}
		}
		gRagdollScale = Saved;
		return true;
//		return Status;
	}

/*	virtual udword Ragdolls_256_OnTerrain::Update(Pint& pint)
	{
		return DoBatchRaycasts(*this, pint, false);
	}*/

END_TEST(Ragdolls_256_OnTerrain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfRagdolls16 = "A pile of 16 ragdolls. Ragdoll-ragdoll collisions are enabled.";

START_TEST(PileOfRagdolls_16, CATEGORY_PERFORMANCE, gDesc_PileOfRagdolls16)

	virtual	void PileOfRagdolls_16::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.68f, 2.85f, -4.21f), Point(-0.73f, -0.30f, -0.62f));
	}

	virtual bool PileOfRagdolls_16::Setup(Pint& pint, const PintCaps& caps)
	{
		return GenerateColumnOfRagdolls(pint, caps, 16);
	}

END_TEST(PileOfRagdolls_16)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicSpheresOnArchipelago = "64*64 dynamic spheres falling on the Archipelago mesh level.";

START_TEST(DynamicSpheresOnArchipelago, CATEGORY_PERFORMANCE, gDesc_DynamicSpheresOnArchipelago)

	virtual void DynamicSpheresOnArchipelago::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
	}

	virtual bool DynamicSpheresOnArchipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool DynamicSpheresOnArchipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 64;
		const udword NbY = 64;
		const float Altitude = 10.0f;
		const float Radius = 1.0f;
		return GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Altitude, Extents.x-8.0f, Extents.z-8.0f, 1.0f, 0, &Offset);
	}

END_TEST(DynamicSpheresOnArchipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicCapsulesOnArchipelago = "64*64 dynamic capsules falling on the Archipelago mesh level.";

START_TEST(DynamicCapsulesOnArchipelago, CATEGORY_PERFORMANCE, gDesc_DynamicCapsulesOnArchipelago)

	virtual void DynamicCapsulesOnArchipelago::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
	}

	virtual bool DynamicCapsulesOnArchipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool DynamicCapsulesOnArchipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 64;
		const udword NbY = 64;
		const float Altitude = 10.0f;
		const float Radius = 0.5f;
		const float HalfHeight = 2.0f;
		return GenerateArrayOfCapsules(pint, Radius, HalfHeight, NbX, NbY, Altitude, Extents.x-8.0f, Extents.z-8.0f, 1.0f, 0, &Offset);
	}

END_TEST(DynamicCapsulesOnArchipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicConvexesOnArchipelago = "32*32 dynamic convexes falling on the Archipelago mesh level.";

START_TEST(DynamicConvexesOnArchipelago, CATEGORY_PERFORMANCE, gDesc_DynamicConvexesOnArchipelago)

	virtual void DynamicConvexesOnArchipelago::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
	}

	virtual bool DynamicConvexesOnArchipelago::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool DynamicConvexesOnArchipelago::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
		udword i=2;	// Small convexes
	//	udword i=4;	// 'Big' convexes
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 10.0f;
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Extents.x-8.0f, Extents.z-8.0f, &Offset);
	}

END_TEST(DynamicConvexesOnArchipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicConvexesOnArchipelago2 = "32*32*4 dynamic convexes falling on the Archipelago mesh level.";

START_TEST(DynamicConvexesOnArchipelago2, CATEGORY_PERFORMANCE, gDesc_DynamicConvexesOnArchipelago2)

	virtual void DynamicConvexesOnArchipelago2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
	}

	virtual bool DynamicConvexesOnArchipelago2::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool DynamicConvexesOnArchipelago2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
		udword i=2;	// Small convexes
	//	udword i=4;	// 'Big' convexes
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const udword NbX = 32;
		const udword NbY = 32;
		const float AltitudeC = 10.0f;
		for(udword i=0;i<4;i++)
		{
			Point CurrentOffset = Offset;
			CurrentOffset.x += float(i)*2.0f;
			CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC+float(i)*3.0f, Extents.x-8.0f, Extents.z-8.0f, &CurrentOffset);
		}
		return true;
	}

END_TEST(DynamicConvexesOnArchipelago2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicConvexesOnArchipelagoTess = "64*64 dynamic convexes falling on the (tesselated) Archipelago mesh level.";

START_TEST(DynamicConvexesOnArchipelagoTess, CATEGORY_PERFORMANCE, gDesc_DynamicConvexesOnArchipelagoTess)

	virtual void DynamicConvexesOnArchipelagoTess::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
	}

	virtual bool DynamicConvexesOnArchipelagoTess::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin", null, false, 2, TESS_POLYHEDRAL);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool DynamicConvexesOnArchipelagoTess::Setup(Pint& pint, const PintCaps& caps)
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
	//	udword i=4;	// 'Big' convexes
	//	udword i=7;
		udword i=13;
		C.LoadFile(i);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const udword NbX = 64;
		const udword NbY = 64;
		const float AltitudeC = 10.0f;
		return CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, AltitudeC, Extents.x-8.0f, Extents.z-8.0f, &Offset);
	}

END_TEST(DynamicConvexesOnArchipelagoTess)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_KinematicObjects = "Kinematic objects.";

START_TEST(KinematicObjects, CATEGORY_PERFORMANCE, gDesc_KinematicObjects)

	udword	mNbX;
	udword	mNbY;
	udword	mNbLayers;
	float	mScaleX;
	float	mScaleY;
	float	mYScale;
	Point*	mKinePositions;

	virtual void KinematicObjects::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-110.0f, -10.0f, -110.0f), Point(110.0f, 10.0f, 110.0f));
		desc.mCamera[0] = CameraPose(Point(94.85f, 9.49f, 95.52f), Point(-0.65f, -0.44f, -0.62f));
	}

/*	virtual	bool KinematicObjects::ProfileUpdate()
	{
		return true;
	}*/

	virtual bool KinematicObjects::CommonSetup()
	{
		mNbX = mNbY = 64;
		mNbLayers = 1;
		mScaleX = mScaleY = 3.1f;
		mYScale = 0.4f;

		mKinePositions = ICE_NEW(Point)[mNbLayers*mNbX*mNbY];
		return TestBase::CommonSetup();
	}

	virtual	void KinematicObjects::CommonRelease()
	{
		DELETEARRAY(mKinePositions);
		TestBase::CommonRelease();
	}

	virtual bool KinematicObjects::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportKinematics)
			return false;

		const Point BoxExtents(1.5f, 0.2f, 1.5f);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= BoxExtents;
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxExtents);

		PintObjectHandle* Handles = (PintObjectHandle*)ICE_ALLOC(sizeof(PintObjectHandle)*mNbLayers*mNbX*mNbY);
		pint.mUserData = Handles;
		udword Index = 0;
		for(udword j=0;j<mNbLayers;j++)
		{
			for(udword y=0;y<mNbY;y++)
			{
				for(udword x=0;x<mNbX;x++)
				{
					const float xf = (float(x)-float(mNbX)*0.5f)*mScaleX;
					const float yf = (float(y)-float(mNbY)*0.5f)*mScaleY;

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &BoxDesc;
					ObjectDesc.mPosition	= Point(xf, 0.2f + float(j)*mYScale, yf);
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mKinematic	= true;
					PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
					ASSERT(Handle);
					Handles[Index++] = Handle;
				}
			}
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual	void KinematicObjects::Close(Pint& pint)
	{
		PintObjectHandle* Handles = (PintObjectHandle*)pint.mUserData;
		ICE_FREE(Handles);
		pint.mUserData = null;

		TestBase::Close(pint);
	}

	virtual void KinematicObjects::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		// Compute new kinematic poses - the same for all engines
		{
			udword Index = 0;
			const float Coeff = 0.2f;
			for(udword j=0;j<mNbLayers;j++)
			{
				for(udword y=0;y<mNbY;y++)
				{
					for(udword x=0;x<mNbX;x++)
					{
						const float xf = (float(x)-float(mNbX)*0.5f)*mScaleX;
						const float yf = (float(y)-float(mNbY)*0.5f)*mScaleY;

						const float h = sinf(mCurrentTime*2.0f + float(x)*Coeff + + float(y)*Coeff)*2.0f;
						mKinePositions[Index++] = Point(xf, h + 2.0f + float(j)*mYScale, yf);
					}
				}
			}
		}
	}

	virtual udword KinematicObjects::Update(Pint& pint)
	{
		const PintObjectHandle* __restrict Handles = (const PintObjectHandle*)pint.mUserData;
		if(Handles)
		{
			const Point* __restrict Positions = mKinePositions;
			udword Nb = mNbLayers*mNbX*mNbY;
			while(Nb--)
				pint.SetKinematicPose(*Handles++, *Positions++);
		}
		return 0;
	}

END_TEST(KinematicObjects)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddStaticObjects = "Add static objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddStaticObjects, CATEGORY_PERFORMANCE, gDesc_AddStaticObjects)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;
	PINT_BOX_CREATE*	mBoxCreate;
	bool*				mFlags;
	Container			mCreated;

	virtual	bool AddStaticObjects::ProfileUpdate()
	{
		return true;
	}

	virtual	void AddStaticObjects::CommonRelease()
	{
		ICE_FREE(mFlags);
		DELETEARRAY(mBoxCreate);
		mCreated.Empty();

		TestBase::CommonRelease();
	}

	virtual bool AddStaticObjects::CommonSetup()
	{
		TestBase::CommonSetup();

		mMoving.SetEmpty();

	//	mBoxSize = 4.0f;
		mBoxSize = 8.0f;

		mAmplitude = 40.0f;
	//	mAmplitude = 80.0f;

	//	const udword NbX = 64;
	//	const udword NbY = 64;
		const udword NbX = 128;
		const udword NbY = 128;
		const float Altitude = 0.0f;

		BasicRandom Rnd(42);

		mBoxCreate = ICE_NEW(PINT_BOX_CREATE)[NbX*NbY];
		mFlags = (bool*)ICE_ALLOC(sizeof(bool)*NbX*NbY);

		udword Index = 0;
		for(udword y=0;y<NbY;y++)
		{
			const float CoeffY = 2.0f * ((float(y)/float(NbY-1)) - 0.5f);
			for(udword x=0;x<NbX;x++)
			{
				const float CoeffX = 2.0f * ((float(x)/float(NbX-1)) - 0.5f);

				Point Random;
				UnitRandomPt(Random, Rnd);
				const Point Extents = Random + Point(1.0f, 1.0f, 1.0f);

				mFlags[Index] = false;
				mBoxCreate[Index].mExtents	= Extents;
				mBoxCreate[Index].mRenderer	= CreateBoxRenderer(Extents);
				Index++;

				UnitRandomPt(Random, Rnd);
				const Point Center = Random + Point(CoeffX * mAmplitude, Altitude, CoeffY * mAmplitude);

				AABB Box;
				Box.SetCenterExtents(Center, Extents);
				RegisterAABB(Box);
			}
		}
		return true;
	}

	virtual bool AddStaticObjects::Setup(Pint& pint, const PintCaps& caps)
	{
	/*
		if(!caps.mSupportRaycasts)
			return false;*/

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void AddStaticObjects::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		const float t = mCurrentTime;
	//	const float t = mCurrentTime * 0.1f;

		const float PosScale = mAmplitude - mBoxSize;
		const float x = sinf(t*0.07f) * cosf(t*1.13f) * PosScale;
		const float y = sinf(t*2.07f) * cosf(t*0.13f) * PosScale;

		mMoving.SetCenterExtents(Point(x, 0.0f, y), Point(mBoxSize, 10.0f, mBoxSize));

		mCreated.Reset();
		udword NbBoxes = GetNbAABBs();
		const AABB* Boxes = (const AABB*)GetAABBs();
		for(udword i=0;i<NbBoxes;i++)
		{
			if(!mFlags[i] && mMoving.Intersect(Boxes[i]))
			{
				mFlags[i] = true;
				mCreated.Add(i);
			}
		}
	}

	virtual void AddStaticObjects::CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
		RenderAllAABBs(renderer);
	}

	virtual udword AddStaticObjects::Update(Pint& pint)
	{
		const AABB* Boxes = (const AABB*)GetAABBs();
		udword NbCreated = mCreated.GetNbEntries();
		const udword* Indices = mCreated.GetEntries();
		while(NbCreated--)
		{
			const udword i = *Indices++;

			Point Center;
			Boxes[i].GetCenter(Center);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= mBoxCreate + i;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Center;
			CreatePintObject(pint, ObjectDesc);
		}
		return mCreated.GetNbEntries();
	}

END_TEST(AddStaticObjects)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddStaticObjects2 = "64*64 static boxes... added at runtime to 128*128 other static boxes.... This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddStaticObjects2, CATEGORY_PERFORMANCE, gDesc_AddStaticObjects2)

	bool	mStopTest;
	bool	mAddObjects;

	virtual	bool AddStaticObjects2::ProfileUpdate()
	{
		return true;
	}

	virtual bool AddStaticObjects2::CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		mAddObjects = false;
		mStopTest = false;
		return true;
	}

	virtual bool AddStaticObjects2::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticBoxes(pint, 100.0f, 128, 128, 0.0f);
	}

	virtual void AddStaticObjects2::CommonUpdate()
	{
		if(mStopTest)
			return;

		if(mAddObjects)
		{
			mStopTest = true;
			mAddObjects = false;
			return;
		}

		mCurrentTime += 1.0f/60.0f;
		if(mCurrentTime>=1.0f)
			mAddObjects = true;
	}

	virtual udword AddStaticObjects2::Update(Pint& pint)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		if(mAddObjects)
			return CreateSeaOfStaticBoxes(pint, 50.0f, 64, 64, 1.0f);

		return 0;
	}

END_TEST(AddStaticObjects2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddDynamicObjects = "Add dynamic objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddDynamicObjects, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjects)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;

	virtual	bool AddDynamicObjects::ProfileUpdate()
	{
		return true;
	}

	virtual	void AddDynamicObjects::CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool AddDynamicObjects::CommonSetup()
	{
		TestBase::CommonSetup();

		mMoving.SetEmpty();

		mBoxSize = 1.0f;
		mAmplitude = 80.0f;

		return true;
	}

	virtual bool AddDynamicObjects::Setup(Pint& pint, const PintCaps& caps)
	{
		return true;
	}

	virtual void AddDynamicObjects::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		const float t = mCurrentTime * 2.0f;
	//	const float t = mCurrentTime * 0.1f;

		const float PosScale = mAmplitude - mBoxSize;
		const float x = sinf(t*0.07f) * cosf(t*1.13f) * PosScale;
		const float y = sinf(t*2.07f) * cosf(t*0.13f) * PosScale;

		mMoving.SetCenterExtents(Point(x, 0.0f, y), Point(mBoxSize, 10.0f, mBoxSize));
	}

	virtual void AddDynamicObjects::CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
	}

	virtual udword AddDynamicObjects::Update(Pint& pint)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		PINT_BOX_CREATE BoxCreate;
		BoxCreate.mExtents	= Point(1.0f, 1.0f, 1.0f);
		BoxCreate.mRenderer	= CreateBoxRenderer(BoxCreate.mExtents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxCreate;
		ObjectDesc.mMass		= 1.0f;
		mMoving.GetCenter(ObjectDesc.mPosition);
		ObjectDesc.mPosition.y += 50.0f;
		CreatePintObject(pint, ObjectDesc);
		return 0;
	}

END_TEST(AddDynamicObjects)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddDynamicObjects2 = "Add dynamic objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddDynamicObjects2, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjects2)

	virtual	bool AddDynamicObjects2::ProfileUpdate()
	{
	//	return false;
		return true;
	}

	virtual	void AddDynamicObjects2::CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool AddDynamicObjects2::CommonSetup()
	{
		TestBase::CommonSetup();
		return true;
	}

	virtual bool AddDynamicObjects2::Setup(Pint& pint, const PintCaps& caps)
	{
		const float BoxHeight = 8.0f;
		const float BoxSide = 0.01f;
		const float BoxDepth = 20.0f;
		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);
		return true;
	}

	virtual void AddDynamicObjects2::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;
		if(mCurrentTime>1.0f)
		{
			mCurrentTime = 0.0f;
		}
	}

	virtual void AddDynamicObjects2::CommonRender(PintRender& renderer)
	{
	}

	virtual udword AddDynamicObjects2::Update(Pint& pint)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		if(mCurrentTime==0.0f)
		{
			GenerateArrayOfBoxes(pint, Point(0.5f, 0.5f, 0.5f), 4, 4, 50.0f, 2.0f, 1.0f);
			GenerateArrayOfBoxes(pint, Point(0.5f, 0.5f, 0.5f), 4, 4, 48.0f, 2.0f, 1.0f);
	//		GenerateArrayOfSpheres(pint, 0.5f, 4, 4, 48.0f, 2.0f, 1.0f);
		}
		return 0;
	}

END_TEST(AddDynamicObjects2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddDynamicObjectsAndDoRaycasts = "Add dynamic objects at runtime, and do raycasts. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_SQ_TEST(AddDynamicObjectsAndDoRaycasts, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjectsAndDoRaycasts)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;

	virtual	void AddDynamicObjectsAndDoRaycasts::CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool AddDynamicObjectsAndDoRaycasts::CommonSetup()
	{
		TestBase::CommonSetup();

		mMoving.SetEmpty();

		mBoxSize = 1.0f;
		mAmplitude = 80.0f;

		bool Status = Setup_PotPourri_Raycasts(*this, 4096, 100.0f);
		mCreateDefaultEnvironment = true;
		return Status;
	}

	virtual bool AddDynamicObjectsAndDoRaycasts::Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 0.0f, 16, 16, 16);
	}

	virtual void AddDynamicObjectsAndDoRaycasts::CommonUpdate()
	{
		mCurrentTime += 1.0f/60.0f;

		const float t = mCurrentTime * 2.0f;
	//	const float t = mCurrentTime * 0.1f;

		const float PosScale = mAmplitude - mBoxSize;
		const float x = sinf(t*0.07f) * cosf(t*1.13f) * PosScale;
		const float y = sinf(t*2.07f) * cosf(t*0.13f) * PosScale;

		mMoving.SetCenterExtents(Point(x, 0.0f, y), Point(mBoxSize, 10.0f, mBoxSize));
	}

	virtual void AddDynamicObjectsAndDoRaycasts::CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
	}

	virtual udword AddDynamicObjectsAndDoRaycasts::Update(Pint& pint)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		PINT_BOX_CREATE BoxCreate;
		BoxCreate.mExtents	= Point(1.0f, 1.0f, 1.0f);
		BoxCreate.mRenderer	= CreateBoxRenderer(BoxCreate.mExtents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxCreate;
		ObjectDesc.mMass		= 1.0f;
		mMoving.GetCenter(ObjectDesc.mPosition);
		ObjectDesc.mPosition.y += 50.0f;
		CreatePintObject(pint, ObjectDesc);
		for(udword i=0;i<10;i++)
		{
			ObjectDesc.mPosition.y += 1.0f;
			CreatePintObject(pint, ObjectDesc);
		}

		return DoBatchRaycasts(*this, pint);
	}

END_TEST(AddDynamicObjectsAndDoRaycasts)

///////////////////////////////////////////////////////////////////////////////

#include "hacdCircularList.h"
#include "hacdVector.h"
#include "hacdICHull.h"
#include "hacdGraph.h"
#include "hacdHACD.h"

static void DecomposeMeshWithHACD(Pint& pint, udword nb, const Point* pos, udword nb_verts, const Point* verts, udword nb_tris, const uword* indices)
{
	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;

	for(udword i=0;i<nb_verts;i++)
	{
		HACD::Vec3<HACD::Real> vertex(verts[i].x, verts[i].y, verts[i].z);
		points.push_back(vertex);
	}

	for(udword i=0;i<nb_tris;i++)
	{
		const int index = i*3;
		HACD::Vec3<long> triangle(indices[index], indices[index+1], indices[index+2]);
		triangles.push_back(triangle);
	}

	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());
	myHACD.SetCompacityWeight(0.1);
	myHACD.SetVolumeWeight(0.0);

	// HACD parameters
	// Recommended parameters: 2 100 0 0 0 0
	size_t nClusters = 2;
	double concavity = 100;
	bool invert = false;
	bool addExtraDistPoints = false;
	bool addNeighboursDistPoints = false;
	bool addFacesPoints = false;       

	myHACD.SetNClusters(nClusters);                     // minimum number of clusters
	myHACD.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
	myHACD.SetConcavity(concavity);                     // maximum concavity
	myHACD.SetAddExtraDistPoints(addExtraDistPoints);   
	myHACD.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
	myHACD.SetAddFacesPoints(addFacesPoints); 

	myHACD.Compute();
	nClusters = myHACD.GetNClusters();	

//	myHACD.Save("output.wrl", false);

	{
		PINT_CONVEX_CREATE* ConvexCreate = ICE_NEW(PINT_CONVEX_CREATE)[nClusters];
		for(size_t c=0;c<nClusters;c++)
		{
			//generate convex result
			size_t nPoints = myHACD.GetNPointsCH(c);
			size_t nTriangles = myHACD.GetNTrianglesCH(c);

			float* vertices = new float[nPoints*3];
//			unsigned int* triangles = new unsigned int[nTriangles*3];
			
			HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
			HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
			myHACD.GetCH(c, pointsCH, trianglesCH);

			// points
			for(size_t v = 0; v < nPoints; v++)
			{
				vertices[3*v] = pointsCH[v].X();
				vertices[3*v+1] = pointsCH[v].Y();
				vertices[3*v+2] = pointsCH[v].Z();
			}
/*			// triangles
			for(size_t f = 0; f < nTriangles; f++)
			{
				triangles[3*f] = trianglesCH[f].X();
				triangles[3*f+1] = trianglesCH[f].Y();
				triangles[3*f+2] = trianglesCH[f].Z();
			}*/

			delete [] pointsCH;
			delete [] trianglesCH;

			Point* Verts = (Point*)vertices;

			ConvexCreate[c].mNbVerts	= nPoints;
			ConvexCreate[c].mVerts		= Verts;

			if(1)
			{
				Point Center(0.0f, 0.0f, 0.0f);
				const float Coeff = 1.0f / float(nPoints);
				for(udword i=0;i<nPoints;i++)
					Center += Verts[i] * Coeff;

				ConvexCreate[c].mLocalPos = Center;

				for(udword i=0;i<nPoints;i++)
					Verts[i] -= Center;
			}

			ConvexCreate[c].mRenderer	= CreateConvexRenderer(nPoints, Verts);
		}

		if(1)
		{
			for(udword i=0;i<nClusters-1;i++)
				ConvexCreate[i].mNext = &ConvexCreate[i+1];

			for(udword i=0;i<nb;i++)
			{
				PintObjectHandle Handle = CreateDynamicObject(pint, ConvexCreate, pos[i]);
				ASSERT(Handle);
			}

			for(udword i=0;i<nClusters;i++)
			{
				float* v = (float*)ConvexCreate[i].mVerts;
				DELETEARRAY(v);
			}

			DELETEARRAY(ConvexCreate);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

#include "IceBunny.h"

static const char* gDesc_HACD_Test_01 = "HACD test. This uses convex decomposition to simulate mesh-mesh interactions using compounds of convex meshes.";

START_TEST(HACD_Test_01, CATEGORY_PERFORMANCE, gDesc_HACD_Test_01)

	virtual void HACD_Test_01::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.88f, 9.75f, 9.58f), Point(-0.68f, -0.33f, -0.65f));
	}

	virtual bool HACD_Test_01::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes)
			return false;

		const udword NbMeshes = 100;

		Point Pos[NbMeshes];
		for(udword i=0;i<NbMeshes;i++)
			Pos[i].Set(0.0f, 2.0f+float(i)*2.0f, 0.0f);

		Bunny Rabbit;
		DecomposeMeshWithHACD(pint, NbMeshes, Pos, Rabbit.GetNbVerts(), Rabbit.GetVerts(), Rabbit.GetNbFaces(), Rabbit.GetFaces());

/*		if(0)
		{
			LoadMeshesFromFile(pint, *this, "Prison.bin");

			mCreateDefaultEnvironment = false;
		}*/
		return true;
	}

END_TEST(HACD_Test_01)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HACD_Test_02 = "64 convex-compounds falling on the Archipelago mesh level.";

START_TEST(HACD_Test_02, CATEGORY_PERFORMANCE, gDesc_HACD_Test_02)

	virtual void HACD_Test_02::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(197.08f, 4.05f, 139.89f), Point(0.60f, -0.28f, 0.75f));
	}

	virtual bool HACD_Test_02::CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool HACD_Test_02::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);

/*		const udword NbX = 64;
		const udword NbY = 64;
		const float Altitude = 10.0f;
		const float Radius = 1.0f;
		GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Altitude, Extents.x-8.0f, Extents.z-8.0f, 1.0f, 0, &Offset);*/

		const udword NbMeshes = 64;

		Point Pos[NbMeshes];
		for(udword i=0;i<NbMeshes;i++)
			Pos[i].Set(Center.x, Center.y + float(i)*2.0f, Center.z);

		Bunny Rabbit;
		DecomposeMeshWithHACD(pint, NbMeshes, Pos, Rabbit.GetNbVerts(), Rabbit.GetVerts(), Rabbit.GetNbFaces(), Rabbit.GetFaces());

		return true;
	}

END_TEST(HACD_Test_02)

///////////////////////////////////////////////////////////////////////////////

#ifndef PEEL_PUBLIC_BUILD
	#include ".\Fracture\Convex.h"
	#include ".\Fracture\MultiConvex.h"
	#include ".\Fracture\FracturePattern.h"
#endif

static const char* gDesc_VoronoiFracture = "Stress test for Voronoi-based fractures.";

START_TEST(VoronoiFracture, CATEGORY_PERFORMANCE, gDesc_VoronoiFracture)

	virtual bool VoronoiFracture::IsPrivate()	const
	{
		return true;
	}

	virtual void VoronoiFracture::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(19.97f, 5.78f, 19.70f), Point(-0.65f, -0.30f, -0.69f));
	}

	virtual bool VoronoiFracture::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

#ifndef PEEL_PUBLIC_BUILD
		for(udword j=0;j<4;j++)
		{
			for(udword i=0;i<4;i++)
			{
				const Point Offset(float(i)*4.0f, 0.0f, float(j)*4.0f);

				MultiConvex* mMultiConvex = new MultiConvex(/*pint*/);

				Convex c;
				std::vector<Point> polyPoints;

			//	float r = 0.5f;
			//	float r = 1.0f;
				float r = 1.0f;
			//	float r = 10.0f;
				float h = 2.0f;
			//	float h = 20.0f;
				int numSegs = 30;
				float dphi = TWOPI / numSegs;
				for (int i = 0; i < numSegs; i++)
				{
					polyPoints.push_back(Point(-r * cosf(i*dphi), 0.0f, -r * sinf(i*dphi)));
				}
				c.createExtrudedPoly(polyPoints, h);

//				mMultiConvex->createFromConvex(&c, Point(0.0f, h, 0.0f)+Offset);
				mMultiConvex->createFromConvex(pint, &c, Point(0.0f, h, 0.0f)+Offset);

				static FracturePattern mFracPattern;
				static bool initdone=false;
				if(!initdone)
				{
					initdone = true;
					const int NumCells = 100;
			//		mFracPattern.create3dVoronoi(Point(30.0f, 30.0f, 30.0f), NumCells, 10.0f);
			//		mFracPattern.create3dVoronoi(Point(10.0f, 10.0f, 10.0f), NumCells, 10.0f);
//					mFracPattern.create3dVoronoi(Point(3.0f, 3.0f, 3.0f), NumCells, 10.0f);
//					mFracPattern.create3dVoronoi(Point(3.0f, 3.0f, 3.0f), NumCells, 5.0f);
					mFracPattern.create3dVoronoi(Point(3.0f, 3.0f, 3.0f), NumCells, 8.0f);
			//		mFracPattern.create3dVoronoi(2.0f*Point(3.0f, 3.0f, 3.0f), NumCells, 10.0f);
				}
				/*
						float vel = 1.0f;
						float dist;
						int convexNr;
						if (mMultiConvex->rayCast(ray, dist, convexNr))
						{
			//				mMultiConvex->randomFracture(convexNr, ray, 2, 5.0f);
							mMultiConvex->patternFracture(convexNr, ray, mFracPattern, 10.0f, vel);
						}*/
				if(1)
				{
			//		float vel = 1.0f;
					float vel = 0.0f;
					Ray R(Point(0.0f, 10.0f, 0.0f)+Offset, Point(0.0f, -1.0f, 0.0f));
			//		Ray R(2.0f*(Point(0.0f, 10.0f, 0.0f)+Offset), Point(0.0f, -1.0f, 0.0f));
//					mMultiConvex->patternFracture(0, R, mFracPattern, 10.0f, vel);
					mMultiConvex->patternFracture(pint, 0, R, mFracPattern, 10.0f, vel);
//					mMultiConvex->randomFracture(pint, 0, R, 2, vel);
				}
			//	DELETESINGLE(mMultiConvex);	// ### leak
			}
		}

		if(0)
		{
			const float Altitude = -1.0f;
			CreatePlanarMesh(*this, pint, Altitude);
			mCreateDefaultEnvironment = false;
		}
#endif
		return true;
	}

END_TEST(VoronoiFracture)

///////////////////////////////////////////////////////////////////////////////
