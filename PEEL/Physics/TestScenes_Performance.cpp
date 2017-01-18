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
#include "GUI_Helpers.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LargeBoxStack50 = "Giga box stack (50-boxes-wide base). This one is just to show off with ICE Physics :)";

START_TEST(LargeBoxStack50, CATEGORY_PERFORMANCE, gDesc_LargeBoxStack50)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(69.17f, 51.73f, 80.45f), Point(-0.57f, -0.08f, -0.82f));
		desc.mCamera[1] = CameraPose(Point(1.25f, 99.61f, 9.47f), Point(-0.28f, -0.16f, -0.95f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 50);
	}

END_TEST(LargeBoxStack50)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LargeBoxStack30 = "Classical large box stack scene (30-boxes-wide base). Things to look for are the time it takes to simulate it, but also whether the stack eventually collapses or not.";

START_TEST(LargeBoxStack30, CATEGORY_PERFORMANCE, gDesc_LargeBoxStack30)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(47.56f, 48.75f, 47.08f), Point(-0.61f, -0.31f, -0.73f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 30);
	}

END_TEST(LargeBoxStack30)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_MediumBoxStack20 = "Similar to the previous test, medium-sized stack (20-boxes-wide base).";

START_TEST(MediumBoxStack20, CATEGORY_PERFORMANCE, gDesc_MediumBoxStack20)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(26.29f, 36.32f, 35.39f), Point(-0.51f, -0.35f, -0.79f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 1, 20);
	//	return CreateBoxStack(pint, caps, 1, 14);
	}

END_TEST(MediumBoxStack20)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_MediumBoxStacks20 = "Multiple medium-sized box stacks (20-boxes-wide base). Similar to the medium box stack scene but with multiple simulation islands.";

START_TEST(MediumBoxStacks20, CATEGORY_PERFORMANCE, gDesc_MediumBoxStacks20)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(35.88f, 38.56f, 60.53f), Point(-0.58f, -0.41f, -0.70f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 10, 20);
	}

END_TEST(MediumBoxStacks20)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ManySmallBoxStacks10 = "Multiple small-sized box stacks (10-boxes-wide base).";

START_TEST(ManySmallBoxStacks10, CATEGORY_PERFORMANCE, gDesc_ManySmallBoxStacks10)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(18.79f, 20.00f, 130.56f), Point(-0.51f, -0.35f, -0.79f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateBoxStack(pint, caps, 30, 10);
	}

END_TEST(ManySmallBoxStacks10)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxContainerAndSpheres = "A static box container filled with 256 dynamic spheres.";

START_TEST(BoxContainerAndSpheres, CATEGORY_PERFORMANCE, gDesc_BoxContainerAndSpheres)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_SPHERE_CREATE SphereDesc(Radius);
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_BOX_CREATE BoxDesc(Radius, Radius, Radius);
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_CAPSULE_CREATE CapsuleDesc(Radius, HalfHeight);
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_SPHERE_CREATE SphereDesc(SphereRadius);
		SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

		PINT_BOX_CREATE BoxDesc(CapsuleRadius, CapsuleRadius, CapsuleRadius);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_CAPSULE_CREATE CapsuleDesc(CapsuleRadius, HalfHeight);
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.85f, 14.54f, 10.42f), Point(-0.52f, -0.69f, -0.50f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_CONVEX_CREATE ConvexCreate(TotalNbVerts, Pts);
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

		PINT_SPHERE_CREATE SphereDesc(SphereRadius);
		SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

		PINT_BOX_CREATE BoxDesc(CapsuleRadius, CapsuleRadius, CapsuleRadius);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_CAPSULE_CREATE CapsuleDesc(CapsuleRadius, HalfHeight);
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

static const char* gDesc_Compounds320NotTouching = "The notorious compounds from the PhysX samples. Each compound is made of 320 boxes. In this test they do not touch, so \
we should not measure the impact of compound-vs-compound interaction. Engines using a single broadphase entry for each compound should perform a lot better than \
engines using a broadphase entry for each sub-shape.";

START_TEST(Compounds320NotTouching, CATEGORY_PERFORMANCE, gDesc_Compounds320NotTouching)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(104.05f, 43.87f, 100.62f), Point(-0.63f, -0.48f, -0.61f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(16.62f, 195.63f, 11.85f), Point(-0.33f, -0.93f, -0.17f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(111.02f, 69.80f, 121.30f), Point(-0.63f, -0.30f, -0.72f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(112.68f, 74.16f, 113.66f), Point(-0.61f, -0.51f, -0.61f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(81.67f, 27.75f, 68.78f), Point(-0.75f, -0.38f, -0.54f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(43.75f, 14.01f, 42.92f), Point(-0.70f, -0.24f, -0.67f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const udword NbX = 12;
		const udword NbY = 12;

		Point Pts[16];
		udword NbPts = GenerateConvex(Pts, 8, 8, 2.0f, 3.0f, 2.0f);
		ASSERT(NbPts==16);

		PINT_CONVEX_CREATE ConvexCreate(16, Pts);
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(75.47f, 20.40f, 73.86f), Point(-0.69f, -0.25f, -0.68f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(88.12f, 62.03f, 92.27f), Point(-0.66f, -0.26f, -0.71f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(164.90f, 67.88f, 164.53f), Point(-0.70f, 0.07f, -0.71f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.45f, -0.68f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(49.77f, 26.35f, 48.67f), Point(-0.68f, -0.35f, -0.64f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

static const char* gDesc_ConvexGalore = "12 layers of 12*12 convexes. Each convex is randomly chosen in a set of 14 predefined convex objects, of various complexities.";

START_TEST(ConvexGalore, CATEGORY_PERFORMANCE, gDesc_ConvexGalore)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(45.18f, 21.90f, 43.90f), Point(-0.69f, -0.21f, -0.69f));
		desc.mCamera[1] = CameraPose(Point(4.23f, 18.80f, 21.53f), Point(-0.18f, -0.70f, -0.69f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(37.22f, 13.73f, 33.16f), Point(-0.76f, -0.15f, -0.63f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

static const char* gDesc_ConvexGalore3 = "40 layers of 16*16 random objects. This is a test for 10000+ dynamic objects.";

START_TEST(ConvexGalore3, CATEGORY_PERFORMANCE, gDesc_ConvexGalore3)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(37.22f, 13.73f, 33.16f), Point(-0.76f, -0.15f, -0.63f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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
		const udword NbLayers = 40;
		const udword NbX = 16;
		const udword NbY = 16;
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

					const Point pos = Point(xf, Rnd.RandomFloat()*2.0f + Amplitude + (Amplitude * 2.0f * float(j)), yf);

//					const udword Index = Rnd.Randomize() % 16;
					const udword Index = Rnd.Randomize() % 14;

					if(Index<14)
					{
						PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate[Index], pos);
						ASSERT(Handle);
					}
					else if(Index==14)
					{
						const float SphereRadius = 1.0f + Rnd.RandomFloat();
						PINT_SPHERE_CREATE SphereDesc(SphereRadius);
						SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

						PintObjectHandle Handle = CreateDynamicObject(pint, &SphereDesc, pos);
						ASSERT(Handle);
					}
					else if(Index==15)
					{
						PINT_BOX_CREATE BoxDesc;
						UnitRandomPt(BoxDesc.mExtents, Rnd);
						BoxDesc.mExtents += Point(0.2f, 0.2f, 0.2f);
						BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

						PintObjectHandle Handle = CreateDynamicObject(pint, &BoxDesc, pos);
						ASSERT(Handle);
					}
				}
			}
		}
		return true;
	}

END_TEST(ConvexGalore3)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_10000_Spheres = "10000+ dynamic spheres.";

START_TEST(TenThousandsSpheres, CATEGORY_PERFORMANCE, gDesc_10000_Spheres)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(37.22f, 13.73f, 33.16f), Point(-0.76f, -0.15f, -0.63f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Amplitude = 1.5f;
		const udword NbLayers = 40;
		const udword NbX = 16;
		const udword NbY = 16;
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

					const Point pos = Point(xf, Rnd.RandomFloat()*2.0f + Amplitude + (Amplitude * 2.0f * float(j)), yf);

					{
						const float SphereRadius = 1.0f + Rnd.RandomFloat();
						PINT_SPHERE_CREATE SphereDesc(SphereRadius);
						SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

						PintObjectHandle Handle = CreateDynamicObject(pint, &SphereDesc, pos);
						ASSERT(Handle);
					}
				}
			}
		}
		return true;
	}

END_TEST(TenThousandsSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_10000_Boxes = "10000+ dynamic boxes.";

START_TEST(TenThousandsBoxes, CATEGORY_PERFORMANCE, gDesc_10000_Boxes)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(37.22f, 13.73f, 33.16f), Point(-0.76f, -0.15f, -0.63f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Amplitude = 1.5f;
		const udword NbLayers = 40;
		const udword NbX = 16;
		const udword NbY = 16;
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

					const Point pos = Point(xf, Rnd.RandomFloat()*2.0f + Amplitude + (Amplitude * 2.0f * float(j)), yf);

					{
						PINT_BOX_CREATE BoxDesc;
						UnitRandomPt(BoxDesc.mExtents, Rnd);
						BoxDesc.mExtents.x = fabsf(BoxDesc.mExtents.x);
						BoxDesc.mExtents.y = fabsf(BoxDesc.mExtents.y);
						BoxDesc.mExtents.z = fabsf(BoxDesc.mExtents.z);
						BoxDesc.mExtents += Point(0.2f, 0.2f, 0.2f);
						BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

						PintObjectHandle Handle = CreateDynamicObject(pint, &BoxDesc, pos);
						ASSERT(Handle);
					}
				}
			}
		}
		return true;
	}

END_TEST(TenThousandsBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexClash = "(Configurable test) - Dynamic convexes are falling towards static convexes. \
This is a stress test for both convex contact generation and the broad phase.";

class ConvexClash : public TestBase
{
			IceComboBox*	mComboBox_ConvexIndex;
			IceEditBox*		mEditBox_Size;
	public:
							ConvexClash() : mComboBox_ConvexIndex(null), mEditBox_Size(null)	{}
	virtual					~ConvexClash()				{								}
	virtual	const char*		GetName()			const	{ return "ConvexClash";			}
	virtual	const char*		GetDescription()	const	{ return gDesc_ConvexClash;		}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_PERFORMANCE;	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 256;
		WD.mHeight	= 160;
		WD.mLabel	= "ConvexClash";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword OffsetX = 70;
		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 60;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			y += YStep;
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Convex:", UIElems);
			mComboBox_ConvexIndex = CreateConvexObjectComboBox(UI, 4+OffsetX, y, true);
			RegisterUIElement(mComboBox_ConvexIndex);
			y += YStep;
		}

		{
			y += YStep;
			{
				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Grid size:", UIElems);
				mEditBox_Size = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "32", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
				y += YStep;
			}
		}

		y += YStep;
		AddResetButton(UI, 4, y, 256-16);
	}

	virtual	bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		ConvexIndex Index = CONVEX_INDEX_0;
		if(mComboBox_ConvexIndex)
			Index = ConvexIndex(mComboBox_ConvexIndex->GetSelectedIndex());

		const udword Size = GetFromEditBox(32, mEditBox_Size);

		const udword i = Index;
		const udword nb_x = Size;
		const udword nb_y = Size;
		{
			const float Altitude = 10.0f;

			MyConvex C;
			C.LoadFile(i);

			PINT_CONVEX_CREATE ConvexCreate(C.mNbVerts, C.mVerts);
			ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

			const float Amplitude = 1.5f;
			const udword NbX = nb_x;
			const udword NbY = nb_y;

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
		return true;
	}

}ConvexClash;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SpheresOnLargeBox = "32*32 dynamic spheres on a large static box.";

START_TEST(SpheresOnLargeBox, CATEGORY_PERFORMANCE, gDesc_SpheresOnLargeBox)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-32.59f, 5.48f, 31.90f), Point(0.63f, -0.42f, -0.65f));
		desc.mCamera[1] = CameraPose(Point(52.18f, 26.82f, 52.73f), Point(-0.65f, -0.40f, -0.65f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;
		const float Radius = 0.5f;
		const udword NbX = 32;
		const udword NbY = 32;
		return GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Altitude + Radius*2.0f, 30.0f, 30.0f);
	}

END_TEST(SpheresOnLargeBox)

///////////////////////////////////////////////////////////////////////////////

// TODO:
// - expose other meshes
// - use tessellation in plane scene
// - name/describe convexes?
// - random shape?
static const char* gDesc_DynamicsOnMeshLevel = "(Configurable test) - dynamic objects falling on a mesh level.";

enum MeshLevel
{
	MESH_LEVEL_ARCHIPELAGO	= 0,
	MESH_LEVEL_TESTZONE		= 1,
	MESH_LEVEL_FLAT			= 2,
};

#define NB_PRESETS	9
static const udword			gPreset_GridSize[NB_PRESETS] = { 32, 32, 32, 32, 64, 64, 32, 32, 64 };
static const float			gPreset_GridScale[NB_PRESETS] = { 30.0f, 30.0f, 35.0f, 35.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const udword			gPreset_Layers[NB_PRESETS] = { 1, 1, 1, 1, 1, 1, 1, 4, 1 };
static const float			gPreset_Altitude[NB_PRESETS] = { 2.0f, 2.0f, 3.0f, 3.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f };
static const float			gPreset_ShapeSize[NB_PRESETS] = { 0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
static const PintShape		gPreset_ShapeType[NB_PRESETS] = { PINT_SHAPE_SPHERE, PINT_SHAPE_BOX, PINT_SHAPE_CONVEX, PINT_SHAPE_CONVEX, PINT_SHAPE_SPHERE, PINT_SHAPE_CAPSULE, PINT_SHAPE_CONVEX, PINT_SHAPE_CONVEX, PINT_SHAPE_CONVEX };
static const ConvexIndex	gPreset_ConvexIndex[NB_PRESETS] = { CONVEX_INDEX_0, CONVEX_INDEX_0, CONVEX_INDEX_2, CONVEX_INDEX_4, CONVEX_INDEX_0, CONVEX_INDEX_0, CONVEX_INDEX_2, CONVEX_INDEX_2, CONVEX_INDEX_13 };
static const MeshLevel		gPreset_MeshLevel[NB_PRESETS] = { MESH_LEVEL_FLAT, MESH_LEVEL_FLAT, MESH_LEVEL_FLAT, MESH_LEVEL_FLAT, MESH_LEVEL_ARCHIPELAGO, MESH_LEVEL_ARCHIPELAGO, MESH_LEVEL_ARCHIPELAGO, MESH_LEVEL_ARCHIPELAGO, MESH_LEVEL_ARCHIPELAGO };
static const bool			gPreset_Tessellation[NB_PRESETS] = { false, false, false, false, false, false, false, false, true };

class DynamicsOnMeshLevel : public TestBase
{
			IceComboBox*	mComboBox_Preset;
			IceComboBox*	mComboBox_ConvexIndex;
			IceComboBox*	mComboBox_ShapeType;
			IceComboBox*	mComboBox_MeshLevel;
			IceEditBox*		mEditBox_GridSize;
			IceEditBox*		mEditBox_GridScale;
			IceEditBox*		mEditBox_Layers;
			IceEditBox*		mEditBox_Altitude;
			IceEditBox*		mEditBox_ShapeSize;
			IceCheckBox*	mCheckBox_Tessellation;
	public:
							DynamicsOnMeshLevel() :
								mComboBox_Preset		(null),
								mComboBox_ConvexIndex	(null),
								mComboBox_ShapeType		(null),
								mComboBox_MeshLevel		(null),
								mEditBox_GridSize		(null),
								mEditBox_GridScale		(null),
								mEditBox_Layers			(null),
								mEditBox_Altitude		(null),
								mEditBox_ShapeSize		(null),
								mCheckBox_Tessellation	(null)	{}
	virtual					~DynamicsOnMeshLevel()		{									}
	virtual	const char*		GetName()			const	{ return "DynamicsOnMeshLevel";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_DynamicsOnMeshLevel;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_PERFORMANCE;		}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 400;
		WD.mHeight	= 300;
		WD.mLabel	= "DynamicsOnMeshLevel";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 130;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			mCheckBox_Tessellation = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Tessellate mesh", UIElems, false, null, null);
			mCheckBox_Tessellation->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Grid size:", UIElems);
			mEditBox_GridSize = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "64", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			mEditBox_GridSize->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Grid scale (0 = automatic):", UIElems);
			mEditBox_GridScale = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			mEditBox_GridScale->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Nb layers:", UIElems);
			mEditBox_Layers = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			mEditBox_Layers->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Altitude:", UIElems);
			mEditBox_Altitude = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "10.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			mEditBox_Altitude->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Shape size:", UIElems);
			mEditBox_ShapeSize = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			mEditBox_ShapeSize->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Shape:", UIElems);
			mComboBox_ShapeType = CreateShapeTypeComboBox(UI, 4+OffsetX, y, false, SSM_UNDEFINED|SSM_SPHERE|SSM_CAPSULE|SSM_BOX|SSM_CONVEX);
			RegisterUIElement(mComboBox_ShapeType);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Convex:", UIElems);
			mComboBox_ConvexIndex = CreateConvexObjectComboBox(UI, 4+OffsetX, y, false);
			RegisterUIElement(mComboBox_ConvexIndex);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Mesh level:", UIElems);
			{
				ComboBoxDesc CBBD;
				CBBD.mID		= 0;
				CBBD.mParent	= UI;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "Mesh level";
				IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);
				CB->Add("Archipelago");
				CB->Add("Test zone");
				CB->Add("Flat mesh plane");
				CB->Select(MESH_LEVEL_ARCHIPELAGO);
				CB->SetVisible(true);
				CB->SetEnabled(false);
				mComboBox_MeshLevel = CB;
			}
			RegisterUIElement(mComboBox_MeshLevel);
			y += YStep;
		}
		{
			y += YStep;
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Presets (PEEL 1.0 scenes):", UIElems);

			class MyComboBox : public IceComboBox
			{
				DynamicsOnMeshLevel&	mTest;
				public:
								MyComboBox(const ComboBoxDesc& desc, DynamicsOnMeshLevel& test) : IceComboBox(desc), mTest(test)	{}
				virtual			~MyComboBox()																						{}
				virtual	void	OnComboBoxEvent(ComboBoxEvent event)
				{
					if(event==CBE_SELECTION_CHANGED)
					{
						const udword SelectedIndex = GetSelectedIndex();
						const bool Enabled = SelectedIndex==GetItemCount()-1;
						mTest.mEditBox_GridSize->SetEnabled(Enabled);
						mTest.mEditBox_GridScale->SetEnabled(Enabled);
						mTest.mEditBox_Layers->SetEnabled(Enabled);
						mTest.mEditBox_Altitude->SetEnabled(Enabled);
						mTest.mEditBox_ShapeSize->SetEnabled(Enabled);
						mTest.mCheckBox_Tessellation->SetEnabled(Enabled);
						mTest.mComboBox_ShapeType->SetEnabled(Enabled);
						mTest.mComboBox_ConvexIndex->SetEnabled(Enabled);
						mTest.mComboBox_MeshLevel->SetEnabled(Enabled);

						if(!Enabled && SelectedIndex<NB_PRESETS)
						{
							mTest.mEditBox_GridSize->SetText(_F("%d", gPreset_GridSize[SelectedIndex]));
							mTest.mEditBox_GridScale->SetText(_F("%.2f", gPreset_GridScale[SelectedIndex]));
							mTest.mEditBox_Layers->SetText(_F("%d", gPreset_Layers[SelectedIndex]));
							mTest.mEditBox_Altitude->SetText(_F("%.2f", gPreset_Altitude[SelectedIndex]));
							mTest.mEditBox_ShapeSize->SetText(_F("%.2f", gPreset_ShapeSize[SelectedIndex]));
							mTest.mCheckBox_Tessellation->SetChecked(gPreset_Tessellation[SelectedIndex]);
							mTest.mComboBox_ShapeType->Select(gPreset_ShapeType[SelectedIndex]);
							mTest.mComboBox_ConvexIndex->Select(gPreset_ConvexIndex[SelectedIndex]);
							mTest.mComboBox_MeshLevel->Select(gPreset_MeshLevel[SelectedIndex]);
						}
					}
				}
			};

			ComboBoxDesc CBBD;
			CBBD.mID		= 0;
			CBBD.mParent	= UI;
			CBBD.mX			= 4+OffsetX;
			CBBD.mY			= y;
			CBBD.mWidth		= 250;
			CBBD.mHeight	= 20;
			CBBD.mLabel		= "Presets";
			mComboBox_Preset = ICE_NEW(MyComboBox)(CBBD, *this);
			RegisterUIElement(mComboBox_Preset);
			mComboBox_Preset->Add("32*32 spheres on planar mesh");
			mComboBox_Preset->Add("32*32 boxes on planar mesh");
			mComboBox_Preset->Add("32*32 low-complexity convexes on planar mesh");
			mComboBox_Preset->Add("32*32 high-complexity convexes on planar mesh");
			mComboBox_Preset->Add("64*64 spheres on Archipelago - no tessellation");
			mComboBox_Preset->Add("64*64 capsules on Archipelago - no tessellation");
			mComboBox_Preset->Add("32*32 convexes on Archipelago - no tessellation");
			mComboBox_Preset->Add("32*32*4 convexes on Archipelago - no tessellation");
			mComboBox_Preset->Add("64*64 convexes on Archipelago - tessellation");
			mComboBox_Preset->Add("User-defined");
			mComboBox_Preset->Select(0);
			mComboBox_Preset->SetVisible(true);
			mComboBox_Preset->OnComboBoxEvent(CBE_SELECTION_CHANGED);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 400-16);
	}

	virtual	const char*		GetSubName()	const
	{
		if(mComboBox_Preset)
		{
			const udword SelectedIndex = mComboBox_Preset->GetSelectedIndex();
			if(SelectedIndex==0)
				return "SpheresOnPlanarMesh";
			else if(SelectedIndex==1)
				return "BoxesOnPlanarMesh";
			else if(SelectedIndex==2)
				return "SmallConvexesOnPlanarMesh";
			else if(SelectedIndex==3)
				return "BigConvexesOnPlanarMesh";
			else if(SelectedIndex==4)
				return "DynamicSpheresOnArchipelago";
			else if(SelectedIndex==5)
				return "DynamicCapsulesOnArchipelago";
			else if(SelectedIndex==6)
				return "DynamicConvexesOnArchipelago";
			else if(SelectedIndex==7)
				return "DynamicConvexesOnArchipelago2";
			else if(SelectedIndex==8)
				return "DynamicConvexesOnArchipelagoTess";
		}
		return null;
	}

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);

		const MeshLevel ML = mComboBox_MeshLevel ? MeshLevel(mComboBox_MeshLevel->GetSelectedIndex()) : MESH_LEVEL_ARCHIPELAGO;
		if(ML==MESH_LEVEL_ARCHIPELAGO)
		{
			desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
			desc.mCamera[0] = CameraPose(Point(7.76f, 29.64f, 10.64f), Point(0.74f, -0.39f, 0.55f));
			desc.mCamera[1] = CameraPose(Point(-74.41f, 161.22f, 314.37f), Point(0.66f, -0.61f, -0.45f));
		}
		else if(ML==MESH_LEVEL_FLAT)
		{
			desc.mCamera[0] = CameraPose(Point(-32.59f, 5.48f, 31.90f), Point(0.63f, -0.42f, -0.65f));
			desc.mCamera[1] = CameraPose(Point(52.18f, 26.82f, 52.73f), Point(-0.65f, -0.40f, -0.65f));
		}
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		const MeshLevel ML = mComboBox_MeshLevel ? MeshLevel(mComboBox_MeshLevel->GetSelectedIndex()) : MESH_LEVEL_ARCHIPELAGO;

		const bool Tessellation = mCheckBox_Tessellation ? mCheckBox_Tessellation->IsChecked() : false;

		if(ML==MESH_LEVEL_ARCHIPELAGO)
			LoadMeshesFromFile_(*this, "Archipelago.bin", null, false, Tessellation ? 2 : 0, TESS_POLYHEDRAL);
		else if(ML==MESH_LEVEL_TESTZONE)
			LoadMeshesFromFile_(*this, "testzone.bin", null, false, Tessellation ? 2 : 0, TESS_POLYHEDRAL);
		else if(ML==MESH_LEVEL_FLAT)
		{
			IndexedSurface* IS = CreateManagedSurface();
//			const udword Nb = Tessellation ? 64 : 32;
			const udword Nb = 32;
			bool status = IS->MakePlane(Nb, Nb);
			ASSERT(status);
//			const float S = Tessellation ? 0.05f : 0.1f;
			const float S = 0.1f;
			IS->Scale(Point(S, 1.0f, S));
			IS->Translate(Point(0.0f, 1.0f, 0.0f));
			IS->Flip();
		}

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)	// Mesh support will be tested in CreateMeshesFromRegisteredSurfaces
			return false;

		const float GridScale = GetFromEditBox(0.0f, mEditBox_GridScale, 0.0f, MAX_FLOAT);
		const udword GridSize = GetFromEditBox(64, mEditBox_GridSize);
		const udword NbLayers = GetFromEditBox(1, mEditBox_Layers);
		const float Altitude = GetFromEditBox(10.0f, mEditBox_Altitude, 0.0f, MAX_FLOAT);
		const float ShapeSize = GetFromEditBox(1.0f, mEditBox_ShapeSize, 0.0f, MAX_FLOAT);

		const PintShape ShapeType = mComboBox_ShapeType ? PintShape(mComboBox_ShapeType->GetSelectedIndex()) : PINT_SHAPE_UNDEFINED;
		if((ShapeType == PINT_SHAPE_CONVEX) && !caps.mSupportConvexes)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		float ScaleX = GridScale;
		float ScaleZ = GridScale;
		Point Offset(0.0f, 0.0f, 0.0f);
		if(GridScale==0.0f)
		{
			Point Extents;
			GetGlobalBounds(Offset, Extents);

			const float Margin = 8.0f;
			ScaleX = Extents.x - Margin;
			ScaleZ = Extents.z - Margin;
		}

		const udword NbX = GridSize;
		const udword NbY = GridSize;
		if(ShapeType == PINT_SHAPE_UNDEFINED)
			return true;
		if(ShapeType == PINT_SHAPE_SPHERE)
		{
			const float Radius = ShapeSize;
			for(udword i=0;i<NbLayers;i++)
			{
				Point CurrentOffset = Offset;
				CurrentOffset.x += float(i)*2.0f;
				GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Altitude+float(i)*3.0f, ScaleX, ScaleZ, 1.0f, 0, &CurrentOffset);
			}
			return true;
		}
		if(ShapeType == PINT_SHAPE_CAPSULE)
		{
			const float Radius = ShapeSize*0.5f;
			const float HalfHeight = 2.0f;
			for(udword i=0;i<NbLayers;i++)
			{
				Point CurrentOffset = Offset;
				CurrentOffset.x += float(i)*2.0f;
				GenerateArrayOfCapsules(pint, Radius, HalfHeight, NbX, NbY, Altitude+float(i)*3.0f, ScaleX, ScaleZ, 1.0f, 0, &CurrentOffset);
			}
			return true;
		}
		if(ShapeType == PINT_SHAPE_BOX)
		{
			const float Radius = ShapeSize;
			for(udword i=0;i<NbLayers;i++)
			{
				Point CurrentOffset = Offset;
				CurrentOffset.x += float(i)*2.0f;
				GenerateArrayOfBoxes(pint, Point(Radius, Radius, Radius), NbX, NbY, Altitude+float(i)*3.0f, ScaleX, ScaleZ, 1.0f, 0, &CurrentOffset);
			}
			return true;
		}
		if(ShapeType == PINT_SHAPE_CONVEX)
		{
			const ConvexIndex Index = mComboBox_ConvexIndex ? ConvexIndex(mComboBox_ConvexIndex->GetSelectedIndex()) : CONVEX_INDEX_0;

			MyConvex C;
			C.LoadFile(Index);
			C.Scale(ShapeSize);

			PINT_CONVEX_CREATE ConvexCreate(C.mNbVerts, C.mVerts);
			ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

			for(udword i=0;i<NbLayers;i++)
			{
				Point CurrentOffset = Offset;
				CurrentOffset.x += float(i)*2.0f;
				CreateArrayOfDynamicConvexes(pint, ConvexCreate, NbX, NbY, Altitude+float(i)*3.0f, ScaleX, ScaleZ, &CurrentOffset);
			}
			return true;
		}
		return false;
	}

}DynamicsOnMeshLevel;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddStaticObjects = "FLAWED TEST - TO REVISIT. Add static objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddStaticObjects, CATEGORY_PERFORMANCE, gDesc_AddStaticObjects)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;
	PINT_BOX_CREATE*	mBoxCreate;
	bool*				mFlags;
	Container			mCreated;

	virtual	bool	ProfileUpdate()
	{
		return true;
	}

	virtual	void	CommonRelease()
	{
		ICE_FREE(mFlags);
		DELETEARRAY(mBoxCreate);
		mCreated.Empty();

		TestBase::CommonRelease();
	}

	virtual bool	CommonSetup()
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

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
	/*
		if(!caps.mSupportRaycasts)
			return false;*/

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

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

	virtual void	CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
		RenderAllAABBs(renderer);
	}

	virtual udword	Update(Pint& pint, float dt)
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

static const char* gDesc_AddStaticObjects2 = "FLAWED TEST - TO REVISIT. 64*64 static boxes... added at runtime to 128*128 other static boxes.... This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddStaticObjects2, CATEGORY_PERFORMANCE, gDesc_AddStaticObjects2)

	bool	mStopTest;
	bool	mAddObjects;

	virtual	bool	ProfileUpdate()
	{
		return true;
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();
		mCreateDefaultEnvironment = false;
		mAddObjects = false;
		mStopTest = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateSeaOfStaticBoxes(pint, 100.0f, 128, 128, 0.0f);
	}

	virtual void	CommonUpdate(float dt)
	{
		if(mStopTest)
			return;

		if(mAddObjects)
		{
			mStopTest = true;
			mAddObjects = false;
			return;
		}

		TestBase::CommonUpdate(dt);

		if(mCurrentTime>=1.0f)
			mAddObjects = true;
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		if(mAddObjects)
			return CreateSeaOfStaticBoxes(pint, 50.0f, 64, 64, 1.0f);

		return 0;
	}

END_TEST(AddStaticObjects2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddDynamicObjects = "FLAWED TEST - TO REVISIT. Add dynamic objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddDynamicObjects, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjects)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;

	virtual	bool	ProfileUpdate()
	{
		return true;
	}

	virtual	void	CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		mMoving.SetEmpty();

		mBoxSize = 1.0f;
		mAmplitude = 80.0f;

		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

		const float t = mCurrentTime * 2.0f;
	//	const float t = mCurrentTime * 0.1f;

		const float PosScale = mAmplitude - mBoxSize;
		const float x = sinf(t*0.07f) * cosf(t*1.13f) * PosScale;
		const float y = sinf(t*2.07f) * cosf(t*0.13f) * PosScale;

		mMoving.SetCenterExtents(Point(x, 0.0f, y), Point(mBoxSize, 10.0f, mBoxSize));
	}

	virtual void	CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		PINT_BOX_CREATE BoxCreate(1.0f, 1.0f, 1.0f);
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

static const char* gDesc_AddDynamicObjects2 = "FLAWED TEST - TO REVISIT. Add dynamic objects at runtime. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_TEST(AddDynamicObjects2, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjects2)

	virtual	bool	ProfileUpdate()
	{
	//	return false;
		return true;
	}

	virtual	void	CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		const float BoxHeight = 8.0f;
		const float BoxSide = 0.01f;
		const float BoxDepth = 20.0f;
		CreateBoxContainer(pint, BoxHeight, BoxSide, BoxDepth);
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

		if(mCurrentTime>1.0f)
		{
			mCurrentTime = 0.0f;
		}
	}

	virtual void	CommonRender(PintRender& renderer)
	{
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		if(mCurrentTime==0.0f)
		{
			GenerateArrayOfBoxes(pint, Point(0.5f, 0.5f, 0.5f), 4, 4, 50.0f, 2.0f, 2.0f, 1.0f);
			GenerateArrayOfBoxes(pint, Point(0.5f, 0.5f, 0.5f), 4, 4, 48.0f, 2.0f, 2.0f, 1.0f);
	//		GenerateArrayOfSpheres(pint, 0.5f, 4, 4, 48.0f, 2.0f, 1.0f);
		}
		return 0;
	}

END_TEST(AddDynamicObjects2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddDynamicObjectsAndDoRaycasts = "FLAWED TEST - TO REVISIT. Add dynamic objects at runtime, and do raycasts. This is a test where the 'SQ Profiling Mode' makes a big difference. Use the 'Combined' mode in the PEEL generic UI to see the true cost of insertions. Please refer to the user manual for details.";

START_SQ_TEST(AddDynamicObjectsAndDoRaycasts, CATEGORY_PERFORMANCE, gDesc_AddDynamicObjectsAndDoRaycasts)

	AABB				mMoving;
	float				mAmplitude;
	float				mBoxSize;

	virtual	void	CommonRelease()
	{
		TestBase::CommonRelease();
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		mMoving.SetEmpty();

		mBoxSize = 1.0f;
		mAmplitude = 80.0f;

		bool Status = Setup_PotPourri_Raycasts(*this, 4096, 100.0f);
		mCreateDefaultEnvironment = true;
		return Status;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_PotPourri_Raycasts(pint, caps, 0.0f, 16, 16, 16);
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

		const float t = mCurrentTime * 2.0f;
	//	const float t = mCurrentTime * 0.1f;

		const float PosScale = mAmplitude - mBoxSize;
		const float x = sinf(t*0.07f) * cosf(t*1.13f) * PosScale;
		const float y = sinf(t*2.07f) * cosf(t*0.13f) * PosScale;

		mMoving.SetCenterExtents(Point(x, 0.0f, y), Point(mBoxSize, 10.0f, mBoxSize));
	}

	virtual void	CommonRender(PintRender& renderer)
	{
		renderer.DrawWirefameAABB(mMoving, Point(1.0f, 0.0f, 0.0f));
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		// PT: TODO: revisit this test. Creating renderers here pollutes the timings. Not good.
		PINT_BOX_CREATE BoxCreate(1.0f, 1.0f, 1.0f);
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

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.88f, 9.75f, 9.58f), Point(-0.68f, -0.33f, -0.65f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-10.0f, -10.0f, 0.0f), Point(430.0f, 100.0f, 300.0f));
		desc.mCamera[0] = CameraPose(Point(197.08f, 4.05f, 139.89f), Point(0.60f, -0.28f, 0.75f));
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

	virtual bool	IsPrivate()	const
	{
		return true;
	}

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(19.97f, 5.78f, 19.70f), Point(-0.65f, -0.30f, -0.69f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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
