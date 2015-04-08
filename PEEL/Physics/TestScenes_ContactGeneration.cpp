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

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicBoxVsLargeStaticSphere = "A dynamic box vs a large static sphere.";

START_TEST(DynamicBoxVsLargeStaticSphere, CATEGORY_CONTACT_GENERATION, gDesc_DynamicBoxVsLargeStaticSphere)

	virtual	void DynamicBoxVsLargeStaticSphere::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.53f, 12.54f, 14.78f), Point(-0.67f, -0.37f, -0.65f));
	}

	virtual bool DynamicBoxVsLargeStaticSphere::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 10.0f;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(1.0f, 1.0f, 1.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateDynamicObject(pint, &Create, Point(0.0f, Radius + 4.0f, 0.0f));
		}

		{
			PINT_SPHERE_CREATE Create;
			Create.mRadius		= Radius;
			Create.mRenderer	= CreateSphereRenderer(Create.mRadius);
			CreateStaticObject(pint, &Create, Point(0.0f, 0.0f, 0.0f));
		}
		return true;
	}

END_TEST(DynamicBoxVsLargeStaticSphere)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DynamicConvexVsVeryLargeStaticSphere = "A dynamic convex vs a very large static sphere. The sphere is so large that it looks more like a plane. \
Most engines are not prepared for such a case and objects jitter on top of the sphere...";

START_TEST(DynamicConvexVsVeryLargeStaticSphere, CATEGORY_CONTACT_GENERATION, gDesc_DynamicConvexVsVeryLargeStaticSphere)

	virtual	void DynamicConvexVsVeryLargeStaticSphere::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(9.38f, 23.17f, 5.67f), Point(-0.80f, -0.14f, -0.58f));
	}

	virtual bool DynamicConvexVsVeryLargeStaticSphere::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes)
			return false;

		{
			PINT_SPHERE_CREATE Create;
			Create.mRadius		= 10000.0f;
			Create.mRenderer	= CreateSphereRenderer(Create.mRadius);
			CreateStaticObject(pint, &Create, Point(0.0f, -10000.0f+20.0f, 0.0f));
		}

		{
			Point Pts[16];
			udword NbPts = GenerateConvex(Pts, 8, 8, 2.0f, 3.0f, 2.0f);
			ASSERT(NbPts==16);

			PINT_CONVEX_CREATE ConvexCreate;
			ConvexCreate.mNbVerts	= 16;
			ConvexCreate.mVerts		= Pts;
			ConvexCreate.mRenderer	= CreateConvexRenderer(16, Pts);

			PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, Point(0.0f, 30.0f, 0.0f));
			ASSERT(Handle);
		}
		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(DynamicConvexVsVeryLargeStaticSphere)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereOnLargeBox = "A sphere on a large box.";

START_TEST(SphereOnLargeBox, CATEGORY_CONTACT_GENERATION, gDesc_SphereOnLargeBox)

	virtual	void SphereOnLargeBox::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-35.08f, 9.73f, 32.02f), Point(0.67f, -0.13f, -0.73f));
	}

	virtual bool SphereOnLargeBox::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 1.0f;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(20.0f, 2.0f, 20.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateDynamicObject(pint, &Create, Point(0.0f, 2.0f, 0.0f));
		}

		{
			PINT_SPHERE_CREATE Create;
			Create.mRadius		= Radius;
			Create.mRenderer	= CreateSphereRenderer(Create.mRadius);
			CreateDynamicObject(pint, &Create, Point(0.0f, 8.0f, 0.0f));
		}
		return true;
	}

END_TEST(SphereOnLargeBox)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxOnLargeBox = "A box on a large box.";

START_TEST(BoxOnLargeBox, CATEGORY_CONTACT_GENERATION, gDesc_BoxOnLargeBox)

	virtual	void BoxOnLargeBox::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-29.88f, 7.64f, 30.09f), Point(0.65f, -0.06f, -0.76f));
	}

	virtual bool BoxOnLargeBox::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(20.0f, 2.0f, 20.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateDynamicObject(pint, &Create, Point(0.0f, 2.0f, 0.0f));
		}

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(4.0f, 1.0f, 1.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateDynamicObject(pint, &Create, Point(0.0f, 8.0f, 0.0f));
		}
		return true;
	}

END_TEST(BoxOnLargeBox)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleOnLargeBox = "A capsule on a large box. Should generate two VF contacts.";

START_TEST(CapsuleOnLargeBox, CATEGORY_CONTACT_GENERATION, gDesc_CapsuleOnLargeBox)

	virtual	void CapsuleOnLargeBox::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-29.88f, 7.64f, 30.09f), Point(0.65f, -0.06f, -0.76f));
	}

	virtual bool CapsuleOnLargeBox::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 1.0f;
		const float HalfHeight = 4.0f;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(20.0f, 2.0f, 20.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateDynamicObject(pint, &Create, Point(0.0f, 2.0f, 0.0f));
		}

		{
			PINT_CAPSULE_CREATE Create;
			Create.mRadius		= Radius;
			Create.mHalfHeight	= HalfHeight;
			Create.mRenderer	= CreateCapsuleRenderer(Create.mRadius, Create.mHalfHeight*2.0f);

			const Quat q = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
			CreateDynamicObject(pint, &Create, Point(0.0f, 8.0f, 0.0f), &q);
		}
		return true;
	}

END_TEST(CapsuleOnLargeBox)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ThinCapsuleOnBox = "Very thin capsule on static box. This is a stress test for the capsule-box contact generation.";

START_TEST(ThinCapsuleOnBox, CATEGORY_CONTACT_GENERATION, gDesc_ThinCapsuleOnBox)

	virtual	void ThinCapsuleOnBox::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-4.55f, 6.41f, 2.30f), Point(0.68f, -0.55f, -0.48f));
	}

	virtual bool ThinCapsuleOnBox::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 0.01f;
		const float HalfHeight = 4.0f;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(2.0f, 2.0f, 2.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateStaticObject(pint, &Create, Point(0.0f, 2.0f, 0.0f));
		}

		{
			PINT_CAPSULE_CREATE Create;
			Create.mRadius		= Radius;
			Create.mHalfHeight	= HalfHeight;
			Create.mRenderer	= CreateCapsuleRenderer(Create.mRadius, Create.mHalfHeight*2.0f);

			const Quat q = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
			CreateDynamicObject(pint, &Create, Point(0.0f, 4.0f-Radius, 0.0f), &q);
		}
		return true;
	}

END_TEST(ThinCapsuleOnBox)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ThinCapsuleOnBoxes = "Very thin capsule on static boxes. This is a stress test for the capsule-box contact generation.";

START_TEST(ThinCapsuleOnBoxes, CATEGORY_CONTACT_GENERATION, gDesc_ThinCapsuleOnBoxes)

	virtual	void ThinCapsuleOnBoxes::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-5.05f, 5.46f, 3.09f), Point(0.74f, -0.22f, -0.64f));
	}

	virtual bool ThinCapsuleOnBoxes::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 0.01f;
		const float HalfHeight = 4.0f;

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(2.0f, 2.0f, 2.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateStaticObject(pint, &Create, Point(-5.5f, 2.0f, 0.0f));
		}

		{
			PINT_BOX_CREATE Create;
			Create.mExtents		= Point(2.0f, 2.0f, 2.0f);
			Create.mRenderer	= CreateBoxRenderer(Create.mExtents);
			CreateStaticObject(pint, &Create, Point(5.5f, 2.0f, 0.0f));
		}

		{
			PINT_CAPSULE_CREATE Create;
			Create.mRadius		= Radius;
			Create.mHalfHeight	= HalfHeight;
			Create.mRenderer	= CreateCapsuleRenderer(Create.mRadius, Create.mHalfHeight*2.0f);

			const Quat q = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
			CreateDynamicObject(pint, &Create, Point(0.0f, 4.0f-Radius, 0.0f), &q);
		}
		return true;
	}

END_TEST(ThinCapsuleOnBoxes)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PCM_Stress = "Stress test for the PCM algorithm. Engines using PCM have a hard time simulating that one (at least with CCD disabled). \
Ideally the objects would fall to the ground and simply stop. PCM failures produce random weird looking motions, or even objects going through the ground.";

START_TEST(PCM_Stress, CATEGORY_CONTACT_GENERATION, gDesc_PCM_Stress)

	virtual	void PCM_Stress::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(33.36f, 23.93f, 38.67f), Point(-0.67f, -0.08f, -0.73f));
	}

	virtual bool PCM_Stress::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 10.0f;

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(100.0f, 0.05f, 100.0f);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxDesc;
		ObjectDesc.mPosition.x	= 0.0f;
		ObjectDesc.mPosition.y	= Altitude;
		ObjectDesc.mPosition.z	= 0.0f;
		ObjectDesc.mMass		= 0.0f;
		CreatePintObject(pint, ObjectDesc);


		PINT_CONVEX_CREATE ConvexCreate;
		MyConvex C;
	//	udword i=2;
		udword i=4;
	//	udword i=7;
	//	udword i=13;
		C.LoadFile(i);

		ConvexCreate.mNbVerts	= C.mNbVerts;
		ConvexCreate.mVerts		= C.mVerts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

		const float Amplitude = 1.5f;
		const udword NbX = 12;
		const udword NbY = 12;

		const float Scale = 4.0f;
		for(udword y=0;y<NbY;y++)
		{
			for(udword x=0;x<NbX;x++)
			{
				const float xf = (float(x)-float(NbX)*0.5f)*Scale;
				const float yf = (float(y)-float(NbY)*0.5f)*Scale;

	//			const Point pos = Point(xf, Altitude + 20.0f, yf);
				const Point pos = Point(xf, Altitude + 30.0f, yf);

				PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
				ASSERT(Handle);
			}
		}
		return true;
	}

END_TEST(PCM_Stress)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxSlidingOnPlanarMesh = "A box sliding on a tesselated planar mesh. This is to test how each engine handles collisions with 'internal edges'. \
Ideally the box would slide smoothly until the end, in a straight trajectory. Typical failures result in the box bumping away from the mesh.";

START_TEST(BoxSlidingOnPlanarMesh, CATEGORY_CONTACT_GENERATION, gDesc_BoxSlidingOnPlanarMesh)

	virtual bool BoxSlidingOnPlanarMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportRigidBodySimulation)
			return false;

		const float Altitude = 1.0f;

		IndexedSurface* IS;
		if(GetNbSurfaces())
		{
			IS = GetFirstSurface();
		}
		else
		{
			IS = CreateManagedSurface();
	//		bool status = IS->MakePlane(32, 32);
			bool status = IS->MakePlane(16, 32);
			ASSERT(status);
			IS->Scale(Point(0.01f, 1.0f, 0.1f));
			IS->Flip();
		}

		PINT_MESH_CREATE MeshDesc;
		MeshDesc.mSurface	= IS->GetSurfaceInterface();
		MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);

		Matrix3x3 m;
		m.RotX(degToRad(42.0f));

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &MeshDesc;
		ObjectDesc.mPosition	= Point(0.0f, Altitude, 0.0f);
		ObjectDesc.mRotation	= m;
		ObjectDesc.mMass		= 0.0f;
		CreatePintObject(pint, ObjectDesc);

		const float BoxExtent = 1.0f;
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 0.0f;
			MatDesc.mDynamicFriction	= 0.0f;
			MatDesc.mRestitution		= 0.0f;

			PintObjectHandle CubeHandle = CreateDynamicBox(pint, 1.0f, 1.0f, 1.0f, Point(0.0f, 40.0f, -41.0f), &ObjectDesc.mRotation, &MatDesc);
			ASSERT(CubeHandle);
		}
		return true;
	}

END_TEST(BoxSlidingOnPlanarMesh)

///////////////////////////////////////////////////////////////////////////////

static bool CreateScene_ShapeSlidingOnCurvedMesh(TestBase& test, Pint& pint, const PintCaps& caps, udword nb_x, udword nb_y, const Point& scale, const Point& shape_pos, PINT_SHAPE_CREATE& shape_create)
{
	if(!caps.mSupportMeshes || !caps.mSupportRigidBodySimulation)
		return false;

	const float Altitude = 1.0f;

	IndexedSurface* IS;
	if(test.GetNbSurfaces())
	{
		IS = test.GetFirstSurface();
	}
	else
	{
		IS = test.CreateManagedSurface();
		bool status = IS->MakePlane(nb_x, nb_y);
		ASSERT(status);
		IS->Scale(scale);
		IS->Flip();
		Point* P = IS->GetVerts();
		float Offset = 0.1f;
		udword i=0;
		for(udword y=0;y<nb_y;y++)
		{
			for(udword x=0;x<nb_x;x++)
			{
				P[i++].y += Offset;
			}
			Offset *= 1.25f;
		}
	}

	PINT_MESH_CREATE MeshDesc;
	MeshDesc.mSurface	= IS->GetSurfaceInterface();
	MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);

	Matrix3x3 m;
//	m.RotX(degToRad(42.0f));
	m.RotY(PI);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes		= &MeshDesc;
	ObjectDesc.mPosition	= Point(0.0f, Altitude, 0.0f);
	ObjectDesc.mRotation	= m;
	ObjectDesc.mMass		= 0.0f;
	CreatePintObject(pint, ObjectDesc);

	if(1)
	{
		PINT_MATERIAL_CREATE MatDesc;
		MatDesc.mStaticFriction		= 0.0f;
		MatDesc.mDynamicFriction	= 0.0f;
		MatDesc.mRestitution		= 0.0f;

		shape_create.mMaterial	= &MatDesc;

		PintObjectHandle ShapeHandle = CreateDynamicObject(pint, &shape_create, shape_pos);
		ASSERT(ShapeHandle);
	}
	return true;
}

static bool CreateScene_BoxSlidingOnCurvedMesh(TestBase& test, Pint& pint, const PintCaps& caps, udword nb_x, udword nb_y, const Point& scale, const Point& box_pos)
{
	PINT_BOX_CREATE BoxDesc;
	BoxDesc.mExtents	= Point(1.0f, 1.0f, 1.0f);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

	return CreateScene_ShapeSlidingOnCurvedMesh(test, pint, caps, nb_x, nb_y, scale, box_pos, BoxDesc);
}

static bool CreateScene_SphereSlidingOnCurvedMesh(TestBase& test, Pint& pint, const PintCaps& caps, udword nb_x, udword nb_y, const Point& scale, const Point& sphere_pos)
{
	PINT_SPHERE_CREATE SphereDesc;
	SphereDesc.mRadius		= 1.0f;
	SphereDesc.mRenderer	= CreateSphereRenderer(SphereDesc.mRadius);

	return CreateScene_ShapeSlidingOnCurvedMesh(test, pint, caps, nb_x, nb_y, scale, sphere_pos, SphereDesc);
}

static const char* gDesc_BoxSlidingOnCurvedMesh = "A box sliding on a tesselated curved mesh. This is to test how each engine handles collisions with 'internal edges'. \
Ideally the box would slide smoothly until the end, in a straight trajectory. Typical failures result in the box bumping away from the mesh.";

START_TEST(BoxSlidingOnCurvedMesh, CATEGORY_CONTACT_GENERATION, gDesc_BoxSlidingOnCurvedMesh)

	virtual bool BoxSlidingOnCurvedMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_BoxSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.1f, 1.0f, 0.1f), Point(0.0f, 80.0f, -55.0f));
	}

END_TEST(BoxSlidingOnCurvedMesh)

START_TEST(BoxSlidingOnCurvedMesh2, CATEGORY_CONTACT_GENERATION, gDesc_BoxSlidingOnCurvedMesh)

	virtual bool BoxSlidingOnCurvedMesh2::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_BoxSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.1f, 1.0f, 0.1f), Point(0.0f, 80.0f, -54.0f));
	}

END_TEST(BoxSlidingOnCurvedMesh2)

START_TEST(BoxSlidingOnCurvedMesh3, CATEGORY_CONTACT_GENERATION, gDesc_BoxSlidingOnCurvedMesh)

	virtual bool BoxSlidingOnCurvedMesh3::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_BoxSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.01f, 1.0f, 0.1f), Point(0.0f, 80.0f, -55.0f));
	}

END_TEST(BoxSlidingOnCurvedMesh3)

START_TEST(BoxSlidingOnCurvedMesh4, CATEGORY_CONTACT_GENERATION, gDesc_BoxSlidingOnCurvedMesh)

	virtual bool BoxSlidingOnCurvedMesh4::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_BoxSlidingOnCurvedMesh(*this, pint, caps, 16, 32, Point(0.01f, 1.0f, 0.1f), Point(0.0f, 80.0f, -54.0f));
	}

END_TEST(BoxSlidingOnCurvedMesh4)

static const char* gDesc_SphereSlidingOnCurvedMesh = "A sphere sliding on a tesselated curved mesh. This is to test how each engine handles collisions with 'internal edges'. \
Ideally the sphere would slide smoothly until the end, in a straight trajectory. Typical failures result in the sphere bumping away from the mesh.";

START_TEST(SphereSlidingOnCurvedMesh, CATEGORY_CONTACT_GENERATION, gDesc_SphereSlidingOnCurvedMesh)

	virtual bool SphereSlidingOnCurvedMesh::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_SphereSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.1f, 1.0f, 0.1f), Point(0.0f, 80.0f, -55.0f));
	}

END_TEST(SphereSlidingOnCurvedMesh)

START_TEST(SphereSlidingOnCurvedMesh2, CATEGORY_CONTACT_GENERATION, gDesc_SphereSlidingOnCurvedMesh)

	virtual bool SphereSlidingOnCurvedMesh2::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_SphereSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.1f, 1.0f, 0.1f), Point(0.0f, 80.0f, -54.0f));
	}

END_TEST(SphereSlidingOnCurvedMesh2)

START_TEST(SphereSlidingOnCurvedMesh3, CATEGORY_CONTACT_GENERATION, gDesc_SphereSlidingOnCurvedMesh)

	virtual bool SphereSlidingOnCurvedMesh3::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_SphereSlidingOnCurvedMesh(*this, pint, caps, 4, 32, Point(0.01f, 1.0f, 0.1f), Point(0.0f, 80.0f, -55.0f));
	}

END_TEST(SphereSlidingOnCurvedMesh3)

START_TEST(SphereSlidingOnCurvedMesh4, CATEGORY_CONTACT_GENERATION, gDesc_SphereSlidingOnCurvedMesh)

	virtual bool SphereSlidingOnCurvedMesh4::Setup(Pint& pint, const PintCaps& caps)
	{
		return CreateScene_SphereSlidingOnCurvedMesh(*this, pint, caps, 16, 32, Point(0.01f, 1.0f, 0.1f), Point(0.0f, 80.0f, -54.0f));
	}

END_TEST(SphereSlidingOnCurvedMesh4)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_VeryLargeTriangle = "Objects on a very large triangle. Used to test accuracy limits of various contact routines.";

START_TEST(VeryLargeTriangle, CATEGORY_CONTACT_GENERATION, gDesc_VeryLargeTriangle)

	virtual	void VeryLargeTriangle::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(8.68f, 4.96f, 9.24f), Point(-0.56f, -0.25f, -0.79f));
	}

	virtual bool VeryLargeTriangle::CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 5000.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool VeryLargeTriangle::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		if(1)
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 1.0f;
			MatDesc.mDynamicFriction	= 1.0f;
			MatDesc.mRestitution		= 0.0f;

			const float Radius = 1.0f;
			PINT_SPHERE_CREATE SphereDesc;
			SphereDesc.mRadius		= Radius;
			SphereDesc.mRenderer	= CreateSphereRenderer(SphereDesc.mRadius);
			SphereDesc.mMaterial	= &MatDesc;

			PintObjectHandle ShapeHandle = CreateDynamicObject(pint, &SphereDesc, Point(0.0f, Radius*2.0f, 0.0f));
			ASSERT(ShapeHandle);

			PINT_CAPSULE_CREATE CapsuleDesc;
			CapsuleDesc.mRadius		= Radius;
			CapsuleDesc.mHalfHeight	= Radius;
			CapsuleDesc.mRenderer	= CreateCapsuleRenderer(CapsuleDesc.mRadius, CapsuleDesc.mHalfHeight);
			CapsuleDesc.mMaterial	= &MatDesc;

			PintObjectHandle ShapeHandle2 = CreateDynamicObject(pint, &CapsuleDesc, Point(4.0f, Radius*2.0f, 0.0f));
			ASSERT(ShapeHandle2);
		}
		return true;
	}

END_TEST(VeryLargeTriangle)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereMeshUnitTest = "Sphere-vs-mesh unit test.";

START_TEST(SphereMeshUnitTest, CATEGORY_CONTACT_GENERATION, gDesc_SphereMeshUnitTest)

	virtual	void SphereMeshUnitTest::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-0.15f, 1.26f, 2.53f), Point(0.16f, -0.31f, -0.94f));
	}

	virtual bool SphereMeshUnitTest::CommonSetup()
	{
		ASSERT(!GetNbSurfaces());
		{
			const Point Offset(476.00385f, 48.477341f, 854.03748f);

			const float verts[] = {
			477.0f, 50.811596f, 853.0f, 476.0f, 50.858505f, 852.0f, 476.0f, 48.431885f, 854.0f,
			477.0f, 50.597446f, 854.0f, 477.0f, 50.811596f, 853.0f, 476.0f, 48.431885f, 854.0f,
			477.0f, 50.597446f, 854.0f, 476.0f, 48.431885f, 854.0f, 477.0f, 48.370255f, 855.0f,
			476.0f, 48.431885f, 854.0f, 474.0f, 48.624294f, 854.0f, 476.0f, 48.852272f, 856.0f,
			477.0f, 48.370255f, 855.0f, 476.0f, 48.431885f, 854.0f, 476.0f, 48.852272f, 856.0f,
			475.0f, 50.538040f, 853.0f, 476.0f, 48.431885f, 854.0f, 476.0f, 50.858505f, 852.0f,
			475.0f, 50.538040f, 853.0f, 474.0f, 48.624294f, 854.0f, 476.0f, 48.431885f, 854.0f
			};

			const udword NbFaces = 7;
			const udword NbVerts = 3*7;

			IndexedSurface* IS = CreateManagedSurface();
			bool Status = IS->Init(NbFaces, NbVerts);
			ASSERT(Status);

			Point* Verts = IS->GetVerts();
			for(udword j=0;j<NbVerts;j++)
			{
				Verts[j].x = verts[j*3+0] - Offset.x;
				Verts[j].y = verts[j*3+1] - Offset.y;
				Verts[j].z = verts[j*3+2] - Offset.z;
			}

			IndexedTriangle* F = const_cast<IndexedTriangle*>(IS->GetFaces());
			for(udword j=0;j<NbFaces;j++)
			{
				F[j].mRef[0] = j*3+0;
				F[j].mRef[1] = j*3+1;
				F[j].mRef[2] = j*3+2;
			}
		}
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SphereMeshUnitTest::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const float Radius = 0.036977537f;

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= Radius;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &SphereDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition.Zero();
		CreatePintObject(pint, ObjectDesc);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(SphereMeshUnitTest)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereMeshUnitTest_FC = "Sphere-vs-mesh unit test. Face contact. The sphere center initially exactly overlaps the triangle to test \
that singular contact are properly handled. This should generate a single vertical contact.";

START_TEST(SphereMeshUnitTest_FC, CATEGORY_CONTACT_GENERATION, gDesc_SphereMeshUnitTest_FC)

	virtual	void SphereMeshUnitTest_FC::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-0.79f, 2.50f, 6.29f), Point(0.16f, -0.31f, -0.94f));
	}

	virtual bool SphereMeshUnitTest_FC::CommonSetup()
	{
		ASSERT(!GetNbSurfaces());
		{
			const udword NbVerts = 3;
			Point Verts[NbVerts];
			const float Scale = 4.0f;
			for(udword i=0;i<NbVerts;i++)
			{
				const float Angle = 6.28f*float(i)/float(NbVerts);
				Verts[i].x = cosf(Angle)*Scale;
				Verts[i].y = 0;
				Verts[i].z = sinf(Angle)*Scale;
			}

			const udword NbFaces = 1;

			IndexedSurface* IS = CreateManagedSurface();
			bool Status = IS->Init(NbFaces, NbVerts, Verts);
			ASSERT(Status);

			IndexedTriangle* F = const_cast<IndexedTriangle*>(IS->GetFaces());
			for(udword j=0;j<NbFaces;j++)
			{
				F[j].mRef[0] = 0;
				F[j].mRef[1] = 2;
				F[j].mRef[2] = 1;
			}
		}
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SphereMeshUnitTest_FC::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const float Radius = 1.0f;

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= Radius;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &SphereDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition.Zero();
		CreatePintObject(pint, ObjectDesc);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(SphereMeshUnitTest_FC)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereMeshUnitTest_VC = "Sphere-vs-mesh unit test. Vertex contact. This should generate a single vertical contact.";

START_TEST(SphereMeshUnitTest_VC, CATEGORY_CONTACT_GENERATION, gDesc_SphereMeshUnitTest_VC)

	virtual	void SphereMeshUnitTest_VC::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-0.79f, 2.50f, 6.29f), Point(0.16f, -0.31f, -0.94f));
	}

	virtual bool SphereMeshUnitTest_VC::CommonSetup()
	{
		ASSERT(!GetNbSurfaces());
		{
			const udword NbVerts = 8;
			Point Verts[NbVerts];
			Verts[0] = Point(0.0f, 0.0f, 0.0f);
			const float Scale = 4.0f;
			const float z = -4.0f;
			for(udword i=0;i<NbVerts-1;i++)
			{
				const float Angle = 6.28f*float(i)/float(NbVerts-1);
				Verts[i+1].x = cosf(Angle)*Scale;
				Verts[i+1].y = z;
				Verts[i+1].z = sinf(Angle)*Scale;
			}

			const udword NbFaces = NbVerts-1;
			udword Indices[NbFaces*3];
			for(udword i=0;i<NbFaces;i++)
			{
				Indices[i*3+0] = 0;
				Indices[i*3+2] = i+1;
				udword j = i+2;
				if(j>=NbVerts)
					j = 1;
				Indices[i*3+1] = j;
			}

			IndexedSurface* IS = CreateManagedSurface();
			bool Status = IS->Init(NbFaces, NbVerts, Verts);
			ASSERT(Status);

			IndexedTriangle* F = const_cast<IndexedTriangle*>(IS->GetFaces());
			for(udword j=0;j<NbFaces;j++)
			{
				F[j].mRef[0] = Indices[j*3+0];
				F[j].mRef[1] = Indices[j*3+1];
				F[j].mRef[2] = Indices[j*3+2];
			}
		}
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SphereMeshUnitTest_VC::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const float Radius = 1.0f;

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= Radius;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &SphereDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= Point(0.0f, 3.0f, 0.0f);
		CreatePintObject(pint, ObjectDesc);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(SphereMeshUnitTest_VC)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereMeshUnitTest_EC = "Sphere-vs-mesh unit test. Edge contact. This should generate a single vertical contact. \
If you wait a bit, the sphere should not start moving along the edge (if it does, it means a wrong contact has been created). \
This simple test is actually quite challenging and few engines get it right.";

START_TEST(SphereMeshUnitTest_EC, CATEGORY_CONTACT_GENERATION, gDesc_SphereMeshUnitTest_EC)

	virtual	void SphereMeshUnitTest_EC::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(3.14f, 0.63f, 4.92f), Point(-0.51f, -0.17f, -0.84f));
	}

	virtual bool SphereMeshUnitTest_EC::CommonSetup()
	{
		ASSERT(!GetNbSurfaces());
		{
			const float Length = 2.0f;
			const float h = -2.0f;
			const float d = 2.0f;
			const udword NbVerts = 9;
			Point Verts[NbVerts];
			Verts[0] = Point(0.0f, 0.0f, -Length);
			Verts[1] = Point(0.0f, 0.0f, 0.0f);
			Verts[2] = Point(0.0f, 0.0f, Length);
			Verts[3] = Point(d, h, -Length);
			Verts[4] = Point(d, h, 0.0f);
			Verts[5] = Point(d, h, Length);
			Verts[6] = Point(-d, h, -Length);
			Verts[7] = Point(-d, h, 0.0f);
			Verts[8] = Point(-d, h, Length);

			const udword NbTris = 8;
			udword Indices[NbTris*3];
			udword o=0;
			Indices[o++] = 0;
			Indices[o++] = 1;
			Indices[o++] = 3;

			Indices[o++] = 1;
			Indices[o++] = 2;
			Indices[o++] = 4;

			IndexedSurface* IS = CreateManagedSurface();
			bool Status = IS->Init(o/3, NbVerts, Verts);
			ASSERT(Status);

			IndexedTriangle* F = const_cast<IndexedTriangle*>(IS->GetFaces());
			for(udword j=0;j<o/3;j++)
			{
				F[j].mRef[0] = Indices[j*3+0];
				F[j].mRef[1] = Indices[j*3+1];
				F[j].mRef[2] = Indices[j*3+2];
			}
		}
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool SphereMeshUnitTest_EC::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		const float Radius = 1.0f;

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRadius		= Radius;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &SphereDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= Point(0.0f, 1.0f, -0.05f);
		CreatePintObject(pint, ObjectDesc);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(SphereMeshUnitTest_EC)

///////////////////////////////////////////////////////////////////////////////
