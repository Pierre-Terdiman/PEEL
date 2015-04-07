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

static const char* gDesc_Dominos_High_Friction = "Classical 'domino' scene. Things to look for are whether the dominoes drift or not after they have all been knocked down. The speed at which dominoes are falling greatly depends on friction. This scene uses a high friction value (1.0)";

START_TEST(DominosHighFriction, CATEGORY_BEHAVIOR, gDesc_Dominos_High_Friction)

	virtual	void DominosHighFriction::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(13.88f, 11.57f, 13.32f), Point(-0.60f, -0.57f, -0.57f));
	}

	virtual bool DominosHighFriction::Setup(Pint& pint, const PintCaps& caps)
	{
		return Dominos_Setup(pint, caps, 1.0f);
	}

END_TEST(DominosHighFriction)

static const char* gDesc_Dominos_Low_Friction = "Classical 'domino' scene. Things to look for are whether the dominoes drift or not after they have all been knocked down. The speed at which dominoes are falling greatly depends on friction. This scene uses a low friction value (0.1)";

START_TEST(DominosLowFriction, CATEGORY_BEHAVIOR, gDesc_Dominos_Low_Friction)

	virtual	void DominosLowFriction::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(13.88f, 11.57f, 13.32f), Point(-0.60f, -0.57f, -0.57f));
	}

	virtual bool DominosLowFriction::Setup(Pint& pint, const PintCaps& caps)
	{
		return Dominos_Setup(pint, caps, 0.1f);
	}

END_TEST(DominosLowFriction)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_DominosEx = "A larger 'domino' scene. Things to look for are whether the dominoes drift or not after they have all been knocked down. The speed at which dominoes are falling greatly depends on friction. This scene uses a medium friction value (0.5)";

START_TEST(DominosEx, CATEGORY_BEHAVIOR, gDesc_DominosEx)

	virtual	void DominosEx::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(24.95f, 21.33f, 24.20f), Point(-0.57f, -0.63f, -0.53f));
	}

	virtual bool DominosEx::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		PINT_MATERIAL_CREATE MatDesc;
		MatDesc.mStaticFriction		= 0.0f;
		MatDesc.mDynamicFriction	= 0.5f;
		MatDesc.mRestitution		= 0.0f;

		const Point Extents(0.1f, 0.5f, 1.0f);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Extents;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mMaterial	= &MatDesc;

		const udword Nb = 128;
		Point Pts[Nb];
		GenerateCirclePts(Nb, Pts, 20.0f, 0.0f);

		const udword NbRounds = 6;
		float Scale = 1.0f;
		for(udword j=0;j<NbRounds;j++)
		{
			for(udword i=0;i<Nb;i++)
			{
				const float CoeffI = float(i)/float(Nb);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes		= &BoxDesc;
				ObjectDesc.mMass		= 1.0f;
				ObjectDesc.mPosition	= Pts[i]*Scale + Point(0.0f, Extents.z, 0.0f);
				Scale -= 0.1f*(1.0f/float(Nb));

				const Point Dir = (Pts[(i+1)%Nb] - Pts[i]).Normalize();
				Point Right, Up;
				ComputeBasis(Dir, Right, Up);

				Matrix3x3 Rot;
				Rot[0] = Dir;
				Rot[1] = Right;
				Rot[2] = Up;
				ObjectDesc.mRotation = Rot;

				if(i==0 && j==0)
				{
					ObjectDesc.mAngularVelocity = Point(10.0f, 0.0f, 0.0f);
				}

				CreatePintObject(pint, ObjectDesc);
			}
		}
		return true;
	}

END_TEST(DominosEx)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleStack = "The old capsule stack from the NovodeX SDK. Engines using 'adaptive force' tend to be stable here... until that feature is disabled.";

START_TEST(CapsuleStack, CATEGORY_BEHAVIOR, gDesc_CapsuleStack)

	virtual	void CapsuleStack::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(16.41f, 15.60f, 14.42f), Point(-0.66f, -0.37f, -0.65f));
	}

	virtual bool CapsuleStack::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 0.5f;
	//	const float Radius = 1.0f;
	//	const float Radius = 2.0f;
		const float HalfHeight = 4.0f;
		const float Scale = 0.75f;

		PINT_CAPSULE_CREATE Create;
		Create.mRadius		= Radius;
		Create.mHalfHeight	= HalfHeight;
		Create.mRenderer	= CreateCapsuleRenderer(Create.mRadius, Create.mHalfHeight*2.0f);

		const Quat q0 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
		const Quat q1 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(0.0f, 0.0f, 1.0f));

		float y = 0.0f;
	//	for(udword i=0;i<1;i++)
		for(udword i=0;i<8;i++)
	//	for(udword i=0;i<6;i++)
	//	for(udword i=0;i<4;i++)
	//	for(udword i=0;i<10;i++)
		{
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, HalfHeight*Scale), &q0);
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, -HalfHeight*Scale), &q0);

			CreateDynamicObject(pint, &Create, Point(HalfHeight*Scale, y+Radius*3.0f, 0.0f), &q1);
			CreateDynamicObject(pint, &Create, Point(-HalfHeight*Scale, y+Radius*3.0f, -0.0f), &q1);

			y += Radius*4.0f;
	//		y += Radius*5.0f;
		}
		return true;
	}

END_TEST(CapsuleStack)

static const char* gDesc_CapsuleStack2 = "Another capsule stack, simpler to simulate.";

START_TEST(CapsuleStack2, CATEGORY_BEHAVIOR, gDesc_CapsuleStack2)

	virtual	void CapsuleStack2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(18.36f, 13.76f, 18.33f), Point(-0.68f, -0.27f, -0.69f));
	}

	virtual bool CapsuleStack2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

	//	const float Radius = 0.5f;
		const float Radius = 1.0f;
	//	const float Radius = 2.0f;
		const float HalfHeight = 4.0f;
		const float Scale = 0.75f;

		PINT_CAPSULE_CREATE Create;
		Create.mRadius		= Radius;
		Create.mHalfHeight	= HalfHeight;
		Create.mRenderer	= CreateCapsuleRenderer(Create.mRadius, Create.mHalfHeight*2.0f);

		const Quat q0 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
		const Quat q1 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(0.0f, 0.0f, 1.0f));

		float y = 0.0f;
	//	for(udword i=0;i<1;i++)
	//	for(udword i=0;i<8;i++)
	//	for(udword i=0;i<6;i++)
		for(udword i=0;i<4;i++)		// We only had 4 layers in the old NX scene
	//	for(udword i=0;i<10;i++)
		{
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, HalfHeight*Scale), &q0);
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, -HalfHeight*Scale), &q0);

			CreateDynamicObject(pint, &Create, Point(HalfHeight*Scale, y+Radius*3.0f, 0.0f), &q1);
			CreateDynamicObject(pint, &Create, Point(-HalfHeight*Scale, y+Radius*3.0f, -0.0f), &q1);

			y += Radius*4.0f;
	//		y += Radius*5.0f;
		}
		return true;
	}

END_TEST(CapsuleStack2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxTower = "The 'capsule stack', but with boxes.";

START_TEST(BoxTower, CATEGORY_BEHAVIOR, gDesc_BoxTower)

	virtual	void BoxTower::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(16.41f, 15.60f, 14.42f), Point(-0.66f, -0.37f, -0.65f));
	}

	virtual bool BoxTower::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 0.5f;
	//	const float Radius = 1.0f;
	//	const float Radius = 2.0f;
		const float HalfHeight = 4.0f;
		const float Scale = 0.75f;

		PINT_BOX_CREATE Create;
		Create.mExtents		= Point(Radius, HalfHeight, Radius);
		Create.mRenderer	= CreateBoxRenderer(Create.mExtents);

		const Quat q0 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
		const Quat q1 = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(0.0f, 0.0f, 1.0f));

		float y = 0.0f;
	//	for(udword i=0;i<1;i++)
		for(udword i=0;i<8;i++)
	//	for(udword i=0;i<6;i++)
	//	for(udword i=0;i<4;i++)
	//	for(udword i=0;i<10;i++)
		{
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, HalfHeight*Scale), &q0);
			CreateDynamicObject(pint, &Create, Point(0.0f, y+Radius, -HalfHeight*Scale), &q0);

			CreateDynamicObject(pint, &Create, Point(HalfHeight*Scale, y+Radius*3.0f, 0.0f), &q1);
			CreateDynamicObject(pint, &Create, Point(-HalfHeight*Scale, y+Radius*3.0f, -0.0f), &q1);

			y += Radius*4.0f;
	//		y += Radius*5.0f;
		}
		return true;
	}

END_TEST(BoxTower)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_StackedSpheres = "Stacked spheres.";

START_TEST(StackedSpheres, CATEGORY_BEHAVIOR, gDesc_StackedSpheres)

	virtual	void StackedSpheres::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(19.12f, 19.17f, 19.28f), Point(-0.68f, -0.21f, -0.70f));
	}

	virtual bool StackedSpheres::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 1.0f;

		PINT_SPHERE_CREATE Create;
		Create.mRadius		= Radius;
		Create.mRenderer	= CreateSphereRenderer(Radius);

		for(udword i=0;i<10;i++)
		{
			CreateDynamicObject(pint, &Create, Point(0.0f, Radius*2.0f + (Radius*2.0f+1.0f)*float(i), 0.0f));
		}
		return true;
	}

END_TEST(StackedSpheres)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_StackedCapsules = "Stacked capsules.";

START_TEST(StackedCapsules, CATEGORY_BEHAVIOR, gDesc_StackedCapsules)

	virtual	void StackedCapsules::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(26.89f, 27.29f, 29.30f), Point(-0.69f, -0.16f, -0.71f));
	}

	virtual bool StackedCapsules::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 1.0f;
		const float HalfHeight = 1.0f;

		PINT_CAPSULE_CREATE Create;
		Create.mRadius		= Radius;
		Create.mHalfHeight	= HalfHeight;
		Create.mRenderer	= CreateCapsuleRenderer(Radius, HalfHeight*2.0f);

		for(udword i=0;i<10;i++)
		{
			CreateDynamicObject(pint, &Create, Point(0.0f, Radius+HalfHeight*2.0f + (Radius+HalfHeight*2.0f+1.0f)*float(i), 0.0f));
		}
		return true;
	}

END_TEST(StackedCapsules)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_InitialVelocities = "Initial velocities.";

START_TEST(InitialVelocities, CATEGORY_BEHAVIOR, gDesc_InitialVelocities)

	virtual	void InitialVelocities::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.05f, 8.77f, 16.60f), Point(-0.64f, -0.20f, -0.74f));
	}

	virtual bool InitialVelocities::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float Radius = 1.0f;

		PINT_SPHERE_CREATE Create;
		Create.mRadius		= Radius;
		Create.mRenderer	= CreateSphereRenderer(Radius);

		PINT_BOX_CREATE Create2;
		Create2.mExtents	= Point(Radius, Radius, Radius);
		Create2.mRenderer	= CreateBoxRenderer(Create2.mExtents);

		const Point LinVel(0.0f, 10.0f, 0.0f);
		const Point LinVel2(4.0f, 10.0f, 0.0f);
		const Point AngVel(0.0f, 0.0f, 10.0f);

		CreateDynamicObject(pint, &Create, Point(0.0f, Radius*2.0f, 0.0f), null, &LinVel, &AngVel);
		CreateDynamicObject(pint, &Create, Point(0.0f, Radius*2.0f, 4.0f), null, &LinVel2, &AngVel);
		CreateDynamicObject(pint, &Create2, Point(0.0f, Radius*2.0f, 8.0f), null, &LinVel, &AngVel);
		return true;
	}

END_TEST(InitialVelocities)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HighMassRatio = "High mass ratio test. The top box in those box stacks is 100, 10000 or 1000000 times heavier than the other boxes.";

START_TEST(HighMassRatio, CATEGORY_BEHAVIOR, gDesc_HighMassRatio)

	virtual	void HighMassRatio::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(23.81f, 20.65f, 39.66f), Point(-0.65f, -0.31f, -0.69f));
	}

	virtual bool HighMassRatio::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 1.0f;

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(BoxExtent, BoxExtent, BoxExtent);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		const udword NbStacks = 3;
		for(udword j=0;j<NbStacks;j++)
		{
			udword NbBoxes = 10;
			float BoxPosY = BoxExtent;
			while(NbBoxes)
			{
				for(udword i=0;i<NbBoxes;i++)
				{
					const float Coeff = float(i) - float(NbBoxes)*0.5f;

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &BoxDesc;
					ObjectDesc.mPosition.x	= Coeff * BoxExtent * 2.0f;
					ObjectDesc.mPosition.y	= BoxPosY;
					ObjectDesc.mPosition.z	= float(j) * BoxExtent * 8.0f;
					ObjectDesc.mMass		= NbBoxes == 1 ? float((j+1)*100): 1.0f;
					PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
					ASSERT(Handle);
				}

				NbBoxes--;
				BoxPosY += BoxExtent*2.0f;
			}
		}
		return true;
	}

END_TEST(HighMassRatio)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_InitialPenetration = "A test to check how engines deal with objects created in an initially overlapping state. Ideally this should be resolved \
gently, without objects exploding.";

START_TEST(InitialPenetration, CATEGORY_BEHAVIOR, gDesc_InitialPenetration)

	virtual bool InitialPenetration::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 3.0f;
		const float BoxPosY = BoxExtent;
		const udword NbBoxes = 4;
		for(udword i=0;i<NbBoxes;i++)
		{
			PintObjectHandle Handle = CreateDynamicBox(pint, BoxExtent, BoxExtent, BoxExtent, Point(0.0f, BoxPosY + float(i)*BoxExtent*1.5f, 0.0f));
	//		PintObjectHandle Handle = CreateDynamicBox(pint, BoxExtent, BoxExtent, BoxExtent, Point(0.0f, BoxPosY + float(i)*BoxExtent*5.0f, 0.0f));
			ASSERT(Handle);
		}
		return true;
	}

END_TEST(InitialPenetration)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Friction = "Classical friction ramp test. Static friction is 0.0. Dynamic friction coeffs range from 0.0 to 1.0.";

START_TEST(Friction, CATEGORY_BEHAVIOR, gDesc_Friction)

	virtual bool Friction::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 10.0f;
		const float BoxPosY = BoxExtent;
		const udword NbBoxes = 8;
		for(udword i=0;i<NbBoxes;i++)
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 0.0f;
			MatDesc.mDynamicFriction	= float(i)/float(NbBoxes-1);
			MatDesc.mRestitution		= 0.0f;

			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mMaterial	= &MatDesc;
			BoxDesc.mExtents.x	= BoxExtent*0.5f;
			BoxDesc.mExtents.y	= 0.5f;
			BoxDesc.mExtents.z	= BoxExtent;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			const float x = (float(i)-float(NbBoxes)*0.5f)*BoxExtent*1.1f;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mPosition.x	= x;
			ObjectDesc.mPosition.y	= BoxPosY;
			ObjectDesc.mPosition.z	= 0.0f;

			Matrix3x3 m;
	//		m.RotX(degToRad(40.0f));
	//		m.RotX(degToRad(45.0f));
			m.RotX(degToRad(42.0f));
			ObjectDesc.mRotation = m;

			ObjectDesc.mMass		= 0.0f;
			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);

			PintObjectHandle CubeHandle = CreateDynamicBox(pint, 1.0f, 1.0f, 1.0f, Point(x, 18.0f, -6.0f), &ObjectDesc.mRotation, &MatDesc);
			ASSERT(CubeHandle);
		}
		return true;
	}

END_TEST(Friction)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Friction2 = "A variation on the classical friction ramp test.";

START_TEST(Friction2, CATEGORY_BEHAVIOR, gDesc_Friction2)

	virtual bool Friction2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 10.0f;
		const float BoxPosY = BoxExtent;
		const udword NbBoxes = 8;
		for(udword i=0;i<NbBoxes;i++)
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 0.0f;
			MatDesc.mDynamicFriction	= float(i)/float(NbBoxes-1);
			MatDesc.mRestitution		= 0.0f;

			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mMaterial	= &MatDesc;
			BoxDesc.mExtents.x	= BoxExtent*0.5f;
			BoxDesc.mExtents.y	= 0.5f;
			BoxDesc.mExtents.z	= BoxExtent;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			const float x = (float(i)-float(NbBoxes)*0.5f)*BoxExtent*1.1f;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mPosition.x	= x;
			ObjectDesc.mPosition.y	= BoxPosY;
			ObjectDesc.mPosition.z	= 0.0f;

			Matrix3x3 m;
	//		m.RotX(degToRad(40.0f));
	//		m.RotX(degToRad(45.0f));
			m.RotX(degToRad(42.0f));
			ObjectDesc.mRotation = m;

			ObjectDesc.mMass		= 0.0f;
			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);

			for(udword j=0;j<4;j++)
			{
				PintObjectHandle BoxHandle = CreateDynamicBox(pint, 4.0f, 0.5f, 4.0f, Point(x, 18.0f + float(j)*2.0f, -6.0f), &ObjectDesc.mRotation, &MatDesc);
				ASSERT(BoxHandle);
			}
		}
		return true;
	}

END_TEST(Friction2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Friction3 = "A variation on the classical friction ramp test. Each box has an initial angular velocity of 10.";

START_TEST(Friction3, CATEGORY_BEHAVIOR, gDesc_Friction3)

	virtual bool Friction3::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 10.0f;
		const float BoxPosY = BoxExtent;
		const udword NbBoxes = 8;
		for(udword i=0;i<NbBoxes;i++)
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 0.0f;
			MatDesc.mDynamicFriction	= float(i)/float(NbBoxes-1);
			MatDesc.mRestitution		= 0.0f;

			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mMaterial	= &MatDesc;
			BoxDesc.mExtents.x	= BoxExtent*0.5f;
			BoxDesc.mExtents.y	= 0.5f;
			BoxDesc.mExtents.z	= BoxExtent;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			const float x = (float(i)-float(NbBoxes)*0.5f)*BoxExtent*1.1f;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mPosition.x	= x;
			ObjectDesc.mPosition.y	= BoxPosY;
			ObjectDesc.mPosition.z	= 0.0f;

			Matrix3x3 m;
			m.RotX(degToRad(30.0f));
			ObjectDesc.mRotation = m;

			ObjectDesc.mMass		= 0.0f;
			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);

	//		PintObjectHandle BoxHandle = CreateDynamicBox(pint, 4.0f, 0.5f, 4.0f, Point(x, 13.5f, -4.0f), &ObjectDesc.mRotation, &MatDesc);
	//		ASSERT(BoxHandle);
			{
				PINT_BOX_CREATE BoxDesc;
				BoxDesc.mMaterial	= &MatDesc;
				BoxDesc.mExtents.x	= 2.0f;
				BoxDesc.mExtents.y	= 0.5f;
				BoxDesc.mExtents.z	= 2.0f;
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 1.0f;
				ObjectDesc.mPosition		= Point(x, 13.5f, -4.0f);
				ObjectDesc.mRotation		= m;
				ObjectDesc.mAngularVelocity	= Point(0.0f, 10.0f, 0.0f);
				PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
				ASSERT(Handle);
			}
		}
		return true;
	}

END_TEST(Friction3)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Restitution = "Classical restitution test. Coeffs of restitution range from 0.0 to 1.0. Ideally when restitution is 1.0, the falling object should \
bounce as high as the position it started from, but not higher. If it does, the physics engine is adding energy into the system (which is wrong). (Disable linear damping \
to check if an engine is correct or not, as damping usually hides the issue).";

START_TEST(Restitution, CATEGORY_BEHAVIOR, gDesc_Restitution)

	virtual bool Restitution::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float BoxExtent = 3.0f;
		const float BoxPosY = BoxExtent;
		const udword NbBoxes = 8;
		const float Radius = 1.0f;

		PintShapeRenderer* RenderObject = CreateSphereRenderer(Radius);

		for(udword i=0;i<NbBoxes;i++)
		{
			PINT_MATERIAL_CREATE MatDesc;
			MatDesc.mStaticFriction		= 0.5f;
			MatDesc.mDynamicFriction	= 0.5f;
			MatDesc.mRestitution		= float(i)/float(NbBoxes-1);

			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mMaterial	= &MatDesc;
			BoxDesc.mExtents.x	= BoxExtent*0.5f;
			BoxDesc.mExtents.y	= 0.5f;
			BoxDesc.mExtents.z	= BoxExtent;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			const float x = (float(i)-float(NbBoxes)*0.5f)*BoxExtent*1.1f;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mPosition.x	= x;
			ObjectDesc.mPosition.y	= BoxPosY;
			ObjectDesc.mPosition.z	= 0.0f;

			ObjectDesc.mMass		= 0.0f;
			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);

			PINT_SPHERE_CREATE SphereDesc;
			SphereDesc.mMaterial	= &MatDesc;
			SphereDesc.mRenderer	= RenderObject;
			SphereDesc.mRadius		= Radius;
			PintObjectHandle SphereHandle = CreateDynamicObject(pint, &SphereDesc, Point(x, 20.0f, 0.0f));
			ASSERT(SphereHandle);
		}
		return true;
	}

END_TEST(Restitution)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_GravityRace = "A dynamic box is dropped 1000 units away from the origin, and falls towards it. In theory boxes simulated with different engines \
should all touch the ground at the same time... In practice the different implementations of 'linear damping' make boxes arrive at different times (i.e. even if they use the \
same numerical value for linear damping). Wait until frame 1000 or so, for the boxes to reach the ground.";

START_TEST(GravityRace, CATEGORY_BEHAVIOR, gDesc_GravityRace)

	virtual bool GravityRace::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const Point Extents(1.0f, 1.0f, 1.0f);

		PINT_BOX_CREATE Desc;
		Desc.mExtents	= Extents;
		Desc.mRenderer	= CreateBoxRenderer(Extents);
		PintObjectHandle Handle = CreateDynamicObject(pint, &Desc, Point(0.0f, 1000.0f, 0.0f));
		ASSERT(Handle);
		return true;
	}

END_TEST(GravityRace)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConvexCompoundsChain = "A chain made of 20 torus object. Each torus is a compound of 32 convexes. This is a good scene to see the artefacts caused \
by the 'adaptive force' hack. The chain will appear to fall more slowly with engines using 'adaptive force'. Pick & drag objects to test each engine's stability. This is also a CCD torture test (all engines fail).";

START_TEST(ConvexCompoundsChain, CATEGORY_BEHAVIOR, gDesc_ConvexCompoundsChain)

	virtual	void ConvexCompoundsChain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(63.41f, 50.44f, 84.89f), Point(-0.47f, -0.07f, -0.88f));
	}

	virtual bool ConvexCompoundsChain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportConvexes || !caps.mSupportRigidBodySimulation || !caps.mSupportCompounds)
			return false;

		// Generate torus
		const float BigRadius = 3.0f;
	//	const float SmallRadius = 1.0f;
		const float SmallRadius = 0.75f;
	//	const udword NbPtsSmallCircle = 8;
	//	const udword NbSlices = 16;
		const udword NbPtsSmallCircle = 16;
		const udword NbSlices = 32;

		Point Pts[NbPtsSmallCircle];
		GeneratePolygon(NbPtsSmallCircle, Pts, sizeof(Point), ORIENTATION_XY, SmallRadius);

		udword TotalNbVerts = NbPtsSmallCircle * NbSlices;
		Point* Verts = ICE_NEW(Point)[TotalNbVerts];

		udword Index = 0;
		for(udword j=0;j<NbSlices;j++)
		{
			const float Coeff = float(j)/float(NbSlices);

			Matrix3x3 Rot;
			Rot.RotX(Coeff * TWOPI);

			const Point Trans = Rot[1]*BigRadius;
			for(udword i=0;i<NbPtsSmallCircle;i++)
				Verts[Index++] = Trans + Pts[i]*Rot; 
		}
		ASSERT(Index==TotalNbVerts);

		PINT_CONVEX_CREATE ConvexCreate[NbSlices];
		Point ConvexPts[NbSlices*NbPtsSmallCircle*2];
		udword Offset = 0;

		for(udword s=0;s<NbSlices;s++)
		{
			const udword SliceIndex0 = s;
			const udword SliceIndex1 = (s+1)%NbSlices;
			const Point* V0 = Verts + SliceIndex0*NbPtsSmallCircle;
			const Point* V1 = Verts + SliceIndex1*NbPtsSmallCircle;

	//		Point ConvexPts[NbPtsSmallCircle*2];
			for(udword i=0;i<NbPtsSmallCircle;i++)
			{
	//			ConvexPts[i] = V0[i];
	//			ConvexPts[i+NbPtsSmallCircle] = V1[i];
				ConvexPts[Offset+i] = V0[i];
				ConvexPts[Offset+i+NbPtsSmallCircle] = V1[i];
			}

			if(1)	// Recenter vertices
			{
				Point Center(0.0f, 0.0f, 0.0f);
				const float Coeff = 1.0f / float(NbPtsSmallCircle*2);
				for(udword i=0;i<NbPtsSmallCircle*2;i++)
					Center += ConvexPts[Offset+i] * Coeff;

				for(udword i=0;i<NbPtsSmallCircle*2;i++)
					ConvexPts[Offset+i] -= Center;

				ConvexCreate[s].mLocalPos = Center;// * 1.1f;
			}

			ConvexCreate[s].mNbVerts	= NbPtsSmallCircle*2;
			ConvexCreate[s].mVerts		= &ConvexPts[Offset];
			ConvexCreate[s].mRenderer	= CreateConvexRenderer(ConvexCreate[s].mNbVerts, ConvexCreate[s].mVerts);
			if(s!=NbSlices-1)
				ConvexCreate[s].mNext	= &ConvexCreate[s+1];
			Offset += NbPtsSmallCircle*2;

	/*
			PINT_CONVEX_CREATE ConvexCreate;
			ConvexCreate.mNbVerts	= NbPtsSmallCircle*2;
			ConvexCreate.mVerts		= ConvexPts;
			ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexPts);

			const Point pos(0.0f, 10.0f, 0.0f);
			PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
			ASSERT(Handle);*/
		}
		DELETEARRAY(Verts);

		const udword NbObjects = 20;
		Matrix3x3 RotX;	RotX.RotZ(HALFPI);
		Matrix3x3 RotY;	RotY.RotY(HALFPI);
		const Quat QX = RotX;
		const Quat QY = RotY;
		for(udword i=0;i<NbObjects;i++)
		{
	//		const Quat* Rot0 = i&1 ? &Q : null;
			const Quat* Rot0 = i&1 ? &QX : &QY;

	//		const Point pos(0.0f, 30.0f + float(i)*4.0f, 0.0f);
			const Point pos(float(i)*4.0f, 10.0f + float(NbObjects)*4.0f, 0.0f);
	//		if(i==NbObjects-1)
			if(i==0)
			{
				PintObjectHandle Handle = CreateStaticObject(pint, &ConvexCreate[0], pos, Rot0);
				ASSERT(Handle);
			}
			else
			{
				PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate[0], pos, Rot0);
				ASSERT(Handle);
			}
		}

		return true;
	}

END_TEST(ConvexCompoundsChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxCompoundsChain = "A chain made of 20 torus object. Each torus is a compound of 32 boxes.This is a good scene to see the artefacts caused \
by the 'adaptive force' hack. The chain will appear to fall more slowly with engines using 'adaptive force'. Pick & drag objects to test each engine's stability. This is also a CCD torture test (all engines fail).";

START_TEST(BoxCompoundsChain, CATEGORY_BEHAVIOR, gDesc_BoxCompoundsChain)

	virtual	void BoxCompoundsChain::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(63.41f, 50.44f, 84.89f), Point(-0.47f, -0.07f, -0.88f));
	}

	virtual bool BoxCompoundsChain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportCompounds)
			return false;

		// Generate torus
		const float BigRadius = 3.0f;
	//	const float SmallRadius = 1.0f;
		const float SmallRadius = 0.75f;
	//	const udword NbPtsSmallCircle = 8;
	//	const udword NbSlices = 16;
		const udword NbPtsSmallCircle = 16;
		const udword NbSlices = 32;

		Point Pts[NbPtsSmallCircle];
		GeneratePolygon(NbPtsSmallCircle, Pts, sizeof(Point), ORIENTATION_XY, SmallRadius);

		udword TotalNbVerts = NbPtsSmallCircle * NbSlices;
		Point* Verts = ICE_NEW(Point)[TotalNbVerts];

		udword Index = 0;
		for(udword j=0;j<NbSlices;j++)
		{
			const float Coeff = float(j)/float(NbSlices);

			Matrix3x3 Rot;
			Rot.RotX(Coeff * TWOPI);

			const Point Trans = Rot[1]*BigRadius;
			for(udword i=0;i<NbPtsSmallCircle;i++)
				Verts[Index++] = Trans + Pts[i]*Rot; 
		}
		ASSERT(Index==TotalNbVerts);

		PINT_BOX_CREATE BoxCreate[NbSlices];
		Point ConvexPts[NbSlices*NbPtsSmallCircle*2];
		udword Offset = 0;

		for(udword s=0;s<NbSlices;s++)
		{
			const udword SliceIndex0 = s;
			const udword SliceIndex1 = (s+1)%NbSlices;
			const Point* V0 = Verts + SliceIndex0*NbPtsSmallCircle;
			const Point* V1 = Verts + SliceIndex1*NbPtsSmallCircle;

	//		Point ConvexPts[NbPtsSmallCircle*2];
			for(udword i=0;i<NbPtsSmallCircle;i++)
			{
	//			ConvexPts[i] = V0[i];
	//			ConvexPts[i+NbPtsSmallCircle] = V1[i];
				ConvexPts[Offset+i] = V0[i];
				ConvexPts[Offset+i+NbPtsSmallCircle] = V1[i];
			}

			if(1)	// Recenter vertices
			{
				Point Center(0.0f, 0.0f, 0.0f);
				const float Coeff = 1.0f / float(NbPtsSmallCircle*2);
				for(udword i=0;i<NbPtsSmallCircle*2;i++)
					Center += ConvexPts[Offset+i] * Coeff;

				for(udword i=0;i<NbPtsSmallCircle*2;i++)
					ConvexPts[Offset+i] -= Center;

				BoxCreate[s].mLocalPos = Center;// * 1.1f;
			}

//			BoxCreate[s].mNbVerts	= NbPtsSmallCircle*2;
//			BoxCreate[s].mVerts		= &ConvexPts[Offset];
			BoxCreate[s].mExtents	= Point(1.0f, 0.5f, 0.5f);
			BoxCreate[s].mRenderer	= CreateBoxRenderer(BoxCreate[s].mExtents);
			if(s!=NbSlices-1)
				BoxCreate[s].mNext	= &BoxCreate[s+1];
			Offset += NbPtsSmallCircle*2;
		}
		DELETEARRAY(Verts);

		const udword NbObjects = 20;
		Matrix3x3 RotX;	RotX.RotZ(HALFPI);
		Matrix3x3 RotY;	RotY.RotY(HALFPI);
		const Quat QX = RotX;
		const Quat QY = RotY;
		for(udword i=0;i<NbObjects;i++)
		{
	//		const Quat* Rot0 = i&1 ? &Q : null;
			const Quat* Rot0 = i&1 ? &QX : &QY;

	//		const Point pos(0.0f, 30.0f + float(i)*4.0f, 0.0f);
			const Point pos(float(i)*4.0f, 10.0f + float(NbObjects)*4.0f, 0.0f);
	//		if(i==NbObjects-1)
			if(i==0)
			{
				PintObjectHandle Handle = CreateStaticObject(pint, &BoxCreate[0], pos, Rot0);
				ASSERT(Handle);
			}
			else
			{
				PintObjectHandle Handle = CreateDynamicObject(pint, &BoxCreate[0], pos, Rot0);
				ASSERT(Handle);
			}
		}

		return true;
	}

END_TEST(BoxCompoundsChain)

///////////////////////////////////////////////////////////////////////////////

static void GenerateFixedJointsTorus(Pint& pint, const Point& torus_pos, const Quat& torus_rot, float mass)
{
	// Generate torus
	const float BigRadius = 3.0f;
	const float SmallRadius = 1.0f;
//	const float SmallRadius = 0.75f;
//	const udword NbPtsSmallCircle = 8;
//	const udword NbSlices = 16;
	const udword NbPtsSmallCircle = 16;
	const udword NbSlices = 32;

	Point Pts[NbPtsSmallCircle];
	GeneratePolygon(NbPtsSmallCircle, Pts, sizeof(Point), ORIENTATION_XY, SmallRadius);

	udword TotalNbVerts = NbPtsSmallCircle * NbSlices;
	Point* Verts = ICE_NEW(Point)[TotalNbVerts];

	udword Index = 0;
	for(udword j=0;j<NbSlices;j++)
	{
		const float Coeff = float(j)/float(NbSlices);

		Matrix3x3 Rot;
		Rot.RotX(Coeff * TWOPI);

		const Point Trans = Rot[1]*BigRadius;
const Matrix3x3 TRot = torus_rot;
		for(udword i=0;i<NbPtsSmallCircle;i++)
//			Verts[Index++] = Trans + Pts[i]*Rot; 
			Verts[Index++] = (Trans + Pts[i]*Rot)*TRot;

	}
	ASSERT(Index==TotalNbVerts);

	PintObjectHandle Handles[NbSlices];
	Point Centers[NbSlices];
	udword GroupBit = 0;
	for(udword s=0;s<NbSlices;s++)
	{
		const udword SliceIndex0 = s;
		const udword SliceIndex1 = (s+1)%NbSlices;
		const Point* V0 = Verts + SliceIndex0*NbPtsSmallCircle;
		const Point* V1 = Verts + SliceIndex1*NbPtsSmallCircle;

		Point ConvexPts[NbPtsSmallCircle*2];
		for(udword i=0;i<NbPtsSmallCircle;i++)
		{
			ConvexPts[i] = V0[i];
			ConvexPts[i+NbPtsSmallCircle] = V1[i];
		}

		PINT_CONVEX_CREATE ConvexCreate;
		Point Center(0.0f, 0.0f, 0.0f);
		if(1)	// Recenter vertices
		{
			const float Coeff = 1.0f / float(NbPtsSmallCircle*2);
			for(udword i=0;i<NbPtsSmallCircle*2;i++)
				Center += ConvexPts[i] * Coeff;

			for(udword i=0;i<NbPtsSmallCircle*2;i++)
				ConvexPts[i] -= Center;
		}
		Centers[s] = Center;

		ConvexCreate.mNbVerts	= NbPtsSmallCircle*2;
		ConvexCreate.mVerts		= ConvexPts;
		ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexPts);

//		Handles[s] = CreateDynamicObject(pint, &ConvexCreate, pos+Center);
		{
			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &ConvexCreate;
			ObjectDesc.mMass			= mass;
			ObjectDesc.mPosition		= torus_pos+Center;
//			ObjectDesc.mRotation		= torus_rot;
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
			Handles[s] = CreatePintObject(pint, ObjectDesc);
		}
		ASSERT(Handles[s]);
	}
	DELETEARRAY(Verts);

	if(mass!=0.0f)
	{
		for(udword s=0;s<NbSlices;s++)
		{
			const udword Index0 = s;
			const udword Index1 = (s+1)%NbSlices;
			const Point Delta = (Centers[Index1] - Centers[Index0])*0.5f;

			PINT_FIXED_JOINT_CREATE Desc;
			Desc.mObject0		= Handles[Index0];
			Desc.mObject1		= Handles[Index1];
			Desc.mLocalPivot0	= Delta;
			Desc.mLocalPivot1	= -Delta;
	//		Desc.mLocalAxis0	= Point(1.0f, 0.0f, 0.0f);
	//		Desc.mLocalAxis1	= Point(1.0f, 0.0f, 0.0f);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
	}
}

static const char* gDesc_FixedJointsTorus = "A torus made of 32 convexes connected by fixed joints. Pick & drag objects to check each engine's stability. Play with solver iteration counts to improve the joints behavior.";

START_TEST(FixedJointsTorus, CATEGORY_BEHAVIOR, gDesc_FixedJointsTorus)

	virtual	void FixedJointsTorus::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.66f, 7.75f, 14.53f), Point(-0.71f, -0.18f, -0.68f));
	}

	virtual bool FixedJointsTorus::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportFixedJoints || !caps.mSupportCollisionGroups)
			return false;

		const PintDisabledGroups DG(1, 2);
		pint.SetDisabledGroups(1, &DG);

		const Point pos(0.0f, 10.0f, 0.0f);
		const Quat Q(1.0f, 0.0f, 0.0f, 0.0f);
		GenerateFixedJointsTorus(pint, pos, Q, 1.0f);
		return true;
	}

END_TEST(FixedJointsTorus)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJointsTorusStressTest = "A chain made of 10 parts, each part is a torus made of 32 convexes connected by fixed joints. Pick & drag objects to check each engine's stability. Play with solver iteration counts to improve the joints behavior.";

START_TEST(FixedJointsTorusStressTest, CATEGORY_BEHAVIOR, gDesc_FixedJointsTorusStressTest)

	virtual	void FixedJointsTorusStressTest::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(55.35f, 49.71f, 55.94f), Point(-0.66f, -0.01f, -0.75f));
	}

	virtual bool FixedJointsTorusStressTest::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportFixedJoints || !caps.mSupportCollisionGroups)
			return false;

		const PintDisabledGroups DG(1, 2);
		pint.SetDisabledGroups(1, &DG);

		Matrix3x3 Rot;	Rot.RotY(degToRad(90.0f));
		const Quat RQ = Rot;
		const udword NbTorus = 10;
		for(udword i=0;i<NbTorus;i++)
		{
			const Point pos(0.0f, 50.0f+float(i)*4.0f, 0.0f);
			const Quat Q(1.0f, 0.0f, 0.0f, 0.0f);
			GenerateFixedJointsTorus(pint, pos, i&1 ? Q : RQ, i==(NbTorus-1) ? 0.0f : 1.0f);
		}
		return true;
	}

END_TEST(FixedJointsTorusStressTest)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointChain = "Chains made of 20 spheres each, connected by spherical joints. Pick & drag objects to check each engine's stability.";

START_TEST(SphericalJointChain, CATEGORY_BEHAVIOR, gDesc_SphericalJointChain)

	virtual bool SphericalJointChain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphericalJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG(1, 2);
			pint.SetDisabledGroups(1, &DG);
		}

		const float Radius = 1.0f;
		const udword NbSpheres = 20;
		const udword NbRows = 20;
	//	const udword NbRows = 1;
		const Point Dir(1.0f, 0.0f, 0.0f);
		const Point Extents = Dir * Radius;
	//	const Point PosOffset = Dir * 0.1f;
		const Point PosOffset = Dir * 0.0f;

		PINT_SPHERE_CREATE SphereDesc;
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);
		SphereDesc.mRadius		= Radius;

		for(udword i=0;i<NbRows;i++)
		{
			PintObjectHandle Handles[NbSpheres];
			Point Positions[NbSpheres];

			Point Pos(0.0f, 40.0f, float(i)*Radius*4.0f);

			Positions[0] = Pos;
			udword GroupBit = 0;
	//		Handles[0] = CreateStaticObject(pint, &SphereDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &SphereDesc;
				ObjectDesc.mMass			= 0.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[0] = CreatePintObject(pint, ObjectDesc);
			}
			Pos += (PosOffset + Extents)*2.0f;

			for(udword i=1;i<NbSpheres;i++)
			{
				Positions[i] = Pos;
	//			Handles[i] = CreateDynamicObject(pint, &SphereDesc, Pos);
				{
					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes			= &SphereDesc;
					ObjectDesc.mMass			= 1.0f;
					ObjectDesc.mPosition		= Pos;
					ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
					Handles[i] = CreatePintObject(pint, ObjectDesc);
				}
				Pos += (PosOffset + Extents)*2.0f;
			}

			for(udword i=0;i<NbSpheres-1;i++)
			{
				PINT_SPHERICAL_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[i];
				Desc.mObject1		= Handles[i+1];
				Desc.mLocalPivot0	= Extents + PosOffset;
				Desc.mLocalPivot1	= -Extents - PosOffset;
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
		return true;
	}

END_TEST(SphericalJointChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HingeJointChain = "Chains made of 20 boxes each, connected by hinge joints. Pick & drag objects to check each engine's stability.";

START_TEST(HingeJointChain, CATEGORY_BEHAVIOR, gDesc_HingeJointChain)

	virtual bool HingeJointChain::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG(1, 2);
			pint.SetDisabledGroups(1, &DG);
		}

		const Point Extents(1.0f, 1.0f, 2.0f);
		const udword NbBoxes = 20;
		const udword NbRows = 20;
		const Point Dir(1.0f, 0.0f, 0.0f);
	//	const Point PosOffset = Dir*(Extents.x + 0.1f);
		const Point PosOffset = Dir*Extents.x;

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		for(udword i=0;i<NbRows;i++)
		{
			PintObjectHandle Handles[NbBoxes];
			Point Positions[NbBoxes];

			Point Pos(0.0f, 40.0f, float(i)*Extents.z*4.0f);
			Positions[0] = Pos;

			udword GroupBit = 0;
	//		Handles[0] = CreateStaticObject(pint, &BoxDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 0.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[0] = CreatePintObject(pint, ObjectDesc);
			}
			Pos += PosOffset*2.0f;

			for(udword i=1;i<NbBoxes;i++)
			{
				Positions[i] = Pos;
	//			Handles[i] = CreateDynamicObject(pint, &BoxDesc, Pos);
				{
					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes			= &BoxDesc;
					ObjectDesc.mMass			= 1.0f;
					ObjectDesc.mPosition		= Pos;
					ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
					Handles[i] = CreatePintObject(pint, ObjectDesc);
				}
				Pos += PosOffset*2.0f;
			}

			for(udword i=0;i<NbBoxes-1;i++)
			{
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[i];
				Desc.mObject1		= Handles[i+1];
				Desc.mLocalPivot0	= PosOffset;
				Desc.mLocalPivot1	= -PosOffset;
	//			Desc.mLocalAxis0	= Point(1.0f, 0.0f, 0.0f);
	//			Desc.mLocalAxis1	= Point(1.0f, 0.0f, 0.0f);
	//			Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
	//			Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
		return true;
	}

END_TEST(HingeJointChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BridgeUsingHinges = "Bridges made of 20 planks connected by hinge joints. Pick & drag objects to check each engine's stability.";

START_TEST(BridgeUsingHinges, CATEGORY_BEHAVIOR, gDesc_BridgeUsingHinges)

	virtual bool BridgeUsingHinges::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG(1, 2);
			pint.SetDisabledGroups(1, &DG);
		}

		const Point Extents(1.0f, 0.1f, 2.0f);
		const udword NbBoxes = 20;
		const udword NbRows = 20;
		const Point Dir(1.0f, 0.0f, 0.0f);
	//	const Point PosOffset = Dir*(Extents.x + 0.1f);
		const Point PosOffset = Dir*Extents.x;

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		for(udword i=0;i<NbRows;i++)
		{
			PintObjectHandle Handles[NbBoxes];
			Point Positions[NbBoxes];

			Point Pos(0.0f, 40.0f, float(i)*Extents.z*4.0f);
			Positions[0] = Pos;

			udword GroupBit = 0;
	//		Handles[0] = CreateStaticObject(pint, &BoxDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 0.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[0] = CreatePintObject(pint, ObjectDesc);
			}
			Pos += PosOffset*2.0f;

			for(udword i=1;i<NbBoxes-1;i++)
			{
				Positions[i] = Pos;
	//			Handles[i] = CreateDynamicObject(pint, &BoxDesc, Pos);
				{
					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes			= &BoxDesc;
					ObjectDesc.mMass			= 1.0f;
					ObjectDesc.mPosition		= Pos;
					ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
					Handles[i] = CreatePintObject(pint, ObjectDesc);
				}
				Pos += PosOffset*2.0f;
			}
			Positions[NbBoxes-1] = Pos;
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 0.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[NbBoxes-1] = CreatePintObject(pint, ObjectDesc);
			}
			Pos += PosOffset*2.0f;

			for(udword i=0;i<NbBoxes-1;i++)
			{
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[i];
				Desc.mObject1		= Handles[i+1];
				Desc.mLocalPivot0	= PosOffset;
				Desc.mLocalPivot1	= -PosOffset;
	//			Desc.mLocalAxis0	= Point(1.0f, 0.0f, 0.0f);
	//			Desc.mLocalAxis1	= Point(1.0f, 0.0f, 0.0f);
	//			Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
	//			Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
		return true;
	}

END_TEST(BridgeUsingHinges)

///////////////////////////////////////////////////////////////////////////////
