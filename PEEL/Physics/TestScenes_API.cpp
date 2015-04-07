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

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CollisionGroups = "Simple filtering test, replicating the old collision groups from the NovodeX SDK. If the test works, the falling spheres should not collide with the spheres below them.";

START_TEST(CollisionGroups, CATEGORY_API, gDesc_CollisionGroups)

	virtual	void CollisionGroups::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.05f, 6.34f, 10.78f), Point(-0.67f, -0.32f, -0.67f));
	}

	virtual bool CollisionGroups::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
			return false;

		// Group 0 is already used by the default static environment so we start with group 1
		const PintCollisionGroup Layer0_Group = 1;
		const PintCollisionGroup Layer1_Group = 2;

		const PintDisabledGroups DG(Layer0_Group, Layer1_Group);
		pint.SetDisabledGroups(1, &DG);

		const float Radius = 1.0f;
		const udword NbX = 4;
		const udword NbY = 4;
		GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Radius, 4.0f, 4.0f, 1.0f, Layer0_Group);
		GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Radius*4.0f, 4.0f, 4.0f, 1.0f, Layer1_Group);
		return true;
	}

END_TEST(CollisionGroups)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJoint = "Simple test scene for spherical joints.";

START_TEST(SphericalJoint, CATEGORY_API, gDesc_SphericalJoint)

	virtual	void SphericalJoint::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(5.39f, 24.70f, 10.00f), Point(-0.41f, -0.31f, -0.86f));
	}

	virtual bool SphericalJoint::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportSphericalJoints)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point DynamicPos = StaticPos - Extents*2.0f;

		PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

		PINT_SPHERICAL_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mLocalPivot0	= -Extents;
		Desc.mLocalPivot1	= Extents;
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(SphericalJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HingeJoint = "Simple test scene for hinge joints.";

START_TEST(HingeJoint, CATEGORY_API, gDesc_HingeJoint)

	virtual	void HingeJoint::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.59f, 22.68f, 9.94f), Point(-0.69f, -0.25f, -0.68f));
	}

	virtual bool HingeJoint::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportHingeJoints)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point Disp(BoxSize*2.0f, -BoxSize*2.0f, 0.0f);
		const Point DynamicPos = StaticPos + Disp;

		PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

		PINT_HINGE_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mLocalPivot0	= Disp*0.5f;
		Desc.mLocalPivot1	= -Disp*0.5f;
		Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
		Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(HingeJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LimitedHingeJoint = "Simple test scene for hinge joints with limits.";

START_TEST(LimitedHingeJoint, CATEGORY_API, gDesc_LimitedHingeJoint)

	virtual	void LimitedHingeJoint::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(20.51f, 23.75f, 17.54f), Point(-0.50f, -0.18f, -0.85f));
	}

	virtual bool LimitedHingeJoint::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		for(udword i=0;i<3;i++)
		{
			const Point StaticPos(float(i)*BoxSize*8.0f, 20.0f, 0.0f);
			const Point Disp(BoxSize*2.0f, -BoxSize*2.0f, 0.0f);
			const Point DynamicPos = StaticPos + Disp;

			PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
			PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

			PINT_HINGE_JOINT_CREATE Desc;
			Desc.mObject0		= StaticObject;
			Desc.mObject1		= DynamicObject;
			Desc.mLocalPivot0	= Disp*0.5f;
			Desc.mLocalPivot1	= -Disp*0.5f;
			if(i==0)
			{
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
			}
			else if(i==1)
			{
				Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
			}
			else if(i==2)
			{
				Desc.mLocalAxis0	= Point(1.0f, 0.0f, 0.0f);
				Desc.mLocalAxis1	= Point(1.0f, 0.0f, 0.0f);
			}
			Desc.mMinLimitAngle	= degToRad(-45.0f);
		//	Desc.mMinLimitAngle	= 0.0f;
		//	Desc.mMaxLimitAngle	= 0.0f;
			Desc.mMaxLimitAngle	= degToRad(45.0f);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
		return true;
	}

END_TEST(LimitedHingeJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LimitedHingeJoint2 = "Simple test scene for hinge joints with limits.";

START_TEST(LimitedHingeJoint2, CATEGORY_API, gDesc_LimitedHingeJoint2)

	virtual	void LimitedHingeJoint2::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.96f, 16.77f, 12.66f), Point(-0.67f, -0.29f, -0.69f));
	}

	virtual bool LimitedHingeJoint2::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point Disp(BoxSize*2.0f, -BoxSize*2.0f, 0.0f);
		const Point DynamicPos = StaticPos + Disp;

		PintObjectHandle StaticObject = CreateDynamicObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

		PINT_HINGE_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mLocalPivot0	= Disp*0.5f;
		Desc.mLocalPivot1	= -Disp*0.5f;
		Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
		Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
		Desc.mMinLimitAngle	= degToRad(-45.0f);
	//	Desc.mMinLimitAngle	= 0.0f;
	//	Desc.mMaxLimitAngle	= 0.0f;
		Desc.mMaxLimitAngle	= degToRad(45.0f);
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(LimitedHingeJoint2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PrismaticJoint = "Simple test scene for prismatic joints.";

START_TEST(PrismaticJoint, CATEGORY_API, gDesc_PrismaticJoint)

	virtual	void PrismaticJoint::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-4.63f, 27.73f, 14.05f), Point(0.66f, -0.36f, -0.66f));
	}

	virtual bool PrismaticJoint::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportPrismaticJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point Disp(BoxSize*2.0f, 0.0f, 0.0f);
		const Point DynamicPos = StaticPos + Disp;
		const Point LinVel(10.0f, 0.0f, 0.0f);

		PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos, null, &LinVel);

		PINT_PRISMATIC_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mLocalPivot0	= Disp*0.5f;
		Desc.mLocalPivot1	= -Disp*0.5f;
		Desc.mLocalAxis0	= Point(1.0f, 0.0f, 0.0f);
		Desc.mLocalAxis1	= Point(1.0f, 0.0f, 0.0f);
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(PrismaticJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJoint = "Simple test scene for fixed joints.";

START_TEST(FixedJoint, CATEGORY_API, gDesc_FixedJoint)

	virtual	void FixedJoint::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(7.10f, 22.91f, -10.82f), Point(-0.63f, -0.29f, 0.72f));
	}

	virtual bool FixedJoint::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportFixedJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point DynamicPos = StaticPos - Extents*2.0f;

		PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

		PINT_FIXED_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mLocalPivot0	= -Extents;
		Desc.mLocalPivot1	= Extents;
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(FixedJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HingeJoint_CheckCollisionBetweenJointed = "Test to check if collision is enabled or disabled by default, between objects \
connected by a joint. All engines do not have the same default behavior here.";

START_TEST(HingeJoint_CheckCollisionBetweenJointed, CATEGORY_API, gDesc_HingeJoint_CheckCollisionBetweenJointed)

	virtual	void HingeJoint_CheckCollisionBetweenJointed::GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(8.17f, 22.24f, 8.67f), Point(-0.57f, -0.33f, -0.75f));
	}

	virtual bool HingeJoint_CheckCollisionBetweenJointed::Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
		BoxDesc.mExtents	= Extents;

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point Disp(BoxSize*2.0f, 0.0f, 0.0f);
		const Point DynamicPos0 = StaticPos + Disp;
		const Point DynamicPos1 = DynamicPos0 + Disp;

		PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		PintObjectHandle DynamicObject0 = CreateDynamicObject(pint, &BoxDesc, DynamicPos0);
		PintObjectHandle DynamicObject1 = CreateDynamicObject(pint, &BoxDesc, DynamicPos1);

		PINT_HINGE_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject0;
		Desc.mLocalPivot0	= Disp*0.5f;
		Desc.mLocalPivot1	= -Disp*0.5f;
		Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
		Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
		PintJointHandle JointHandle0 = pint.CreateJoint(Desc);
		ASSERT(JointHandle0);

		Desc.mObject0		= DynamicObject0;
		Desc.mObject1		= DynamicObject1;
		PintJointHandle JointHandle1 = pint.CreateJoint(Desc);
		ASSERT(JointHandle1);
		return true;
	}

END_TEST(HingeJoint_CheckCollisionBetweenJointed)

///////////////////////////////////////////////////////////////////////////////
