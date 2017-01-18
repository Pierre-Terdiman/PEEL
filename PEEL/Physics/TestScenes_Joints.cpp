///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Render.h"
#include "Random.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "Loader_Bin.h"
#include "GUI_Helpers.h"
#include "Cylinder.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJoint = "Simple test scene for spherical joints. Some engines will automatically disable collisions between the two cubes. Some will not.";

START_TEST(SphericalJoint, CATEGORY_JOINTS, gDesc_SphericalJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(0.10f, 14.65f, 8.24f), Point(-0.09f, 0.39f, -0.92f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportSphericalJoints)
			return false;

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point DynamicPos = StaticPos - Extents*2.0f;

		const PintObjectHandle StaticObject = CreateStaticObject(pint, &BoxDesc, StaticPos);
		const PintObjectHandle DynamicObject = CreateDynamicObject(pint, &BoxDesc, DynamicPos);

		PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(StaticObject, DynamicObject, -Extents, Extents));
		ASSERT(JointHandle);
		return true;
	}

END_TEST(SphericalJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HingeJoint = "Simple test scene for hinge joints. Some engines will automatically disable collisions between the two cubes. Some will not.";

START_TEST(HingeJoint, CATEGORY_JOINTS, gDesc_HingeJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(5.93f, 20.16f, 5.70f), Point(-0.67f, -0.28f, -0.68f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportHingeJoints)
			return false;

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

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

//		pint.mUserData = (void*)DynamicObject;
		return true;
	}

	virtual	udword		Update(Pint& pint, float dt)
	{
//		pint.AddLocalTorque(PintObjectHandle(pint.mUserData), Point(0.0f, 0.0f, 1000.0f));
//		pint.SetLocalRot(PintObjectHandle(pint.mUserData),Quat(1.0f, 0.0f, 0.0f, 0.0f));

		return 0;
	}

END_TEST(HingeJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LimitedHingeJoint = "Simple test scene for hinge joints with limits. Filtering is used to disable collisions between jointed objects.";

START_TEST(LimitedHingeJoint, CATEGORY_JOINTS, gDesc_LimitedHingeJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(19.00f, 22.95f, 13.39f), Point(-0.48f, -0.16f, -0.86f));
		desc.mCamera[1] = CameraPose(Point(3.57f, 19.59f, 5.08f), Point(-0.48f, -0.15f, -0.86f));
		desc.mCamera[2] = CameraPose(Point(9.60f, 23.57f, 3.19f), Point(-0.14f, -0.79f, -0.59f));
		desc.mCamera[3] = CameraPose(Point(17.59f, 21.81f, 4.68f), Point(-0.13f, -0.50f, -0.85f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes	= &BoxDesc;

		udword GroupBit = 0;
//		udword i=0;
		for(udword i=0;i<3;i++)
		{
			const Point StaticPos(float(i)*BoxSize*8.0f, 20.0f, 0.0f);
//			const Point StaticPos(float(i)*BoxSize*8.0f, 2.0f, 0.0f);
			const Point Disp(BoxSize*2.0f, -BoxSize*2.0f, 0.0f);
			const Point DynamicPos = StaticPos + Disp;

			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition		= StaticPos;
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
			PintObjectHandle StaticObject = CreatePintObject(pint, ObjectDesc);

			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= DynamicPos;
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
			PintObjectHandle DynamicObject = CreatePintObject(pint, ObjectDesc);

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

//				Desc.mMaxLimitAngle	= 0.0f;
//				Desc.mMinLimitAngle	= -PI/4.0f;

			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
		return true;
	}

END_TEST(LimitedHingeJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_LimitedHingeJoint2 = "Simple test scene for hinge joints with limits, between two dynamic objects.";

START_TEST(LimitedHingeJoint2, CATEGORY_JOINTS, gDesc_LimitedHingeJoint2)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(3.62f, 4.58f, 8.59f), Point(-0.31f, -0.27f, -0.91f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		const Point StaticPos(0.0f, 4.0f, 0.0f);
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

START_TEST(PrismaticJoint, CATEGORY_JOINTS, gDesc_PrismaticJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-4.63f, 27.73f, 14.05f), Point(0.66f, -0.36f, -0.66f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportPrismaticJoints || !caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

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

static const char* gDesc_LimitedPrismaticJoint = "Simple test scene for prismatic joints with limits.";

START_TEST(LimitedPrismaticJoint, CATEGORY_JOINTS, gDesc_LimitedPrismaticJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-2.55f, 5.55f, 10.57f), Point(0.29f, -0.27f, -0.92f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportPrismaticJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		const Point DynamicPos0(0.0f, BoxSize, 0.0f);
		const Point Disp(0.0f, BoxSize*2.0f, 0.0f);
		const Point DynamicPos1 = DynamicPos0 + Disp;

		PintObjectHandle DynamicObject0 = CreateDynamicObject(pint, &BoxDesc, DynamicPos0);
		PintObjectHandle DynamicObject1 = CreateDynamicObject(pint, &BoxDesc, DynamicPos1);

		PINT_PRISMATIC_JOINT_CREATE Desc;
		Desc.mObject0		= DynamicObject0;
		Desc.mObject1		= DynamicObject1;
		Desc.mLocalPivot0	= Disp*0.5f;
		Desc.mLocalPivot1	= -Disp*0.5f;
		Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
		Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
		Desc.mMaxLimit		= BoxSize*2.0f;
		Desc.mMinLimit		= 0.0f;
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(LimitedPrismaticJoint)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PrismaticSpring = "Simple test scene for prismatic springs.";

START_TEST(PrismaticSpring, CATEGORY_JOINTS, gDesc_PrismaticSpring)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-2.55f, 5.55f, 10.57f), Point(0.29f, -0.27f, -0.92f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportPrismaticJoints || !caps.mSupportRigidBodySimulation)
			return false;

		const Point Extents(4.0f, 2.0f, 4.0f);

		PintObjectHandle Chassis;
		const Point ChassisPos(0.0f, Extents.y*4.0f, 0.0f);

		{
			PINT_BOX_CREATE BoxDesc(Extents);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
//			Chassis = CreateDynamicObject(pint, &BoxDesc, ChassisPos);
			Chassis = CreateSimpleObject(pint, &BoxDesc, 10.0f, ChassisPos);
		}

		const Point ChildExtents(1.0f, 1.0f, 1.0f);
		for(udword j=0;j<2;j++)
		{
			const float z = j ? 1.0f : -1.0f;
			for(udword i=0;i<2;i++)
			{
				const float x = i ? 1.0f : -1.0f;

				const Point Offset(x*(Extents.x-ChildExtents.x), -Extents.y-ChildExtents.y, z*(Extents.z-ChildExtents.z));
				const Point ChildPos = ChassisPos + Offset;

				PINT_BOX_CREATE BoxDesc(ChildExtents);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
				PintObjectHandle Child = CreateDynamicObject(pint, &BoxDesc, ChildPos);

				PINT_PRISMATIC_JOINT_CREATE Desc;
				Desc.mObject0			= Chassis;
				Desc.mObject1			= Child;
				Desc.mLocalPivot0		= Offset;
				Desc.mLocalPivot1		= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalAxis0		= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1		= Point(0.0f, 1.0f, 0.0f);
				Desc.mMaxLimit			= 0.01f;
				Desc.mMinLimit			= 0.0f;
				Desc.mSpringStiffness	= 100.0f;
				Desc.mSpringDamping		= 10.0f;
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
		return true;
	}

END_TEST(PrismaticSpring)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJoint = "Simple test scene for fixed joints.";

START_TEST(FixedJoint, CATEGORY_JOINTS, gDesc_FixedJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(7.10f, 22.91f, -10.82f), Point(-0.63f, -0.29f, 0.72f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportFixedJoints || !caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

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

START_TEST(HingeJoint_CheckCollisionBetweenJointed, CATEGORY_JOINTS, gDesc_HingeJoint_CheckCollisionBetweenJointed)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(8.17f, 22.24f, 8.67f), Point(-0.57f, -0.33f, -0.75f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const float BoxSize = 1.0f;
		const Point Extents(BoxSize, BoxSize, BoxSize);

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

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

static const char* gDesc_DistanceJoint = "Simple test scene for distance joints.";

START_TEST(DistanceJoint, CATEGORY_JOINTS, gDesc_DistanceJoint)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-0.61f, 20.57f, 6.71f), Point(0.12f, -0.20f, -0.97f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportDistanceJoints || !caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const float Radius = 1.0f;
		PINT_SPHERE_CREATE SphereDesc(Radius);
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

		const Point StaticPos(0.0f, 20.0f, 0.0f);
		const Point DynamicPos = StaticPos + Point(Radius*2.0f, 0.0f, 0.0f);

		PintObjectHandle StaticObject = CreateStaticObject(pint, &SphereDesc, StaticPos);
//		PintObjectHandle StaticObject = CreateDynamicObject(pint, &SphereDesc, StaticPos);
//		SphereDesc.mRadius		= Radius/2.0f;
		PintObjectHandle DynamicObject = CreateDynamicObject(pint, &SphereDesc, DynamicPos);

		PINT_DISTANCE_JOINT_CREATE Desc;
		Desc.mObject0		= StaticObject;
		Desc.mObject1		= DynamicObject;
		Desc.mMaxDistance	= 2.0f * Radius;
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
		return true;
	}

END_TEST(DistanceJoint)

///////////////////////////////////////////////////////////////////////////////

static void GenerateFixedJointsTorus(Pint& pint, const Point& torus_pos, const Quat& torus_rot, float mass, udword nb_loops)
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

		PINT_CONVEX_CREATE ConvexCreate(NbPtsSmallCircle*2, ConvexPts);
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
		for(udword i=0;i<nb_loops;i++)
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
	//			for(udword i=0;i<8;i++)
				{
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
				}
			}
		}
	}
}

static const char* gDesc_FixedJointsTorus = "A torus made of 32 convexes connected by fixed joints. You can pick & drag the torus to check each engine's stability. \
Ideally the torus should behave like a rigid body, but iterative solvers make it look like a soft body. Increasing the number of solver iterations typically makes it look \
more rigid - the more iterations the closer to the ideal. Different engines give a different softness by default, and have a different cost for each additional solver iteration.";

START_TEST(FixedJointsTorus, CATEGORY_JOINTS, gDesc_FixedJointsTorus)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.66f, 7.75f, 14.53f), Point(-0.71f, -0.18f, -0.68f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportFixedJoints || !caps.mSupportCollisionGroups)
			return false;

		const PintDisabledGroups DG(1, 2);
		pint.SetDisabledGroups(1, &DG);

		const Point pos(0.0f, 10.0f, 0.0f);
		const Quat Q(1.0f, 0.0f, 0.0f, 0.0f);
		GenerateFixedJointsTorus(pint, pos, Q, 1.0f, 1);
		return true;
	}

END_TEST(FixedJointsTorus)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJointsTorusMultipleConstraints = "This scene demonstrates an alternative way to improve the joints behavior. This is the same torus as in the \
previous test. Instead of increasing the number of solver iterations, we create the same fixed joints constraints multiple times (4 in this case). This can have the same \
effect overall (increasing the torus' rigidity) but with a more controllable cost. The cost here is localized to the jointed object, while the cost of increasing the number \
of solver iterations can spread to a full simulation island in some engines. On the other hand this uses more memory than adding extra solver iterations. Note that this trick, \
which works fine for PhysX 3, does not work well in all engines.";

START_TEST(FixedJointsTorusMultipleConstraints, CATEGORY_JOINTS, gDesc_FixedJointsTorusMultipleConstraints)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(15.66f, 7.75f, 14.53f), Point(-0.71f, -0.18f, -0.68f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportFixedJoints || !caps.mSupportCollisionGroups)
			return false;

		const PintDisabledGroups DG(1, 2);
		pint.SetDisabledGroups(1, &DG);

		const Point pos(0.0f, 10.0f, 0.0f);
		const Quat Q(1.0f, 0.0f, 0.0f, 0.0f);
		GenerateFixedJointsTorus(pint, pos, Q, 1.0f, 4);
		return true;
	}

END_TEST(FixedJointsTorusMultipleConstraints)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJointsTorusStressTest = "A chain made of 10 parts, each part is a torus made of 32 convexes connected by fixed joints (like in previous test). \
Pick & drag objects to check each engine's stability. Play with solver iteration counts to improve the joints behavior.";

START_TEST(FixedJointsTorusStressTest, CATEGORY_JOINTS, gDesc_FixedJointsTorusStressTest)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(55.35f, 49.71f, 55.94f), Point(-0.66f, -0.01f, -0.75f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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
			GenerateFixedJointsTorus(pint, pos, i&1 ? Q : RQ, i==(NbTorus-1) ? 0.0f : 1.0f, 1);
		}
		return true;
	}

END_TEST(FixedJointsTorusStressTest)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_FixedJointsTorusStressTestMultipleConstraints = "Same scene as before but we create the same constraints 8 times (see FixedJointsTorusMultipleConstraints). \
Note that you can combine both things: increase the number of solver iterations AND create constraints multiple times.";

START_TEST(FixedJointsTorusStressTestMultipleConstraints, CATEGORY_JOINTS, gDesc_FixedJointsTorusStressTestMultipleConstraints)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(55.35f, 49.71f, 55.94f), Point(-0.66f, -0.01f, -0.75f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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
			GenerateFixedJointsTorus(pint, pos, i&1 ? Q : RQ, i==(NbTorus-1) ? 0.0f : 1.0f, 8);
		}
		return true;
	}

END_TEST(FixedJointsTorusStressTestMultipleConstraints)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointChain = "Chains made of 20 spheres each, connected by spherical joints. Pick & drag objects to check each engine's stability.";

START_TEST(SphericalJointChain, CATEGORY_JOINTS, gDesc_SphericalJointChain)

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_SPHERE_CREATE SphereDesc(Radius);
		SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

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
				const Point Offset = Extents + PosOffset;				
				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i], Handles[i+1], Offset, -Offset));
				ASSERT(JointHandle);
			}
		}
		return true;
	}

END_TEST(SphericalJointChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_HingeJointChain = "Chains made of 20 boxes each, connected by hinge joints. Pick & drag objects to check each engine's stability.";

START_TEST(HingeJointChain, CATEGORY_JOINTS, gDesc_HingeJointChain)

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

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

START_TEST(BridgeUsingHinges, CATEGORY_JOINTS, gDesc_BridgeUsingHinges)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(42.34f, 49.89f, 132.99f), Point(-0.72f, -0.50f, -0.48f));
		desc.mCamera[1] = CameraPose(Point(18.33f, 41.38f, 155.88f), Point(-0.03f, -0.48f, -0.88f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);

		for(udword j=0;j<NbRows;j++)
		{
			PintObjectHandle Handles[NbBoxes];
			Point Positions[NbBoxes];

			Point Pos(0.0f, 40.0f, float(j)*Extents.z*4.0f);
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
				// you must allow a wiggle room otherwise the bridge the mass matrix is singular leading to a explosion of engine with force base solvers
//				Pos += (PosOffset - Point (Extents.x * 0.01f, 0.0f, 0.0f)) *2.0f;
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
				if(1)
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
				else
				{
					PINT_FIXED_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+1];
					Desc.mLocalPivot0	= PosOffset;
					Desc.mLocalPivot1	= -PosOffset;
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}
		return true;
	}

END_TEST(BridgeUsingHinges)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CatenaryBridge = "Catenary bridges.";

START_TEST(CatenaryBridge, CATEGORY_JOINTS, gDesc_CatenaryBridge)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.43f, 40.32f, 95.38f), Point(0.06f, -0.04f, -1.00f));
		desc.mCamera[1] = CameraPose(Point(0.77f, 56.76f, -20.68f), Point(0.45f, -0.36f, 0.82f));
//		desc.mCamera[0] = CameraPose(Point(-0.04f, 49.89f, 26.35f), Point(0.46f, -0.22f, -0.86f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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
		const udword NbRows = 10;
		const Point Dir(1.0f, 0.0f, 0.0f);
	//	const Point PosOffset = Dir*(Extents.x + 0.1f);
		const Point PosOffset = Dir*Extents.x;

		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
//		BoxDesc.mExtents	= Point(2.0f, 0.1f, 2.0f);

		float Chainette[NbBoxes+1];
		float Dy[NbBoxes+1];
		const float Coeff = 20.0f;
		for(udword i=0;i<NbBoxes+1;i++)
		{
			const float f0 = float(i) - float(NbBoxes/2);
			const float f1 = float(i+1) - float(NbBoxes/2);

			Chainette[i] = Coeff*coshf(f0/Coeff);

			Dy[i] = Coeff*(coshf(f1/Coeff) - coshf(f0/Coeff));
		}

		for(udword j=0;j<NbRows;j++)
		{
			const float RowCoeff = float(j)/float(NbRows-1);
			PintObjectHandle Handles[NbBoxes];

			Point Pos(0.0f, 40.0f, float(j)*Extents.z*4.0f);

			if(1)
			{
				const float Radius = 1.0f;

				PINT_SPHERE_CREATE SphereDesc(Radius);
				SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes		= &SphereDesc;
				ObjectDesc.mMass		= 10.0f;
				ObjectDesc.mPosition	= Pos;
				ObjectDesc.mPosition.y	+= Radius*3.0f;
//				ObjectDesc.mPosition	+= Point(12.0f, 10.0f, 0.0f);
				CreatePintObject(pint, ObjectDesc);
			}

			Point RightPos(0.0f, 0.0f, 0.0f);
			udword GroupBit = 0;
			for(udword i=0;i<NbBoxes;i++)
			{
				float Mass = 1.0f;
				if(i==0 || i==NbBoxes-1)
					Mass = 0.0f;

				Matrix3x3 Rot;
				const float Alpha = atanf(Dy[i]);
				const float CurveCoeff = 0.5f + RowCoeff * 2.5f;
				Rot.RotZ(Alpha*CurveCoeff);

				Point R;
				Rot.GetRow(0, R);

				{
					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes			= &BoxDesc;
					ObjectDesc.mMass			= Mass;
					ObjectDesc.mPosition		= Pos;
//					ObjectDesc.mPosition.y		= (Chainette[i] + Chainette[i+1])*0.5f;
//					ObjectDesc.mPosition.y		+= Chainette[i] - 10.0f;

					ObjectDesc.mRotation = Rot;

					const Point LeftPos = Pos - R*Extents.x;
					if(i)
					{
						Pos += RightPos - LeftPos;
						//ObjectDesc.mPosition += RightPos - LeftPos;
						ObjectDesc.mPosition		= Pos;
					}

					ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
					Handles[i] = CreatePintObject(pint, ObjectDesc);
				}

				RightPos = Pos + R*Extents.x;

//				Pos += PosOffset*2.0f;
				Pos += R*2.0f*Extents.x;
			}

			for(udword i=0;i<NbBoxes-1;i++)
			{
				if(1)
				{
					PINT_HINGE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+1];
					Desc.mLocalPivot0	= PosOffset;
					Desc.mLocalPivot1	= -PosOffset;
					Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
					Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
				else
				{
					PINT_FIXED_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+1];
					Desc.mLocalPivot0	= PosOffset;
					Desc.mLocalPivot1	= -PosOffset;
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}
		return true;
	}

END_TEST(CatenaryBridge)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointNet = "Stress test for spherical joints. The net is made of 40*40 spheres, all connected by spherical joints.";

START_TEST(SphericalJointNet, CATEGORY_JOINTS, gDesc_SphericalJointNet)

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-50.0f, -50.0f, -50.0f), Point(50.0f, 50.0f, 50.0f));
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.47f, -0.66f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphericalJoints || !caps.mSupportRigidBodySimulation)
			return false;

		{
			PINT_SPHERE_CREATE ShapeDesc(16.0f);
			ShapeDesc.mRenderer	= CreateSphereRenderer(16.0f);

			CreateStaticObject(pint, &ShapeDesc, Point(0.0f, 10.0f, 0.0f));
		}

		const udword NbX = 40;
		const udword NbY = 40;
		const float Scale = 40.0f;
		const float Altitude = 30.0f;

		PINT_SPHERE_CREATE ShapeDesc(1.0f);
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
//				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[Base+x], Handles[Base+x+1], Point(CenterX, 0.0f, 0.0f), Point(-CenterX, 0.0f, 0.0f)));
				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[Base+x], Handles[Base+x+1], Point(CenterX*2.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
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
//				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[x+(y*NbX)], Handles[x+(y+1)*NbX], Point(0.0f, 0.0f, CenterY), Point(0.0f, 0.0f, -CenterY)));
				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[x+(y*NbX)], Handles[x+(y+1)*NbX], Point(0.0f, 0.0f, CenterY*2.0f), Point(0.0f, 0.0f, 0.0f)));
			}
		}

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(SphericalJointNet)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphericalJointNet2 = "Stress test for spherical joints. The net is made of 40*40 spheres, all connected by spherical joints.";

START_TEST(SphericalJointNet2, CATEGORY_JOINTS, gDesc_SphericalJointNet2)

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mGlobalBounds.SetMinMax(Point(-50.0f, -50.0f, -50.0f), Point(50.0f, 50.0f, 50.0f));
		desc.mCamera[0] = CameraPose(Point(60.07f, 35.87f, 60.52f), Point(-0.64f, -0.40f, -0.65f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphericalJoints || !caps.mSupportRigidBodySimulation)
			return false;

		if(0)
		{
			PINT_SPHERE_CREATE ShapeDesc(16.0f);
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
			float yy = 10.0f;
			BasicRandom Rnd(42);

			PINT_SPHERE_CREATE SphereDesc(SphereRadius);
			SphereDesc.mRenderer = CreateSphereRenderer(SphereRadius);

			PINT_BOX_CREATE BoxDesc(CapsuleRadius, CapsuleRadius, CapsuleRadius);
			BoxDesc.mRenderer = CreateBoxRenderer(BoxDesc.mExtents);

			PINT_CAPSULE_CREATE CapsuleDesc(CapsuleRadius, HalfHeight);
			CapsuleDesc.mRenderer = CreateCapsuleRenderer(CapsuleRadius, HalfHeight*2.0f);

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
				yy += HalfHeight*5.0f;
			}
		}


		const udword NbX = 40;
		const udword NbY = 40;
		const float Scale = 40.0f;
		const float Altitude = 0.0f;

		PINT_SPHERE_CREATE ShapeDesc(1.0f);
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
//				ObjectDesc.mMassForInertia	= 10.0f;
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
//				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[Base+x], Handles[Base+x+1], Point(CenterX, 0.0f, 0.0f), Point(-CenterX, 0.0f, 0.0f)));
				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[Base+x], Handles[Base+x+1], Point(CenterX*2.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
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
//				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[x+(y*NbX)], Handles[x+(y+1)*NbX], Point(0.0f, 0.0f, CenterY), Point(0.0f, 0.0f, -CenterY)));
				pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[x+(y*NbX)], Handles[x+(y+1)*NbX], Point(0.0f, 0.0f, CenterY*2.0f), Point(0.0f, 0.0f, 0.0f)));
			}
		}

		mCreateDefaultEnvironment = false;

		return true;
	}

END_TEST(SphericalJointNet2)

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
	PintObjectHandle	mBody;
	PR					mPose;
};

class TestRagdoll
{
public:
	enum { NUM_BONES = 19 };

	TestRagdoll();
	~TestRagdoll();

	bool				Init(Pint& pint, const Point& offset, PintCollisionGroup group, bool use_compound, udword constraint_multiplier);

	Bone				mBones[NUM_BONES];
	PintJointHandle*	mJoints[NUM_BONES-1];

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

static udword LocalVectors_Data[]={
0x3d13b29c, 
0xb9e269c0, 0x39752680, 0xbd90392d, 0xbb18aa20, 0x3b023760, 0xb4a80000, 0xb5300000, 0x3f7fffff, 0xbe2115c1, 0x3f354685, 0x3f303843, 0x3e03b7f2, 0xbc99df24, 0x3b37ad60, 0xbd6fc276, 0xb9e22a00, 
0x39760a80, 0xb42c0000, 0xb5500000, 0x3f7ffffe, 0x393c6800, 0xb88b3000, 0x3f800000, 0x3d9515f5, 0xbc3cd7a0, 0x3aa8b690, 0xbd606487, 0xbc994d22, 0x3b379740, 0x33500000, 0xb5080000, 0x3f800000, 
0x3dac2459, 0x3dc8d6f8, 0x3f7ddb0e, 0x3ded30f6, 0x3cd7d820, 0xb5ca8000, 0xbd6a05b2, 0xbc3a6fe4, 0x3aa8cf80, 0x34100000, 0xb4c00000, 0x3f7fffff, 0x3da307f7, 0xbd760d18, 0x3f7eb94b, 0xbe23122b, 
0x3b6ac3ec, 0x3bc07081, 0xbd52ee09, 0x3a7e28c0, 0x3d9e7cea, 0x35100000, 0xb4b80000, 0x3f7ffffc, 0xbdb5d9e2, 0x3e8c088e, 0xbf753001, 0xbe3f67ef, 0x3b24fd48, 0x3c0c891c, 0x3e6def63, 0x3b6ac1f8, 
0x3bc06aee, 0x36270000, 0x34000000, 0x3f7ffffd, 0x34000000, 0xb5600000, 0x3f7ffffd, 0xbd67d0cb, 0xbd837370, 0x3c41c128, 0x3e4d02c2, 0x3b24e630, 0x3c0c8410, 0x36560000, 0x35d40000, 0x3f7ffffe, 
0xbe5a56ea, 0x3d883a24, 0x3f798856, 0xbe231251, 0x3b6ad032, 0xbbb96bfc, 0xbd52edd6, 0x3a7e1a00, 0xbd9e7ede, 0x34728000, 0x33000000, 0x3f7ffffc, 0x3ec72db4, 0xbe9e78f6, 0xbf5e2008, 0xbe3f6816, 
0x3b24d7cb, 0xbbc910b0, 0x3e6def24, 0x3b6acda8, 0xbbb96fcc, 0x35180000, 0xb40c0000, 0x3f7ffffd, 0x34910000, 0xb4a00000, 0x3f7ffffc, 0xbd67d067, 0xbd83743e, 0xbc320030, 0x3e4d02b4, 0x3b24dc7e, 
0xbbc90e40, 0x34f00000, 0x355c0000, 0x3f7fffff, 0x3cb75a20, 0xbd7ddb24, 0x3f7f718f, 0xbd90b19f, 0x3b00de90, 0x3cb87512, 0x3df65e49, 0xbcc81a6c, 0x3d8a3c19, 0xb5a20000, 0x35a80000, 0x3f7ffffe, 
0x3f6fb7d9, 0xbd009a5c, 0x3eb2f584, 0xbe0cd413, 0xbb1f25e0, 0x3a767780, 0x3d2073c5, 0x3b00e936, 0x3cb874fd, 0xb5e40000, 0x34d00000, 0x3f7fffff, 0x3f063b90, 0x3ec6ecec, 0x3f41f893, 0xbdb27fb1, 
0xbc51a90f, 0xbb1a6ba0, 0x3de224b3, 0xbb1f2480, 0x3a767480, 0xb5f80000, 0xb5980000, 0x3f7ffffc, 0xb6160000, 0x34f80000, 0x3f7ffffe, 0xbd730f3d, 0xbc36e99f, 0x3c30c3e1, 0x3de05d99, 0xbc51a821, 
0xbb1a6460, 0xb59c0000, 0x35b40000, 0x3f800001, 0x3e79f4da, 0x3f783d72, 0x3c34def0, 0xbd99e60f, 0x3b1d1530, 0xbca81be7, 0x3df66b16, 0xbcc81a50, 0xbd7d86e8, 0xb5d20000, 0xb65a0000, 0x3f7fffff, 
0xbf7097b6, 0x3cfa6f6c, 0x3eae3d85, 0xbe0cd812, 0xbb1f3f00, 0x3b21b1c0, 0x3d0e0ad5, 0x3b1d11dc, 0xbca81b81, 0xb56e0000, 0x35f80000, 0x3f800000, 0xbe9f9ad3, 0x3ef52cda, 0x3f521848, 0xbdb2864b, 
0xbc51a470, 0x3b9e83c0, 0x3de21c74, 0xbb1f3ab8, 0x3b21b240, 0xb6100000, 0xb5da0000, 0x3f7ffffb, 0xb5930000, 0x36200000, 0x3f7fffff, 0xbd7317ed, 0xbc139c52, 0xbc30bd61, 0x3de05658, 0xbc51a540, 
0x3b9e8330, 0xb60c0000, 0x35540000, 0x3f800000, 0xbe6ae3ae, 0xbf6ef4e1, 0xbe8d3e51
};

TestRagdoll::TestRagdoll()
{
	for(udword i=0;i<18;i++)
		mJoints[i] = null;
}

TestRagdoll::~TestRagdoll()
{
}

const Bone* TestRagdoll::FindBoneByName(int name) const
{
	for(udword i=0;i<19;i++)
		if(mBones[i].mID==name)
			return &mBones[i];
	return NULL;
}

static PintObjectHandle createBodyPart(Pint& pint, const Point& extents, const PR& pose, float mass, PintCollisionGroup group, bool use_compound)
{
	PINT_BOX_CREATE BoxDesc(extents);
	BoxDesc.mRenderer	= CreateBoxRenderer(extents);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes			= &BoxDesc;
	ObjectDesc.mPosition		= pose.mPos;
	ObjectDesc.mRotation		= pose.mRot;
	ObjectDesc.mMass			= mass;
	ObjectDesc.mCollisionGroup	= group;
	if(use_compound)
		ObjectDesc.mAddToWorld	= false;
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

bool TestRagdoll::Init(Pint& pint, const Point& offset, PintCollisionGroup group, bool use_compound, udword constraint_multiplier)
{
	BinReader Data((const char*)gRagdollData);
	const int NbBones = Data.readDword();
	ASSERT(NbBones==19);

	PintObjectHandle Aggregate = null;
	if(use_compound)
		Aggregate = pint.CreateAggregate(NbBones, false);

	for(int i=0;i<NbBones;i++)
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
		mBones[i].mBody = createBodyPart(pint, dimensions, massLocalPose, density, group, use_compound);
		mBones[i].mPose = massLocalPose;

		if(Aggregate)
			pint.AddToAggregate(mBones[i].mBody, Aggregate);
	}

	const char* SavedPtr = Data.mData;
	for(udword j=0;j<constraint_multiplier;j++)
	{
		Data.mData = SavedPtr;

		const Point* LocalVectors = reinterpret_cast<const Point*>(LocalVectors_Data);
		for(udword i=0;i<18;i++)
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

	/*		Matrix4x4 Mat0 = b0->mPose;
			Mat0.Invert();
			Matrix4x4 Mat1 = b1->mPose;
			Mat1.Invert();*/

			PINT_HINGE_JOINT_CREATE Desc;
			Desc.mObject0		= b0->mBody;
			Desc.mObject1		= b1->mBody;
	/*		Desc.mLocalPivot0	= globalAnchor * Mat0;
			Desc.mLocalPivot1	= globalAnchor * Mat1;
			Desc.mLocalAxis0	= globalAxis * Matrix3x3(Mat0);
			Desc.mLocalAxis1	= globalAxis * Matrix3x3(Mat1);*/

			if(1)
			{
	// ### test
	ComputeLocalAnchor(Desc.mLocalPivot0, globalAnchor, &b0->mPose);
	ComputeLocalAnchor(Desc.mLocalPivot1, globalAnchor, &b1->mPose);
	ComputeLocalAxis(Desc.mLocalAxis0, globalAxis, &b0->mPose);
	ComputeLocalAxis(Desc.mLocalAxis1, globalAxis, &b1->mPose);

			Desc.mGlobalAnchor	= globalAnchor;
			Desc.mGlobalAxis	= globalAxis;
			}
			else
			{
				// ### this path doesn't work yet with ragdoll scale != 0.1
				Desc.mLocalPivot0 = LocalVectors[i*4+0];
				Desc.mLocalPivot1 = LocalVectors[i*4+1];
				Desc.mLocalAxis0 = LocalVectors[i*4+2];
				Desc.mLocalAxis1 = LocalVectors[i*4+3];
			}
			Desc.mMinLimitAngle	= -0.2f;
			Desc.mMaxLimitAngle	= 0.2f;
	//		Desc.mMinLimitAngle	= -0.02f;
	//		Desc.mMaxLimitAngle	= 0.02f;
	//		Desc.mMinLimitAngle	= degToRad(-45.0f);
	//		Desc.mMaxLimitAngle	= degToRad(45.0f);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
	}

	if(Aggregate)
		pint.AddAggregateToScene(Aggregate);

	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool GenerateArrayOfRagdolls(Pint& pint, const PintCaps& caps, udword NbX, udword NbY, float Scale, Point* offset, bool use_aggregates, udword constraint_multiplier)
{
	if(!caps.mSupportHingeJoints || !caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
		return false;

	if(use_aggregates && !caps.mSupportAggregates)
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

			RD.Init(pint, Offset, 1, use_aggregates, constraint_multiplier);
		}
	}
	return true;
}

static bool GenerateColumnOfRagdolls(Pint& pint, const PintCaps& caps, udword nb, udword constraint_multiplier)
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
		RD.Init(pint, Offset, i+1, false, constraint_multiplier);
		Offset.y += Inc;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_RagdollConfigurable = "(Configurable test) - Each ragdoll is made of 19 bones connected by 18 hinge joints. Ragdoll-ragdoll collisions are disabled.";

#define NB_PRESETS	7
static const udword	gPreset_GridSize[NB_PRESETS]   = { 1,     1,    10,    10,   16,    16,   1 };
static const udword	gPreset_Multiplier[NB_PRESETS] = { 1,     1,    1,     1,    1,     1,    4 };
static const bool	gPreset_Aggregates[NB_PRESETS] = { false, true, false, true, false, true, false };

class RagdollConfigurable : public TestBase
{
			IceComboBox*	mComboBox_Preset;
			IceEditBox*		mEditBox_Size;
			IceEditBox*		mEditBox_Multiplier;
			IceCheckBox*	mCheckBox_Aggregates;
	public:
							RagdollConfigurable() :
								mComboBox_Preset	(null),
								mEditBox_Size		(null),
								mEditBox_Multiplier	(null),
								mCheckBox_Aggregates(null)	{}
	virtual					~RagdollConfigurable()		{									}
	virtual	const char*		GetName()			const	{ return "Ragdolls";				}
	virtual	const char*		GetDescription()	const	{ return gDesc_RagdollConfigurable;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_JOINTS;			}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 280;
		WD.mHeight	= 160;
		WD.mLabel	= "Ragdolls";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 100;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			mCheckBox_Aggregates = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Use aggregates", UIElems, false, null, null);
			mCheckBox_Aggregates->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Ragdoll array size:", UIElems);
			mEditBox_Size = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			mEditBox_Size->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Constraint multiplier:", UIElems);
			mEditBox_Multiplier = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			mEditBox_Multiplier->SetEnabled(false);
			y += YStep;
		}
		{
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Presets:", UIElems);

			class MyComboBox : public IceComboBox
			{
				RagdollConfigurable&	mTest;
				public:
								MyComboBox(const ComboBoxDesc& desc, RagdollConfigurable& test) :
									IceComboBox(desc),
									mTest(test)			{}
				virtual			~MyComboBox()			{}
				virtual	void	OnComboBoxEvent(ComboBoxEvent event)
				{
					if(event==CBE_SELECTION_CHANGED)
					{
						const udword SelectedIndex = GetSelectedIndex();
						const bool Enabled = SelectedIndex==GetItemCount()-1;
						mTest.mEditBox_Size->SetEnabled(Enabled);
						mTest.mEditBox_Multiplier->SetEnabled(Enabled);
						mTest.mCheckBox_Aggregates->SetEnabled(Enabled);

						if(!Enabled && SelectedIndex<NB_PRESETS)
						{
							mTest.mEditBox_Size->SetText(_F("%d", gPreset_GridSize[SelectedIndex]));
							mTest.mEditBox_Multiplier->SetText(_F("%d", gPreset_Multiplier[SelectedIndex]));
							mTest.mCheckBox_Aggregates->SetChecked(gPreset_Aggregates[SelectedIndex]);
						}
					}
				}
			};

			ComboBoxDesc CBBD;
			CBBD.mID		= 0;
			CBBD.mParent	= UI;
			CBBD.mX			= 4+OffsetX;
			CBBD.mY			= y;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;
			CBBD.mLabel		= "Presets";
			mComboBox_Preset = ICE_NEW(MyComboBox)(CBBD, *this);
			RegisterUIElement(mComboBox_Preset);
			mComboBox_Preset->Add("1 radgoll - regular");
			mComboBox_Preset->Add("1 radgoll - aggregate");
			mComboBox_Preset->Add("100 radgolls - regular");
			mComboBox_Preset->Add("100 radgolls - aggregates");
			mComboBox_Preset->Add("256 radgolls - regular");
			mComboBox_Preset->Add("256 radgolls - aggregates");
			mComboBox_Preset->Add("1 radgoll - constraints * 4");
			mComboBox_Preset->Add("User-defined");
			mComboBox_Preset->Select(0);
			mComboBox_Preset->SetVisible(true);
			mComboBox_Preset->OnComboBoxEvent(CBE_SELECTION_CHANGED);
			y += YStep;

			y += YStep;
			AddResetButton(UI, 4, y, 264);
		}
	}

	virtual	const char*		GetSubName()	const
	{
		if(mComboBox_Preset)
		{
			const udword SelectedIndex = mComboBox_Preset->GetSelectedIndex();
			if(SelectedIndex==0)
				return "Ragdoll";
			else if(SelectedIndex==1)
				return "Ragdoll_Aggregate";
			else if(SelectedIndex==2)
				return "Ragdolls_100";
			else if(SelectedIndex==3)
				return "Ragdolls_100_Aggregate";
			else if(SelectedIndex==4)
				return "Ragdolls_256";
			else if(SelectedIndex==5)
				return "Ragdolls_256_Aggregate";
			else if(SelectedIndex==6)
				return "Ragdoll_MultipleConstraints";
		}
		return null;
	}

	virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.58f, 1.32f, -14.27f), Point(-0.98f, -0.02f, -0.21f));
		desc.mCamera[1] = CameraPose(Point(15.68f, 2.85f, -4.21f), Point(-0.73f, -0.30f, -0.62f));
		desc.mCamera[2] = CameraPose(Point(17.08f, 2.98f, -1.78f), Point(-0.75f, -0.15f, -0.65f));
	}

	virtual	bool			Setup(Pint& pint, const PintCaps& caps)
	{
		const udword Size = GetFromEditBox(1, mEditBox_Size);
		const udword Multiplier = GetFromEditBox(1, mEditBox_Multiplier);
		const bool UseAggregates = mCheckBox_Aggregates ? mCheckBox_Aggregates->IsChecked() : false;
		return GenerateArrayOfRagdolls(pint, caps, Size, Size, 2.0f * float(Size), null, UseAggregates, Multiplier);
	}

}RagdollConfigurable;

///////////////////////////////////////////////////////////////////////////////

// TODO: fix this. It doesn't work when multiple engines are selected
#ifdef REMOVED
static const char* gDesc_ExplodingRagdoll = "Exploding ragdoll.";

START_TEST(ExplodingRagdoll, CATEGORY_WIP, gDesc_ExplodingRagdoll)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.58f, 1.32f, -14.27f), Point(-0.98f, -0.02f, -0.21f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
			return false;

		const PintDisabledGroups DG(1, 1);
		pint.SetDisabledGroups(1, &DG);

		TestRagdoll RD;
		const Point Offset(0.0f, 0.0f, 0.0f);
		RD.Init(pint, Offset, 1, false, 1);

//		pint.mUserData = RD.mBones[0].mBody;
		pint.mUserData = RD.mBones[1].mBody;

		return true;
	}

	virtual	udword	Update(Pint& pint, float dt)
	{
		PintObjectHandle h = (PintObjectHandle)pint.mUserData;
		if(h)
		{
			if(mCurrentTime>4.0f)
			{
				mCurrentTime = -FLT_MAX;

				const PR Pose = pint.GetWorldTransform(h);

				pint.AddWorldImpulseAtWorldPos(h, Point(0.0f, 500.0f, 0.0f), Pose.mPos);
			}
		}
		return TestBase::Update(pint, dt);
	}

END_TEST(ExplodingRagdoll)
#endif

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Ragdoll256_OnTerrain = "256 ragdolls on a terrain. Ragdoll-ragdoll collisions are disabled.";

//static udword DoBatchRaycasts(TestBase& test, Pint& pint, bool use_phantoms);

START_TEST(Ragdolls_256_OnTerrain, CATEGORY_JOINTS, gDesc_Ragdoll256_OnTerrain)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
//		desc.mCamera[0] = CameraPose(Point(17.08f, 2.98f, -1.78f), Point(-0.75f, -0.15f, -0.65f));
		desc.mCamera[0] = CameraPose(Point(2271.26f, 326.41f, 1416.32f), Point(-0.63f, -0.44f, 0.64f));
		desc.mCamera[1] = CameraPose(Point(2055.45f, 20.57f, 1983.79f), Point(-0.96f, -0.23f, 0.13f));
	}

	virtual bool	CommonSetup()
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

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
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

					RD.Init(pint, Offset, 1, false, 1);
				}
			}
		}
		gRagdollScale = Saved;
		return true;
//		return Status;
	}

/*	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchRaycasts(*this, pint, false);
	}*/

END_TEST(Ragdolls_256_OnTerrain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PileOfRagdolls16 = "A pile of 16 ragdolls. Ragdoll-ragdoll collisions are enabled.";

START_TEST(PileOfRagdolls_16, CATEGORY_JOINTS, gDesc_PileOfRagdolls16)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.88f, 2.47f, -11.72f), Point(-0.78f, -0.26f, -0.56f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return GenerateColumnOfRagdolls(pint, caps, 16, 1);
	}

END_TEST(PileOfRagdolls_16)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_StableSphericalChain = "Demonstrates how to make a long, stable spherical chain with regular joints (no articulations). \
This is a rope made of 256 spheres of mass 1, at the end of which we attach a box of mass 100. Ideally the rope should not stretch and the box should not \
go below the edge of the render window. You can increase the number of solver iteration counts in each engine to reduce stretching if needed.";

START_TEST(StableSphericalChain, CATEGORY_JOINTS, gDesc_StableSphericalChain)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(57.35f, -242.04f, 493.93f), Point(-0.06f, -0.05f, -1.00f));
		desc.mNbSimulateCallsPerFrame = 4;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportSphericalJoints)
			return false;

		bool UseDistanceConstraints = true;
		if(!caps.mSupportDistanceJoints)
		{
			printf(_F("WARNING: %s doesn't support distance joints, feature is disabled.\n", pint.GetName()));
			UseDistanceConstraints = false;
		}

		if(!caps.mSupportMassForInertia)
		{
			printf(_F("WARNING: %s doesn't support 'mass for inertia', feature is ignored.\n", pint.GetName()));
		}

		// Filtering is used to disable collisions between two jointed objects.
		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG(1, 2);
			pint.SetDisabledGroups(1, &DG);
		}

		const udword NbSpheres = 256;
		const float Radius = 1.0f;
		const float Mass = 1.0f;

		const Point InitPos(0.0f, 0.0f, 0.0f);
		Point Pos = InitPos;
		PintObjectHandle* Handles = new PintObjectHandle[NbSpheres];
		Point* Positions = ICE_NEW(Point)[NbSpheres];

		udword GroupBit = 0;
		{
			PINT_SPHERE_CREATE SphereDesc(Radius);
			SphereDesc.mRenderer = CreateSphereRenderer(Radius);

			const Point Offset(Radius*2.0f, 0.0f, 0.0f);
			const Point LocalPivot0	= Point(0.0f, 0.0f, 0.0f);
			const Point LocalPivot1	= Point(-Radius*2.0f, 0.0f, 0.0f);

			for(udword i=0;i<NbSpheres;i++)
			{
				Positions[i] = Pos;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &SphereDesc;
				ObjectDesc.mMass			= Mass;
				if(i==0)
					ObjectDesc.mMass		= 0.0f;
				ObjectDesc.mMassForInertia = Mass*10.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[i] = CreatePintObject(pint, ObjectDesc);

				Pos += Offset;
			}

			for(udword i=0;i<NbSpheres-1;i++)
			{
				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i], Handles[i+1], LocalPivot0, LocalPivot1));
				ASSERT(JointHandle);
			}

			if(UseDistanceConstraints)
			{
				for(udword i=0;i<NbSpheres;i++)
				{
					if(i+2<NbSpheres)
					{
						PINT_DISTANCE_JOINT_CREATE Desc;
						Desc.mObject0		= Handles[i];
						Desc.mObject1		= Handles[i+2];
						Desc.mMaxDistance	= Positions[i].Distance(Positions[i+2]);
						PintJointHandle JointHandle = pint.CreateJoint(Desc);
						ASSERT(JointHandle);
					}
				}
			}
		}

		// Attach heavy box to last sphere
		{
			PintObjectHandle HeavyBox;
			const Point BoxExtents(Radius*10.0f, Radius*10.0f, Radius*10.0f);
			{
				PINT_BOX_CREATE BoxDesc(BoxExtents);
				BoxDesc.mRenderer = CreateBoxRenderer(BoxExtents);

				Pos.x += BoxExtents.x - Radius;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= Mass * 100.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				HeavyBox = CreatePintObject(pint, ObjectDesc);
			}

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[NbSpheres-1], HeavyBox, Point(Radius, 0.0f, 0.0f), Point(-BoxExtents.x, 0.0f, 0.0f)));
			ASSERT(JointHandle);
		}

		DELETEARRAY(Positions);
		DELETEARRAY(Handles);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(StableSphericalChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CaterpillarTrack = "Caterpillar track using regular joints. This is an experimental, work-in-progress test. It has not been properly tuned or \
optimized, and it has not been tested in engines other than PhysX 3.4.";

START_TEST(CaterpillarTrack, CATEGORY_WIP, gDesc_CaterpillarTrack)

	CylinderMesh mCylinder;
	CylinderMesh mCylinder2;

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-0.09f, 2.56f, 10.53f), Point(-0.01f, 0.04f, -1.00f));
	}

	virtual	bool	CommonSetup()
	{
		const float HalfHeight = 0.5f;
		const float Radius = 0.6f;
		const udword NbCirclePts = 16;
		mCylinder.Generate(NbCirclePts, Radius, HalfHeight);
		RegisterRenderer(CreateConvexRenderer(mCylinder.mNbVerts, mCylinder.mVerts));

		const float HalfHeight2 = 0.15f;
		const float Radius2 = 1.0f;
		mCylinder2.Generate(NbCirclePts, Radius2, HalfHeight2);
		RegisterRenderer(CreateConvexRenderer(mCylinder2.mNbVerts, mCylinder2.mVerts));

		return TestBase::CommonSetup();
	}

	virtual	void	CommonRelease()
	{
		mCylinder.Reset();
		mCylinder2.Reset();
		TestBase::CommonRelease();
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportCompounds || !caps.mSupportHingeJoints)
			return false;

		CaterpillarTrackObjects TO;
		CreateCaterpillarTrack_OLD(pint, TO, &mHighFrictionMaterial, Point(0.0f, 2.0f, 0.0f), mCylinder, GetRegisteredRenderers()[0], mCylinder2, GetRegisteredRenderers()[1]);
		return true;
	}

END_TEST(CaterpillarTrack)

///////////////////////////////////////////////////////////////////////////////

#include "Camera.h"

namespace
{
static const bool gDriveVehicle = true;

	struct WheelData : public Allocateable
	{
		WheelData() : mParent(null), mWheel(null)
		{
		}

		PintObjectHandle	mParent;
		PintObjectHandle	mWheel;
	};

	struct VehicleData : public Allocateable
	{
		VehicleData() : mChassis(null)//, mNbWheels(0)
		{
			for(udword i=0;i<2;i++)
				mFrontAxleObject[i] = null;
		}
		PintObjectHandle	mChassis;
		WheelData			mFront[2];
		WheelData			mRear[2];
		PintObjectHandle	mFrontAxleObject[2];
	};

	struct VehicleCamera
	{
		VehicleCamera() : mUpOffset(0.0f), mDistToTarget(0.0f)	{}

		float	mUpOffset;
		float	mDistToTarget;
	};

	class VehicleBase : public TestBase
	{
		public:
									VehicleBase();
		virtual						~VehicleBase();

		virtual	void				Close(Pint& pint);
		virtual	bool				CommonSetup();
		virtual	void				CommonRelease();
		virtual	udword				GetFlags()	const;
		virtual	bool				SpecialKeyCallback(int key, int x, int y, bool down);

				PINT_VEHICLE_INPUT	mInput;
				float				mAcceleration;
				float				mMaxAngularVelocity;
				float				mSteeringForce;
				//
				Point				mFilteredCameraPos;
				Point				mFilteredCameraDir;
				VehicleCamera		mCamera;
				udword				mCameraMode;
				//
				bool				mClampAngularVelocity;
				bool				mControlCamera;
				bool				mLoadTerrain;

				bool				ClampAngularVelocity(Pint& pint, const VehicleData& vehicle_data);
				void				UpdateCamera(Pint& pint, PintObjectHandle focus_object);
	};

VehicleBase::VehicleBase() :
	mAcceleration			(0.0f),
	mMaxAngularVelocity		(0.0f),
	mSteeringForce			(0.0f),
	mFilteredCameraPos		(Point(0.0f, 0.0f, 0.0f)),
	mFilteredCameraDir		(Point(0.0f, 0.0f, 0.0f)),
	mCameraMode				(0),
	mClampAngularVelocity	(false),
	mControlCamera			(false),
	mLoadTerrain			(false)
{
}

VehicleBase::~VehicleBase()
{
}

void VehicleBase::Close(Pint& pint)
{
	VehicleData* UserData = (VehicleData*)pint.mUserData;
	DELETESINGLE(UserData);
	pint.mUserData = null;

	TestBase::Close(pint);
}

bool VehicleBase::CommonSetup()
{
	TestBase::CommonSetup();
	if(mLoadTerrain)
	{
		LoadMeshesFromFile_(*this, "Terrain.bin", null, false, 0);
		mCreateDefaultEnvironment = false;
	}
	return true;
}

void VehicleBase::CommonRelease()
{
	TestBase::CommonRelease();
}

udword VehicleBase::GetFlags() const
{
	return mControlCamera ? TEST_FLAGS_USE_CURSOR_KEYS : TEST_FLAGS_DEFAULT;
}

bool VehicleBase::SpecialKeyCallback(int key, int x, int y, bool down)
{
	switch(key)
	{
		case GLUT_KEY_UP:
		{
			mInput.mAccelerate = down;
		}
		break;
		case GLUT_KEY_DOWN:
		{
			mInput.mBrake = down;
		}
		break;
		case GLUT_KEY_LEFT:
		{
			mInput.mLeft = down;
		}
		break;
		case GLUT_KEY_RIGHT:
		{
			mInput.mRight = down;
		}
		break;
		case GLUT_KEY_PAGE_UP:
		{
			if(down)
				mCameraMode++;
		}
		break;
		case GLUT_KEY_PAGE_DOWN:
		{
			if(down)
				mCameraMode--;
		}
		break;
	}
	return true;
}

bool VehicleBase::ClampAngularVelocity(Pint& pint, const VehicleData& vehicle_data)
{
	bool CanAccelerate = true;
	if(mClampAngularVelocity)
	{
		const float MaxAngularVelocity = mMaxAngularVelocity;
		for(udword i=0;i<2;i++)
		{
			if(vehicle_data.mFront[i].mWheel && vehicle_data.mFront[i].mParent)
			{
//				Point AngularVelocity = pint.GetAngularVelocity(vehicle_data.mFront[i].mWheel);
//				printf("Angular velocity %d: %f %f %f\n", i, AngularVelocity.x, AngularVelocity.y, AngularVelocity.z);

				if(::ClampAngularVelocity(pint, vehicle_data.mFront[i].mParent, vehicle_data.mFront[i].mWheel, MaxAngularVelocity))
					CanAccelerate = false;
			}
		}
		for(udword i=0;i<2;i++)
		{
			if(vehicle_data.mRear[i].mWheel && vehicle_data.mRear[i].mParent)
			{
//				Point AngularVelocity = pint.GetAngularVelocity(vehicle_data.mRear[i].mWheel);
//				printf("Angular velocity %d: %f %f %f\n", i, AngularVelocity.x, AngularVelocity.y, AngularVelocity.z);

				if(::ClampAngularVelocity(pint, vehicle_data.mRear[i].mParent, vehicle_data.mRear[i].mWheel, MaxAngularVelocity))
					CanAccelerate = false;
			}
		}
	}
	return CanAccelerate;
}

void VehicleBase::UpdateCamera(Pint& pint, PintObjectHandle focus_object)
{
	if(!focus_object)
		return;

	// Won't work well more more than 1 engine at a time of course
	const PR Pose = pint.GetWorldTransform(focus_object);
//	printf("%f\n", Pose.mPos.y);
//	const Point CamPos = GetCameraPos();
//	const Point Dir = (Pose.mPos - CamPos).Normalize();
//	SetCamera(CamPos, Dir);

	Point D(0.0f, 0.0f, 0.0f);

	const Point Target = Pose.mPos;

	float Up = mCamera.mUpOffset;

//	printf("mCameraMode: %d\n", mCameraMode);
	const udword NbModes = 4;
	const udword Index = mCameraMode % NbModes;
//	printf("Index: %d\n", Index);
	if(Index==0)
	{
		const Matrix3x3 M(Pose.mRot);
		D = M[0];
		D.y = 0.0f;
		D.Normalize();
	}
	else if(Index==1)
	{
		const Matrix3x3 M(Pose.mRot);
		D = M[2];
		D.y = 0.0f;
		D.Normalize();
	}
	else if(Index==2)
	{
		const Matrix3x3 M(Pose.mRot);
		D = -M[0];
		D.y = 0.0f;
		D.Normalize();
	}
	else if(Index==3)
	{
		Up = 0.0f;
		D = Point(1.0f, 0.0f, 0.0f);
	}

	const Point CamPos = Pose.mPos + Point(0.0f, Up, 0.0f) - D*mCamera.mDistToTarget;

	const Point Dir = (Target - CamPos).Normalize();

	const float Sharpness = 0.2f;
	FeedbackFilter(CamPos.x, mFilteredCameraPos.x, Sharpness);
	FeedbackFilter(CamPos.y, mFilteredCameraPos.y, Sharpness);
	FeedbackFilter(CamPos.z, mFilteredCameraPos.z, Sharpness);
	FeedbackFilter(Dir.x, mFilteredCameraDir.x, Sharpness);
	FeedbackFilter(Dir.y, mFilteredCameraDir.y, Sharpness);
	FeedbackFilter(Dir.z, mFilteredCameraDir.z, Sharpness);

	Point Tmp = mFilteredCameraDir;
	Tmp.Normalize();

	SetCamera(mFilteredCameraPos, Tmp);
}

}

#define START_VEHICLE_TEST(name, category, desc)										\
	class name : public VehicleBase														\
	{																					\
		public:																			\
								name()						{						}	\
		virtual					~name()						{						}	\
		virtual	const char*		GetName()			const	{ return #name;			}	\
		virtual	const char*		GetDescription()	const	{ return desc;			}	\
		virtual	TestCategory	GetCategory()		const	{ return category;		}

static const char* gDesc_Tank = "Tank test. This uses regular joints (no articulation). This is an experimental, work-in-progress test. It has not been properly tuned or \
optimized, and it has not been tested in engines other than PhysX 3.4. Use the arrow keys to control the tank. Use the PageUp and PageDown keys to switch between different \
camera views. Controls are experimental.";

START_VEHICLE_TEST(Tank, CATEGORY_WIP, gDesc_Tank)

	CylinderMesh mCylinder;
	CylinderMesh mCylinder2;

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		VehicleBase::GetSceneParams(desc);
//		desc.mCamera[0] = CameraPose(Point(10.76f, 11.91f, 14.07f), Point(-0.54f, -0.49f, -0.68f));
		desc.mCamera[0] = CameraPose(Point(-27.42f, 10.88f, 79.49f), Point(0.97f, -0.26f, 0.01f));

		mFilteredCameraPos = desc.mCamera[0].mPos;
		mFilteredCameraDir = desc.mCamera[0].mDir;
	}

	virtual	bool	CommonSetup()
	{
		mSteeringForce			= 100.0f;
		mAcceleration			= 100.0f;
		mMaxAngularVelocity		= 10.0f;
//		mMaxAngularVelocity		= 5.0f;
		mCamera.mUpOffset		= 4.0f;
		mCamera.mDistToTarget	= 15.0f;
		mClampAngularVelocity	= true;
		mControlCamera			= gDriveVehicle;
		mLoadTerrain			= true;

		const float HalfHeight = 0.5f;
		const float Radius = 0.6f;
		const udword NbCirclePts = 16;
		mCylinder.Generate(NbCirclePts, Radius, HalfHeight);
		RegisterRenderer(CreateConvexRenderer(mCylinder.mNbVerts, mCylinder.mVerts));

		const float HalfHeight2 = 0.15f;
		const float Radius2 = 1.0f;
//		const float Radius2 = 1.1f;
		mCylinder2.Generate(NbCirclePts, Radius2, HalfHeight2);
		RegisterRenderer(CreateConvexRenderer(mCylinder2.mNbVerts, mCylinder2.mVerts));

		return VehicleBase::CommonSetup();
	}

	virtual	void	CommonRelease()
	{
		mCylinder.Reset();
		mCylinder2.Reset();
		VehicleBase::CommonRelease();
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportConvexes || !caps.mSupportCompounds || !caps.mSupportHingeJoints)
			return false;

		CreateMeshesFromRegisteredSurfaces(pint, caps, *this, &mHighFrictionMaterial);

		const float Altitude = 4.0f;
		const float D = 6.0f;

		Point RefPoint(0.0f, 4.0f+Altitude, D*0.5f);
		Point Pos0(0.0f, 2.0f+Altitude, 0.0f);
		Point Pos1(0.0f, 2.0f+Altitude, D);
		const Point Offset0 = RefPoint - Pos0;
		const Point Offset1 = RefPoint - Pos1;

		const Point StartPosPoint(-13.69f, 10.50f, 79.74f);
//		const Point StartPosPoint(11.05f, 7.94f, 63.99f);
		RefPoint = StartPosPoint;
		Pos0 = StartPosPoint - Offset0;
		Pos1 = StartPosPoint - Offset1;

		//
		CaterpillarTrackObjects TO0;
		CreateCaterpillarTrack_OLD(pint, TO0, &mHighFrictionMaterial, Pos0, mCylinder, GetRegisteredRenderers()[0], mCylinder2, GetRegisteredRenderers()[1]);

		CaterpillarTrackObjects TO1;
		CreateCaterpillarTrack_OLD(pint, TO1, &mHighFrictionMaterial, Pos1, mCylinder, GetRegisteredRenderers()[0], mCylinder2, GetRegisteredRenderers()[1]);

		VehicleData* UserData = ICE_NEW(VehicleData);
		pint.mUserData = UserData;
//		UserData->mChassis = TO0.mChassis;
		UserData->mFront[0].mWheel = TO0.mGears[0];
		UserData->mFront[1].mWheel = TO1.mGears[0];
		UserData->mFront[0].mParent = TO0.mChassis;
		UserData->mFront[1].mParent = TO0.mChassis;
		UserData->mRear[0].mWheel = TO0.mGears[1];
		UserData->mRear[1].mWheel = TO1.mGears[1];
		UserData->mRear[0].mParent = TO0.mChassis;
		UserData->mRear[1].mParent = TO0.mChassis;
//		UserData->mFrontAxleObject[0] = TO.mChassis;
//		UserData->mFrontAxleObject[1] = TO.mChassis;
		{
			PINT_FIXED_JOINT_CREATE fjc;
			fjc.mObject0 = TO0.mChassis;
			fjc.mObject1 = TO1.mChassis;
			fjc.mLocalPivot0 = Point(0.0f, 0.0f, D*0.5f);
			fjc.mLocalPivot1 = Point(0.0f, 0.0f, -D*0.5f);
			pint.CreateJoint(fjc);
		}
		{
			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mExtents	= Point(3.0f, 2.0f, D*0.2f);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mPosition		= RefPoint;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 1.0f;
//			ObjectDesc.mCollisionGroup	= WheelGroup;
			PintObjectHandle Object = pint.CreateObject(ObjectDesc);
			UserData->mChassis = Object;

			PINT_FIXED_JOINT_CREATE fjc;
			fjc.mObject0 = Object;
			fjc.mObject1 = TO0.mChassis;
			fjc.mLocalPivot0 = Point(0.0f, -2.0f, -BoxDesc.mExtents.x);
			fjc.mLocalPivot1 = Point(0.0f, 0.0f, D*0.5f-BoxDesc.mExtents.x);
			pint.CreateJoint(fjc);

			fjc.mObject0 = Object;
			fjc.mObject1 = TO1.mChassis;
			fjc.mLocalPivot0 = Point(0.0f, -2.0f, BoxDesc.mExtents.x);
			fjc.mLocalPivot1 = Point(0.0f, 0.0f, -D*0.5f+BoxDesc.mExtents.x);
			pint.CreateJoint(fjc);
		}

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		if(!gDriveVehicle)
			return 0;

		VehicleData* UserData = (VehicleData*)pint.mUserData;
		if(!UserData)
			return 0;

		bool CanAccelerate = ClampAngularVelocity(pint, *UserData);

		{
			const float Coeff = mSteeringForce;
			if(mInput.mRight)
			{
				pint.AddLocalTorque(UserData->mFront[0].mWheel, Point(0.0f, 0.0f, -Coeff));
				pint.AddLocalTorque(UserData->mRear[0].mWheel, Point(0.0f, 0.0f, -Coeff));
				pint.AddLocalTorque(UserData->mFront[1].mWheel, Point(0.0f, 0.0f, Coeff));
				pint.AddLocalTorque(UserData->mRear[1].mWheel, Point(0.0f, 0.0f, Coeff));
			}
			if(mInput.mLeft)
			{
				pint.AddLocalTorque(UserData->mFront[1].mWheel, Point(0.0f, 0.0f, -Coeff));
				pint.AddLocalTorque(UserData->mRear[1].mWheel, Point(0.0f, 0.0f, -Coeff));
				pint.AddLocalTorque(UserData->mFront[0].mWheel, Point(0.0f, 0.0f, Coeff));
				pint.AddLocalTorque(UserData->mRear[0].mWheel, Point(0.0f, 0.0f, Coeff));
			}
		}


		const bool FWD = true;
		const bool RWD = true;
		{
			const float Coeff = mAcceleration;
			if(mInput.mAccelerate /*&& CanAccelerate*/)
			{
				if(FWD)
				{
					for(udword i=0;i<2;i++)
					{
						if(UserData->mFront[i].mWheel)
							pint.AddLocalTorque(UserData->mFront[i].mWheel, Point(0.0f, 0.0f, -Coeff));
					}
				}
				if(RWD)
				{
					for(udword i=0;i<2;i++)
					{
						if(UserData->mRear[i].mWheel)
							pint.AddLocalTorque(UserData->mRear[i].mWheel, Point(0.0f, 0.0f, -Coeff));
					}
				}
			}
			if(mInput.mBrake)
			{
				if(FWD)
				{
					for(udword i=0;i<2;i++)
					{
						if(UserData->mFront[i].mWheel)
							pint.AddLocalTorque(UserData->mFront[i].mWheel, Point(0.0f, 0.0f, Coeff));
					}
				}
				if(RWD)
				{
					for(udword i=0;i<2;i++)
					{
						if(UserData->mRear[i].mWheel)
							pint.AddLocalTorque(UserData->mRear[i].mWheel, Point(0.0f, 0.0f, Coeff));
					}
				}
			}
		}

		// Camera
		UpdateCamera(pint, UserData->mChassis);

		return 0;
	}

END_TEST(Tank)

///////////////////////////////////////////////////////////////////////////////
