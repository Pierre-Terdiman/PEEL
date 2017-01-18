///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Camera.h"
#include "Cylinder.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"

///////////////////////////////////////////////////////////////////////////////

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
//		udword				mNbWheels;
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
		virtual	udword				Update(Pint& pint, float dt);
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
	mControlCamera			(false)
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
				if(::ClampAngularVelocity(pint, vehicle_data.mFront[i].mParent, vehicle_data.mFront[i].mWheel, MaxAngularVelocity))
					CanAccelerate = false;
			}
		}
		for(udword i=0;i<2;i++)
		{
			if(vehicle_data.mRear[i].mWheel && vehicle_data.mRear[i].mParent)
			{
				if(::ClampAngularVelocity(pint, vehicle_data.mRear[i].mParent, vehicle_data.mRear[i].mWheel, MaxAngularVelocity))
					CanAccelerate = false;
			}
		}
	}
	return CanAccelerate;
}

udword VehicleBase::Update(Pint& pint, float dt)
{
	if(!gDriveVehicle)
		return 0;

	VehicleData* UserData = (VehicleData*)pint.mUserData;
	if(!UserData)
		return 0;

	if(UserData->mFrontAxleObject[0] && UserData->mFrontAxleObject[1])
	{
		const float Steering = mSteeringForce;
		if(mInput.mRight)
		{
			pint.AddLocalTorque(UserData->mFrontAxleObject[0], Point(0.0f, -Steering, 0.0f));
			pint.AddLocalTorque(UserData->mFrontAxleObject[1], Point(0.0f, -Steering, 0.0f));
		}
		if(mInput.mLeft)
		{
			pint.AddLocalTorque(UserData->mFrontAxleObject[0], Point(0.0f, Steering, 0.0f));
			pint.AddLocalTorque(UserData->mFrontAxleObject[1], Point(0.0f, Steering, 0.0f));
		}
	}

	bool CanAccelerate = ClampAngularVelocity(pint, *UserData);

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

#define START_VEHICLE_TEST(name, category, desc)										\
	class name : public VehicleBase														\
	{																					\
		public:																			\
								name()						{						}	\
		virtual					~name()						{						}	\
		virtual	const char*		GetName()			const	{ return #name;			}	\
		virtual	const char*		GetDescription()	const	{ return desc;			}	\
		virtual	TestCategory	GetCategory()		const	{ return category;		}

}

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_VehicleTest = "Vehicle test. This uses regular joints (no articulation). This is an experimental, work-in-progress test. It has not been properly tuned or \
optimized, and it has not been tested in engines other than PhysX 3.4. Use the arrow keys to control the vehicle. Use the PageUp and PageDown keys to switch between different \
camera views.";

START_VEHICLE_TEST(VehicleTest, CATEGORY_WIP, gDesc_VehicleTest)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		VehicleBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.76f, 11.91f, 14.07f), Point(-0.54f, -0.49f, -0.68f));
	}

	virtual	bool	CommonSetup()
	{
		mAcceleration			= 100.0f;
		mMaxAngularVelocity		= 17.0f;
		mSteeringForce			= 200.0f;
		mCamera.mUpOffset		= 4.0f;
		mCamera.mDistToTarget	= 10.0f;
		mClampAngularVelocity	= true;
		mControlCamera			= gDriveVehicle;

		mCreateDefaultEnvironment = false;

		return VehicleBase::CommonSetup();
	}

	virtual void	CreateVehicle(Pint& pint, const Point& pos, VehicleData* UserData, const PINT_MATERIAL_CREATE& LowFrictionMaterial, PintCollisionGroup ChassisGroup, PintCollisionGroup WheelGroup)
	{
		//### experimental stuff, to revisit. Suspension doesn't work yet, etc

		const float MainScale = 0.25f;

		const float M = 4.0f;
		const float ChassisMass = M;
		const float WheelMass = M;
		const float StructMass = M;

		const float WheelRadius = 4.0f * MainScale;
		const float WheelWidth = 4.0f * MainScale;

		const udword NbPts = 60;
		const CylinderMesh Cylinder(NbPts, WheelRadius, WheelWidth*0.5f);
		const udword TotalNbVerts = Cylinder.mNbVerts;
		const Point* Verts = Cylinder.mVerts;

		//

		Container JointDescs;

		const Point Extents(MainScale*8.0f, MainScale*2.0f, MainScale*4.0f);

		PintObjectHandle Chassis;
		const Point ChassisPos = pos;

		{
			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mExtents	= Extents;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= ChassisMass;
			ObjectDesc.mPosition		= ChassisPos;
			ObjectDesc.mCollisionGroup	= ChassisGroup;
			ObjectDesc.mCOMLocalOffset	= Point(0.0f, -5.0f, 0.0f);
			Chassis = CreatePintObject(pint, ObjectDesc);
			if(UserData)
			UserData->mChassis = Chassis;
		}

		const Point SuspExtents(MainScale*1.0f, MainScale*1.0f, MainScale*1.0f);
		Point LocalAnchorPt[2];
		Point GlobalAnchorPt[2];
		PintObjectHandle AnchorObjects[2];
		for(udword j=0;j<2;j++)
		{
			const float z = j ? 1.0f : -1.0f;
			for(udword i=0;i<2;i++)
			{
				const float x = i ? 1.0f : -1.0f;
				const bool FrontWheels = i!=0;

				const Point Offset(x*(Extents.x-SuspExtents.x), -Extents.y-SuspExtents.y, z*(Extents.z-SuspExtents.z));
				const Point SuspPos = ChassisPos + Offset;
				PintObjectHandle SuspObject;
				{
					PINT_BOX_CREATE BoxDesc;
					BoxDesc.mExtents	= SuspExtents;
					BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes			= &BoxDesc;
					ObjectDesc.mMass			= StructMass;
					ObjectDesc.mPosition		= SuspPos;
					ObjectDesc.mCollisionGroup	= ChassisGroup;
					SuspObject = CreatePintObject(pint, ObjectDesc);

					PINT_PRISMATIC_JOINT_CREATE Desc;
					Desc.mObject0			= Chassis;
					Desc.mObject1			= SuspObject;
					Desc.mLocalPivot0		= Offset;
					Desc.mLocalPivot1		= Point(0.0f, 0.0f, 0.0f);
					Desc.mLocalAxis0		= Point(0.0f, 1.0f, 0.0f);
					Desc.mLocalAxis1		= Point(0.0f, 1.0f, 0.0f);
					Desc.mMaxLimit			= 0.01f;
					Desc.mMinLimit			= 0.0f;
					Desc.mSpringStiffness	= 1000.0f;
					Desc.mSpringDamping		= 100.0f;

					PINT_PRISMATIC_JOINT_CREATE* Copy = ICE_NEW(PINT_PRISMATIC_JOINT_CREATE);
					*Copy = Desc;
					JointDescs.Add(udword(Copy));
				}

				// We only create the hinge for the front wheels. For the rear wheels we'll attach the wheels
				// directly to the suspension.
				PintObjectHandle WheelAttachObject;
				Point WheelPosOffset;
				Point WheelJointOffset;
				if(FrontWheels)
				{
					const float Extension = -1.0f*MainScale;
					const Point HingeExtents(SuspExtents.x+fabsf(Extension), SuspExtents.y*0.5f, SuspExtents.z);
					WheelPosOffset = WheelJointOffset = Point(-Extension, 0.0f, z*HingeExtents.z);
					WheelPosOffset.x = 0.0f;
					{
						PINT_BOX_CREATE BoxDesc;
						BoxDesc.mExtents	= HingeExtents;
						BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

						PINT_OBJECT_CREATE ObjectDesc;
						ObjectDesc.mPosition		= SuspPos + Point(Extension, 0.0f, 0.0f);
						ObjectDesc.mShapes			= &BoxDesc;
						ObjectDesc.mMass			= StructMass;
						ObjectDesc.mCollisionGroup	= WheelGroup;
						const PintObjectHandle HingeObject = pint.CreateObject(ObjectDesc);
						if(UserData)
						UserData->mFrontAxleObject[j] = HingeObject;
						WheelAttachObject = HingeObject;

						LocalAnchorPt[j] = Point(Extension<0.0f ? -HingeExtents.x : HingeExtents.x, 0.0f, 0.0f);
						GlobalAnchorPt[j] = ObjectDesc.mPosition + LocalAnchorPt[j];
						AnchorObjects[j] = HingeObject;

						{
//							const float Limit = PI/6.0f;
							const float Limit = PI/4.0f;
							PINT_HINGE_JOINT_CREATE Desc;
							Desc.mObject0		= SuspObject;
							Desc.mObject1		= HingeObject;
							Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
							Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
							Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
							Desc.mLocalPivot1	= Point(-Extension, 0.0f, 0.0f);
							Desc.mMinLimitAngle	= -Limit;
							Desc.mMaxLimitAngle	= Limit;

							PINT_HINGE_JOINT_CREATE* Copy = ICE_NEW(PINT_HINGE_JOINT_CREATE);
							*Copy = Desc;
							JointDescs.Add(udword(Copy));
						}
					}
				}
				else
				{
					WheelAttachObject = SuspObject;
					WheelPosOffset = WheelJointOffset = Point(0.0f, 0.0f, z*SuspExtents.z);
				}

				{
					PINT_CONVEX_CREATE WheelDesc(TotalNbVerts, Verts);
					WheelDesc.mRenderer	= CreateConvexRenderer(WheelDesc.mNbVerts, WheelDesc.mVerts);
					if(FrontWheels)
						WheelDesc.mMaterial	= &mHighFrictionMaterial;
					else
						WheelDesc.mMaterial	= &LowFrictionMaterial;					

					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mPosition		= SuspPos + WheelPosOffset;
					ObjectDesc.mMass			= WheelMass;
					ObjectDesc.mShapes			= &WheelDesc;
					ObjectDesc.mCollisionGroup	= WheelGroup;

					{
						PintObjectHandle WheelObject = pint.CreateObject(ObjectDesc);
						if(UserData)
						{
						if(FrontWheels)
						{
							UserData->mFront[j].mWheel = WheelObject;
							UserData->mFront[j].mParent = WheelAttachObject;
						}
						else
						{
							UserData->mRear[j].mWheel = WheelObject;
							UserData->mRear[j].mParent = WheelAttachObject;
						}
						}

						PINT_HINGE_JOINT_CREATE Desc;
						Desc.mObject0		= WheelObject;
						Desc.mObject1		= WheelAttachObject;
						Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
						Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
						Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
						Desc.mLocalPivot1	= WheelJointOffset;

						PINT_HINGE_JOINT_CREATE* Copy = ICE_NEW(PINT_HINGE_JOINT_CREATE);
						*Copy = Desc;
						JointDescs.Add(udword(Copy));
					}
				}
			}
		}

		//###refactor with other test code
		const float Length = GlobalAnchorPt[0].Distance(GlobalAnchorPt[1]);
		{
			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mExtents	= Point(0.1f*MainScale, 0.1f*MainScale, Length*0.5f);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mPosition		= (GlobalAnchorPt[0] + GlobalAnchorPt[1])*0.5f;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= StructMass;
			ObjectDesc.mCollisionGroup	= WheelGroup;
			PintObjectHandle RodObject = pint.CreateObject(ObjectDesc);

			PINT_SPHERICAL_JOINT_CREATE Desc;

			Desc.mObject0		= AnchorObjects[0];
			Desc.mObject1		= RodObject;
			Desc.mLocalPivot0	= LocalAnchorPt[0];
			Desc.mLocalPivot1	= Point(0.0f, 0.0f, -BoxDesc.mExtents.z);

			PINT_SPHERICAL_JOINT_CREATE* Copy = ICE_NEW(PINT_SPHERICAL_JOINT_CREATE);
			*Copy = Desc;
			JointDescs.Add(udword(Copy));

			Desc.mObject0		= AnchorObjects[1];
			Desc.mObject1		= RodObject;
			Desc.mLocalPivot0	= LocalAnchorPt[1];
			Desc.mLocalPivot1	= Point(0.0f, 0.0f, BoxDesc.mExtents.z);

			PINT_SPHERICAL_JOINT_CREATE* Copy2 = ICE_NEW(PINT_SPHERICAL_JOINT_CREATE);
			*Copy2 = Desc;
			JointDescs.Add(udword(Copy2));
		}

		const udword NbJoints = JointDescs.GetNbEntries();
		for(udword j=0;j<32;j++)
		{
			for(udword i=0;i<NbJoints;i++)
			{
				const PINT_JOINT_CREATE* jc = (const PINT_JOINT_CREATE*)JointDescs.GetEntry(i);
				PintJointHandle JointHandle = pint.CreateJoint(*jc);
				ASSERT(JointHandle);
			}
		}
		for(udword i=0;i<NbJoints;i++)
		{
			const PINT_JOINT_CREATE* jc = (const PINT_JOINT_CREATE*)JointDescs.GetEntry(i);
			DELETESINGLE(jc);
		}
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportHingeJoints || !caps.mSupportPrismaticJoints || !caps.mSupportRigidBodySimulation)
			return false;

		PINT_MATERIAL_CREATE LowFrictionMaterial;
		LowFrictionMaterial.mStaticFriction		= 0.75f;
		LowFrictionMaterial.mDynamicFriction	= 0.75f;
		LowFrictionMaterial.mRestitution		= 0.0f;

		const PintCollisionGroup ChassisGroup = 1;
		const PintCollisionGroup WheelGroup = 2;

		// ### there's some weird contact creating trouble, just disable everything for now
		//		const PintDisabledGroups DG(ChassisGroup, WheelGroup);
		//		pint.SetDisabledGroups(1, &DG);
		//		const PintDisabledGroups DG[2] = { PintDisabledGroups(1, 2), PintDisabledGroups(1, 1)	};
		//		pint.SetDisabledGroups(2, DG);
		const PintDisabledGroups DG[3*2] = {
			PintDisabledGroups(1, 2),
			PintDisabledGroups(1, 1),
			PintDisabledGroups(2, 2),

			PintDisabledGroups(3, 4),
			PintDisabledGroups(3, 3),
			PintDisabledGroups(4, 4)
		};
		pint.SetDisabledGroups(3*2, DG);

		VehicleData* UserData = ICE_NEW(VehicleData);
		pint.mUserData = UserData;

		const float MainScale = 0.25f;
		const Point Extents(MainScale*8.0f, MainScale*2.0f, MainScale*4.0f);
		const Point ChassisPos(0.0f, 4.0f+Extents.y*4.0f, 0.0f);

		CreateVehicle(pint, ChassisPos, UserData, LowFrictionMaterial, ChassisGroup, WheelGroup);
		CreateVehicle(pint, ChassisPos + Point(10.0f, 0.0f, 0.0f), null, LowFrictionMaterial, 3, 4);

		// Box stack
		{
			PINT_MATERIAL_CREATE BoxMaterial;
			BoxMaterial.mStaticFriction		= 0.2f;
			BoxMaterial.mDynamicFriction	= 0.2f;
		//	BoxMaterial.mStaticFriction		= 1.0f;
		//	BoxMaterial.mDynamicFriction	= 1.0f;
			BoxMaterial.mRestitution		= 0.0f;

			const float BoxExtent = 0.2f;

			PINT_BOX_CREATE BoxDesc(BoxExtent, BoxExtent, BoxExtent);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
			BoxDesc.mMaterial	= &BoxMaterial;

			const udword NbStacks = 1;
			for(udword j=0;j<NbStacks;j++)
			{
				udword NbBoxes = 20;
				float BoxPosY = BoxExtent;
				while(NbBoxes)
				{
					for(udword i=0;i<NbBoxes;i++)
					{
				//		const float Coeff = float(i)/float(NbBoxes-1);
						const float Coeff = float(i) - float(NbBoxes)*0.5f;

						PINT_OBJECT_CREATE ObjectDesc;
						ObjectDesc.mShapes		= &BoxDesc;
						ObjectDesc.mPosition.x	= Coeff * BoxExtent * 2.0f;
						ObjectDesc.mPosition.y	= BoxPosY;
						ObjectDesc.mPosition.z	= float(j) * BoxExtent * 4.0f;
						ObjectDesc.mPosition.z += 10.0f;
						ObjectDesc.mMass		= 1.0f;
						ObjectDesc.mMass		= 0.1f;
						PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
						ASSERT(Handle);
					}

					NbBoxes--;
					BoxPosY += BoxExtent*2.0f;
				}
			}
		}

		const float Altitude = 0.0f;
		CreatePlanarMesh(*this, pint, Altitude, 100, 0.1f, null);

		return true;
	}

END_TEST(VehicleTest)

///////////////////////////////////////////////////////////////////////////////
