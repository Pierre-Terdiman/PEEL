///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_H
#define PINT_H

#include "PintDef.h"

	class PintSQ;
	class ObjectsManager;

	#define PINT_MAX_CAMERA_POSES	16

	enum PintShape
	{
		PINT_SHAPE_UNDEFINED,
		PINT_SHAPE_SPHERE,
		PINT_SHAPE_CAPSULE,
		PINT_SHAPE_CYLINDER,
		PINT_SHAPE_BOX,
		PINT_SHAPE_CONVEX,
		PINT_SHAPE_MESH,
	};

	enum PintJoint
	{
		PINT_JOINT_UNDEFINED,
		PINT_JOINT_SPHERICAL,	// Spherical joint a.k.a. point-to-point constraint a.k.a. ball-and-socket
		PINT_JOINT_HINGE,		// Hinge joints a.k.a. revolute joints
		PINT_JOINT_PRISMATIC,	// Prismatic joints, a.k.a. slider constraints
		PINT_JOINT_FIXED,		// Fixed joints
		PINT_JOINT_DISTANCE,	// Distance joints
	};

	enum PintActionType
	{
		PINT_ACTION_FORCE,
		PINT_ACTION_IMPULSE,
	};

	class PintShapeRenderer	: public Allocateable
	{
		public:
									PintShapeRenderer()		{}
		virtual						~PintShapeRenderer()	{}

		virtual	void				Render(const PR& pose)						= 0;
		virtual	void				SetColor(const Point& color, bool isStatic)	= 0;
		virtual	void				SetShadows(bool flag)						= 0;
	};

	class PintGUIHelper	: public Allocateable
	{
		public:
									PintGUIHelper()		{}
		virtual						~PintGUIHelper()	{}

		virtual	const char*			Convert(float value)	= 0;

		virtual	IceWindow*			CreateMainWindow(Container*& gui, IceWidget* parent, udword id, const char* label)	= 0;
		virtual	IceLabel*			CreateLabel		(IceWidget* parent,				sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner)	= 0;
		virtual	IceCheckBox*		CreateCheckBox	(IceWidget* parent, udword id,	sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, bool state, CBCallback callback, const char* tooltip=null)	= 0;
		virtual	IceEditBox*			CreateEditBox	(IceWidget* parent, udword id,	sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, EditBoxFilter filter, EBCallback callback, const char* tooltip=null)	= 0;
	};

	struct CameraPose
	{
				CameraPose()															{ mPos = Point(50.0f, 50.0f, 50.0f); mDir = Point(-0.6f, -0.2f, -0.7f);	}
				CameraPose(const Point& pos, const Point& dir) : mPos(pos), mDir(dir)	{}

		bool	operator == (const CameraPose& other)	const
		{
			return mPos==other.mPos && mDir==other.mDir;
		}

		bool	operator != (const CameraPose& other)	const
		{
			return mPos!=other.mPos || mDir!=other.mDir;
		}

		Point	mPos;
		Point	mDir;
	};

	//! Contains scene-related parameters. This is used to initialize each PINT engine, *before* the test itself is setup.
	class PINT_WORLD_CREATE : public Allocateable
	{
		protected:
		const char*					mTestName;	// Setup by the system
		public:
									PINT_WORLD_CREATE() :
										mTestName				(null),
										mGravity				(0.0f, 0.0f, 0.0f),
										mNbSimulateCallsPerFrame(1),
										mTimestep				(1.0f/60.0f)
									{
										mGlobalBounds.SetEmpty();
									}

		// If set by the test, this is used to setup the broadphase bounds in engines that need them. Otherwise some
		// default bounds are used. Fine-tuning the bounds may improve the broadphase performance.
				AABB				mGlobalBounds;

		// Camera poses for current test. You can define up to PINT_MAX_CAMERA_POSES poses per test. Just write them
		// sequentially in the array, PEEL will automatically find how many there are.
				CameraPose			mCamera[PINT_MAX_CAMERA_POSES];

		// Gravity vector for current test.
				Point				mGravity;

		// Number of simulate calls per render frame. It is usually 1 but it can be set to more than 1 to
		// artificially speed up the test scene.
				udword				mNbSimulateCallsPerFrame;

		// Timestep for one simulate call. It is usually 1/60 (for 60Hz).
				float				mTimestep;

		inline	const char*			GetTestName()	const	{ return mTestName;	}
	};

	struct PINT_MATERIAL_CREATE	: public Allocateable
	{
									PINT_MATERIAL_CREATE() :
										mStaticFriction		(0.0f),
										mDynamicFriction	(0.0f),
										mRestitution		(0.0f)
									{
									}

		float						mStaticFriction;
		float						mDynamicFriction;
		float						mRestitution;
	};

	struct PINT_SHAPE_CREATE : public Allocateable
	{
									PINT_SHAPE_CREATE() :
										mType		(PINT_SHAPE_UNDEFINED),
										mMaterial	(null),
										mRenderer	(null),
										mNext		(null)
									{
										mLocalPos.Zero();
										mLocalRot.Identity();
									}

		PintShape					mType;
		Point						mLocalPos;
		Quat						mLocalRot;
		const PINT_MATERIAL_CREATE*	mMaterial;
		PintShapeRenderer*			mRenderer;
		const PINT_SHAPE_CREATE*	mNext;
	};

	struct PINT_SPHERE_CREATE : PINT_SHAPE_CREATE
	{
									PINT_SPHERE_CREATE(float radius=0.0f) : mRadius(radius)
									{
										mType	= PINT_SHAPE_SPHERE;
									}

		float						mRadius;
	};

	struct PINT_CAPSULE_CREATE : PINT_SHAPE_CREATE
	{
									PINT_CAPSULE_CREATE(float radius=0.0f, float half_height=0.0f) : mRadius(radius), mHalfHeight(half_height)
									{
										mType	= PINT_SHAPE_CAPSULE;
									}

		float						mRadius;
		float						mHalfHeight;
	};

	struct PINT_CYLINDER_CREATE : PINT_SHAPE_CREATE
	{
									PINT_CYLINDER_CREATE(float radius=0.0f, float half_height=0.0f) : mRadius(radius), mHalfHeight(half_height)
									{
										mType	= PINT_SHAPE_CYLINDER;
									}

		float						mRadius;
		float						mHalfHeight;
	};

	struct PINT_BOX_CREATE : PINT_SHAPE_CREATE
	{
									PINT_BOX_CREATE(float x=0.0f, float y=0.0f, float z=0.0f) : mExtents(x, y, z)
									{
										mType	= PINT_SHAPE_BOX;
									}
									PINT_BOX_CREATE(const Point& extents) : mExtents(extents)
									{
										mType	= PINT_SHAPE_BOX;
									}

		Point						mExtents;
	};

	struct PINT_CONVEX_DATA_CREATE : public Allocateable
	{
									PINT_CONVEX_DATA_CREATE(udword nb_verts=0, const Point* verts=null) :
										mNbVerts	(nb_verts),
										mVerts		(verts),
										mRenderer	(null)
									{
									}

		udword						mNbVerts;
		const Point*				mVerts;
		PintShapeRenderer*			mRenderer;
	};

	struct PINT_CONVEX_CREATE : PINT_SHAPE_CREATE
	{
									PINT_CONVEX_CREATE(udword nb_verts=0, const Point* verts=null) :
										mNbVerts	(nb_verts),
										mVerts		(verts)
									{
										mType	= PINT_SHAPE_CONVEX;
									}

		udword						mNbVerts;
		const Point*				mVerts;
	};

	struct PINT_MESH_CREATE : PINT_SHAPE_CREATE
	{
									PINT_MESH_CREATE()
									{
										mType	= PINT_SHAPE_MESH;
									}

		SurfaceInterface			mSurface;
	};

	struct PINT_OBJECT_CREATE : public Allocateable
	{
									PINT_OBJECT_CREATE() :
										mShapes			(null),
										mMass			(0.0f),
										mMassForInertia	(-1.0f),
										mCollisionGroup	(0),
										mKinematic		(false),
										mAddToWorld		(true)
									{
										mPosition.Zero();
										mRotation.Identity();
										mLinearVelocity.Zero();
										mAngularVelocity.Zero();
										mCOMLocalOffset.Zero();
									}

		const PINT_SHAPE_CREATE*	mShapes;
		Point						mPosition;
		Quat						mRotation;
		Point						mCOMLocalOffset;
		Point						mLinearVelocity;
		Point						mAngularVelocity;
		float						mMass;
		float						mMassForInertia;	// If negative, use the same as mMass.
		PintCollisionGroup			mCollisionGroup;	// 0-31
		bool						mKinematic;
		bool						mAddToWorld;

		udword						GetNbShapes()	const
									{
										udword NbShapes = 0;
										const PINT_SHAPE_CREATE* CurrentShape = mShapes;
										while(CurrentShape)
										{
											NbShapes++;
											CurrentShape = CurrentShape->mNext;
										}
										return NbShapes;
									}
	};

	struct PINT_JOINT_CREATE : public Allocateable
	{
									PINT_JOINT_CREATE() :
										mType		(PINT_JOINT_UNDEFINED),
										mObject0	(null),
										mObject1	(null)
									{
									}

		PintJoint					mType;
		PintObjectHandle			mObject0;
		PintObjectHandle			mObject1;
	};

	struct PINT_SPHERICAL_JOINT_CREATE : PINT_JOINT_CREATE
	{
									PINT_SPHERICAL_JOINT_CREATE()
									{
										mType = PINT_JOINT_SPHERICAL;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
									}
									PINT_SPHERICAL_JOINT_CREATE(PintObjectHandle object0, PintObjectHandle object1, const Point& p0, const Point& p1)
									{
										mType = PINT_JOINT_SPHERICAL;
										mLocalPivot0 = p0;
										mLocalPivot1 = p1;
										mObject0 = object0;
										mObject1 = object1;
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
	};

	struct PINT_HINGE_JOINT_CREATE : PINT_JOINT_CREATE
	{
									PINT_HINGE_JOINT_CREATE()
									{
										mType = PINT_JOINT_HINGE;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
										mLocalAxis0.Zero();
										mLocalAxis1.Zero();
										mMinLimitAngle	= MIN_FLOAT;
										mMaxLimitAngle	= MAX_FLOAT;
										//###temp
										mGlobalAnchor.SetNotUsed();
										mGlobalAxis.SetNotUsed();
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
		Point						mLocalAxis0;
		Point						mLocalAxis1;
		float						mMinLimitAngle;
		float						mMaxLimitAngle;
		//###temp
		Point						mGlobalAnchor;
		Point						mGlobalAxis;
	};

	struct PINT_PRISMATIC_JOINT_CREATE : PINT_JOINT_CREATE
	{
										PINT_PRISMATIC_JOINT_CREATE() :
										// Limits valid if min<=max
										mMinLimit		(1.0f),
										mMaxLimit		(-1.0f),
										mSpringStiffness(0.0f),
										mSpringDamping	(0.0f)
									{
										mType = PINT_JOINT_PRISMATIC;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
										mLocalAxis0.Zero();
										mLocalAxis1.Zero();
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
		Point						mLocalAxis0;
		Point						mLocalAxis1;
		float						mMinLimit;
		float						mMaxLimit;
		float						mSpringStiffness;
		float						mSpringDamping;
	};

	struct PINT_FIXED_JOINT_CREATE : PINT_JOINT_CREATE
	{
									PINT_FIXED_JOINT_CREATE()
									{
										mType = PINT_JOINT_FIXED;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
	};

	struct PINT_DISTANCE_JOINT_CREATE : PINT_JOINT_CREATE
	{
									PINT_DISTANCE_JOINT_CREATE()
									{
										mType = PINT_JOINT_DISTANCE;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
										mMinDistance = -1.0f;
										mMaxDistance = -1.0f;
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
		float						mMinDistance;
		float						mMaxDistance;
	};

	struct PINT_ARTICULATION_CREATE : public Allocateable
	{
									PINT_ARTICULATION_CREATE()
									{
									}
	};

	struct PINT_ARTICULATED_MOTOR_CREATE : public Allocateable
	{
									PINT_ARTICULATED_MOTOR_CREATE() :
										mExternalCompliance	(0.0f),
										mInternalCompliance	(0.0f),
										mStiffness			(0.0f),
										mDamping			(0.0f)
									{
										mTargetVelocity.SetNotUsed();
										mTargetOrientation.SetNotUsed();
									}

		Quat						mTargetOrientation;
		Point						mTargetVelocity;
		float						mExternalCompliance;
		float						mInternalCompliance;
		float						mStiffness;
		float						mDamping;
	};

	struct PINT_ARTICULATED_BODY_CREATE	: public Allocateable
	{
									PINT_ARTICULATED_BODY_CREATE() :
										mParent				(null),
										mLocalPivot0		(Point(0.0f, 0.0f, 0.0f)),
										mLocalPivot1		(Point(0.0f, 0.0f, 0.0f)),
										mX					(Point(1.0f, 0.0f, 0.0f)),
										mSwingYLimit		(0.0f),
										mSwingZLimit		(0.0f),
										mTwistLowerLimit	(0.0f),
										mTwistUpperLimit	(0.0f),
										mEnableTwistLimit	(false),
										mEnableSwingLimit	(false),
										//
										mUseMotor			(false)
										{
										}

		PintObjectHandle				mParent;
		Point							mLocalPivot0;	// parent
		Point							mLocalPivot1;	// child
		Point							mX;
		float							mSwingYLimit;
		float							mSwingZLimit;
		float							mTwistLowerLimit;
		float							mTwistUpperLimit;
		bool							mEnableTwistLimit;
		bool							mEnableSwingLimit;
		//
		bool							mUseMotor;
		PINT_ARTICULATED_MOTOR_CREATE	mMotor;
	};

	struct PINT_VEHICLE_INPUT : public Allocateable
	{
				PINT_VEHICLE_INPUT() :
					mAccelerate	(false),
					mBrake		(false),
					mLeft		(false),
					mRight		(false)
				{
				}

		bool	mAccelerate;
		bool	mBrake;
		bool	mLeft;
		bool	mRight;
	};

	enum PintVehicleDifferential
	{
		DIFFERENTIAL_LS_4WD,		// limited slip differential for car with 4 driven wheels
		DIFFERENTIAL_LS_FRONTWD,	// limited slip differential for car with front-wheel drive
		DIFFERENTIAL_LS_REARWD,		// limited slip differential for car with rear-wheel drive
		DIFFERENTIAL_OPEN_4WD,		// open differential for car with 4 driven wheels 
		DIFFERENTIAL_OPEN_FRONTWD,	// open differential for car with front-wheel drive
		DIFFERENTIAL_OPEN_REARWD,	// open differential for car with rear-wheel drive
		DIFFERENTIAL_UNDEFINED
	};

	// This struct is currently based on the PhysX API.
	struct PINT_VEHICLE_CREATE	: public Allocateable
	{
									PINT_VEHICLE_CREATE() :
										mChassisMass				(0.0f),
										mChassisMOICoeffY			(0.0f),
										mChassisCMOffsetY			(0.0f),
										mChassisCMOffsetZ			(0.0f),
										mForceApplicationCMOffsetY	(0.0f),
										mWheelMass					(0.0f),
										mWheelMaxBrakeTorqueFront	(0.0f),
										mWheelMaxBrakeTorqueRear	(0.0f),
										mWheelMaxSteerFront			(0.0f),
										mWheelMaxSteerRear			(0.0f),
										mTireFrictionMultiplier		(0.0f),
										mEnginePeakTorque			(0.0f),
										mEngineMaxOmega				(0.0f),
										mGearsSwitchTime			(0.0f),
										mClutchStrength				(0.0f),
										mDifferential				(DIFFERENTIAL_UNDEFINED),
										mSuspMaxCompression			(0.0f),
										mSuspMaxDroop				(0.0f),
										mSuspSpringStrength			(0.0f),
										mSuspSpringDamperRate		(0.0f),
										mSuspCamberAngleAtRest		(0.0f),
										mSuspCamberAngleAtMaxCompr	(0.0f),
										mSuspCamberAngleAtMaxDroop	(0.0f)
									{
										mStartPose.Identity();
										for(udword i=0;i<4;i++)
											mWheelOffset[i].Zero();
									}

		PR							mStartPose;

		// Chassis
		PINT_CONVEX_CREATE			mChassis;
		float						mChassisMass;				// See PxVehicleChassisData::mMass for details
		// "A bit of tweaking here.  The car will have more responsive turning if we reduce the y-component of the chassis moment of inertia."
		float						mChassisMOICoeffY;			// See PxVehicleChassisData::mMOI for details
		float						mChassisCMOffsetY;			// See PxVehicleChassisData::mCMOffset for details
		float						mChassisCMOffsetZ;			// See PxVehicleChassisData::mCMOffset for details
		float						mForceApplicationCMOffsetY;

		// Wheels
		PINT_CONVEX_CREATE			mWheel;
		Point						mWheelOffset[4];			// Only supports 4-wheeled vehicles for now
		float						mWheelMass;					// See PxVehicleWheelData::mMass for details
		float						mWheelMaxBrakeTorqueFront;	// See PxVehicleWheelData::mMaxBrakeTorque for details
		float						mWheelMaxBrakeTorqueRear;	// See PxVehicleWheelData::mMaxBrakeTorque for details
		float						mWheelMaxSteerFront;		// See PxVehicleWheelData::mMaxSteer for details
		float						mWheelMaxSteerRear;			// See PxVehicleWheelData::mMaxSteer for details
		float						mTireFrictionMultiplier;

		// Engine
		float						mEnginePeakTorque;			// See PxVehicleEngineData::mPeakTorque for details
		float						mEngineMaxOmega;			// See PxVehicleEngineData::mMaxOmega for details
		// Gears
		float						mGearsSwitchTime;			// See PxVehicleGearsData::mSwitchTime for details
		// Clutch
		float						mClutchStrength;			// See PxVehicleClutchData::mStrength for details
		// Differential
		PintVehicleDifferential		mDifferential;				// See PxVehicleDifferential4WData for details

		// Suspension
		float						mSuspMaxCompression;		// See PxVehicleSuspensionData::mMaxCompression for details
		float						mSuspMaxDroop;				// See PxVehicleSuspensionData::mMaxDroop for details
		float						mSuspSpringStrength;		// See PxVehicleSuspensionData::mSpringStrength for details
		float						mSuspSpringDamperRate;		// See PxVehicleSuspensionData::mSpringDamperRate for details
		float						mSuspCamberAngleAtRest;		// See PxVehicleSuspensionData::mCamberAtRest for details
		float						mSuspCamberAngleAtMaxCompr;	// See PxVehicleSuspensionData::mCamberAtMaxCompression for details
		float						mSuspCamberAngleAtMaxDroop;	// See PxVehicleSuspensionData::mCamberAtMaxDroop for details
	};

	struct PintVehicleData
	{
		PintObjectHandle	mChassis;
	};

	// See the PintCaps ctor comments for explanations about the caps.
	struct PintCaps : public Allocateable
	{
				PintCaps();

		bool	mSupportRigidBodySimulation;
		bool	mSupportCylinders;
		bool	mSupportConvexes;
		bool	mSupportMeshes;
		bool	mSupportMassForInertia;
		bool	mSupportKinematics;
		bool	mSupportCollisionGroups;
		bool	mSupportCompounds;
		bool	mSupportAggregates;
		//
		bool	mSupportSphericalJoints;
		bool	mSupportHingeJoints;
		bool	mSupportFixedJoints;
		bool	mSupportPrismaticJoints;
		bool	mSupportDistanceJoints;
		bool	mSupportArticulations;
		//
		bool	mSupportPhantoms;
		bool	mSupportRaycasts;
		//
		bool	mSupportBoxSweeps;
		bool	mSupportSphereSweeps;
		bool	mSupportCapsuleSweeps;
		bool	mSupportConvexSweeps;
		//
		bool	mSupportSphereOverlaps;
		bool	mSupportBoxOverlaps;
		bool	mSupportCapsuleOverlaps;
		bool	mSupportConvexOverlaps;
		//
		bool	mSupportVehicles;
	};

	struct PintDisabledGroups : public Allocateable
	{
							PintDisabledGroups(PintCollisionGroup group0, PintCollisionGroup group1) : mGroup0(group0), mGroup1(group1)		{}

		PintCollisionGroup	mGroup0;
		PintCollisionGroup	mGroup1;
	};

	struct PintBooleanHit : public Allocateable
	{
		bool	mHit;
	};

	struct PintRaycastHit : public Allocateable
	{
		PintObjectHandle	mObject;
		Point				mImpact;
		Point				mNormal;
		float				mDistance;
		udword				mTriangleIndex;
	};

	struct PintOverlapObjectHit : public Allocateable
	{
		udword	mNbObjects;
	};

	struct PintRaycastData : public Allocateable
	{
		Point	mOrigin;
		Point	mDir;
		float	mMaxDist;
	};

	struct PintSphereOverlapData : public Allocateable
	{
		Sphere	mSphere;
	};

	struct PintBoxOverlapData : public Allocateable
	{
		OBB		mBox;
	};

	struct PintCapsuleOverlapData : public Allocateable
	{
		LSS		mCapsule;
	};

	struct PintConvexOverlapData : public Allocateable
	{
		udword				mConvexObjectIndex;
		PR					mTransform;
	};

	struct PintSweepData
	{
		Point	mDir;
		float	mMaxDist;
	};

	struct PintBoxSweepData : public PintBoxOverlapData, public PintSweepData
	{
	};

	struct PintSphereSweepData : public PintSphereOverlapData, public PintSweepData
	{
	};

	struct PintCapsuleSweepData : public PintCapsuleOverlapData, public PintSweepData
	{
	};

	struct PintConvexSweepData : public PintConvexOverlapData, public PintSweepData
	{
		PintShapeRenderer*			mRenderer;
	};

	class PintRender : public Allocateable
	{
		public:
									PintRender()					{}
		virtual						~PintRender()					{}

		virtual	void				DrawLine						(const Point& p0, const Point& p1, const Point& color)																	= 0;
		virtual	void				DrawTriangle					(const Point& p0, const Point& p1, const Point& p2, const Point& color)													= 0;
		virtual	void				DrawWirefameAABB				(const AABB& box, const Point& color)																					= 0;
		virtual	void				DrawWirefameOBB					(const OBB& box, const Point& color)																					= 0;
		virtual	void				DrawSphere						(float radius, const PR& pose)																							= 0;
		virtual	void				DrawBox							(const Point& extents, const PR& pose)																					= 0;
		virtual	void				DrawCapsule						(float radius, float height, const PR& pose)																			= 0;
		//
		virtual	void				DrawRaycastData					(udword nb, const PintRaycastData* raycast_data, const PintRaycastHit* hits, const Point& color)						= 0;
		virtual	void				DrawRaycastAnyData				(udword nb, const PintRaycastData* raycast_data, const PintBooleanHit* hits, const Point& color)						= 0;
		//
		virtual	void				DrawBoxSweepData				(udword nb, const PintBoxSweepData* box_sweep_data, const PintRaycastHit* hits, const Point& color)						= 0;
		virtual	void				DrawSphereSweepData				(udword nb, const PintSphereSweepData* sphere_sweep_data, const PintRaycastHit* hits, const Point& color)				= 0;
		virtual	void				DrawCapsuleSweepData			(udword nb, const PintCapsuleSweepData* capsule_sweep_data, const PintRaycastHit* hits, const Point& color)				= 0;
		virtual	void				DrawConvexSweepData				(udword nb, const PintConvexSweepData* convex_sweep_data, const PintRaycastHit* hits, const Point& color)				= 0;
		//
		virtual	void				DrawSphereOverlapAnyData		(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintBooleanHit* hits, const Point& color)			= 0;
		virtual	void				DrawSphereOverlapObjectsData	(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintOverlapObjectHit* hits, const Point& color)		= 0;
		virtual	void				DrawBoxOverlapAnyData			(udword nb, const PintBoxOverlapData* box_overlap_data, const PintBooleanHit* hits, const Point& color)					= 0;
		virtual	void				DrawBoxOverlapObjectsData		(udword nb, const PintBoxOverlapData* box_overlap_data, const PintOverlapObjectHit* hits, const Point& color)			= 0;
		virtual	void				DrawCapsuleOverlapAnyData		(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintBooleanHit* hits, const Point& color)			= 0;
		virtual	void				DrawCapsuleOverlapObjectsData	(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintOverlapObjectHit* hits, const Point& color)	= 0;
	};

	enum PintFlag
	{
		PINT_IS_ACTIVE				= (1<<0),
		PINT_HAS_RAYTRACING_WINDOW	= (1<<1),
		PINT_DEFAULT				= PINT_IS_ACTIVE|PINT_HAS_RAYTRACING_WINDOW,
	};

	class Pint : public Allocateable	// PINT = Physics INTerface
	{
		public:
									Pint() : mOMHelper(null), mSQHelper(null), mUserData(null)																{}
		virtual						~Pint()																													{}

		inline_ void				NotImplemented(const char* name)
		{
			printf("%s: non-implemented function %s has been called. Test may behave incorrectly.\n", GetName(), name);
		}

		virtual	const char*			GetName()				const																							= 0;
		virtual	void				GetCaps(PintCaps& caps)	const																							= 0;
		virtual	udword				GetFlags()				const																							{ return PINT_DEFAULT;	}
		virtual	void				Init(const PINT_WORLD_CREATE& desc)																						= 0;
		virtual	void				SetGravity(const Point& gravity)																						= 0;
		virtual	void				Close()																													= 0;
		virtual	udword				Update(float dt)																										= 0;
		virtual	void				UpdateNonProfiled(float dt)																								{}
		virtual	Point				GetMainColor()																											= 0;
		virtual	void				Render(PintRender& renderer)																							= 0;

		virtual	void				SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)													= 0;
		virtual	PintObjectHandle	CreateObject(const PINT_OBJECT_CREATE& desc)																			= 0;
		virtual	bool				ReleaseObject(PintObjectHandle handle)																					= 0;
		virtual	PintJointHandle		CreateJoint(const PINT_JOINT_CREATE& desc)																				= 0;

		virtual	void*				CreatePhantom(const AABB& box)																												{ NotImplemented("CreatePhantom");			return null;}
		virtual	udword				BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**)												{ NotImplemented("BatchRaycastsPhantom");	return 0;	}

		// Raycasts
		virtual	udword				BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)					{ NotImplemented("BatchRaycasts");		return 0;	}
		virtual	udword				BatchRaycastAny				(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)					{ NotImplemented("BatchRaycastAny");	return 0;	}
		virtual	udword				BatchRaycastAll				(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintRaycastData* raycasts)			{ NotImplemented("BatchRaycastAll");	return 0;	}
		// Sweeps
		virtual	udword				BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)					{ NotImplemented("BatchBoxSweeps");		return 0;	}
		virtual	udword				BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)				{ NotImplemented("BatchSphereSweeps");	return 0;	}
		virtual	udword				BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)				{ NotImplemented("BatchCapsuleSweeps");	return 0;	}
		// Overlaps
		virtual	udword				BatchSphereOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)			{ NotImplemented("BatchSphereOverlapAny");		return 0;	}
		virtual	udword				BatchSphereOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)		{ NotImplemented("BatchSphereOverlapObjects");	return 0;	}
		virtual	udword				BatchBoxOverlapAny			(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)				{ NotImplemented("BatchBoxOverlapAny");			return 0;	}
		virtual	udword				BatchBoxOverlapObjects		(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)		{ NotImplemented("BatchBoxOverlapObjects");		return 0;	}
		virtual	udword				BatchCapsuleOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)			{ NotImplemented("BatchCapsuleOverlapAny");		return 0;	}
		virtual	udword				BatchCapsuleOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)	{ NotImplemented("BatchCapsuleOverlapObjects"); return 0;	}
		//
		virtual	udword				FindTriangles_MeshSphereOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)	{ NotImplemented("FindTriangles_MeshSphereOverlap");	return 0;	}
		virtual	udword				FindTriangles_MeshBoxOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps)		{ NotImplemented("FindTriangles_MeshBoxOverlap");		return 0;	}
		virtual	udword				FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps)	{ NotImplemented("FindTriangles_MeshCapsuleOverlap");	return 0;	}

		virtual	PR					GetWorldTransform(PintObjectHandle handle)																				{ NotImplemented("GetWorldTransform");	PR Idt; Idt.Identity();	return Idt;	}
		virtual	void				SetWorldTransform(PintObjectHandle handle, const PR& pose)																{ NotImplemented("SetWorldTransform");	}

		// Deprecated
//		virtual	void				ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)			{ NotImplemented("ApplyActionAtPoint");	}
		virtual	void				AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos)					{ NotImplemented("AddWorldImpulseAtWorldPos");	}
		virtual	void				AddLocalTorque(PintObjectHandle handle, const Point& local_torque)														{ NotImplemented("AddLocalTorque");	}

		virtual	Point				GetAngularVelocity(PintObjectHandle handle)																				{ NotImplemented("GetAngularVelocity");	return Point(0.0f, 0.0f, 0.0f);	}
		virtual	void				SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity)												{ NotImplemented("SetAngularVelocity");	}

		virtual	float				GetMass(PintObjectHandle handle)																						{ NotImplemented("GetMass");			return 0.0f;	}
		virtual	Point				GetLocalInertia(PintObjectHandle handle)																				{ NotImplemented("GetLocalInertia");	return Point(0.0f, 0.0f, 0.0f);	}

		virtual	udword				GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)															{ NotImplemented("GetShapes");			return 0; }
		virtual	void				SetLocalRot(PintObjectHandle handle, const Quat& q)																		{ NotImplemented("SetLocalRot");	}

		virtual	bool				SetKinematicPose(PintObjectHandle handle, const Point& pos)																{ NotImplemented("SetKinematicPose");	return false;	}
		virtual	bool				SetKinematicPose(PintObjectHandle handle, const PR& pr)																	{ NotImplemented("SetKinematicPose");	return false;	}

		// Creates/releases an optional per-thread structure (e.g. caches) for scene queries.
		virtual	PintSQThreadContext	CreateSQThreadContext()																									{ return null;	}
		virtual	void				ReleaseSQThreadContext(PintSQThreadContext)																				{}

		// Experimental convex sweep support
		// So there is a design issue here. For simpler shapes we don't need per-plugin data in the sweeps (they can all share the same data)
		// but for convexes we need to create per-plugin convex objects, and the convex sweep data becomes engine dependent. The following methods
		// try to make this work with the existing infrastructure.
		virtual	udword				CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)																	{ NotImplemented("CreateConvexObject");	return INVALID_ID;	}
		virtual	udword				BatchConvexSweeps	(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)	{ NotImplemented("BatchConvexSweeps");	return 0;	}

		// Aggregates - currently based on the PhysX API, may change later to support the equivalent in other libs.
		virtual	PintObjectHandle	CreateAggregate(udword max_size, bool enable_self_collision)															{ NotImplemented("CreateAggregate");	return null;	}
		virtual	bool				AddToAggregate(PintObjectHandle object, PintObjectHandle aggregate)														{ NotImplemented("AddToAggregate");	return false;	}
		virtual	bool				AddAggregateToScene(PintObjectHandle aggregate)																			{ NotImplemented("AddAggregateToScene");	return false;	}

		// Articulations - currently based on the PhysX API, may change later to support the equivalent in other libs.
		virtual	PintObjectHandle	CreateArticulation(const PINT_ARTICULATION_CREATE&)																		{ NotImplemented("CreateArticulation");	return null;	}
		virtual	PintObjectHandle	CreateArticulatedObject(const PINT_OBJECT_CREATE&, const PINT_ARTICULATED_BODY_CREATE&, PintObjectHandle articulation)	{ NotImplemented("CreateArticulatedObject");	return null;	}
		virtual	bool				AddArticulationToScene(PintObjectHandle articulation)																	{ NotImplemented("AddArticulationToScene");	return false;	}
		virtual	void				SetArticulatedMotor(PintObjectHandle object, const PINT_ARTICULATED_MOTOR_CREATE& motor)								{ NotImplemented("SetArticulatedMotor");	}

		// Vehicles - WIP
		virtual	PintObjectHandle	CreateVehicle(PintVehicleData& data, const PINT_VEHICLE_CREATE& vehicle)												{ NotImplemented("CreateVehicle");	return null;	}
		virtual	void				SetVehicleInput(PintObjectHandle vehicle, const PINT_VEHICLE_INPUT& input)												{ NotImplemented("SetVehicleInput");	}

		virtual	void				TestNewFeature()																										{}

				ObjectsManager*		mOMHelper;
				PintSQ*				mSQHelper;
				void*				mUserData;
	};

	class PintPlugin : public Allocateable
	{
		public:
									PintPlugin()										{}
		virtual						~PintPlugin()										{}

		virtual	IceWindow*			InitGUI(IceWidget* parent, PintGUIHelper& helper)	= 0;
		virtual	void				CloseGUI()											= 0;
		virtual	void				Init(const PINT_WORLD_CREATE& desc)					= 0;
		virtual	void				Close()												= 0;
		virtual	Pint*				GetPint()											= 0;
	};

#endif
