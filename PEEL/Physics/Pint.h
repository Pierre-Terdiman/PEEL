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

		Point	mPos;
		Point	mDir;
	};

	//! Contains scene-related parameters. This is used to initialize each PINT engine, *before* the test itself is setup.
	struct PINT_WORLD_CREATE : public Allocateable
	{
									PINT_WORLD_CREATE()
									{
										mGlobalBounds.SetEmpty();
										mGravity.Zero();
									}

		AABB						mGlobalBounds;
		CameraPose					mCamera[PINT_MAX_CAMERA_POSES];
		Point						mGravity;
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
									PINT_SPHERE_CREATE() :
										mRadius	(0.0f)
									{
										mType	= PINT_SHAPE_SPHERE;
									}

		float						mRadius;
	};

	struct PINT_CAPSULE_CREATE : PINT_SHAPE_CREATE
	{
									PINT_CAPSULE_CREATE() :
										mRadius		(0.0f),
										mHalfHeight	(0.0f)
									{
										mType	= PINT_SHAPE_CAPSULE;
									}

		float						mRadius;
		float						mHalfHeight;
	};

	struct PINT_BOX_CREATE : PINT_SHAPE_CREATE
	{
									PINT_BOX_CREATE()
									{
										mType	= PINT_SHAPE_BOX;
										mExtents.Zero();
									}

		Point						mExtents;
	};

	struct PINT_CONVEX_DATA_CREATE : public Allocateable
	{
									PINT_CONVEX_DATA_CREATE() :
										mNbVerts	(0),
										mVerts		(null),
										mRenderer	(null)
									{
									}

		udword						mNbVerts;
		const Point*				mVerts;
		PintShapeRenderer*			mRenderer;
	};

	struct PINT_CONVEX_CREATE : PINT_SHAPE_CREATE
	{
									PINT_CONVEX_CREATE() :
										mNbVerts	(0),
										mVerts		(null)
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
										mCollisionGroup	(0),
										mKinematic		(false),
										mAddToWorld		(true)
									{
										mPosition.Zero();
										mRotation.Identity();
										mLinearVelocity.Zero();
										mAngularVelocity.Zero();
									}

		const PINT_SHAPE_CREATE*	mShapes;
		Point						mPosition;
		Quat						mRotation;
		Point						mLinearVelocity;
		Point						mAngularVelocity;
		float						mMass;
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

	struct PINT_JOINT_CREATE	: public Allocateable
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

	struct PINT_SPHERICAL_JOINT_CREATE	: PINT_JOINT_CREATE
	{
									PINT_SPHERICAL_JOINT_CREATE()
									{
										mType = PINT_JOINT_SPHERICAL;
										mLocalPivot0.Zero();
										mLocalPivot1.Zero();
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
	};

	struct PINT_HINGE_JOINT_CREATE	: PINT_JOINT_CREATE
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
										mGlobalAnchor.SetNotUsed();
										mGlobalAxis.SetNotUsed();
									}

		Point						mLocalPivot0;
		Point						mLocalPivot1;
		Point						mLocalAxis0;
		Point						mLocalAxis1;
		float						mMinLimitAngle;
		float						mMaxLimitAngle;
		Point						mGlobalAnchor;
		Point						mGlobalAxis;
	};

	struct PINT_PRISMATIC_JOINT_CREATE	: PINT_JOINT_CREATE
	{
									PINT_PRISMATIC_JOINT_CREATE()
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
	};

	struct PINT_FIXED_JOINT_CREATE	: PINT_JOINT_CREATE
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

	struct PintCaps : public Allocateable
	{
				PintCaps();

		bool	mSupportRigidBodySimulation;
		bool	mSupportKinematics;
		bool	mSupportCollisionGroups;
		bool	mSupportCompounds;
		bool	mSupportConvexes;
		bool	mSupportMeshes;
		//
		bool	mSupportSphericalJoints;
		bool	mSupportHingeJoints;
		bool	mSupportFixedJoints;
		bool	mSupportPrismaticJoints;
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
									PintRender()									{}
		virtual						~PintRender()									{}

		virtual	void				DrawLine						(const Point& p0, const Point& p1, const Point& color)																	= 0;
		virtual	void				DrawTriangle					(const Point& p0, const Point& p1, const Point& p2, const Point& color)													= 0;
		virtual	void				DrawWirefameAABB				(const AABB& box, const Point& color)																					= 0;
		virtual	void				DrawWirefameOBB					(const OBB& box, const Point& color)																					= 0;
		virtual	void				DrawSphere						(float radius, const PR& pose)																							= 0;
		virtual	void				DrawBox							(const Point& extents, const PR& pose)																					= 0;
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
		virtual							~Pint()																													{}

		virtual	const char*				GetName()				const																							= 0;
		virtual	void					GetCaps(PintCaps& caps)	const																							= 0;
		virtual	void					Init(const PINT_WORLD_CREATE& desc)																						= 0;
		virtual	void					SetGravity(const Point& gravity)																						= 0;
		virtual	void					Close()																													= 0;
		virtual	udword					Update(float dt)																										= 0;
		virtual	void					UpdateNonProfiled(float dt)																								{}
		virtual	Point					GetMainColor()																											= 0;
		virtual	void					Render(PintRender& renderer)																							= 0;

		virtual	void					SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)													= 0;
		virtual	PintObjectHandle		CreateObject(const PINT_OBJECT_CREATE& desc)																			= 0;
		virtual	bool					ReleaseObject(PintObjectHandle handle)																					= 0;
		virtual	PintJointHandle			CreateJoint(const PINT_JOINT_CREATE& desc)																				= 0;

		virtual	void*					CreatePhantom(const AABB& box)																							{ return null;	}
		virtual	udword					BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**)							{ return 0;	}

		// Raycasts
		virtual	udword					BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)					{ return 0;	}
		virtual	udword					BatchRaycastAny				(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)					{ return 0;	}
		virtual	udword					BatchRaycastAll				(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintRaycastData* raycasts)			{ return 0;	}
		// Sweeps
		virtual	udword					BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)					{ return 0;	}
		virtual	udword					BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)				{ return 0;	}
		virtual	udword					BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)				{ return 0;	}
		// Overlaps
		virtual	udword					BatchSphereOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)			{ return 0;	}
		virtual	udword					BatchSphereOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)		{ return 0;	}
		virtual	udword					BatchBoxOverlapAny			(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)				{ return 0;	}
		virtual	udword					BatchBoxOverlapObjects		(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)		{ return 0;	}
		virtual	udword					BatchCapsuleOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)			{ return 0;	}
		virtual	udword					BatchCapsuleOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)	{ return 0;	}
		//
		virtual	udword					FindTriangles_MeshSphereOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)	{ return 0;	}
		virtual	udword					FindTriangles_MeshBoxOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps)		{ return 0;	}
		virtual	udword					FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps)	{ return 0;	}

		virtual	PR						GetWorldTransform(PintObjectHandle handle)																				= 0;
		virtual	void					ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)			= 0;

		virtual	void					TestNewFeature()																										{}

		// XP
		virtual	udword					GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)															{ return 0; }
		virtual	void					SetLocalRot(PintObjectHandle handle, const Quat& q)																		{}
		virtual	bool					GetConvexData(PintObjectHandle handle, PINT_CONVEX_CREATE& data)														{ return false;	}

		virtual	bool					SetKinematicPose(PintObjectHandle handle, const Point& pos)																{ return false;	}
		virtual	udword					GetFlags()				const																							{ return PINT_DEFAULT;	}

		virtual	PintSQThreadContext		CreateSQThreadContext()																									{ return null;	}
		virtual	void					ReleaseSQThreadContext(PintSQThreadContext)																				{				}

		// Experimental convex sweep support
		// So there is a design issue here. For simpler shapes we don't need per-plugin data in the sweeps (they can all share the same data)
		// but for convexes we need to create per-plugin convex objects, and the convex sweep data becomes engine dependent. The following methods
		// try to make this work with the existing infrastructure.
		virtual	udword					CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)																	{ return INVALID_ID;	}
		virtual	udword					BatchConvexSweeps	(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)	{ return 0;	}

				ObjectsManager*			mOMHelper;
				PintSQ*					mSQHelper;
				void*					mUserData;
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
