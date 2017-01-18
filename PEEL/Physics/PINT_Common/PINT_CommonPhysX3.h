///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_COMMON_PHYSX3_H
#define PINT_COMMON_PHYSX3_H

#include "..\Pint.h"

	#define SAFE_RELEASE(x)	if(x) { x->release(); x = null; }

	inline_ Point	ToPoint(const PxVec3& p)	{ return Point(p.x, p.y, p.z);				}
	inline_ Quat	ToQuat(const PxQuat& q)		{ return Quat(q.w, q.x, q.y, q.z);			}
	inline_ PxVec3	ToPxVec3(const Point& p)	{ return PxVec3(p.x, p.y, p.z);				}
	inline_ PxQuat	ToPxQuat(const Quat& q)		{ return PxQuat(q.p.x, q.p.y, q.p.z, q.w);	}

	inline_	PintObjectHandle CreateHandle(PxRigidActor* actor)
	{
		const size_t binary = size_t(actor);
		ASSERT(!(binary&1));
		return PintObjectHandle(binary);
	}

	inline_	PintObjectHandle CreateHandle(PxShape* shape)
	{
		const size_t binary = size_t(shape);
		ASSERT(!(binary&1));
		return PintObjectHandle(binary|1);
	}

	inline_	PxRigidActor* GetActorFromHandle(PintObjectHandle handle)
	{
		const size_t binary = size_t(handle);
		return (binary & 1) ? null : (PxRigidActor*)binary;
	}

	inline_	PxShape* GetShapeFromHandle(PintObjectHandle handle)
	{
		const size_t binary = size_t(handle);
		return (binary & 1) ? (PxShape*)(binary&~1) : null;
	}

	class PEEL_PhysX3_ErrorCallback : public PxErrorCallback
	{
	public:
				PEEL_PhysX3_ErrorCallback()		{}
		virtual	~PEEL_PhysX3_ErrorCallback()	{}

		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line);
	};

	class PEEL_PhysX3_AllocatorCallback : public PxAllocatorCallback
	{
	public:
		struct Header
		{
			udword		mMagic;
			udword		mSize;
			const char*	mType;
			const char*	mFilename;
			udword		mLine;
		};

						PEEL_PhysX3_AllocatorCallback();
		virtual			~PEEL_PhysX3_AllocatorCallback();

		virtual	void*	allocate(size_t size, const char* typeName, const char* filename, int line);
		virtual	void	deallocate(void* ptr);

				udword	mTotalNbAllocs;
				udword	mNbAllocs;
				udword	mCurrentMemory;
				bool	mLog;
	};

	class MemoryOutputStream : public PxOutputStream
	{
	public:
						MemoryOutputStream(PEEL_PhysX3_AllocatorCallback* allocator=null);
	virtual				~MemoryOutputStream();

			PxU32		write(const void* src, PxU32 count);

			PxU32		getSize()	const	{	return mSize; }
			PxU8*		getData()	const	{	return mData; }
	private:
			PEEL_PhysX3_AllocatorCallback*	mCallback;
			PxU8*		mData;
			PxU32		mSize;
			PxU32		mCapacity;
	};

	class MemoryInputData : public PxInputData
	{
	public:
						MemoryInputData(PxU8* data, PxU32 length);

			PxU32		read(void* dest, PxU32 count);
			PxU32		getLength() const;
			void		seek(PxU32 pos);
			PxU32		tell() const;

	private:
			PxU32		mSize;
			const PxU8*	mData;
			PxU32		mPos;
	};

	namespace PhysX3
	{
		void			ComputeCapsuleTransform(PR& dst, const PR& src);
		void			SetGroup(PxShape& shape, PxU16 collision_group);
		PxRigidBody*	GetRigidBody(PintObjectHandle handle);
	}

	struct EditableParams
	{
										EditableParams();
		// Main
		udword							mNbThreads;
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
		udword							mScratchSize;
#endif
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
		PxBroadPhaseType::Enum			mBroadPhaseType;
		udword							mMBPSubdivLevel;
		float							mMBPRange;
#endif
		bool							mUseCCD;
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
		bool							mUseAngularCCD;
#endif
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
		bool							mUseRaycastCCD;
		bool							mUseRaycastCCD_DynaDyna;
#endif
		bool							mShareMeshData;
		bool							mShareShapes;
#ifdef PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
		bool							mUseTightConvexBounds;
#endif
		bool							mPCM;
#ifdef PHYSX_SUPPORT_SSE_FLAG
		bool							mEnableSSE;
#endif
		bool							mEnableActiveTransforms;
		bool							mEnableContactCache;
		bool							mFlushSimulation;
		bool							mUsePVD;
		bool							mUseFullPvdConnection;
#ifdef PHYSX_SUPPORT_GPU
		bool							mUseGPU;
#endif
		//float							mGlobalBoxSize;
		float							mDefaultFriction;
		float							mContactOffset;
		float							mRestOffset;
#ifdef PHYSX_SUPPORT_SUBSTEPS
		udword							mNbSubsteps;
#endif

		// Dynamics
		bool							mEnableSleeping;
		bool							mDisableStrongFriction;
		bool							mEnableOneDirFriction;
		bool							mEnableTwoDirFriction;
		bool							mAdaptiveForce;
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
		bool							mStabilization;
#endif
#ifndef IS_PHYSX_3_2
		udword							mMaxNbCCDPasses;
#endif
		udword							mSolverIterationCountPos;
		udword							mSolverIterationCountVel;
		float							mLinearDamping;
		float							mAngularDamping;
		float							mMaxAngularVelocity;
#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
		float							mMaxDepenVelocity;
#endif
		float							mSleepThreshold;

		// Scene queries
		PxPruningStructureType::Enum	mStaticPruner;
		PxPruningStructureType::Enum	mDynamicPruner;
		udword							mSQDynamicRebuildRateHint;
		bool							mSQFlag;
		bool							mSQFilterOutAllShapes;
		bool							mSQInitialOverlap;
		//bool							mSQManualFlushUpdates;
		bool							mSQPreciseSweeps;

		// Joints
		bool							mEnableJointProjection;
		bool							mUseD6Joint;
#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
		bool							mDisablePreprocessing;
#endif
#ifndef IS_PHYSX_3_2
	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
		bool							mEnableJoint32Compatibility;
	#endif
#endif
		float							mProjectionLinearTolerance;
		float							mProjectionAngularTolerance;
#ifndef IS_PHYSX_3_2
		float							mInverseInertiaScale;
		float							mInverseMassScale;
#endif
#ifdef PHYSX_SUPPORT_ARTICULATIONS
		// Articulations
		bool							mDisableArticulations;
		udword							mMaxProjectionIterations;
		float							mSeparationTolerance;
		udword							mExternalDriveIterations;
		udword							mInternalDriveIterations;
#endif
		// Cooking
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
		PxMeshMidPhase::Enum			mMidPhaseType;
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
		PxMeshCookingHint::Enum			mMeshCookingHint;
#endif
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
		udword							mGaussMapLimit;
#endif
#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
		bool							mPrecomputeActiveEdges;
#endif
		bool							mLast;
	};

	inline_	void	SetupShape(const EditableParams& params, const PINT_SHAPE_CREATE* create, PxShape& shape, PxU16 collision_group, bool debug_viz_flag)
	{
//		shape.setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		shape.setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, params.mSQFlag);
		shape.setFlag(PxShapeFlag::eVISUALIZATION, debug_viz_flag);
//		shape.setFlag(PxShapeFlag::eUSE_SWEPT_BOUNDS, gUseCCD);
		shape.setContactOffset(params.mContactOffset);
		shape.setRestOffset(params.mRestOffset);
//		const float contactOffset = shape.getContactOffset();	// 0.02
//		const float restOffset = shape.getRestOffset();		// 0.0
//		printf("contactOffset: %f\n", contactOffset);
//		printf("restOffset: %f\n", restOffset);

		// Setup query filter data so that we can filter out all shapes - debug purpose
		if(params.mSQFlag)
			shape.setQueryFilterData(PxFilterData(1, 0, 0, 0));

		if(create->mRenderer)
			shape.userData = create->mRenderer;

		PhysX3::SetGroup(shape, collision_group);
	}

	class SharedPhysX : public Pint
	{
		public:
											SharedPhysX(const EditableParams& params);
		virtual								~SharedPhysX();

		virtual	void						SetGravity(const Point& gravity);

		virtual	void						SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle			CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool						ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle				CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	PR							GetWorldTransform(PintObjectHandle handle);
		virtual	void						SetWorldTransform(PintObjectHandle handle, const PR& pose);

//		virtual	void						ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void						AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
		virtual	void						AddLocalTorque(PintObjectHandle handle, const Point& local_torque);

		virtual	Point						GetAngularVelocity(PintObjectHandle handle);
		virtual	void						SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity);

		virtual	float						GetMass(PintObjectHandle handle);
		virtual	Point						GetLocalInertia(PintObjectHandle handle);

		virtual	udword						GetShapes(PintObjectHandle* shapes, PintObjectHandle handle);
		virtual	void						SetLocalRot(PintObjectHandle handle, const Quat& q);

		virtual	bool						SetKinematicPose(PintObjectHandle handle, const Point& pos);
		virtual	bool						SetKinematicPose(PintObjectHandle handle, const PR& pr);

		virtual	udword						CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc);

		virtual	PintObjectHandle			CreateAggregate(udword max_size, bool enable_self_collision);
		virtual	bool						AddToAggregate(PintObjectHandle object, PintObjectHandle aggregate);
		virtual	bool						AddAggregateToScene(PintObjectHandle aggregate);

		virtual	PintObjectHandle			CreateArticulation(const PINT_ARTICULATION_CREATE&);
		virtual	PintObjectHandle			CreateArticulatedObject(const PINT_OBJECT_CREATE&, const PINT_ARTICULATED_BODY_CREATE&, PintObjectHandle articulation);
		virtual	bool						AddArticulationToScene(PintObjectHandle articulation);
		virtual	void						SetArticulatedMotor(PintObjectHandle object, const PINT_ARTICULATED_MOTOR_CREATE& motor);

		virtual	udword						BatchRaycastAny				(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts);
		virtual	udword						BatchSphereOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword						BatchBoxOverlapAny			(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps);
		virtual	udword						BatchCapsuleOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps);

		virtual	udword						BatchSphereOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword						BatchBoxOverlapObjects		(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps);
		virtual	udword						BatchCapsuleOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps);

		virtual	udword						FindTriangles_MeshSphereOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps);
		virtual	udword						FindTriangles_MeshBoxOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps);
		virtual	udword						FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps);

				PintObjectHandle			CreateArticulationLink(PxArticulation* articulation, PxArticulationLink* parent, Pint& pint, const PINT_OBJECT_CREATE& desc);
		virtual	void						CreateShapes		(const PINT_OBJECT_CREATE& desc, PxRigidActor* actor){}

		virtual	void						Render(PintRender& renderer);

		protected:
				PxFoundation*				mFoundation;
				PxPhysics*					mPhysics;
				PxScene*					mScene;
				PxCooking*					mCooking;
				PxMaterial*					mDefaultMaterial;
				std::vector<PxConvexMesh*>	mConvexObjects;

				const EditableParams&		mParams;

#ifndef IS_PHYSX_3_2
				void						CreateCooking(const PxTolerancesScale& scale, PxMeshPreprocessingFlags mesh_preprocess_params);
#endif
				PxMaterial*					CreateMaterial(const PINT_MATERIAL_CREATE& desc);
				PxConvexMesh*				CreateConvexMesh(const Point* verts, udword vertCount, PxConvexFlags flags, PintShapeRenderer* renderer);
				PxTriangleMesh*				CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer);

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
		inline_	void*						GetScratchPad()				{ return mScratchPad;		}
		inline_	udword						GetScratchPadSize()	const	{ return mScratchPadSize;	}
#else
		inline_	void*						GetScratchPad()				{ return null;				}
		inline_	udword						GetScratchPadSize()	const	{ return 0;					}
#endif
				void						SetupDynamic(PxRigidDynamic& rigidDynamic, const PINT_OBJECT_CREATE& desc);
				void						SetupArticulationLink(PxArticulationLink& link, const PINT_OBJECT_CREATE& desc);
				void						InitCommon();
				void						CloseCommon();
				void						UpdateCommon(float dt);

		inline_	PxQueryFilterData			GetSQFilterData()
											{
												return PxQueryFilterData(PxFilterData(!mParams.mSQFilterOutAllShapes, mParams.mSQFilterOutAllShapes, 0, 0), PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC);
											}
		private:
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
				void*						mScratchPad;
				udword						mScratchPadSize;
#endif
				struct ConvexRender
				{
					ConvexRender(PxConvexMesh* convexMesh, PintShapeRenderer* renderer) :
						mConvexMesh	(convexMesh),
						mRenderer	(renderer)
					{
					}
					PxConvexMesh*			mConvexMesh;
					PintShapeRenderer*		mRenderer;
				};

				struct MeshRender
				{
					MeshRender(PxTriangleMesh* triMesh, PintShapeRenderer* renderer) :
						mTriangleMesh	(triMesh),
						mRenderer		(renderer)
					{
					}
					PxTriangleMesh*			mTriangleMesh;
					PintShapeRenderer*		mRenderer;
				};

				std::vector<PxMaterial*>	mMaterials;
				std::vector<ConvexRender>	mConvexes;
				std::vector<MeshRender>		mMeshes;

				struct LocalTorque
				{
					LocalTorque(PintObjectHandle handle, const Point& local_torque) :
						mHandle		(handle),
						mLocalTorque(local_torque)
					{
					}
					PintObjectHandle		mHandle;
					Point					mLocalTorque;
				};
				std::vector<LocalTorque>	mLocalTorques;
	};

	template<class T>
	inline_ void SetupSleeping(T* dynamic, bool enable_sleeping)
	{
#ifdef IS_PHYSX_3_2
		if(!enable_sleeping)
			dynamic->wakeUp(9999999999.0f);
#else
		dynamic->wakeUp();
		if(!enable_sleeping)
			dynamic->setWakeCounter(9999999999.0f);
#endif
	}

	inline_ void SetSceneFlag(PxSceneDesc& desc, PxSceneFlag::Enum flag, bool b)
	{
		if(b)
			desc.flags	|= flag;
		else
			desc.flags	&= ~flag;
	}

	class UICallback
	{
		public:
		virtual	void			UIModificationCallback()	= 0;
	};

	namespace PhysX3
	{
		IceWindow*				InitSharedGUI(IceWidget* parent, PintGUIHelper& helper, UICallback& callback, udword nb_debug_viz_params, bool* debug_viz_params, const char** debug_viz_names);
		const EditableParams&	GetEditableParams();
		void					GetOptionsFromGUI(const char* test_name);
		void					CloseSharedGUI();
	}

#endif