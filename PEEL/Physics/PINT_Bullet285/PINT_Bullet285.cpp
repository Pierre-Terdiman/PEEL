///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "..\PINT_Common\PINT_Common.h"
#include "..\PINT_Common\PINT_CommonBullet.h"

/*#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"*/

// Bullet3-2.85.1. For this one I just generated the VS2010 projects using Bullet's provided script, and compiled them as-is.

///////////////////////////////////////////////////////////////////////////////

/*
		m_damping = btScalar(1.0);
		m_friction = btScalar(0.3);
		m_timeStep = btScalar(1.f/60.f);
		m_restitution = btScalar(0.);
		m_maxErrorReduction = btScalar(20.);
		m_globalCfm = btScalar(0.);
		m_sor = btScalar(1.);
		m_splitImpulsePenetrationThreshold = -.04f;
		m_splitImpulseTurnErp = 0.1f;
		m_linearSlop = btScalar(0.0);
		m_warmstartingFactor=btScalar(0.85);
		//m_solverMode =  SOLVER_USE_WARMSTARTING |  SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;// | SOLVER_RANDMIZE_ORDER;
		m_solverMode = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
		m_restingContactRestitutionThreshold = 2;//unused as of 2.81
		m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
		m_maxGyroscopicForce = 100.f; ///it is only used for 'explicit' version of gyroscopic force
		m_singleAxisRollingFrictionThreshold = 1e30f;///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
*/

//static	const	bool	gEnableCollisionBetweenJointed	= true;
static	const	float	gGlobalBoxSize					= 10000.0f;
static			float	gLinearDamping					= 0.1f;
static			float	gAngularDamping					= 0.05f;
// Default ERP value (0.2) worked fine for box stacks up to Bullet 2.82 but made the jointed objects act very soft. A higher ERP improved the joints
// but made the box stacks behave very badly (go try ERP=0.8 in Bullet 2.82 on LargeBoxStack30 for example). On the other hand this is not the case
// anymore in Bullet 2.85, which is great because we can now set ERP = 0.8 (for example), which improves joints scenes like FixedJointsTorusStressTest.
//static			float	gErp							= 0.2f;		// Controls btContactSolverInfoData::m_erp. Default value from btContactSolverInfo()
static			float	gErp							= 0.8f;		// Controls btContactSolverInfoData::m_erp.
static			float	gErp2							= 0.2f;		// Controls btContactSolverInfoData::m_erp2. Default value from btContactSolverInfo()
static			float	gTau							= 0.6f;		// Controls btContactSolverInfoData::m_tau. Default value from btContactSolverInfo()
static			udword	gSolverIterationCount			= 4;		// Controls btContactSolverInfoData::m_numIterations.
static			bool	gUseSplitImpulse				= true;		// Controls btContactSolverInfoData::m_splitImpulse. Default value from btContactSolverInfo()
static			float	gCollisionMargin				= 0.005f;
static			bool	gSolverRandomizeOrder			= false;	// Controls btSolverMode::SOLVER_RANDMIZE_ORDER
static			bool	gSolverFrictionSeparate			= false;	// Controls btSolverMode::SOLVER_FRICTION_SEPARATE
static			bool	gSolverWarmStarting				= true;		// Controls btSolverMode::SOLVER_USE_WARMSTARTING
static			bool	gSolverEnableFrictionDirCaching	= true;		// Controls btSolverMode::SOLVER_ENABLE_FRICTION_DIRECTION_CACHING
static			bool	gSolverUse2FrictionDirections	= false;	// Controls btSolverMode::SOLVER_USE_2_FRICTION_DIRECTIONS
static			bool	gSolverSIMD						= true;		// Controls btSolverMode::SOLVER_SIMD
static			bool	gSolverInterleaveConstraints	= false;	// Controls btSolverMode::SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
/*
	SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
	SOLVER_CACHE_FRIENDLY = 128,
	SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS = 512,
	SOLVER_ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS = 1024
*/
static			bool	gShareShapes					= true;
static			bool	gUseDbvt						= true;
static			bool	gUseCCD							= false;
static			bool	gEnableSleeping					= false;
static	const	bool	gUseCustomMemoryAllocator		= true;
static			bool	gUseInternalEdgeUtility			= true;
static			udword	gGyroscopicIndex				= 0;	// Index from gyroscopic force combo box
static			float	gDefaultFriction				= 0.5f;

#define	NB_DEBUG_VIZ_PARAMS	6
static			bool	gDebugVizParams[NB_DEBUG_VIZ_PARAMS] = {0};
static	const	char*	gDebugVizNames[NB_DEBUG_VIZ_PARAMS] =
{
	"Enable debug visualization",
	"Draw wireframe",
	"Draw AABBs",
	"Draw contact points",
	"Draw constraints",
	"Draw constraints limits",
};
static			int	gDebugVizMask[NB_DEBUG_VIZ_PARAMS] =
{
	btIDebugDraw::DBG_NoDebug,
	btIDebugDraw::DBG_DrawWireframe,
	btIDebugDraw::DBG_DrawAabb,
	btIDebugDraw::DBG_DrawContactPoints,
	btIDebugDraw::DBG_DrawConstraints,
	btIDebugDraw::DBG_DrawConstraintLimits,
};

/*
		DBG_DrawWireframe = 1,
		DBG_DrawAabb=2,
		DBG_DrawFeaturesText=4,
		DBG_DrawContactPoints=8,
		DBG_NoDeactivation=16,
		DBG_NoHelpText = 32,
		DBG_DrawText=64,
		DBG_ProfileTimings = 128,
		DBG_EnableSatComparison = 256,
		DBG_DisableBulletLCP = 512,
		DBG_EnableCCD = 1024,
		DBG_DrawConstraints = (1 << 11),
		DBG_DrawConstraintLimits = (1 << 12),
		DBG_FastWireframe = (1<<13),
		DBG_MAX_DEBUG_DRAW_MODE
*/

///////////////////////////////////////////////////////////////////////////////

static udword	gNbAllocs = 0;
static udword	gCurrentMemory = 0;

	struct Header
	{
		udword	mMagic;
		udword	mSize;
	};

static void* __btAlignedAllocFunc(size_t size, int alignment)
{
	char* memory = (char*)_aligned_malloc(size+16, 16);
	Header* H = (Header*)memory;
	H->mMagic = 0x12345678;
	H->mSize = size;
	gNbAllocs++;
	gCurrentMemory+=size;
	return memory + 16;
}

static void __btAlignedFreeFunc(void* ptr)
{
	if(!ptr)
		return;
	char* bptr = (char*)ptr;
	Header* H = (Header*)(bptr - 16);
	ASSERT(H->mMagic==0x12345678);
	const udword Size = H->mSize;
	_aligned_free(H);
	gNbAllocs--;
	gCurrentMemory-=Size;
}

static void* __btAllocFunc(size_t size)
{
	char* memory = (char*)_aligned_malloc(size+16, 16);
	Header* H = (Header*)memory;
	H->mMagic = 0x12345678;
	H->mSize = size;
	gNbAllocs++;
	gCurrentMemory+=size;
	return memory + 16;
}

static void __btFreeFunc(void* ptr)
{
	if(!ptr)
		return;
	char* bptr = (char*)ptr;
	Header* H = (Header*)(bptr - 16);
	ASSERT(H->mMagic==0x12345678);
	const udword Size = H->mSize;
	_aligned_free(H);
	gNbAllocs--;
	gCurrentMemory-=Size;
}

///////////////////////////////////////////////////////////////////////////////

class MyDebugDrawer : public btIDebugDraw
{
	public:

	MyDebugDrawer() : mRenderer(null), mDebugMode(0)
	{
	}

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		ASSERT(mRenderer);
		mRenderer->DrawLine(ToPoint(from), ToPoint(to), ToPoint(color));
	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{
		ASSERT(mRenderer);
		mRenderer->DrawLine(ToPoint(PointOnB), ToPoint(PointOnB+normalOnB), ToPoint(color));
	}

	virtual void	reportErrorWarning(const char* warningString)
	{
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}

	virtual void	setDebugMode(int debugMode)
	{
		mDebugMode = debugMode;
	}

	virtual int		getDebugMode() const
	{
		return mDebugMode;
	}

	void updateDebugMode()
	{
		mDebugMode = 0;
		if(gDebugVizParams[0])
		{
			for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
			{
				if(gDebugVizParams[i])
					mDebugMode |= gDebugVizMask[i];
			}
		}
	}

	PintRender*	mRenderer;
	int			mDebugMode;

}gDebugDrawer;

///////////////////////////////////////////////////////////////////////////////

// Only used for gUseInternalEdgeUtility
static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
{
	if(gUseInternalEdgeUtility)
	{
		btAdjustInternalEdgeContacts(cp,colObj1Wrap,colObj0Wrap, partId1,index1);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	}

/*	float friction0 = colObj0Wrap->getCollisionObject()->getFriction();
	float friction1 = colObj1Wrap->getCollisionObject()->getFriction();
	float restitution0 = colObj0Wrap->getCollisionObject()->getRestitution();
	float restitution1 = colObj1Wrap->getCollisionObject()->getRestitution();

	if (colObj0Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		if (index1&1)
		{
			friction1 = 1.0f;//partId1,index1
		} else
		{
			friction1 = 0.f;
		}
		restitution1 = 0.f;
	}

	cp.m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);
*/
	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return false;
}

// Err, is that some global inside Bullet?
extern ContactAddedCallback gContactAddedCallback;

///////////////////////////////////////////////////////////////////////////////

static inline_	udword 	RemapCollisionGroup(udword group)	{ return group+2;	}

Bullet::Bullet() :
	mCollisionConfiguration	(null),
	mDispatcher				(null),
	mBroadPhase				(null),
	mSolver					(null),
	mDynamicsWorld			(null)
{
}

Bullet::~Bullet()
{
	ASSERT(!mCollisionConfiguration);
	ASSERT(!mDispatcher);
	ASSERT(!mBroadPhase);
	ASSERT(!mSolver);
	ASSERT(!mDynamicsWorld);
}

void Bullet::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportMassForInertia			= true;
	caps.mSupportCylinders				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= true;
	caps.mSupportSphereSweeps			= true;
	caps.mSupportCapsuleSweeps			= true;
	caps.mSupportConvexSweeps			= true;
}

void Bullet::Init(const PINT_WORLD_CREATE& desc)
{
	for(udword i=0;i<32;i++)
		mGroupMasks[i] = 0xffffffff;

	gNbAllocs = 0;
	gCurrentMemory = 0;
	if(gUseCustomMemoryAllocator)
	{
		btAlignedAllocSetCustomAligned(__btAlignedAllocFunc, __btAlignedFreeFunc);
		btAlignedAllocSetCustom(__btAllocFunc, __btFreeFunc);
	}

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConstructionInfo cci;
//	cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
	cci.m_defaultMaxPersistentManifoldPoolSize = 32;
	mCollisionConfiguration = new btDefaultCollisionConfiguration(cci);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);
//	mDispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	if(gUseDbvt)
	{
		mBroadPhase = new btDbvtBroadphase();
	}
	else
	{
		if(desc.mGlobalBounds.IsValid())
		{
			Point m, M;
			desc.mGlobalBounds.GetMin(m);
			desc.mGlobalBounds.GetMax(M);

			mBroadPhase = new btAxisSweep3(ToBtVector3(m), ToBtVector3(M));
		}
		else
		{
			mBroadPhase = new btAxisSweep3(btVector3(-gGlobalBoxSize, -gGlobalBoxSize, -gGlobalBoxSize), btVector3(gGlobalBoxSize, gGlobalBoxSize, gGlobalBoxSize));
		}
	}

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	mSolver = new btSequentialImpulseConstraintSolver;
//	mSolver = new btOdeQuickstepConstraintSolver;
/*
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		mSolver = new btMLCPSolver(mlcp);
*/

	mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadPhase, mSolver, mCollisionConfiguration);
	mDynamicsWorld->setGravity(ToBtVector3(desc.mGravity));

	btContactSolverInfo& SolverInfo = mDynamicsWorld->getSolverInfo();
	if(gSolverEnableFrictionDirCaching)
		SolverInfo.m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING; //don't recalculate friction values each frame
	else
		SolverInfo.m_solverMode &= ~SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
	if(gSolverUse2FrictionDirections)
		SolverInfo.m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;
	else
		SolverInfo.m_solverMode &= ~SOLVER_USE_2_FRICTION_DIRECTIONS;
	if(gSolverSIMD)
		SolverInfo.m_solverMode |= SOLVER_SIMD;
	else
		SolverInfo.m_solverMode &= ~SOLVER_SIMD;
	if(gSolverInterleaveConstraints)
		SolverInfo.m_solverMode |= SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
	else
		SolverInfo.m_solverMode &= ~SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
	if(gSolverRandomizeOrder)
		SolverInfo.m_solverMode |= SOLVER_RANDMIZE_ORDER;
	else
		SolverInfo.m_solverMode &= ~SOLVER_RANDMIZE_ORDER;
	if(gSolverFrictionSeparate)
		SolverInfo.m_solverMode |= SOLVER_FRICTION_SEPARATE;
	else
		SolverInfo.m_solverMode &= ~SOLVER_FRICTION_SEPARATE;
	if(gSolverWarmStarting)
		SolverInfo.m_solverMode |= SOLVER_USE_WARMSTARTING;
	else
		SolverInfo.m_solverMode &= ~SOLVER_USE_WARMSTARTING;
	SolverInfo.m_numIterations	= gSolverIterationCount;
//	SolverInfo.m_minimumSolverBatchSize
	SolverInfo.m_splitImpulse	= gUseSplitImpulse;
	SolverInfo.m_erp			= gErp;
	SolverInfo.m_erp2			= gErp2;
	SolverInfo.m_tau			= gTau;
//	SolverInfo.m_friction		= 1.0f;
	
/*
		discreteWorld->getSolverInfo().m_linearSlop = gSlop;
		discreteWorld->getSolverInfo().m_warmstartingFactor = gWarmStartingParameter;
*/

//	SolverInfo.m_maxGyroscopicForce = 10.f;

	mDynamicsWorld->getDispatchInfo().m_useContinuous = gUseCCD;

	mDynamicsWorld->setDebugDrawer(&gDebugDrawer);

	mDynamicsWorld->setForceUpdateAllAabbs(false);

	gContactAddedCallback = CustomMaterialCombinerCallback;
}

void Bullet::SetGravity(const Point& gravity)
{
	ASSERT(mDynamicsWorld);
	mDynamicsWorld->setGravity(ToBtVector3(gravity));
}

void Bullet::Close()
{
	//cleanup in the reverse order of creation/initialization

	//delete constraints
	for(udword j=0;j<mConstraints.size();j++)
	{
		btTypedConstraint* c = mConstraints[j];
		mDynamicsWorld->removeConstraint(c);
		DELETESINGLE(c);
	}

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for(i=mDynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = mDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		mDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	for(udword j=0;j<mMeshShapes.size();j++)
	{
		InternalMeshShape& shape = mMeshShapes[j];
		DELETESINGLE(shape.mMeshData);
		DELETESINGLE(shape.mTriangleInfoMap);
	}

	//delete collision shapes
	for(udword j=0;j<mCollisionShapes.size();j++)
	{
		btCollisionShape* shape = mCollisionShapes[j];
		DELETESINGLE(shape);
	}

	for(udword j=0;j<mConvexObjects.size();j++)
	{
		btConvexHullShape* shape = mConvexObjects[j];
		DELETESINGLE(shape);
	}

	DELETESINGLE(mDynamicsWorld);
	DELETESINGLE(mSolver);
	DELETESINGLE(mBroadPhase);
	DELETESINGLE(mDispatcher);
	DELETESINGLE(mCollisionConfiguration);

	if(gNbAllocs)
		printf("Bullet 2.82: %d leaks found (%d bytes)\n", gNbAllocs, gCurrentMemory);
}

udword Bullet::Update(float dt)
{
	if(mDynamicsWorld)
	{
		mDynamicsWorld->stepSimulation(dt, 1, dt);
	}
	return gCurrentMemory;
}

static void DrawLeafShape(PintRender& renderer, const btCollisionShape* shape, const PR& pose)
{
	ASSERT(shape->getUserPointer());
	if(shape->getUserPointer())
	{
		PintShapeRenderer* psr = (PintShapeRenderer*)shape->getUserPointer();
		psr->Render(pose);
	}
/*	else
	{
		const int Type = shape->getShapeType();
		if(Type==SPHERE_SHAPE_PROXYTYPE)
		{
			const btSphereShape* SphereShape = static_cast<const btSphereShape*>(shape);
			const float Radius = SphereShape->getRadius();
			renderer.DrawSphere(Radius, pose);
		}
		else if(Type==BOX_SHAPE_PROXYTYPE)
		{
			const btBoxShape* BoxShape = static_cast<const btBoxShape*>(shape);
			const btVector3& Extents = BoxShape->getHalfExtentsWithoutMargin();
			renderer.DrawBox(ToPoint(Extents), pose);
		}
		else if(Type==CONVEX_HULL_SHAPE_PROXYTYPE)
		{
			const btConvexHullShape* ConvexShape = static_cast<const btConvexHullShape*>(shape);
			ASSERT(0);
		}
		else ASSERT(0);
	}*/
}

void Bullet::Render(PintRender& renderer)
{
	const int size = mDynamicsWorld->getNumCollisionObjects();
	for(int i=0;i<size;i++)
	{
		btCollisionObject* obj = mDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);

		btTransform trans;
		body->getMotionState()->getWorldTransform(trans);
		const PR BodyPose = ToPR(trans);

		const btCollisionShape* CurrentShape = body->getCollisionShape();
		if(CurrentShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
		{
			const btCompoundShape* CompoundShape = static_cast<const btCompoundShape*>(CurrentShape);
			const int NbShapes = CompoundShape->getNumChildShapes();
			for(int j=0;j<NbShapes;j++)
			{
				const btCollisionShape* ChildShape = CompoundShape->getChildShape(j);
				const btTransform& ChildTransform = CompoundShape->getChildTransform(j);

				const btTransform Combo = trans * ChildTransform;

				const PR ComboPose = ToPR(Combo);

				DrawLeafShape(renderer, ChildShape, ComboPose);
			}
		}
		else
		{
			DrawLeafShape(renderer, CurrentShape, BodyPose);
		}
	}

	gDebugDrawer.mRenderer = &renderer;
	mDynamicsWorld->debugDrawWorld();
}

//static inline_ void RegisterShape()

btSphereShape* Bullet::FindSphereShape(const PINT_SPHERE_CREATE& create)
{
	const float Radius = create.mRadius;
	if(gShareShapes)
	{
		const int size = mSphereShapes.size();
		for(int i=0;i<size;i++)
		{
			btSphereShape* CurrentShape = mSphereShapes[i];
			if(CurrentShape->getRadius()==Radius)
				return CurrentShape;
		}
	}

	btSphereShape* shape = new btSphereShape(create.mRadius);
	ASSERT(shape);
	mSphereShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);

	return shape;
}

btBoxShape* Bullet::FindBoxShape(const PINT_BOX_CREATE& create)
{
	if(gShareShapes)
	{
		const int size = mBoxShapes.size();
		for(int i=0;i<size;i++)
		{
			const InternalBoxShape& CurrentShape = mBoxShapes[i];
			if(		CurrentShape.mExtents.x==create.mExtents.x
				&&	CurrentShape.mExtents.y==create.mExtents.y
				&&	CurrentShape.mExtents.z==create.mExtents.z)
				return CurrentShape.mShape;
		}
	}

	btBoxShape* shape = new btBoxShape(ToBtVector3(create.mExtents));
	ASSERT(shape);
	InternalBoxShape Internal;
	Internal.mShape		= shape;
	Internal.mExtents	= create.mExtents;
	mBoxShapes.push_back(Internal);
	mCollisionShapes.push_back(shape);

	if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);

	return shape;
}

btCapsuleShape* Bullet::FindCapsuleShape(const PINT_CAPSULE_CREATE& create)
{
	const float Radius = create.mRadius;
	const float HalfHeight = create.mHalfHeight;
	if(gShareShapes)
	{
		const int size = mCapsuleShapes.size();
		for(int i=0;i<size;i++)
		{
			btCapsuleShape* CurrentShape = mCapsuleShapes[i];
			if(CurrentShape->getRadius()==Radius && CurrentShape->getHalfHeight()==HalfHeight)
				return CurrentShape;
		}
	}

	btCapsuleShape* shape = new btCapsuleShape(Radius, HalfHeight*2.0f);
	ASSERT(shape);
	mCapsuleShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);

	return shape;
}

btCylinderShape* Bullet::FindCylinderShape(const PINT_CYLINDER_CREATE& create)
{
	const float Radius = create.mRadius;
	const float HalfHeight = create.mHalfHeight;
	if(gShareShapes)
	{
		const int size = mCylinderShapes.size();
		for(int i=0;i<size;i++)
		{
			btCylinderShape* CurrentShape = mCylinderShapes[i];
			if(CurrentShape->getRadius()==Radius && CurrentShape->getHalfExtentsWithMargin().y()==HalfHeight)
				return CurrentShape;
		}
	}

	btCylinderShape* shape = new btCylinderShape(btVector3(Radius, HalfHeight, Radius));
	ASSERT(shape);
	mCylinderShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);

	return shape;
}

btConvexHullShape* Bullet::FindConvexShape(const PINT_CONVEX_CREATE& create)
{
	if(gShareShapes && create.mRenderer)
	{
		const int size = mConvexShapes.size();
		for(int i=0;i<size;i++)
		{
			btConvexHullShape* CurrentShape = mConvexShapes[i];
			if(CurrentShape->getUserPointer()==create.mRenderer)
				return CurrentShape;
		}
	}

	btConvexHullShape* shape = new btConvexHullShape(&create.mVerts->x, create.mNbVerts, sizeof(Point));
	ASSERT(shape);
	mConvexShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);

	return shape;
}

btCollisionShape* Bullet::CreateBulletShape(const PINT_SHAPE_CREATE& desc)
{
	btCollisionShape* BulletShape = null;
	if(desc.mType==PINT_SHAPE_SPHERE)
	{
		const PINT_SPHERE_CREATE& SphereCreate = static_cast<const PINT_SPHERE_CREATE&>(desc);
		BulletShape = FindSphereShape(SphereCreate);
	}
	else if(desc.mType==PINT_SHAPE_BOX)
	{
		const PINT_BOX_CREATE& BoxCreate = static_cast<const PINT_BOX_CREATE&>(desc);
		BulletShape = FindBoxShape(BoxCreate);
	}
	else if(desc.mType==PINT_SHAPE_CAPSULE)
	{
		const PINT_CAPSULE_CREATE& CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE&>(desc);
		BulletShape = FindCapsuleShape(CapsuleCreate);
	}
	else if(desc.mType==PINT_SHAPE_CYLINDER)
	{
		const PINT_CYLINDER_CREATE& CylinderCreate = static_cast<const PINT_CYLINDER_CREATE&>(desc);
		BulletShape = FindCylinderShape(CylinderCreate);
	}
	else if(desc.mType==PINT_SHAPE_CONVEX)
	{
		const PINT_CONVEX_CREATE& ConvexCreate = static_cast<const PINT_CONVEX_CREATE&>(desc);
		BulletShape = FindConvexShape(ConvexCreate);
	}
	else if(desc.mType==PINT_SHAPE_MESH)
	{
		const PINT_MESH_CREATE& MeshCreate = static_cast<const PINT_MESH_CREATE&>(desc);

		// ### share meshes

		btTriangleIndexVertexArray* m_indexVertexArrays = new btTriangleIndexVertexArray(
			MeshCreate.mSurface.mNbFaces,
			(int*)MeshCreate.mSurface.mDFaces,
			3*sizeof(udword),
			MeshCreate.mSurface.mNbVerts,
			(float*)&MeshCreate.mSurface.mVerts->x,
			sizeof(Point));

		btBvhTriangleMeshShape* shape  = new btBvhTriangleMeshShape(m_indexVertexArrays, true, true);
		ASSERT(shape);

		btTriangleInfoMap* triangleInfoMap = null;
		if(gUseInternalEdgeUtility)
		{
			triangleInfoMap = new btTriangleInfoMap();
			btGenerateInternalEdgeInfo(shape, triangleInfoMap);
		}

		InternalMeshShape MeshData;
		MeshData.mShape				= shape;
		MeshData.mMeshData			= m_indexVertexArrays;
		MeshData.mTriangleInfoMap	= triangleInfoMap;
		mMeshShapes.push_back(MeshData);

		mCollisionShapes.push_back(shape);

		if(desc.mRenderer)
			shape->setUserPointer(desc.mRenderer);

		BulletShape = shape;
	}
	else ASSERT(0);

	if(BulletShape)
	{
		BulletShape->setMargin(gCollisionMargin);
	}
	return BulletShape;
}

PintObjectHandle Bullet::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(mDynamicsWorld);

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;

	float FrictionCoeff = gDefaultFriction;
	float RestitutionCoeff = 0.0f;
	btCollisionShape* colShape = null;
	if(NbShapes>1)
	{
		btCompoundShape* CompoundShape = new btCompoundShape();
		colShape = CompoundShape;
		mCollisionShapes.push_back(colShape);

		while(CurrentShape)
		{
			if(CurrentShape->mMaterial)
			{
				FrictionCoeff		= CurrentShape->mMaterial->mDynamicFriction;
				RestitutionCoeff	= CurrentShape->mMaterial->mRestitution;
			}

			const btTransform LocalPose(ToBtQuaternion(CurrentShape->mLocalRot), ToBtVector3(CurrentShape->mLocalPos));

			// ### TODO: where is this deleted?
			btCollisionShape* subShape = CreateBulletShape(*CurrentShape);
			if(subShape)
			{
				CompoundShape->addChildShape(LocalPose, subShape);
			}

			CurrentShape = CurrentShape->mNext;
		}
	}
	else
	{
		colShape = CreateBulletShape(*CurrentShape);

		if(CurrentShape->mMaterial)
		{
			FrictionCoeff		= CurrentShape->mMaterial->mDynamicFriction;
			RestitutionCoeff	= CurrentShape->mMaterial->mRestitution;
		}
	}

	const bool isDynamic = (desc.mMass != 0.0f);

	btVector3 localInertia(0,0,0);
	if(isDynamic)
		colShape->calculateLocalInertia(desc.mMassForInertia<0.0f ? desc.mMass : desc.mMassForInertia, localInertia);

	const btTransform startTransform(ToBtQuaternion(desc.mRotation), ToBtVector3(desc.mPosition));

	btTransform centerOfMassOffset = btTransform::getIdentity();
	//###doesn't do what I need
//	if(desc.mCOMLocalOffset.IsNonZero())
//		centerOfMassOffset.setOrigin(ToBtVector3(desc.mCOMLocalOffset));

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform, centerOfMassOffset);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(desc.mMass, myMotionState, colShape, localInertia);
	{
		rbInfo.m_friction		= FrictionCoeff;
		rbInfo.m_restitution	= RestitutionCoeff;

	//	rbInfo.m_startWorldTransform;
		rbInfo.m_linearDamping				= gLinearDamping;
		rbInfo.m_angularDamping				= gAngularDamping;
		if(!gEnableSleeping)
		{
//			rbInfo.m_linearSleepingThreshold	= 99999999.0f;
//			rbInfo.m_angularSleepingThreshold	= 99999999.0f;
//			rbInfo.m_linearSleepingThreshold	= 0.0f;
//			rbInfo.m_angularSleepingThreshold	= 0.0f;
		}
	//	rbInfo.m_additionalDamping;
	//	rbInfo.m_additionalDampingFactor;
	//	rbInfo.m_additionalLinearDampingThresholdSqr;
	//	rbInfo.m_additionalAngularDampingThresholdSqr;
	//	rbInfo.m_additionalAngularDampingFactor;
	}
	btRigidBody* body = new btRigidBody(rbInfo);
	ASSERT(body);
	if(!gEnableSleeping)
		body->setActivationState(DISABLE_DEACTIVATION);

	sword collisionFilterGroup, collisionFilterMask;
	if(isDynamic)
	{
		body->setLinearVelocity(ToBtVector3(desc.mLinearVelocity));
		body->setAngularVelocity(ToBtVector3(desc.mAngularVelocity));

		collisionFilterGroup = short(btBroadphaseProxy::DefaultFilter);
		collisionFilterMask = short(btBroadphaseProxy::AllFilter);

		if(desc.mCollisionGroup)
		{
			const udword btGroup = RemapCollisionGroup(desc.mCollisionGroup);
			ASSERT(btGroup<32);
			collisionFilterGroup = short(1<<btGroup);
			collisionFilterMask = short(mGroupMasks[btGroup]);
		}
	}
	else
	{
//		body->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
		body->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		collisionFilterGroup = short(btBroadphaseProxy::StaticFilter);
		collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
	}

/*	if(isDynamic && desc.mCOMLocalOffset.IsNonZero())
	{
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(ToBtVector3(desc.mCOMLocalOffset));
		body->setCenterOfMassTransform(tr);
	}*/

	if(desc.mAddToWorld)
//		mDynamicsWorld->addRigidBody(body);
		mDynamicsWorld->addRigidBody(body, collisionFilterGroup, collisionFilterMask);

	if(gUseCCD)
	{
//		body->setCcdMotionThreshold(1e-7);
//		body->setCcdSweptSphereRadius(0.9*CUBE_HALF_EXTENTS);

		body->setCcdMotionThreshold(0.0001f);
		body->setCcdSweptSphereRadius(0.4f);
	}

	// ### trying to make the gyro test work but no luck so far
	if(gGyroscopicIndex)
	{
		if(gGyroscopicIndex==1)
			body->setFlags(BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT);
		else if(gGyroscopicIndex==2)
			body->setFlags(BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD);
		else if(gGyroscopicIndex==3)
			body->setFlags(BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY);
	}
	return body;
}

bool Bullet::ReleaseObject(PintObjectHandle handle)
{
	//### shapes and stuff?
	btRigidBody* body = (btRigidBody*)handle;
	mDynamicsWorld->removeRigidBody(body);
	DELETESINGLE(body);
	return true;
}

static btTransform CreateFrame(const Point& local_pivot, const Point& local_axis)
{
	btTransform frame;

	Point Right, Up;
	ComputeBasis(local_axis, Right, Up);

	frame.setOrigin(ToBtVector3(local_pivot));

	btMatrix3x3 basis;
	basis[0] = ToBtVector3(local_axis);
	basis[1] = ToBtVector3(Right);
	basis[2] = ToBtVector3(Up);
	frame.setBasis(basis);

	return frame;
}

PintJointHandle Bullet::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mDynamicsWorld);

	btRigidBody* body0 = (btRigidBody*)desc.mObject0;
	btRigidBody* body1 = (btRigidBody*)desc.mObject1;

	btTypedConstraint* constraint = null;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			constraint = new btPoint2PointConstraint(*body0, *body1, ToBtVector3(jc.mLocalPivot0), ToBtVector3(jc.mLocalPivot1));
			ASSERT(constraint);
		}
		break;

		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			if(1)
			{
				ASSERT(jc.mGlobalAnchor.IsNotUsed());
				ASSERT(jc.mGlobalAxis.IsNotUsed());

//				const btTransform frameInA = CreateFrame(jc.mLocalPivot0, jc.mLocalAxis0);
//				const btTransform frameInB = CreateFrame(jc.mLocalPivot1, jc.mLocalAxis1);
//				btHingeConstraint* hc = new btHingeConstraint(*body0, *body1, frameInA, frameInB, false);

				btHingeConstraint* hc = new btHingeConstraint(	*body0, *body1,
																ToBtVector3(jc.mLocalPivot0), ToBtVector3(jc.mLocalPivot1),
																ToBtVector3(jc.mLocalAxis0), ToBtVector3(jc.mLocalAxis1));
				ASSERT(hc);
				constraint = hc;
	//			float	targetVelocity = 1.f;
	//			float	maxMotorImpulse = 1.0f;
	//			hinge->enableAngularMotor(true,targetVelocity,maxMotorImpulse);

				if(jc.mMinLimitAngle!=MIN_FLOAT || jc.mMaxLimitAngle!=MAX_FLOAT)
					hc->setLimit(jc.mMinLimitAngle, jc.mMaxLimitAngle);
//				hc->setLimit(0.0f, 0.0f, 1.0f);
			}
			else
			{
				const btTransform frameInA = CreateFrame(jc.mLocalPivot0, jc.mLocalAxis0);
				const btTransform frameInB = CreateFrame(jc.mLocalPivot1, jc.mLocalAxis1);

				btGeneric6DofConstraint* hc = new btGeneric6DofConstraint(*body0, *body1, frameInA, frameInB, false);
				ASSERT(hc);
				constraint = hc;

//				hc->setAngularLowerLimit(btVector3(jc.mMinLimitAngle,0,0));
//				hc->setAngularUpperLimit(btVector3(jc.mMaxLimitAngle,0,0));

/*				btVector3 lowerSliderLimit = btVector3(-20,0,0);
				btVector3 hiSliderLimit = btVector3(-10,0,0);
			//	btVector3 lowerSliderLimit = btVector3(-20,-5,-5);
			//	btVector3 hiSliderLimit = btVector3(-10,5,5);
				spSlider1->setLinearLowerLimit(lowerSliderLimit);
				spSlider1->setLinearUpperLimit(hiSliderLimit);
				spSlider1->setAngularLowerLimit(btVector3(0,0,0));
				spSlider1->setAngularUpperLimit(btVector3(0,0,0));*/
			}
		}
		break;

		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			if(1)	// Emulating fixed joint with limited hinge
			{
				const Point LocalAxis(1,0,0);
				btHingeConstraint* hc = new btHingeConstraint(	*body0, *body1,
																ToBtVector3(jc.mLocalPivot0), ToBtVector3(jc.mLocalPivot1),
																ToBtVector3(LocalAxis), ToBtVector3(LocalAxis));
				ASSERT(hc);

				hc->setLimit(0.0f, 0.0f, 1.0f);

				constraint = hc;
			}
		}
		break;

		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			const btTransform frameInA = CreateFrame(jc.mLocalPivot0, jc.mLocalAxis0);
			const btTransform frameInB = CreateFrame(jc.mLocalPivot1, jc.mLocalAxis1);

			btSliderConstraint* sc = new btSliderConstraint(	*body0, *body1,
																frameInA, frameInB,
																false);

/*			if(jc.mMinLimit<=jc.mMaxLimit)
			{
				sc->setUpperLinLimit(jc.mMaxLimit);
				sc->setLowerLinLimit(jc.mMinLimit);
			}*/

			constraint = sc;
		}
		break;
	}

	if(constraint)
	{
		mDynamicsWorld->addConstraint(constraint, true);
		mConstraints.push_back(constraint);
	}
	return constraint;
}

void Bullet::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i=0;i<nb_groups;i++)
	{
		// The Bullet collision groups are a little bit of a mindfuck!
		const udword btGroup0 = RemapCollisionGroup(groups[i].mGroup0);
		const udword btGroup1 = RemapCollisionGroup(groups[i].mGroup1);
		ASSERT(btGroup0<32);
		ASSERT(btGroup1<32);

		udword Mask0 = mGroupMasks[btGroup0];
		Mask0 ^= 1<<btGroup1;
		mGroupMasks[btGroup0] = Mask0;

		udword Mask1 = mGroupMasks[btGroup1];
		Mask1 ^= 1<<btGroup0;
		mGroupMasks[btGroup1] = Mask0;
	}
}






static Bullet* gBullet = null;
static void gBullet_GetOptionsFromGUI();

void Bullet_Init(const PINT_WORLD_CREATE& desc)
{
	gBullet_GetOptionsFromGUI();

	ASSERT(!gBullet);
	gBullet = ICE_NEW(Bullet);
	gBullet->Init(desc);
}

void Bullet_Close()
{
	if(gBullet)
	{
		gBullet->Close();
		delete gBullet;
		gBullet = null;
	}
}

Bullet* GetBullet()
{
	return gBullet;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gBulletGUI = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_DefaultFriction = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_Erp = null;
static IceEditBox*	gEditBox_Erp2 = null;
static IceEditBox*	gEditBox_Tau = null;
static IceEditBox*	gEditBox_CollisionMargin = null;
static IceComboBox*	gComboBox_Gyro = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum BulletGUIElement
{
	BULLET_GUI_MAIN,
	//
	BULLET_GUI_ENABLE_SLEEPING,
	BULLET_GUI_SHARE_SHAPES,
	BULLET_GUI_USE_SPLIT_IMPULSE,
	BULLET_GUI_SOLVER_RANDOMIZE_ORDER,
	BULLET_GUI_SOLVER_FRICTION_SEPARATE,
	BULLET_GUI_SOLVER_USE_2_FRICTION_DIRECTIONS,
	BULLET_GUI_SOLVER_ENABLE_FRICTION_DIR_CACHING,
	BULLET_GUI_SOLVER_SIMD,
	BULLET_GUI_SOLVER_INTERLEAVE_CONSTRAINTS,
	BULLET_GUI_SOLVER_WARM_STARTING,
	BULLET_GUI_USE_DBVT,
	BULLET_GUI_USE_CCD,
	BULLET_GUI_INTERNAL_EDGE_UTILITY,
	//
	BULLET_GUI_SOLVER_ITER,
	BULLET_GUI_LINEAR_DAMPING,
	BULLET_GUI_ANGULAR_DAMPING,
	BULLET_GUI_ERP,
	BULLET_GUI_ERP2,
	BULLET_GUI_TAU,
	BULLET_GUI_COLLISION_MARGIN,
	BULLET_GUI_DEFAULT_FRICTION,
	//
	BULLET_GUI_GYRO,
	//
	BULLET_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case BULLET_GUI_INTERNAL_EDGE_UTILITY:
			gUseInternalEdgeUtility = checked;
			break;
		case BULLET_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case BULLET_GUI_SOLVER_ENABLE_FRICTION_DIR_CACHING:
			gSolverEnableFrictionDirCaching = checked;
			break;
		case BULLET_GUI_SOLVER_USE_2_FRICTION_DIRECTIONS:
			gSolverUse2FrictionDirections = checked;
			break;
		case BULLET_GUI_SOLVER_SIMD:
			gSolverSIMD = checked;
			break;
		case BULLET_GUI_SOLVER_INTERLEAVE_CONSTRAINTS:
			gSolverInterleaveConstraints = checked;
			break;
		case BULLET_GUI_SHARE_SHAPES:
			gShareShapes = checked;
			break;
		case BULLET_GUI_USE_SPLIT_IMPULSE:
			gUseSplitImpulse = checked;
			break;
		case BULLET_GUI_SOLVER_RANDOMIZE_ORDER:
			gSolverRandomizeOrder = checked;
			break;
		case BULLET_GUI_SOLVER_FRICTION_SEPARATE:
			gSolverFrictionSeparate = checked;
			break;
		case BULLET_GUI_SOLVER_WARM_STARTING:
			gSolverWarmStarting = checked;
			break;
		case BULLET_GUI_USE_DBVT:
			gUseDbvt = checked;
			break;
		case BULLET_GUI_USE_CCD:
			gUseCCD = checked;
			break;
		case BULLET_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>BULLET_GUI_ENABLE_DEBUG_VIZ && id<BULLET_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-BULLET_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	gDebugDrawer.updateDebugMode();
}

static void gBullet_GetOptionsFromGUI()
{
	if(gEditBox_SolverIter)
	{
		sdword tmp;
		bool status = gEditBox_SolverIter->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gSolverIterationCount = udword(tmp);
	}

	if(gEditBox_DefaultFriction)
	{
		float tmp;
		bool status = gEditBox_DefaultFriction->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gDefaultFriction = tmp;
	}

	if(gEditBox_LinearDamping)
	{
		float tmp;
		bool status = gEditBox_LinearDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gLinearDamping = tmp;
	}

	if(gEditBox_AngularDamping)
	{
		float tmp;
		bool status = gEditBox_AngularDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gAngularDamping = tmp;
	}

	if(gEditBox_Erp)
	{
		float tmp;
		bool status = gEditBox_Erp->GetTextAsFloat(tmp);
		ASSERT(status);
		gErp = tmp;
	}

	if(gEditBox_Erp2)
	{
		float tmp;
		bool status = gEditBox_Erp2->GetTextAsFloat(tmp);
		ASSERT(status);
		gErp2 = tmp;
	}

	if(gEditBox_Tau)
	{
		float tmp;
		bool status = gEditBox_Tau->GetTextAsFloat(tmp);
		ASSERT(status);
		gTau = tmp;
	}

	if(gEditBox_CollisionMargin)
	{
		float tmp;
		bool status = gEditBox_CollisionMargin->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gCollisionMargin = tmp;
	}

	if(gComboBox_Gyro)
	{
		gGyroscopicIndex = gComboBox_Gyro->GetSelectedIndex();
	}

}

IceWindow* Bullet_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gBulletGUI, parent, BULLET_GUI_MAIN, "Bullet 2.82 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, BULLET_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gBulletGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SHARE_SHAPES, 4, y, CheckBoxWidth, 20, "Share shapes", gBulletGUI, gShareShapes, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_SPLIT_IMPULSE, 4, y, CheckBoxWidth, 20, "Use split impulse", gBulletGUI, gUseSplitImpulse, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_RANDOMIZE_ORDER, 4, y, CheckBoxWidth, 20, "Solver randomize order", gBulletGUI, gSolverRandomizeOrder, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_FRICTION_SEPARATE, 4, y, CheckBoxWidth, 20, "Solver friction separate", gBulletGUI, gSolverFrictionSeparate, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_WARM_STARTING, 4, y, CheckBoxWidth, 20, "Solver warm starting", gBulletGUI, gSolverWarmStarting, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_USE_2_FRICTION_DIRECTIONS, 4, y, CheckBoxWidth, 20, "Solver use 2 friction directions", gBulletGUI, gSolverUse2FrictionDirections, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_ENABLE_FRICTION_DIR_CACHING, 4, y, CheckBoxWidth, 20, "Solver enable friction dir caching", gBulletGUI, gSolverEnableFrictionDirCaching, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_SIMD, 4, y, CheckBoxWidth, 20, "Solver SIMD", gBulletGUI, gSolverSIMD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SOLVER_INTERLEAVE_CONSTRAINTS, 4, y, CheckBoxWidth, 20, "Solver interleave constraints", gBulletGUI, gSolverInterleaveConstraints, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_DBVT, 4, y, CheckBoxWidth, 20, "Use DBVT (else SAP)", gBulletGUI, gUseDbvt, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gBulletGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_INTERNAL_EDGE_UTILITY, 4, y, CheckBoxWidth, 20, "Use internal edge utility", gBulletGUI, gUseInternalEdgeUtility, gCheckBoxCallback);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, BULLET_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gBulletGUI);

	y += YStep;

	const sdword OffsetX = 90;
	const sdword LabelWidth = 90;
	const sdword EditBoxWidth = 60;
	const sdword LabelOffsetY = 2;
	const sdword ColumnOffsetX = 200;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Num solver iter:", gBulletGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, BULLET_GUI_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gBulletGUI, EDITBOX_INTEGER_POSITIVE, null);

			helper.CreateLabel(Main, 4+ColumnOffsetX, y+LabelOffsetY, LabelWidth, 20, "Default friction:", gBulletGUI);
			gEditBox_DefaultFriction = helper.CreateEditBox(Main, BULLET_GUI_DEFAULT_FRICTION, 4+OffsetX+ColumnOffsetX, y, EditBoxWidth, 20, helper.Convert(gDefaultFriction), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);

		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Linear damping:", gBulletGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, BULLET_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Angular damping:", gBulletGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, BULLET_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "ERP:", gBulletGUI);
		gEditBox_Erp = helper.CreateEditBox(Main, BULLET_GUI_ERP, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gErp), gBulletGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "ERP2:", gBulletGUI);
		gEditBox_Erp2 = helper.CreateEditBox(Main, BULLET_GUI_ERP2, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gErp2), gBulletGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Tau:", gBulletGUI);
		gEditBox_Tau = helper.CreateEditBox(Main, BULLET_GUI_TAU, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gTau), gBulletGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Collision margin:", gBulletGUI);
		gEditBox_CollisionMargin = helper.CreateEditBox(Main, BULLET_GUI_COLLISION_MARGIN, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gCollisionMargin), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		{
			ComboBoxDesc CBBD;
			CBBD.mParent	= Main;
			CBBD.mX			= 4+OffsetX;
			CBBD.mWidth		= 300;
			CBBD.mHeight	= 20;

			helper.CreateLabel(Main, 4, y+LabelOffsetY, LabelWidth, 20, "Gyroscopic force:", gBulletGUI);
			CBBD.mID		= BULLET_GUI_GYRO;
			CBBD.mY			= y;
			CBBD.mLabel		= "Gyroscopic";
			gComboBox_Gyro = ICE_NEW(IceComboBox)(CBBD);
			gBulletGUI->Add(udword(gComboBox_Gyro));
			gComboBox_Gyro->Add("Disabled");
			gComboBox_Gyro->Add("BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT");
			gComboBox_Gyro->Add("BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD");
			gComboBox_Gyro->Add("BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY");
			gComboBox_Gyro->Select(0);
			gComboBox_Gyro->SetVisible(true);
			y += YStep;
		}
	}

	y += YStep;

	return Main;
}

void Bullet_CloseGUI()
{
	Common_CloseGUI(gBulletGUI);

	gEditBox_SolverIter = null;
	gEditBox_DefaultFriction = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	gEditBox_Erp = null;
	gEditBox_Erp2 = null;
	gEditBox_Tau = null;
	gEditBox_CollisionMargin = null;
	gComboBox_Gyro = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}

///////////////////////////////////////////////////////////////////////////////

class BulletPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Bullet_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Bullet_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Bullet_Init(desc);						}
	virtual	void		Close()												{ Bullet_Close();							}
	virtual	Pint*		GetPint()											{ return GetBullet();						}
};
static BulletPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
