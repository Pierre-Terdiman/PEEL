///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// PhysX 3.3.2 DLLs:
// - PhysX3PROFILE_x86_3_3_2.dll
// - PhysX3CookingPROFILE_x86_3_3_2.dll
// - PhysX3CommonPROFILE_x86_3_3_2.dll
//
// - PhysX3DEBUG_x86_3_3_2.dll
// - PhysX3CookingDEBUG_x86_3_3_2.dll
// - PhysX3CommonDEBUG_x86_3_3_2.dll
//
// - PhysX3_x86_3_3_2.dll
// - PhysX3Cooking_x86_3_3_2.dll
// - PhysX3Common_x86_3_3_2.dll
//
// - nvToolsExt32_1.dll (debug)

#include "stdafx.h"
#include "PINT_PhysX332.h"
#include "..\PINT_Common\PINT_Common.h"

#include "extensions\PxExtensionsAPI.h"
//#include "common/PxIO.h"
#include "common/PxRenderBuffer.h"
#include "physxprofilesdk/PxProfileZoneManager.h"
#include "PxVolumeCache.h"

//#define USE_LOAD_LIBRARY

///////////////////////////////////////////////////////////////////////////////

#define	NB_DEBUG_VIZ_PARAMS	17
static			bool						gDebugVizParams[NB_DEBUG_VIZ_PARAMS] = {0};
static	const	char*						gDebugVizNames[NB_DEBUG_VIZ_PARAMS] =
{
	"Enable debug visualization",
	"Visualize body axes",
	"Visualize body mass axes",
	"Visualize linear velocity",
	"Visualize angular velocity",
	"Visualize contact points",
	"Visualize contact normals",
	"Visualize actor axes",
	"Visualize collision AABBs",
	"Visualize collision shapes",
	"Visualize collision axes",
	"Visualize collision compounds",
	"Visualize collision statics",
	"Visualize collision dynamics",
	"Visualize joint local frames",
	"Visualize joint limits",
	"Visualize MBP regions",
};
static			PxVisualizationParameter::Enum	gDebugVizIndex[NB_DEBUG_VIZ_PARAMS] =
{
	PxVisualizationParameter::eSCALE,
	PxVisualizationParameter::eBODY_AXES,
	PxVisualizationParameter::eBODY_MASS_AXES,
	PxVisualizationParameter::eBODY_LIN_VELOCITY,
	PxVisualizationParameter::eBODY_ANG_VELOCITY,
	PxVisualizationParameter::eCONTACT_POINT,
	PxVisualizationParameter::eCONTACT_NORMAL,
	PxVisualizationParameter::eACTOR_AXES,
	PxVisualizationParameter::eCOLLISION_AABBS,
	PxVisualizationParameter::eCOLLISION_SHAPES,
	PxVisualizationParameter::eCOLLISION_AXES,
	PxVisualizationParameter::eCOLLISION_COMPOUNDS,
	PxVisualizationParameter::eCOLLISION_STATIC,
	PxVisualizationParameter::eCOLLISION_DYNAMIC,
	PxVisualizationParameter::eJOINT_LOCAL_FRAMES,
	PxVisualizationParameter::eJOINT_LIMITS,
	PxVisualizationParameter::eMBP_REGIONS,
};

///////////////////////////////////////////////////////////////////////////////

#define SUPPORT_PVD

#ifdef SUPPORT_PVD

using namespace physx::debugger;
using namespace physx::debugger::comm;

#include "PvdConnection.h"
#include "PvdConnectionManager.h"

class PVDHelper : public PvdConnectionHandler //receive notifications when pvd is connected and disconnected.
{
public:
						PVDHelper(PxPhysics* physics) : mPhysics(physics)	{}

	// PvdConnectionHandler
	virtual	void		onPvdSendClassDescriptions(PvdConnection&) {}
	virtual	void		onPvdConnected(PvdConnection& inFactory);
	virtual	void		onPvdDisconnected(PvdConnection& inFactory);
	//~PvdConnectionHandler

			void		togglePvdConnection(const EditableParams& params);
			void		createPvdConnection(const EditableParams& params);

			PxPhysics*	mPhysics;
};

#define	PVD_HOST	"127.0.0.1"

void PVDHelper::togglePvdConnection(const EditableParams& params)
{
	PvdConnectionManager* pvd = mPhysics->getPvdConnectionManager();
	if(!pvd)
		return;

	if(pvd->isConnected())
		pvd->disconnect();
	else
		createPvdConnection(params);
}

void PVDHelper::createPvdConnection(const EditableParams& params)
{
	PvdConnectionManager* pvd = mPhysics->getPvdConnectionManager();
	if(!pvd)
		return;

/*	if(0)
	{
		PxDebuggerConnectionFlags PDebuggerFlags;
		PDebuggerFlags |= PxExtensionConnectionType::Debug;
		PDebuggerFlags |= PxExtensionConnectionType::Profile;
		PDebuggerFlags |= PxExtensionConnectionType::Memory;

		PxExtensionVisualDebugger::connect( 
						pvd,
						PVD_HOST,
						5425,
						3000,
						PDebuggerFlags
						);
		return;
	}*/

	//The connection flags state overall what data is to be sent to PVD.  Currently
	//the Debug connection flag requires support from the implementation (don't send
	//the data when debug isn't set) but the other two flags, profile and memory
	//are taken care of by the PVD SDK.

	//Use these flags for a clean profile trace with minimal overhead
	//TConnectionFlagsType theConnectionFlags( PvdConnectionType::Profile )
//	PxVisualDebuggerConnectionFlags theConnectionFlags( PxVisualDebuggerExt::getAllConnectionFlags() );
//	PxVisualDebuggerConnectionFlags theConnectionFlags( PxVisualDebuggerConnectionFlags( PxVisualDebuggerConnectionFlag::Profile|PxVisualDebuggerConnectionFlag::Memory ) );
//	if(!gUseFullPvdConnection)
//		theConnectionFlags = PxVisualDebuggerConnectionFlag::Profile;

	PxVisualDebuggerConnectionFlags theConnectionFlags;
	if(params.mUseFullPvdConnection)
		theConnectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();
	else
		theConnectionFlags = PxVisualDebuggerConnectionFlag::ePROFILE;

	//Create a pvd connection that writes data straight to the filesystem.  This is
	//the fastest connection on windows for various reasons.  First, the transport is quite fast as
	//pvd writes data in blocks and filesystems work well with that abstraction.
	//Second, you don't have the PVD application parsing data and using CPU and memory bandwidth
	//while your application is running.
	//PxExtensionVisualDebugger::connect(pvd,"c:\\temp.pxd2", PxDebuggerConnectionFlags( (PxU32)theConnectionFlags));
	
	//The normal way to connect to pvd.  PVD needs to be running at the time this function is called.
	//We don't worry about the return value because we are already registered as a listener for connections
	//and thus our onPvdConnected call will take care of setting up our basic connection state.
/*	PVD::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(), PVD_HOST, 5425, 10, theConnectionFlags );
	if(theConnection)
		theConnection->release();*/
	PxVisualDebuggerExt::createConnection(pvd, PVD_HOST, 5425, 10, theConnectionFlags);
}

void PVDHelper::onPvdConnected(PvdConnection& )
{
	//setup joint visualization.  This gets piped to pvd.
	mPhysics->getVisualDebugger()->setVisualizeConstraints(true);
	mPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
}

void PVDHelper::onPvdDisconnected(PvdConnection& )
{
}
#endif

///////////////////////////////////////////////////////////////////////////////

//static PxDefaultAllocator* gDefaultAllocator = null;
//static PxDefaultErrorCallback* gDefaultErrorCallback = null;

/*static*/ PEEL_PhysX3_AllocatorCallback* gDefaultAllocator = null;
//static PxAllocatorCallback* gDefaultAllocator = null;
/*static*/ PxErrorCallback* gDefaultErrorCallback = null;

static PxDefaultCpuDispatcher* gDefaultCPUDispatcher = null;
#ifdef SUPPORT_PVD
static PVDHelper* gPVDHelper = null;
#endif

///////////////////////////////////////////////////////////////////////////////

PhysX::PhysX(const EditableParams& params) :
	SharedPhysX_Vehicles(params),
	mProfileZoneManager	(null)
{
}

PhysX::~PhysX()
{
	ASSERT(!gDefaultCPUDispatcher);
	ASSERT(!gDefaultErrorCallback);
	ASSERT(!gDefaultAllocator);
	ASSERT(!mProfileZoneManager);
}

void PhysX::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportMassForInertia			= true;
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
	caps.mSupportAggregates				= true;
	caps.mSupportArticulations			= true;
	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	caps.mSupportDistanceJoints			= true;
	caps.mSupportPhantoms				= true;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= true;
	caps.mSupportSphereSweeps			= true;
	caps.mSupportCapsuleSweeps			= true;
	caps.mSupportConvexSweeps			= true;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= true;
	caps.mSupportCapsuleOverlaps		= true;
	caps.mSupportVehicles				= true;
}

static PxFilterFlags CCDSimulationFilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PxFilterFlags DefaultFlags = PxDefaultSimulationFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
	pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
	return DefaultFlags;
}

#ifdef USE_LOAD_LIBRARY
typedef physx::PxPhysics*	(*PxFunction_CreateBasePhysics)	(physx::PxU32 version, physx::PxFoundation& foundation, const physx::PxTolerancesScale& scale, bool trackOutstandingAllocations, physx::PxProfileZoneManager* profileZoneManager);
typedef void				(*PxFunction_RegisterPCM)		(physx::PxPhysics& physics);
typedef physx::PxCooking*	(*PxFunction_CreateCooking)		(physx::PxU32 version, physx::PxFoundation& foundation, const physx::PxCookingParams& params);
typedef physx::PxFoundation*(*PxFunction_CreateFoundation)	(physx::PxU32 version, physx::PxAllocatorCallback& allocator, physx::PxErrorCallback& errorCallback);
//PX_C_EXPORT bool PX_CALL_CONV PxInitExtensions(physx::PxPhysics& physics);
#endif

class MyBroadPhaseCallback : public PxBroadPhaseCallback
{
	public:
	virtual		void	onObjectOutOfBounds(PxShape& shape, PxActor& actor)
	{}
	virtual		void	onObjectOutOfBounds(PxAggregate& aggregate)
	{}

}gBroadPhaseCallback;

void PhysX::Init(const PINT_WORLD_CREATE& desc)
{
#ifdef USE_LOAD_LIBRARY
	udword FPUEnv[256];
	FillMemory(FPUEnv, 256*4, 0xff);
	__asm fstenv FPUEnv
		HMODULE handle0 = ::LoadLibraryA("PhysX3_x86.dll");
		HMODULE handle1 = ::LoadLibraryA("PhysX3Cooking_x86.dll");
		HMODULE handle2 = ::LoadLibraryA("PhysX3Common_x86.dll");
	__asm fldenv FPUEnv
	if(!handle0 || !handle1 || !handle2)
		return;
	PxFunction_CreateBasePhysics func0	= (PxFunction_CreateBasePhysics)	GetProcAddress(handle0, "PxCreateBasePhysics");
	PxFunction_RegisterPCM func1		= (PxFunction_RegisterPCM)			GetProcAddress(handle0, "PxRegisterPCM");
	PxFunction_CreateCooking func2		= (PxFunction_CreateCooking)		GetProcAddress(handle0, "PxCreateCooking");
	PxFunction_CreateFoundation func3	= (PxFunction_CreateFoundation)		GetProcAddress(handle0, "PxCreateFoundation");

	if(!func0 || !func1 || !func2 || !func3)
	{
		FreeLibrary(handle2);
		FreeLibrary(handle1);
		FreeLibrary(handle0);
		return;
	}
#endif

//	gDefaultAllocator = new PxDefaultAllocator;
//	gDefaultErrorCallback = new PxDefaultErrorCallback;
	gDefaultAllocator = new PEEL_PhysX3_AllocatorCallback;
	gDefaultErrorCallback = new PEEL_PhysX3_ErrorCallback;

	ASSERT(!mFoundation);
#ifdef USE_LOAD_LIBRARY
	mFoundation = (func3)(PX_PHYSICS_VERSION, *gDefaultAllocator, *gDefaultErrorCallback);
#else
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gDefaultAllocator, *gDefaultErrorCallback);
//	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gDefaultAllocator, gMyErrorCallback);
#endif
	ASSERT(mFoundation);

#ifdef SUPPORT_PVD
	if(mParams.mUsePVD)
	{
		ASSERT(!mProfileZoneManager);
		mProfileZoneManager = &PxProfileZoneManager::createProfileZoneManager(mFoundation);
		ASSERT(mProfileZoneManager);
	}
#endif

	PxTolerancesScale scale;
	{
		ASSERT(!mPhysics);
	#ifdef USE_LOAD_LIBRARY
		mPhysics = (func0)(PX_PHYSICS_VERSION, *mFoundation, scale, false, mProfileZoneManager);
	//	PxRegisterArticulations(*mPhysics);
	//	PxRegisterHeightFields(*mPhysics);
		(func1)(*mPhysics);
	#else
		mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale, false, mProfileZoneManager);
	#endif
		ASSERT(mPhysics);
	}

	bool status = PxInitExtensions(*mPhysics);
	ASSERT(status);
	gDefaultCPUDispatcher = PxDefaultCpuDispatcherCreate(mParams.mNbThreads, null);

	CreateCooking(scale, PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES|PxMeshPreprocessingFlag::eREMOVE_UNREFERENCED_VERTICES|PxMeshPreprocessingFlag::eREMOVE_DUPLICATED_TRIANGLES));

#ifdef SUPPORT_PVD
	if(mParams.mUsePVD)
	{
		gPVDHelper = new PVDHelper(mPhysics);

		gPVDHelper->togglePvdConnection(mParams);

		if(mPhysics->getPvdConnectionManager())
			mPhysics->getPvdConnectionManager()->addHandler(*gPVDHelper);
	}
#endif

	{
		ASSERT(!mScene);
		PxSceneDesc sceneDesc(scale);
		sceneDesc.gravity					= ToPxVec3(desc.mGravity);
		sceneDesc.filterShader				= mParams.mUseCCD ? CCDSimulationFilterShader : PxDefaultSimulationFilterShader;
		sceneDesc.cpuDispatcher				= gDefaultCPUDispatcher;
		sceneDesc.staticStructure			= mParams.mStaticPruner;
		sceneDesc.dynamicStructure			= mParams.mDynamicPruner;
		sceneDesc.dynamicTreeRebuildRateHint= mParams.mSQDynamicRebuildRateHint;

		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_PCM,				mParams.mPCM);
		SetSceneFlag(sceneDesc, PxSceneFlag::eADAPTIVE_FORCE,			mParams.mAdaptiveForce);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_STABILIZATION,		mParams.mStabilization);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_ACTIVETRANSFORMS,	mParams.mEnableActiveTransforms);
		SetSceneFlag(sceneDesc, PxSceneFlag::eDISABLE_CONTACT_CACHE,	!mParams.mEnableContactCache);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_CCD,				mParams.mUseCCD);

	//	if(!gEnableSSE)
	//		sceneDesc.flags					|= PxSceneFlag::eDISABLE_SSE;
		if(mParams.mEnableOneDirFriction)
			//sceneDesc.flags					|= PxSceneFlag::eENABLE_ONE_DIRECTIONAL_FRICTION;
			sceneDesc.frictionType			= PxFrictionType::eONE_DIRECTIONAL;
		if(mParams.mEnableTwoDirFriction)
//			sceneDesc.flags					|= PxSceneFlag::eENABLE_TWO_DIRECTIONAL_FRICTION;
			sceneDesc.frictionType			= PxFrictionType::eTWO_DIRECTIONAL;

		sceneDesc.broadPhaseType			= mParams.mBroadPhaseType;
	//	sceneDesc.simulationOrder			= PxSimulationOrder::eSOLVE_COLLIDE;
		sceneDesc.ccdMaxPasses				= mParams.mMaxNbCCDPasses;

		mScene = mPhysics->createScene(sceneDesc);
		ASSERT(mScene);
	}

	if(mParams.mBroadPhaseType==PxBroadPhaseType::eMBP)
	{
		PxVec3 min, max;
		if(desc.mGlobalBounds.IsValid())
		{
			min.x = desc.mGlobalBounds.GetMin(0);
			min.y = desc.mGlobalBounds.GetMin(1);
			min.z = desc.mGlobalBounds.GetMin(2);
			max.x = desc.mGlobalBounds.GetMax(0);
			max.y = desc.mGlobalBounds.GetMax(1);
			max.z = desc.mGlobalBounds.GetMax(2);
		}
		else
		{
			min = PxVec3(-mParams.mMBPRange);
			max = PxVec3(mParams.mMBPRange);
		}
		const PxBounds3 globalBounds(min, max);

		PxBounds3 regions[256];
		const PxU32 nbRegions = PxBroadPhaseExt::createRegionsFromWorldBounds(regions, globalBounds, mParams.mMBPSubdivLevel);
		for(PxU32 i=0;i<nbRegions;i++)
		{
			PxBroadPhaseRegion region;
			region.bounds = regions[i];
			region.userData = (void*)i;
			mScene->addBroadPhaseRegion(region);
		}

		mScene->setBroadPhaseCallback(&gBroadPhaseCallback);
	}

	// Create default material
	{
		PINT_MATERIAL_CREATE Desc;
		Desc.mStaticFriction	= mParams.mDefaultFriction;
		Desc.mDynamicFriction	= mParams.mDefaultFriction;
		Desc.mRestitution		= 0.0f;
		mDefaultMaterial = CreateMaterial(Desc);
		ASSERT(mDefaultMaterial);
	}

	InitCommon();
	UpdateFromUI();
}

void PhysX::Close()
{
	CloseCommon();
	CloseVehicles();

#ifdef SUPPORT_PVD
	if(gPVDHelper)
		gPVDHelper->togglePvdConnection(mParams);
	DELETESINGLE(gPVDHelper);
#endif

	SAFE_RELEASE(mCooking)
	SAFE_RELEASE(mDefaultMaterial)
	SAFE_RELEASE(mScene)
	DELETESINGLE(gDefaultCPUDispatcher);

	PxCloseExtensions();

	SAFE_RELEASE(mPhysics)
	SAFE_RELEASE(mProfileZoneManager)
	SAFE_RELEASE(mFoundation)
	DELETESINGLE(gDefaultErrorCallback);
	DELETESINGLE(gDefaultAllocator);
}

void PhysX::UpdateFromUI()
{
	if(!mScene)
		return;

	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		mScene->setVisualizationParameter(gDebugVizIndex[i], float(gDebugVizParams[i]));
}

udword PhysX::Update(float dt)
{
	UpdateVehicles();
	UpdateCommon(dt);
	return gDefaultAllocator->mCurrentMemory;
}

Point PhysX::GetMainColor()
{
	return Point(1.0f, 0.75f, 0.0f);
}

static inline_ void SetupShape(const PINT_SHAPE_CREATE* create, PxShape* shape, PxU16 collision_group, const EditableParams& params)
{
	if(shape)
		SetupShape(params, create, *shape, collision_group, gDebugVizParams[0]);
}

static inline_ PxShape* CreateNonSharedShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group, const EditableParams& params)
{
	ASSERT(actor);

	PxShape* NewShape = actor->createShape(geometry, material);
	ASSERT(NewShape);
	NewShape->setLocalPose(local_pose);

	SetupShape(create, NewShape, collision_group, params);

	return NewShape;
}

static inline_ PxShape* CreateSharedShape(PxPhysics* physics, const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group, const EditableParams& params)
{
	ASSERT(physics);
	ASSERT(actor);

	PxShape* NewShape = physics->createShape(geometry, material, false);
	ASSERT(NewShape);
	NewShape->setLocalPose(local_pose);

	SetupShape(create, NewShape, collision_group, params);

	actor->attachShape(*NewShape);

	return NewShape;
}

PxShape* PhysX::CreateSphereShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxSphereGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!mParams.mShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group, mParams);

	const float Radius = geometry.radius;
	{
		const int Size = mSphereShapes.size();
		for(int i=0;i<Size;i++)
		{
			const InternalSphereShape& CurrentShape = mSphereShapes[i];
			if(		CurrentShape.mRadius==Radius
				&&	CurrentShape.Compare(material, local_pose, collision_group))
			{
				actor->attachShape(*CurrentShape.mShape);
				return CurrentShape.mShape;
			}
		}
	}

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group, mParams);

	mSphereShapes.push_back(InternalSphereShape(Radius, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateBoxShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxBoxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!mParams.mShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group, mParams);

	{
		const int Size = mBoxShapes.size();
		for(int i=0;i<Size;i++)
		{
			const InternalBoxShape& CurrentShape = mBoxShapes[i];
			if(		CurrentShape.mExtents.x==geometry.halfExtents.x
				&&	CurrentShape.mExtents.y==geometry.halfExtents.y
				&&	CurrentShape.mExtents.z==geometry.halfExtents.z
				&&	CurrentShape.Compare(material, local_pose, collision_group))
			{
				actor->attachShape(*CurrentShape.mShape);
				return CurrentShape.mShape;
			}
		}
	}

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group, mParams);

	mBoxShapes.push_back(InternalBoxShape(geometry.halfExtents, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateCapsuleShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxCapsuleGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!mParams.mShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group, mParams);

	{
		const int Size = mCapsuleShapes.size();
		for(int i=0;i<Size;i++)
		{
			const InternalCapsuleShape& CurrentShape = mCapsuleShapes[i];
			if(		CurrentShape.mRadius==geometry.radius
				&&	CurrentShape.mHalfHeight==geometry.halfHeight
				&&	CurrentShape.Compare(material, local_pose, collision_group))
			{
				actor->attachShape(*CurrentShape.mShape);
				return CurrentShape.mShape;
			}
		}
	}

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group, mParams);

	mCapsuleShapes.push_back(InternalCapsuleShape(geometry.radius, geometry.halfHeight, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateConvexShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxConvexMeshGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!mParams.mShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group, mParams);

	{
		const int Size = mConvexShapes.size();
		for(int i=0;i<Size;i++)
		{
			const InternalConvexShape& CurrentShape = mConvexShapes[i];
			if(		CurrentShape.mConvexMesh==geometry.convexMesh
				&&	CurrentShape.Compare(material, local_pose, collision_group))
			{
				actor->attachShape(*CurrentShape.mShape);
				return CurrentShape.mShape;
			}
		}
	}

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group, mParams);

	mConvexShapes.push_back(InternalConvexShape(geometry.convexMesh, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

void PhysX::CreateShapes(const PINT_OBJECT_CREATE& desc, PxRigidActor* actor)
{
	ASSERT(actor);
	ASSERT(mDefaultMaterial);
	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
		PxTransform LocalPose;
		LocalPose.p	= ToPxVec3(CurrentShape->mLocalPos);
		LocalPose.q	= ToPxQuat(CurrentShape->mLocalRot);

		PxMaterial* ShapeMaterial = mDefaultMaterial;
		if(CurrentShape->mMaterial)
		{
			ShapeMaterial = CreateMaterial(*CurrentShape->mMaterial);
			ASSERT(ShapeMaterial);
		}

		PxShape* shape = null;
		if(CurrentShape->mType==PINT_SHAPE_SPHERE)
		{
			const PINT_SPHERE_CREATE* SphereCreate = static_cast<const PINT_SPHERE_CREATE*>(CurrentShape);
			shape = CreateSphereShape(CurrentShape, actor, PxSphereGeometry(SphereCreate->mRadius), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(CurrentShape);
			shape = CreateBoxShape(CurrentShape, actor, PxBoxGeometry(ToPxVec3(BoxCreate->mExtents)), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CAPSULE)
		{
			const PINT_CAPSULE_CREATE* CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE*>(CurrentShape);

/*			// ### PhysX is weird with capsules
			Matrix3x3 Rot;
			Rot.RotY(HALFPI);
			LocalPose.q *= ToPxQuat(Quat(Rot));
*/
			const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f, 1.0f, 0.0f));
			LocalPose.q *= q;

			shape = CreateCapsuleShape(CurrentShape, actor, PxCapsuleGeometry(CapsuleCreate->mRadius, CapsuleCreate->mHalfHeight), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);

			ASSERT(mCooking);
//			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX|PxConvexFlag::eINFLATE_CONVEX, CurrentShape->mRenderer);
			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX, CurrentShape->mRenderer);
			ASSERT(ConvexMesh);

			shape = CreateConvexShape(CurrentShape, actor, PxConvexMeshGeometry(ConvexMesh), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			ASSERT(mCooking);
			PxTriangleMesh* TriangleMesh = CreateTriangleMesh(MeshCreate->mSurface, CurrentShape->mRenderer);
			ASSERT(TriangleMesh);

			shape = CreateNonSharedShape(CurrentShape, actor, PxTriangleMeshGeometry(TriangleMesh), *ShapeMaterial, LocalPose, desc.mCollisionGroup, mParams);
		}
		else ASSERT(0);

		CurrentShape = CurrentShape->mNext;
	}
}

///////////////////////////////////////////////////////////////////////////////

static inline_ void FillResultStruct(PintRaycastHit& hit, const PxRaycastHit& result)
{
//	hit.mObject			= CreateHandle(result.shape);
	hit.mObject			= CreateHandle(result.actor);
	hit.mImpact			= ToPoint(result.position);
	hit.mNormal			= ToPoint(result.normal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceIndex;
}

void* PhysX::CreatePhantom(const AABB& box)
{
	ASSERT(mScene);

	const PxU32 MaxStaticShapes = 32;
	const PxU32 MaxDynamicShapes = 8;
	PxVolumeCache* Phantom = mScene->createVolumeCache(MaxStaticShapes, MaxDynamicShapes);
	if(Phantom)
	{
		Point Center, Extents;
		box.GetCenter(Center);
		box.GetExtents(Extents);

		PxVolumeCache::FillStatus FS = Phantom->fill(PxBoxGeometry(ToPxVec3(Extents)), PxTransform(ToPxVec3(Center)));
		ASSERT(FS==PxVolumeCache::FILL_OK);
	}
	return Phantom;
}

udword PhysX::BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void** phantoms)
{
/*	ASSERT(phantoms);
	PxVolumeCache** ph = (PxVolumeCache**)phantoms;

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	while(nb--)
	{
		PxVolumeCache* phantom = *ph++;

		PxRaycastHit Hit;
		if(phantom->raycastSingle(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist,
							   PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE,
							   Hit, PF))
		{
			NbHits++;
			FillResultStruct(*dest, Hit);
		}
		else
		{
			dest->mObject = null;
		}

		raycasts++;
		dest++;
	}
	return NbHits;*/
	return 0;
}

udword PhysX::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;

	if(0)
	{
//		static
			PxBatchQuery* BatchQuery = null;
//		static
			PxBatchQueryDesc BatchQueryDesc(nb, 0, 0);
		if(!BatchQuery)
		{
			BatchQueryDesc.queryMemory.userRaycastResultBuffer	= new PxRaycastQueryResult[nb];
			BatchQueryDesc.queryMemory.userRaycastTouchBuffer	= new PxRaycastHit[nb];
			BatchQueryDesc.queryMemory.raycastTouchBufferSize	= nb;

			BatchQuery = mScene->createBatchQuery(BatchQueryDesc);
		}

		udword NbToGo = nb;
		while(nb--)
		{
			PxRaycastHit Hit;
			BatchQuery->raycast(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, 0, sqFlags, PF);
			raycasts++;
		}

		BatchQuery->execute();
		BatchQuery->release();

		udword NbHits = 0;
		for(udword i=0;i<NbToGo;i++)
		{
			if(BatchQueryDesc.queryMemory.userRaycastResultBuffer[i].getNbAnyHits())
			{
				NbHits++;
//				ASSERT(BatchQueryDesc.queryMemory.userRaycastResultBuffer[i].queryStatus==PxBatchQueryStatus::eSUCCESS);
//				ASSERT(BatchQueryDesc.queryMemory.userRaycastResultBuffer[i].nbHits==1);
				FillResultStruct(*dest, BatchQueryDesc.queryMemory.userRaycastResultBuffer[i].getAnyHit(0));
			}
			else
			{
				dest->mObject = null;
			}
			dest++;
		}
		DELETEARRAY(BatchQueryDesc.queryMemory.userRaycastTouchBuffer);
		DELETEARRAY(BatchQueryDesc.queryMemory.userRaycastResultBuffer);
		return NbHits;
	}
	else
	{
//PxSceneQueryCache Cache;
		udword NbHits = 0;
		while(nb--)
		{
//bool blockingHit;
//PxRaycastHit HitBuffer[256];
//if(mScene->raycastMultiple(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags, HitBuffer, 256, blockingHit, PF))

			PxRaycastHit Hit;
			if(mScene->raycastSingle(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags,
									Hit, PF
//, null, Cache.shape ? &Cache : null
									))
			{
				NbHits++;
				FillResultStruct(*dest, Hit);
//FillResultStruct(*dest, HitBuffer[0]);
//Cache.shape	= Hit.shape;
//Cache.actor	= Hit.actor;
			}
			else
			{
				dest->mObject = null;
			}

			raycasts++;
			dest++;
		}
		return NbHits;
	}
}

class MyQueryFilterCallback : public PxQueryFilterCallback
{
public:
	/**
	\brief This filter callback is executed before the exact intersection test.

	\param[in] filterData custom filter data specified as the query's filterData.data parameter.
	\param[in] shape A shape that has not yet passed the exact intersection test.
	\param[in] actor The shape's actor.
	\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
	\return the updated type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum preFilter(
		const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags)
	{
		return PxQueryHitType::eTOUCH;
	}

	/**
	\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.

	\param[in] filterData custom filter data of the query
	\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
	\return the updated hit type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit)
	{
		return PxQueryHitType::eTOUCH;
	}
	virtual ~MyQueryFilterCallback() {}
}gQueryFilterCallback;

udword PhysX::BatchRaycastAll(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;

	PxRaycastHit hitBuffer[2048];
	PxU32 hitBufferSize = 2048;
	bool blockingHit;

	udword NbHits = 0;
	while(nb--)
	{
		const PxI32 CurrentNbHits = mScene->raycastMultiple(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags,
															hitBuffer, hitBufferSize, blockingHit, PF/*, &gQueryFilterCallback*/);
		NbHits += CurrentNbHits;
		dest->mNbObjects = CurrentNbHits;
		raycasts++;
		dest++;
	}
	return NbHits;
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const PxSweepHit& result)
{
	hit.mObject			= CreateHandle(result.shape);
	hit.mImpact			= ToPoint(result.position);
	hit.mNormal			= ToPoint(result.normal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceIndex;
}

static inline_ PxSceneQueryFlags GetSweepQueryFlags(const EditableParams& params)
{
	PxSceneQueryFlags flags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;
	if(!params.mSQInitialOverlap)
//		flags |= PxSceneQueryFlag::eINITIAL_OVERLAP_DISABLE;
		flags |= PxSceneQueryFlag::eASSUME_NO_INITIAL_OVERLAP;
	if(params.mSQPreciseSweeps)
		flags |= PxSceneQueryFlag::ePRECISE_SWEEP;
	return flags;
}

udword PhysX::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags(mParams);

	udword NbHits = 0;
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = sweeps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(sweeps->mBox.mCenter), ToPxQuat(Q));

		PxSweepHit Hit;
//		if(mScene->sweepAny(PxBoxGeometry(ToPxVec3(sweeps->mBox.mExtents)), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
		if(mScene->sweepSingle(PxBoxGeometry(ToPxVec3(sweeps->mBox.mExtents)), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
		{
			NbHits++;
			FillResultStruct(*dest, Hit);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags(mParams);

	udword NbHits = 0;
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(sweeps->mSphere.mCenter), PxQuat::createIdentity());

		PxSweepHit Hit;
		if(mScene->sweepSingle(PxSphereGeometry(sweeps->mSphere.mRadius), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
		{
			NbHits++;
			FillResultStruct(*dest, Hit);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags(mParams);

	udword NbHits = 0;
	while(nb--)
	{
		const Point Center = (sweeps->mCapsule.mP0 + sweeps->mCapsule.mP1)*0.5f;
		Point CapsuleAxis = sweeps->mCapsule.mP1 - sweeps->mCapsule.mP0;
		const float M = CapsuleAxis.Magnitude();
		CapsuleAxis /= M;
		const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(CapsuleAxis));

		const PxTransform Pose(ToPxVec3(Center), q);

		PxSweepHit Hit;
		if(mScene->sweepSingle(PxCapsuleGeometry(sweeps->mCapsule.mRadius, M*0.5f), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
		{
			NbHits++;
			FillResultStruct(*dest, Hit);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags(mParams);

	PxConvexMeshGeometry convexGeom;

	udword NbHits = 0;
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(sweeps->mTransform.mPos), ToPxQuat(sweeps->mTransform.mRot));

		convexGeom.convexMesh = mConvexObjects[sweeps->mConvexObjectIndex];

		PxSweepHit Hit;
		if(mScene->sweepSingle(convexGeom, Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
		{
			NbHits++;
			FillResultStruct(*dest, Hit);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	return NbHits;
}




static PhysX* gPhysX = null;

class MyUICallback : public UICallback
{
	public:
	virtual	void			UIModificationCallback()
	{
		if(gPhysX)
			gPhysX->UpdateFromUI();
	}

}gUICallback;

void PhysX_Init(const PINT_WORLD_CREATE& desc)
{
	PhysX3::GetOptionsFromGUI(desc.GetTestName());

	for(PxU16 j=0;j<32;j++)
		for(PxU16 i=0;i<32;i++)
			PxSetGroupCollisionFlag(i, j, true);

	ASSERT(!gPhysX);
	gPhysX = ICE_NEW(PhysX)(PhysX3::GetEditableParams());
	gPhysX->Init(desc);
}

void PhysX_Close()
{
	if(gPhysX)
	{
		gPhysX->Close();
		delete gPhysX;
		gPhysX = null;
	}
}

PhysX* GetPhysX()
{
	return gPhysX;
}

///////////////////////////////////////////////////////////////////////////////

IceWindow* PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	return PhysX3::InitSharedGUI(parent, helper, gUICallback, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames);
}

void PhysX_CloseGUI()
{
	PhysX3::CloseSharedGUI();
}

///////////////////////////////////////////////////////////////////////////////

class PhysXPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return PhysX_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ PhysX_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ PhysX_Init(desc);						}
	virtual	void		Close()												{ PhysX_Close();						}
	virtual	Pint*		GetPint()											{ return GetPhysX();					}
};
static PhysXPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
