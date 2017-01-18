///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_PhysX33.h"
#include "..\PINT_Common\PINT_Common.h"

#include "extensions\PxExtensionsAPI.h"
#include "common/PxIO.h"
#include "common/PxRenderBuffer.h"
#include "physxprofilesdk/PxProfileZoneManager.h"
#include "PxVolumeCache.h"

#include "..\PINT_Common\PINT_CommonPhysX3.h"

//#define USE_LOAD_LIBRARY

///////////////////////////////////////////////////////////////////////////////

static	const	bool	gEnableCollisionBetweenJointed	= false;
static	const	bool	gDumpSceneBoundsEachFrame		= false;
static			bool	gVisualizeMBPRegions			= false;

//static			float						gGlobalBoxSize					= 10000.0f;
static			udword						gNbThreads						= 0;
static			PxPruningStructure::Enum	gStaticPruner					= PxPruningStructure::eDYNAMIC_AABB_TREE;
static			PxPruningStructure::Enum	gDynamicPruner					= PxPruningStructure::eDYNAMIC_AABB_TREE;
static			PxBroadPhaseType::Enum		gBroadPhaseType					= PxBroadPhaseType::eSAP;
static			udword						gSolverIterationCountPos		= 4;
static			udword						gSolverIterationCountVel		= 1;
static			bool						gSQFlag							= true;
static			bool						gSQFilterOutAllShapes			= false;
static			bool						gSQInitialOverlap				= false;
//static			bool						gSQManualFlushUpdates			= true;
static			bool						gSQPreciseSweeps				= false;
static			bool						gEnableSleeping					= false;
static			bool						gShareMeshData					= true;
static			bool						gShareShapes					= true;
static			bool						gPCM							= false;
static			bool						gAdaptiveForce					= false;
static			bool						gStabilization					= true;
//static			bool						gEnableSSE						= true;
static			bool						gEnableActiveTransforms			= false;
static			bool						gEnableContactCache				= true;
static			bool						gFlushSimulation				= false;
static			bool						gDisableStrongFriction			= false;
static			bool						gEnableOneDirFriction			= false;
static			bool						gEnableTwoDirFriction			= false;
static			float						gLinearDamping					= 0.1f;
static			float						gAngularDamping					= 0.05f;
#ifdef PINT_SUPPORT_PVD	// Defined in project's properties
static			bool						gUsePVD							= true;
#else
static			bool						gUsePVD							= false;
#endif
static			bool						gUseFullPvdConnection			= true;
static			bool						gUseCCD							= false;
static			float						gContactOffset					= 0.002f;
static			float						gRestOffset						= 0.0f;
static			float						gSleepThreshold					= 0.05f;
static			udword						gMBPSubdivLevel					= 4;

#define MBP_RANGE	1000.0f

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

			void		togglePvdConnection();
			void		createPvdConnection();

			PxPhysics*	mPhysics;
};

#define	PVD_HOST	"127.0.0.1"

void PVDHelper::togglePvdConnection()
{
	PvdConnectionManager* pvd = mPhysics->getPvdConnectionManager();
	if(!pvd)
		return;

	if(pvd->isConnected())
		pvd->disconnect();
	else
		createPvdConnection();
}

void PVDHelper::createPvdConnection()
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
	if(gUseFullPvdConnection)
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

static PEEL_PhysX3_AllocatorCallback* gDefaultAllocator = null;
//static PxAllocatorCallback* gDefaultAllocator = null;
static PxErrorCallback* gDefaultErrorCallback = null;

static PxDefaultCpuDispatcher* gDefaultCPUDispatcher = null;
#ifdef SUPPORT_PVD
static PVDHelper* gPVDHelper = null;
#endif

///////////////////////////////////////////////////////////////////////////////

PhysX::PhysX(const EditableParams& params) :
	SharedPhysX			(params),
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
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
	caps.mSupportAggregates				= true;
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
	caps.mSupportConvexOverlaps			= false;
}

static PxFilterFlags CCDSimulationFilterShader(
	PxFilterObjectAttributes attributes0,
	PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1,
	PxFilterData filterData1,
	PxPairFlags& pairFlags,
	const void* constantBlock,
	PxU32 constantBlockSize)
{
	// let triggers through
	if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlags();
	}

	pairFlags = PxPairFlag::eCONTACT_DEFAULT|PxPairFlag::eCCD_LINEAR;

	return PxFilterFlags();
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
	if(gUsePVD)
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
	gDefaultCPUDispatcher = PxDefaultCpuDispatcherCreate(gNbThreads, null);

	{
		ASSERT(!mCooking);
		PxCookingParams Params(scale);

		if(gPCM)
		{
			Params.meshWeldTolerance = 0.001f;
			Params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES | PxMeshPreprocessingFlag::eREMOVE_UNREFERENCED_VERTICES | PxMeshPreprocessingFlag::eREMOVE_DUPLICATED_TRIANGLES);
		}

	#ifdef USE_LOAD_LIBRARY
		mCooking = (func2)(PX_PHYSICS_VERSION, *mFoundation, Params);
	#else
		mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, Params);
	#endif
		ASSERT(mCooking);
	}

#ifdef SUPPORT_PVD
	if(gUsePVD)
	{
		gPVDHelper = new PVDHelper(mPhysics);

		gPVDHelper->togglePvdConnection();

		if(mPhysics->getPvdConnectionManager())
			mPhysics->getPvdConnectionManager()->addHandler(*gPVDHelper);
	}
#endif

	{
		ASSERT(!mScene);
		PxSceneDesc sceneDesc(scale);
		sceneDesc.gravity					= ToPxVec3(desc.mGravity);
		sceneDesc.filterShader				= gUseCCD ? CCDSimulationFilterShader : PxDefaultSimulationFilterShader;
		sceneDesc.cpuDispatcher				= gDefaultCPUDispatcher;
		sceneDesc.staticStructure			= gStaticPruner;
		sceneDesc.dynamicStructure			= gDynamicPruner;
	//	sceneDesc.dynamicTreeRebuildRateHint= 10;

		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_PCM,				gPCM);
		SetSceneFlag(sceneDesc, PxSceneFlag::eADAPTIVE_FORCE,			gAdaptiveForce);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_STABILIZATION,		gStabilization);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_ACTIVETRANSFORMS,	gEnableActiveTransforms);
		SetSceneFlag(sceneDesc, PxSceneFlag::eDISABLE_CONTACT_CACHE,	!gEnableContactCache);
		SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_CCD,				gUseCCD);

	//	if(!gEnableSSE)
	//		sceneDesc.flags					|= PxSceneFlag::eDISABLE_SSE;
		if(gEnableOneDirFriction)
			//sceneDesc.flags					|= PxSceneFlag::eENABLE_ONE_DIRECTIONAL_FRICTION;
			sceneDesc.frictionType			= PxFrictionType::eONE_DIRECTIONAL;
		if(gEnableTwoDirFriction)
//			sceneDesc.flags					|= PxSceneFlag::eENABLE_TWO_DIRECTIONAL_FRICTION;
			sceneDesc.frictionType			= PxFrictionType::eTWO_DIRECTIONAL;

		sceneDesc.broadPhaseType			= gBroadPhaseType;
	//	sceneDesc.simulationOrder			= PxSimulationOrder::eSOLVE_COLLIDE;
	//	sceneDesc.ccdMaxPasses				= 4;

		mScene = mPhysics->createScene(sceneDesc);
		ASSERT(mScene);
	}

	if(gBroadPhaseType==PxBroadPhaseType::eMBP)
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
			min = PxVec3(-MBP_RANGE);
			max = PxVec3(MBP_RANGE);
		}
		const PxBounds3 globalBounds(min, max);

		PxBounds3 regions[256];
		const PxU32 nbRegions = PxBroadPhaseExt::createRegionsFromWorldBounds(regions, globalBounds, gMBPSubdivLevel);
		for(PxU32 i=0;i<nbRegions;i++)
		{
			PxBroadPhaseRegion region;
			region.bounds = regions[i];
			region.userData = (void*)i;
			mScene->addBroadPhaseRegion(region);
		}

		mScene->setBroadPhaseCallback(&gBroadPhaseCallback);
	}

//	mScene->setDynamicTreeRebuildRateHint(10);
//	mScene->setDynamicTreeRebuildRateHint(5);

	// Create default material
	{
		PINT_MATERIAL_CREATE Desc;
		Desc.mStaticFriction	= 0.5f;
		Desc.mDynamicFriction	= 0.5f;
		Desc.mRestitution		= 0.0f;
		mDefaultMaterial = CreateMaterial(Desc, gDisableStrongFriction);
		ASSERT(mDefaultMaterial);
	}

	UpdateFromUI();
}

void PhysX::Close()
{
#ifdef SUPPORT_PVD
	if(gPVDHelper)
		gPVDHelper->togglePvdConnection();
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
	if(mScene)
	{
		mScene->simulate(dt);
		mScene->fetchResults(true);
//		mScene->fetchResults(false);

/*		mScene->setFlag(PxSceneFlag::eENABLE_MANUAL_QUERY_UPDATE, gSQManualFlushUpdates);
		if(gSQManualFlushUpdates)
			mScene->flushQueryUpdates();*/

		if(0)
		{
			PxActor* Actors[2048];
			udword Nb = mScene->getActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC, Actors, 2048);
			for(udword i=0;i<Nb;i++)
			{
				PxVec3 LinVel = ((PxRigidDynamic*)Actors[i])->getLinearVelocity();
				//LinVel.x = 0.0f;
				LinVel.y = 0.0f;
				//LinVel.z = 0.0f;
				((PxRigidDynamic*)Actors[i])->setLinearVelocity(LinVel);
				//printf("%f\n", LinVel.y);
			}
		}

		if(0)
		{
			PxActor* Actors[2048];
			udword Nb = mScene->getActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC, Actors, 2048);
			for(udword i=0;i<Nb;i++)
			{
				PxVec3 AngVel = ((PxRigidDynamic*)Actors[i])->getAngularVelocity();
				AngVel.x = 0.0f;
				AngVel.y = 0.0f;
				AngVel.z = 0.0f;
				((PxRigidDynamic*)Actors[i])->setAngularVelocity(AngVel);
				//printf("%f\n", LinVel.y);
			}
		}

		if(gFlushSimulation)
			mScene->flushSimulation();
	}
	return gDefaultAllocator->mCurrentMemory;
//	return 0;
}

Point PhysX::GetMainColor()
{
	return Point(1.0f, 0.75f, 0.0f);
}

void PhysX::Render(PintRender& renderer)
{
	if(mScene)
	{
		AABB GlobalBounds;
		GlobalBounds.SetEmpty();

		const PxActorTypeSelectionFlags selectionFlags = PxActorTypeSelectionFlag::eRIGID_STATIC | PxActorTypeSelectionFlag::eRIGID_DYNAMIC;
		const PxU32 nbActors = mScene->getNbActors(selectionFlags);

		PxActor* buffer[64];
		PxU32 nbProcessed = 0;
		while(nbProcessed!=nbActors)
		{
			const PxU32 nb = mScene->getActors(selectionFlags, buffer, 64, nbProcessed);
			nbProcessed += nb;

			for(PxU32 i=0;i<nb;i++)
			{
				PxActor* actor = buffer[i];
				const PxType type = actor->getConcreteType();
				if(type==PxConcreteType::eRIGID_STATIC || type==PxConcreteType::eRIGID_DYNAMIC)
				{
					PxRigidActor* rigidActor = static_cast<PxRigidActor*>(actor);

					if(gDumpSceneBoundsEachFrame)
					{
						const PxBounds3 ActorBounds = rigidActor->getWorldBounds();
						AABB tmp;
						tmp.mMin = ToPoint(ActorBounds.minimum);
						tmp.mMax = ToPoint(ActorBounds.maximum);
						GlobalBounds.Add(tmp);
					}

//					const PxTransform pose = rigidActor->getGlobalPose();

					PxU32 nbShapes = rigidActor->getNbShapes();
					for(PxU32 j=0;j<nbShapes;j++)
					{
						PxShape* shape = null;
						PxU32 nb = rigidActor->getShapes(&shape, 1, j);
						ASSERT(nb==1);
						ASSERT(shape);

#ifdef PHYSX_SUPPORT_SHARED_SHAPES
						const PxTransform Pose = PxShapeExt::getGlobalPose(*shape, *rigidActor);
#else
						const PxTransform Pose = PxShapeExt::getGlobalPose(*shape);
#endif
						const PR IcePose(ToPoint(Pose.p), ToQuat(Pose.q));

						ASSERT(shape->userData);
						if(shape->userData)
						{
							PintShapeRenderer* shapeRenderer = (PintShapeRenderer*)shape->userData;

//							shapeRenderer->SetColor(GetMainColor(), type==PxConcreteType::eRIGID_STATIC);

							const PxGeometryType::Enum geomType = shape->getGeometryType();
							if(geomType==PxGeometryType::eCAPSULE)
							{
								// ### PhysX is weird with capsules
/*								Matrix3x3 Rot;
								Rot.RotZ(HALFPI);
								Quat QQ = IcePose.mRot * Quat(Rot);*/

								// ### precompute
								const PxQuat q = PxShortestRotation(PxVec3(0.0f, 1.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
								Quat QQ = IcePose.mRot * ToQuat(q);

								shapeRenderer->Render(PR(IcePose.mPos, QQ));
							}
							else
							{
								shapeRenderer->Render(IcePose);
							}
						}
/*						else
						{
							const PxGeometryType::Enum geomType = shape->getGeometryType();
							if(geomType==PxGeometryType::eSPHERE)
							{
								PxSphereGeometry geometry;
								bool status = shape->getSphereGeometry(geometry);
								ASSERT(status);

								renderer.DrawSphere(geometry.radius, IcePose);
							}
							else if(geomType==PxGeometryType::eBOX)
							{
								PxBoxGeometry geometry;
								bool status = shape->getBoxGeometry(geometry);
								ASSERT(status);

								renderer.DrawBox(ToPoint(geometry.halfExtents), IcePose);
							}
							else if(geomType==PxGeometryType::eCAPSULE)
							{
								ASSERT(0);
							}
							else if(geomType==PxGeometryType::eCONVEXMESH)
							{
								ASSERT(0);
							}
							else ASSERT(0);
						}*/
					}
				}
			}
		}

		if(gDumpSceneBoundsEachFrame)
		{
			printf("Min.x = %f\n", GlobalBounds.mMin.x);
			printf("Min.y = %f\n", GlobalBounds.mMin.y);
			printf("Min.z = %f\n", GlobalBounds.mMin.z);
			printf("Max.x = %f\n", GlobalBounds.mMax.x);
			printf("Max.y = %f\n", GlobalBounds.mMax.y);
			printf("Max.z = %f\n", GlobalBounds.mMax.z);
			printf("\n");
		}

		const PxRenderBuffer& RenderBuffer = mScene->getRenderBuffer();
		udword NbLines = RenderBuffer.getNbLines();
		const PxDebugLine* Lines = RenderBuffer.getLines();
		const Point LineColor(1.0f, 1.0f, 1.0f);
		for(udword i=0;i<NbLines;i++)
		{
			renderer.DrawLine(ToPoint(Lines[i].pos0), ToPoint(Lines[i].pos1), LineColor);
		}

		udword NbTris = RenderBuffer.getNbTriangles();
		const PxDebugTriangle* Triangles = RenderBuffer.getTriangles();
		const Point TrisColor(1.0f, 1.0f, 1.0f);
		for(udword i=0;i<NbTris;i++)
		{
			renderer.DrawTriangle(ToPoint(Triangles[i].pos0), ToPoint(Triangles[i].pos1), ToPoint(Triangles[i].pos2), TrisColor);
		}

		if(gVisualizeMBPRegions && gBroadPhaseType == PxBroadPhaseType::eMBP)
		{
			PxU32 NbRegions = mScene->getNbBroadPhaseRegions();
			for(PxU32 i=0;i<NbRegions;i++)
			{
				PxBroadPhaseRegionInfo Region;
				mScene->getBroadPhaseRegions(&Region, 1, i);
				if(Region.active)
				{
					const Point m = ToPoint(Region.region.bounds.minimum);
					const Point M = ToPoint(Region.region.bounds.maximum);

					AABB Bounds;
					Bounds.SetMinMax(m, M);

					renderer.DrawWirefameAABB(Bounds, Point(1.0f, 0.0f, 0.0f));
				}
			}
		}
	}
}

static inline_ void SetupShape(const PINT_SHAPE_CREATE* create, PxShape* shape, PxU16 collision_group)
{
	if(shape)
	{
//		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, gSQFlag);
		shape->setFlag(PxShapeFlag::eVISUALIZATION, gDebugVizParams[0]);
//		shape->setFlag(PxShapeFlag::eUSE_SWEPT_BOUNDS, gUseCCD);
		shape->setContactOffset(gContactOffset);
		shape->setRestOffset(gRestOffset);
//		const float contactOffset = shape->getContactOffset();	// 0.02
//		const float restOffset = shape->getRestOffset();		// 0.0
//		printf("contactOffset: %f\n", contactOffset);
//		printf("restOffset: %f\n", restOffset);

		// Setup query filter data so that we can filter out all shapes - debug purpose
		if(gSQFlag)
			shape->setQueryFilterData(PxFilterData(1, 0, 0, 0));

		if(create->mRenderer)
			shape->userData = create->mRenderer;

		PhysX3::SetGroup(*shape, collision_group);
	}
}

static inline_ PxShape* CreateNonSharedShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	ASSERT(actor);

	PxShape* NewShape = actor->createShape(geometry, material);
	ASSERT(NewShape);
	NewShape->setLocalPose(local_pose);

	SetupShape(create, NewShape, collision_group);

	return NewShape;
}

static inline_ PxShape* CreateSharedShape(PxPhysics* physics, const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	ASSERT(physics);
	ASSERT(actor);

	PxShape* NewShape = physics->createShape(geometry, material, false);
	ASSERT(NewShape);
	NewShape->setLocalPose(local_pose);

	SetupShape(create, NewShape, collision_group);

	actor->attachShape(*NewShape);

	return NewShape;
}

PxShape* PhysX::CreateSphereShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxSphereGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!gShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group);

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

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group);

	mSphereShapes.push_back(InternalSphereShape(Radius, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateBoxShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxBoxGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!gShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group);

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

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group);

	mBoxShapes.push_back(InternalBoxShape(geometry.halfExtents, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateCapsuleShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxCapsuleGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!gShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group);

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

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group);

	mCapsuleShapes.push_back(InternalCapsuleShape(geometry.radius, geometry.halfHeight, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PxShape* PhysX::CreateConvexShape(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxConvexMeshGeometry& geometry, const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)
{
	if(!gShareShapes)
		return CreateNonSharedShape(create, actor, geometry, material, local_pose, collision_group);

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

	PxShape* NewShape = CreateSharedShape(mPhysics, create, actor, geometry, material, local_pose, collision_group);

	mConvexShapes.push_back(InternalConvexShape(geometry.convexMesh, NewShape, &material, local_pose, collision_group));
	return NewShape;
}

PintObjectHandle PhysX::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(mPhysics);
	ASSERT(mScene);

	const PxTransform pose(ToPxVec3(desc.mPosition), ToPxQuat(desc.mRotation));

	PxRigidActor* actor;
	PxRigidDynamic* rigidDynamic = null;
	if(desc.mMass!=0.0f)
	{
		rigidDynamic = mPhysics->createRigidDynamic(pose);
		ASSERT(rigidDynamic);
		actor = rigidDynamic;
	}
	else
	{
		PxRigidStatic* rigidStatic = mPhysics->createRigidStatic(pose);
		ASSERT(rigidStatic);
		actor = rigidStatic;
	}
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
			ShapeMaterial = CreateMaterial(*CurrentShape->mMaterial, gDisableStrongFriction);
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
			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX, CurrentShape->mRenderer, gShareMeshData);
			ASSERT(ConvexMesh);

			shape = CreateConvexShape(CurrentShape, actor, PxConvexMeshGeometry(ConvexMesh), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			ASSERT(mCooking);
			PxTriangleMesh* TriangleMesh = CreateTriangleMesh(MeshCreate->mSurface, CurrentShape->mRenderer, gShareMeshData);
			ASSERT(TriangleMesh);

			shape = CreateNonSharedShape(CurrentShape, actor, PxTriangleMeshGeometry(TriangleMesh), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else ASSERT(0);

		CurrentShape = CurrentShape->mNext;
	}

	if(rigidDynamic)
	{
		rigidDynamic->setLinearDamping(gLinearDamping);
		rigidDynamic->setAngularDamping(gAngularDamping);
		rigidDynamic->setLinearVelocity(ToPxVec3(desc.mLinearVelocity));
		rigidDynamic->setAngularVelocity(ToPxVec3(desc.mAngularVelocity));
		rigidDynamic->setMaxAngularVelocity(100.0f);
//		printf("%f\n", rigidDynamic->getSleepThreshold());
		rigidDynamic->setSleepThreshold(gSleepThreshold);
//		rigidDynamic->setMass(create.mMass);
		bool status = PxRigidBodyExt::setMassAndUpdateInertia(*rigidDynamic, desc.mMass);
		ASSERT(status);
//rigidDynamic->setMass(desc.mMass);
//rigidDynamic->setMassSpaceInertiaTensor(PxVec3(1.0f, 1.0f, 1.0f));

		rigidDynamic->setSolverIterationCounts(gSolverIterationCountPos, gSolverIterationCountVel);

		rigidDynamic->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, gUseCCD);
		rigidDynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, desc.mKinematic);
	}

	// Removed since doesn't work with shared shapes
//	PxSetGroup(*actor, desc.mCollisionGroup);

	if(desc.mAddToWorld)
	{
		mScene->addActor(*actor);

		if(rigidDynamic && !desc.mKinematic)
			SetupSleeping(rigidDynamic, gEnableSleeping);
	}
	return CreateHandle(actor);
}

bool PhysX::ReleaseObject(PintObjectHandle handle)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(RigidActor)
	{
		// ### what about ConvexRender/etc?
		RigidActor->release();
		return true;
	}
	PxShape* Shape = GetShapeFromHandle(handle);
	if(Shape)
	{
#ifdef PHYSX_SUPPORT_SHARED_SHAPES
		RigidActor = Shape->getActor();
#else
		RigidActor = &Shape->getActor();
#endif
		RigidActor->release();
		return true;
	}
	ASSERT(0);
	return false;
}

PintObjectHandle PhysX::CreateAggregate(udword max_size, bool enable_self_collision)
{
	ASSERT(mPhysics);
	// TODO: where are these released?
	PxAggregate* Aggregate = mPhysics->createAggregate(max_size, enable_self_collision);
	return Aggregate;
}

bool PhysX::AddToAggregate(PintObjectHandle object, PintObjectHandle aggregate)
{
	PxRigidActor* Actor = GetActorFromHandle(object);
	if(!Actor)
		return false;

	PxAggregate* Aggregate = (PxAggregate*)aggregate;
	return Aggregate->addActor(*Actor);
}

bool PhysX::AddAggregateToScene(PintObjectHandle aggregate)
{
	PxAggregate* Aggregate = (PxAggregate*)aggregate;
	mScene->addAggregate(*Aggregate);

	const udword NbActors = Aggregate->getNbActors();
	for(udword i=0;i<NbActors;i++)
	{
		PxActor* Actor;
		udword N = Aggregate->getActors(&Actor, 1, i);
		ASSERT(N==1);

		if(Actor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(Actor);
			if(!(RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
				SetupSleeping(RigidDynamic, gEnableSleeping);
		}
	}
	return true;
}

PintJointHandle PhysX::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mPhysics);
	return PhysX3::CreateJoint(*mPhysics, desc, gEnableCollisionBetweenJointed, mParams.mUseD6Joint, mParams.mEnableJoint32Compatibility,
								mParams.mEnableJointProjection, mParams.mProjectionLinearTolerance, mParams.mProjectionAngularTolerance,
								mParams.mInverseInertiaScale, mParams.mInverseMassScale);
}

void PhysX::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i=0;i<nb_groups;i++)
		PxSetGroupCollisionFlag(groups[i].mGroup0, groups[i].mGroup1, false);
}

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
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

udword PhysX::BatchRaycastAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	while(nb--)
	{
		PxRaycastHit Hit;
		const bool b = mScene->raycastAny(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, Hit, PF);
		NbHits += b;
		dest->mHit = b;
		raycasts++;
		dest++;
	}
	return NbHits;
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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC/* | PxSceneQueryFilterFlag::ePREFILTER*/);
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

static inline_ PxSceneQueryFlags GetSweepQueryFlags()
{
	PxSceneQueryFlags flags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;
	if(!gSQInitialOverlap)
//		flags |= PxSceneQueryFlag::eINITIAL_OVERLAP_DISABLE;
		flags |= PxSceneQueryFlag::eASSUME_NO_INITIAL_OVERLAP;
	if(gSQPreciseSweeps)
		flags |= PxSceneQueryFlag::ePRECISE_SWEEP;
	return flags;
}

udword PhysX::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC/* | PxSceneQueryFilterFlag::eANY_HIT*/);
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags();

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags();

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags();

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags();

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

udword PhysX::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		PxOverlapHit Hit;
		if(mScene->overlapAny(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hit, PF))
		{
			NbHits++;
			dest->mHit = true;
		}
		else
		{
			dest->mHit = false;
		}
		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	PxOverlapHit Hits[4096];
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		PxI32 Nb = mScene->overlapMultiple(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hits, 4096, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = overlaps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(overlaps->mBox.mCenter), ToPxQuat(Q));

		PxOverlapHit Hit;
		if(mScene->overlapAny(PxBoxGeometry(ToPxVec3(overlaps->mBox.mExtents)), Pose, Hit, PF))
		{
			NbHits++;
			dest->mHit = true;
		}
		else
		{
			dest->mHit = false;
		}
		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	PxOverlapHit Hits[4096];
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = overlaps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(overlaps->mBox.mCenter), ToPxQuat(Q));

		PxI32 Nb = mScene->overlapMultiple(PxBoxGeometry(ToPxVec3(overlaps->mBox.mExtents)), Pose, Hits, 4096, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchCapsuleOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	while(nb--)
	{
		// ### refactor this!
		const Point Center = (overlaps->mCapsule.mP0 + overlaps->mCapsule.mP1)*0.5f;
		Point CapsuleAxis = overlaps->mCapsule.mP1 - overlaps->mCapsule.mP0;
		const float M = CapsuleAxis.Magnitude();
		CapsuleAxis /= M;
		const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(CapsuleAxis));

		const PxTransform Pose(ToPxVec3(Center), q);

		PxOverlapHit Hit;
		if(mScene->overlapAny(PxCapsuleGeometry(overlaps->mCapsule.mRadius, M*0.5f), Pose, Hit, PF))
		{
			NbHits++;
			dest->mHit = true;
		}
		else
		{
			dest->mHit = false;
		}
		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX::BatchCapsuleOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)
{
/*	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	PxOverlapHit Hits[4096];
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		PxI32 Nb = mScene->overlapMultiple(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hits, 4096, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;*/
	return 0;
}

udword PhysX::FindTriangles_MeshSphereOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
		return 0;

	if(RigidActor->getNbShapes()!=1)
		return 0;

	PxShape* meshShape = null;
	RigidActor->getShapes(&meshShape, 1);
	if(!meshShape)
		return 0;

	PxTriangleMeshGeometry meshGeom;
	if(!meshShape->getTriangleMeshGeometry(meshGeom))
		return 0;

	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);

	PxU32 Results[8192];
	udword NbTouchedTriangles = 0;
	const PxU32 startIndex = 0;
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		bool Overflow = false;
		PxU32 Nb = PxMeshQuery::findOverlapTriangleMesh(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, meshGeom, meshPose, Results, 8192, startIndex, Overflow);
		ASSERT(!Overflow);

		NbTouchedTriangles += Nb;
		overlaps++;
	}
	return NbTouchedTriangles;
}

udword PhysX::FindTriangles_MeshBoxOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
		return 0;

	if(RigidActor->getNbShapes()!=1)
		return 0;

	PxShape* meshShape = null;
	RigidActor->getShapes(&meshShape, 1);
	if(!meshShape)
		return 0;

	PxTriangleMeshGeometry meshGeom;
	if(!meshShape->getTriangleMeshGeometry(meshGeom))
		return 0;

	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);

	PxU32 Results[8192];
	udword NbTouchedTriangles = 0;
	const PxU32 startIndex = 0;
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = overlaps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(overlaps->mBox.mCenter), ToPxQuat(Q));

		bool Overflow = false;
		PxU32 Nb = PxMeshQuery::findOverlapTriangleMesh(PxBoxGeometry(ToPxVec3(overlaps->mBox.mExtents)), Pose, meshGeom, meshPose, Results, 8192, startIndex, Overflow);
		ASSERT(!Overflow);

		NbTouchedTriangles += Nb;
		overlaps++;
	}
	return NbTouchedTriangles;
}

udword PhysX::FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
		return 0;

	if(RigidActor->getNbShapes()!=1)
		return 0;

	PxShape* meshShape = null;
	RigidActor->getShapes(&meshShape, 1);
	if(!meshShape)
		return 0;

	PxTriangleMeshGeometry meshGeom;
	if(!meshShape->getTriangleMeshGeometry(meshGeom))
		return 0;

	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);

	PxU32 Results[8192];
	udword NbTouchedTriangles = 0;
	const PxU32 startIndex = 0;
	while(nb--)
	{
		// ### refactor this!
		const Point Center = (overlaps->mCapsule.mP0 + overlaps->mCapsule.mP1)*0.5f;
		Point CapsuleAxis = overlaps->mCapsule.mP1 - overlaps->mCapsule.mP0;
		const float M = CapsuleAxis.Magnitude();
		CapsuleAxis /= M;
		const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(CapsuleAxis));

		const PxTransform Pose(ToPxVec3(Center), q);

		bool Overflow = false;
		PxU32 Nb = PxMeshQuery::findOverlapTriangleMesh(PxCapsuleGeometry(overlaps->mCapsule.mRadius, M*0.5f), Pose, meshGeom, meshPose, Results, 8192, startIndex, Overflow);
		ASSERT(!Overflow);

		NbTouchedTriangles += Nb;
		overlaps++;
	}
	return NbTouchedTriangles;
}

udword PhysX::GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)
{
//	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	PxRigidActor* RigidActor = (PxRigidActor*)handle;
	return RigidActor->getShapes((PxShape**)shapes, 3);
}

void PhysX::SetLocalRot(PintObjectHandle handle, const Quat& q)
{
//	PxShape* Shape = GetShapeFromHandle(handle);
	PxShape* Shape = (PxShape*)handle;
	PxTransform lp = Shape->getLocalPose();
	lp.q = ToPxQuat(q);
	Shape->setLocalPose(lp);
}




static PhysX* gPhysX = null;
static void gPhysX_GetOptionsFromGUI();

void PhysX_Init(const PINT_WORLD_CREATE& desc)
{
	gPhysX_GetOptionsFromGUI();

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

static Container*	gPhysXGUI = null;
static IceComboBox*	gComboBox_NbThreads = null;
static IceComboBox*	gComboBox_StaticPruner = null;
static IceComboBox*	gComboBox_DynamicPruner = null;
static IceComboBox*	gComboBox_BroadPhase = null;
static IceEditBox*	gEditBox_SolverIterPos = null;
static IceEditBox*	gEditBox_SolverIterVel = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
//static IceEditBox*	gEditBox_GlobalBoxSize = null;
static IceEditBox*	gEditBox_ContactOffset = null;
static IceEditBox*	gEditBox_RestOffset = null;
static IceEditBox*	gEditBox_SleepThreshold = null;
static IceEditBox*	gEditBox_MBPSubdivLevel = null;
static IceCheckBox*	gCheckBox_FullPVD = null;
static IceCheckBox*	gCheckBox_SQ_FilterOutAllShapes = null;
static IceCheckBox*	gCheckBox_SQ_InitialOverlap = null;
//static IceCheckBox*	gCheckBox_SQ_ManualFlushUpdates = null;
static IceCheckBox*	gCheckBox_SQ_PreciseSweeps = null;
//static IceCheckBox*	gCheckBox_DrawMBPRegions = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum PhysXGUIElement
{
	PHYSX_GUI_MAIN,
	//
	PHYSX_GUI_ENABLE_SLEEPING,
	PHYSX_GUI_ENABLE_SQ,
	PHYSX_GUI_ENABLE_CCD,
	PHYSX_GUI_SQ_FILTER_OUT,
	PHYSX_GUI_SQ_INITIAL_OVERLAP,
//	PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES,
	PHYSX_GUI_SQ_PRECISE_SWEEPS,
	PHYSX_GUI_SHARE_MESH_DATA,
	PHYSX_GUI_SHARE_SHAPES,
	PHYSX_GUI_PCM,
	PHYSX_GUI_ADAPTIVE_FORCE,
	PHYSX_GUI_STABILIZATION,
//	PHYSX_GUI_ENABLE_SSE,
	PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS,
	PHYSX_GUI_ENABLE_CONTACT_CACHE,
	PHYSX_GUI_FLUSH_SIMULATION,
	PHYSX_GUI_DISABLE_STRONG_FRICTION,
	PHYSX_GUI_ENABLE_ONE_DIR_FRICTION,
	PHYSX_GUI_ENABLE_TWO_DIR_FRICTION,
	PHYSX_GUI_USE_PVD,
	PHYSX_GUI_USE_FULL_PVD_CONNECTION,
//	PHYSX_GUI_DRAW_MBP_REGIONS,
	//
	PHYSX_GUI_NB_THREADS,
	PHYSX_GUI_STATIC_PRUNER,
	PHYSX_GUI_DYNAMIC_PRUNER,
	PHYSX_GUI_BROAD_PHASE,
	//
	PHYSX_GUI_NB_SOLVER_ITER_POS,
	PHYSX_GUI_NB_SOLVER_ITER_VEL,
	PHYSX_GUI_LINEAR_DAMPING,
	PHYSX_GUI_ANGULAR_DAMPING,
	PHYSX_GUI_NUM_16K_CONTACT_DATA_BLOCKS,
	PHYSX_GUI_GLOBAL_BOX_SIZE,
	PHYSX_GUI_CONTACT_OFFSET,
	PHYSX_GUI_REST_OFFSET,
	PHYSX_GUI_SLEEP_THRESHOLD,
	PHYSX_GUI_MBP_SUBDIV_LEVEL,
	//
	PHYSX_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case PHYSX_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case PHYSX_GUI_ENABLE_SQ:
			gSQFlag = checked;
			if(gCheckBox_SQ_FilterOutAllShapes)
				gCheckBox_SQ_FilterOutAllShapes->SetEnabled(checked);
			if(gCheckBox_SQ_InitialOverlap)
				gCheckBox_SQ_InitialOverlap->SetEnabled(checked);
//			if(gCheckBox_SQ_ManualFlushUpdates)
//				gCheckBox_SQ_ManualFlushUpdates->SetEnabled(checked);
			if(gCheckBox_SQ_PreciseSweeps)
				gCheckBox_SQ_PreciseSweeps->SetEnabled(checked);
			if(gComboBox_StaticPruner)
				gComboBox_StaticPruner->SetEnabled(checked);
			if(gComboBox_DynamicPruner)
				gComboBox_DynamicPruner->SetEnabled(checked);
			break;
		case PHYSX_GUI_ENABLE_CCD:
			gUseCCD = checked;
			break;
		case PHYSX_GUI_SQ_FILTER_OUT:
			gSQFilterOutAllShapes = checked;
			break;
		case PHYSX_GUI_SQ_INITIAL_OVERLAP:
			gSQInitialOverlap = checked;
			break;
/*		case PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES:
			gSQManualFlushUpdates = checked;
			break;*/
		case PHYSX_GUI_SQ_PRECISE_SWEEPS:
			gSQPreciseSweeps = checked;
			break;
		case PHYSX_GUI_SHARE_MESH_DATA:
			gShareMeshData = checked;
			break;
		case PHYSX_GUI_SHARE_SHAPES:
			gShareShapes = checked;
			break;
		case PHYSX_GUI_PCM:
			gPCM = checked;
			break;
		case PHYSX_GUI_ADAPTIVE_FORCE:
			gAdaptiveForce = checked;
			break;
		case PHYSX_GUI_STABILIZATION:
			gStabilization = checked;
			break;
//		case PHYSX_GUI_ENABLE_SSE:
//			gEnableSSE = checked;
//			break;
		case PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS:
			gEnableActiveTransforms = checked;
			break;
		case PHYSX_GUI_ENABLE_CONTACT_CACHE:
			gEnableContactCache = checked;
			break;
		case PHYSX_GUI_FLUSH_SIMULATION:
			gFlushSimulation = checked;
			break;
		case PHYSX_GUI_DISABLE_STRONG_FRICTION:
			gDisableStrongFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_ONE_DIR_FRICTION:
			gEnableOneDirFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_TWO_DIR_FRICTION:
			gEnableTwoDirFriction = checked;
			break;
		case PHYSX_GUI_USE_PVD:
			gUsePVD = checked;
			if(gCheckBox_FullPVD)
				gCheckBox_FullPVD->SetEnabled(checked);
			break;
		case PHYSX_GUI_USE_FULL_PVD_CONNECTION:
			gUseFullPvdConnection = checked;
			break;
//		case PHYSX_GUI_DRAW_MBP_REGIONS:
//			gVisualizeMBPRegions = checked;
//			break;
		case PHYSX_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>PHYSX_GUI_ENABLE_DEBUG_VIZ && id<PHYSX_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-PHYSX_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	if(gPhysX)
		gPhysX->UpdateFromUI();
}

static udword gNbThreadsToIndex[] = { 0, 0, 1, 2, 3 };
static udword gIndexToNbThreads[] = { 0, 2, 3, 4 };

static void gPhysX_GetOptionsFromGUI()
{
	if(gComboBox_NbThreads)
	{
		const udword Index = gComboBox_NbThreads->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToNbThreads)/sizeof(gIndexToNbThreads[0]));
		gNbThreads = gIndexToNbThreads[Index];
	}

	if(gComboBox_StaticPruner)
	{
		const udword Index = gComboBox_StaticPruner->GetSelectedIndex();
		gStaticPruner = PxPruningStructure::Enum(Index);
	}

	if(gComboBox_DynamicPruner)
	{
		const udword Index = gComboBox_DynamicPruner->GetSelectedIndex();
		gDynamicPruner = PxPruningStructure::Enum(Index);
	}

	if(gComboBox_BroadPhase)
	{
		const udword Index = gComboBox_BroadPhase->GetSelectedIndex();
		gBroadPhaseType = PxBroadPhaseType::Enum(Index);
	}

	if(gEditBox_SolverIterPos)
	{
		sdword tmp;
		bool status = gEditBox_SolverIterPos->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gSolverIterationCountPos = udword(tmp);
	}

	if(gEditBox_SolverIterVel)
	{
		sdword tmp;
		bool status = gEditBox_SolverIterVel->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gSolverIterationCountVel = udword(tmp);
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

/*	if(gEditBox_GlobalBoxSize)
	{
		float tmp;
		bool status = gEditBox_GlobalBoxSize->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gGlobalBoxSize = tmp;
	}*/

	if(gEditBox_ContactOffset)
	{
		float tmp;
		bool status = gEditBox_ContactOffset->GetTextAsFloat(tmp);
		ASSERT(status);
		gContactOffset = tmp;
	}

	if(gEditBox_RestOffset)
	{
		float tmp;
		bool status = gEditBox_RestOffset->GetTextAsFloat(tmp);
		ASSERT(status);
		gRestOffset = tmp;
	}

	if(gEditBox_SleepThreshold)
	{
		float tmp;
		bool status = gEditBox_SleepThreshold->GetTextAsFloat(tmp);
		ASSERT(status);
		gSleepThreshold = tmp;
	}

	if(gEditBox_MBPSubdivLevel)
	{
		sdword tmp;
		bool status = gEditBox_MBPSubdivLevel->GetTextAsInt(tmp);
		ASSERT(status);
		gMBPSubdivLevel = tmp;
	}
}

	// ### would be easier to use a callback here
	class BPComboBox : public IceComboBox
	{
		public:
								BPComboBox(const ComboBoxDesc& desc) : IceComboBox(desc)	{}
		virtual	void			OnComboBoxEvent(ComboBoxEvent event)
		{
			if(event==CBE_SELECTION_CHANGED)
			{
				if(gEditBox_MBPSubdivLevel)
				{
					const udword Index = GetSelectedIndex();
					if(PxBroadPhaseType::Enum(Index)==PxBroadPhaseType::eSAP)
						gEditBox_MBPSubdivLevel->SetEnabled(false);
					else
						gEditBox_MBPSubdivLevel->SetEnabled(true);
				}
			}
		}
	};

IceWindow* PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gPhysXGUI, parent, PHYSX_GUI_MAIN, "PhysX 3.3 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gPhysXGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SQ, 4, y, CheckBoxWidth, 20, "Enable scene queries", gPhysXGUI, gSQFlag, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_SQ_FilterOutAllShapes = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_FILTER_OUT, 4, y, CheckBoxWidth, 20, "Filter out all shapes (DEBUG)", gPhysXGUI, gSQFilterOutAllShapes, gCheckBoxCallback);
		gCheckBox_SQ_FilterOutAllShapes->SetEnabled(gSQFlag);
		y += YStepCB;

		gCheckBox_SQ_InitialOverlap = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_INITIAL_OVERLAP, 4, y, CheckBoxWidth, 20, "eINITIAL_OVERLAP flag (sweeps)", gPhysXGUI, gSQInitialOverlap, gCheckBoxCallback);
		gCheckBox_SQ_InitialOverlap->SetEnabled(gSQFlag);
		y += YStepCB;

/*		gCheckBox_SQ_ManualFlushUpdates = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES, 4, y, CheckBoxWidth, 20, "Manual flush updates", gPhysXGUI, gSQManualFlushUpdates, gCheckBoxCallback);
		gCheckBox_SQ_ManualFlushUpdates->SetEnabled(gSQFlag);
		y += YStepCB;*/

		gCheckBox_SQ_PreciseSweeps = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_PRECISE_SWEEPS, 4, y, CheckBoxWidth, 20, "Precise sweeps", gPhysXGUI, gSQPreciseSweeps, gCheckBoxCallback);
		gCheckBox_SQ_PreciseSweeps->SetEnabled(gSQFlag);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gPhysXGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_SHARE_MESH_DATA, 4, y, CheckBoxWidth, 20, "Share mesh data", gPhysXGUI, gShareMeshData, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_SHARE_SHAPES, 4, y, CheckBoxWidth, 20, "Share shapes", gPhysXGUI, gShareShapes, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_PCM, 4, y, CheckBoxWidth, 20, "Enable PCM", gPhysXGUI, gPCM, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gPhysXGUI, gAdaptiveForce, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_STABILIZATION, 4, y, CheckBoxWidth, 20, "Stabilization", gPhysXGUI, gStabilization, gCheckBoxCallback);
		y += YStepCB;

//		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SSE, 4, y, CheckBoxWidth, 20, "Enable SSE", gPhysXGUI, gEnableSSE, gCheckBoxCallback);
//		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS, 4, y, CheckBoxWidth, 20, "Enable active transforms", gPhysXGUI, gEnableActiveTransforms, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_CONTACT_CACHE, 4, y, CheckBoxWidth, 20, "Enable contact cache", gPhysXGUI, gEnableContactCache, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_FLUSH_SIMULATION, 4, y, CheckBoxWidth, 20, "Flush simulation buffers", gPhysXGUI, gFlushSimulation, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_DISABLE_STRONG_FRICTION, 4, y, CheckBoxWidth, 20, "Disable strong friction", gPhysXGUI, gDisableStrongFriction, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_ONE_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable one dir. friction", gPhysXGUI, gEnableOneDirFriction, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_TWO_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable two dir. friction", gPhysXGUI, gEnableTwoDirFriction, gCheckBoxCallback);
		y += YStepCB;

		IceCheckBox* gCheckBox_PVD = helper.CreateCheckBox(Main, PHYSX_GUI_USE_PVD, 4, y, CheckBoxWidth, 20, "Use PVD", gPhysXGUI, gUsePVD, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_FullPVD = helper.CreateCheckBox(Main, PHYSX_GUI_USE_FULL_PVD_CONNECTION, 4, y, CheckBoxWidth, 20, "Full connection", gPhysXGUI, gUseFullPvdConnection, gCheckBoxCallback);
#ifdef PINT_SUPPORT_PVD	// Defined in project's properties
		gCheckBox_FullPVD->SetEnabled(gUsePVD);
#else
		gCheckBox_PVD->SetEnabled(false);
		gCheckBox_FullPVD->SetEnabled(false);
#endif
		y += YStepCB;

		sdword LastY = Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, PHYSX_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gPhysXGUI);
//		gCheckBox_DrawMBPRegions = helper.CreateCheckBox(Main, PHYSX_GUI_DRAW_MBP_REGIONS, 290+10, LastY, CheckBoxWidth, 20, "Draw MBP regions", gPhysXGUI, gVisualizeMBPRegions, gCheckBoxCallback);
	}

	y += YStep;

	const sdword OffsetX = 90;
	const sdword LabelOffsetY = 2;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num threads:", gPhysXGUI);
		ComboBoxDesc CBBD;
		CBBD.mID		= PHYSX_GUI_NB_THREADS;
		CBBD.mParent	= Main;
		CBBD.mX			= 4+OffsetX;
		CBBD.mY			= y;
		CBBD.mWidth		= 150;
		CBBD.mHeight	= 20;
		CBBD.mLabel		= "Num threads";
		gComboBox_NbThreads = ICE_NEW(IceComboBox)(CBBD);
		gPhysXGUI->Add(udword(gComboBox_NbThreads));
		gComboBox_NbThreads->Add("Single threaded");
		gComboBox_NbThreads->Add("2 threads");
		gComboBox_NbThreads->Add("3 threads");
		gComboBox_NbThreads->Add("4 threads");
		ASSERT(gNbThreads<sizeof(gNbThreadsToIndex)/sizeof(gNbThreadsToIndex[0]));
		gComboBox_NbThreads->Select(gNbThreadsToIndex[gNbThreads]);
		gComboBox_NbThreads->SetVisible(true);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Static pruner:", gPhysXGUI);
		CBBD.mID		= PHYSX_GUI_STATIC_PRUNER;
		CBBD.mY			= y;
		CBBD.mLabel		= "Static pruner";
		gComboBox_StaticPruner = ICE_NEW(IceComboBox)(CBBD);
		gPhysXGUI->Add(udword(gComboBox_StaticPruner));
		gComboBox_StaticPruner->Add("eNONE");
		gComboBox_StaticPruner->Add("eDYNAMIC_AABB_TREE");
		gComboBox_StaticPruner->Add("eSTATIC_AABB_TREE");
		gComboBox_StaticPruner->Select(gStaticPruner);
		gComboBox_StaticPruner->SetVisible(true);
		gComboBox_StaticPruner->SetEnabled(gSQFlag);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Dynamic pruner:", gPhysXGUI);
		CBBD.mID		= PHYSX_GUI_DYNAMIC_PRUNER;
		CBBD.mY			= y;
		CBBD.mLabel		= "Dynamic pruner";
		gComboBox_DynamicPruner = ICE_NEW(IceComboBox)(CBBD);
		gPhysXGUI->Add(udword(gComboBox_DynamicPruner));
		gComboBox_DynamicPruner->Add("eNONE");
		gComboBox_DynamicPruner->Add("eDYNAMIC_AABB_TREE");
		gComboBox_DynamicPruner->Select(gDynamicPruner);
		gComboBox_DynamicPruner->SetVisible(true);
		gComboBox_DynamicPruner->SetEnabled(gSQFlag);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Broad phase:", gPhysXGUI);
		CBBD.mID		= PHYSX_GUI_BROAD_PHASE;
		CBBD.mY			= y;
		CBBD.mLabel		= "Broad phase";
		gComboBox_BroadPhase = ICE_NEW(BPComboBox)(CBBD);
		gPhysXGUI->Add(udword(gComboBox_BroadPhase));
		gComboBox_BroadPhase->Add("eSAP");
		gComboBox_BroadPhase->Add("eMBP");
		gComboBox_BroadPhase->Select(gBroadPhaseType);
		gComboBox_BroadPhase->SetVisible(true);
		y += YStep;
	}

	y += YStep;

	const sdword EditBoxWidth = 60;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Solver iter pos:", gPhysXGUI);
		gEditBox_SolverIterPos = helper.CreateEditBox(Main, PHYSX_GUI_NB_SOLVER_ITER_POS, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCountPos), gPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Solver iter vel:", gPhysXGUI);
		gEditBox_SolverIterVel = helper.CreateEditBox(Main, PHYSX_GUI_NB_SOLVER_ITER_VEL, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCountVel), gPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gPhysXGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, PHYSX_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gPhysXGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, PHYSX_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

//		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "World bounds size:", gPhysXGUI);
//		gEditBox_GlobalBoxSize = helper.CreateEditBox(Main, PHYSX_GUI_GLOBAL_BOX_SIZE, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gGlobalBoxSize), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
//		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Contact offset:", gPhysXGUI);
		gEditBox_ContactOffset = helper.CreateEditBox(Main, PHYSX_GUI_CONTACT_OFFSET, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gContactOffset), gPhysXGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Rest offset:", gPhysXGUI);
		gEditBox_RestOffset = helper.CreateEditBox(Main, PHYSX_GUI_REST_OFFSET, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gRestOffset), gPhysXGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Sleep threshold:", gPhysXGUI);
		gEditBox_SleepThreshold = helper.CreateEditBox(Main, PHYSX_GUI_REST_OFFSET, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gSleepThreshold), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "MBP subdiv level:", gPhysXGUI);
		gEditBox_MBPSubdivLevel = helper.CreateEditBox(Main, PHYSX_GUI_MBP_SUBDIV_LEVEL, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gMBPSubdivLevel), gPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
		if(gBroadPhaseType==PxBroadPhaseType::eSAP)
			gEditBox_MBPSubdivLevel->SetEnabled(false);
		else
			gEditBox_MBPSubdivLevel->SetEnabled(true);
		y += YStep;
	}
	return Main;
}

void PhysX_CloseGUI()
{
	Common_CloseGUI(gPhysXGUI);

	gComboBox_NbThreads = null;
	gComboBox_StaticPruner = null;
	gComboBox_DynamicPruner = null;
	gComboBox_BroadPhase = null;
	gEditBox_SolverIterPos = null;
	gEditBox_SolverIterVel = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
//	gEditBox_GlobalBoxSize = null;
	gEditBox_ContactOffset = null;
	gEditBox_RestOffset = null;
	gEditBox_SleepThreshold = null;
	gEditBox_MBPSubdivLevel = null;
	gCheckBox_FullPVD = null;
	gCheckBox_SQ_FilterOutAllShapes = null;
	gCheckBox_SQ_InitialOverlap = null;
//	gCheckBox_SQ_ManualFlushUpdates = null;
	gCheckBox_SQ_PreciseSweeps = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
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
