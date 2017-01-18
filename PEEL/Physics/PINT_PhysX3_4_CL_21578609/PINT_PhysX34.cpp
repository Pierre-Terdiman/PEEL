///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_PhysX34.h"
#include "..\PINT_Common\PINT_Common.h"

#include "extensions\PxExtensionsAPI.h"
//#include "common/PxIO.h"
#include "common/PxRenderBuffer.h"
//#include "physxprofilesdk/PxProfileZoneManager.h"
//#include "physxprofilesdk/PxProfileSDK.h"
//#include "NvProfileZoneManager.h"

//#define MODIFY_CONTACTS

//..\..\..\PEEL_Externals\PhysX-3.4_Trunk\Lib\vc11win32\PhysX3Extensions.lib

//#define USE_LOAD_LIBRARY

///////////////////////////////////////////////////////////////////////////////

#define	NB_DEBUG_VIZ_PARAMS	19
static			bool	gDebugVizParams[NB_DEBUG_VIZ_PARAMS] = {0};
static	const	char*	gDebugVizNames[NB_DEBUG_VIZ_PARAMS] =
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
	"Visualize collision face normals",
	"Visualize collision edges",
	"Visualize collision statics",
	"Visualize collision dynamics",
	"Visualize joint local frames",
	"Visualize joint limits",
	"Visualize MBP regions",
};
static PxVisualizationParameter::Enum gDebugVizIndex[NB_DEBUG_VIZ_PARAMS] =
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
	PxVisualizationParameter::eCOLLISION_FNORMALS,
	PxVisualizationParameter::eCOLLISION_EDGES,
	PxVisualizationParameter::eCOLLISION_STATIC,
	PxVisualizationParameter::eCOLLISION_DYNAMIC,
	PxVisualizationParameter::eJOINT_LOCAL_FRAMES,
	PxVisualizationParameter::eJOINT_LIMITS,
	PxVisualizationParameter::eMBP_REGIONS,
};

///////////////////////////////////////////////////////////////////////////////

// TODO: the API has completely changed :(   ...make this work again
//#define SUPPORT_PVD

#ifdef SUPPORT_PVD

#include "pvd\PxPvd.h"

using namespace physx::debugger;
using namespace physx::debugger::comm;

//#include "PvdConnection.h"
//#include "PvdConnectionManager.h"

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
//	mPhysics->getVisualDebugger()->setVisualizeConstraints(true);
	mPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
	mPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONSTRAINTS, true);
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

#ifdef PHYSX_SUPPORT_GPU
static PxCudaContextManager* gCudaContextManager = NULL;
#endif

///////////////////////////////////////////////////////////////////////////////

PhysX::PhysX(const EditableParams& params) :
	SharedPhysX_Vehicles	(params)
//	mProfileZoneManager		(null),
{
}

PhysX::~PhysX()
{
	ASSERT(!gDefaultCPUDispatcher);
	ASSERT(!gDefaultErrorCallback);
	ASSERT(!gDefaultAllocator);
//	ASSERT(!mProfileZoneManager);
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

#ifdef SETUP_FILTERING
static PxFilterFlags SampleVehicleFilterShader(	
	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);

	// let triggers through
	if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlags();
	}

	// use a group-based mechanism for all other pairs:
	// - Objects within the default group (mask 0) always collide
	// - By default, objects of the default group do not collide
	//   with any other group. If they should collide with another
	//   group then this can only be specified through the filter
	//   data of the default group objects (objects of a different
	//   group can not choose to do so)
	// - For objects that are not in the default group, a bitmask
	//   is used to define the groups they should collide with
	if ((filterData0.word0 != 0 || filterData1.word0 != 0) &&
		!(filterData0.word0&filterData1.word1 || filterData1.word0&filterData0.word1))
		return PxFilterFlag::eSUPPRESS;

	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// The pairFlags for each object are stored in word2 of the filter data. Combine them.
	pairFlags |= PxPairFlags(PxU16(filterData0.word2 | filterData1.word2));
	return PxFilterFlags();
}
#endif

#ifdef MODIFY_CONTACTS
static PxFilterFlags ContactModifySimulationFilterShader(
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

	pairFlags = PxPairFlag::eCONTACT_DEFAULT|PxPairFlag::eMODIFY_CONTACTS;
//	pairFlags = PxPairFlag::eCONTACT_DEFAULT|PxPairFlag::eNOTIFY_CONTACT_POINTS|PxPairFlag::eNOTIFY_TOUCH_FOUND|PxPairFlag::eNOTIFY_TOUCH_PERSISTS;

	return PxFilterFlags();
}
#endif

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

#ifdef MODIFY_CONTACTS
class MyContactModifyCallback : public PxContactModifyCallback
{
	public:
	virtual void onContactModify(PxContactModifyPair* const pairs, PxU32 count)
	{
//		printf("onContactModify: %d pairs\n", count);
		const PxReal minS = -0.2f;
		for(PxU32 i=0;i<count;i++)
		{
			const PxU32 nbContacts = pairs[i].contacts.size();
			for(PxU32 j=0;j<nbContacts;j++)
			{
				const PxReal s = pairs[i].contacts.getSeparation(j);
//				printf("%f\n", s);
				if(s<minS)
				{
//					printf("%f\n", s);
					pairs[i].contacts.setSeparation(j, minS);
				}
			}
		}
	}
}gContactModifyCallback;
#endif

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
	mFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, *gDefaultAllocator, *gDefaultErrorCallback);
//	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gDefaultAllocator, gMyErrorCallback);
#endif
	ASSERT(mFoundation);

//	nvidia::pvdsdk::NvPvd* PvdSDK = null;
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
//		mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale, false, mProfileZoneManager);
		mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale, false, null);
	#endif
		ASSERT(mPhysics);
	}

//	bool status = PxInitExtensions(*mPhysics, PvdSDK);
	bool status = PxInitExtensions(*mPhysics, null);
	ASSERT(status);
	gDefaultCPUDispatcher = PxDefaultCpuDispatcherCreate(mParams.mNbThreads, null);

	CreateCooking(scale, PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES));

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

//	PxU32					nbContactDataBlocks;
//	PxU32					maxNbContactDataBlocks;
//		sceneDesc.nbContactDataBlocks		= 0;
//		sceneDesc.maxNbContactDataBlocks	= 1;
#ifdef MODIFY_CONTACTS
		sceneDesc.contactModifyCallback		= &gContactModifyCallback;
		sceneDesc.filterShader				= ContactModifySimulationFilterShader;
#else
		sceneDesc.filterShader				= mParams.mUseCCD ? CCDSimulationFilterShader : PxDefaultSimulationFilterShader;
	#ifdef SETUP_FILTERING
		sceneDesc.filterShader				= SampleVehicleFilterShader;
	#endif
#endif
		sceneDesc.cpuDispatcher				= gDefaultCPUDispatcher;
		sceneDesc.staticStructure			= mParams.mStaticPruner;
		sceneDesc.dynamicStructure			= mParams.mDynamicPruner;
		sceneDesc.dynamicTreeRebuildRateHint= mParams.mSQDynamicRebuildRateHint;

//		sceneDesc.maxNbContactDataBlocks	= PX_MAX_U32;

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

#ifdef PHYSX_SUPPORT_GPU
		if(mParams.mUseGPU)
		{
			printf("Using GPU\n");
			sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
			sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
			sceneDesc.gpuMaxNumPartitions = 8;
/*			sceneDesc.gpuDynamicsConfig.patchStreamCapacity *= 2; //KS - must increase because we can exceed the default 4MB buffer with the arena demo!
			sceneDesc.gpuDynamicsConfig.contactStreamCapacity *= 2; //KS - must increase because we can exceed the default 4MB buffer with the arena demo!
			sceneDesc.gpuDynamicsConfig.contactStreamCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.forceStreamCapacity *= 2;
//			sceneDesc.gpuDynamicsConfig.frictionBufferCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.patchStreamCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.tempBufferCapacity *= 2;*/
			sceneDesc.gpuDynamicsConfig.constraintBufferCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.contactBufferCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.tempBufferCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.contactStreamSize *= 2;
			sceneDesc.gpuDynamicsConfig.patchStreamSize *= 2;
			sceneDesc.gpuDynamicsConfig.forceStreamCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.heapCapacity *= 2;
			sceneDesc.gpuDynamicsConfig.foundLostPairsCapacity *= 2;

			PxCudaContextManagerDesc cudaContextManagerDesc;
			cudaContextManagerDesc.interopMode = PxCudaInteropMode::OGL_INTEROP;
			gCudaContextManager = PxCreateCudaContextManager(*mFoundation, cudaContextManagerDesc);
			if(gCudaContextManager)
			{
				if(!gCudaContextManager->contextIsValid())
				{
					gCudaContextManager->release();
					gCudaContextManager = NULL;
				}
			}	
			if(gCudaContextManager)
				sceneDesc.gpuDispatcher = gCudaContextManager->getGpuDispatcher();	//Set the GPU dispatcher, used by GRB to dispatch CUDA kernels.
		}
#endif

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

/*	static inline_ PintObjectHandle CreatePintObject(Pint& pint, const PINT_OBJECT_CREATE& desc)
	{
		PintObjectHandle handle = pint.CreateObject(desc);
//		pint.mOMHelper->AddObject(handle);
		return handle;
	}*/

static void setupJoint(PxArticulationJoint* j)
{
	j->setExternalCompliance(1.0f);
	j->setDamping(0.0f);

/*	j->setSwingLimitEnabled(true);
	j->setSwingLimitEnabled(false);
//	j->setSwingLimit(PxPi/6, PxPi/6);
	j->setSwingLimit(0.00001f, 0.00001f);
//	j->setTwistLimitEnabled(true);
	j->setTwistLimitEnabled(false);
//	j->setTwistLimit(-PxPi/12, PxPi/12);
	j->setTwistLimit(-0.00001f, 0.00001f);

	if(0)
	{
//		const float Limit = 0.00001f;
		const float Limit = 0.01f;
		j->setSwingLimitEnabled(true);
		j->setSwingLimit(Limit, Limit);
		j->setTwistLimitEnabled(true);
		j->setTwistLimit(-Limit, Limit);
	}*/
}

static void setupJoint(PxArticulationJoint* j, const PINT_ARTICULATED_BODY_CREATE& bc)
{
//	setupJoint(j);
	j->setSwingLimitEnabled(bc.mEnableSwingLimit);
	j->setSwingLimit(bc.mSwingYLimit, bc.mSwingZLimit);

	j->setTwistLimitEnabled(bc.mEnableTwistLimit);
	j->setTwistLimit(bc.mTwistLowerLimit, bc.mTwistUpperLimit);

	if(bc.mUseMotor)
	{
		if(bc.mMotor.mExternalCompliance!=0.0f)
			j->setExternalCompliance(bc.mMotor.mExternalCompliance);
		if(bc.mMotor.mInternalCompliance!=0.0f)
			j->setInternalCompliance(bc.mMotor.mInternalCompliance);
		j->setDamping(bc.mMotor.mDamping);
		j->setStiffness(bc.mMotor.mStiffness);
		if(!bc.mMotor.mTargetVelocity.IsNotUsed())
			j->setTargetVelocity(ToPxVec3(bc.mMotor.mTargetVelocity));
		if(!bc.mMotor.mTargetOrientation.IsNotUsed())
			j->setTargetOrientation(ToPxQuat(bc.mMotor.mTargetOrientation));
	}
}

void PhysX::TestNewFeature()
{
	return;
#ifdef REMOVED
	PxArticulation* articulation = (PxArticulation*)CreateArticulation();

//	PxArticulationLink* link = articulation->createLink(parent, linkPose);
//	PxRigidActorExt::createExclusiveShape(*link, linkGeometry, material);
//	PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

	const bool UseFiltering = true;
	if(UseFiltering)
	{
		const PintDisabledGroups DG(1, 2);
		SetDisabledGroups(1, &DG);
	}

	const float Radius = 1.0f;
	const udword NbSpheres = 20;
	const udword NbRows = 1;
	const Point Dir(1.0f, 0.0f, 0.0f);
//	const Point Extents = Dir * Radius;
	const Point Extents = Dir * (Radius + 1.0f);
	const Point PosOffset = Dir * 0.0f;

	Matrix3x3 m;
	m.RotZ(degToRad(90.0f));

//	PINT_SPHERE_CREATE SphereDesc;
	PINT_CAPSULE_CREATE SphereDesc;
//	SphereDesc.mRenderer	= CreateSphereRenderer(Radius);		#####
	SphereDesc.mRadius		= Radius;
	SphereDesc.mHalfHeight	= 1.0f;
	SphereDesc.mLocalRot	= m;

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
//			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mMass			= 1.0f;
			ObjectDesc.mPosition		= Pos;
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
			Handles[0] = CreateArticulationLink(articulation, null, *this, ObjectDesc);
//			Handles[0] = CreatePintObject(*this, ObjectDesc);

				// Make it "static"...
//				PINT_SPHERICAL_JOINT_CREATE Desc;
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mLocalAxis0 = Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1 = Point(0.0f, 0.0f, 1.0f);
				Desc.mObject0		= null;
				Desc.mObject1		= Handles[0];
				// #### WTF !!!!!
				Desc.mLocalPivot0	= Pos + Point(0.0f, 0.0f, 0.0f);
//				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Pos + Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				PintJointHandle JointHandle = CreateJoint(Desc);
				ASSERT(JointHandle);

		}
		Pos += (PosOffset + Extents)*2.0f;

		for(udword i=1;i<NbSpheres-1;i++)
		{
			Positions[i] = Pos;
//			Handles[i] = CreateDynamicObject(pint, &SphereDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &SphereDesc;
				ObjectDesc.mMass			= 1.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				// Note that this already creates the joint between the objects!
				Handles[i] = CreateArticulationLink(articulation, (PxArticulationLink*)Handles[i-1], *this, ObjectDesc);

				// ...so we setup the joint data immediately
				PxArticulationJoint* joint = ((PxArticulationLink*)Handles[i])->getInboundJoint();
				if(joint)
				{
					joint->setParentPose(PxTransform(ToPxVec3(Extents + PosOffset)));
					joint->setChildPose(PxTransform(ToPxVec3(-Extents - PosOffset)));
					setupJoint(joint);
				}
			}
			Pos += (PosOffset + Extents)*2.0f;
		}

		const Point BoxExtents(10.0f, 10.0f, 10.0f);
		{
			const udword i=NbSpheres-1;
			PINT_BOX_CREATE BoxDesc;
//			BoxDesc.mRenderer	= CreateBoxRenderer(BoxExtents);		#####
			BoxDesc.mExtents	= BoxExtents;

			//###
			Pos.x += BoxExtents.x - Radius;

			Positions[i] = Pos;
//			Handles[i] = CreateDynamicObject(pint, &SphereDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 100.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				Handles[i] = CreateArticulationLink(articulation, (PxArticulationLink*)Handles[i-1], *this, ObjectDesc);

				printf("Big Mass: %f\n", ((PxArticulationLink*)Handles[i])->getMass());

				PxArticulationJoint* joint = ((PxArticulationLink*)Handles[i])->getInboundJoint();
				if(joint)
				{
					joint->setParentPose(PxTransform(ToPxVec3(Extents + PosOffset)));
					joint->setChildPose(PxTransform(PxVec3(-BoxExtents.x, 0.0f, 0.0f)));
					setupJoint(joint);
				}
			}
			Pos += (PosOffset + Extents)*2.0f;
		}
	}
	AddArticulationToScene(articulation);

	if(1)
	{
		PINT_BOX_CREATE BoxDesc;
		BoxDesc.mExtents	= Point(2.0f, 4.0f, 20.0f);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= 1.0f;
		ObjectDesc.mPosition		= Point(0.0f, -50.0f, -15.0f);
		ObjectDesc.mCollisionGroup	= 2;
		PintObjectHandle h = CreateObject(ObjectDesc);

		if(1)
		{
			PINT_HINGE_JOINT_CREATE Desc;
			Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
			Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
			Desc.mObject0		= null;
			Desc.mObject1		= h;
			// #### WTF !!!!!
			Desc.mLocalPivot0	= ObjectDesc.mPosition;
			Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
			PintJointHandle JointHandle = CreateJoint(Desc);
			ASSERT(JointHandle);
		}
	}
#endif
}

void PhysX::Close()
{
	CloseCommon();
	CloseVehicles();

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
#ifdef PHYSX_SUPPORT_GPU
	SAFE_RELEASE(gCudaContextManager);
#endif
//	SAFE_RELEASE(mPvd);
//	SAFE_RELEASE(mTransport);	
//	SAFE_RELEASE(mProfileZoneManager)
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
//	return Point(0.1f, 0.2f, 0.3f);
	return Point(0.8f, 0.75f, 0.9f);
}

//static const char* gGroundDebugName = "Ground";
static inline_ void SetupShape(const PINT_SHAPE_CREATE* create, PxShape* shape, PxU16 collision_group, const EditableParams& params)
{
	if(shape)
	{
//		shape->setName(gGroundDebugName);

		SetupShape(params, create, *shape, collision_group, gDebugVizParams[0]);

#ifdef SETUP_FILTERING
		PxFilterData simulationFilterData;
		simulationFilterData.word0=COLLISION_FLAG_GROUND;
		simulationFilterData.word1=COLLISION_FLAG_GROUND_AGAINST;

		PxFilterData queryFilterData;
		SampleVehicleSetupDrivableShapeQueryFilterData(&queryFilterData);

		shape->setSimulationFilterData(simulationFilterData);
		shape->setQueryFilterData(queryFilterData);
#endif
	}
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
//			LocalPose.q = q * LocalPose.q;

			shape = CreateCapsuleShape(CurrentShape, actor, PxCapsuleGeometry(CapsuleCreate->mRadius, CapsuleCreate->mHalfHeight), *ShapeMaterial, LocalPose, desc.mCollisionGroup);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);

			ASSERT(mCooking);
//			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX|PxConvexFlag::eINFLATE_CONVEX, CurrentShape->mRenderer);
			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX, CurrentShape->mRenderer);
			ASSERT(ConvexMesh);

			PxConvexMeshGeometry ConvexGeom(ConvexMesh);
			if(mParams.mUseTightConvexBounds)
				ConvexGeom.meshFlags = PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;

			shape = CreateConvexShape(CurrentShape, actor, ConvexGeom, *ShapeMaterial, LocalPose, desc.mCollisionGroup);
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
							   PxSceneQueryFlag::ePOSITION|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE,
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

#ifdef SETUP_FILTERING
	PxFilterData fd;
	SampleVehicleSetupVehicleShapeQueryFilterData(&fd);
	const PxSceneQueryFilterData PF(fd, PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
#else
	const PxQueryFilterData PF = GetSQFilterData();
#endif
//	const PxSceneQueryFilterData PF(PxFilterData(0, 0, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::ePOSITION|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;

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
		PxRaycastBuffer buf;
		PxQueryFilterData fd1 = PF;
		fd1.clientId = PX_DEFAULT_CLIENT;

		udword NbHits = 0;
		while(nb--)
		{
//			PxRaycastHit Hit;
//			if(raycastSingle(mScene, ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags, Hit, PF))

/*			PX_INLINE bool raycastSingle(PxScene* scene,
			const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
			PxSceneQueryFlags outputFlags, PxRaycastHit& hit,
			const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
			PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
			PxClientID queryClient = PX_DEFAULT_CLIENT)*/

			mScene->raycast((const PxVec3&)(raycasts->mOrigin), (const PxVec3&)(raycasts->mDir), raycasts->mMaxDist, buf, sqFlags, fd1, null, null);
//			Hit = buf.block;
			if(buf.hasBlock)
			{
				NbHits++;
//				FillResultStruct(*dest, Hit);
				FillResultStruct(*dest, buf.block);
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
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::ePOSITION|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE|PxSceneQueryFlag::eMESH_MULTIPLE;

	PxRaycastHit hitBuffer[2048];
	PxU32 hitBufferSize = 2048;
	bool blockingHit;

	udword NbHits = 0;
	while(nb--)
	{
		const PxI32 CurrentNbHits = raycastMultiple(mScene, ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags,
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
	PxSceneQueryFlags flags = PxSceneQueryFlag::ePOSITION|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;
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
		if(sweepSingle(mScene, PxBoxGeometry(ToPxVec3(sweeps->mBox.mExtents)), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
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
		const PxTransform Pose(ToPxVec3(sweeps->mSphere.mCenter), PxQuat(PxIdentity));

		PxSweepHit Hit;
		if(sweepSingle(mScene, PxSphereGeometry(sweeps->mSphere.mRadius), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
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
		if(sweepSingle(mScene, PxCapsuleGeometry(sweeps->mCapsule.mRadius, M*0.5f), Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
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
		if(sweepSingle(mScene, convexGeom, Pose, ToPxVec3(sweeps->mDir), sweeps->mMaxDist, sweepQueryFlags, Hit, PF))
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
