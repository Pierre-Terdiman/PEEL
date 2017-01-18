///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// PhysX 3.2 DLLs:
// - PhysX3PROFILE_x86_3_2.dll
// - PhysX3CookingPROFILE_x86_3_2.dll
// - PhysX3CommonPROFILE_x86_3_2.dll
//
// - PhysX3DEBUG_x86_3_2.dll
// - PhysX3CookingDEBUG_x86_3_2.dll
// - PhysX3CommonDEBUG_x86_3_2.dll
//
// - PhysX3_x86_3_2.dll
// - PhysX3Cooking_x86_3_2.dll
// - PhysX3Common_x86_3_2.dll

#include "stdafx.h"
#include "PINT_PhysX32.h"
#include "..\PINT_Common\PINT_Common.h"

#include "extensions\PxExtensionsAPI.h"
#include "common/PxIO.h"
#include "common/PxRenderBuffer.h"
#include "physxprofilesdk/PxProfileZoneManager.h"

#include "..\PINT_Common\PINT_CommonPhysX3.h"

//#define USE_LOAD_LIBRARY

/*
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3CharacterKinematic_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3Common_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3Cooking_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3Extensions.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3Vehicle.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PxTask.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysXVisualDebuggerSDK.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysXProfileSDK.lib
*/

/*
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3PROFILE_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3CharacterKinematicPROFILE_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3CommonPROFILE_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3CookingPROFILE_x86.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3ExtensionsPROFILE.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysX3VehiclePROFILE.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PxTaskPROFILE.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysXVisualDebuggerSDKPROFILE.lib
..\PhysX-3.2_PC_SDK_Core\Lib\win32\PhysXProfileSDKPROFILE.lib
*/

///////////////////////////////////////////////////////////////////////////////

/*static			float						gGlobalBoxSize					= 10000.0f;
static			udword						gNbThreads						= 0;
static			PxPruningStructure::Enum	gStaticPruner					= PxPruningStructure::eSTATIC_AABB_TREE;
static			PxPruningStructure::Enum	gDynamicPruner					= PxPruningStructure::eDYNAMIC_AABB_TREE;
static			udword						gSolverIterationCountPos		= 4;
static			udword						gSolverIterationCountVel		= 1;
static			bool						gSQFlag							= true;
static			bool						mParams.mSQFilterOutAllShapes			= false;
static			bool						gSQInitialOverlap				= false;
static			bool						gEnableSleeping					= false;
static			bool						gShareMeshData					= true;
static			bool						gPCM							= false;
static			bool						gAdaptiveForce					= false;
static			bool						gEnableSSE						= true;
static			bool						gEnableActiveTransforms			= false;
static			bool						gFlushSimulation				= false;
static			bool						gDisableStrongFriction			= false;
static			bool						gEnableOneDirFriction			= false;
static			bool						gEnableTwoDirFriction			= false;
static			float						gLinearDamping					= 0.1f;
static			float						gAngularDamping					= 0.05f;
#ifdef BETA2
static			udword						gNum16KContactDataBlocks		= 256;
#endif
static			bool						gUsePVD							= false;
static			bool						gUseFullPvdConnection			= true;
static			bool						gUseCCD							= false;
static			float						gContactOffset					= 0.002f;
static			float						gRestOffset						= 0.0f;*/

#define	NB_DEBUG_VIZ_PARAMS	16
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
};

///////////////////////////////////////////////////////////////////////////////

#include "windows/PxWindowsDelayLoadHook.h"

class SampleDelayLoadHook : public PxDelayLoadHook
{
public:
	virtual const char* GetPhysXCommonDEBUGDllName()	const { return "PhysX3CommonDEBUG_x86_3_2.dll";		}
	virtual const char* GetPhysXCommonCHECKEDDllName()	const { return "PhysX3CommonCHECKED_x86_3_2.dll";	}
	virtual const char* GetPhysXCommonPROFILEDllName()	const { return "PhysX3CommonPROFILE_x86_3_2.dll";	}
	virtual const char* GetPhysXCommonDllName()			const { return "PhysX3Common_x86_3_2.dll";			}
} gDelayLoadHook;

static void SetupDelayHook()
{
	PxDelayLoadHook::SetPhysXInstance(&gDelayLoadHook);
	PxDelayLoadHook::SetPhysXCookingInstance(&gDelayLoadHook);
}

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
#ifdef BETA2
	TConnectionFlagsType theConnectionFlags( PvdConnectionType::Debug | PvdConnectionType::Profile | PvdConnectionType::Memory );
	if(!params.mUseFullPvdConnection)
		theConnectionFlags = TConnectionFlagsType(PvdConnectionType::Profile);
#else
	PxVisualDebuggerConnectionFlags theConnectionFlags( PxVisualDebuggerExt::getAllConnectionFlags() );
	if(!params.mUseFullPvdConnection)
		theConnectionFlags = PxVisualDebuggerConnectionFlag::Profile;
#endif

	//Create a pvd connection that writes data straight to the filesystem.  This is
	//the fastest connection on windows for various reasons.  First, the transport is quite fast as
	//pvd writes data in blocks and filesystems work well with that abstraction.
	//Second, you don't have the PVD application parsing data and using CPU and memory bandwidth
	//while your application is running.
	//PxExtensionVisualDebugger::connect(pvd,"c:\\temp.pxd2", PxDebuggerConnectionFlags( (PxU32)theConnectionFlags));
	
	//The normal way to connect to pvd.  PVD needs to be running at the time this function is called.
	//We don't worry about the return value because we are already registered as a listener for connections
	//and thus our onPvdConnected call will take care of setting up our basic connection state.
#ifdef BETA2
	PxExtensionVisualDebugger::connect(pvd, PVD_HOST, 5425, 10, PxDebuggerConnectionFlags( (PxU32)theConnectionFlags) );
#else
	PVD::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(), PVD_HOST, 5425, 10, theConnectionFlags );
	if(theConnection)
		theConnection->release();
#endif
}

void PVDHelper::onPvdConnected(PvdConnection& )
{
	//setup joint visualization.  This gets piped to pvd.
	mPhysics->getVisualDebugger()->setVisualizeConstraints(true);
	mPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlags::eTRANSMIT_CONTACTS, true);
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
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= true;
	caps.mSupportSphereSweeps			= true;
	caps.mSupportCapsuleSweeps			= true;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= true;
	caps.mSupportCapsuleOverlaps		= true;
}

static PxFilterFlags CCDSimulationFilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PxFilterFlags DefaultFlags = PxDefaultSimulationFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
	pairFlags |= PxPairFlag::eSWEPT_INTEGRATION_LINEAR;
	return DefaultFlags;
}

#ifdef USE_LOAD_LIBRARY
typedef physx::PxPhysics*	(*PxFunction_CreateBasePhysics)	(physx::PxU32 version, physx::PxFoundation& foundation, const physx::PxTolerancesScale& scale, bool trackOutstandingAllocations, physx::PxProfileZoneManager* profileZoneManager);
typedef void				(*PxFunction_RegisterPCM)		(physx::PxPhysics& physics);
typedef physx::PxCooking*	(*PxFunction_CreateCooking)		(physx::PxU32 version, physx::PxFoundation& foundation, const physx::PxCookingParams& params);
typedef physx::PxFoundation*(*PxFunction_CreateFoundation)	(physx::PxU32 version, physx::PxAllocatorCallback& allocator, physx::PxErrorCallback& errorCallback);
//PX_C_EXPORT bool PX_CALL_CONV PxInitExtensions(physx::PxPhysics& physics);
#endif

void PhysX::Init(const PINT_WORLD_CREATE& desc)
{
	SetupDelayHook();

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

	ASSERT(!mPhysics);
	PxTolerancesScale scale;
#ifdef USE_LOAD_LIBRARY
	mPhysics = (func0)(PX_PHYSICS_VERSION, *mFoundation, scale, false, mProfileZoneManager);
//	PxRegisterArticulations(*mPhysics);
//	PxRegisterHeightFields(*mPhysics);
	(func1)(*mPhysics);
#else
	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale, false, mProfileZoneManager);
#endif
	ASSERT(mPhysics);

	bool status = PxInitExtensions(*mPhysics);
	ASSERT(status);
	gDefaultCPUDispatcher = PxDefaultCpuDispatcherCreate(mParams.mNbThreads, null);

	{
		ASSERT(!mCooking);
		PxCookingParams Params;
	#ifdef USE_LOAD_LIBRARY
		mCooking = (func2)(PX_PHYSICS_VERSION, *mFoundation, Params);
	#else
		mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, Params);
	#endif
		ASSERT(mCooking);
	}

#ifdef SUPPORT_PVD
	if(mParams.mUsePVD)
	{
		gPVDHelper = new PVDHelper(mPhysics);

		gPVDHelper->togglePvdConnection(mParams);

		if(mPhysics->getPvdConnectionManager())
			mPhysics->getPvdConnectionManager()->addHandler(*gPVDHelper);
	}
#endif

	ASSERT(!mScene);
	PxSceneDesc sceneDesc(scale);
	sceneDesc.gravity					= ToPxVec3(desc.mGravity);
	sceneDesc.filterShader				= mParams.mUseCCD ? CCDSimulationFilterShader : PxDefaultSimulationFilterShader;
	sceneDesc.cpuDispatcher				= gDefaultCPUDispatcher;
#ifdef BETA2
	sceneDesc.maxBounds					= PxBounds3(PxVec3(-gGlobalBoxSize), PxVec3(gGlobalBoxSize));
	sceneDesc.upAxis					= 1;
#endif
	sceneDesc.staticStructure			= mParams.mStaticPruner;
	sceneDesc.dynamicStructure			= mParams.mDynamicPruner;
	sceneDesc.dynamicTreeRebuildRateHint= mParams.mSQDynamicRebuildRateHint;
#ifdef BETA2
	sceneDesc.num16KContactDataBlocks	= gNum16KContactDataBlocks;
#endif
	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_PCM,				mParams.mPCM);
	SetSceneFlag(sceneDesc, PxSceneFlag::eADAPTIVE_FORCE,			mParams.mAdaptiveForce);
//	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_STABILIZATION,		mParams.mStabilization);
	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_ACTIVETRANSFORMS,	mParams.mEnableActiveTransforms);
	SetSceneFlag(sceneDesc, PxSceneFlag::eDISABLE_CONTACT_CACHE,	!mParams.mEnableContactCache);
	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_SWEPT_INTEGRATION,	mParams.mUseCCD);

	SetSceneFlag(sceneDesc, PxSceneFlag::eDISABLE_SSE,						!mParams.mEnableSSE);
	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_ONE_DIRECTIONAL_FRICTION,	mParams.mEnableOneDirFriction);
	SetSceneFlag(sceneDesc, PxSceneFlag::eENABLE_TWO_DIRECTIONAL_FRICTION,	mParams.mEnableTwoDirFriction);

//	sceneDesc.ccdMaxPasses	= mParams.mMaxNbCCDPasses;

	mScene = mPhysics->createScene(sceneDesc);
	ASSERT(mScene);

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
	UpdateCommon(dt);
	return gDefaultAllocator->mCurrentMemory;
}

Point PhysX::GetMainColor()
{
	return Point(1.0f, 0.0f, 0.0f);
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
			shape = actor->createShape(PxSphereGeometry(SphereCreate->mRadius), *ShapeMaterial, LocalPose);
			ASSERT(shape);
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(CurrentShape);
			shape = actor->createShape(PxBoxGeometry(ToPxVec3(BoxCreate->mExtents)), *ShapeMaterial, LocalPose);
			ASSERT(shape);
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

			shape = actor->createShape(PxCapsuleGeometry(CapsuleCreate->mRadius, CapsuleCreate->mHalfHeight), *ShapeMaterial, LocalPose);
			ASSERT(shape);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);

			ASSERT(mCooking);
//			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX|PxConvexFlag::eINFLATE_CONVEX, CurrentShape->mRenderer);
			PxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX, CurrentShape->mRenderer);
			ASSERT(ConvexMesh);

			shape = actor->createShape(PxConvexMeshGeometry(ConvexMesh), *ShapeMaterial, LocalPose);
			ASSERT(shape);
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			ASSERT(mCooking);
			PxTriangleMesh* TriangleMesh = CreateTriangleMesh(MeshCreate->mSurface, CurrentShape->mRenderer);
			ASSERT(TriangleMesh);

			shape = actor->createShape(PxTriangleMeshGeometry(TriangleMesh), *ShapeMaterial, LocalPose);
			ASSERT(shape);
		}
		else ASSERT(0);

		if(shape)
		{
			SetupShape(mParams, CurrentShape, *shape, desc.mCollisionGroup, gDebugVizParams[0]);
			shape->setFlag(PxShapeFlag::eUSE_SWEPT_BOUNDS, mParams.mUseCCD);
		}

		CurrentShape = CurrentShape->mNext;
	}
}

/*PintObjectHandle PhysX::CreateObject(const PINT_OBJECT_CREATE& desc)
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

	CreateShapes(desc, actor);

	if(rigidDynamic)
		SetupDynamic(*rigidDynamic, desc);

	if(desc.mAddToWorld)
		mScene->addActor(*actor);

	return CreateHandle(actor);
}*/

static inline_ void FillResultStruct(PintRaycastHit& hit, const PxRaycastHit& result)
{
	hit.mObject			= CreateHandle(result.shape);
	hit.mImpact			= ToPoint(result.impact);
	hit.mNormal			= ToPoint(result.normal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceIndex;
}

udword PhysX::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;

	if(0)
	{
		static bool InitDone = false;
		static PxBatchQuery* BatchQuery = null;
		static PxBatchQueryDesc BatchQueryDesc;
		if(!InitDone)
		{
			InitDone = true;

			BatchQueryDesc.userRaycastResultBuffer	= new PxRaycastQueryResult[nb];
			BatchQueryDesc.userRaycastHitBuffer		= new PxRaycastHit[nb];
			BatchQueryDesc.raycastHitBufferSize		= nb;

			BatchQuery = mScene->createBatchQuery(BatchQueryDesc);
		}

		udword NbToGo = nb;
		while(nb--)
		{
			PxRaycastHit Hit;
			BatchQuery->raycastSingle(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, PF, sqFlags);
			raycasts++;
		}

		BatchQuery->execute();
//		BatchQuery->release();

		udword NbHits = 0;
		for(udword i=0;i<NbToGo;i++)
		{
			if(BatchQueryDesc.userRaycastResultBuffer[i].nbHits)
			{
				NbHits++;
				ASSERT(BatchQueryDesc.userRaycastResultBuffer[i].queryStatus==PxBatchQueryStatus::eSUCCESS);
				ASSERT(BatchQueryDesc.userRaycastResultBuffer[i].nbHits==1);
				FillResultStruct(*dest, *BatchQueryDesc.userRaycastResultBuffer[i].hits);
			}
			else
			{
				dest->mObject = null;
			}
			dest++;
		}
//		DELETEARRAY(BatchQueryDesc.userRaycastResultBuffer);
//		DELETEARRAY(BatchQueryDesc.userRaycastHitBuffer);
		return NbHits;
	}
	else
	{
//PxSceneQueryCache Cache;
		udword NbHits = 0;
		while(nb--)
		{
			PxRaycastHit Hit;
			if(mScene->raycastSingle(ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, sqFlags,
				Hit, PF
//				, null, Cache.shape ? &Cache : null
				))
			{
				NbHits++;
				FillResultStruct(*dest, Hit);
//Cache.shape	= Hit.shape;
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

static inline_ void FillResultStruct(PintRaycastHit& hit, const PxSweepHit& result)
{
	hit.mObject			= CreateHandle(result.shape);
	hit.mImpact			= ToPoint(result.impact);
	hit.mNormal			= ToPoint(result.normal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceIndex;
}

static inline_ PxSceneQueryFlags GetSweepQueryFlags(const EditableParams& params)
{
	return params.mSQInitialOverlap	?	PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE|PxSceneQueryFlag::eINITIAL_OVERLAP
									:	PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;
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


/*void PhysX::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		RigidActor = &Shape->getActor();
	}

	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		
		PxForceMode::Enum mode;
		if(action_type==PINT_ACTION_FORCE)
			mode = PxForceMode::eFORCE;
		else if(action_type==PINT_ACTION_IMPULSE)
			mode = PxForceMode::eIMPULSE;
		else ASSERT(0);

		PxRigidBodyExt::addForceAtPos(*RigidDynamic, ToPxVec3(action), ToPxVec3(pos), mode);
	}
}*/

void PhysX::AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		RigidActor = &Shape->getActor();
	}

	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		PxRigidBodyExt::addForceAtPos(*RigidDynamic, ToPxVec3(world_impulse), ToPxVec3(world_pos), PxForceMode::eIMPULSE);
	}
}

void PhysX::AddLocalTorque(PintObjectHandle handle, const Point& local_torque)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		RigidActor = &Shape->getActor();
	}

	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		const PxVec3 GlobalTorque = RigidDynamic->getGlobalPose().rotate(ToPxVec3(local_torque));
		RigidDynamic->addTorque(GlobalTorque, PxForceMode::eACCELERATION, true);
	}
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

#ifdef REMOVED

static Container*	gPhysXGUI = null;
static IceComboBox*	gComboBox_NbThreads = null;
static IceComboBox*	gComboBox_StaticPruner = null;
static IceComboBox*	gComboBox_DynamicPruner = null;
static IceEditBox*	gEditBox_SolverIterPos = null;
static IceEditBox*	gEditBox_SolverIterVel = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
#ifdef BETA2
static IceEditBox*	gEditBox_Num16KContactDataBlocks = null;
#endif
static IceEditBox*	gEditBox_GlobalBoxSize = null;
static IceEditBox*	gEditBox_ContactOffset = null;
static IceEditBox*	gEditBox_RestOffset = null;
static IceCheckBox*	gCheckBox_FullPVD = null;
static IceCheckBox*	gCheckBox_SQ_FilterOutAllShapes = null;
static IceCheckBox*	gCheckBox_SQ_InitialOverlap = null;
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
	PHYSX_GUI_SHARE_MESH_DATA,
	PHYSX_GUI_PCM,
	PHYSX_GUI_ADAPTIVE_FORCE,
	PHYSX_GUI_ENABLE_SSE,
	PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS,
	PHYSX_GUI_FLUSH_SIMULATION,
	PHYSX_GUI_DISABLE_STRONG_FRICTION,
	PHYSX_GUI_ENABLE_ONE_DIR_FRICTION,
	PHYSX_GUI_ENABLE_TWO_DIR_FRICTION,
	PHYSX_GUI_USE_PVD,
	PHYSX_GUI_USE_FULL_PVD_CONNECTION,
	//
	PHYSX_GUI_NB_THREADS,
	PHYSX_GUI_STATIC_PRUNER,
	PHYSX_GUI_DYNAMIC_PRUNER,
	//
	PHYSX_GUI_NB_SOLVER_ITER_POS,
	PHYSX_GUI_NB_SOLVER_ITER_VEL,
	PHYSX_GUI_LINEAR_DAMPING,
	PHYSX_GUI_ANGULAR_DAMPING,
	PHYSX_GUI_NUM_16K_CONTACT_DATA_BLOCKS,
	PHYSX_GUI_GLOBAL_BOX_SIZE,
	PHYSX_GUI_CONTACT_OFFSET,
	PHYSX_GUI_REST_OFFSET,
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
			if(gComboBox_StaticPruner)
				gComboBox_StaticPruner->SetEnabled(checked);
			if(gComboBox_DynamicPruner)
				gComboBox_DynamicPruner->SetEnabled(checked);
			break;
		case PHYSX_GUI_ENABLE_CCD:
			gUseCCD = checked;
			break;
		case PHYSX_GUI_SQ_FILTER_OUT:
			mParams.mSQFilterOutAllShapes = checked;
			break;
		case PHYSX_GUI_SQ_INITIAL_OVERLAP:
			gSQInitialOverlap = checked;
			break;
		case PHYSX_GUI_SHARE_MESH_DATA:
			gShareMeshData = checked;
			break;
		case PHYSX_GUI_PCM:
			gPCM = checked;
			break;
		case PHYSX_GUI_ADAPTIVE_FORCE:
			gAdaptiveForce = checked;
			break;
		case PHYSX_GUI_ENABLE_SSE:
			gEnableSSE = checked;
			break;
		case PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS:
			gEnableActiveTransforms = checked;
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

#ifdef BETA2
	if(gEditBox_Num16KContactDataBlocks)
	{
		sdword tmp;
		bool status = gEditBox_Num16KContactDataBlocks->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gNum16KContactDataBlocks = tmp;
	}
#endif

	if(gEditBox_GlobalBoxSize)
	{
		float tmp;
		bool status = gEditBox_GlobalBoxSize->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gGlobalBoxSize = tmp;
	}

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
}

IceWindow* PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gPhysXGUI, parent, PHYSX_GUI_MAIN, "PhysX 3.2 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gPhysXGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SQ, 4, y, CheckBoxWidth, 20, "Enable scene queries", gPhysXGUI, gSQFlag, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_SQ_FilterOutAllShapes = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_FILTER_OUT, 4, y, CheckBoxWidth, 20, "Filter out all shapes (DEBUG)", gPhysXGUI, mParams.mSQFilterOutAllShapes, gCheckBoxCallback);
		gCheckBox_SQ_FilterOutAllShapes->SetEnabled(gSQFlag);
		y += YStepCB;

		gCheckBox_SQ_InitialOverlap = helper.CreateCheckBox(Main, PHYSX_GUI_SQ_INITIAL_OVERLAP, 4, y, CheckBoxWidth, 20, "eINITIAL_OVERLAP flag (sweeps)", gPhysXGUI, gSQInitialOverlap, gCheckBoxCallback);
		gCheckBox_SQ_InitialOverlap->SetEnabled(gSQFlag);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gPhysXGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_SHARE_MESH_DATA, 4, y, CheckBoxWidth, 20, "Share mesh data", gPhysXGUI, gShareMeshData, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_PCM, 4, y, CheckBoxWidth, 20, "Enable PCM", gPhysXGUI, gPCM, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gPhysXGUI, gAdaptiveForce, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SSE, 4, y, CheckBoxWidth, 20, "Enable SSE", gPhysXGUI, gEnableSSE, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS, 4, y, CheckBoxWidth, 20, "Enable active transforms", gPhysXGUI, gEnableActiveTransforms, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_FLUSH_SIMULATION, 4, y, CheckBoxWidth, 20, "Flush simulation buffers", gPhysXGUI, gFlushSimulation, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_DISABLE_STRONG_FRICTION, 4, y, CheckBoxWidth, 20, "Disable strong friction", gPhysXGUI, gDisableStrongFriction, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_ONE_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable one dir. friction", gPhysXGUI, gEnableOneDirFriction, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_TWO_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable two dir. friction", gPhysXGUI, gEnableTwoDirFriction, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_USE_PVD, 4, y, CheckBoxWidth, 20, "Use PVD", gPhysXGUI, gUsePVD, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_FullPVD = helper.CreateCheckBox(Main, PHYSX_GUI_USE_FULL_PVD_CONNECTION, 4, y, CheckBoxWidth, 20, "Full connection", gPhysXGUI, gUseFullPvdConnection, gCheckBoxCallback);
		gCheckBox_FullPVD->SetEnabled(gUsePVD);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, PHYSX_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gPhysXGUI);

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
#ifdef BETA2
		gComboBox_StaticPruner->Add("eOCTREE");
		gComboBox_StaticPruner->Add("eQUADTREE");
#endif
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
#ifdef BETA2
		gComboBox_DynamicPruner->Add("eOCTREE");
		gComboBox_DynamicPruner->Add("eQUADTREE");
#endif
		gComboBox_DynamicPruner->Add("eDYNAMIC_AABB_TREE");
		gComboBox_DynamicPruner->Select(gDynamicPruner);
		gComboBox_DynamicPruner->SetVisible(true);
		gComboBox_DynamicPruner->SetEnabled(gSQFlag);
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

#ifdef BETA2
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num 16K blocks:", gPhysXGUI);
		gEditBox_Num16KContactDataBlocks = helper.CreateEditBox(Main, PHYSX_GUI_NUM_16K_CONTACT_DATA_BLOCKS, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gNum16KContactDataBlocks), gPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;
#endif

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "World bounds size:", gPhysXGUI);
		gEditBox_GlobalBoxSize = helper.CreateEditBox(Main, PHYSX_GUI_GLOBAL_BOX_SIZE, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gGlobalBoxSize), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Contact offset:", gPhysXGUI);
		gEditBox_ContactOffset = helper.CreateEditBox(Main, PHYSX_GUI_CONTACT_OFFSET, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gContactOffset), gPhysXGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Rest offset:", gPhysXGUI);
		gEditBox_RestOffset = helper.CreateEditBox(Main, PHYSX_GUI_REST_OFFSET, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gRestOffset), gPhysXGUI, EDITBOX_FLOAT, null);
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
	gEditBox_SolverIterPos = null;
	gEditBox_SolverIterVel = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
#ifdef BETA2
	gEditBox_Num16KContactDataBlocks = null;
#endif
	gEditBox_GlobalBoxSize = null;
	gEditBox_ContactOffset = null;
	gEditBox_RestOffset = null;
	gCheckBox_FullPVD = null;
	gCheckBox_SQ_FilterOutAllShapes = null;
	gCheckBox_SQ_InitialOverlap = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}
#endif

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
