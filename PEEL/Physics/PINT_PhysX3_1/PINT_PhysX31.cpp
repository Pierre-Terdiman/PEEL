///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// PhysX 3.1 DLLs:
// - PhysX3Cooking_x86.dll
// - PhysX3_x86.dll

#include "stdafx.h"
#include "PINT_PhysX31.h"
#include "..\PINT_Common\PINT_Common.h"

#include "extensions\PxExtensionsAPI.h"
#include "common/PxRenderBuffer.h"
/*#include "common/PxIO.h"
#include "physxprofilesdk/PxProfileZoneManager.h"
*/

#define SAFE_RELEASE(x)		if (x) { (x)->release();		(x) = null; }

//#include "..\PINT_Common\PINT_CommonPhysX3.h"
	inline_ Point	ToPoint(const PxVec3& p)	{ return Point(p.x, p.y, p.z);				}
	inline_ Quat	ToQuat(const PxQuat& q)		{ return Quat(q.w, q.x, q.y, q.z);			}
	inline_ PxVec3	ToPxVec3(const Point& p)	{ return PxVec3(p.x, p.y, p.z);				}
	inline_ PxQuat	ToPxQuat(const Quat& q)		{ return PxQuat(q.p.x, q.p.y, q.p.z, q.w);	}

#define BETA2

static PxQuat PxShortestRotation(const PxVec3& v0, const PxVec3& v1)
{
	const PxReal d = v0.dot(v1);
	const PxVec3 cross = v0.cross(v1);

	PxQuat q = d>-1 ? PxQuat(cross.x, cross.y, cross.z, 1+d) 
					: PxAbs(v0.x)<0.1f ? PxQuat(0.0f, v0.z, -v0.y, 0.0f) : PxQuat(v0.y, -v0.x, 0.0f, 0.0f);

	return q.getNormalized();
}
///////////////////////////////////////////////////////////////////////////////

static			float						gGlobalBoxSize					= 10000.0f;
static			udword						gNbThreads						= 0;
static			PxPruningStructure::Enum	gStaticPruner					= PxPruningStructure::eSTATIC_AABB_TREE;
static			PxPruningStructure::Enum	gDynamicPruner					= PxPruningStructure::eDYNAMIC_AABB_TREE;
static			udword						gSolverIterationCountPos		= 4;
static			udword						gSolverIterationCountVel		= 1;
static			bool						gSQFlag							= true;
static			bool						gSQFilterOutAllShapes			= false;
static			bool						gSQInitialOverlap				= false;
static			bool						gEnableSleeping					= false;
static			bool						gShareMeshData					= true;
static			bool						gAdaptiveForce					= false;
static			bool						gEnableSSE						= true;
static			bool						gDisableStrongFriction			= false;
static	const	bool						gEnableCollisionBetweenJointed	= false;
static			float						gLinearDamping					= 0.1f;
static			float						gAngularDamping					= 0.05f;
#ifdef BETA2
static			udword						gNum16KContactDataBlocks		= 256;
#endif
static			bool						gUsePVD							= false;
static			bool						gUseFullPvdConnection			= true;
static			bool						gUseCCD							= false;

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

static inline_	PintObjectHandle	CreateHandle(PxRigidActor* actor)
{
	const size_t binary = size_t(actor);
	ASSERT(!(binary&1));
	return PintObjectHandle(binary);
}

static inline_	PintObjectHandle	CreateHandle(PxShape* shape)
{
	const size_t binary = size_t(shape);
	ASSERT(!(binary&1));
	return PintObjectHandle(binary|1);
}

static inline_	PxRigidActor*		GetActorFromHandle(PintObjectHandle handle)
{
	const size_t binary = size_t(handle);
	return (binary & 1) ? null : (PxRigidActor*)binary;
}

static inline_	PxShape*			GetShapeFromHandle(PintObjectHandle handle)
{
	const size_t binary = size_t(handle);
	return (binary & 1) ? (PxShape*)(binary&~1) : null;
}

///////////////////////////////////////////////////////////////////////////////

#ifdef BETA2
// ### calling the PxExtensions version crashes, for some reason
static void _PxSetGroup(const PxRigidActor& actor, const PxU16 collisionGroup)
{
	PX_ASSERT(collisionGroup < 32);

	PxFilterData fd;
	
	if (actor.getNbShapes() == 1)
	{
		PxShape* shape = NULL;
		actor.getShapes(&shape, 1);

		// retrieve current group mask
		fd = shape->getSimulationFilterData();
		fd.word0 = collisionGroup;
		
		// set new filter data
		shape->setSimulationFilterData(fd);
	}
	else
	{
		PxShape** shapes;
//		shapes = PX_NEW(PxShape*)[actor.getNbShapes()];
		shapes = new PxShape*[actor.getNbShapes()];
		actor.getShapes(shapes, actor.getNbShapes());

		for(PxU32 i=0; i < actor.getNbShapes(); i++)
		{
			// retrieve current group mask
			fd = shapes[i]->getSimulationFilterData();
			fd.word0 = collisionGroup;
			
			// set new filter data
			shapes[i]->setSimulationFilterData(fd);
		}

//		PX_DELETE_ARRAY(shapes);
		delete [] shapes;
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////

//#define SUPPORT_PVD

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
#ifdef BETA2
	TConnectionFlagsType theConnectionFlags( PvdConnectionType::Debug | PvdConnectionType::Profile | PvdConnectionType::Memory );
	if(!gUseFullPvdConnection)
		theConnectionFlags = TConnectionFlagsType(PvdConnectionType::Profile);
#else
	PxVisualDebuggerConnectionFlags theConnectionFlags( PxVisualDebuggerExt::getAllConnectionFlags() );
	if(!gUseFullPvdConnection)
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

	class MyErrorCallback : public PxErrorCallback
	{
	public:
		MyErrorCallback()	{}
		~MyErrorCallback()	{}

		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
		{
			printf(message);
		}
	};

	class MyAllocator : public PxAllocatorCallback
	{
	public:
		struct Header
		{
			udword	mMagic;
			udword	mSize;
		};

		MyAllocator() :
			mNbAllocs		(0),
			mCurrentMemory	(0)
		{
		}
		~MyAllocator()
		{
			if(mNbAllocs)
				printf("PhysX 3.2: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
		}

		void* allocate(size_t size, const char*, const char*, int)
		{
//return _aligned_malloc(size, 16);
			char* memory = (char*)_aligned_malloc(size+16, 16);
			Header* H = (Header*)memory;
			H->mMagic = 0x12345678;
			H->mSize = size;
			mNbAllocs++;
			mCurrentMemory+=size;
			return memory + 16;
		}

		void deallocate(void* ptr)
		{
//_aligned_free(ptr);return;
			if(!ptr)
				return;
			char* bptr = (char*)ptr;
			Header* H = (Header*)(bptr - 16);
			ASSERT(H->mMagic==0x12345678);
			const udword Size = H->mSize;
			_aligned_free(H);
			mNbAllocs--;
			mCurrentMemory-=Size;
		}

		udword	mNbAllocs;
		udword	mCurrentMemory;
	};

//static PxDefaultAllocator* gDefaultAllocator = null;
//static PxDefaultErrorCallback* gDefaultErrorCallback = null;

static MyAllocator* gDefaultAllocator = null;
//static PxAllocatorCallback* gDefaultAllocator = null;
static PxErrorCallback* gDefaultErrorCallback = null;

static PxDefaultCpuDispatcher* gDefaultCPUDispatcher = null;
#ifdef SUPPORT_PVD
static PVDHelper* gPVDHelper = null;
#endif

///////////////////////////////////////////////////////////////////////////////

	class MemoryOutputStream : public PxStream
	{
	public:
						MemoryOutputStream();
	virtual				~MemoryOutputStream();

			PxU32		write(const void* src, PxU32 count);

	virtual	PxU8		readByte()								const	{ ASSERT(0);	return 0;	}
	virtual	PxU16		readWord()								const	{ ASSERT(0);	return 0;	}
	virtual	PxU32		readDword()								const	{ ASSERT(0);	return 0;	}
	virtual	PxF32		readFloat()								const	{ ASSERT(0);	return 0.0f;}
	virtual	PxF64		readDouble()							const	{ ASSERT(0);	return 0.0;	}
	virtual	void		readBuffer(void* buffer, PxU32 size)	const	{ ASSERT(0);				}
	
	virtual	PxStream&	storeByte(PxU8 b)								{ write(&b, sizeof(PxU8));	return *this;	}
	virtual	PxStream&	storeWord(PxU16 w)								{ write(&w, sizeof(PxU16));	return *this;	}
	virtual	PxStream&	storeDword(PxU32 d)								{ write(&d, sizeof(PxU32));	return *this;	}
	virtual	PxStream&	storeFloat(PxF32 f)								{ write(&f, sizeof(PxF32));	return *this;	}
	virtual	PxStream&	storeDouble(PxF64 f)							{ write(&f, sizeof(PxF64));	return *this;	}
	virtual	PxStream&	storeBuffer(const void* buffer, PxU32 size)		{ write(buffer, size);	return *this;		}

			PxU32		getSize()	const	{	return mSize; }
			PxU8*		getData()	const	{	return mData; }
	private:
			PxU8*		mData;
			PxU32		mSize;
			PxU32		mCapacity;
	};

MemoryOutputStream::MemoryOutputStream() :
	mData		(NULL),
	mSize		(0),
	mCapacity	(0)
{
}

MemoryOutputStream::~MemoryOutputStream()
{
	if(mData)
		delete[] mData;
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 size)
{
	PxU32 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
		mCapacity = expectedSize + 4096;

		PxU8* newData = new PxU8[mCapacity];
		PX_ASSERT(newData!=NULL);

		if(newData)
		{
			memcpy(newData, mData, mSize);
			delete[] mData;
		}
		mData = newData;
	}
	memcpy(mData+mSize, src, size);
	mSize += size;
	return size;
}

	class MemoryInputData : public PxStream
	{
	public:
						MemoryInputData(PxU8* data, PxU32 length);

			PxU32		read(void* dest, PxU32 count)	const;
			PxU32		getLength() const;
			void		seek(PxU32 pos);
			PxU32		tell() const;

	virtual	PxU8		readByte()								const	{ PxU8 data;	read(&data, sizeof(PxU8));	return data;	}
	virtual	PxU16		readWord()								const	{ PxU16 data;	read(&data, sizeof(PxU16));	return data;	}
	virtual	PxU32		readDword()								const	{ PxU32 data;	read(&data, sizeof(PxU32));	return data;	}
	virtual	PxF32		readFloat()								const	{ PxF32 data;	read(&data, sizeof(PxF32));	return data;	}
	virtual	PxF64		readDouble()							const	{ PxF64 data;	read(&data, sizeof(PxF64));	return data;	}
	virtual	void		readBuffer(void* buffer, PxU32 size)	const	{ read(buffer, size);	}
	
	virtual	PxStream&	storeByte(PxU8 b)								{ ASSERT(0);	return *this;	}
	virtual	PxStream&	storeWord(PxU16 w)								{ ASSERT(0);	return *this;	}
	virtual	PxStream&	storeDword(PxU32 d)								{ ASSERT(0);	return *this;	}
	virtual	PxStream&	storeFloat(PxF32 f)								{ ASSERT(0);	return *this;	}
	virtual	PxStream&	storeDouble(PxF64 f)							{ ASSERT(0);	return *this;	}
	virtual	PxStream&	storeBuffer(const void* buffer, PxU32 size)		{ ASSERT(0);	return *this;	}

	private:
			PxU32		mSize;
			const PxU8*	mData;
	mutable	PxU32		mPos;
	};

MemoryInputData::MemoryInputData(PxU8* data, PxU32 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU32 MemoryInputData::read(void* dest, PxU32 count) const
{
	PxU32 length = PxMin<PxU32>(count, mSize-mPos);
	memcpy(dest, mData+mPos, length);
	mPos += length;
	return length;
}

PxU32 MemoryInputData::getLength() const
{
	return mSize;
}

void MemoryInputData::seek(PxU32 offset)
{
	mPos = PxMin<PxU32>(mSize, offset);
}

PxU32 MemoryInputData::tell() const
{
	return mPos;
}

PhysX::PhysX() :
	mFoundation			(null),
//	mProfileZoneManager	(null),
	mPhysics			(null),
	mCooking			(null),
	mScene				(null),
	mDefaultMaterial	(null),
	mScratchPad			(null)
{
}

PhysX::~PhysX()
{
	ASSERT(!gDefaultCPUDispatcher);
	ASSERT(!gDefaultErrorCallback);
	ASSERT(!gDefaultAllocator);
//	ASSERT(!mFoundation);
	ASSERT(!mPhysics);
	ASSERT(!mCooking);
	ASSERT(!mScene);
	ASSERT(!mDefaultMaterial);
//	ASSERT(!mProfileZoneManager);
	ASSERT(!mScratchPad);
}

void PhysX::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
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
	caps.mSupportSphereOverlaps			= true;
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

	pairFlags = PxPairFlag::eCONTACT_DEFAULT|PxPairFlag::eSWEPT_INTEGRATION_LINEAR;

	return PxFilterFlags();
}

void PhysX::Init(const PINT_WORLD_CREATE& desc)
{
//	gDefaultAllocator = new PxDefaultAllocator;
//	gDefaultErrorCallback = new PxDefaultErrorCallback;
	gDefaultAllocator = new MyAllocator;
	gDefaultErrorCallback = new MyErrorCallback;

/*	ASSERT(!mFoundation);
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gDefaultAllocator, *gDefaultErrorCallback);
//	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, *gDefaultAllocator, gMyErrorCallback);
	ASSERT(mFoundation);*/

#ifdef SUPPORT_PVD
	if(gUsePVD)
	{
		ASSERT(!mProfileZoneManager);
		mProfileZoneManager = &PxProfileZoneManager::createProfileZoneManager(mFoundation);
		ASSERT(mProfileZoneManager);
	}
#endif

	ASSERT(!mPhysics);
	PxTolerancesScale scale;
	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gDefaultAllocator, *gDefaultErrorCallback, scale, false);
	ASSERT(mPhysics);
	mFoundation = &mPhysics->getFoundation();

	bool status = PxInitExtensions(*mPhysics);
	ASSERT(status);
	gDefaultCPUDispatcher = PxDefaultCpuDispatcherCreate(gNbThreads, null);

	ASSERT(!mCooking);
	PxCookingParams Params;
	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, mFoundation, Params);
	ASSERT(mCooking);

#ifdef SUPPORT_PVD
	if(gUsePVD)
	{
		gPVDHelper = new PVDHelper(mPhysics);

		gPVDHelper->togglePvdConnection();

		if(mPhysics->getPvdConnectionManager())
			mPhysics->getPvdConnectionManager()->addHandler(*gPVDHelper);
	}
#endif

	ASSERT(!mScene);
	PxSceneDesc sceneDesc(scale);
	sceneDesc.gravity					= ToPxVec3(desc.mGravity);
	sceneDesc.filterShader				= gUseCCD ? CCDSimulationFilterShader : PxDefaultSimulationFilterShader;
	sceneDesc.cpuDispatcher				= gDefaultCPUDispatcher;
#ifdef BETA2
	sceneDesc.maxBounds					= PxBounds3(PxVec3(-gGlobalBoxSize), PxVec3(gGlobalBoxSize));
	sceneDesc.upAxis					= 1;
#endif
	sceneDesc.staticStructure			= gStaticPruner;
	sceneDesc.dynamicStructure			= gDynamicPruner;
//	sceneDesc.dynamicTreeRebuildRateHint= 10;
#ifdef BETA2
	sceneDesc.num16KContactDataBlocks	= gNum16KContactDataBlocks;
#endif
	if(gAdaptiveForce)
		sceneDesc.flags					|= PxSceneFlag::eADAPTIVE_FORCE;
	if(!gEnableSSE)
		sceneDesc.flags					|= PxSceneFlag::eDISABLE_SSE;
	if(gUseCCD)
		sceneDesc.flags					|= PxSceneFlag::eENABLE_SWEPT_INTEGRATION;

	mScene = mPhysics->createScene(sceneDesc);
	ASSERT(mScene);

//	mScene->setDynamicTreeRebuildRateHint(10);
//	mScene->setDynamicTreeRebuildRateHint(5);

	// Create default material
	{
		PINT_MATERIAL_CREATE Desc;
		Desc.mStaticFriction	= 0.5f;
		Desc.mDynamicFriction	= 0.5f;
		Desc.mRestitution		= 0.0f;
		mDefaultMaterial = CreateMaterial(Desc);
		ASSERT(mDefaultMaterial);
	}

	UpdateFromUI();

#ifdef SCRATCHPAD_SIZE
	mScratchPad = _aligned_malloc(SCRATCHPAD_SIZE, 16);
#endif
}

void PhysX::SetGravity(const Point& gravity)
{
	ASSERT(mScene);
	mScene->setGravity(ToPxVec3(gravity));
}

void PhysX::Close()
{
	_aligned_free(mScratchPad);

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
//	SAFE_RELEASE(mProfileZoneManager)
//	SAFE_RELEASE(mFoundation)
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
#ifdef SCRATCHPAD_SIZE
		mScene->simulate(dt, NULL, mScratchPad, SCRATCHPAD_SIZE);
#else
		mScene->simulate(dt);
#endif
		mScene->fetchResults(true);
//		mScene->flush();
	}
	return gDefaultAllocator->mCurrentMemory;
//	return 0;
}

Point PhysX::GetMainColor()
{
	return Point(0.75f, 0.25f, 0.5f);
}

void PhysX::Render(PintRender& renderer)
{
	if(mScene)
	{
		const PxActorTypeSelectionFlags selectionFlags = PxActorTypeSelectionFlag::eRIGID_STATIC | PxActorTypeSelectionFlag::eRIGID_DYNAMIC;
		const PxU32 nbActors = mScene->getNbActors(selectionFlags);

//		PxActor* buffer[64];
		PxActor** buffer = new PxActor*[nbActors];
//		PxU32 nbProcessed = 0;
//		while(nbProcessed!=nbActors)
		{
//			const PxU32 Remain = nbActors - nbProcessed;
//			const PxU32 NbToGo = MIN(Remain, 64);
//			/*const PxU32 nb =*/ mScene->getActors(selectionFlags, buffer, NbToGo, nbProcessed);
//			const PxU32 nb = NbToGo;
//			nbProcessed += nb;

			mScene->getActors(selectionFlags, buffer, nbActors, 0);

//			for(PxU32 i=0;i<nb;i++)
			for(PxU32 i=0;i<nbActors;i++)
			{
				PxActor* actor = buffer[i];
				const PxActorType::Enum type = actor->getType();
				if(type==PxActorType::eRIGID_STATIC || type==PxActorType::eRIGID_DYNAMIC)
				{
					PxRigidActor* rigidActor = static_cast<PxRigidActor*>(actor);

//					const PxTransform pose = rigidActor->getGlobalPose();

					PxU32 nbShapes = rigidActor->getNbShapes();
					for(PxU32 j=0;j<nbShapes;j++)
					{
						PxShape* shape = null;
						PxU32 nb = rigidActor->getShapes(&shape, 1, j);
						ASSERT(nb==1);
						ASSERT(shape);

						const PxTransform Pose = PxShapeExt::getGlobalPose(*shape);
						const PR IcePose(ToPoint(Pose.p), ToQuat(Pose.q));

						ASSERT(shape->userData);
						if(shape->userData)
						{
							PintShapeRenderer* shapeRenderer = (PintShapeRenderer*)shape->userData;

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
		DELETEARRAY(buffer);

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
	}
}

PxMaterial* PhysX::CreateMaterial(const PINT_MATERIAL_CREATE& desc)
{
	const PxU32 NbMaterials = mMaterials.size();
	for(PxU32 i=0;i<NbMaterials;i++)
	{
		PxMaterial* M = mMaterials[i];
		if(		M->getRestitution()==desc.mRestitution
			&&	M->getStaticFriction()==desc.mStaticFriction
			&&	M->getDynamicFriction()==desc.mDynamicFriction)
		{
			return M;
		}
	}

	ASSERT(mPhysics);
	PxMaterial* M = mPhysics->createMaterial(desc.mStaticFriction, desc.mDynamicFriction, desc.mRestitution);
	ASSERT(M);
	if(gDisableStrongFriction)
		M->setFlags(PxMaterialFlag::eDISABLE_STRONG_FRICTION);
//	M->setFrictionCombineMode(PxCombineMode::eMIN);
//	M->setRestitutionCombineMode(PxCombineMode::eMIN);
	PxCombineMode::Enum defMode = M->getFrictionCombineMode();
	mMaterials.push_back(M);
	return M;
}

PxConvexMesh* PhysX::CreateConvexMesh(const Point* verts, udword vertCount, PxConvexFlags flags, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mPhysics);

	if(gShareMeshData && renderer)
	{
		const udword Size = mConvexes.size();
		for(udword i=0;i<Size;i++)
		{
			const ConvexRender& CurrentConvex = mConvexes[i];
			if(CurrentConvex.mRenderer==renderer)
			{
//				printf("Sharing convex mesh\n");
				return CurrentConvex.mConvexMesh;
			}
		}
	}

	PxConvexMeshDesc ConvexDesc;
	ConvexDesc.points.count		= vertCount;
	ConvexDesc.points.stride	= sizeof(PxVec3);
	ConvexDesc.points.data		= verts;
	ConvexDesc.flags			= flags;

	MemoryOutputStream buf;
	if(!mCooking->cookConvexMesh(ConvexDesc, buf))
		return null;

	MemoryInputData input(buf.getData(), buf.getSize());
	PxConvexMesh* NewConvex = mPhysics->createConvexMesh(input);

	if(renderer)
	{
		ConvexRender NewConvexRender;
		NewConvexRender.mConvexMesh = NewConvex;
		NewConvexRender.mRenderer = renderer;
		mConvexes.push_back(NewConvexRender);
	}
	return NewConvex;
}

PxTriangleMesh* PhysX::CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mPhysics);

	if(gShareMeshData && renderer)
	{
		const udword Size = mMeshes.size();
		for(udword i=0;i<Size;i++)
		{
			const MeshRender& CurrentMesh = mMeshes[i];
			if(CurrentMesh.mRenderer==renderer)
			{
				return CurrentMesh.mTriangleMesh;
			}
		}
	}

	PxTriangleMeshDesc MeshDesc;
	MeshDesc.points.count		= surface.mNbVerts;
	MeshDesc.points.stride		= sizeof(PxVec3);
	MeshDesc.points.data		= surface.mVerts;
	MeshDesc.triangles.count	= surface.mNbFaces;
	MeshDesc.triangles.stride	= sizeof(udword)*3;
	MeshDesc.triangles.data		= surface.mDFaces;
//	MeshDesc.flags				= PxMeshFlag::eFLIPNORMALS;
//	MeshDesc.flags				= 0;

	MemoryOutputStream buf;
	if(!mCooking->cookTriangleMesh(MeshDesc, buf))
		return null;

	MemoryInputData input(buf.getData(), buf.getSize());
	PxTriangleMesh* NewMesh = mPhysics->createTriangleMesh(input);

	if(renderer)
	{
		MeshRender NewMeshRender;
		NewMeshRender.mTriangleMesh = NewMesh;
		NewMeshRender.mRenderer = renderer;
		mMeshes.push_back(NewMeshRender);
	}
	return NewMesh;
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
//			shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, gSQFlag);
			shape->setFlag(PxShapeFlag::eVISUALIZATION, gDebugVizParams[0]);
			shape->setFlag(PxShapeFlag::eUSE_SWEPT_BOUNDS, gUseCCD);

			// Setup query filter data so that we can filter out all shapes - debug purpose
			if(gSQFlag)
				shape->setQueryFilterData(PxFilterData(1, 0, 0, 0));

			if(CurrentShape->mRenderer)
				shape->userData = CurrentShape->mRenderer;
		}

		CurrentShape = CurrentShape->mNext;
	}

	if(rigidDynamic)
	{
		rigidDynamic->setLinearDamping(gLinearDamping);
		rigidDynamic->setAngularDamping(gAngularDamping);
		rigidDynamic->setLinearVelocity(ToPxVec3(desc.mLinearVelocity));
		rigidDynamic->setAngularVelocity(ToPxVec3(desc.mAngularVelocity));
		rigidDynamic->setMaxAngularVelocity(100.0f);
//		PhysX3::SetMassProperties(desc, *rigidDynamic);
		{
			bool status = PxRigidBodyExt::setMassAndUpdateInertia(*rigidDynamic, desc.mMass);
			ASSERT(status);

			if(desc.mCOMLocalOffset.IsNonZero())
			{
				PxTransform Pose = rigidDynamic->getCMassLocalPose();
				Pose.p += ToPxVec3(desc.mCOMLocalOffset);
				rigidDynamic->setCMassLocalPose(Pose);
			}
		}

		if(!gEnableSleeping)
			rigidDynamic->wakeUp(9999999999.0f);
		rigidDynamic->setSolverIterationCounts(gSolverIterationCountPos, gSolverIterationCountVel);
	}

#ifdef BETA2
	_PxSetGroup(*actor, desc.mCollisionGroup);
#else
	PxSetGroup(*actor, desc.mCollisionGroup);
#endif

	if(desc.mAddToWorld)
		mScene->addActor(*actor);

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
		RigidActor = &Shape->getActor();
		RigidActor->release();
		return true;
	}
	ASSERT(0);
	return false;
}

PintJointHandle PhysX::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mPhysics);

	PxRigidActor* actor0 = (PxRigidActor*)desc.mObject0;
	PxRigidActor* actor1 = (PxRigidActor*)desc.mObject1;

	PxJoint* CreatedJoint = null;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			// ### what's the point of having a rotation for pivots here?
			PxSphericalJoint* j = PxSphericalJointCreate(*mPhysics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
			ASSERT(j);
			CreatedJoint = j;
		}
		break;
		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

//			PxTransform pose0 = PxTransformFromSegment(PxVec3(0.0f), ToPxVec3(jc.mLocalAxis0));
//			PxTransform pose1 = PxTransformFromSegment(PxVec3(0.0f), ToPxVec3(jc.mLocalAxis1));

			// ### which one??
			const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
			const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
//			const PxQuat q0 = PxShortestRotation(ToPxVec3(jc.mLocalAxis0), PxVec3(1.0f, 0.0f, 0.0f));
//			const PxQuat q1 = PxShortestRotation(ToPxVec3(jc.mLocalAxis0), PxVec3(1.0f, 0.0f, 0.0f));

			PxRevoluteJoint* j = PxRevoluteJointCreate(*mPhysics,	actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0),
																	actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
			ASSERT(j);

			if(!jc.mGlobalAnchor.IsNotUsed() && !jc.mGlobalAxis.IsNotUsed())
			{
				const PxVec3 GlobalAnchor = ToPxVec3(jc.mGlobalAnchor);
				const PxVec3 GlobalAxis = ToPxVec3(jc.mGlobalAxis);
				PxSetJointGlobalFrame(*j, &GlobalAnchor, &GlobalAxis);
			}

			// ### what about axes?
//	const PxQuat q = Ps::computeQuatFromNormal(up);
//	const PxQuat q = Ps::rotationArc(PxVec3(1.0f, 0.0f, 0.0f), up);

			if(0)
			{
				// ### really tedious to setup!
				const PxTransform m0 = actor0->getGlobalPose();
				const PxTransform m1 = actor1->getGlobalPose();
				PxVec3 wsAnchor;
				{
					PxVec3 wp0 = m0.transform(ToPxVec3(jc.mLocalPivot0));
					PxVec3 wp1 = m1.transform(ToPxVec3(jc.mLocalPivot1));
					wsAnchor = (wp0+wp1)*0.5f;
				}
				PxVec3 wsAxis;
				{
					PxVec3 wp0 = m0.rotate(ToPxVec3(jc.mLocalAxis0));
					PxVec3 wp1 = m1.rotate(ToPxVec3(jc.mLocalAxis1));
					wsAxis = (wp0+wp1)*0.5f; 
					wsAxis.normalize();
				}
				PxSetJointGlobalFrame(*j, &wsAnchor, &wsAxis);
			}

			if(jc.mMinLimitAngle!=MIN_FLOAT || jc.mMaxLimitAngle!=MAX_FLOAT)
			{
				const float limitContactDistance = 0.05f;
//				const float limitContactDistance = 0.0f;

//				PxJointLimitPair limit(-PxPi/2, PxPi/2, 10.0f);
//				PxJointLimitPair limit(-PxPi/2, PxPi/2, 0.05f);
				PxJointLimitPair limit(0.0f, 0.0f, limitContactDistance);
//				limit.restitution	= 0.0f;
				//### wtf
				limit.lower			= -jc.mMaxLimitAngle;
				limit.upper			= -jc.mMinLimitAngle;
//limit.lower			= -degToRad(45.0f);
//limit.upper			= 0.0f;

				j->setLimit(limit);
//		j->setConstraintFlags(PxConstraintFlag::ePROJECTION);
//		j->setProjectionLinearTolerance(0.1f);

//				j->setLimit(PxJointLimitPair(jc.mMinLimitAngle, jc.mMaxLimitAngle, TWOPI));
				j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
//				j->setRevoluteJointFlags(PxRevoluteJointFlag::eLIMIT_ENABLED);
			}

/*
		PxRevoluteJoint* rv = PxRevoluteJointCreate(physics, b0->mBody, PxTransform::createIdentity(), b1->mBody, PxTransform::createIdentity());
		mJoints[i] = rv;
		rv->setConstraintFlags(PxConstraintFlag::ePROJECTION);
		rv->setProjectionLinearTolerance(0.1f);
		if(1)
		{
			PxJointLimitPair limit(-PxPi/2, PxPi/2, 0.05f);
			limit.restitution	= 0.0f;
			limit.lower			= -0.2f;
			limit.upper			= 0.2f;
			rv->setLimit(limit);

			rv->setRevoluteJointFlags(PxRevoluteJointFlag::eLIMIT_ENABLED);
		}

		PxSetJointGlobalFrame(*rv, &globalAnchor, &globalAxis);
*/

//			j->setConstraintFlags(PxConstraintFlag::ePROJECTION);
//			j->setProjectionLinearTolerance(0.1f);

			CreatedJoint = j;
		}
		break;
		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			// ### what's the point of having a rotation for pivots here?
			PxFixedJoint* j = PxFixedJointCreate(*mPhysics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
			ASSERT(j);
			CreatedJoint = j;

//			j->setConstraintFlags(PxConstraintFlag::ePROJECTION);
//			j->setProjectionLinearTolerance(0.1f);
		}
		break;
		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
			const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));

			PxPrismaticJoint* j = PxPrismaticJointCreate(*mPhysics,	actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0),
																	actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
			ASSERT(j);
			CreatedJoint = j;
		}
		break;
	}

	if(CreatedJoint)
	{
		if(gEnableCollisionBetweenJointed)
			CreatedJoint->setConstraintFlags(PxConstraintFlag::eCOLLISION_ENABLED);
	}
	return CreatedJoint;
}

void PhysX::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
//	for(udword i=0;i<nb_groups;i++)
//		PxSetGroupCollisionFlag(groups[i].mGroup0, groups[i].mGroup1, false);
}

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

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sqFlags = PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;

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

static inline_ PxSceneQueryFlags GetSweepQueryFlags()
{
	return gSQInitialOverlap	?	PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE|PxSceneQueryFlag::eINITIAL_OVERLAP
								:	PxSceneQueryFlag::eIMPACT|PxSceneQueryFlag::eNORMAL|PxSceneQueryFlag::eDISTANCE;
}

udword PhysX::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);
	const PxSceneQueryFlags sweepQueryFlags = GetSweepQueryFlags();

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

udword PhysX::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxSceneQueryFilterData PF(PxFilterData(!gSQFilterOutAllShapes, gSQFilterOutAllShapes, 0, 0), PxSceneQueryFilterFlag::eDYNAMIC | PxSceneQueryFilterFlag::eSTATIC);

	udword NbHits = 0;
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		PxShape* Hit;
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
	while(nb--)
	{
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());

		PxShape* Hits[256];
		PxI32 Nb = mScene->overlapMultiple(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hits, 256, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

PR PhysX::GetWorldTransform(PintObjectHandle handle)
{
	PxTransform Pose;

	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(RigidActor)
	{
		Pose = RigidActor->getGlobalPose();
	}
	else
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		Pose = PxShapeExt::getGlobalPose(*Shape);
	}

	return PR(ToPoint(Pose.p), ToQuat(Pose.q));
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

	if(RigidActor->getType()==PxActorType::eRIGID_DYNAMIC)
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

	if(RigidActor->getType()==PxActorType::eRIGID_DYNAMIC)
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

	if(RigidActor->getType()==PxActorType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		const PxVec3 GlobalTorque = RigidDynamic->getGlobalPose().rotate(ToPxVec3(local_torque));
		RigidDynamic->addTorque(GlobalTorque, PxForceMode::eACCELERATION, true);
	}
}

/*float PhysX::GetMass(PintObjectHandle handle)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(!RigidBody)
		return 0.0f;
	return RigidBody->getMass();
}

Point PhysX::GetLocalInertia(PintObjectHandle handle)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(!RigidBody)
		return Point(0.0f, 0.0f, 0.0f);
	return ToPoint(RigidBody->getMassSpaceInertiaTensor());
}*/

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

//	for(PxU16 j=0;j<32;j++)
//		for(PxU16 i=0;i<32;i++)
//			PxSetGroupCollisionFlag(i, j, true);

	ASSERT(!gPhysX);
	gPhysX = ICE_NEW(PhysX);
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
static IceEditBox*	gEditBox_SolverIterPos = null;
static IceEditBox*	gEditBox_SolverIterVel = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
#ifdef BETA2
static IceEditBox*	gEditBox_Num16KContactDataBlocks = null;
#endif
static IceEditBox*	gEditBox_GlobalBoxSize = null;
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
	PHYSX_GUI_ADAPTIVE_FORCE,
	PHYSX_GUI_ENABLE_SSE,
	PHYSX_GUI_DISABLE_STRONG_FRICTION,
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
			gSQFilterOutAllShapes = checked;
			break;
		case PHYSX_GUI_SQ_INITIAL_OVERLAP:
			gSQInitialOverlap = checked;
			break;
		case PHYSX_GUI_SHARE_MESH_DATA:
			gShareMeshData = checked;
			break;
		case PHYSX_GUI_ADAPTIVE_FORCE:
			gAdaptiveForce = checked;
			break;
		case PHYSX_GUI_ENABLE_SSE:
			gEnableSSE = checked;
			break;
		case PHYSX_GUI_DISABLE_STRONG_FRICTION:
			gDisableStrongFriction = checked;
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
}

IceWindow* PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gPhysXGUI, parent, PHYSX_GUI_MAIN, "PhysX 3.1 options");

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

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gPhysXGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_SHARE_MESH_DATA, 4, y, CheckBoxWidth, 20, "Share mesh data", gPhysXGUI, gShareMeshData, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gPhysXGUI, gAdaptiveForce, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SSE, 4, y, CheckBoxWidth, 20, "Enable SSE", gPhysXGUI, gEnableSSE, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_DISABLE_STRONG_FRICTION, 4, y, CheckBoxWidth, 20, "Disable strong friction", gPhysXGUI, gDisableStrongFriction, gCheckBoxCallback);
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
	gCheckBox_FullPVD = null;
	gCheckBox_SQ_FilterOutAllShapes = null;
	gCheckBox_SQ_InitialOverlap = null;
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
