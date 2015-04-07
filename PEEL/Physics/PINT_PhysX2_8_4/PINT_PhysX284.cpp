///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_PhysX284.h"
#include "..\PINT_Common\PINT_Common.h"

#include "NxPhysics.h"
#include "NxCooking.h"
#include "NxStream.h"

///////////////////////////////////////////////////////////////////////////////

static			float				gGlobalBoxSize					= 10000.0f;
static			NxPruningStructure	gDynamicPruner					= NX_PRUNING_DYNAMIC_AABB_TREE;
static			bool				gAdaptiveForce					= true;
static	const	bool				gEnableCollisionBetweenJointed	= false;
static			bool				gSQFlag							= true;
static			udword				gSolverIterationCount			= 4;
static			bool				gEnableSleeping					= false;
static			bool				gShareMeshData					= true;
static			float				gLinearDamping					= 0.1f;
static			float				gAngularDamping					= 0.05f;

#define	NB_DEBUG_VIZ_PARAMS	17
static			bool				gDebugVizParams[NB_DEBUG_VIZ_PARAMS] = {0};
static	const	char*				gDebugVizNames[NB_DEBUG_VIZ_PARAMS] =
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
	"Visualize collision free",
	"Visualize joint local frames",
	"Visualize joint limits",
};
static			NxParameter	gDebugVizIndex[NB_DEBUG_VIZ_PARAMS] =
{
	NX_VISUALIZATION_SCALE,
	NX_VISUALIZE_BODY_AXES,
	NX_VISUALIZE_BODY_MASS_AXES,
	NX_VISUALIZE_BODY_LIN_VELOCITY,
	NX_VISUALIZE_BODY_ANG_VELOCITY,
	NX_VISUALIZE_CONTACT_POINT,
	NX_VISUALIZE_CONTACT_NORMAL,
	NX_VISUALIZE_ACTOR_AXES,
	NX_VISUALIZE_COLLISION_AABBS,
	NX_VISUALIZE_COLLISION_SHAPES,
	NX_VISUALIZE_COLLISION_AXES,
	NX_VISUALIZE_COLLISION_COMPOUNDS,
	NX_VISUALIZE_COLLISION_STATIC,
	NX_VISUALIZE_COLLISION_DYNAMIC,
	NX_VISUALIZE_COLLISION_FREE,
	NX_VISUALIZE_JOINT_LOCAL_AXES,
	NX_VISUALIZE_JOINT_LIMITS,
};

///////////////////////////////////////////////////////////////////////////////

static inline_ Point	ToPoint(const NxVec3& p)	{ return Point(p.x, p.y, p.z);						}
static inline_ Quat		ToQuat(const NxQuat& q)		{ return Quat(q.w, q.x, q.y, q.z);					}
static inline_ NxVec3	ToNxVec3(const Point& p)	{ return NxVec3(p.x, p.y, p.z);						}
static inline_ NxQuat	ToNxQuat(const Quat& q)		{ return NxQuat(NxVec3(q.p.x, q.p.y, q.p.z), q.w);	}

///////////////////////////////////////////////////////////////////////////////

static inline_	PintObjectHandle	CreateHandle(NxActor* actor)
{
	const size_t binary = size_t(actor);
	ASSERT(!(binary&1));
	return PintObjectHandle(binary);
}

static inline_	PintObjectHandle	CreateHandle(NxShape* shape)
{
	const size_t binary = size_t(shape);
	ASSERT(!(binary&1));
	return PintObjectHandle(binary|1);
}

static inline_	NxActor*		GetActorFromHandle(PintObjectHandle handle)
{
	const size_t binary = size_t(handle);
	return (binary & 1) ? null : (NxActor*)binary;
}

static inline_	NxShape*			GetShapeFromHandle(PintObjectHandle handle)
{
	const size_t binary = size_t(handle);
	return (binary & 1) ? (NxShape*)(binary&~1) : null;
}

///////////////////////////////////////////////////////////////////////////////

class MemoryWriteBuffer : public NxStream
	{
	public:
								MemoryWriteBuffer();
	virtual						~MemoryWriteBuffer();
				void			clear();

	virtual		NxU8			readByte()								const	{ NX_ASSERT(0);	return 0;	}
	virtual		NxU16			readWord()								const	{ NX_ASSERT(0);	return 0;	}
	virtual		NxU32			readDword()								const	{ NX_ASSERT(0);	return 0;	}
	virtual		float			readFloat()								const	{ NX_ASSERT(0);	return 0.0f;}
	virtual		double			readDouble()							const	{ NX_ASSERT(0);	return 0.0;	}
	virtual		void			readBuffer(void* buffer, NxU32 size)	const	{ NX_ASSERT(0);				}

	virtual		NxStream&		storeByte(NxU8 b);
	virtual		NxStream&		storeWord(NxU16 w);
	virtual		NxStream&		storeDword(NxU32 d);
	virtual		NxStream&		storeFloat(NxReal f);
	virtual		NxStream&		storeDouble(NxF64 f);
	virtual		NxStream&		storeBuffer(const void* buffer, NxU32 size);

				NxU32			currentSize;
				NxU32			maxSize;
				NxU8*			data;
	};

class MemoryReadBuffer : public NxStream
	{
	public:
								MemoryReadBuffer(const NxU8* data);
	virtual						~MemoryReadBuffer();

	virtual		NxU8			readByte()								const;
	virtual		NxU16			readWord()								const;
	virtual		NxU32			readDword()								const;
	virtual		float			readFloat()								const;
	virtual		double			readDouble()							const;
	virtual		void			readBuffer(void* buffer, NxU32 size)	const;

	virtual		NxStream&		storeByte(NxU8 b)							{ NX_ASSERT(0);	return *this;	}
	virtual		NxStream&		storeWord(NxU16 w)							{ NX_ASSERT(0);	return *this;	}
	virtual		NxStream&		storeDword(NxU32 d)							{ NX_ASSERT(0);	return *this;	}
	virtual		NxStream&		storeFloat(NxReal f)						{ NX_ASSERT(0);	return *this;	}
	virtual		NxStream&		storeDouble(NxF64 f)						{ NX_ASSERT(0);	return *this;	}
	virtual		NxStream&		storeBuffer(const void* buffer, NxU32 size)	{ NX_ASSERT(0);	return *this;	}

	mutable		const NxU8*		buffer;
	};

MemoryWriteBuffer::MemoryWriteBuffer() : currentSize(0), maxSize(0), data(NULL)
	{
	}

MemoryWriteBuffer::~MemoryWriteBuffer()
	{
	NX_DELETE_ARRAY(data);
	}

void MemoryWriteBuffer::clear()
	{
	currentSize = 0;
	}

NxStream& MemoryWriteBuffer::storeByte(NxU8 b)
	{
	storeBuffer(&b, sizeof(NxU8));
	return *this;
	}
NxStream& MemoryWriteBuffer::storeWord(NxU16 w)
	{
	storeBuffer(&w, sizeof(NxU16));
	return *this;
	}
NxStream& MemoryWriteBuffer::storeDword(NxU32 d)
	{
	storeBuffer(&d, sizeof(NxU32));
	return *this;
	}
NxStream& MemoryWriteBuffer::storeFloat(NxReal f)
	{
	storeBuffer(&f, sizeof(NxReal));
	return *this;
	}
NxStream& MemoryWriteBuffer::storeDouble(NxF64 f)
	{
	storeBuffer(&f, sizeof(NxF64));
	return *this;
	}
NxStream& MemoryWriteBuffer::storeBuffer(const void* buffer, NxU32 size)
	{
	NxU32 expectedSize = currentSize + size;
	if(expectedSize > maxSize)
		{
		maxSize = expectedSize + 4096;

		NxU8* newData = new NxU8[maxSize];
		NX_ASSERT(newData!=NULL);

		if(data)
			{
			memcpy(newData, data, currentSize);
			delete[] data;
			}
		data = newData;
		}
	memcpy(data+currentSize, buffer, size);
	currentSize += size;
	return *this;
	}


MemoryReadBuffer::MemoryReadBuffer(const NxU8* data) : buffer(data)
	{
	}

MemoryReadBuffer::~MemoryReadBuffer()
	{
	// We don't own the data => no delete
	}

NxU8 MemoryReadBuffer::readByte() const
	{
	NxU8 b;
	memcpy(&b, buffer, sizeof(NxU8));
	buffer += sizeof(NxU8);
	return b;
	}

NxU16 MemoryReadBuffer::readWord() const
	{
	NxU16 w;
	memcpy(&w, buffer, sizeof(NxU16));
	buffer += sizeof(NxU16);
	return w;
	}

NxU32 MemoryReadBuffer::readDword() const
	{
	NxU32 d;
	memcpy(&d, buffer, sizeof(NxU32));
	buffer += sizeof(NxU32);
	return d;
	}

float MemoryReadBuffer::readFloat() const
	{
	float f;
	memcpy(&f, buffer, sizeof(float));
	buffer += sizeof(float);
	return f;
	}

double MemoryReadBuffer::readDouble() const
	{
	double f;
	memcpy(&f, buffer, sizeof(double));
	buffer += sizeof(double);
	return f;
	}

void MemoryReadBuffer::readBuffer(void* dest, NxU32 size) const
	{
	memcpy(dest, buffer, size);
	buffer += size;
	}

	class MyNxUserAllocator : public NxUserAllocator
	{
		public:

		struct Header
		{
			udword	mMagic;
			udword	mSize;
		};

		MyNxUserAllocator() :
			mNbAllocs		(0),
			mCurrentMemory	(0)
		{
		}
		~MyNxUserAllocator()
		{
			if(mNbAllocs)
				printf("PhysX 3.2: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
		}

		virtual void* mallocDEBUG(size_t size, const char* fileName, int line)
		{
//			return ::malloc(size);

			char* memory = (char*)_aligned_malloc(size+16, 16);
			Header* H = (Header*)memory;
			H->mMagic = 0x12345678;
			H->mSize = size;
			mNbAllocs++;
			mCurrentMemory+=size;
			return memory + 16;
		}

		virtual void* malloc(size_t size)
		{
//			return ::malloc(size);

			char* memory = (char*)_aligned_malloc(size+16, 16);
			Header* H = (Header*)memory;
			H->mMagic = 0x12345678;
			H->mSize = size;
			mNbAllocs++;
			mCurrentMemory+=size;
			return memory + 16;
		}

		virtual void* realloc(void* memory, size_t size)
		{
//			return ::realloc(memory, size);
			void* newbuffer = malloc(size);
			memcpy(newbuffer, memory, size);
			free(memory);
			return newbuffer;
		}

		virtual void free(void* ptr)
		{
//			return ::free(ptr);
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

	class MyNxUserOutputStream : public NxUserOutputStream
	{
		public:

		virtual void reportError(NxErrorCode code, const char * message, const char *file, int line)
		{
			printf(message);
		}

		virtual NxAssertResponse reportAssertViolation(const char * message, const char *file, int line)
		{
			printf(message);
			return NX_AR_BREAKPOINT;
		}

		virtual void print(const char * message)
		{
			printf(message);
		}
	}gMyNxUserOutputStream;

static MyNxUserAllocator* gMyNxUserAllocator = null;

PhysX284::PhysX284() :
	mSDK				(null),
	mScene				(null),
	mCooking			(null)
//	mDefaultMaterial	(null)
{
}

PhysX284::~PhysX284()
{
	ASSERT(!gMyNxUserAllocator);
	ASSERT(!mSDK);
	ASSERT(!mScene);
	ASSERT(!mCooking);
//	ASSERT(!mDefaultMaterial);
}

void PhysX284::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	caps.mSupportPhantoms				= false;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= true;
	caps.mSupportSphereSweeps			= true;
	caps.mSupportCapsuleSweeps			= true;
	caps.mSupportConvexSweeps			= false;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= true;
	caps.mSupportCapsuleOverlaps		= true;
	caps.mSupportConvexOverlaps			= false;
}

void PhysX284::Init(const PINT_WORLD_CREATE& desc)
{
	ASSERT(!gMyNxUserAllocator);
	gMyNxUserAllocator = new MyNxUserAllocator;
	ASSERT(gMyNxUserAllocator);

	ASSERT(!mSDK);
	NxPhysicsSDKDesc sdkdesc;
	NxSDKCreateError errorCode = NXCE_NO_ERROR;
	mSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, gMyNxUserAllocator, &gMyNxUserOutputStream, sdkdesc, &errorCode);
	ASSERT(mSDK);
	if(!mSDK)
		return;

	mSDK->setParameter(NX_SKIN_WIDTH, 0.001f);
//	mSDK->setParameter(NX_SKIN_WIDTH, 0.05f);
//	mSDK->setParameter(NX_PENALTY_FORCE, 0.0f);
	if(gAdaptiveForce)
		mSDK->setParameter(NX_ADAPTIVE_FORCE, 1.0f);
	else
		mSDK->setParameter(NX_ADAPTIVE_FORCE, 0.0f);
//NX_COLL_VETO_JOINTED

	NxBounds3 maxBounds;
	maxBounds.set(NxVec3(-gGlobalBoxSize), NxVec3(gGlobalBoxSize));

	ASSERT(!mScene);
	NxSceneDesc sceneDesc;
	sceneDesc.flags				= 0;
	sceneDesc.gravity			= ToNxVec3(desc.mGravity);
	sceneDesc.maxBounds			= &maxBounds;
	sceneDesc.upAxis			= 1;
//	sceneDesc.staticStructure	= NX_PRUNING_DYNAMIC_AABB_TREE;
	sceneDesc.dynamicStructure	= gDynamicPruner;

/*	sceneDesc.userNotify
	sceneDesc.userContactModify
	sceneDesc.userTriggerReport
	sceneDesc.userContactReport
	sceneDesc.userActorPairFiltering
	sceneDesc.maxTimestep
	sceneDesc.maxIter
	sceneDesc.timeStepMethod
	sceneDesc.limits
	sceneDesc.simType
	sceneDesc.groundPlane
	sceneDesc.boundsPlanes
	sceneDesc.customScheduler
	sceneDesc.simThreadStackSize
	sceneDesc.simThreadPriority
	sceneDesc.simThreadMask
	sceneDesc.internalThreadCount
	sceneDesc.workerThreadStackSize
	sceneDesc.workerThreadPriority
	sceneDesc.threadMask
	sceneDesc.backgroundThreadCount
	sceneDesc.backgroundThreadPriority
	sceneDesc.backgroundThreadMask
	sceneDesc.subdivisionLevel
	sceneDesc.staticStructure
	sceneDesc.dynamicTreeRebuildRateHint
	sceneDesc.bpType
	sceneDesc.nbGridCellsX
	sceneDesc.nbGridCellsY
	sceneDesc.solverBatchSize*/
	mScene = mSDK->createScene(sceneDesc);
	ASSERT(mScene);

	mCooking = NxGetCookingLib(NX_PHYSICS_SDK_VERSION);
	ASSERT(mCooking);
	bool Status = mCooking->NxInitCooking(gMyNxUserAllocator, &gMyNxUserOutputStream);
	ASSERT(Status);

	// Create default material
	{
		NxMaterial* defaultMaterial = mScene->getMaterialFromIndex(0); 
		defaultMaterial->setRestitution(0.0f);
		defaultMaterial->setStaticFriction(0.5f);
		defaultMaterial->setDynamicFriction(0.5f);		
/*
		PINT_MATERIAL_CREATE Desc;
		Desc.mStaticFriction	= 0.5f;
		Desc.mDynamicFriction	= 0.5f;
		Desc.mRestitution		= 0.0f;
		mDefaultMaterial = CreateMaterial(Desc);
		ASSERT(mDefaultMaterial);*/
	}

	UpdateFromUI();
}

void PhysX284::SetGravity(const Point& gravity)
{
	ASSERT(mScene);
	mScene->setGravity(ToNxVec3(gravity));
}

void PhysX284::Close()
{
	if(mCooking)
	{
		mCooking->NxCloseCooking();
		mCooking = null;
	}
//	mDefaultMaterial = null;
	if(mSDK && mScene)
	{
		if(mScene)
		{
			mSDK->releaseScene(*mScene);
			mScene = null;
		}

		NxReleasePhysicsSDK(mSDK);
		mSDK = null;
	}

	DELETESINGLE(gMyNxUserAllocator);
}

void PhysX284::UpdateFromUI()
{
	if(!mScene)
		return;

	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		mSDK->setParameter(gDebugVizIndex[i], float(gDebugVizParams[i]));
}

udword PhysX284::Update(float dt)
{
	if(mScene)
	{
		mScene->setTiming(1.0f/60.0f, 1, NX_TIMESTEP_FIXED);
		mScene->simulate(dt);
		mScene->flushStream();
		mScene->fetchResults(NX_ALL_FINISHED, true);
	}
	return gMyNxUserAllocator->mCurrentMemory;
}

Point PhysX284::GetMainColor()
{
	return Point(1.0f, 0.0f, 1.0f);
}

void PhysX284::Render(PintRender& renderer)
{
	if(mScene)
	{
		NxU32 nbActors = mScene->getNbActors();
		NxActor** actors = mScene->getActors();
		for(NxU32 i=0;i<nbActors;i++)
		{
			NxActor* actor = actors[i];

			const NxU32 nbShapes = actor->getNbShapes();
			NxShape*const* shapes = actor->getShapes();
			for(NxU32 j=0;j<nbShapes;j++)
			{
				NxShape* shape = shapes[j];

				const NxVec3 ShapeGlobalPos = shape->getGlobalPosition();
				const NxMat33 ShapeGlobalRot = shape->getGlobalOrientation();
				NxQuat ShapeGlobalQ;
				ShapeGlobalRot.toQuat(ShapeGlobalQ);

				const PR IcePose(ToPoint(ShapeGlobalPos), ToQuat(ShapeGlobalQ));

				ASSERT(shape->userData);
				if(shape->userData)
				{
					PintShapeRenderer* shapeRenderer = (PintShapeRenderer*)shape->userData;
					shapeRenderer->Render(IcePose);
				}
/*				else
				{
					const NxShapeType shapeType = shape->getType();
					if(shapeType==NX_SHAPE_SPHERE)
					{
						NxSphereShape* sphereShape = shape->isSphere();

						renderer.DrawSphere(sphereShape->getRadius(), IcePose);
					}
					else if(shapeType==NX_SHAPE_BOX)
					{
						NxBoxShape* boxShape = shape->isBox();

						const NxVec3& extents = boxShape->getDimensions();
						renderer.DrawBox(Point(extents.x, extents.y, extents.z), IcePose);
					}
					else ASSERT(0);
				}*/
			}
		}

		const NxDebugRenderable* dbg = mScene->getDebugRenderable();
		if(dbg)
		{
			udword NbLines = dbg->getNbLines();
			const NxDebugLine* Lines = dbg->getLines();
			const Point LineColor(1.0f, 1.0f, 1.0f);
			for(udword i=0;i<NbLines;i++)
			{
				renderer.DrawLine(ToPoint(Lines[i].p0), ToPoint(Lines[i].p1), LineColor);
			}
		}
	}
}


NxMaterial* PhysX284::CreateMaterial(const PINT_MATERIAL_CREATE& desc)
{
	const NxU32 NbMaterials = mMaterials.size();
	for(NxU32 i=0;i<NbMaterials;i++)
	{
		NxMaterial* M = mMaterials[i];
		if(		M->getRestitution()==desc.mRestitution
			&&	M->getStaticFriction()==desc.mStaticFriction
			&&	M->getDynamicFriction()==desc.mDynamicFriction)
		{
			return M;
		}
	}

	ASSERT(mScene);
	NxMaterialDesc matDesc;
	matDesc.dynamicFriction	= desc.mDynamicFriction;
	matDesc.staticFriction	= desc.mStaticFriction;
	matDesc.restitution		= desc.mRestitution;
	NxMaterial* M = mScene->createMaterial(matDesc);
	ASSERT(M);

	mMaterials.push_back(M);
	return M;
}

NxConvexMesh* PhysX284::CreateConvexMesh(const Point* verts, udword vertCount, udword flags, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mSDK);

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

	NxConvexMeshDesc ConvexDesc;
	ConvexDesc.numVertices		= vertCount;
	ConvexDesc.pointStrideBytes	= sizeof(NxVec3);
	ConvexDesc.points			= verts;
	ConvexDesc.flags			= flags;

	MemoryWriteBuffer buf;
	if(!mCooking->NxCookConvexMesh(ConvexDesc, buf))
		return null;

	NxConvexMesh* NewConvex = mSDK->createConvexMesh(MemoryReadBuffer(buf.data));

	if(renderer)
	{
		ConvexRender NewConvexRender;
		NewConvexRender.mConvexMesh = NewConvex;
		NewConvexRender.mRenderer = renderer;
		mConvexes.push_back(NewConvexRender);
	}
	return NewConvex;
}

NxTriangleMesh* PhysX284::CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mSDK);

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

	NxTriangleMeshDesc meshDesc;
	meshDesc.numVertices			= surface.mNbVerts;
	meshDesc.numTriangles			= surface.mNbFaces;
	meshDesc.pointStrideBytes		= sizeof(NxVec3);
	meshDesc.triangleStrideBytes	= sizeof(udword)*3;
	meshDesc.points					= surface.mVerts;
	meshDesc.triangles				= surface.mDFaces;
//	meshDesc.flags					= NX_MF_FLIPNORMALS;
	meshDesc.flags					= 0;

	MemoryWriteBuffer buf;
	if(!mCooking->NxCookTriangleMesh(meshDesc, buf))
		return null;

	NxTriangleMesh* NewMesh = mSDK->createTriangleMesh(MemoryReadBuffer(buf.data));

	if(renderer)
	{
		MeshRender NewMeshRender;
		NewMeshRender.mTriangleMesh = NewMesh;
		NewMeshRender.mRenderer = renderer;
		mMeshes.push_back(NewMeshRender);
	}
	return NewMesh;
}

PintObjectHandle PhysX284::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(mSDK);
	ASSERT(mScene);
	if(!mSDK)
		return null;

	NxBodyDesc bodyDesc;
//	bodyDesc.massLocalPose			= ;
//	bodyDesc.massSpaceInertia		= ;
//	bodyDesc.massSpaceInertia		= NxVec3(1,1,1);
	bodyDesc.mass					= desc.mMass;
	bodyDesc.linearVelocity			= ToNxVec3(desc.mLinearVelocity);
	bodyDesc.angularVelocity		= ToNxVec3(desc.mAngularVelocity);
	if(!gEnableSleeping)
		bodyDesc.wakeUpCounter		= 999999999.0f;
	bodyDesc.linearDamping			= gLinearDamping;
	bodyDesc.angularDamping			= gAngularDamping;
//	bodyDesc.maxAngularVelocity		= ;
//	bodyDesc.CCDMotionThreshold		= ;
	bodyDesc.flags					= desc.mKinematic ? NX_BF_KINEMATIC : 0;
//	bodyDesc.sleepLinearVelocity	= ;
//	bodyDesc.sleepAngularVelocity	= ;
	bodyDesc.solverIterationCount	= gSolverIterationCount;
//	bodyDesc.sleepEnergyThreshold	= ;
//	bodyDesc.sleepDamping			= ;
//	bodyDesc.contactReportThreshold	= ;

	NxActorDesc actorDesc;
	actorDesc.globalPose.t			= ToNxVec3(desc.mPosition);
	actorDesc.globalPose.M			= NxMat33(ToNxQuat(desc.mRotation));
//	actorDesc.density				= ;
//	actorDesc.flags					= ;
//	actorDesc.group					= ;
//	actorDesc.dominanceGroup		= ;
//	actorDesc.contactReportFlags	= ;
//	actorDesc.forceFieldMaterial	= ;
//	actorDesc.userData				= ;
//	actorDesc.name					= ;
//	actorDesc.compartment			= ;

	if(desc.mMass!=0.0f)
		actorDesc.body		= &bodyDesc;

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
		NxShapeDesc* shapeDesc = null;
		if(CurrentShape->mType==PINT_SHAPE_SPHERE)
		{
			const PINT_SPHERE_CREATE* SphereCreate = static_cast<const PINT_SPHERE_CREATE*>(CurrentShape);

			NxSphereShapeDesc* sphereShapeDesc = new NxSphereShapeDesc;
			sphereShapeDesc->radius	= SphereCreate->mRadius;
			shapeDesc = sphereShapeDesc;
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(CurrentShape);

			NxBoxShapeDesc* boxShapeDesc = new NxBoxShapeDesc;
			boxShapeDesc->dimensions = ToNxVec3(BoxCreate->mExtents);
			shapeDesc = boxShapeDesc;
		}
		else if(CurrentShape->mType==PINT_SHAPE_CAPSULE)
		{
			const PINT_CAPSULE_CREATE* CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE*>(CurrentShape);

			NxCapsuleShapeDesc* capsuleShapeDesc = new NxCapsuleShapeDesc;
			capsuleShapeDesc->radius	= CapsuleCreate->mRadius;
			capsuleShapeDesc->height	= CapsuleCreate->mHalfHeight * 2.0f;
			shapeDesc = capsuleShapeDesc;
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);

			ASSERT(mCooking);
//			NxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, NX_CF_COMPUTE_CONVEX|NX_CF_INFLATE_CONVEX, CurrentShape->mRenderer);
			NxConvexMesh* ConvexMesh = CreateConvexMesh(ConvexCreate->mVerts, ConvexCreate->mNbVerts, NX_CF_COMPUTE_CONVEX, CurrentShape->mRenderer);
			ASSERT(ConvexMesh);

			NxConvexShapeDesc* convexShapeDesc = new NxConvexShapeDesc;
			convexShapeDesc->meshData	= ConvexMesh;
			shapeDesc = convexShapeDesc;
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			ASSERT(mCooking);
			NxTriangleMesh* TriangleMesh = CreateTriangleMesh(MeshCreate->mSurface, CurrentShape->mRenderer);
			ASSERT(TriangleMesh);

			NxTriangleMeshShapeDesc* triangleMeshShapeDesc = new NxTriangleMeshShapeDesc;
			triangleMeshShapeDesc->meshData		= TriangleMesh;
			triangleMeshShapeDesc->meshFlags	= 0;
			shapeDesc = triangleMeshShapeDesc;
		}
		else ASSERT(0);

		if(shapeDesc)
		{
			NxMat34 LocalPose;
			LocalPose.t	= ToNxVec3(CurrentShape->mLocalPos);
			LocalPose.M	= NxMat33(ToNxQuat(CurrentShape->mLocalRot));

			shapeDesc->localPose		= LocalPose;
			shapeDesc->shapeFlags		= 0;

			if(!gSQFlag)
				shapeDesc->shapeFlags	|= NX_SF_DISABLE_SCENE_QUERIES|NX_SF_DISABLE_RAYCASTING;
			else
				shapeDesc->shapeFlags	&= ~(NX_SF_DISABLE_SCENE_QUERIES|NX_SF_DISABLE_RAYCASTING);

			if(gDebugVizParams[0])
				shapeDesc->shapeFlags	|= NX_SF_VISUALIZATION;
			else
				shapeDesc->shapeFlags	&= ~NX_SF_VISUALIZATION;

			shapeDesc->group			= desc.mCollisionGroup;
//			shapeDesc->ccdSkeleton
//			shapeDesc->density	
//			shapeDesc->mass
//			shapeDesc->skinWidth
//			shapeDesc->userData			= ;
//			shapeDesc->name				= ;
//			shapeDesc->groupsMask
//			shapeDesc->nonInteractingCompartmentTypes

			if(CurrentShape->mMaterial)
			{
				PINT_MATERIAL_CREATE Desc;
				Desc.mStaticFriction		= CurrentShape->mMaterial->mStaticFriction;
				Desc.mDynamicFriction		= CurrentShape->mMaterial->mDynamicFriction;
				Desc.mRestitution			= CurrentShape->mMaterial->mRestitution;
				NxMaterial* M				= CreateMaterial(Desc);
				ASSERT(M);
				shapeDesc->materialIndex	= M->getMaterialIndex();
			}
			actorDesc.shapes.pushBack(shapeDesc);
		}

		CurrentShape = CurrentShape->mNext;
	}

	NxActor* actor = mScene->createActor(actorDesc);
	ASSERT(actor);

	if(gDebugVizParams[0] && actor->isDynamic())
		actor->raiseBodyFlag(NX_BF_VISUALIZATION);

	const NxU32 Nb = actorDesc.shapes.size();
	const NxU32 NbActorShapes = actor->getNbShapes();
	ASSERT(Nb==NbActorShapes);

	NxShape*const* ActorShapes = actor->getShapes();
	CurrentShape = desc.mShapes;
	udword ShapeIndex = 0;
	while(CurrentShape)
	{
		if(CurrentShape->mRenderer)
			ActorShapes[ShapeIndex]->userData = CurrentShape->mRenderer;
		ShapeIndex++;
		CurrentShape = CurrentShape->mNext;
	}
	ASSERT(ShapeIndex==NbActorShapes);

	for(NxU32 i=0;i<Nb;i++)
	{
		NxShapeDesc* desc = actorDesc.shapes[i];
		delete desc;
	}

	return CreateHandle(actor);
}

bool PhysX284::ReleaseObject(PintObjectHandle handle)
{
	NxActor* Actor = GetActorFromHandle(handle);
	if(Actor)
	{
		// ### what about ConvexRender/etc?
		mScene->releaseActor(*Actor);
		return true;
	}
	NxShape* Shape = GetShapeFromHandle(handle);
	if(Shape)
	{
		Actor = &Shape->getActor();
		mScene->releaseActor(*Actor);
		return true;
	}
	ASSERT(0);
	return false;
}

static void SetupJointDesc(NxJointDesc& desc)
{
	if(gEnableCollisionBetweenJointed)
		desc.jointFlags |= NX_JF_COLLISION_ENABLED;
	else
		desc.jointFlags &= ~NX_JF_COLLISION_ENABLED;
}

PintJointHandle PhysX284::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mScene);

	NxActor* actor0 = (NxActor*)desc.mObject0;
	NxActor* actor1 = (NxActor*)desc.mObject1;

	NxJoint* CreatedJoint = null;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			NxSphericalJointDesc Desc;
			SetupJointDesc(Desc);
			Desc.actor[0]		= actor0->isDynamic() ? actor0 : null;
			Desc.actor[1]		= actor1->isDynamic() ? actor1 : null;
			// ### this doesn't work
//			Desc.localAnchor[0]	= ToNxVec3(jc.mLocalPivot0);
//			Desc.localAnchor[1]	= ToNxVec3(jc.mLocalPivot1);
			// ### really tedious to setup!
			if(1)
			{
				NxMat34 m0 = actor0->getGlobalPose();
				NxMat34 m1 = actor1->getGlobalPose();
				NxVec3 wp0;	m0.multiply(ToNxVec3(jc.mLocalPivot0), wp0);
				NxVec3 wp1;	m1.multiply(ToNxVec3(jc.mLocalPivot1), wp1);
				Desc.setGlobalAnchor((wp0+wp1)*0.5f);
			}
			CreatedJoint = mScene->createJoint(Desc);
			ASSERT(CreatedJoint);
		}
		break;

		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			const NxMat34 m0 = actor0->getGlobalPose();
			const NxMat34 m1 = actor1->getGlobalPose();

			NxRevoluteJointDesc Desc;
			SetupJointDesc(Desc);
			Desc.actor[0]		= actor0->isDynamic() ? actor0 : null;
			Desc.actor[1]		= actor1->isDynamic() ? actor1 : null;
//			Desc.localAnchor[0]	= ToNxVec3(sp.mLocalPivot0);
//			Desc.localAnchor[1]	= ToNxVec3(sp.mLocalPivot1);

			if(!jc.mGlobalAnchor.IsNotUsed() && !jc.mGlobalAxis.IsNotUsed())
			{
				Desc.setGlobalAnchor(ToNxVec3(jc.mGlobalAnchor));
				Desc.setGlobalAxis(ToNxVec3(jc.mGlobalAxis));
			}
			else
			{
				// ### really tedious to setup!
				{
					NxVec3 wp0;	m0.multiply(ToNxVec3(jc.mLocalPivot0), wp0);
					NxVec3 wp1;	m1.multiply(ToNxVec3(jc.mLocalPivot1), wp1);
					Desc.setGlobalAnchor((wp0+wp1)*0.5f);
				}
				if(1)
				{
					NxVec3 wp0;	m0.M.multiply(ToNxVec3(jc.mLocalAxis0), wp0);
					NxVec3 wp1;	m1.M.multiply(ToNxVec3(jc.mLocalAxis1), wp1);
					NxVec3 GlobalAxis = (wp0+wp1)*0.5f; 
					GlobalAxis.normalize();
					Desc.setGlobalAxis(GlobalAxis);
				}
			}
			if(jc.mMinLimitAngle!=MIN_FLOAT || jc.mMaxLimitAngle!=MAX_FLOAT)
			{
				NxJointLimitPairDesc limit;
				Desc.limit.low.value	= jc.mMinLimitAngle;
				Desc.limit.high.value	= jc.mMaxLimitAngle;
				Desc.flags |= NX_RJF_LIMIT_ENABLED;
			}
			CreatedJoint = mScene->createJoint(Desc);
			ASSERT(CreatedJoint);
		}
		break;

		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			NxFixedJointDesc Desc;
			SetupJointDesc(Desc);
			Desc.actor[0]		= actor0->isDynamic() ? actor0 : null;
			Desc.actor[1]		= actor1->isDynamic() ? actor1 : null;
			// ### really tedious to setup!
			if(1)
			{
				NxMat34 m0 = actor0->getGlobalPose();
				NxMat34 m1 = actor1->getGlobalPose();
				NxVec3 wp0;	m0.multiply(ToNxVec3(jc.mLocalPivot0), wp0);
				NxVec3 wp1;	m1.multiply(ToNxVec3(jc.mLocalPivot1), wp1);
				Desc.setGlobalAnchor((wp0+wp1)*0.5f);
			}
			CreatedJoint = mScene->createJoint(Desc);

//CreatedJoint->setBreakable(8.0f, 100000.0f);
			ASSERT(CreatedJoint);
		}
		break;

		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			NxPrismaticJointDesc Desc;
			SetupJointDesc(Desc);
			Desc.actor[0]		= actor0->isDynamic() ? actor0 : null;
			Desc.actor[1]		= actor1->isDynamic() ? actor1 : null;
			// ### really tedious to setup!
			NxMat34 m0 = actor0->getGlobalPose();
			NxMat34 m1 = actor1->getGlobalPose();
			if(1)
			{
				NxVec3 wp0;	m0.multiply(ToNxVec3(jc.mLocalPivot0), wp0);
				NxVec3 wp1;	m1.multiply(ToNxVec3(jc.mLocalPivot1), wp1);
				Desc.setGlobalAnchor((wp0+wp1)*0.5f);
			}
			if(1)
			{
				NxVec3 wp0;	m0.M.multiply(ToNxVec3(jc.mLocalAxis0), wp0);
				NxVec3 wp1;	m1.M.multiply(ToNxVec3(jc.mLocalAxis1), wp1);
				NxVec3 GlobalAxis = (wp0+wp1)*0.5f; 
				GlobalAxis.normalize();
				Desc.setGlobalAxis(GlobalAxis);
			}
			CreatedJoint = mScene->createJoint(Desc);
			ASSERT(CreatedJoint);
		}
		break;
	}
	return CreatedJoint;









/*	ASSERT(mPhysics);

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
			PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
			PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
//			PxQuat q0 = PxShortestRotation(ToPxVec3(jc.mLocalAxis0), PxVec3(1.0f, 0.0f, 0.0f));
//			PxQuat q1 = PxShortestRotation(ToPxVec3(jc.mLocalAxis0), PxVec3(1.0f, 0.0f, 0.0f));

			PxRevoluteJoint* j = PxRevoluteJointCreate(*mPhysics,	actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0),
																	actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
			ASSERT(j);

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
		}
		break;
	}

	if(CreatedJoint)
	{
		if(gEnableCollisionBetweenJointed)
			CreatedJoint->setConstraintFlags(PxConstraintFlag::eCOLLISION_ENABLED);
	}
	return CreatedJoint;*/
	return null;
}

void PhysX284::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	ASSERT(mScene);
	for(udword i=0;i<nb_groups;i++)
		mScene->setGroupCollisionFlag(NxCollisionGroup(groups[i].mGroup0), NxCollisionGroup(groups[i].mGroup1), false);
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const NxRaycastHit& result)
{
	hit.mObject			= CreateHandle(result.shape);
	hit.mImpact			= ToPoint(result.worldImpact);
	hit.mNormal			= ToPoint(result.worldNormal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceID;
}

udword PhysX284::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	udword NbHits = 0;
	while(nb--)
	{
		NxRaycastHit Hit;

		NxRay worldRay;
		worldRay.orig	= ToNxVec3(raycasts->mOrigin);
		worldRay.dir	= ToNxVec3(raycasts->mDir);

		const NxU32 groups = 0xffffffff;
		const NxU32 hintFlags = 0xffffffff;

		if(mScene->raycastClosestShape(worldRay, NX_ALL_SHAPES, Hit, groups, raycasts->mMaxDist, hintFlags))
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

	return NbHits;
}

udword PhysX284::BatchRaycastAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	udword NbHits = 0;
	while(nb--)
	{
		NxRay worldRay;
		worldRay.orig	= ToNxVec3(raycasts->mOrigin);
		worldRay.dir	= ToNxVec3(raycasts->mDir);
		const NxU32 groups = 0xffffffff;
		const bool b = mScene->raycastAnyShape(worldRay, NX_ALL_SHAPES, groups, raycasts->mMaxDist);
		NbHits += b;
		dest->mHit = b;
		raycasts++;
		dest++;
	}
	return NbHits;
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const NxSweepQueryHit& result, float max_dist)
{
	hit.mObject			= CreateHandle(result.hitShape);
	hit.mImpact			= ToPoint(result.point);
	hit.mNormal			= ToPoint(result.normal);
	hit.mDistance		= result.t * max_dist;
	hit.mTriangleIndex	= result.faceID;
}

udword PhysX284::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mScene);

	udword NbHits = 0;
	while(nb--)
	{
		/*const*/ NxBox WorldBox(	ToNxVec3(sweeps->mBox.mCenter),
								ToNxVec3(sweeps->mBox.mExtents),
								NxMat33(ToNxVec3(sweeps->mBox.mRot[0]), ToNxVec3(sweeps->mBox.mRot[1]), ToNxVec3(sweeps->mBox.mRot[2])));
WorldBox.rot.setTransposed();
		NxSweepQueryHit Hit;
		if(mScene->linearOBBSweep(WorldBox, ToNxVec3(sweeps->mDir)*sweeps->mMaxDist, NX_SF_STATICS|NX_SF_DYNAMICS, NULL, 1, &Hit, NULL))
		{
			NbHits++;
			FillResultStruct(*dest, Hit, sweeps->mMaxDist);
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

udword PhysX284::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	ASSERT(mScene);

	udword NbHits = 0;
	while(nb--)
	{
		const NxCapsule WorldCapsule(NxSegment(ToNxVec3(sweeps->mSphere.mCenter), ToNxVec3(sweeps->mSphere.mCenter)), sweeps->mSphere.mRadius);
		NxSweepQueryHit Hit;
		if(mScene->linearCapsuleSweep(WorldCapsule, ToNxVec3(sweeps->mDir)*sweeps->mMaxDist, NX_SF_STATICS|NX_SF_DYNAMICS, NULL, 1, &Hit, NULL))
		{
			NbHits++;
			FillResultStruct(*dest, Hit, sweeps->mMaxDist);
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

udword PhysX284::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	ASSERT(mScene);

	udword NbHits = 0;
	while(nb--)
	{
		const NxCapsule WorldCapsule(NxSegment(ToNxVec3(sweeps->mCapsule.mP0), ToNxVec3(sweeps->mCapsule.mP1)), sweeps->mCapsule.mRadius);
		NxSweepQueryHit Hit;
		if(mScene->linearCapsuleSweep(WorldCapsule, ToNxVec3(sweeps->mDir)*sweeps->mMaxDist, NX_SF_STATICS|NX_SF_DYNAMICS, NULL, 1, &Hit, NULL))
		{
			NbHits++;
			FillResultStruct(*dest, Hit, sweeps->mMaxDist);
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

udword PhysX284::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	NxShape* touchedShape;
	udword NbHits = 0;
	while(nb--)
	{
		if(mScene->overlapSphereShapes(NxSphere(ToNxVec3(overlaps->mSphere.mCenter), overlaps->mSphere.mRadius), NX_ALL_SHAPES, 1, &touchedShape, null, 0xffffffff, null, true))
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

udword PhysX284::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	NxShape* touchedShapes[4096];
	udword NbHits = 0;
	while(nb--)
	{
		const NxU32 Nb = mScene->overlapSphereShapes(NxSphere(ToNxVec3(overlaps->mSphere.mCenter), overlaps->mSphere.mRadius), NX_ALL_SHAPES, 4096, touchedShapes, null, 0xffffffff, null, true);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX284::BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	NxShape* touchedShape;
	udword NbHits = 0;
	while(nb--)
	{
		/*const*/ NxBox WorldBox(	ToNxVec3(overlaps->mBox.mCenter),
								ToNxVec3(overlaps->mBox.mExtents),
								NxMat33(ToNxVec3(overlaps->mBox.mRot[0]), ToNxVec3(overlaps->mBox.mRot[1]), ToNxVec3(overlaps->mBox.mRot[2])));
WorldBox.rot.setTransposed();

		if(mScene->overlapOBBShapes(WorldBox, NX_ALL_SHAPES, 1, &touchedShape, null, 0xffffffff, null, true))
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

udword PhysX284::BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	NxShape* touchedShapes[4096];
	udword NbHits = 0;
	while(nb--)
	{
		/*const*/ NxBox WorldBox(	ToNxVec3(overlaps->mBox.mCenter),
								ToNxVec3(overlaps->mBox.mExtents),
								NxMat33(ToNxVec3(overlaps->mBox.mRot[0]), ToNxVec3(overlaps->mBox.mRot[1]), ToNxVec3(overlaps->mBox.mRot[2])));
WorldBox.rot.setTransposed();

		const NxU32 Nb = mScene->overlapOBBShapes(WorldBox, NX_ALL_SHAPES, 4096, touchedShapes, null, 0xffffffff, null, true);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword PhysX284::BatchCapsuleOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)
{
/*	ASSERT(mScene);

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

		PxShape* Hit;
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
	return NbHits;*/
	return 0;
}

udword PhysX284::BatchCapsuleOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)
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

PR PhysX284::GetWorldTransform(PintObjectHandle handle)
{
	NxVec3 Pos;
	NxMat33 Rot;

	NxActor* Actor = GetActorFromHandle(handle);
	if(Actor)
	{
		Pos = Actor->getGlobalPosition();
		Rot = Actor->getGlobalOrientation();
	}
	else
	{
		NxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		Pos = Shape->getGlobalPosition();
		Rot = Shape->getGlobalOrientation();
	}

	NxQuat Q;
	Rot.toQuat(Q);

	return PR(ToPoint(Pos), ToQuat(Q));
}

void PhysX284::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	NxActor* Actor = GetActorFromHandle(handle);
	if(!Actor)
	{
		NxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
		Actor = &Shape->getActor();
	}

	if(Actor->isDynamic())
	{
		NxForceMode mode;
		if(action_type==PINT_ACTION_FORCE)
			mode = NX_FORCE;
		else if(action_type==PINT_ACTION_IMPULSE)
			mode = NX_IMPULSE;
		else ASSERT(0);

		Actor->addForceAtPos(ToNxVec3(action), ToNxVec3(pos), mode);
	}
}

bool PhysX284::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	NxActor* Actor = GetActorFromHandle(handle);
	if(!Actor)
		return false;

	Actor->moveGlobalPosition(ToNxVec3(pos));
	return true;
}


static PhysX284* gPhysX284 = null;
static void gPhysX284_GetOptionsFromGUI();

void PhysX284_Init(const PINT_WORLD_CREATE& desc)
{
	gPhysX284_GetOptionsFromGUI();

	ASSERT(!gPhysX284);
	gPhysX284 = ICE_NEW(PhysX284);
	gPhysX284->Init(desc);
}

void PhysX284_Close()
{
	if(gPhysX284)
	{
		gPhysX284->Close();
		delete gPhysX284;
		gPhysX284 = null;
	}
}

PhysX284* GetPhysX284()
{
	return gPhysX284;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gPhysXGUI = null;
//static IceComboBox*	gComboBox_NbThreads = null;
static IceComboBox*	gComboBox_DynamicPruner = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_GlobalBoxSize = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum PhysXGUIElement
{
	PHYSX_GUI_MAIN,
	//
	PHYSX_GUI_ENABLE_SLEEPING,
	PHYSX_GUI_ENABLE_SQ,
	PHYSX_GUI_SHARE_MESH_DATA,
//	PHYSX_GUI_PCM,
	PHYSX_GUI_ADAPTIVE_FORCE,
//	PHYSX_GUI_DISABLE_STRONG_FRICTION,
//	PHYSX_GUI_ENABLE_ONE_DIR_FRICTION,
//	PHYSX_GUI_ENABLE_TWO_DIR_FRICTION,
	//
//	PHYSX_GUI_NB_THREADS,
	PHYSX_GUI_DYNAMIC_PRUNER,
	//
	PHYSX_GUI_NB_SOLVER_ITER,
	PHYSX_GUI_LINEAR_DAMPING,
	PHYSX_GUI_ANGULAR_DAMPING,
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
			if(gComboBox_DynamicPruner)
				gComboBox_DynamicPruner->SetEnabled(checked);
			break;
		case PHYSX_GUI_SHARE_MESH_DATA:
			gShareMeshData = checked;
			break;
		case PHYSX_GUI_ADAPTIVE_FORCE:
			gAdaptiveForce = checked;
			break;
/*		case PHYSX_GUI_PCM:
			gPCM = checked;
			break;
		case PHYSX_GUI_DISABLE_STRONG_FRICTION:
			gDisableStrongFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_ONE_DIR_FRICTION:
			gEnableOneDirFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_TWO_DIR_FRICTION:
			gEnableTwoDirFriction = checked;
			break;*/
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

	if(gPhysX284)
		gPhysX284->UpdateFromUI();
}

//static udword gNbThreadsToIndex[] = { 0, 0, 1, 2, 3 };
//static udword gIndexToNbThreads[] = { 0, 2, 3, 4 };

static void gPhysX284_GetOptionsFromGUI()
{
/*	if(gComboBox_NbThreads)
	{
		const udword Index = gComboBox_NbThreads->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToNbThreads)/sizeof(gIndexToNbThreads[0]));
		gNbThreads = gIndexToNbThreads[Index];
	}*/

	if(gComboBox_DynamicPruner)
	{
		const udword Index = gComboBox_DynamicPruner->GetSelectedIndex();
		gDynamicPruner = NxPruningStructure(Index);
	}

	if(gEditBox_SolverIter)
	{
		sdword tmp;
		bool status = gEditBox_SolverIter->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gSolverIterationCount = udword(tmp);
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

	if(gEditBox_GlobalBoxSize)
	{
		float tmp;
		bool status = gEditBox_GlobalBoxSize->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gGlobalBoxSize = tmp;
	}
}

IceWindow* PhysX284_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gPhysXGUI, parent, PHYSX_GUI_MAIN, "PhysX 2.8.4 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gPhysXGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ENABLE_SQ, 4, y, CheckBoxWidth, 20, "Enable scene queries", gPhysXGUI, gSQFlag, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_SHARE_MESH_DATA, 4, y, CheckBoxWidth, 20, "Share mesh data", gPhysXGUI, gShareMeshData, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, PHYSX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gPhysXGUI, gAdaptiveForce, gCheckBoxCallback);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, PHYSX_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gPhysXGUI);

	y += YStep;

	const sdword LabelOffsetY = 2;
	const sdword OffsetX = 90;
	const sdword EditBoxWidth = 60;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Dynamic pruner:", gPhysXGUI);
		ComboBoxDesc CBBD;
		CBBD.mID		= PHYSX_GUI_DYNAMIC_PRUNER;
		CBBD.mParent	= Main;
		CBBD.mX			= 4+OffsetX;
		CBBD.mY			= y;
		CBBD.mWidth		= 150;
		CBBD.mHeight	= 20;
		CBBD.mLabel		= "Dynamic pruner";
		gComboBox_DynamicPruner = ICE_NEW(IceComboBox)(CBBD);
		gPhysXGUI->Add(udword(gComboBox_DynamicPruner));
		gComboBox_DynamicPruner->Add("NX_PRUNING_NONE");
		gComboBox_DynamicPruner->Add("NX_PRUNING_OCTREE");
		gComboBox_DynamicPruner->Add("NX_PRUNING_QUADTREE");
		gComboBox_DynamicPruner->Add("NX_PRUNING_DYNAMIC_AABB_TREE");
		gComboBox_DynamicPruner->Select(gDynamicPruner);
		gComboBox_DynamicPruner->SetVisible(true);
		gComboBox_DynamicPruner->SetEnabled(gSQFlag);
		y += YStep;
	}
	y += YStep;

	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Solver iter:", gPhysXGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, PHYSX_GUI_NB_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gPhysXGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, PHYSX_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gPhysXGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, PHYSX_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "World bounds size:", gPhysXGUI);
		gEditBox_GlobalBoxSize = helper.CreateEditBox(Main, PHYSX_GUI_GLOBAL_BOX_SIZE, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gGlobalBoxSize), gPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;
	}
	return Main;
}

void PhysX284_CloseGUI()
{
	Common_CloseGUI(gPhysXGUI);

//	gComboBox_NbThreads = null;
	gEditBox_SolverIter = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	gComboBox_DynamicPruner = null;
	gEditBox_GlobalBoxSize = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}

///////////////////////////////////////////////////////////////////////////////

class PhysX284PlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return PhysX284_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ PhysX284_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ PhysX284_Init(desc);						}
	virtual	void		Close()												{ PhysX284_Close();							}
	virtual	Pint*		GetPint()											{ return GetPhysX284();						}
};
static PhysX284PlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
