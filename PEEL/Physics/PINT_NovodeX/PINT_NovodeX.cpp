///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_Novodex.h"
#include "..\PINT_Common\PINT_Common.h"

#include "NxPhysics.h"

#define SAFE_RELEASE(x)	if(x) { x->release(); x = null; }

///////////////////////////////////////////////////////////////////////////////

static	const	bool	gEnableCollisionBetweenJointed	= false;
static			bool	gEnableSleeping					= false;
static			bool	gAdaptiveForce					= true;
static			bool	gMeshHintSpeed					= true;
static			udword	gSolverIterationCount			= 4;
static			float	gLinearDamping					= 0.1f;
static			float	gAngularDamping					= 0.05f;
static			bool	gSQFilterOutAllShapes			= false;

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
	virtual	~MyNxUserAllocator()
	{
		if(mNbAllocs)
			printf("NovodeX 2.1.1: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
	}

	virtual void* mallocDEBUG(size_t size, const char* fileName, int line)
	{
//		return ::malloc(size);
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
//		return ::malloc(size);
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
//		return ::realloc(memory, size);
		void* newbuffer = malloc(size);
		memcpy(newbuffer, memory, size);
		free(memory);
		return newbuffer;
	}

	virtual void free(void* ptr)
	{
//		::free(ptr);
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

	virtual void reportError(NxErrorCode, const char * message, const char *file, int line)
	{
		printf(message);
	}
	virtual NxAssertResponse reportAssertViolation(const char * message, const char *file, int line)
	{
		printf(message);
		return NX_AR_BREAKPOINT;
	}
	virtual void print(const char* message)
	{
		printf(message);
	}
	}gMyNxUserOutputStream;

static MyNxUserAllocator* gMyNxUserAllocator = null;

NovodeX::NovodeX() :
	mSDK			(null),
	mScene			(null),
	mTouchedShapes	(null)
{
}

NovodeX::~NovodeX()
{
	ASSERT(!gMyNxUserAllocator);
	ASSERT(!mSDK);
	ASSERT(!mScene);
	ASSERT(!mTouchedShapes);
}

void NovodeX::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= false;
	caps.mSupportMeshes					= true;
	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	caps.mSupportPhantoms				= false;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= false;
	caps.mSupportSphereSweeps			= false;
	caps.mSupportCapsuleSweeps			= false;
	caps.mSupportConvexSweeps			= false;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= false;
	caps.mSupportCapsuleOverlaps		= false;
	caps.mSupportConvexOverlaps			= false;
}

void NovodeX::Init(const PINT_WORLD_CREATE& desc)
{
	if(0)
	{
		_clearfp();
	   udword x86_cw;
	   udword sse2_cw;
		#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_IC | _MCW_RC | _MCW_PC
		__control87_2(_CW_DEFAULT | _DN_FLUSH, _MCW_ALL, &x86_cw, &sse2_cw);
	}

	ASSERT(!gMyNxUserAllocator);
	gMyNxUserAllocator = new MyNxUserAllocator;
	ASSERT(gMyNxUserAllocator);

	ASSERT(!mSDK);
	mSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, gMyNxUserAllocator, &gMyNxUserOutputStream);
	ASSERT(mSDK);

	mSDK->setParameter(NX_PENALTY_FORCE, 0.6f);
//	mSDK->setParameter(NX_PENALTY_FORCE, 0.8f);
//	mSDK->setParameter(NX_MIN_SEPARATION_FOR_PENALTY, -0.05f);
	mSDK->setParameter(NX_MIN_SEPARATION_FOR_PENALTY, -0.005f);
	if(gAdaptiveForce)
		mSDK->setParameter(NX_ADAPTIVE_FORCE, 1.0f);
	else
		mSDK->setParameter(NX_ADAPTIVE_FORCE, 0.0f);

	if(gMeshHintSpeed)
		mSDK->setParameter(NX_MESH_HINT_SPEED, 1.0f);
	else
		mSDK->setParameter(NX_MESH_HINT_SPEED, 0.0f);

	// Create default material
	{
/*		NxMaterial DefaultMaterial;
		DefaultMaterial.dynamicFriction	= 0.5f;
		DefaultMaterial.staticFriction	= 0.5f;
		DefaultMaterial.restitution		= 0.0f;
		mSDK->setMaterialAtIndex(0, &DefaultMaterial);*/

		PINT_MATERIAL_CREATE Desc;
		Desc.mStaticFriction	= 0.5f;
		Desc.mDynamicFriction	= 0.5f;
		Desc.mRestitution		= 0.0f;
		udword Index = CreateMaterial(Desc);
		ASSERT(Index==0);
	}

	NxBounds3 Bounds;
	Bounds.setCenterExtents(NxVec3(0.0f, 0.0f, 0.0f), NxVec3(100.0f, 100.0f, 100.0f));

	ASSERT(!mScene);
	NxSceneDesc sceneDesc;
//	sceneDesc.broadPhase	= NX_BROADPHASE_FULL;
	sceneDesc.gravity		= ToNxVec3(desc.mGravity);
//	sceneDesc.maxBounds		= &Bounds;
	mScene = mSDK->createScene(sceneDesc);
	ASSERT(mScene);

	mTouchedShapes = NX_NEW(NxArraySDK<NxShape*>);

	UpdateFromUI();
}

void NovodeX::SetGravity(const Point& gravity)
{
	ASSERT(mScene);
	mScene->setGravity(ToNxVec3(gravity));
}

void NovodeX::Close()
{
//	printf("CHECKPOINT 0\n");
	const NxU32 NbMaterials = mMaterials.size();
	for(NxU32 i=0;i<NbMaterials;i++)
	{
		NxMaterial* M = mMaterials[i];
		DELETESINGLE(M);
	}

//	printf("CHECKPOINT 1\n");
	if(mScene)
	{
		mSDK->releaseScene(*mScene);
		mScene = null;
	}
//	printf("CHECKPOINT 2\n");
	SAFE_RELEASE(mSDK)

	DELETESINGLE(mTouchedShapes);

	DELETESINGLE(gMyNxUserAllocator);
}

void NovodeX::UpdateFromUI()
{
	if(!mScene)
		return;

	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		mSDK->setParameter(gDebugVizIndex[i], float(gDebugVizParams[i]));
}

udword NovodeX::Update(float dt)
{
	if(mScene)
	{
		mScene->setTiming(1.0f/60.0f, 8, NX_TIMESTEP_FIXED);
		mScene->startRun(dt);
		mScene->finishRun();
		while(!mScene->wait(NX_FENCE_RUN_FINISHED, true));
	}
	return gMyNxUserAllocator->mCurrentMemory;
}

Point NovodeX::GetMainColor()
{
	return Point(0.0f, 1.0f, 0.0f);
}

	class MyDebugRenderer : public NxUserDebugRenderer
	{
		public:
		virtual void renderData(const NxDebugRenderable& data) const
		{
			udword NbLines = data.getNbLines();
			const NxDebugLine* Lines = data.getLines();
			const Point LineColor(1.0f, 1.0f, 1.0f);
			for(udword i=0;i<NbLines;i++)
			{
				mRenderer->DrawLine(ToPoint(Lines[i].p0), ToPoint(Lines[i].p1), LineColor);
			}
		}

		PintRender*	mRenderer;
	}gDebugRender;

void NovodeX::Render(PintRender& renderer)
{
	if(mScene)
	{
		NxU32 nbActors = mScene->getNbActors();
		NxActor** actors = mScene->getActors();
		for(NxU32 i=0;i<nbActors;i++)
		{
			NxActor* actor = actors[i];

			const NxU32 nbShapes = actor->getNbShapes();
			NxShape** shapes = actor->getShapes();
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
	}

	if(mSDK && gDebugVizParams[0])
	{
		gDebugRender.mRenderer = &renderer;
		mSDK->visualize(gDebugRender);
	}
}

udword NovodeX::CreateMaterial(const PINT_MATERIAL_CREATE& desc)
{
	const NxU32 NbMaterials = mMaterials.size();
	for(NxU32 i=0;i<NbMaterials;i++)
	{
		NxMaterial* M = mMaterials[i];
		if(		M->restitution==desc.mRestitution
			&&	M->staticFriction==desc.mStaticFriction
			&&	M->dynamicFriction==desc.mDynamicFriction)
		{
			return i;
		}
	}

	NxMaterial* M = new NxMaterial;
	ASSERT(M);
	M->restitution		= desc.mRestitution;
	M->staticFriction	= desc.mStaticFriction;
	M->dynamicFriction	= desc.mDynamicFriction;
	mMaterials.push_back(M);

	ASSERT(mSDK);
	mSDK->setMaterialAtIndex(NxMaterialIndex(NbMaterials), M);

	return NbMaterials;
}

PintObjectHandle NovodeX::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(mSDK);
	ASSERT(mScene);

	NxBodyDesc bodyDesc;
//	bodyDesc.massLocalPose			= ;
//	bodyDesc.massSpaceInertia		= ;
	bodyDesc.mass					= desc.mMass;
	bodyDesc.linearVelocity			= ToNxVec3(desc.mLinearVelocity);
	bodyDesc.angularVelocity		= ToNxVec3(desc.mAngularVelocity);
	if(!gEnableSleeping)
		bodyDesc.wakeUpCounter		= 999999999.0f;
	bodyDesc.linearDamping			= gLinearDamping;
	bodyDesc.angularDamping			= gAngularDamping;
//	bodyDesc.maxAngularVelocity		= ;
	bodyDesc.flags					= desc.mKinematic ? NX_BF_KINEMATIC : 0;
//	bodyDesc.sleepLinearVelocity	= ;
//	bodyDesc.sleepAngularVelocity	= ;
	bodyDesc.solverIterationCount	= gSolverIterationCount;

	NxActorDesc actorDesc;
	actorDesc.globalPose.t	= ToNxVec3(desc.mPosition);
	actorDesc.globalPose.M	= NxMat33(ToNxQuat(desc.mRotation));
//	actorDesc.density		= ;
//	actorDesc.flags			= ;
//	actorDesc.group			= ;
//	actorDesc.userData		= ;
//	actorDesc.name			= ;

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
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			NxTriangleMeshDesc meshDesc;
			meshDesc.numVertices			= MeshCreate->mSurface.mNbVerts;
			meshDesc.numTriangles			= MeshCreate->mSurface.mNbFaces;
			meshDesc.pointStrideBytes		= sizeof(NxVec3);
			meshDesc.triangleStrideBytes	= sizeof(udword)*3;
			meshDesc.points					= MeshCreate->mSurface.mVerts;
			meshDesc.triangles				= MeshCreate->mSurface.mDFaces;
//			meshDesc.flags					= NX_MF_FLIPNORMALS;
			meshDesc.flags					= 0;

			NxTriangleMesh* TriMesh = mSDK->createTriangleMesh(meshDesc);
			mMeshes.push_back(TriMesh);

			NxTriangleMeshShapeDesc* triangleMeshShapeDesc = new NxTriangleMeshShapeDesc;
			triangleMeshShapeDesc->meshData		= TriMesh;
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
			shapeDesc->group			= desc.mCollisionGroup;
		//	shapeDesc->userData			= ;
		//	shapeDesc->name				= ;

			if(gDebugVizParams[0])
				shapeDesc->shapeFlags	|= NX_SF_VISUALIZATION;
			else
				shapeDesc->shapeFlags	&= ~NX_SF_VISUALIZATION;

			if(CurrentShape->mMaterial)
			{
				PINT_MATERIAL_CREATE Desc;
				Desc.mStaticFriction		= CurrentShape->mMaterial->mStaticFriction;
				Desc.mDynamicFriction		= CurrentShape->mMaterial->mDynamicFriction;
				Desc.mRestitution			= CurrentShape->mMaterial->mRestitution;
				shapeDesc->materialIndex	= (NxMaterialIndex)CreateMaterial(Desc);
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

	NxShape** ActorShapes = actor->getShapes();
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

bool NovodeX::ReleaseObject(PintObjectHandle handle)
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

PintJointHandle NovodeX::CreateJoint(const PINT_JOINT_CREATE& desc)
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
}

void NovodeX::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	ASSERT(mSDK);
	for(udword i=0;i<nb_groups;i++)
		mSDK->setGroupCollisionFlag(NxCollisionGroup(groups[i].mGroup0), NxCollisionGroup(groups[i].mGroup1), false);
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const NxRaycastHit& result)
{
	hit.mObject			= CreateHandle(result.shape);
	hit.mImpact			= ToPoint(result.worldImpact);
	hit.mNormal			= ToPoint(result.worldNormal);
	hit.mDistance		= result.distance;
	hit.mTriangleIndex	= result.faceID;
}

udword NovodeX::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* _dest, const PintRaycastData* _raycasts)
{
	ASSERT(mScene);

	udword NbHits = 0;
	const NxU32 groups = gSQFilterOutAllShapes ? 0 : 0xffffffff;
	const NxU32 hintFlags = 0xffffffff;

#define DEFAULT_CODE
#ifdef DEFAULT_CODE
	PintRaycastHit* __restrict dest = _dest;
	const PintRaycastData* __restrict raycasts = _raycasts;
	NxRaycastHit Hit;
	while(nb--)
	{
		const NxRay worldRay(ToNxVec3(raycasts->mOrigin), ToNxVec3(raycasts->mDir));
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
#endif

	mScene->dumpRaycastStats();

	return NbHits;
}

udword NovodeX::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);
	ASSERT(mTouchedShapes);

	udword NbHits = 0;
	while(nb--)
	{
		mTouchedShapes->clear();
		const NxU32 Nb = mScene->overlapSphereShapes(NxSphere(ToNxVec3(overlaps->mSphere.mCenter), overlaps->mSphere.mRadius), NX_ALL_SHAPES, *mTouchedShapes);
		if(Nb)
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

udword NovodeX::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);
	ASSERT(mTouchedShapes);

	udword NbHits = 0;
	while(nb--)
	{
		mTouchedShapes->clear();
		const NxU32 Nb = mScene->overlapSphereShapes(NxSphere(ToNxVec3(overlaps->mSphere.mCenter), overlaps->mSphere.mRadius), NX_ALL_SHAPES, *mTouchedShapes);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

PR NovodeX::GetWorldTransform(PintObjectHandle handle)
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

void NovodeX::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
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

bool NovodeX::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	NxActor* Actor = GetActorFromHandle(handle);
	if(!Actor)
		return false;

	Actor->moveGlobalPosition(ToNxVec3(pos));
	return true;
}


static NovodeX* gNovodeX = null;
static void gNovodeX_GetOptionsFromGUI();

void NovodeX_Init(const PINT_WORLD_CREATE& desc)
{
	gNovodeX_GetOptionsFromGUI();

	ASSERT(!gNovodeX);
	gNovodeX = ICE_NEW(NovodeX);
	gNovodeX->Init(desc);
}

void NovodeX_Close()
{
	if(gNovodeX)
	{
		gNovodeX->Close();
		delete gNovodeX;
		gNovodeX = null;
	}
}

NovodeX* GetNovodeX()
{
	return gNovodeX;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gNovodeXGUI = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum NovodeXGUIElement
{
	NOVODEX_GUI_MAIN,
	//
	NOVODEX_GUI_ENABLE_SLEEPING,
	NOVODEX_GUI_ADAPTIVE_FORCE,
	NOVODEX_GUI_MESH_HINT_SPEED,
	NOVODEX_GUI_SQ_FILTER_OUT,
	//
	NOVODEX_GUI_SOLVER_ITER,
	NOVODEX_GUI_LINEAR_DAMPING,
	NOVODEX_GUI_ANGULAR_DAMPING,
	//
	NOVODEX_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case NOVODEX_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case NOVODEX_GUI_ADAPTIVE_FORCE:
			gAdaptiveForce = checked;
			break;
		case NOVODEX_GUI_MESH_HINT_SPEED:
			gMeshHintSpeed = checked;
			break;
		case NOVODEX_GUI_SQ_FILTER_OUT:
			gSQFilterOutAllShapes = checked;
			break;
		case NOVODEX_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>NOVODEX_GUI_ENABLE_DEBUG_VIZ && id<NOVODEX_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-NOVODEX_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	if(gNovodeX)
		gNovodeX->UpdateFromUI();
}

static void gNovodeX_GetOptionsFromGUI()
{
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
}

IceWindow* NovodeX_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gNovodeXGUI, parent, NOVODEX_GUI_MAIN, "NovodeX 2.1.1 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, NOVODEX_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gNovodeXGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, NOVODEX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gNovodeXGUI, gAdaptiveForce, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, NOVODEX_GUI_MESH_HINT_SPEED, 4, y, CheckBoxWidth, 20, "Mesh hint speed", gNovodeXGUI, gMeshHintSpeed, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, NOVODEX_GUI_SQ_FILTER_OUT, 4, y, CheckBoxWidth, 20, "Filter out all shapes (DEBUG)", gNovodeXGUI, gSQFilterOutAllShapes, gCheckBoxCallback);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, NOVODEX_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gNovodeXGUI);

	y += YStep;

	sdword OffsetX = 90;
	const sdword EditBoxWidth = 60;
	const sdword LabelOffsetY = 2;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num solver iter:", gNovodeXGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, NOVODEX_GUI_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gNovodeXGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gNovodeXGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, NOVODEX_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gNovodeXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gNovodeXGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, NOVODEX_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gNovodeXGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;
	}

	y += YStep;
	return Main;
}

void NovodeX_CloseGUI()
{
	Common_CloseGUI(gNovodeXGUI);

	gEditBox_SolverIter = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}

///////////////////////////////////////////////////////////////////////////////

class NovodeXPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return NovodeX_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ NovodeX_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ NovodeX_Init(desc);						}
	virtual	void		Close()												{ NovodeX_Close();							}
	virtual	Pint*		GetPint()											{ return GetNovodeX();						}
};
static NovodeXPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
