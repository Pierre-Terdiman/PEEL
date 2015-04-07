///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_Opcode13.h"
#include "..\PINT_Common\PINT_Common.h"
#include "..\PINT_Common\PINT_IceAllocatorSwitch.h"

static	bool	gDrawMeshAABBs	= false;
static	bool	gQuantized		= false;
static	bool	gNoLeaf			= false;

// For some insane reason this function was missing in Opcode 1.3 (only from the OBB collider!) and I never noticed!
class SceneOBBCollider : public OBBCollider
{
	public:

	bool	Collide(OBBCache& cache, const OBB& box, const AABBTree* tree);
	void	_Collide(const AABBTreeNode* node);
};

bool SceneOBBCollider::Collide(OBBCache& cache, const OBB& box, const AABBTree* tree)
{
	// This is typically called for a scene tree, full of -AABBs-, not full of triangles.
	// So we don't really have "primitives" to deal with. Hence it doesn't work with
	// "FirstContact" + "TemporalCoherence".
	ASSERT( !(FirstContactEnabled() && TemporalCoherenceEnabled()) );

	// Checkings
	if(!tree)	return false;

	// Init collision query
	if(InitQuery(cache, box))	return true;

	// Perform collision query
	_Collide(tree);

	return true;
}

void SceneOBBCollider::_Collide(const AABBTreeNode* node)
{
	// Perform OBB-AABB overlap test
	Point Center, Extents;
	node->GetAABB()->GetCenter(Center);
	node->GetAABB()->GetExtents(Extents);
	if(!BoxBoxOverlap(Extents, Center))	return;

	if(node->IsLeaf() || OBBContainsBox(Center, Extents))
	{
		mFlags |= OPC_CONTACT;
		mTouchedPrimitives->Add(node->GetPrimitives(), node->GetNbPrimitives());
	}
	else
	{
		_Collide(node->GetPos());
		_Collide(node->GetNeg());
	}
}



///////////////////////////////////////////////////////////////////////////////

OpcodeMesh::OpcodeMesh() : mRenderer(null)
{
}

OpcodeMesh::~OpcodeMesh()
{
}

///////////////////////////////////////////////////////////////////////////////

OpcodeActor::OpcodeActor()
{
	mMeshTM.Identity();
}

OpcodeActor::~OpcodeActor()
{
}

///////////////////////////////////////////////////////////////////////////////

Opcode13Pint::Opcode13Pint() : mSceneTree(null)
{
}

Opcode13Pint::~Opcode13Pint()
{
	ASSERT(!mSceneTree);
}

void Opcode13Pint::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= false;
	caps.mSupportKinematics				= false;
	caps.mSupportCollisionGroups		= false;
	caps.mSupportCompounds				= false;
	caps.mSupportConvexes				= false;
	caps.mSupportMeshes					= true;
	caps.mSupportSphericalJoints		= false;
	caps.mSupportHingeJoints			= false;
	caps.mSupportFixedJoints			= false;
	caps.mSupportPrismaticJoints		= false;
	caps.mSupportPhantoms				= false;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= false;
	caps.mSupportSphereSweeps			= false;
	caps.mSupportCapsuleSweeps			= false;
	caps.mSupportConvexSweeps			= false;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= true;
	caps.mSupportCapsuleOverlaps		= false;
	caps.mSupportConvexOverlaps			= false;
}

void Opcode13Pint::Init(const PINT_WORLD_CREATE& desc)
{
	InitIceAllocator(GetName());

	AllocSwitch _;
}

void Opcode13Pint::SetGravity(const Point& gravity)
{
}

void Opcode13Pint::Close()
{
	{
		AllocSwitch _;

		DELETESINGLE(mSceneTree);

		const udword NbMeshes = mMeshes.GetNbEntries();
		for(udword i=0;i<NbMeshes;i++)
		{
			OpcodeMesh* Mesh = (OpcodeMesh*)mMeshes.GetEntries()[i];
			DELETESINGLE(Mesh);
		}

		const udword NbActors = mActors.GetNbEntries();
		for(udword i=0;i<NbActors;i++)
		{
			OpcodeActor* Actor = (OpcodeActor*)mActors.GetEntries()[i];
			DELETESINGLE(Actor);
		}

		mActors.Empty();
		mMeshes.Empty();
		mWorldBoxes.Empty();
		//mBoxIndices.Empty();
	}

	ReleaseIceAllocator();
}

udword Opcode13Pint::Update(float dt)
{
	AllocSwitch _;

	if(!mSceneTree)
	{
		mSceneTree = ICE_NEW(AABBTree);

		const udword NbObjects = mActors.GetNbEntries();
		const AABB* Boxes = (const AABB*)mWorldBoxes.GetEntries();

		AABBTreeOfAABBsBuilder TB;
		TB.mNbPrimitives	= NbObjects;
		TB.mAABBArray		= Boxes;
		TB.mSettings.mRules	= SPLIT_SPLATTER_POINTS|SPLIT_GEOM_CENTER;
		TB.mSettings.mLimit	= 1;
		bool Status = mSceneTree->Build(&TB);
	}

	return GetIceAllocatorUsedMemory();
}

Point Opcode13Pint::GetMainColor()
{
	return Point(1.0f, 1.0f, 1.0f);
}

void Opcode13Pint::Render(PintRender& renderer)
{
	AllocSwitch _;

	const udword NbActors = mActors.GetNbEntries();
	for(udword i=0;i<NbActors;i++)
	{
		const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[i];
		const OpcodeMesh* Mesh = Actor->mMesh;
		if(Mesh->mRenderer)
			Mesh->mRenderer->Render(Actor->mMeshTM);
	}

	if(gDrawMeshAABBs)
	{
		const AABB* Boxes = (const AABB*)mWorldBoxes.GetEntries();
		for(udword i=0;i<NbActors;i++)
		{
			renderer.DrawWirefameAABB(Boxes[i], Point(1.0f, 0.0f, 0.0f));
		}
	}
}

PintObjectHandle Opcode13Pint::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	AllocSwitch _;

	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	if(desc.mMass!=0.0f)
		return null;

	Matrix4x4 M = desc.mRotation;
	M.SetTrans(desc.mPosition);

	PintObjectHandle Handle = null;

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
		if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);

			ASSERT(MeshCreate->mSurface.mNbFaces);
			ASSERT(MeshCreate->mSurface.mDFaces);

			Matrix4x4 M2 = CurrentShape->mLocalRot;
			M2.SetTrans(CurrentShape->mLocalPos);

			OpcodeMesh* NewMesh = null;
			{
				const udword NbMeshes = mMeshes.GetNbEntries();
				for(udword i=0;i<NbMeshes;i++)
				{
					OpcodeMesh* CurrentMesh = (OpcodeMesh*)mMeshes.GetEntry(i);
					if(CurrentMesh->mRenderer==CurrentShape->mRenderer)
					{
						NewMesh = CurrentMesh;
						break;
					}
				}
			}

			DELETESINGLE(mSceneTree);
			if(!NewMesh)
			{
				NewMesh = ICE_NEW(OpcodeMesh);
				mMeshes.Add(udword(NewMesh));

				NewMesh->mSurface.Init(MeshCreate->mSurface.mNbFaces, MeshCreate->mSurface.mNbVerts, MeshCreate->mSurface.mVerts, (const IndexedTriangle*)MeshCreate->mSurface.mDFaces);

				NewMesh->mMeshInterface.SetNbVertices(NewMesh->mSurface.GetNbVerts());
				NewMesh->mMeshInterface.SetNbTriangles(NewMesh->mSurface.GetNbFaces());
				NewMesh->mMeshInterface.SetPointers(NewMesh->mSurface.GetFaces(), NewMesh->mSurface.GetVerts());

				OPCODECREATE opcodeCreate;
				opcodeCreate.mIMesh				= &NewMesh->mMeshInterface;
				opcodeCreate.mNoLeaf			= gNoLeaf;
				opcodeCreate.mQuantized			= gQuantized;
				opcodeCreate.mSettings.mLimit	= 1;
				opcodeCreate.mSettings.mRules	= Opcode::SPLIT_SPLATTER_POINTS | Opcode::SPLIT_GEOM_CENTER;
				opcodeCreate.mKeepOriginal		= false;

				bool Status = NewMesh->mModel.Build(opcodeCreate);
				ASSERT(Status);

				NewMesh->mRenderer = CurrentShape->mRenderer;
			}

			OpcodeActor* NewActor = ICE_NEW(OpcodeActor);
			Handle = NewActor;	// #### this is wrong
			mActors.Add(udword(NewActor));
			NewActor->mMesh = NewMesh;
			NewActor->mMeshTM = M * M2;

			{
				AABB* Memory = (AABB*)mWorldBoxes.Reserve(sizeof(AABB)/sizeof(udword));
				ComputeAABB(*Memory, NewMesh->mSurface.GetVerts(), NewMesh->mSurface.GetNbVerts(), NewActor->mMeshTM);
			}
		}
		CurrentShape = CurrentShape->mNext;
	}

	return Handle;
}

bool Opcode13Pint::ReleaseObject(PintObjectHandle handle)
{
	return false;
}

PintJointHandle Opcode13Pint::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	return null;
}

void Opcode13Pint::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
}

void* Opcode13Pint::CreatePhantom(const AABB& box)
{
	return null;
}

udword Opcode13Pint::BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void** phantoms)
{
	return 0;
}

PintSQThreadContext Opcode13Pint::CreateSQThreadContext()
{
	AllocSwitch _;
//	printf("Creating thread context\n");
	return ICE_NEW(SQThreadContext);
}

void Opcode13Pint::ReleaseSQThreadContext(PintSQThreadContext context)
{
	AllocSwitch _;
//	printf("Releasing thread context\n");
	SQThreadContext* C = (SQThreadContext*)context;
	DELETESINGLE(C);
}

udword Opcode13Pint::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		RayCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		CollisionFace Memory;
		CollisionFaces CF;
		CF.InitSharedBuffers(sizeof(CollisionFace)/sizeof(udword), (udword*)&Memory);

		RayCollider RC;
		RC.SetFirstContact(false);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);
		RC.SetCulling(true);
		RC.SetClosestHit(true);
		RC.SetDestination(&CF);

		udword NbHits = 0;
		while(nb--)
		{
			const Ray& CurrentRay = *reinterpret_cast<const Ray*>(&raycasts->mOrigin.x);

			SceneRC.SetMaxDist(raycasts->mMaxDist);
			mBoxIndices.Reset();
			if(SceneRC.Collide(CurrentRay, mSceneTree, mBoxIndices))
			{
				const OpcodeActor* TouchedActor = null;
				CollisionFace Hit;
				Hit.mDistance = raycasts->mMaxDist;

				udword NbMeshes = mBoxIndices.GetNbEntries();
				for(udword i=0;i<NbMeshes;i++)
				{
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[mBoxIndices[i]];
					const OpcodeMesh* Mesh = Actor->mMesh;

					RC.SetMaxDist(Hit.mDistance);
					Memory.mDistance = MAX_FLOAT;
					if(RC.Collide(CurrentRay, Mesh->mModel, &Actor->mMeshTM/*, udword* cache=null*/))
					{
						if(Memory.mDistance<Hit.mDistance)
						{
							Hit = Memory;
							TouchedActor = Actor;
						}
					}
				}

				if(TouchedActor)
				{
					const OpcodeMesh* TouchedMesh = TouchedActor->mMesh;

					NbHits++;

					dest->mObject			= (PintObjectHandle)TouchedActor;	// ###
					dest->mDistance			= Hit.mDistance;
					dest->mTriangleIndex	= Hit.mFaceID;

					const Point* V = TouchedMesh->mMeshInterface.GetVerts();
					const IndexedTriangle& T = TouchedMesh->mMeshInterface.GetTris()[Hit.mFaceID];
					const Point& p0 = V[T.mRef[0]];
					const Point& p1 = V[T.mRef[1]];
					const Point& p2 = V[T.mRef[2]];

					Point LocalPt;
					ComputeBarycentricPoint(LocalPt, p0, p1, p2, Hit.mU, Hit.mV);
					TransformPoint4x3(dest->mImpact, LocalPt, TouchedActor->mMeshTM);

					const Point LocalNormal = ((p0-p1)^(p0-p2)).Normalize();
					TransformPoint3x3(dest->mNormal, LocalNormal, TouchedActor->mMeshTM);
				}
				else
				{
					dest->mObject = null;
				}
			}

			raycasts++;
			dest++;
		}
		return NbHits;
	}
	return 0;

#ifdef OLD
	CollisionFace Memory;
	CollisionFaces CF;
	CF.InitSharedBuffers(sizeof(CollisionFace)/sizeof(udword), (udword*)&Memory);

	RayCollider RC;
	RC.SetFirstContact(false);
	RC.SetTemporalCoherence(false);
	RC.SetPrimitiveTests(true);
	RC.SetCulling(true);
	RC.SetClosestHit(true);
	RC.SetDestination(&CF);

	udword NbHits = 0;
	while(nb--)
	{
		const OpcodeMesh* TouchedMesh = null;
		CollisionFace Hit;
		Hit.mDistance = raycasts->mMaxDist;

		udword NbMeshes = mMeshes.GetNbEntries();
		for(udword i=0;i<NbMeshes;i++)
		{
			const OpcodeMesh* Mesh = (const OpcodeMesh*)mMeshes.GetEntries()[i];

			RC.SetMaxDist(Hit.mDistance);
			Memory.mDistance = MAX_FLOAT;
			if(RC.Collide(Ray(raycasts->mOrigin, raycasts->mDir), Mesh->mModel, &Mesh->mMeshTM/*, udword* cache=null*/))
			{
				if(Memory.mDistance<Hit.mDistance)
				{
					Hit = Memory;
					TouchedMesh = Mesh;
				}
			}
		}

		if(TouchedMesh)
		{
			NbHits++;

			dest->mObject			= (PintObjectHandle)TouchedMesh;
			dest->mDistance			= Hit.mDistance;
			dest->mTriangleIndex	= Hit.mFaceID;

			const Point* V = TouchedMesh->mMeshInterface.GetVerts();
			const IndexedTriangle& T = TouchedMesh->mMeshInterface.GetTris()[Hit.mFaceID];
			const Point& p0 = V[T.mRef[0]];
			const Point& p1 = V[T.mRef[1]];
			const Point& p2 = V[T.mRef[2]];

			const float w = 1.0f - Hit.mU - Hit.mV;
			dest->mImpact.x = w*p0.x + Hit.mU*p1.x + Hit.mV*p2.x;
			dest->mImpact.y = w*p0.y + Hit.mU*p1.y + Hit.mV*p2.y;
			dest->mImpact.z = w*p0.z + Hit.mU*p1.z + Hit.mV*p2.z;

			dest->mNormal = ((p0-p1)^(p0-p2)).Normalize();
		}
		else
		{
			dest->mObject = null;
		}

		raycasts++;
		dest++;
	}
	return NbHits;
#endif
}

udword Opcode13Pint::BatchRaycastAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		RayCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		RayCollider RC;
		RC.SetFirstContact(true);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);
		RC.SetCulling(true);
		RC.SetClosestHit(false);

		udword NbHits = 0;
		while(nb--)
		{
			const Ray& CurrentRay = *reinterpret_cast<const Ray*>(&raycasts->mOrigin.x);

			SceneRC.SetMaxDist(raycasts->mMaxDist);
			mBoxIndices.Reset();
			if(SceneRC.Collide(CurrentRay, mSceneTree, mBoxIndices))
			{
				bool AnyHit=false;
				udword NbMeshes = mBoxIndices.GetNbEntries();
				for(udword i=0;i<NbMeshes;i++)
				{
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[mBoxIndices[i]];
					const OpcodeMesh* Mesh = Actor->mMesh;

					RC.SetMaxDist(raycasts->mMaxDist);
					if(RC.Collide(CurrentRay, Mesh->mModel, &Actor->mMeshTM/*, udword* cache=null*/))
					{
						if(RC.GetContactStatus())
						{
							AnyHit = true;
							break;
						}
					}
				}
				NbHits += AnyHit;
				dest->mHit = AnyHit;
			}

			raycasts++;
			dest++;
		}
		return NbHits;
	}
	return 0;
}

udword Opcode13Pint::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	return 0;
}

udword Opcode13Pint::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	return 0;
}

udword Opcode13Pint::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	return 0;
}

udword Opcode13Pint::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		SphereCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		SphereCollider RC;
		RC.SetFirstContact(true);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);

		SphereCache Cache;
		Cache.TouchedPrimitives = mBoxIndices;

		Container mBoxIndices2;
		udword Entry;
		mBoxIndices2.InitSharedBuffers(1, &Entry);
		SphereCache Cache2;
		Cache2.TouchedPrimitives = mBoxIndices2;

		udword NbHits = 0;
		while(nb--)
		{
			udword Touched=0;
			if(SceneRC.Collide(Cache, overlaps->mSphere, mSceneTree) && SceneRC.GetContactStatus())
			{
				const udword NbMeshes = SceneRC.GetNbTouchedPrimitives();
				for(udword i=0;i<NbMeshes;i++)
				{
					const udword Index = SceneRC.GetTouchedPrimitives()[i];
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[Index];
					const OpcodeMesh* Mesh = Actor->mMesh;

					if(RC.Collide(Cache2, overlaps->mSphere, Mesh->mModel, null, &Actor->mMeshTM) && RC.GetContactStatus())
					{
						Touched = 1;
						NbHits++;
						break;
					}
				}
			}

			dest->mHit = Touched!=0;
			dest++;
			overlaps++;
		}
		return NbHits;
	}
	return 0;
}

udword Opcode13Pint::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		SphereCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		SphereCollider RC;
		RC.SetFirstContact(true);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);

		SphereCache Cache;
		Cache.TouchedPrimitives = mBoxIndices;

		Container mBoxIndices2;
		udword Entry;
		mBoxIndices2.InitSharedBuffers(1, &Entry);
		SphereCache Cache2;
		Cache2.TouchedPrimitives = mBoxIndices2;

		udword NbHits = 0;
		while(nb--)
		{
			udword Touched=0;
			if(SceneRC.Collide(Cache, overlaps->mSphere, mSceneTree) && SceneRC.GetContactStatus())
			{
				const udword NbMeshes = SceneRC.GetNbTouchedPrimitives();
				for(udword i=0;i<NbMeshes;i++)
				{
					const udword Index = SceneRC.GetTouchedPrimitives()[i];
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[Index];
					const OpcodeMesh* Mesh = Actor->mMesh;

					if(RC.Collide(Cache2, overlaps->mSphere, Mesh->mModel, null, &Actor->mMeshTM) && RC.GetContactStatus())
					{
						Touched++;
					}
				}
			}

			NbHits += Touched;
			dest->mNbObjects = Touched;
			dest++;
			overlaps++;
		}
		return NbHits;
	}
	return 0;
}

udword Opcode13Pint::BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		SceneOBBCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		OBBCollider RC;
		RC.SetFirstContact(true);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);
		RC.SetFullBoxBoxTest(false);

		OBBCache Cache;
		Cache.TouchedPrimitives = mBoxIndices;

		Container mBoxIndices2;
		udword Entry;
		mBoxIndices2.InitSharedBuffers(1, &Entry);
		OBBCache Cache2;
		Cache2.TouchedPrimitives = mBoxIndices2;

		udword NbHits = 0;
		while(nb--)
		{
			udword Touched=0;
			if(SceneRC.Collide(Cache, overlaps->mBox, mSceneTree) && SceneRC.GetContactStatus())
			{
				const udword NbMeshes = SceneRC.GetNbTouchedPrimitives();
				for(udword i=0;i<NbMeshes;i++)
				{
					const udword Index = SceneRC.GetTouchedPrimitives()[i];
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[Index];
					const OpcodeMesh* Mesh = Actor->mMesh;

					if(RC.Collide(Cache2, overlaps->mBox, Mesh->mModel, null, &Actor->mMeshTM) && RC.GetContactStatus())
					{
						Touched = 1;
						NbHits++;
						break;
					}
				}
			}

			dest->mHit = Touched!=0;
			dest++;
			overlaps++;
		}
		return NbHits;
	}
	return 0;
}

udword Opcode13Pint::BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(mSceneTree)
	{
		SQThreadContext* C = (SQThreadContext*)context;
		Container& mBoxIndices = C->mBoxIndices;

		SceneOBBCollider SceneRC;
		SceneRC.SetFirstContact(false);
		SceneRC.SetTemporalCoherence(false);
		SceneRC.SetPrimitiveTests(true);

		OBBCollider RC;
		RC.SetFirstContact(true);
		RC.SetTemporalCoherence(false);
		RC.SetPrimitiveTests(true);
		RC.SetFullBoxBoxTest(false);

		OBBCache Cache;
		Cache.TouchedPrimitives = mBoxIndices;

		Container mBoxIndices2;
		udword Entry;
		mBoxIndices2.InitSharedBuffers(1, &Entry);
		OBBCache Cache2;
		Cache2.TouchedPrimitives = mBoxIndices2;

		udword NbHits = 0;
		while(nb--)
		{
			udword Touched=0;
			if(SceneRC.Collide(Cache, overlaps->mBox, mSceneTree) && SceneRC.GetContactStatus())
			{
				const udword NbMeshes = SceneRC.GetNbTouchedPrimitives();
				for(udword i=0;i<NbMeshes;i++)
				{
					const udword Index = SceneRC.GetTouchedPrimitives()[i];
					const OpcodeActor* Actor = (const OpcodeActor*)mActors.GetEntries()[Index];
					const OpcodeMesh* Mesh = Actor->mMesh;

					if(RC.Collide(Cache2, overlaps->mBox, Mesh->mModel, null, &Actor->mMeshTM) && RC.GetContactStatus())
					{
						Touched++;
					}
				}
			}

			NbHits += Touched;
			dest->mNbObjects = Touched;
			dest++;
			overlaps++;
		}
		return NbHits;
	}
	return 0;
}

udword Opcode13Pint::FindTriangles_MeshSphereOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)
{
	AllocSwitch _;

	ASSERT(mSceneTree);

	if(!mSceneTree)
		return 0;

	SphereCollider RC;
	RC.SetFirstContact(false);
	RC.SetTemporalCoherence(false);
	RC.SetPrimitiveTests(true);

	SphereCache Cache;

	OpcodeActor* Actor = (OpcodeActor*)handle;
	const OpcodeMesh* Mesh = Actor->mMesh;

	udword NbTouchedTriangles = 0;
	while(nb--)
	{
		if(RC.Collide(Cache, overlaps->mSphere, Mesh->mModel, null, &Actor->mMeshTM) && RC.GetContactStatus())
		{
			NbTouchedTriangles += Cache.TouchedPrimitives.GetNbEntries();
		}

		overlaps++;
	}
	return NbTouchedTriangles;
}

PR Opcode13Pint::GetWorldTransform(PintObjectHandle handle)
{
	return PR();
}

void Opcode13Pint::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
}

udword Opcode13Pint::GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)
{
	return 0;
}

void Opcode13Pint::SetLocalRot(PintObjectHandle handle, const Quat& q)
{
}

bool Opcode13Pint::GetConvexData(PintObjectHandle handle, PINT_CONVEX_CREATE& data)
{
	return false;
}


static Opcode13Pint* gOpcode = null;

void Opcode_Init(const PINT_WORLD_CREATE& desc)
{
	ASSERT(!gOpcode);
	gOpcode = ICE_NEW(Opcode13Pint);
	gOpcode->Init(desc);
}

void Opcode_Close()
{
	if(gOpcode)
	{
		gOpcode->Close();
		delete gOpcode;
		gOpcode = null;
	}
}

Opcode13Pint* GetOpcode()
{
	return gOpcode;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gOpcodeGUI = null;
static IceCheckBox*	gCheckBox_DrawMeshAABBS = null;
static IceCheckBox*	gCheckBox_Quantized = null;
static IceCheckBox*	gCheckBox_NoLeaf = null;

enum OpcodeGUIElement
{
	OPCODE_GUI_MAIN,
	//
	OPCODE_GUI_DRAW_MESH_AABBS,
	//
	OPCODE_GUI_QUANTIZED,
	OPCODE_GUI_NOLEAF,
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case OPCODE_GUI_DRAW_MESH_AABBS:
			gDrawMeshAABBs = checked;
			break;
		case OPCODE_GUI_QUANTIZED:
			gQuantized = checked;
			break;
		case OPCODE_GUI_NOLEAF:
			gNoLeaf = checked;
			break;
	}

//	if(gPhysX)
//		gPhysX->UpdateFromUI();
}

IceWindow* Opcode_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gOpcodeGUI, parent, OPCODE_GUI_MAIN, "Opcode 1.3 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		gCheckBox_DrawMeshAABBS = helper.CreateCheckBox(Main, OPCODE_GUI_DRAW_MESH_AABBS, 4, y, CheckBoxWidth, 20, "Draw mesh AABBs", gOpcodeGUI, gDrawMeshAABBs, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_Quantized = helper.CreateCheckBox(Main, OPCODE_GUI_QUANTIZED, 4, y, CheckBoxWidth, 20, "Quantized", gOpcodeGUI, gQuantized, gCheckBoxCallback);
		y += YStepCB;

		gCheckBox_NoLeaf = helper.CreateCheckBox(Main, OPCODE_GUI_NOLEAF, 4, y, CheckBoxWidth, 20, "No Leaf", gOpcodeGUI, gNoLeaf, gCheckBoxCallback);
		y += YStepCB;
	}

	return Main;
}

void Opcode_CloseGUI()
{
	Common_CloseGUI(gOpcodeGUI);
	gCheckBox_DrawMeshAABBS = null;
	gCheckBox_Quantized = null;
	gCheckBox_NoLeaf = null;
}

///////////////////////////////////////////////////////////////////////////////

class OpcodePlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Opcode_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Opcode_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Opcode_Init(desc);						}
	virtual	void		Close()												{ Opcode_Close();							}
	virtual	Pint*		GetPint()											{ return GetOpcode();						}
};
static OpcodePlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
