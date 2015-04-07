///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

/*
This plugin was adapted from the 3.9 version provided by Julio. It currently has the same restrictions.

Note that you currently cannot run PEEL with both Newton versions, since they use the same DLLs (same names).
So you must compile the plugin you want to try each time (which copies the correct DLLs to PEEL's build folder).
*/

#include "stdafx.h"
#include "PINT_Newton3_13.h"
#include "..\PINT_Common\PINT_Common.h"

#include "Newton.h"
#include "CustomJoint.h"
#include "CustomHinge.h"
#include "CustomSlider.h"
#include "CustomBallAndSocket.h"

///////////////////////////////////////////////////////////////////////////////

class PinkCookie 
{
	public:
	PinkCookie (PintShapeRenderer* const renderer, int groupID)
		:m_collisionGroup (groupID)
		,m_renderer(renderer)
	{

	}
	int m_collisionGroup;
	PintShapeRenderer* m_renderer;
};



static			float	gLinearDamping					= 0.1f;
static			float	gAngularDamping					= 0.05f;
static			bool	gUseCCD							= false;
static			bool	gEnableSleeping					= false;
static			udword	gSolverIterationCount			= 1;


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
	0,
	0,
	0,
	0,
	0,
	0,
};

///////////////////////////////////////////////////////////////////////////////


// this is the call back for allocation newton memory
class AllocatiorSyngleton
{
	public:
	struct Header
	{
		udword	mMagic;
		udword	mSize;
	};

	static AllocatiorSyngleton& GetAllocator()
	{
		static AllocatiorSyngleton allocator;
		return allocator;
	}

	AllocatiorSyngleton()
		:gNbAllocs(0)
		,gCurrentMemory(0)
	{
		// set the memory allocators
		NewtonSetMemorySystem (AllocMemory, FreeMemory);
	}

	static void* AllocMemory (int sizeInBytes)
	{
		AllocatiorSyngleton& me = GetAllocator();
		me.gNbAllocs++;
		me.gCurrentMemory+=sizeInBytes;
		return malloc (sizeInBytes);
	}

	// this is the callback for freeing Newton Memory
	static void FreeMemory (void *ptr, int sizeInBytes)
	{
		AllocatiorSyngleton& me = GetAllocator();
		me.gNbAllocs--;
		me.gCurrentMemory-=sizeInBytes;
		free (ptr);
	}

	udword	gNbAllocs;
	udword	gCurrentMemory;
};

///////////////////////////////////////////////////////////////////////////////

class MyDebugDrawer
{
	public:

	MyDebugDrawer() : mRenderer(null), mDebugMode(0)
	{
	}

	virtual void	reportErrorWarning(const char* warningString)
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
Newton::Newton() 
	:mWorld(NULL)
	,mGlobalGravity(0.0f, -9.8f, 0.0f)
{
}

Newton::~Newton()
{
}

void Newton::GetCaps(PintCaps& caps) const
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
	caps.mSupportPhantoms				= true;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= true;
	caps.mSupportSphereSweeps			= true;
	caps.mSupportCapsuleSweeps			= true;
	caps.mSupportConvexSweeps			= true;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= true;
	caps.mSupportCapsuleOverlaps		= true;
	caps.mSupportConvexOverlaps			= true;
}

void Newton::Init(const PINT_WORLD_CREATE& desc)
{
	// make sure the global allocation singletons is initialized
	AllocatiorSyngleton::GetAllocator();

	for(udword i=0; i<32; i++) {
		mGroupMasks[i] = 0xffffffff;
	}

	// create the Newton World
	mWorld = NewtonCreate ();

	// configure the Newton world to use iterative solve mode 0
	// this is the most efficient but the less accurate mode
	NewtonSetSolverModel(mWorld, gSolverIterationCount);

	// link this world and his his owner
	// Set the Newton world user data
	NewtonWorldSetUserData(mWorld, this);

	// all Physx functionality can be support with only one node of the newton material graph
	int materialNodeID = NewtonMaterialGetDefaultGroupID (mWorld);
	NewtonMaterialSetCollisionCallback (mWorld, materialNodeID, materialNodeID, this, OnAABBOverlap, NULL);


	// set constructor and destructor for collision shapes 
	NewtonWorldSetCollisionConstructorDestuctorCallback (mWorld, DuplicateCollisionShape, DestroyCollisionShape);
}


void Newton::Close()
{
	// Terminate any asynchronous operation.
	NewtonWaitForUpdateToFinish (mWorld);

	// destroy all rigid bodies, this is no necessary because Newton Destroy world will also destroy all bodies
	// but if you want to change level and restart you can call this function to clean the world without destroying the world.
	NewtonDestroyAllBodies (mWorld);

	// finally destroy the newton world 
	NewtonDestroy (mWorld);
}

const char*	Newton::GetName() const	
{ 
	static bool versionRead;
	static char name [256];
	{
		int ver = NewtonWorldGetVersion ();
		int major = ver / 100;
		int minor = ver - major * 100;
		sprintf (name, "Newton %d.%d", major, minor);
	}
	return name;	
}


void Newton::SetGravity(const Point& gravity)
{
	mGlobalGravity = gravity;
}

udword Newton::Update(float dt)
{
	// run the newton update function
	NewtonUpdate(mWorld,dt);

	return AllocatiorSyngleton::GetAllocator().gCurrentMemory;
}

Point Newton::GetMainColor()
{
	return Point(0.4f, 1.0f, 0.4f);
}

void Newton::Render(PintRender& renderer, NewtonCollision* const collision, const IceMaths::Matrix4x4& pose) const
{
	NewtonCollisionInfoRecord info;
	NewtonCollisionGetInfo(collision, &info);

	IceMaths::Matrix4x4 localPose = *(const IceMaths::Matrix4x4 *)info.m_offsetMatrix;
	IceMaths::Matrix4x4 worldPose (localPose * pose);

	switch (info.m_collisionType)
	{
		case SERIALIZE_ID_BOX:
		case SERIALIZE_ID_SPHERE:
		case SERIALIZE_ID_TREE:
		case SERIALIZE_ID_CONVEXHULL:
		{
			PR rpose(worldPose);
			PinkCookie * const cookie = (PinkCookie *) NewtonCollisionGetUserData(collision);
			_ASSERTE (cookie);
			cookie->m_renderer->Render(rpose);
			break;
		}

		case SERIALIZE_ID_CAPSULE:
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(-3.141592f * 90.0f / 180.0f);
			worldPose = aligment * worldPose;
			PR rpose(worldPose);
			PinkCookie * const cookie = (PinkCookie *) NewtonCollisionGetUserData(collision);
			_ASSERTE (cookie);
			cookie->m_renderer->Render(rpose);
			break;
		}

//		case SERIALIZE_ID_CONVEXHULL:
//		{
//			_ASSERTE (0);
//			int faceArray[2048];
//			int indexArray[2048 * 3];
//			int indexCount = 0;
//			int faceCount = info.m_convexHull.m_faceCount;
//			_ASSERTE (faceCount < sizeof (faceArray) / sizeof (faceArray[0]));
//			for (int i = 0; i < info.m_convexHull.m_faceCount; i ++) {
//				faceArray[i] = NewtonConvexHullGetFaceIndices (collision, i, &indexArray[indexCount]);
//				indexCount += faceArray[i];
//				_ASSERTE (indexCount < sizeof (indexArray) / sizeof (indexArray[0]));
//			}
//			const float* const vertex = info.m_convexHull.m_vertex;
//			int vertexCount = info.m_convexHull.m_vertexCount;
//			int stride = info.m_convexHull.m_vertexStrideInBytes / sizeof (float);
//			PR rpose(worldPose);
//			renderer.DrawMesh (vertexCount, stride, vertex, faceCount, faceArray, indexArray, pose);
//			break;
//		}

//		case SERIALIZE_ID_TREE:
//		{
//			_ASSERTE (0);
//			Point p0(-1000.0f, -1000.0f, -1000.0f);
//			Point p1( 1000.0f,  1000.0f,  1000.0f);
//			int indexList[1024 * 16]; 
//			int faceAttribute[sizeof (indexList) / (3 * sizeof (indexList[0]))];
//			int vertexCount;
//			int stride;
//			int maxIndexCount = sizeof (indexList) / sizeof (indexList[0]);
//			const float* vertexArray;
//			int faceCount = NewtonTreeCollisionGetVertexListTriangleListInAABB (collision, &p0.x, &p1.x, &vertexArray, &vertexCount, &stride, indexList, maxIndexCount, faceAttribute); 				

			// trash the faceAttribute, and write teh faceIndexcount
//			for (int i = 0; i < faceCount; i ++) {
//				faceAttribute[i] = 3;
//			}
//			PR rpose(worldPose);
//			renderer.DrawMesh (vertexCount, stride/ sizeof (float), vertexArray, faceCount, faceAttribute, indexList, pose);
//			break;
//		}

		case SERIALIZE_ID_COMPOUND:
		{
			for (void* collisionNode = NewtonCompoundCollisionGetFirstNode (collision); collisionNode; collisionNode = NewtonCompoundCollisionGetNextNode(collision, collisionNode)) {
				NewtonCollision* const childCollision = NewtonCompoundCollisionGetCollisionFromNode (collision, collisionNode);
				Render(renderer, childCollision, worldPose);
			}
			break;
		}

		default:
			_ASSERTE (0);
	}

}

void Newton::Render(PintRender& renderer)
{
	for (NewtonBodyVector::iterator i=mBodies.begin(); i!=mBodies.end(); ++i)
	{
		NewtonBody *nb = (*i);
		IceMaths::Matrix4x4 pose;
		NewtonBodyGetMatrix(nb,(float *)&pose);

		NewtonCollision *nc = NewtonBodyGetCollision(nb);
		IceMaths::Matrix4x4 worldPose (pose);
		Render (renderer, nc, worldPose);
	}
}

// callback to apply external forces to body
void Newton::ApplyForceAndTorqueCallback (const NewtonBody* const body, dFloat timestep, int threadIndex)
{
	float Ixx;
	float Iyy;
	float Izz;
	float mass;

	const Newton* const peelNewton = (Newton*) NewtonWorldGetUserData (NewtonBodyGetWorld(body));

	// for this tutorial the only external force in the Gravity
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	Point weight;
	weight = weight.Mult (peelNewton->mGlobalGravity, mass);
	NewtonBodySetForce(body, &weight[0]);
}


NewtonCollision* Newton::CreateShape(NewtonWorld* const world, const PINT_SHAPE_CREATE* const shape, const PintShapeRenderer* const renderer, int groupID) const
{
	IceMaths::Matrix4x4 localPose = shape->mLocalRot;
	localPose.SetTrans(shape->mLocalPos);

	NewtonCollision *ret = NULL;
	// Physx only handle a subset of the assortment of newton collision shapes in newton
	switch (shape->mType)
	{

		case PINT_SHAPE_SPHERE:
		{
			const PINT_SPHERE_CREATE* const collShape = static_cast<const PINT_SPHERE_CREATE*>(shape);
			ret = NewtonCreateSphere (world, collShape->mRadius, 0, (const float *)&localPose);
			break;
		}

		case PINT_SHAPE_CAPSULE:
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);

			localPose = aligment * localPose;
			const PINT_CAPSULE_CREATE* const collShape = static_cast<const PINT_CAPSULE_CREATE*>(shape);
			ret = NewtonCreateCapsule (world, collShape->mRadius, collShape->mHalfHeight * 2.0f, 0, (const float *)&localPose);
			break;
		}

		case PINT_SHAPE_BOX: 
		{
			const PINT_BOX_CREATE* const collShape = static_cast<const PINT_BOX_CREATE*>(shape);
			ret = NewtonCreateBox(world, collShape->mExtents.x * 2.0f, collShape->mExtents.y * 2.0f, collShape->mExtents.z * 2.0f, 0,(const float *)&localPose);
			break;
		}


		case PINT_SHAPE_CONVEX:
		{
			const PINT_CONVEX_CREATE* const collShape = static_cast<const PINT_CONVEX_CREATE*>(shape);
			ret = NewtonCreateConvexHull(world, collShape->mNbVerts, &collShape->mVerts[0].x, sizeof (Point), 0.0f, 0, (const float *)&localPose);
			break;
		}

		case PINT_SHAPE_MESH:
		{
			const PINT_MESH_CREATE* const collShape = static_cast<const PINT_MESH_CREATE*>(shape);

			ret = NewtonCreateTreeCollision (world, 0);
			NewtonTreeCollisionBeginBuild(ret);
			int faceCount = collShape->mSurface.mNbFaces;
			const Point* const points = collShape->mSurface.mVerts;
			Point face[32];
			if (collShape->mSurface.mWFaces) {
				const uword* const indexArray = collShape->mSurface.mWFaces;
				for (int i = 0; i < faceCount; i ++) {
					face[0] = points[indexArray[i * 3 + 0]];
					face[1] = points[indexArray[i * 3 + 1]];
					face[2] = points[indexArray[i * 3 + 2]];
					NewtonTreeCollisionAddFace(ret, 3, &face[0].x, sizeof (Point), 0);
				}
			} else {
				const udword* const indexArray = collShape->mSurface.mDFaces;
				for (int i = 0; i < faceCount; i ++) {
					face[0] = points[indexArray[i * 3 + 0]];
					face[1] = points[indexArray[i * 3 + 1]];
					face[2] = points[indexArray[i * 3 + 2]];
					NewtonTreeCollisionAddFace(ret, 3, &face[0].x, sizeof (Point), 0);
				}
			}
			NewtonTreeCollisionEndBuild(ret, 0);
			break;
		}
	}

	// save the display render as the shape user data

	PinkCookie * const cookie = new PinkCookie ((PintShapeRenderer*) renderer, groupID);
	NewtonCollisionSetUserData(ret, cookie);
	return ret;
}

void Newton::DuplicateCollisionShape (const NewtonWorld* const newtonWorld, NewtonCollision* const newCollision, const NewtonCollision* const sourceCollision)
{
	PinkCookie * const srcCookie = (PinkCookie *)NewtonCollisionGetUserData(sourceCollision);
	PinkCookie * const newCookie = new PinkCookie (*srcCookie);
	NewtonCollisionSetUserData(newCollision, newCookie);
}

void Newton::DestroyCollisionShape (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision)
{
	PinkCookie * const cookie = (PinkCookie *)NewtonCollisionGetUserData(collision);
	if (cookie) {
		delete cookie;
	}
}


NewtonCollision* Newton::CreateShape(const PINT_OBJECT_CREATE& desc) const
{
	// see if this is a single shape
	const PINT_SHAPE_CREATE* const currentShape = desc.mShapes;
	if (!currentShape->mNext) {
		// make a single collision shape, and return that
		return CreateShape(mWorld, currentShape, currentShape->mRenderer, desc.mCollisionGroup);
	} else {
		//we have a compound or a scene collision
		// compound can only have convex shapes on them, while scene can have all other shapes except  
		// other compounds or other scene collisions.

		// first see if this shape has any static mesh on then
		bool isCompoundCollision = true;
		for (const PINT_SHAPE_CREATE* shape = currentShape; shape; shape = shape->mNext) {
			if (shape->mType == PINT_SHAPE_MESH) {
				isCompoundCollision = false;
				break;
			}
		}

		NewtonCollision* collision = NULL;
		if (isCompoundCollision) {
			collision = NewtonCreateCompoundCollision (mWorld, 0);
			NewtonCompoundCollisionBeginAddRemove (collision);	
			for (const PINT_SHAPE_CREATE* shape = currentShape; shape; shape = shape->mNext) {
				NewtonCollision* const subCollision = CreateShape(mWorld, shape, shape->mRenderer, desc.mCollisionGroup);
				NewtonCompoundCollisionAddSubCollision (collision, subCollision);	
				NewtonDestroyCollision(subCollision);
			}
			NewtonCompoundCollisionEndAddRemove (collision);	

			// add a dummy cookie to keep the collision callback happy
			PinkCookie * const cookie = new PinkCookie (NULL, desc.mCollisionGroup);
			NewtonCollisionSetUserData(collision, cookie);


		} else {
			collision = NewtonCreateSceneCollision (mWorld, 0);
			NewtonSceneCollisionBeginAddRemove (collision);
			for (const PINT_SHAPE_CREATE* shape = currentShape; shape; shape = shape->mNext) {
				NewtonCollision* const subCollision = CreateShape(mWorld, shape, shape->mRenderer, desc.mCollisionGroup);
				NewtonSceneCollisionAddSubCollision (collision, subCollision);	
				NewtonDestroyCollision(subCollision);
			}
			NewtonSceneCollisionEndAddRemove (collision);	

			// add a dummy cookie to keep the collision callback happy
			PinkCookie * const cookie = new PinkCookie (NULL, desc.mCollisionGroup);
			NewtonCollisionSetUserData(collision, cookie);

		}
		return collision;
	}

	return NULL;
}


PintObjectHandle Newton::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	if(!desc.mShapes) {
		return NULL;
	}

	ASSERT(mWorld);

	NewtonCollision* const shape = CreateShape(desc);
	if ( shape == NULL )
	{
		return NULL;
	}

	IceMaths::Matrix4x4 pose = desc.mRotation;
	pose.SetTrans(desc.mPosition);

	if ( mBodies.empty() )
	{
		pose.SetTrans( desc.mPosition.x, desc.mPosition.y, desc.mPosition.z );
	}

	NewtonBody* const ret = NewtonCreateDynamicBody(mWorld,shape,(const float*)&pose);
	if ( desc.mMass != 0.0f ) // if it's a static object
	{
		NewtonBodySetMassProperties (ret, desc.mMass, shape);
		NewtonBodySetVelocity(ret,&desc.mLinearVelocity.x);
		NewtonBodySetOmega(ret,&desc.mAngularVelocity.x);
		NewtonBodySetForceAndTorqueCallback(ret, ApplyForceAndTorqueCallback);
	}

	NewtonDestroyCollision(shape);

	mBodies.push_back(ret);
	return ret;
}

bool Newton::ReleaseObject(PintObjectHandle handle)
{
	return true;
}

PintJointHandle Newton::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	CustomJoint* joint = NULL;

	NewtonBody* const body0 = (NewtonBody*)desc.mObject0;
	NewtonBody* const body1 = (NewtonBody*)desc.mObject1;

	switch (desc.mType) 
	{
		case PINT_JOINT_SPHERICAL:	// Spherical joint a.k.a. point-to-point constraint a.k.a. ball-and-socket
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			matrix.m_posit = matrix.TransformVector (dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f));
			joint = new CustomBallAndSocket(matrix, body0, body1);
			break;
		}

		case PINT_JOINT_HINGE:		// Hinge joints a.k.a. revolute joints
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			dMatrix pinMatrix (dGrammSchmidt (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 1.0f)));
			pinMatrix.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);

			matrix = pinMatrix * matrix;
			CustomHinge* const hinge = new CustomHinge (matrix, matrix, body0, body1);
			hinge->EnableLimits (true);
			hinge->SetLimis (jc.mMinLimitAngle, jc.mMaxLimitAngle);
			joint = hinge;
			break;
		}

		case PINT_JOINT_PRISMATIC:	// Prismatic joints, a.k.a. slider constraints
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			dMatrix pinMatrix (dGrammSchmidt (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 1.0f)));
			pinMatrix.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);

			matrix = pinMatrix * matrix;
			CustomSlider* const slider = new CustomSlider (matrix, body0, body1);

			// it appears that physx slider do not have limits
			joint = slider;
			break;
		}

		case PINT_JOINT_FIXED:
		{
			// Newton do no have a dedicated fix joint by we can use a hinge with zero angular limits
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			CustomHinge* const fixHinge = new CustomHinge (matrix, matrix, body0, body1);
			fixHinge->EnableLimits (true);
			fixHinge->SetLimis (0.0f, 0.0f);
			joint = fixHinge;
			break;
		}

	}
	
	return joint;
}

void Newton::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i = 0; i < nb_groups; i++) {
		int group0 = groups[i].mGroup0; 
		int group1 = groups[i].mGroup1; 

		unsigned bit1 = 1 << group1;
		mGroupMasks[group0] ^= bit1;

		unsigned bit0 = 1 << group0;
		mGroupMasks[group1] ^= bit0;
	}
}

int	Newton::OnAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{
//	Newton* const me = NewtonMaterialGetUserData(material);
	Newton* const me = (Newton*) NewtonWorldGetUserData(NewtonBodyGetWorld(body0));
	NewtonCollision* const collsion0 = NewtonBodyGetCollision(body0);
	NewtonCollision* const collsion1 = NewtonBodyGetCollision(body1);

	PinkCookie* const cookie0 = (PinkCookie*) NewtonCollisionGetUserData(collsion0);
	PinkCookie* const cookie1 = (PinkCookie*) NewtonCollisionGetUserData(collsion1);

	_ASSERTE (cookie0);
	_ASSERTE (cookie1);

	return (me->mGroupMasks[cookie0->m_collisionGroup] & (1<<cookie1->m_collisionGroup)) ? 1 : 0;
}

dFloat Newton::GenericRayCast (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const normal, int* const collisionID, void* const userData, dFloat intersetParam)
{
	PintRaycastHit* const data = (PintRaycastHit*) userData;
//	struct PintRaycastHit : public Allocateable
//	{
//		PintObjectHandle	mObject;
//		Point				mImpact;
//		Point				mNormal;
//		float				mDistance;
//		udword				mTriangleIndex;
//	};
	if (intersetParam < data->mDistance) {
		data->mDistance = intersetParam;
		data->mObject = (PintObjectHandle*)body;
		data->mNormal = Point (normal[0], normal[1], normal[2]);
		// phyx doe not provide room for passing what sub shape of a collision shape was hit. only what face of a polginal mesh
	}
	return data->mDistance;
}

static dFloat gNewtonWorldRayFilterCallback(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam)
{
	PintRaycastHit* const data = (PintRaycastHit*) userData;
//	struct PintRaycastHit : public Allocateable
//	{
//		PintObjectHandle	mObject;
//		Point				mImpact;
//		Point				mNormal;
//		float				mDistance;
//		udword				mTriangleIndex;
//	};
	if (intersectParam < data->mDistance) {
		data->mDistance = intersectParam;
		data->mObject = (PintObjectHandle*)body;
		data->mNormal = Point (hitNormal[0], hitNormal[1], hitNormal[2]);
		// phyx doe not provide room for passing what sub shape of a collision shape was hit. only what face of a polginal mesh
	}
	return data->mDistance;
}

udword Newton::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	// if we are making a small number of cast do not bother with batching
/*	if (nb == 1) {
		dest->mDistance = 1.2f;
		dVector p0 (raycasts->mOrigin.x, raycasts->mOrigin.y, raycasts->mOrigin.z, 1.0f); 
		dVector p1 (p0 + dVector (raycasts->mDir.x, raycasts->mDir.y, raycasts->mDir.z, 1.0f).Scale (raycasts->mMaxDist)); 
		dFloat parameter = 1.2f;
		NewtonWorldRayCast (mWorld, &p0[0], &p1[0], GenericRayCast, dest, NULL);
		if (dest->mDistance < 1.0f) {
			dVector p (p0 + (p1 - p0).Scale (dest->mDistance));

			dest->mDistance *= raycasts->mMaxDist;
			dest->mImpact = Point (p[0], p[1], p[2]);
			return 1;
		}
	} else {

	}

	return 0;*/


	udword NbHits = 0;
	while(nb--)
	{
		dest->mDistance = 1.2f;
		dVector p0 (raycasts->mOrigin.x, raycasts->mOrigin.y, raycasts->mOrigin.z, 1.0f); 
		dVector p1 (p0 + dVector (raycasts->mDir.x, raycasts->mDir.y, raycasts->mDir.z, 1.0f).Scale (raycasts->mMaxDist)); 
		dFloat parameter = 1.2f;
//		NewtonWorldRayCast (mWorld, &p0[0], &p1[0], GenericRayCast, dest, NULL);
		NewtonWorldRayCast (mWorld, &p0[0], &p1[0], gNewtonWorldRayFilterCallback, dest, 0, 0);
		if (dest->mDistance < 1.0f)
		{
			dVector p (p0 + (p1 - p0).Scale (dest->mDistance));

			dest->mDistance *= raycasts->mMaxDist;
			dest->mImpact = Point (p[0], p[1], p[2]);
			NbHits++;
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


udword Newton::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	_ASSERTE (0);
	return 0;
}

udword Newton::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	_ASSERTE (0);
	return 0;
}


udword Newton::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	return 0;
}

PR Newton::GetWorldTransform(PintObjectHandle handle)
{
	NewtonBody* const body = (NewtonBody*)handle;
	Matrix4x4 matrix;
	NewtonBodyGetMatrix(body, &matrix.m[0][0]);
	return PR (matrix);
}

void Newton::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	NewtonBody* const body = (NewtonBody*)handle;
	if(action_type==PINT_ACTION_FORCE) {
		// this is one case were Physx and Newton are different,
		// force can only be applied form with in a force and torque call back.
		// still can apply the force in the form of an impulse bu we need to know the time step
		// for now I will simply assert 
		_ASSERTE (0);
	} else {
		NewtonBodyApplyImpulseArray (body, 1, sizeof (Point), &action.x, &pos[0]);
	}

}


static Newton* gNewton = null;
static void gNewton_GetOptionsFromGUI();

void Newton_Init(const PINT_WORLD_CREATE& desc)
{
	gNewton_GetOptionsFromGUI();

	ASSERT(!gNewton);
	gNewton = ICE_NEW(Newton);
	gNewton->Init(desc);
}

void Newton_Close()
{
	if(gNewton)
	{
		gNewton->Close();
		delete gNewton;
		gNewton = null;
	}
}

Newton* GetNewton()
{
	return gNewton;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gNewtonGUI = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_CollisionMargin = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum NewtonGUIElement
{

	NEWTON_GUI_MAIN,
	//
	NEWTON_GUI_SOLVER_ITER,
	NEWTON_GUI_ENABLE_SLEEPING,
	NEWTON_GUI_USE_CCD,
	NEWTON_GUI_LINEAR_DAMPING,
	NEWTON_GUI_ANGULAR_DAMPING,
	//
	NEWTON_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case NEWTON_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case NEWTON_GUI_USE_CCD:
			gUseCCD = checked;
			break;
		case NEWTON_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>NEWTON_GUI_ENABLE_DEBUG_VIZ && id<NEWTON_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-NEWTON_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	gDebugDrawer.updateDebugMode();
}

static void gNewton_GetOptionsFromGUI()
{
	if(gEditBox_SolverIter)
	{
		sdword tmp;
		bool status = gEditBox_SolverIter->GetTextAsInt(tmp);
		ASSERT(status);
		if (tmp < 1) {
			tmp = 1;
			gEditBox_SolverIter->SetText("1");
		}

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

IceWindow* Newton_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gNewtonGUI, parent, NEWTON_GUI_MAIN, "Newton 3.08 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, NEWTON_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gNewtonGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, NEWTON_GUI_USE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gNewtonGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, NEWTON_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gNewtonGUI);

	y += YStep;

	sdword OffsetX = 90;
	const sdword EditBoxWidth = 60;
	const sdword LabelOffsetY = 2;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num solver iter:", gNewtonGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, NEWTON_GUI_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gNewtonGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gNewtonGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, NEWTON_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gNewtonGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gNewtonGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, NEWTON_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gNewtonGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;
	}

	y += YStep;

	return Main;
}

void Newton_CloseGUI()
{
	Common_CloseGUI(gNewtonGUI);

	gEditBox_SolverIter = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	gEditBox_CollisionMargin = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}

///////////////////////////////////////////////////////////////////////////////

class NewtonPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Newton_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Newton_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Newton_Init(desc);						}
	virtual	void		Close()												{ Newton_Close();							}
	virtual	Pint*		GetPint()											{ return GetNewton();						}
};
static NewtonPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
