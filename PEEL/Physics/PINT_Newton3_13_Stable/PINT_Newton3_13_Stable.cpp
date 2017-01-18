///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

/*
PINT Plugin for Newton 3.13's stable version released on Jun 17, 2015.

Uses the vs2010 static libs:
- vs2010 to be able to link it in a VC9 project
- static libs to be able to run multiple versions of Newton at the same time

The Newton library has been compiled using the provided Visual Studio project. The only change is that
DG_SSE4_INSTRUCTIONS_SET has been undefined, so that the library doesn't crash on machines that do not
support SSE4.
*/

#include "stdafx.h"
#include "PINT_Newton3_13_Stable.h"
#include "..\PINT_Common\PINT_Common.h"

#include "Newton.h"
#include "CustomJoint.h"
#include "CustomGear.h"
#include "CustomHinge.h"
#include "CustomSlider.h"
#include "CustomPulley.h"
#include "CustomUniversal.h"
#include "CustomCorkScrew.h"
#include "CustomRackAndPinion.h"
#include "CustomBallAndSocket.h"
#include "..\PINT_Common\PINT_CommonNewton.h"

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

class CriticalSection
{
public: 
	CriticalSection (int* atomicLock)
		:m_atomicLock (atomicLock)
	{
		while (NewtonAtomicSwap(m_atomicLock, 1)) {
			NewtonYield();
		}
	}
	~CriticalSection ()
	{
		NewtonAtomicSwap(m_atomicLock, 0);
	}

	int* m_atomicLock; 
};

class PinkCollisionShapeCookie 
{
public:
	PinkCollisionShapeCookie (PintShapeRenderer* const renderer, int groupID)
		:m_collisionGroup (groupID)
		,m_renderer(renderer)
	{
	}

	void *operator new (size_t size)
	{
		return AllocatiorSyngleton::GetAllocator().AllocMemory (int (size));
	}

	void operator delete (void* ptr)
	{
		AllocatiorSyngleton::GetAllocator().FreeMemory (ptr, sizeof (PinkCollisionShapeCookie));
	}

	int m_collisionGroup;
	PintShapeRenderer* m_renderer;
	NewtonPint::PinkShapeMaterial m_material;
};

class PinkRigidBodyCookie 
{
public:
	PinkRigidBodyCookie(bool is_kine) :
		m_extLinearImpulse	(0.0f, 0.0f, 0.0f, 0.0f),
		m_extAngularImpulse	(0.0f, 0.0f, 0.0f, 0.0f),
		m_lock				(0),
		mIsKinematic		(is_kine),
		// added by PT:
		mTorque				(0.0f, 0.0f, 0.0f, 0.0f)
	{
	}

	void* operator new (size_t size)
	{
		return AllocatiorSyngleton::GetAllocator().AllocMemory (int (size));
	}

	void operator delete (void* ptr)
	{
		AllocatiorSyngleton::GetAllocator().FreeMemory (ptr, sizeof (PinkCollisionShapeCookie));
	}

	void CalculatePickForceAndTorque (const NewtonBody* const body, dFloat timestep, dVector& force, dVector& torque)
	{
		force = dVector (0.0f, 0.0f, 0.0f, 0.0f);
		torque = dVector (0.0f, 0.0f, 0.0f, 0.0f);

		CriticalSection (&this->m_lock);
		if (((m_extLinearImpulse % m_extLinearImpulse) != 0.0f) || ((m_extAngularImpulse % m_extAngularImpulse) != 0.0f)) {
			dVector com; 
			dMatrix matrix; 
			dVector omega0;
			dVector veloc0;
			dVector omega1;
			dVector veloc1;
			dVector pointVeloc;

			dFloat invTimeStep = 1.0f / timestep;

			// calculate the desired impulse
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetOmega (body, &omega0[0]);
			NewtonBodyGetVelocity (body, &veloc0[0]);

			dFloat stiffness = 0.3f;
			dVector posit (m_extLinearImpulse);
			dVector deltaVeloc (m_extAngularImpulse);

			NewtonBodyGetPointVelocity (body, &posit[0], &pointVeloc[0]);
			deltaVeloc = deltaVeloc.Scale (stiffness * invTimeStep) - pointVeloc;

			for (int i = 0; i < 3; i ++) {
				dVector veloc (0.0f, 0.0f, 0.0f, 0.0f);
				veloc[i] = deltaVeloc[i];
				NewtonBodyAddImpulse (body, &veloc[0], &posit[0]);
			}

			// damp angular velocity
			NewtonBodyGetOmega (body, &omega1[0]);
			NewtonBodyGetVelocity (body, &veloc1[0]);
			omega1 = omega1.Scale (0.9f);

			// restore body velocity and angular velocity
			NewtonBodySetOmega (body, &omega0[0]);
			NewtonBodySetVelocity(body, &veloc0[0]);

			// convert the delta velocity change to a external force and torque
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

			dVector angularMomentum (Ixx, Iyy, Izz);
			angularMomentum = matrix.RotateVector (angularMomentum.CompProduct(matrix.UnrotateVector(omega1 - omega0)));

			force = (veloc1 - veloc0).Scale (mass * invTimeStep);
			torque = angularMomentum.Scale(invTimeStep);

			// reset the impulse accumulators for the next update
			m_extLinearImpulse = dVector (0.0f, 0.0f, 0.0f, 0.0f);
			m_extAngularImpulse = dVector (0.0f, 0.0f, 0.0f, 0.0f);
		}
	}

	dVector	m_extLinearImpulse;
	dVector	m_extAngularImpulse;
	int		m_lock;
	bool	mIsKinematic;
	// added by PT:
	dVector	mTorque;
};

///////////////////////////////////////////////////////////////////////////////

static	dFloat	gLinearDamping					= 0.1f;
static	dFloat	gAngularDamping					= 0.05f;
static	bool	gAsyncronousUpdate				= false;
static	bool	gUseCCD							= false;
static	bool	gEnableSleeping					= false;
static	bool	gResolveLargeIsalndInParallel	= false;
static	udword	gSolverIterationCount			= 4;
static	udword	gNbThreads						= 0;
static	udword	gBroadPhaseType					= 1;

static	const	char*	gDebugVizNames[] =
{
	"Enable debug visualization",
	"Draw collision shapes",
	//	"Draw AABBs",
	//	"Draw contact points",
	//	"Draw constraints",
	//	"Draw constraints limits",
};

#define	NB_DEBUG_VIZ_PARAMS	(sizeof (gDebugVizNames)/sizeof (gDebugVizNames[0]))

static	bool gDebugVizParams[NB_DEBUG_VIZ_PARAMS];
static	int	gDebugVizMask[NB_DEBUG_VIZ_PARAMS];


///////////////////////////////////////////////////////////////////////////////

class MyDebugDrawer
{
public:

	MyDebugDrawer() : mRenderer(null), mDebugMode(0)
	{
	}

	virtual ~MyDebugDrawer()
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

	static void RenderCollision (void* userData, int vertexCount, const dFloat* faceVertec, int id)
	{
		MyDebugDrawer* const me = (MyDebugDrawer*)userData;
		int i = vertexCount - 1;
		Point p0 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		for (int i = 0; i < vertexCount; i ++) {

			Point p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			me->mRenderer->DrawLine (p0, p1, me->m_color);
			p0 = p1;
		}
	}

	void RenderBodyCollision (const NewtonBody* const body)
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);	
		if (mass > 0.0f) {
			int sleepState = NewtonBodyGetSleepState(body);
			if (sleepState == 1) {
				// indicate when body is sleeping 
				m_color = Point (0.42f, 0.73f, 0.98f);
			} else {
				// body is active
				m_color = Point (1.0f, 1.0f, 1.0f);
			}
			dMatrix matrix;
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonCollisionForEachPolygonDo (NewtonBodyGetCollision(body), &matrix[0][0], RenderCollision, (void*)this);

		} else {

			NewtonCollisionInfoRecord info;

			//glColor3f(1.0f, 1.0f, 0.0f);
			m_color = Point (0.42f, 0.73f, 0.98f);
			NewtonCollision* const collision = NewtonBodyGetCollision (body);
			NewtonCollisionGetInfo (collision, &info);

			switch (info.m_collisionType) 
			{
				//case SERIALIZE_ID_TREE:
				//case SERIALIZE_ID_SCENE:
				case SERIALIZE_ID_USERMESH:
				case SERIALIZE_ID_HEIGHTFIELD:
				{
					break;
				}

				default: 
				{
					dMatrix matrix;
					NewtonBodyGetMatrix(body, &matrix[0][0]);
					//NewtonCollisionForEachPolygonDo (NewtonBodyGetCollision(body), &matrix[0][0], DebugShowGeometryCollision, NULL);
					NewtonCollisionForEachPolygonDo (NewtonBodyGetCollision(body), &matrix[0][0], RenderCollision, (void*)this);
					break;
				}
			}
		}
	}

	void RenderCollisions (const NewtonWorld* const world, PintRender* const renderer)
	{
		mRenderer = renderer;
		for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
			RenderBodyCollision (body);
		}
	}

	Point		m_color;
	PintRender*	mRenderer;
	int			mDebugMode;

}gDebugDrawer;

///////////////////////////////////////////////////////////////////////////////
NewtonPint::NewtonPint() :
	mWorld			(NULL),
	mCurrentTimeStep(0.0f),
	mGlobalGravity	(0.0f, 0.0f, 0.0f),
	mGravityMag		(0.0f),
	m_lastShapeCache(NULL)
{
}

NewtonPint::~NewtonPint()
{
}

void NewtonPint::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
	//JULIO
//	caps.mSupportCones					= true;	
	caps.mSupportCylinders				= true;
//	caps.mSupportTaperedCylinders		= true;
//	caps.mSupportChamferedCylinders		= true;
//	caps.mSupportTaperedCapsules		= true;
//	caps.mCollisionUniformScaling		= true;
//	caps.mCollisionNonUniformScaling	= true;

	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	//JULIO
//	caps.mSupportRelationalJoints		= true;
//	caps.mSupportCylindricalJoints		= true;
//	caps.mSupportUniversalJoints		= true;
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
	//JULIO
//	caps.mSupportVoronoidDecompositionGeneration = true;	 
}

void NewtonPint::Init(const PINT_WORLD_CREATE& desc)
{
	// make sure the global allocation singletons is initialized
	AllocatiorSyngleton::GetAllocator();

	for(udword i=0; i<32; i++) {
		mGroupMasks[i] = 0xffffffff;
	}

	SetGravity(desc.mGravity);

	// create the Newton World
	mWorld = NewtonCreate ();

	// configure the Newton world to use iterative solve mode 0
	// this is the most efficient but the less accurate mode
	NewtonSetSolverModel(mWorld, gSolverIterationCount);

	// Set the Broad Phase Type: 
	// 1: generic and is the default ( is kept around because in the future this will be converted to HPC multiprocessors system)
	// 2: pertinent good for single processors systems. 
	NewtonSelectBroadphaseAlgorithm (mWorld, gBroadPhaseType);

	// set the parallel solve option solve for large island
	NewtonSetMultiThreadSolverOnSingleIsland (mWorld, gResolveLargeIsalndInParallel);

	// set the number of micro thread
	NewtonSetThreadsCount(mWorld, gNbThreads ); 

	// link this world and his his owner
	// Set the Newton world user data
	NewtonWorldSetUserData(mWorld, this);

	// all Physx functionality can be support with only one node of the newton material graph
	int materialNodeID = NewtonMaterialGetDefaultGroupID (mWorld);
	NewtonMaterialSetCollisionCallback (mWorld, materialNodeID, materialNodeID, this, OnAABBOverlap, OnContactsProcess);

	// set constructor and destructor for collision shapes 
	NewtonWorldSetCollisionConstructorDestuctorCallback (mWorld, DuplicateCollisionShape, DestroyCollisionShape);

	// set joint serialization call back
	CustomJoint::Initalize(mWorld);

	// set the clast shape cache to NULL;
	m_lastShapeCache = NULL;
}


void NewtonPint::Close()
{
	// Terminate any asynchronous operation.
	NewtonWaitForUpdateToFinish (mWorld);

	// destroy the last cached queries if there is one
	if (m_lastShapeCache) {
		NewtonDestroyCollision(m_lastShapeCache);
		m_lastShapeCache = NULL;
	}

	// destroy all rigid bodies, this is no necessary because Newton Destroy world will also destroy all bodies
	// but if you want to change level and restart you can call this function to clean the world without destroying the world.
	NewtonDestroyAllBodies (mWorld);

	// finally destroy the newton world 
	NewtonDestroy (mWorld);
}

const char*	NewtonPint::GetName() const	
{ 
	static bool versionRead = false;
	static char name [256];
	if(!versionRead)
	{
		versionRead = true;
		int ver = NewtonWorldGetVersion ();
		int major = ver / 100;
		int minor = ver - major * 100;
		sprintf (name, "Newton %d.%02d", major, minor);
	}
	return name;	
}


void NewtonPint::SetGravity(const Point& gravity)
{
	mGlobalGravity = gravity;
	mGravityMag = dSqrt (mGlobalGravity.Dot(mGlobalGravity));
}

udword NewtonPint::Update(dFloat dt)
{
//	UpdateKinematics(dt);

	// save the current time step for future use
	mCurrentTimeStep = dt;
//	NewtonSerializeToFile (mWorld, "C:/tmp/test.bin", NULL, NULL);

	// run the newton update function
	if (gAsyncronousUpdate) {
		NewtonUpdateAsync(mWorld, dt);
	} else {
		NewtonUpdate(mWorld, dt);
	}
	UpdateKinematics(dt);

	return AllocatiorSyngleton::GetAllocator().gCurrentMemory;
}

Point NewtonPint::GetMainColor()
{
	return NEWTON_MAIN_COLOR;
}

void NewtonPint::Render(PintRender& renderer, NewtonCollision* const collision, const IceMaths::Matrix4x4& pose) const
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
			PinkCollisionShapeCookie * const cookie = (PinkCollisionShapeCookie *) NewtonCollisionGetUserData(collision);
			_ASSERTE (cookie);
			if (cookie->m_renderer) {
				cookie->m_renderer->Render(rpose);
			}
			break;
		}
		case SERIALIZE_ID_CONE:
		case SERIALIZE_ID_CAPSULE:
		case SERIALIZE_ID_CYLINDER:
		case SERIALIZE_ID_TAPEREDCAPSULE:
		case SERIALIZE_ID_TAPEREDCYLINDER:
		case SERIALIZE_ID_CHAMFERCYLINDER:
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(-3.141592f * 90.0f / 180.0f);
			worldPose = aligment * worldPose;
			PR rpose(worldPose);
			PinkCollisionShapeCookie * const cookie = (PinkCollisionShapeCookie *) NewtonCollisionGetUserData(collision);
			_ASSERTE (cookie);
			if (cookie->m_renderer) {
				cookie->m_renderer->Render(rpose);
			}
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
		//			const dFloat* const vertex = info.m_convexHull.m_vertex;
		//			int vertexCount = info.m_convexHull.m_vertexCount;
		//			int stride = info.m_convexHull.m_vertexStrideInBytes / sizeof (dFloat);
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
		//			const dFloat* vertexArray;
		//			int faceCount = NewtonTreeCollisionGetVertexListTriangleListInAABB (collision, &p0.x, &p1.x, &vertexArray, &vertexCount, &stride, indexList, maxIndexCount, faceAttribute); 				

		// trash the faceAttribute, and write teh faceIndexcount
		//			for (int i = 0; i < faceCount; i ++) {
		//				faceAttribute[i] = 3;
		//			}
		//			PR rpose(worldPose);
		//			renderer.DrawMesh (vertexCount, stride/ sizeof (dFloat), vertexArray, faceCount, faceAttribute, indexList, pose);
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

void NewtonPint::Render(PintRender& renderer)
{
	for (NewtonBody* body = NewtonWorldGetFirstBody(mWorld); body; body = NewtonWorldGetNextBody (mWorld, body))
	{
		IceMaths::Matrix4x4 pose;
		NewtonBodyGetMatrix(body, (dFloat *)&pose);

		NewtonCollision *nc = NewtonBodyGetCollision(body);
		IceMaths::Matrix4x4 worldPose (pose);
		Render (renderer, nc, worldPose);
	}

	if (gDebugVizParams[0])
	{
		// check if show collision shapes is on
		if (gDebugVizParams[1]) {
			gDebugDrawer.RenderCollisions (mWorld, &renderer);
		}
	}
}

// callback to apply external forces to body
void NewtonPint::ApplyForceAndTorqueCallback (const NewtonBody* const body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	// get the application relevant objects
	const NewtonPint* const peelNewton = (NewtonPint*) NewtonWorldGetUserData (NewtonBodyGetWorld(body));
	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	PinkRigidBodyCookie* const cookie = (PinkRigidBodyCookie*) NewtonBodyGetUserData(body);
	_ASSERTE (cookie);

	dVector force;
	dVector torque;
	cookie->CalculatePickForceAndTorque(body, timestep, force, torque);

	torque += cookie->mTorque;
	cookie->mTorque = dVector(0.0f, 0.0f, 0.0f);

	// calculate the weight
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	force += dVector (peelNewton->mGlobalGravity.x, peelNewton->mGlobalGravity.y, peelNewton->mGlobalGravity.z, 0.0f).Scale (mass);

	// add any other external force and impulses
	NewtonBodySetForce(body, &force[0]);

	dVector omega;
	dMatrix inertiaMatrix;
	NewtonBodyGetOmega(body, &omega[0]);
	NewtonBodyGetInertiaMatrix(body, &inertiaMatrix[0][0]);
	dVector angularMomentum (inertiaMatrix.RotateVector(omega));
	torque += angularMomentum * omega;
	NewtonBodySetTorque(body, &torque[0]);
}


NewtonCollision* NewtonPint::CreateShape(NewtonWorld* const world, const PINT_SHAPE_CREATE* const shape, const PintShapeRenderer* const renderer, int groupID) const
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
			ret = NewtonCreateSphere (world, collShape->mRadius, 0, &localPose.m[0][0]);
			break;
		}

		case PINT_SHAPE_CAPSULE:
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);

			localPose = aligment * localPose;
			const PINT_CAPSULE_CREATE* const collShape = static_cast<const PINT_CAPSULE_CREATE*>(shape);
			ret = NewtonCreateCapsule (world, collShape->mRadius, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}

		case PINT_SHAPE_BOX: 
		{
			const PINT_BOX_CREATE* const collShape = static_cast<const PINT_BOX_CREATE*>(shape);
			ret = NewtonCreateBox(world, collShape->mExtents.x * 2.0f, collShape->mExtents.y * 2.0f, collShape->mExtents.z * 2.0f, 0,&localPose.m[0][0]);
			break;
		}

	//JULIO
/*		case PINT_SHAPE_CONE: 
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);
			localPose = aligment * localPose;
			const PINT_CONE_CREATE* const collShape = static_cast<const PINT_CONE_CREATE*>(shape);
			ret = NewtonCreateCone (world, collShape->mRadius, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}
*/
		case PINT_SHAPE_CYLINDER: 
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);
			localPose = aligment * localPose;
			const PINT_CYLINDER_CREATE* const collShape = static_cast<const PINT_CYLINDER_CREATE*>(shape);
			ret = NewtonCreateCylinder(world, collShape->mRadius, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}
/*
		case PINT_SHAPE_CHAMFERED_CYLINDER: 
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);
			localPose = aligment * localPose;
			const PINT_CHAMFERED_CYLINDER_CREATE* const collShape = static_cast<const PINT_CHAMFERED_CYLINDER_CREATE*>(shape);
			ret = NewtonCreateChamferCylinder(world, collShape->mRadius, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}


		case PINT_SHAPE_TAPERED_CYLINDER: 
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);
			localPose = aligment * localPose;
			const PINT_TAPERED_CYLINDER_CREATE* const collShape = static_cast<const PINT_TAPERED_CYLINDER_CREATE*>(shape);
			ret = NewtonCreateTaperedCylinder(world, collShape->mRadius0, collShape->mRadius1, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}

		case PINT_SHAPE_TAPERED_CAPSULE: 
		{
			IceMaths::Matrix4x4 aligment;
			aligment.RotZ(3.141592f * 90.0f / 180.0f);
			localPose = aligment * localPose;
			const PINT_TAPERED_CAPSULE_CREATE* const collShape = static_cast<const PINT_TAPERED_CAPSULE_CREATE*>(shape);
			ret = NewtonCreateTaperedCapsule(world, collShape->mRadius0, collShape->mRadius1, collShape->mHalfHeight * 2.0f, 0, &localPose.m[0][0]);
			break;
		}*/


		case PINT_SHAPE_CONVEX:
		{
			const PINT_CONVEX_CREATE* const collShape = static_cast<const PINT_CONVEX_CREATE*>(shape);
			ret = NewtonCreateConvexHull(world, collShape->mNbVerts, &collShape->mVerts[0].x, sizeof (Point), 0.0f, 0, &localPose.m[0][0]);
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
			NewtonTreeCollisionEndBuild(ret, 1);
			break;
		}
	}

	// save the display render as the shape user data
	PinkCollisionShapeCookie* const cookie = new PinkCollisionShapeCookie ((PintShapeRenderer*) renderer, groupID);
	if (shape->mMaterial) {
		cookie->m_material.m_restitution = shape->mMaterial->mRestitution;
		cookie->m_material.m_staticFriction = dMax (shape->mMaterial->mDynamicFriction, shape->mMaterial->mStaticFriction); 
		cookie->m_material.m_kineticFriction = shape->mMaterial->mDynamicFriction; 
		if (cookie->m_material.m_staticFriction <= cookie->m_material.m_kineticFriction) {
			// this is a violation of coulomb friction model, 
			//static friction can never be equal or lower than kinetic friction, simple add a small amount to static friction
			cookie->m_material.m_staticFriction += 0.1f;
		}
	} else {
		cookie->m_material = m_defailtMaterial;
	}

	NewtonCollisionSetUserData(ret, cookie);
	return ret;
}

void NewtonPint::DuplicateCollisionShape (const NewtonWorld* const newtonWorld, NewtonCollision* const newCollision, const NewtonCollision* const sourceCollision)
{
	PinkCollisionShapeCookie * const srcCookie = (PinkCollisionShapeCookie *)NewtonCollisionGetUserData(sourceCollision);
	if (srcCookie) {
		PinkCollisionShapeCookie * const newCookie = new PinkCollisionShapeCookie (*srcCookie);
		NewtonCollisionSetUserData(newCollision, newCookie);
	}
}

void NewtonPint::DestroyCollisionShape (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision)
{
	PinkCollisionShapeCookie * const cookie = (PinkCollisionShapeCookie *)NewtonCollisionGetUserData(collision);
	if (cookie) {
		delete cookie;
	}
}


NewtonCollision* NewtonPint::CreateShape(const PINT_OBJECT_CREATE& desc) const
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
			PinkCollisionShapeCookie * const cookie = new PinkCollisionShapeCookie (NULL, desc.mCollisionGroup);
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
			PinkCollisionShapeCookie * const cookie = new PinkCollisionShapeCookie (NULL, desc.mCollisionGroup);
			NewtonCollisionSetUserData(collision, cookie);

		}
		return collision;
	}

	return NULL;
}

//JULIO
/*void* NewtonPint::BeginCreateObjectHint ()
{
	// here we are beginning to build a collection of static bodies than can all be grouped into a single scene collision mesh 
	NewtonWaitForUpdateToFinish (mWorld);
	return NewtonCreateSceneCollision (mWorld, 0);
}*/

//JULIO
/*void NewtonPint::EndCreateObjectHint (void* creationHintHandle)
{
	NewtonCollision* const sceneCollision = (NewtonCollision*) creationHintHandle;
	NewtonSceneCollisionEndAddRemove (sceneCollision);	

	// add a dummy cookie to keep the collision callback happy
	PinkCollisionShapeCookie * const cookie = new PinkCollisionShapeCookie (NULL, 0);
	NewtonCollisionSetUserData(sceneCollision, cookie);

	dVector veloc(0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix matrix (dGetIdentityMatrix());
	CreateBody (sceneCollision, 0.0f, &matrix[0][0], &veloc[0], &veloc[0], false, false);

	NewtonDestroyCollision(sceneCollision);
}*/


NewtonBody* NewtonPint::CreateBody(const PINT_OBJECT_CREATE& desc, NewtonCollision* const shape, float mass, const dFloat* const worldPose, const dFloat* const veloc, const dFloat* const omega, bool isKinematic, bool ccdMode)
{
	NewtonBody* ret = NULL;
	if(isKinematic)
	{
		ret = NewtonCreateKinematicBody(mWorld, shape, worldPose);
		NewtonBodySetMassProperties(ret, mass, shape);
		NewtonBodySetCollidable(ret, 1);
		const float Zero[] = {0.0f, 0.0f, 0.0f};
		NewtonBodySetVelocity(ret, Zero);
		NewtonBodySetOmega(ret, Zero);
//		NewtonBodyIntegrateVelocity(ret, 1.0f/60.0f);
/*
		KineData Data;
		Data.mKine = ret;
		Data.mPreviousPos.Zero();
		mKinematics.push_back(Data);*/
	}
	else
	{
		ret = NewtonCreateDynamicBody(mWorld, shape, worldPose);
	}

	if(mass != 0.0f)
	{
		NewtonBodySetContinuousCollisionMode(ret, ccdMode ? 1 : 0);
		NewtonBodySetMassProperties(ret, mass, shape);
		NewtonBodySetVelocity(ret, veloc);
		NewtonBodySetOmega(ret, omega);
		NewtonBodySetForceAndTorqueCallback(ret, ApplyForceAndTorqueCallback);

		if(desc.mCOMLocalOffset.IsNonZero())
		{
			dFloat COM[3];
			NewtonBodyGetCentreOfMass(ret, COM);
			COM[0] += desc.mCOMLocalOffset.x;
			COM[1] += desc.mCOMLocalOffset.y;
			COM[2] += desc.mCOMLocalOffset.z;
			NewtonBodySetCentreOfMass(ret, COM);
		}
	}

	// set the testing configuration now.
	NewtonBodySetAutoSleep(ret, gEnableSleeping);

	// attack a cookie to the body for some application specific stuff
	PinkRigidBodyCookie* const cookie = new PinkRigidBodyCookie(isKinematic);
	NewtonBodySetUserData(ret, cookie);
	NewtonBodySetDestructorCallback(ret, OnDestroyBody);

	// set the damping coefficients
	NewtonBodySetLinearDamping(ret, gLinearDamping);
	dVector angularDamp(gAngularDamping, gAngularDamping, gAngularDamping, 0.0f);
	NewtonBodySetAngularDamping(ret, &angularDamp[0]);

	return ret;
}

PintObjectHandle NewtonPint::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	return CreateObject(desc, NULL);
}

PintObjectHandle NewtonPint::CreateObject(const PINT_OBJECT_CREATE& desc, void* creationHintHandle)
{
	if(!desc.mShapes) {
		return NULL;
	}

	ASSERT(mWorld);

	NewtonWaitForUpdateToFinish (mWorld);
	NewtonCollision* const shape = CreateShape(desc);
	if ( shape == NULL )
	{
		return NULL;
	}

	NewtonBody* ret = NULL;
	IceMaths::Matrix4x4 pose = desc.mRotation;
	pose.SetTrans(desc.mPosition);
	pose.SetTrans(desc.mPosition.x, desc.mPosition.y, desc.mPosition.z );

	if (creationHintHandle) {
		// add this collision shape to the scene collision
		_ASSERTE (desc.mMass == 0.0f);
		NewtonCollision* const sceneCollision = (NewtonCollision*) creationHintHandle;

		// bake the object matrix into the shape matrix
		dMatrix matrix (&pose.m[0][0]);
		dMatrix localMatrix;
		NewtonCollisionGetMatrix (shape, &localMatrix[0][0]);
		matrix = localMatrix * matrix;
		NewtonCollisionSetMatrix (shape, &matrix[0][0]);
		NewtonSceneCollisionAddSubCollision (sceneCollision, shape);	
		NewtonDestroyCollision(shape);

	} else {
		//ret = NewtonCreateDynamicBody(mWorld,shape,(const dFloat*)&pose);
		ret = CreateBody(desc, shape, desc.mMass, &pose.m[0][0], &desc.mLinearVelocity.x, &desc.mAngularVelocity.x, desc.mKinematic, gUseCCD);
		NewtonDestroyCollision(shape);
	}
	return ret;
}



//JULIO
/*void* NewtonPint::CreatePhantom(const AABB& box, void* creationHintHandle)
{
	PINT_BOX_CREATE BoxDesc;
	dVector p0 (box.GetMin());
	dVector p1 (box.GetMax());
	dVector size ((p1 - p0).Scale(0.5f));
	dVector origin ((p1 + p0).Scale(0.5f));
	BoxDesc.mExtents = Point (size.m_x, size.m_y, size.m_z);
	BoxDesc.mRenderer	= NULL;

	PINT_OBJECT_CREATE phantonDesc;
	phantonDesc.mShapes = &BoxDesc;
	phantonDesc.mMass = 0;
	phantonDesc.mPosition = Point (origin.m_x, origin.m_y, origin.m_z);
	phantonDesc.mCollisionGroup = 0;
	phantonDesc.mKinematic = true;
	return CreateObject(phantonDesc, creationHintHandle);
}*/

void NewtonPint::OnDestroyBody (const NewtonBody* const body)
{
	PinkRigidBodyCookie* const cookie = (PinkRigidBodyCookie*) NewtonBodyGetUserData (body);
	delete cookie;
}

bool NewtonPint::ReleaseObject(PintObjectHandle handle)
{
	NewtonDestroyBody ((NewtonBody*) handle);
	return true;
}

//JULIO
/*void NewtonPint::SetLinearAndAngularDamp (PintJointHandle object, float linearDamp, float angularDamp)
{
	NewtonBody* body = (NewtonBody*) object;
	NewtonBodySetLinearDamping(body, linearDamp);
	NewtonBodySetAngularDamping(body, &angularDamp);
}*/

PintJointHandle NewtonPint::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	CustomJoint* joint = NULL;

	NewtonBody* const body0 = (NewtonBody*)desc.mObject0;
	NewtonBody* const body1 = (NewtonBody*)desc.mObject1;

	switch (desc.mType) 
	{
		case PINT_JOINT_SPHERICAL:	// Spherical joint a.k.a. point-to-point constraint a.k.a. ball-and-socket
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			//dMatrix matrix;
			//NewtonBodyGetMatrix (body0, &matrix[0][0]);
			dMatrix matrix0;
			dMatrix matrix1(dGetIdentityMatrix());
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}

			dMatrix pinMatrix0 (dGetIdentityMatrix());
			dMatrix pinMatrix1 (pinMatrix0 * matrix0 * matrix1.Inverse());
			pinMatrix0.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);
			pinMatrix1.m_posit = dVector (jc.mLocalPivot1.x, jc.mLocalPivot1.y, jc.mLocalPivot1.z, 1.0f);

			matrix0 = pinMatrix0 * matrix0;
			matrix1 = pinMatrix1 * matrix1;
			joint = new CustomBallAndSocket(matrix0, matrix1, body0, body1);
			break;
		}

		case PINT_JOINT_HINGE:		// Hinge joints a.k.a. revolute joints
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			dMatrix matrix0;
			dMatrix matrix1;
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}
			dMatrix pinMatrix0 (dGrammSchmidt (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 1.0f)));
			dMatrix pinMatrix1 (pinMatrix0 * matrix0 * matrix1.Inverse());
			pinMatrix0.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);
			pinMatrix1.m_posit = dVector (jc.mLocalPivot1.x, jc.mLocalPivot1.y, jc.mLocalPivot1.z, 1.0f);

			matrix0 = pinMatrix0 * matrix0;
			matrix1 = pinMatrix1 * matrix1;
			CustomHinge* const hinge = new CustomHinge (matrix0, matrix1, body0, body1);
			hinge->EnableLimits (true);
			hinge->SetLimis (jc.mMinLimitAngle, jc.mMaxLimitAngle);
			joint = hinge;
			break;
		}

//JULIO
/*		case PINT_JOINT_UNIVERSAL:		
		{
			const PINT_UNIVERSAL_JOINT_CREATE& jc = static_cast<const PINT_UNIVERSAL_JOINT_CREATE&>(desc);

			dMatrix matrix;
			dMatrix pinMatrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);

			pinMatrix.m_front = dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 0.0f);
			pinMatrix.m_up = dVector (jc.mLocalSecundAxis0.x, jc.mLocalSecundAxis0.y, jc.mLocalSecundAxis0.z, 0.0f);
			pinMatrix.m_right = pinMatrix.m_front * pinMatrix.m_up;
			pinMatrix.m_right = pinMatrix.m_right.Scale (1.0f / dSqrt (pinMatrix.m_right % pinMatrix.m_right));
			pinMatrix.m_up = pinMatrix.m_right * pinMatrix.m_front;
			pinMatrix.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);

			matrix = pinMatrix * matrix;
			CustomUniversal* const universal = new CustomUniversal (matrix, body0, body1);

			universal->EnableLimit_0 (true);
			universal->SetLimis_0 (jc.mMinLimitAngle, jc.mMaxLimitAngle);

			universal->EnableLimit_1 (true);
			universal->SetLimis_1 (jc.mMinLimitSecundAngle, jc.mMaxLimitSecundAngle);

			joint = universal;

			break;
		}*/


		case PINT_JOINT_PRISMATIC:	// Prismatic joints, a.k.a. slider constraints
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			dMatrix pinMatrix (dGrammSchmidt (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 1.0f)));
			pinMatrix.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);

			matrix = pinMatrix * matrix;
			CustomSlider* const slider = new CustomSlider (matrix, body0, body1);

			if(jc.mMinLimit<=jc.mMaxLimit)
			{
				slider->SetLimis(jc.mMinLimit, jc.mMaxLimit);
				slider->EnableLimits(true);
			}
			else
				slider->EnableLimits(false);

			joint = slider;
			break;
		}

//JULIO
/*		case PINT_JOINT_CYLINDRICAL:
		{
			const PINT_CYLINDRCAL_JOINT_CREATE jc = static_cast<const PINT_CYLINDRCAL_JOINT_CREATE&>(desc);

			dMatrix matrix;
			NewtonBodyGetMatrix (body0, &matrix[0][0]);
			dMatrix pinMatrix (dGrammSchmidt (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z, 1.0f)));
			pinMatrix.m_posit = dVector (jc.mLocalPivot0.x, jc.mLocalPivot0.y, jc.mLocalPivot0.z, 1.0f);

			matrix = pinMatrix * matrix;
			CustomCorkScrew* const cylinder = new CustomCorkScrew (matrix, body0, body1);

			cylinder->EnableLinearLimits(true);
			cylinder->SetLinearLimis (jc.mMinLimit, jc.mMaxLimit);

			cylinder->EnableAngularLimits(true);
			cylinder->SetAngularLimis (jc.mMinLimitAngle, jc.mMaxLimitAngle);

			joint = cylinder;
			break;
		}*/


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

//JULIO
/*		case PINT_JOINT_GEAR:
		{
			const PINT_GEAR_JOINT_CREATE jc = static_cast<const PINT_GEAR_JOINT_CREATE&>(desc);

			dMatrix matrix0;
			dMatrix matrix1(dGetIdentityMatrix());
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}

			dVector pin0 (matrix0.RotateVector (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z)));
			dVector pin1 (matrix1.RotateVector (dVector (jc.mLocalAxis1.x, jc.mLocalAxis1.y, jc.mLocalAxis1.z)));

			CustomGear* const gear = new CustomGear (jc.mRatio, pin0, pin1, body0, body1);
			joint = gear;
			break;
		}

		case PINT_JOINT_PULLEY:
		{
			const PINT_PULLEY_JOINT_CREATE jc = static_cast<const PINT_PULLEY_JOINT_CREATE&>(desc);

			dMatrix matrix0;
			dMatrix matrix1(dGetIdentityMatrix());
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}

			dVector pin0 (matrix0.RotateVector (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z)));
			dVector pin1 (matrix1.RotateVector (dVector (jc.mLocalAxis1.x, jc.mLocalAxis1.y, jc.mLocalAxis1.z)));

			CustomPulley* const gear = new CustomPulley (jc.mRatio, pin0, pin1, body0, body1);
			joint = gear;
			break;
		}

		case PINT_JOINT_PINION_AND_RACK:
		{
			const PINT_PINION_AND_RACK_JOINT_CREATE jc = static_cast<const PINT_PINION_AND_RACK_JOINT_CREATE&>(desc);

			dMatrix matrix0;
			dMatrix matrix1(dGetIdentityMatrix());
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}

			dVector pin0 (matrix0.RotateVector (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z)));
			dVector pin1 (matrix1.RotateVector (dVector (jc.mLocalAxis1.x, jc.mLocalAxis1.y, jc.mLocalAxis1.z)));

			CustomRackAndPinion* const gear = new CustomRackAndPinion (jc.mRatio, pin1, pin0, body1, body0);
			joint = gear;
			break;
		}

		case PINT_JOINT_CORK_AND_SCREW:
		{
			const PINT_CORK_AND_SCREW_JOINT_CREATE jc = static_cast<const PINT_CORK_AND_SCREW_JOINT_CREATE&>(desc);

			dMatrix matrix0;
			dMatrix matrix1(dGetIdentityMatrix());
			NewtonBodyGetMatrix (body0, &matrix0[0][0]);
			if (body1) {
				NewtonBodyGetMatrix (body1, &matrix1[0][0]);
			} else {
				matrix1 = matrix0;
			}

			dVector pin0 (matrix0.RotateVector (dVector (jc.mLocalAxis0.x, jc.mLocalAxis0.y, jc.mLocalAxis0.z)));
			dVector pin1 (matrix1.RotateVector (dVector (jc.mLocalAxis1.x, jc.mLocalAxis1.y, jc.mLocalAxis1.z)));

			CustomGearAndSlide* const gear = new CustomGearAndSlide (jc.mAngularRatio, jc.mLinearRatio, pin1, pin0, body1, body0);
			joint = gear;
			break;
		}*/
	}

	return joint;
}

void NewtonPint::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i = 0; i < nb_groups; i++) {
		int group0 = groups[i].mGroup0; 
		int group1 = groups[i].mGroup1; 

		unsigned bit1 = 1 << group1;
		mGroupMasks[group0] ^= bit1;

		if (group0 != group1) {
			unsigned bit0 = 1 << group0;
			mGroupMasks[group1] ^= bit0;
		}
	}
}

int	NewtonPint::OnAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{
	// PT: added this to discard kinematic-kinematic pairs. Without this the engine's perf & memory usage explodes in some kinematic scenes.
	{
		const PinkRigidBodyCookie* BodyCookie0 = (const PinkRigidBodyCookie*)NewtonBodyGetUserData(body0);
		const PinkRigidBodyCookie* BodyCookie1 = (const PinkRigidBodyCookie*)NewtonBodyGetUserData(body1);
		if(BodyCookie0->mIsKinematic && BodyCookie1->mIsKinematic)
			return 0;
	}

	//	Newton* const me = NewtonMaterialGetUserData(material);
	NewtonPint* const me = (NewtonPint*) NewtonWorldGetUserData(NewtonBodyGetWorld(body0));
	NewtonCollision* const collsion0 = NewtonBodyGetCollision(body0);
	NewtonCollision* const collsion1 = NewtonBodyGetCollision(body1);

	PinkCollisionShapeCookie* const cookie0 = (PinkCollisionShapeCookie*) NewtonCollisionGetUserData(collsion0);
	PinkCollisionShapeCookie* const cookie1 = (PinkCollisionShapeCookie*) NewtonCollisionGetUserData(collsion1);

	_ASSERTE (cookie0);
	_ASSERTE (cookie1);

	unsigned mask0 = me->mGroupMasks[cookie0->m_collisionGroup] & (1<<cookie1->m_collisionGroup);
	unsigned mask1 = me->mGroupMasks[cookie1->m_collisionGroup] & (1<<cookie0->m_collisionGroup);

	return (mask0 | mask1) ? 1 : 0;
}

void NewtonPint::OnContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		//		dFloat speed;
		//		dVector point;
		//		dVector normal;	
		//		dVector dir0;	
		//		dVector dir1;	
		//		dVector force;
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);
		NewtonCollision* const collision0 = NewtonMaterialGetBodyCollidingShape (material, body0);
		NewtonCollision* const collision1 = NewtonMaterialGetBodyCollidingShape (material, body1);

		PinkCollisionShapeCookie* const coopkie0 =  (PinkCollisionShapeCookie*) NewtonCollisionGetUserData(collision0);
		PinkCollisionShapeCookie* const coopkie1 =  (PinkCollisionShapeCookie*) NewtonCollisionGetUserData(collision1);

		const PinkShapeMaterial* const pinkfrictionMaterial = (coopkie0->m_material.m_frictionCombine > coopkie0->m_material.m_frictionCombine) ? &coopkie0->m_material : &coopkie1->m_material;
		const PinkShapeMaterial* const pinkRestitutionMaterial = (coopkie0->m_material.m_restitutionCombine > coopkie0->m_material.m_restitutionCombine) ? &coopkie0->m_material : &coopkie1->m_material;

		//NewtonMaterialSetContactFrictionState (material, 1, 0);
		//NewtonMaterialSetContactFrictionState (material, 1, 1);
		NewtonMaterialSetContactElasticity (material, pinkRestitutionMaterial->m_restitution);
		NewtonMaterialSetContactFrictionCoef (material, pinkfrictionMaterial->m_staticFriction, pinkfrictionMaterial->m_kineticFriction, 0);
		NewtonMaterialSetContactFrictionCoef (material, pinkfrictionMaterial->m_staticFriction, pinkfrictionMaterial->m_kineticFriction, 1);

		//		NewtonMaterialGetContactForce (material, body, &force.m_x);
		//		NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);
		//		NewtonMaterialGetContactTangentDirections (material, body, &dir0.m_x, &dir1.m_x);
		//		speed = NewtonMaterialGetContactNormalSpeed(material);
	}
}

void NewtonPint::UpdateQueiriesCache (NewtonCollision* lastShapeCache)
{
	if (!m_lastShapeCache) {
		m_lastShapeCache = NewtonCollisionCreateInstance (lastShapeCache);
	} else {
		if (NewtonCollisionDataPointer (lastShapeCache) != NewtonCollisionDataPointer (m_lastShapeCache)) {
			NewtonDestroyCollision(m_lastShapeCache);
			m_lastShapeCache = NewtonCollisionCreateInstance (lastShapeCache);
		}
	}
	NewtonDestroyCollision(lastShapeCache);
}

dFloat NewtonPint::GenericRayCastCallback (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
{
	PintRaycastHit* const data = (PintRaycastHit*) userData;
	if (intersetParam < data->mDistance) {
		data->mDistance = intersetParam;
		data->mObject = (PintObjectHandle*)body;
		data->mNormal = Point (normal[0], normal[1], normal[2]);
		data->mImpact = Point (contact[0], contact[1], contact[2]);
		// phyx does not provide room for passing what sub shape of a collision shape was hit. only what face of a polginal mesh
	}
	return data->mDistance;
}

void NewtonPint::RayCastKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	int index = int (context);
	NewtonPint* const me = (NewtonPint*)NewtonWorldGetUserData(world);
	PintRaycastHit* const dest = &me->mBatchRaycastData.m_destArray[index];
	const PintRaycastData* const raycast = &me->mBatchRaycastData.m_raycasts[index];

	dVector p0 (raycast->mOrigin.x, raycast->mOrigin.y, raycast->mOrigin.z, 1.0f); 
	dVector p1 (p0 + dVector (raycast->mDir.x, raycast->mDir.y, raycast->mDir.z, 1.0f).Scale (raycast->mMaxDist)); 
	dFloat parameter = 1.2f;

	dest->mDistance = 1.2f;
	dest->mImpact = Point (p1.m_x, p1.m_y, p1.m_z);

	NewtonWorldRayCast (me->mWorld, &p0[0], &p1[0], GenericRayCastCallback, dest, NULL, threadIndex);

	int hitCount = 0;
	if (dest->mDistance < 1.0f) {
		dest->mDistance *= raycast->mMaxDist;
		hitCount = 1;
	}
	NewtonAtomicAdd (&me->mBatchRaycastData.m_hitCount, hitCount);
}


udword NewtonPint::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	mBatchRaycastData.m_hitCount = 0;
	mBatchRaycastData.m_destArray = dest;
	mBatchRaycastData.m_raycasts = raycasts;

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		// initialize the result in case we do not hit anything
		PintRaycastHit* const raycast = &dest[i];
		const PintRaycastData* const data = &raycasts[i];
		raycast->mObject = NULL;
		raycast->mDistance = data->mMaxDist;
		raycast->mImpact = data->mOrigin + data->mDir * data->mMaxDist;
		raycast->mNormal = Point (0.0f, 0.0f, 0.0f);
		raycast->mTriangleIndex = 0;
		NewtonDispachThreadJob(mWorld, RayCastKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);
	return mBatchRaycastData.m_hitCount;
}


void NewtonPint::ConvexRayCastKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	int index = int (context);
	NewtonPint* const me = (NewtonPint*)NewtonWorldGetUserData(world);
	PintRaycastHit* const dest = &me->mBatchSweepData.m_destArray[index];

	dMatrix matrix;		
	dVector target;
	dFloat maxDist;
	switch (me->mBatchSweepData.m_shapeType) 
	{
		case BatchSweepData::m_box:
		{
			PintBoxSweepData* const sweep = &((PintBoxSweepData*) me->mBatchSweepData.m_sweeps)[index];
			IceMaths::Matrix4x4 peelMatrix (IceMaths::Matrix4x4 (sweep->mBox.mRot, sweep->mBox.mCenter));
			matrix = dMatrix (&peelMatrix.m[0][0]);
			target = matrix.m_posit  + dVector(sweep->mDir.x, sweep->mDir.y, sweep->mDir.z, 0.0f).Scale (sweep->mMaxDist);
			maxDist = sweep->mMaxDist;
			break;
		}

		case BatchSweepData::m_sphere:
		{
			PintSphereSweepData* const sweep = &((PintSphereSweepData*) me->mBatchSweepData.m_sweeps)[index];
			matrix = dGetIdentityMatrix();
			matrix.m_posit = dVector (sweep->mSphere.mCenter.x, sweep->mSphere.mCenter.y, sweep->mSphere.mCenter.z, 1.0f);
			target = matrix.m_posit + dVector(sweep->mDir.x, sweep->mDir.y, sweep->mDir.z, 0.0f).Scale (sweep->mMaxDist);
			maxDist = sweep->mMaxDist;
			break;
		}

		case BatchSweepData::m_capsule:
		{
			PintCapsuleSweepData* const sweep = &((PintCapsuleSweepData*) me->mBatchSweepData.m_sweeps)[index];
			//matrix.GetTrans (sweep->mSphere.mCenter);
			Point dir (sweep->mCapsule.mP1 - sweep->mCapsule.mP0);
			Point origin ((sweep->mCapsule.mP1 + sweep->mCapsule.mP0) * 0.5f);
			matrix = dRollMatrix (3.141592f * 90.0f / 180.0f) * dGrammSchmidt (dVector (dir.x, dir.y, dir.z, 0.0f));
			matrix.m_posit = dVector (origin.x, origin.y, origin.z, 1.0f);
			target = matrix.m_posit + dVector(sweep->mDir.x, sweep->mDir.y, sweep->mDir.z, 0.0f).Scale (sweep->mMaxDist);
			maxDist = sweep->mMaxDist;
			break;
		}

		case BatchSweepData::m_convex:
		{
			PintConvexSweepData* const sweep = &((PintConvexSweepData*) me->mBatchSweepData.m_sweeps)[index];
			//IceMaths::Matrix4x4 peelMatrix (IceMaths::Matrix4x4 (sweep->mBox.mRot, sweep->mBox.mCenter));
			IceMaths::Matrix4x4 peelMatrix (IceMaths::Matrix4x4 (sweep->mTransform));
			matrix = dMatrix (&peelMatrix.m[0][0]);
			target = matrix.m_posit  + dVector(sweep->mDir.x, sweep->mDir.y, sweep->mDir.z, 0.0f).Scale (sweep->mMaxDist);
			maxDist = sweep->mMaxDist;
			break;
		}
	}

	dest->mDistance = 1.2f;
	dest->mImpact = Point (target.m_x, target.m_y, target.m_z);
	NewtonWorldConvexRayCast (me->mWorld, me->mBatchSweepData.m_castingShape, &matrix[0][0], &target[0], GenericRayCastCallback, dest, NULL, threadIndex);

	int hitCount = 0;
	if (dest->mDistance < 1.0f) {
		dest->mDistance *= maxDist;
		hitCount = 1;
	}
	NewtonAtomicAdd (&me->mBatchSweepData.m_hitCount, hitCount);
}

udword NewtonPint::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	mBatchSweepData.m_shapeType = BatchSweepData::m_box;
	mBatchSweepData.m_hitCount = 0;
	mBatchSweepData.m_destArray = dest;
	mBatchSweepData.m_sweeps = sweeps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchSweepData.m_castingShape = NewtonCreateBox(mWorld, sweeps->mBox.mExtents.x * 2.0f, sweeps->mBox.mExtents.y * 2.0f, sweeps->mBox.mExtents.z * 2.0f, 0, NULL);

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		// intitalize the return data;
		PintRaycastHit* const raycast = &dest[i];
		const PintBoxSweepData* const data = &sweeps[i];
		raycast->mObject = NULL;
		raycast->mDistance = data->mMaxDist;
		raycast->mImpact = data->mBox.mCenter + data->mDir * data->mMaxDist;
		raycast->mNormal = Point (0.0f, 0.0f, 0.0f);
		raycast->mTriangleIndex = 0;
		// send the queries
		NewtonDispachThreadJob(mWorld, ConvexRayCastKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	UpdateQueiriesCache (mBatchSweepData.m_castingShape);
	return mBatchSweepData.m_hitCount;
}

udword NewtonPint::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	mBatchSweepData.m_shapeType = BatchSweepData::m_sphere;
	mBatchSweepData.m_hitCount = 0;
	mBatchSweepData.m_destArray = dest;
	mBatchSweepData.m_sweeps = sweeps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchSweepData.m_castingShape = NewtonCreateSphere(mWorld, sweeps->mSphere.mRadius, 0, NULL);

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintRaycastHit* const raycast = &dest[i];
		const PintSphereSweepData* const data = &sweeps[i];
		raycast->mObject = NULL;
		raycast->mDistance = data->mMaxDist;
		raycast->mImpact = data->mSphere.mCenter + data->mDir * data->mMaxDist;
		raycast->mNormal = Point (0.0f, 0.0f, 0.0f);
		raycast->mTriangleIndex = 0;

		NewtonDispachThreadJob(mWorld, ConvexRayCastKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	UpdateQueiriesCache (mBatchSweepData.m_castingShape);
	return mBatchSweepData.m_hitCount;
}


udword NewtonPint::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	mBatchSweepData.m_shapeType = BatchSweepData::m_capsule;
	mBatchSweepData.m_hitCount = 0;
	mBatchSweepData.m_destArray = dest;
	mBatchSweepData.m_sweeps = sweeps;
	// we should really cache these shapes, but for now I will just create and destroy one

	dMatrix aligment (dRollMatrix (3.141592f * 90.0f / 180.0f));
	dFloat height = sweeps->mCapsule.ComputeLength();
	mBatchSweepData.m_castingShape = NewtonCreateCapsule(mWorld, sweeps->mCapsule.mRadius, height, 0, &aligment[0][0]);

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintRaycastHit* const raycast = &dest[i];
		const PintCapsuleSweepData* const data = &sweeps[i];
		raycast->mObject = NULL;
		raycast->mDistance = data->mMaxDist;
		raycast->mImpact = (data->mCapsule.mP0 + data->mCapsule.mP1) * 0.5f + data->mDir * data->mMaxDist;
		raycast->mNormal = Point (0.0f, 0.0f, 0.0f);
		raycast->mTriangleIndex = 0;
		NewtonDispachThreadJob(mWorld, ConvexRayCastKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	UpdateQueiriesCache (mBatchSweepData.m_castingShape);
	return mBatchSweepData.m_hitCount;
}


udword NewtonPint::CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)
{
	NewtonCollision* const shape = NewtonCreateConvexHull (mWorld, desc.mNbVerts, &desc.mVerts[0].x, sizeof (Point), 0.0f, 0, NULL);
	UpdateQueiriesCache (shape);	
	return 0;
}


udword	NewtonPint::BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)
{
	mBatchSweepData.m_shapeType = BatchSweepData::m_convex;
	mBatchSweepData.m_hitCount = 0;
	mBatchSweepData.m_destArray = dest;
	mBatchSweepData.m_sweeps = sweeps;
	// we should really cache these shapes, but for now I will just create and destroy one

//	dMatrix aligment (dRollMatrix (3.141592f * 90.0f / 180.0f));
//	dFloat height = sweeps->mCapsule.ComputeLength();
//	mBatchSweepData.m_castingShape = NewtonCreateCapsule(mWorld, sweeps->mCapsule.mRadius, height, 0, &aligment[0][0]);
	mBatchSweepData.m_castingShape = m_lastShapeCache;

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintRaycastHit* const raycast = &dest[i];
		const PintConvexSweepData* const data = &sweeps[i];
		raycast->mObject = NULL;
		raycast->mDistance = data->mMaxDist;
		raycast->mImpact = data->mTransform.mPos + data->mDir * data->mMaxDist;
		raycast->mNormal = Point (0.0f, 0.0f, 0.0f);
		raycast->mTriangleIndex = 0;
		NewtonDispachThreadJob(mWorld, ConvexRayCastKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);
	return mBatchSweepData.m_hitCount;
}

int NewtonPint::BodyOverlapAnyCallback (const NewtonBody* const body, void* const userData)
{
	int index = int (userData);
	const NewtonWorld* const world = NewtonBodyGetWorld(body);
	NewtonPint* const me = (NewtonPint*)NewtonWorldGetUserData(world);

	dMatrix matrixA;
	dMatrix matrixB;

	switch (me->mBatchOverlapData.m_shapeType)
	{
		case BatchOverlapData::m_sphere:
		{
			const PintSphereOverlapData* const sphepe = &((PintSphereOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
			matrixA = dGetIdentityMatrix();
			matrixA.m_posit = dVector (sphepe->mSphere.Center().x, sphepe->mSphere.Center().y, sphepe->mSphere.Center().z, 1.0f);
			break;
		}
		case BatchOverlapData::m_box:
		{
			const PintBoxOverlapData* const box = &((PintBoxOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
			IceMaths::Matrix4x4 matrix (box->mBox.mRot, box->mBox.mCenter);
			matrixA = dMatrix(&matrix.m[0][0]);
			break;
		}
	}

	NewtonBodyGetMatrix(body, &matrixB[0][0]);

	NewtonCollision* const collisionA = me->mBatchOverlapData.m_overlapShape;
	NewtonCollision* const collisionB = NewtonBodyGetCollision(body);

	// we can use any of these function, each with higher degree of complexity, the fastest being NewtonCollisionIntersectionTest whi only chek fo collision but does no calculate contacts
	//	dVector contactA;
	//	dVector contactB;
	//	int hit = NewtonCollisionClosestPoint (world, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], &contactA[0], &contactB[0], &normalAB[0], 0);
	//	dVector normal;
	//	dVector contact;
	//	dFloat penetration; 
	//	dLong attributeA;
	//	dLong attributeB;
	//	int hit = NewtonCollisionCollide (world, 1, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], &contact[0], &normal[0], &penetration, &attributeA, &attributeB, 0);
	int hit = NewtonCollisionIntersectionTest (world, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], 0);
	if (hit) {
		PintBooleanHit* const dest = &((PintBooleanHit*)me->mBatchOverlapData.m_destArray)[index];
		dest->mHit = true;
	}
	return !hit;
}

int NewtonPint::BodyOverlapAllCallback (const NewtonBody* const body, void* const userData)
{
	int index = int (userData);
	const NewtonWorld* const world = NewtonBodyGetWorld(body);
	NewtonPint* const me = (NewtonPint*)NewtonWorldGetUserData(world);

	dMatrix matrixA;
	dMatrix matrixB;
	switch (me->mBatchOverlapData.m_shapeType)
	{
	case BatchOverlapData::m_sphere:
		{
			const PintSphereOverlapData* const sphepe = &((PintSphereOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
			matrixA = dGetIdentityMatrix();
			matrixA.m_posit = dVector (sphepe->mSphere.Center().x, sphepe->mSphere.Center().y, sphepe->mSphere.Center().z, 1.0f);
			break;
		}
	case BatchOverlapData::m_box:
		{
			const PintBoxOverlapData* const box = &((PintBoxOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
			IceMaths::Matrix4x4 matrix (box->mBox.mRot, box->mBox.mCenter);
			matrixA = dMatrix(&matrix.m[0][0]);
			break;
		}
	}
	NewtonBodyGetMatrix(body, &matrixB[0][0]);

	NewtonCollision* const collisionA = me->mBatchOverlapData.m_overlapShape;
	NewtonCollision* const collisionB = NewtonBodyGetCollision(body);

	// we can use any of these function, each with higher degree of complexity, the fastest being NewtonCollisionIntersectionTest whi only chek fo collision but does no calculate contacts
	//	dVector contactA;
	//	dVector contactB;
	//	int hit = NewtonCollisionClosestPoint (world, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], &contactA[0], &contactB[0], &normalAB[0], 0);

	//	dVector normal;
	//	dVector contact;
	//	dFloat penetration; 
	//	dLong attributeA;
	//	dLong attributeB;
	//	int hit = NewtonCollisionCollide (world, 1, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], &contact[0], &normal[0], &penetration, &attributeA, &attributeB, 0);

	int hit = NewtonCollisionIntersectionTest (world, collisionA, &matrixA[0][0], collisionB, &matrixB[0][0], 0);
	if (hit) {
		PintOverlapObjectHit* const dest = &((PintOverlapObjectHit*)me->mBatchOverlapData.m_destArray)[index];
		if (int (dest->mNbObjects) < me->mBatchOverlapData.m_maxBodiesPerQueires) {
			int entry = index * me->mBatchOverlapData.m_maxBodiesPerQueires + dest->mNbObjects;
			dest->mNbObjects ++;
			me->mBatchOverlapData.m_hitBodiesBuffer[entry] = (NewtonBody*) body;
			// continue searching
			hit = 0;
		}
	}
	return !hit;
}



void NewtonPint::BodyOverlapKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	int index = int (context);
	NewtonPint* const me = (NewtonPint*)NewtonWorldGetUserData(world);

	switch (me->mBatchOverlapData.m_type)
	{
	case BatchOverlapData::m_any:
		{
			Point p0;
			Point p1;
			switch (me->mBatchOverlapData.m_shapeType)
			{
			case BatchOverlapData::m_sphere:
				{
					const PintSphereOverlapData* const sphepe = &((PintSphereOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
					p0 = sphepe->mSphere.Center() - Point(sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius());
					p1 = sphepe->mSphere.Center() + Point(sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius());
					break;
				}
			case BatchOverlapData::m_box:
				{
					const PintBoxOverlapData* const box = &((PintBoxOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
					p0 = box->mBox.mCenter - box->mBox.mExtents;
					p1 = box->mBox.mCenter + box->mBox.mExtents;
					break;
				}
			}

			NewtonWorldForEachBodyInAABBDo (world, &p0.x, &p1.x, BodyOverlapAnyCallback, context);
			PintBooleanHit* const dest = &((PintBooleanHit*)me->mBatchOverlapData.m_destArray)[index];
			if (dest->mHit) {
				NewtonAtomicAdd (&me->mBatchOverlapData.m_hitCount, 1);
			}
			break;
		}

	case BatchOverlapData::m_all:
		{
			Point p0;
			Point p1;
			switch (me->mBatchOverlapData.m_shapeType)
			{
			case BatchOverlapData::m_sphere:
				{
					const PintSphereOverlapData* const sphepe = &((PintSphereOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
					p0 = sphepe->mSphere.Center() - Point(sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius());
					p1 = sphepe->mSphere.Center() + Point(sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius(), sphepe->mSphere.GetRadius());
					break;
				}

			case BatchOverlapData::m_box:
				{
					const PintBoxOverlapData* const box = &((PintBoxOverlapData*)me->mBatchOverlapData.m_overlaps)[index];
					p0 = box->mBox.mCenter - box->mBox.mExtents;
					p1 = box->mBox.mCenter + box->mBox.mExtents;
					break;
				}
			}


			NewtonWorldForEachBodyInAABBDo (world, &p0.x, &p1.x, BodyOverlapAllCallback, context);
			PintOverlapObjectHit* const dest = &((PintOverlapObjectHit*)me->mBatchOverlapData.m_destArray)[index];
			if (dest->mNbObjects) {
				NewtonAtomicAdd (&me->mBatchOverlapData.m_hitCount, dest->mNbObjects);
			}
			break;
		}
	}
}


udword NewtonPint::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	mBatchOverlapData.m_type = BatchOverlapData::m_any;
	mBatchOverlapData.m_shapeType = BatchOverlapData::m_sphere;
	mBatchOverlapData.m_hitCount = 0;
	mBatchOverlapData.m_destArray = dest;
	mBatchOverlapData.m_overlaps = overlaps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchOverlapData.m_overlapShape = NewtonCreateSphere(mWorld, overlaps->mSphere.mRadius, 0, NULL);

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintBooleanHit* const result = &dest[i];
		result->mHit = false; 
		NewtonDispachThreadJob(mWorld, BodyOverlapKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	UpdateQueiriesCache (mBatchOverlapData.m_overlapShape);
	return mBatchOverlapData.m_hitCount;
}

udword NewtonPint::BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)
{
	mBatchOverlapData.m_type = BatchOverlapData::m_any;
	mBatchOverlapData.m_shapeType = BatchOverlapData::m_box;
	mBatchOverlapData.m_hitCount = 0;
	mBatchOverlapData.m_destArray = dest;
	mBatchOverlapData.m_overlaps = overlaps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchOverlapData.m_overlapShape = NewtonCreateBox(mWorld, overlaps->mBox.mExtents.x * 2.0f, overlaps->mBox.mExtents.y * 2.0f, overlaps->mBox.mExtents.z * 2.0f, 0, NULL);

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintBooleanHit* const result = &dest[i];
		result->mHit = false; 
		NewtonDispachThreadJob(mWorld, BodyOverlapKernel, (void*)i);
	}

	NewtonSyncThreadJobs(mWorld);

	UpdateQueiriesCache (mBatchOverlapData.m_overlapShape);
	return mBatchOverlapData.m_hitCount;
}


udword NewtonPint::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	mBatchOverlapData.m_type = BatchOverlapData::m_all;
	mBatchOverlapData.m_shapeType = BatchOverlapData::m_sphere;
	mBatchOverlapData.m_hitCount = 0;
	mBatchOverlapData.m_destArray = dest;
	mBatchOverlapData.m_overlaps = overlaps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchOverlapData.m_overlapShape = NewtonCreateSphere(mWorld, overlaps->mSphere.mRadius, 0, NULL);

	mBatchOverlapData.m_maxBodiesPerQueires = 512;
	mBatchOverlapData.m_hitBodiesBuffer = new NewtonBody*[nb * mBatchOverlapData.m_maxBodiesPerQueires];

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintOverlapObjectHit* const result = &dest[i];
		result->mNbObjects = 0; 
		NewtonDispachThreadJob(mWorld, BodyOverlapKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	delete[] mBatchOverlapData.m_hitBodiesBuffer;

	UpdateQueiriesCache (mBatchOverlapData.m_overlapShape);
	return mBatchOverlapData.m_hitCount;
}


udword NewtonPint::BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)
{
	mBatchOverlapData.m_type = BatchOverlapData::m_all;
	mBatchOverlapData.m_shapeType = BatchOverlapData::m_box;
	mBatchOverlapData.m_hitCount = 0;
	mBatchOverlapData.m_destArray = dest;
	mBatchOverlapData.m_overlaps = overlaps;
	// we should really cache these shapes, but for now I will just create and destroy one
	mBatchOverlapData.m_overlapShape = NewtonCreateBox(mWorld, overlaps->mBox.mExtents.x * 2.0f, overlaps->mBox.mExtents.y * 2.0f, overlaps->mBox.mExtents.z * 2.0f, 0, NULL);

	mBatchOverlapData.m_maxBodiesPerQueires = 512;
	mBatchOverlapData.m_hitBodiesBuffer = new NewtonBody*[nb * mBatchOverlapData.m_maxBodiesPerQueires];

	NewtonWaitForUpdateToFinish (mWorld);
	for (int i = 0; i < int (nb); i ++) {
		PintOverlapObjectHit* const result = &dest[i];
		result->mNbObjects = 0; 
		NewtonDispachThreadJob(mWorld, BodyOverlapKernel, (void*)i);
	}
	NewtonSyncThreadJobs(mWorld);

	delete[] mBatchOverlapData.m_hitBodiesBuffer;

	UpdateQueiriesCache (mBatchOverlapData.m_overlapShape);
	return mBatchOverlapData.m_hitCount;
}

#ifdef REMOVED
void NewtonPint::UpdateKinematics(dFloat dt)
{
	const udword NbKinematics = mKinematics.size();
	const float OneOverDt = 1.0f / dt;
	for(udword i=0;i<NbKinematics;i++)
	{
		const KineData& Data = mKinematics[i];

		Matrix4x4 M;
		NewtonBodyGetMatrix(Data.mKine, &M.m[0][0]);
		const Point& CurrentPos = M.GetTrans();

		const Point V = (CurrentPos - Data.mPreviousPos) * OneOverDt;
		NewtonBodySetVelocity(Data.mKine, &V.x);

		const float Zero[] = {0.0f, 0.0f, 0.0f};
		NewtonBodySetOmega(Data.mKine, Zero);

//		NewtonBodyIntegrateVelocity(Data.mKine, dt);
	}
	mKinematics.clear();
}
#endif

void NewtonPint::UpdateKinematics(dFloat dt)
{
	const float OneOverDt = 1.0f / dt;

	{
		const udword NbKinematics = mKinematics.size();
		for(udword i=0;i<NbKinematics;i++)
		{
			const KineData& Data = mKinematics[i];

			Matrix4x4 M;
			NewtonBodyGetMatrix(Data.mKine, &M.m[0][0]);

				const Point& CurrentPos = M.GetTrans();

				const Point V = (Data.mKinematicTarget - CurrentPos) * OneOverDt;
				NewtonBodySetVelocity(Data.mKine, &V.x);

				const float Zero[] = {0.0f, 0.0f, 0.0f};
				NewtonBodySetOmega(Data.mKine, Zero);

//				NewtonBodyIntegrateVelocity(Data.mKine, dt);


			M.SetTrans(Data.mKinematicTarget);
			NewtonBodySetMatrix(Data.mKine, &M.m[0][0]);
		}
		mKinematics.clear();
	}

	{
		const udword NbKinematics = mKinematics2.size();
		for(udword i=0;i<NbKinematics;i++)
		{
			const KineData2& Data = mKinematics2[i];

			Matrix4x4 M;
			NewtonBodyGetMatrix(Data.mKine, &M.m[0][0]);

				const Point& CurrentPos = M.GetTrans();

				const Point V = (Data.mKinematicTarget.mPos - CurrentPos) * OneOverDt;
				NewtonBodySetVelocity(Data.mKine, &V.x);

				//### this one is wrong
				const float Zero[] = {0.0f, 0.0f, 0.0f};
				NewtonBodySetOmega(Data.mKine, Zero);

//				NewtonBodyIntegrateVelocity(Data.mKine, dt);

			const Matrix4x4 matrix = Data.mKinematicTarget;
			NewtonBodySetMatrix(Data.mKine, &matrix.m[0][0]);

//			M.SetTrans(Data.mKinematicTarget);
//			NewtonBodySetMatrix(Data.mKine, &M.m[0][0]);
		}
		mKinematics2.clear();
	}
}

#ifdef REMOVED
bool NewtonPint::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	NewtonBody* const body = (NewtonBody*)handle;
	Matrix4x4 matrix;
	NewtonBodyGetMatrix(body, &matrix.m[0][0]);

	if(1)
		{
			const Point PreviousPos = matrix.GetTrans();

/*			const float Zero[] = {0.0f, 0.0f, 0.0f};
			const Point V = (pos - PreviousPos)*60.0f;
			NewtonBodySetVelocity(body, &V.x);
			NewtonBodySetOmega(body, Zero);
		//	NewtonBodyIntegrateVelocity(body, 1.0f/60.0f);*/

			KineData Data;
			Data.mKine = body;
			Data.mPreviousPos = PreviousPos;
			mKinematics.push_back(Data);

		}


	matrix.SetTrans(pos);
	NewtonBodySetMatrix(body, &matrix.m[0][0]);
	return true;
}
#endif

bool NewtonPint::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	NewtonBody* const body = (NewtonBody*)handle;
	KineData Data;
	Data.mKine = body;
	Data.mKinematicTarget = pos;
	mKinematics.push_back(Data);
	return true;
}

bool NewtonPint::SetKinematicPose(PintObjectHandle handle, const PR& pr)
{
	NewtonBody* const body = (NewtonBody*)handle;
//	const Matrix4x4 matrix = pr;
//	NewtonBodySetMatrix(body, &matrix.m[0][0]);
	KineData2 Data;
	Data.mKine = body;
	Data.mKinematicTarget = pr;
	mKinematics2.push_back(Data);

	return true;
}

PR NewtonPint::GetWorldTransform(PintObjectHandle handle)
{
	NewtonBody* const body = (NewtonBody*)handle;
	Matrix4x4 matrix;
	NewtonBodyGetMatrix(body, &matrix.m[0][0]);
	return PR (matrix);
}

void NewtonPint::SetWorldTransform(PintObjectHandle handle, const PR& pose)
{
	NewtonBody* body = (NewtonBody*)handle;

	Matrix4x4 matrix = pose.mRot;
	matrix.SetTrans(pose.mPos);

	NewtonBodySetMatrix(body, &matrix.m[0][0]);
}

/*void NewtonPint::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	NewtonBody* const body = (NewtonBody*)handle;
	if(action_type==PINT_ACTION_FORCE)
	{
		// this is one case were Physx and Newton are different,
		// force can only be applied form with in a force and torque call back.
		// still can apply the force in the form of an impulse bu we need to know the time step
		// for now I will simply assert 
		_ASSERTE (0);

		//		dVector com; 
		//		dMatrix matrix; 
		//		NewtonBody* const body = (NewtonBody*)handle;
		//		NewtonBodyGetMatrix(body, &matrix[0][0]);
		//		NewtonBodyGetCentreOfMass(body, &com[0]);
		//		dVector point (dVector (pos.x, pos.y, pos.z, 0.0f) - matrix.TransformVector (com)); 
		//		dVector force (action.x, action.y, action.z, 0.0f);
		//		dVector torque (point * force);
		//		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		//		PinkCookie* const cookie = (PinkCookie*) NewtonCollisionGetUserData(collision);
		//		cookie->m_externalForce += force;
		//		cookie->m_externalTorque += torque;

	}
	else if(action_type==PINT_ACTION_IMPULSE)
	{
		PinkRigidBodyCookie* const cookie = (PinkRigidBodyCookie*) NewtonBodyGetUserData(body);
		_ASSERTE (cookie);

		CriticalSection (&cookie->m_lock);
		// save this values so that they can be applied at the proper time
		if (((cookie->m_extLinearImpulse % cookie->m_extLinearImpulse) == 0.0f) && ((cookie->m_extAngularImpulse % cookie->m_extAngularImpulse) == 0.0f)) {
			cookie->m_extAngularImpulse = dVector (action.x, action.y, action.z, 0.0f);
			cookie->m_extLinearImpulse = dVector (pos.x, pos.y, pos.z, 0.0f);
		}
	}
}*/

void NewtonPint::AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos)
{
	NewtonBody* const body = (NewtonBody*)handle;
	NewtonBodyApplyImpulseArray(body, 1, sizeof(Point), &world_impulse.x, &world_pos[0]);
}

void NewtonPint::AddLocalTorque(PintObjectHandle handle, const Point& local_torque)
{
	NewtonBody* const body = (NewtonBody*)handle;

	PinkRigidBodyCookie* const cookie = (PinkRigidBodyCookie*) NewtonBodyGetUserData(body);
	cookie->mTorque += dVector(local_torque.x, local_torque.y, local_torque.z);
}

Point NewtonPint::GetAngularVelocity(PintObjectHandle handle)
{
	NewtonBody* const body = (NewtonBody*)handle;
	dFloat Omega[3];
	NewtonBodyGetOmega(body, Omega);
	return Point(Omega[0], Omega[1], Omega[2]);
}

void NewtonPint::SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity)
{
	NewtonBody* const body = (NewtonBody*)handle;
	NewtonBodySetOmega(body, &angular_velocity.x);
}

float NewtonPint::GetMass(PintObjectHandle handle)
{
	NewtonBody* const body = (NewtonBody*)handle;

	dFloat Mass, Ixx, Iyy, Izz;
	NewtonBodyGetMassMatrix(body, &Mass, &Ixx, &Iyy, &Izz);

	return Mass;
}

Point NewtonPint::GetLocalInertia(PintObjectHandle handle)
{
	NewtonBody* const body = (NewtonBody*)handle;

	dFloat Mass, Ixx, Iyy, Izz;
	NewtonBodyGetMassMatrix(body, &Mass, &Ixx, &Iyy, &Izz);

	return Point(Ixx, Iyy, Izz);
}


static NewtonPint* gNewton = null;
static void gNewton_GetOptionsFromGUI();

void Newton_Init(const PINT_WORLD_CREATE& desc)
{
	gNewton_GetOptionsFromGUI();

	ASSERT(!gNewton);
	gNewton = ICE_NEW(NewtonPint);
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

NewtonPint* GetNewton()
{
	return gNewton;
}


/*void NewtonPint::CreateFractureFromConvex (const char* const fileName, const Point* const convexCloud, int convexCloudCount, const Point* const interiorCloud, int interiorCloudCount)
{
	dMatrix matrix (dGetIdentityMatrix());
	NewtonCollision* const hullCollision = NewtonCreateConvexHull(mWorld, convexCloudCount, &convexCloud[0].x, sizeof (Point), 0.0f, 0, &matrix[0][0]);

	NewtonMesh* const hullMesh = NewtonMeshCreateFromCollision(hullCollision);
	NewtonMesh* const voronoi = NewtonMeshCreateVoronoiConvexDecomposition (mWorld, interiorCloudCount, &interiorCloud[0].x, sizeof (Point), 0, &matrix[0][0]);

	FILE* const file = fopen (fileName, "wb");

	// some how I have a bug in this functions
	//NewtonMesh* const collisionMesh = NewtonMeshIntersection (hullMesh, voronoi, &matrix[0][0]);

	NewtonMesh* nextSegment = NULL;
	for (NewtonMesh* segment = NewtonMeshCreateFirstSingleSegment (voronoi); segment; segment = nextSegment) {
		nextSegment = NewtonMeshCreateNextSingleSegment (voronoi, segment);

		NewtonMesh* const fracturePiece = NewtonMeshConvexMeshIntersection (segment, hullMesh);
		if (fracturePiece) {
			int vetexCount = NewtonMeshGetVertexCount (fracturePiece); 
			dAssert (vetexCount);
			int stride = NewtonMeshGetVertexStrideInByte (fracturePiece) / sizeof (dFloat64); 
			const dFloat64* const points = NewtonMeshGetVertexArray (fracturePiece); 
			fprintf (file, "convexmesh\n"); 
			fprintf (file, "vetexCount %d\n", vetexCount);
			for (int i = 0; i < vetexCount; i ++) {
				dFloat x = dFloat(points[i * stride + 0]);
				dFloat y = dFloat(points[i * stride + 1]);
				dFloat z = dFloat(points[i * stride + 2]);
				fprintf (file, "%f %f %f\n", x, y, z);
			}
			NewtonMeshDestroy(fracturePiece);
		}

		NewtonMeshDestroy(segment);
	}
	fprintf (file, "end\n"); 
	fclose (file);

	NewtonMeshDestroy (voronoi);
	NewtonMeshDestroy (hullMesh);
	NewtonDestroyCollision(hullCollision);
}*/

///////////////////////////////////////////////////////////////////////////////

static Container*	gNewtonGUI = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_CollisionMargin = null;

static IceComboBox*	gComboBox_NbThreads = null;
static IceComboBox*	gComboBox_BroardPhaseType = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum NewtonGUIElement
{

	NEWTON_GUI_MAIN,
	//
	NEWTON_GUI_SOLVER_ITER,
	NEWTON_GUI_ENABLE_SLEEPING,
	NEWTON_GUI_USE_CCD,
	NEWTON_GUI_ASYNCRONOUS_UPDATE,
	NEWTON_GUI_LINEAR_DAMPING,
	NEWTON_GUI_ANGULAR_DAMPING,
	NEWTON_GUI_RESOLVE_LARGE_ISLAND_IN_PARALLEL,


	NEWTON_GUI_NB_THREADS,
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
	case NEWTON_GUI_ASYNCRONOUS_UPDATE:
		gAsyncronousUpdate = checked;
		break;

	case NEWTON_GUI_RESOLVE_LARGE_ISLAND_IN_PARALLEL:
		gResolveLargeIsalndInParallel = checked;
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

static udword gNbThreadsToIndex[] = { 0, 0, 2, 3, 4, 4, 4, 4, 8, 8, 8, 8, 8, 8, 8, 8, 16, 16};
static udword gIndexToNbThreads[] = { 0, 2, 3, 4, 8, 16};


static void gNewton_GetOptionsFromGUI()
{
	if(gEditBox_SolverIter)
	{
		sdword tmp;
		bool status = gEditBox_SolverIter->GetTextAsInt(tmp);
		ASSERT(status);
/*		if (tmp < 1) {
			tmp = 1;
			gEditBox_SolverIter->SetText("1");
		}*/

		gSolverIterationCount = udword(tmp);
	}

	if(gEditBox_LinearDamping)
	{
		dFloat tmp;
		bool status = gEditBox_LinearDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gLinearDamping = tmp;
	}


	if(gEditBox_AngularDamping)
	{
		dFloat tmp;
		bool status = gEditBox_AngularDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gAngularDamping = tmp;
	}

	if(gComboBox_NbThreads)
	{
		const udword Index = gComboBox_NbThreads->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToNbThreads)/sizeof(gIndexToNbThreads[0]));
		gNbThreads = gIndexToNbThreads[Index];
	}

	if(gComboBox_BroardPhaseType)
	{
		const udword Index = gComboBox_BroardPhaseType->GetSelectedIndex();
		gBroadPhaseType = Index;
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

		helper.CreateCheckBox(Main, NEWTON_GUI_ASYNCRONOUS_UPDATE, 4, y, CheckBoxWidth, 20, "Asynchronous simulation update", gNewtonGUI, gAsyncronousUpdate, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, NEWTON_GUI_RESOLVE_LARGE_ISLAND_IN_PARALLEL, 4, y, CheckBoxWidth, 20, "Resolve Large Island In parallel", gNewtonGUI, gResolveLargeIsalndInParallel, gCheckBoxCallback);
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


	helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num threads:", gNewtonGUI);
	ComboBoxDesc CBBD;
	CBBD.mID		= NEWTON_GUI_NB_THREADS;
	CBBD.mParent	= Main;
	CBBD.mX			= 4+OffsetX;
	CBBD.mY			= y;
	CBBD.mWidth		= 150;
	CBBD.mHeight	= 20;
	CBBD.mLabel		= "Num threads";
	gComboBox_NbThreads = ICE_NEW(IceComboBox)(CBBD);
	gNewtonGUI->Add(udword(gComboBox_NbThreads));
	gComboBox_NbThreads->Add("Single threaded");
	gComboBox_NbThreads->Add("2 threads");
	gComboBox_NbThreads->Add("3 threads");
	gComboBox_NbThreads->Add("4 threads");
	gComboBox_NbThreads->Add("8 threads");
	gComboBox_NbThreads->Add("16 threads");
	ASSERT(gNbThreads<sizeof(gNbThreadsToIndex)/sizeof(gNbThreadsToIndex[0]));
	gComboBox_NbThreads->Select(gNbThreadsToIndex[gNbThreads]);
	gComboBox_NbThreads->SetVisible(true);
	y += YStep;


	helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Broad phase type:", gNewtonGUI);
	CBBD.mID		= NEWTON_GUI_NB_THREADS;
	CBBD.mParent	= Main;
	CBBD.mX			= 4+OffsetX;
	CBBD.mY			= y;
	CBBD.mWidth		= 150;
	CBBD.mHeight	= 20;
	CBBD.mLabel		= "Broad phase type";
	gComboBox_BroardPhaseType = ICE_NEW(IceComboBox)(CBBD);
	gNewtonGUI->Add(udword(gComboBox_BroardPhaseType));
	gComboBox_BroardPhaseType->Add("Generic");
	gComboBox_BroardPhaseType->Add("Persistent");
	gComboBox_BroardPhaseType->Select(gBroadPhaseType);
	gComboBox_BroardPhaseType->SetVisible(true);
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
