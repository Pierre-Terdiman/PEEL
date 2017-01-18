///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// WARNING: this file is compiled by all PhysX3 plug-ins, so put only the code here that is "the same" for all versions.

#include "stdafx.h"
#include "..\Pint.h"
#include "PINT_Common.h"
#include "PINT_CommonPhysX3.h"

#ifdef PHYSX_SUPPORT_RAYCAST_CCD
#include "PxRaycastCCD.h"
#endif

using namespace PhysX3;

///////////////////////////////////////////////////////////////////////////////

MemoryOutputStream::MemoryOutputStream(PEEL_PhysX3_AllocatorCallback* allocator) :
	mCallback	(allocator),
	mData		(NULL),
	mSize		(0),
	mCapacity	(0)
{
}

MemoryOutputStream::~MemoryOutputStream()
{
	if(mData)
	{
		if(mCallback)	mCallback->deallocate(mData);
		else			delete[] mData;
	}
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 size)
{
	const PxU32 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
//		mCapacity = expectedSize + 4096;
		mCapacity = NextPowerOfTwo(expectedSize);
		PxU8* newData = mCallback ? (PxU8*)mCallback->allocate(mCapacity, null, null, 0) : new PxU8[mCapacity];
		PX_ASSERT(newData!=NULL);

		if(newData)
		{
			memcpy(newData, mData, mSize);
			if(mCallback)	mCallback->deallocate(mData);
			else			delete[] mData;
		}
		mData = newData;
	}
	memcpy(mData+mSize, src, size);
	mSize += size;
	return size;
}

///////////////////////////////////////////////////////////////////////////////

MemoryInputData::MemoryInputData(PxU8* data, PxU32 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU32 MemoryInputData::read(void* dest, PxU32 count)
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

///////////////////////////////////////////////////////////////////////////////

static udword gNbErrors = 0;
void PEEL_PhysX3_ErrorCallback::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
	gNbErrors++;
	printf(message);
}

///////////////////////////////////////////////////////////////////////////////

PEEL_PhysX3_AllocatorCallback::PEEL_PhysX3_AllocatorCallback() :
	mTotalNbAllocs	(0),
	mNbAllocs		(0),
	mCurrentMemory	(0),
	mLog			(false)
{
}

PEEL_PhysX3_AllocatorCallback::~PEEL_PhysX3_AllocatorCallback()
{
	if(mNbAllocs)
		printf("PhysX 3: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
}

static int atomicIncrement(volatile int* val)
{
	return (int)InterlockedIncrement((volatile LONG*)val);
}

static int atomicDecrement(volatile int* val)
{
	return (int)InterlockedDecrement((volatile LONG*)val);
}

static int atomicAdd(volatile int* val, int delta)
{
	LONG newValue, oldValue;
	do
	{
		oldValue = *val;
		newValue = oldValue + delta;
	} while(InterlockedCompareExchange((volatile LONG*)val, newValue, oldValue) != oldValue);

	return newValue;
}

PX_COMPILE_TIME_ASSERT(sizeof(PEEL_PhysX3_AllocatorCallback::Header)<=32);
void* PEEL_PhysX3_AllocatorCallback::allocate(size_t size, const char* typeName, const char* filename, int line)
{
	char* memory = (char*)_aligned_malloc(size+32, 16);
	Header* H = (Header*)memory;
	H->mMagic		= 0x12345678;
	H->mSize		= size;
	H->mType		= typeName;
	H->mFilename	= filename;
	H->mLine		= line;

	atomicIncrement((int*)&mTotalNbAllocs);
//	mTotalNbAllocs++;

	atomicIncrement((int*)&mNbAllocs);
//	mNbAllocs++;

	atomicAdd((int*)&mCurrentMemory, size);
//	mCurrentMemory+=size;

	return memory + 32;
}

void PEEL_PhysX3_AllocatorCallback::deallocate(void* ptr)
{
	if(!ptr)
		return;

	char* bptr = (char*)ptr;
	Header* H = (Header*)(bptr - 32);
	ASSERT(H->mMagic==0x12345678);
	const udword Size = H->mSize;
	_aligned_free(H);

	atomicDecrement((int*)&mNbAllocs);
//	mNbAllocs--;

	atomicAdd((int*)&mCurrentMemory, -(int)Size);
//	mCurrentMemory-=Size;
}

///////////////////////////////////////////////////////////////////////////////

void PhysX3::ComputeCapsuleTransform(PR& dst, const PR& src)
{
	// ### PhysX is weird with capsules
/*	Matrix3x3 Rot;
	Rot.RotZ(HALFPI);
	Quat QQ = src.mRot * Quat(Rot);*/

	// ### precompute
	const PxQuat q = PxShortestRotation(PxVec3(0.0f, 1.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
	Quat QQ = src.mRot * ToQuat(q);

	dst.mPos = src.mPos;
	dst.mRot = QQ;
}

///////////////////////////////////////////////////////////////////////////////

// PxSetGroup does not work with shared shapes, since they are already attached to the
// actor. This version working on shapes directly should be called before attaching
// the shape to actors.
void PhysX3::SetGroup(PxShape& shape, PxU16 collision_group)
{
	// retrieve current group mask
	PxFilterData fd = shape.getSimulationFilterData();
	fd.word0 = collision_group;
	// set new filter data
	shape.setSimulationFilterData(fd);
}

///////////////////////////////////////////////////////////////////////////////

PxRigidBody* PhysX3::GetRigidBody(PintObjectHandle handle)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
#ifdef PHYSX_SUPPORT_SHARED_SHAPES
		RigidActor = Shape->getActor();
#else
		RigidActor = &Shape->getActor();
#endif
	}

	PxRigidBody* RigidBody = null;
#ifdef IS_PHYSX_3_2
	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		if(RigidDynamic->getRigidDynamicFlags() & PxRigidDynamicFlag::eKINEMATIC)
			return null;

		RigidBody = static_cast<PxRigidBody*>(RigidActor);
	}
	else if(RigidActor->getConcreteType()==PxConcreteType::eARTICULATION_LINK)
		RigidBody = static_cast<PxRigidBody*>(RigidActor);

	return RigidBody;
#else
	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
		RigidBody = static_cast<PxRigidBody*>(RigidActor);
	else if(RigidActor->getConcreteType()==PxConcreteType::eARTICULATION_LINK)
		RigidBody = static_cast<PxRigidBody*>(RigidActor);

	if(RigidBody && !(RigidBody->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
		return RigidBody;
#endif
	return null;
}

///////////////////////////////////////////////////////////////////////////////

static void SetMassProperties(const PINT_OBJECT_CREATE& desc, PxRigidBody& rigid_body)
{
	if(desc.mMassForInertia<0.0f)
	{
		bool status = PxRigidBodyExt::setMassAndUpdateInertia(rigid_body, desc.mMass);
		ASSERT(status);
	}
	else
	{
		bool status = PxRigidBodyExt::setMassAndUpdateInertia(rigid_body, desc.mMassForInertia);
		ASSERT(status);
		rigid_body.setMass(desc.mMass);
	}

	if(desc.mCOMLocalOffset.IsNonZero())
	{
		PxTransform Pose = rigid_body.getCMassLocalPose();
		Pose.p += ToPxVec3(desc.mCOMLocalOffset);
		rigid_body.setCMassLocalPose(Pose);
	}
}

///////////////////////////////////////////////////////////////////////////////

static void normalToTangents(const PxVec3& n, PxVec3& t1, PxVec3& t2)
{
	const PxReal m_sqrt1_2 = PxReal(0.7071067811865475244008443621048490);
	if(fabsf(n.z) > m_sqrt1_2)
	{
		const PxReal a = n.y*n.y + n.z*n.z;
		const PxReal k = PxReal(1.0)/PxSqrt(a);
		t1 = PxVec3(0,-n.z*k,n.y*k);
		t2 = PxVec3(a*k,-n.x*t1.z,n.x*t1.y);
	}
	else 
	{
		const PxReal a = n.x*n.x + n.y*n.y;
		const PxReal k = PxReal(1.0)/PxSqrt(a);
		t1 = PxVec3(-n.y*k,n.x*k,0);
		t2 = PxVec3(-n.z*t1.y,n.z*t1.x,a*k);
	}
	t1.normalize();
	t2.normalize();
}

//#define CAPTURE_VECTORS
#ifdef CAPTURE_VECTORS
PxU32 gNbLocalVectors = 0;
PxVec3 gLocalVectors[256];

static void CaptureLocalVectors(const PxJoint& joint, const PxVec3* wsAnchor, const PxVec3* axisIn)
{
	PxRigidActor* actors[2];
	joint.getActors(actors[0], actors[1]);

	PxTransform localPose[2];
	for(PxU32 i=0; i<2; i++)
		localPose[i] = PxTransform(PxIdentity);

	// 1) global anchor
	if(wsAnchor)
	{
		//transform anchorPoint to local space
		for(PxU32 i=0; i<2; i++)
			localPose[i].p = actors[i] ? actors[i]->getGlobalPose().transformInv(*wsAnchor) : *wsAnchor;
	}

gLocalVectors[gNbLocalVectors++] = localPose[0].p;
gLocalVectors[gNbLocalVectors++] = localPose[1].p;

	// 2) global axis
	if(axisIn)
	{
		PxVec3 localAxis[2], localNormal[2];

		//find 2 orthogonal vectors.
		//gotta do this in world space, if we choose them
		//separately in local space they won't match up in worldspace.
		PxVec3 axisw = *axisIn;
		axisw.normalize();

		PxVec3 normalw, binormalw;
		::normalToTangents(axisw, binormalw, normalw);
		//because axis is supposed to be the Z axis of a frame with the other two being X and Y, we need to negate
		//Y to make the frame right handed. Note that the above call makes a right handed frame if we pass X --> Y,Z, so 
		//it need not be changed.

		for(PxU32 i=0; i<2; i++)
		{
			if(actors[i])
			{
				const PxTransform& m = actors[i]->getGlobalPose();
				PxMat33 mM(m.q);
				localAxis[i]   = mM.transformTranspose(axisw);
				localNormal[i] = mM.transformTranspose(normalw);
			}
			else
			{
				localAxis[i] = axisw;
				localNormal[i] = normalw;
			}

//			PxVec3 tmp = localAxis[i].cross(localNormal[i]);
//gLocalVectors[gNbLocalVectors++] = localAxis[i];
gLocalVectors[gNbLocalVectors++] = localNormal[i];
//gLocalVectors[gNbLocalVectors++] = tmp;

			PxMat33 rot(localAxis[i], localNormal[i], localAxis[i].cross(localNormal[i]));
			
			localPose[i].q = PxQuat(rot);
			localPose[i].q.normalize();

PxMat33 rot2;
//rot2.column0 = localAxis[i];
//::normalToTangents(localAxis[i], rot2.column1, rot2.column2);
rot2.column1 = localNormal[i];
::normalToTangents(localNormal[i], rot2.column2, rot2.column0);
const PxQuat q0(rot2);

//			const PxVec3 axisCandidate0 = localPose[i].q.rotate(PxVec3(1.0f, 0.0f, 0.0f));
//			const PxVec3 axisCandidate1 = localPose[i].q.rotateInv(PxVec3(1.0f, 0.0f, 0.0f));
//const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), axisCandidate0);
//const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), axisCandidate1);

//const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), rot.column0);
//const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), rot.column1);
//const PxQuat q2 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), rot.column2);
int stop=1;

		}
	}
}
#endif

static PxQuat ComputeJointQuat(PxRigidActor* actor, const PxVec3& localAxis)
{
	//find 2 orthogonal vectors.
	//gotta do this in world space, if we choose them
	//separately in local space they won't match up in worldspace.
	PxVec3 axisw = actor ? actor->getGlobalPose().rotate(localAxis) : localAxis;
	axisw.normalize();

	PxVec3 normalw, binormalw;
	::normalToTangents(axisw, binormalw, normalw);

	const PxVec3 localNormal = actor ? actor->getGlobalPose().rotateInv(normalw) : normalw;

	const PxMat33 rot(localAxis, localNormal, localAxis.cross(localNormal));
	PxQuat q(rot);
	q.normalize();
	return q;
}

/*static void PxSetJointGlobalFrame(PxJoint& joint, PxRigidActor* actor, const PxVec3& localAnchor, const PxVec3& localAxis, PxU32 i)
{
	PxQuat localRot;
	{
		//find 2 orthogonal vectors.
		//gotta do this in world space, if we choose them
		//separately in local space they won't match up in worldspace.
		PxVec3 axisw = actor ? actor->getGlobalPose().rotate(localAxis) : localAxis;
		axisw.normalize();

		PxVec3 normalw, binormalw;
		::normalToTangents(axisw, binormalw, normalw);

		const PxVec3 localNormal = actor ? actor->getGlobalPose().rotateInv(normalw) : normalw;

		const PxMat33 rot(localAxis, localNormal, localAxis.cross(localNormal));
		localRot = PxQuat(rot);
		localRot.normalize();
	}

	joint.setLocalPose(PxJointActorIndex::Enum(i), PxTransform(localAnchor, localRot));
}*/

static void SetupD6Projection(PxD6Joint* j, bool enable_projection, float projection_linear_tolerance, float projection_angular_tolerance)
{
	if(enable_projection)
	{
		j->setProjectionLinearTolerance(projection_linear_tolerance);
		j->setProjectionAngularTolerance(projection_angular_tolerance);
	}
}

static PintJointHandle CreateJoint(PxPhysics& physics, const PINT_JOINT_CREATE& desc, bool enable_collision_between_jointed, const EditableParams& params)
{
	const bool use_d6_joint = params.mUseD6Joint;
	const bool enable_projection = params.mEnableJointProjection;
	const float projection_linear_tolerance = params.mProjectionLinearTolerance;
	const float projection_angular_tolerance = params.mProjectionAngularTolerance * DEGTORAD;

	PxRigidActor* actor0 = (PxRigidActor*)desc.mObject0;
	PxRigidActor* actor1 = (PxRigidActor*)desc.mObject1;

//	projection_angular_tolerance *= DEGTORAD;

	PxJoint* CreatedJoint = null;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			if(use_d6_joint)
			{
				PxD6Joint* j = PxD6JointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
				ASSERT(j);
				CreatedJoint = j;

				j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
				j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
				j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

				SetupD6Projection(j, enable_projection, projection_linear_tolerance, projection_angular_tolerance);
			}
			else
			{
				PxSphericalJoint* j = PxSphericalJointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
				ASSERT(j);
				CreatedJoint = j;

				if(enable_projection)
				{
					// Angular tolerance not used for spherical joints
					j->setProjectionLinearTolerance(projection_linear_tolerance);
				}
			}
		}
		break;

		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			const PxQuat q0 = ComputeJointQuat(actor0, ToPxVec3(jc.mLocalAxis0));
			const PxQuat q1 = ComputeJointQuat(actor1, ToPxVec3(jc.mLocalAxis1));

			if(use_d6_joint)
			{
//				const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
//				const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis1));

				PxD6Joint* j = PxD6JointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
				ASSERT(j);
				CreatedJoint = j;

/*			if(1)
			{
				PxSetJointGlobalFrame(*j, actor0, ToPxVec3(jc.mLocalPivot0), ToPxVec3(jc.mLocalAxis0), 0);
				PxSetJointGlobalFrame(*j, actor1, ToPxVec3(jc.mLocalPivot1), ToPxVec3(jc.mLocalAxis1), 1);
			}*/

				j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

				if(jc.mMinLimitAngle!=MIN_FLOAT || jc.mMaxLimitAngle!=MAX_FLOAT)
				{
//					j->setTwistLimitEnabled(true);
					//j->setTwistLimit(bc.mTwistLowerLimit, bc.mTwistUpperLimit);
				const float limitContactDistance = 0.05f;
				PxJointAngularLimitPair limit(0.0f, 0.0f, limitContactDistance);
//				limit.restitution	= 0.0f;
				//### wtf
				limit.lower			= -jc.mMaxLimitAngle;
				limit.upper			= -jc.mMinLimitAngle;

				limit.lower			= jc.mMinLimitAngle;
				limit.upper			= jc.mMaxLimitAngle;

					j->setTwistLimit(limit);
				j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
				}

				SetupD6Projection(j, enable_projection, projection_linear_tolerance, projection_angular_tolerance);
			}
			else
			{

			// "The axis of the hinge is specified as the direction of the x-axis in the body's joint frame."
			// ### which one??
/*			const PxQuat q0 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis0));
			const PxQuat q1 = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(jc.mLocalAxis1));*/

			PxRevoluteJoint* j = PxRevoluteJointCreate(physics,	actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0),
																actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
			ASSERT(j);
			CreatedJoint = j;

/*			if(!jc.mGlobalAnchor.IsNotUsed() && !jc.mGlobalAxis.IsNotUsed())
			{
				const PxVec3 GlobalAnchor = ToPxVec3(jc.mGlobalAnchor);
				const PxVec3 GlobalAxis = ToPxVec3(jc.mGlobalAxis);
#ifdef CAPTURE_VECTORS
				CaptureLocalVectors(*j, &GlobalAnchor, &GlobalAxis);
#endif
				PxSetJointGlobalFrame(*j, &GlobalAnchor, &GlobalAxis);
			}*/

/*			if(0)
			{
				PxSetJointGlobalFrame(*j, actor0, ToPxVec3(jc.mLocalPivot0), ToPxVec3(jc.mLocalAxis0), 0);
				PxSetJointGlobalFrame(*j, actor1, ToPxVec3(jc.mLocalPivot1), ToPxVec3(jc.mLocalAxis1), 1);
			}*/


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
//				const float limitContactDistance = 10.0f;
				const float limitContactDistance = 0.05f;
//				const float limitContactDistance = 0.0f;

				PxJointAngularLimitPair limit(0.0f, 0.0f, limitContactDistance);
//				limit.restitution	= 0.0f;
				//### wtf
				limit.lower			= -jc.mMaxLimitAngle;
				limit.upper			= -jc.mMinLimitAngle;
//limit.lower			= -degToRad(45.0f);
//limit.upper			= 0.0f;

				limit.lower			= jc.mMinLimitAngle;
				limit.upper			= jc.mMaxLimitAngle;

				j->setLimit(limit);

//				j->setLimit(PxJointLimitPair(jc.mMinLimitAngle, jc.mMaxLimitAngle, TWOPI));
				j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
			}

			if(enable_projection)
			{
				j->setProjectionLinearTolerance(projection_linear_tolerance);
				j->setProjectionAngularTolerance(projection_angular_tolerance);
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
			}
		}
		break;

		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			if(use_d6_joint)
			{
				PxD6Joint* j = PxD6JointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
				ASSERT(j);
				CreatedJoint = j;

				SetupD6Projection(j, enable_projection, projection_linear_tolerance, projection_angular_tolerance);
			}
			else
			{
				PxFixedJoint* j = PxFixedJointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
				ASSERT(j);
				CreatedJoint = j;

				if(enable_projection)
				{
					j->setProjectionLinearTolerance(projection_linear_tolerance);
					j->setProjectionAngularTolerance(projection_angular_tolerance);
				}
			}
		}
		break;

		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			const PxQuat q0 = ComputeJointQuat(actor0, ToPxVec3(jc.mLocalAxis0));
			const PxQuat q1 = ComputeJointQuat(actor1, ToPxVec3(jc.mLocalAxis1));

			if(0 && use_d6_joint)
			{
				PxD6Joint* j = PxD6JointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
				ASSERT(j);
				CreatedJoint = j;

				j->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);


				SetupD6Projection(j, enable_projection, projection_linear_tolerance, projection_angular_tolerance);
			}
			else
			{
				PxPrismaticJoint* j = PxPrismaticJointCreate(physics,	actor0, PxTransform(ToPxVec3(jc.mLocalPivot0), q0),
																		actor1, PxTransform(ToPxVec3(jc.mLocalPivot1), q1));
				ASSERT(j);
				CreatedJoint = j;

				if(jc.mMinLimit<=jc.mMaxLimit)
				{
#ifdef IS_PHYSX_3_2
					PxJointLimitPair Limits(jc.mMinLimit, jc.mMaxLimit, 100.0f);
					Limits.spring = jc.mSpringStiffness;
					Limits.damping = jc.mSpringDamping;
#else
					const PxJointLinearLimitPair Limits(jc.mMinLimit, jc.mMaxLimit, PxSpring(jc.mSpringStiffness, jc.mSpringDamping));
#endif
	//				const PxJointLinearLimitPair Limits(PxTolerancesScale(), jc.mMinLimit, jc.mMaxLimit);
	//				Limits.contactDistance	= 10.0f;
					j->setLimit(Limits);
					j->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
				}
				else
					j->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, false);

				if(enable_projection)
				{
					j->setProjectionLinearTolerance(projection_linear_tolerance);
					j->setProjectionAngularTolerance(projection_angular_tolerance);
				}
			}
		}
		break;

		case PINT_JOINT_DISTANCE:
		{
			const PINT_DISTANCE_JOINT_CREATE& jc = static_cast<const PINT_DISTANCE_JOINT_CREATE&>(desc);

			PxDistanceJoint* j = PxDistanceJointCreate(physics, actor0, PxTransform(ToPxVec3(jc.mLocalPivot0)), actor1, PxTransform(ToPxVec3(jc.mLocalPivot1)));
			ASSERT(j);
			CreatedJoint = j;
			if(jc.mMaxDistance>=0.0f)
			{
				j->setMaxDistance(jc.mMaxDistance);
				j->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
			}
			else j->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, false);

			if(jc.mMinDistance>=0.0f)
			{
				j->setMinDistance(jc.mMinDistance);
				j->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
			}
			else j->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, false);

/*			if(enable_projection)
			{
				j->setProjectionLinearTolerance(projection_linear_tolerance);
				j->setProjectionAngularTolerance(projection_angular_tolerance);
			}*/
//			j->setTolerance(projection_linear_tolerance);
		}
		break;
	}

	if(CreatedJoint)
	{
		if(enable_collision_between_jointed)
			CreatedJoint->setConstraintFlags(PxConstraintFlag::eCOLLISION_ENABLED);

		CreatedJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
		CreatedJoint->setConstraintFlag(PxConstraintFlag::ePROJECTION, enable_projection);
#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
		const bool disable_preprocessing = params.mDisablePreprocessing;
		CreatedJoint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, disable_preprocessing);
#endif
#ifndef IS_PHYSX_3_2
		const float inverse_inertia_scale = params.mInverseInertiaScale;
		const float inverse_mass_scale = params.mInverseMassScale;

	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
		const bool enable_32_compatibility = params.mEnableJoint32Compatibility;
		CreatedJoint->setConstraintFlag(PxConstraintFlag::eDEPRECATED_32_COMPATIBILITY, enable_32_compatibility);
	#endif
		if(inverse_inertia_scale>=0.0f)
		{
			CreatedJoint->setInvInertiaScale0(inverse_inertia_scale);
			CreatedJoint->setInvInertiaScale1(inverse_inertia_scale);
		}

		if(inverse_mass_scale>=0.0f)
		{
			CreatedJoint->setInvMassScale0(inverse_mass_scale);
			CreatedJoint->setInvMassScale1(inverse_mass_scale);
		}
#endif
		// - Stabilization can create artefacts on jointed objects so we just disable it
		// - Ropes and thin cables can go through each other easily if we limit their depenetration velocity. So we just disable
		// this feature for jointed & articulated objects.
		if(actor0 && actor0->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* RD = static_cast<PxRigidDynamic*>(actor0);
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
			RD->setStabilizationThreshold(0.0f);
			RD->setMaxDepenetrationVelocity(MAX_FLOAT);
#endif
		}
		if(actor1 && actor1->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* RD = static_cast<PxRigidDynamic*>(actor1);
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
			RD->setStabilizationThreshold(0.0f);
			RD->setMaxDepenetrationVelocity(MAX_FLOAT);
#endif
		}
	}
	return CreatedJoint;
}

///////////////////////////////////////////////////////////////////////////////

SharedPhysX::SharedPhysX(const EditableParams& params) :
	mFoundation		(null),
	mPhysics		(null),
	mScene			(null),
	mCooking		(null),
	mDefaultMaterial(null),
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	mScratchPad		(null),
	mScratchPadSize	(0),
#endif
	mParams			(params)
{
}

SharedPhysX::~SharedPhysX()
{
	ASSERT(!mFoundation);
	ASSERT(!mCooking);
	ASSERT(!mDefaultMaterial);
	ASSERT(!mPhysics);
	ASSERT(!mScene);
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	ASSERT(!mScratchPad);
#endif
}

void SharedPhysX::SetGravity(const Point& gravity)
{
	ASSERT(mScene);
	mScene->setGravity(ToPxVec3(gravity));
}

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
static const udword gIndexToScratchSize[] = {
	0,				// Disabled
	16*2*1024,		// 32 Kb
	16*8*1024,		// 128 Kb
	16*16*1024,		// 256 Kb
	16*32*1024,		// 512 Kb
	16*64*1024,		// 1024 Kb
	16*128*1024,	// 2048 Kb
};
#endif

#ifdef PHYSX_SUPPORT_RAYCAST_CCD
static RaycastCCDManager* gRaycastCCDManager = null;
#endif

void SharedPhysX::InitCommon()
{
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	if(mParams.mUseRaycastCCD)
		gRaycastCCDManager = new RaycastCCDManager(mScene);
#endif

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	udword Size = mParams.mScratchSize;
	if(Size)
	{
		Size = gIndexToScratchSize[Size];
		mScratchPadSize = Size;
		mScratchPad = _aligned_malloc(Size, 16);
	}
#endif
}

void SharedPhysX::CloseCommon()
{
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	DELETESINGLE(gRaycastCCDManager);
#endif

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	if(mScratchPad)
	{
		_aligned_free(mScratchPad);
		mScratchPad = null;
		mScratchPadSize = 0;
	}
#endif
}

#ifndef IS_PHYSX_3_2
void SharedPhysX::CreateCooking(const PxTolerancesScale& scale, PxMeshPreprocessingFlags mesh_preprocess_params)
{
	ASSERT(!mCooking);
	PxCookingParams Params(scale);
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
	#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE2
	Params.midphaseDesc.setToDefault(mParams.mMidPhaseType);
	#else
	Params.midphaseStructure = mParams.mMidPhaseType;
	#endif
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
	Params.meshCookingHint = mParams.mMeshCookingHint;
#endif
//	Params.buildTriangleAdjacencies = false;

#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
	Params.meshPreprocessParams = mParams.mPrecomputeActiveEdges ? PxMeshPreprocessingFlags(0) : PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
#else
	Params.meshPreprocessParams = PxMeshPreprocessingFlags(0);
#endif

	if(mParams.mPCM)
	{
		Params.meshWeldTolerance = 0.001f;
//		Params.meshPreprocessParams |= PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES | PxMeshPreprocessingFlag::eREMOVE_UNREFERENCED_VERTICES | PxMeshPreprocessingFlag::eREMOVE_DUPLICATED_TRIANGLES);
//		Params.meshPreprocessParams |= PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
		Params.meshPreprocessParams |= mesh_preprocess_params;
	}

#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
	Params.gaussMapLimit = mParams.mGaussMapLimit;
#endif

#ifdef PHYSX_SUPPORT_GPU
	if(mParams.mUseGPU)
		Params.BUILD_GPU_DATA = true;
#endif

#ifdef USE_LOAD_LIBRARY
	mCooking = (func2)(PX_PHYSICS_VERSION, *mFoundation, Params);
#else
	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, Params);
#endif
	ASSERT(mCooking);
}
#endif

void SharedPhysX::SetupArticulationLink(PxArticulationLink& link, const PINT_OBJECT_CREATE& desc)
{
	// - solver iteration counts are set on the articulation itself
	// - sleep threshold is set on the articulation itself
	// - articulations don't support linear/angular damping
	// - articulations don't support velocity limits

	link.setLinearVelocity(ToPxVec3(desc.mLinearVelocity));
	link.setAngularVelocity(ToPxVec3(desc.mAngularVelocity));
#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
	// Ropes and thin cables can go through each other easily if we limit their depenetration velocity. So we just disable
	// this feature for jointed & articulated objects.
//	link.setMaxDepenetrationVelocity(mParams.mMaxDepenVelocity);
	link.setMaxDepenetrationVelocity(MAX_FLOAT);
//	PxReal mdp = link.getMaxDepenetrationVelocity();
//	printf("MDP: %f\n", mdp);
#endif

	::SetMassProperties(desc, link);

//rigidDynamic.setMass(desc.mMass);
//rigidDynamic.setMassSpaceInertiaTensor(PxVec3(1.0f, 1.0f, 1.0f));
//	printf("Mass: %f\n", rigidDynamic.getMass());
//	const PxVec3 msit = rigidDynamic.getMassSpaceInertiaTensor();
//	printf("MSIT: %f %f %f\n", msit.x, msit.y, msit.z);

#ifdef IS_PHYSX_3_2
//	if(!mParams.mEnableSleeping)
//		link.wakeUp(9999999999.0f);
//	link.setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, desc.mKinematic);
#else
	link.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, mParams.mUseCCD);
	link.setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, desc.mKinematic);
#endif
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
	link.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, mParams.mUseAngularCCD);
#endif
}

void SharedPhysX::SetupDynamic(PxRigidDynamic& rigidDynamic, const PINT_OBJECT_CREATE& desc)
{
	rigidDynamic.setLinearDamping(mParams.mLinearDamping);
	rigidDynamic.setAngularDamping(mParams.mAngularDamping);
	rigidDynamic.setLinearVelocity(ToPxVec3(desc.mLinearVelocity));
	rigidDynamic.setAngularVelocity(ToPxVec3(desc.mAngularVelocity));
	rigidDynamic.setMaxAngularVelocity(mParams.mMaxAngularVelocity);
#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
	rigidDynamic.setMaxDepenetrationVelocity(mParams.mMaxDepenVelocity);
#endif
//	printf("%f\n", rigidDynamic.getSleepThreshold());
	rigidDynamic.setSleepThreshold(mParams.mSleepThreshold);
	::SetMassProperties(desc, rigidDynamic);

//rigidDynamic.setMass(desc.mMass);
//rigidDynamic.setMassSpaceInertiaTensor(PxVec3(1.0f, 1.0f, 1.0f));
//	printf("Mass: %f\n", rigidDynamic.getMass());
//	const PxVec3 msit = rigidDynamic.getMassSpaceInertiaTensor();
//	printf("MSIT: %f %f %f\n", msit.x, msit.y, msit.z);

	rigidDynamic.setSolverIterationCounts(mParams.mSolverIterationCountPos, mParams.mSolverIterationCountVel);

#ifdef IS_PHYSX_3_2
	if(!mParams.mEnableSleeping)
		rigidDynamic.wakeUp(9999999999.0f);
	rigidDynamic.setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, desc.mKinematic);
#else
	rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, mParams.mUseCCD);
	rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, desc.mKinematic);
#endif
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
	rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, mParams.mUseAngularCCD);
#endif

#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	if(gRaycastCCDManager)
	{
		PxShape* S;
		rigidDynamic.getShapes(&S, 1);
		gRaycastCCDManager->registerRaycastCCDObject(&rigidDynamic, S);
	}
#endif
}

//#define CAPTURE_VECTORS
#ifdef CAPTURE_VECTORS
extern PxU32 gNbLocalVectors;
extern PxVec3 gLocalVectors[256];
#endif

void SharedPhysX::UpdateCommon(float dt)
{
#ifdef CAPTURE_VECTORS
	SaveAsSource("D://tmp//LocalVectors.cpp", "LocalVectors", gLocalVectors, gNbLocalVectors*sizeof(PxVec3), gNbLocalVectors*sizeof(PxVec3), PACK_NONE);
#endif

	if(mScene)
	{
		if(1)
		{
			const udword NbSubsteps = mParams.mNbSubsteps;
			const float sdt = dt/float(NbSubsteps);
			for(udword i=0;i<NbSubsteps;i++)
			{
				if(NbSubsteps>1)
				{
					const udword Size = mLocalTorques.size();
					for(udword j=0;j<Size;j++)
					{
						const LocalTorque& Current = mLocalTorques[j];

						PxRigidBody* RigidBody = GetRigidBody(Current.mHandle);
						if(RigidBody)
						{
							const PxVec3 GlobalTorque = RigidBody->getGlobalPose().rotate(ToPxVec3(Current.mLocalTorque));
					//		RigidBody->addTorque(GlobalTorque, PxForceMode::eFORCE, true);
							RigidBody->addTorque(GlobalTorque, PxForceMode::eACCELERATION, true);
						}
					}
				}
				mScene->simulate(sdt, null, GetScratchPad(), GetScratchPadSize());
				mScene->fetchResults(true);
			}
			mLocalTorques.clear();
		}

/*		mScene->setFlag(PxSceneFlag::eENABLE_MANUAL_QUERY_UPDATE, gSQManualFlushUpdates);
		if(gSQManualFlushUpdates)
			mScene->flushQueryUpdates();*/

/*		if(0)
		{
			PxActor* Actors[2048];
			udword Nb = mScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, Actors, 2048);
			for(udword i=0;i<Nb;i++)
			{
				PxVec3 LinVel = ((PxRigidDynamic*)Actors[i])->getLinearVelocity();
				//LinVel.x = 0.0f;
				LinVel.y = 0.0f;
				//LinVel.z = 0.0f;
				((PxRigidDynamic*)Actors[i])->setLinearVelocity(LinVel);
				//printf("%f\n", LinVel.y);
			}
		}*/

/*		if(0)
		{
			PxActor* Actors[2048];
			udword Nb = mScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, Actors, 2048);
			for(udword i=0;i<Nb;i++)
			{
				PxVec3 AngVel = ((PxRigidDynamic*)Actors[i])->getAngularVelocity();
				AngVel.x = 0.0f;
				AngVel.y = 0.0f;
				AngVel.z = 0.0f;
				((PxRigidDynamic*)Actors[i])->setAngularVelocity(AngVel);
				//printf("%f\n", LinVel.y);
			}
		}*/

		if(mParams.mFlushSimulation)
#ifdef IS_PHYSX_3_2
			mScene->flush();
#else
			mScene->flushSimulation();
#endif
	}

#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	if(gRaycastCCDManager)
		gRaycastCCDManager->doRaycastCCD(mParams.mUseRaycastCCD_DynaDyna);
#endif
}

void SharedPhysX::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i=0;i<nb_groups;i++)
		PxSetGroupCollisionFlag(groups[i].mGroup0, groups[i].mGroup1, false);
}

PintObjectHandle SharedPhysX::CreateObject(const PINT_OBJECT_CREATE& desc)
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

	// Removed since doesn't work with shared shapes
//	PxSetGroup(*actor, desc.mCollisionGroup);

	if(desc.mAddToWorld)
	{
		mScene->addActor(*actor);

		if(rigidDynamic && !desc.mKinematic)
			SetupSleeping(rigidDynamic, mParams.mEnableSleeping);
	}
	return CreateHandle(actor);
}

bool SharedPhysX::ReleaseObject(PintObjectHandle handle)
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

static	const	bool	gEnableCollisionBetweenJointed	= false;
PintJointHandle SharedPhysX::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mPhysics);
	return ::CreateJoint(*mPhysics, desc, gEnableCollisionBetweenJointed, mParams);
}

PR SharedPhysX::GetWorldTransform(PintObjectHandle handle)
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
#ifdef PHYSX_SUPPORT_SHARED_SHAPES
		ASSERT(Shape->getActor());
		Pose = PxShapeExt::getGlobalPose(*Shape, *Shape->getActor());
#else
		Pose = PxShapeExt::getGlobalPose(*Shape);
#endif
	}

	return PR(ToPoint(Pose.p), ToQuat(Pose.q));
}

void SharedPhysX::SetWorldTransform(PintObjectHandle handle, const PR& pose)
{
	const PxTransform Pose(ToPxVec3(pose.mPos), ToPxQuat(pose.mRot));

	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(RigidActor)
	{
		RigidActor->setGlobalPose(Pose);
//		mScene->resetFiltering(*RigidActor);
	}
	else
	{
/*		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
#ifdef PHYSX_SUPPORT_SHARED_SHAPES
		ASSERT(Shape->getActor());
		Pose = PxShapeExt::getGlobalPose(*Shape, *Shape->getActor());
#else
		Pose = PxShapeExt::getGlobalPose(*Shape);
#endif*/
		ASSERT(0);
	}
}

/*void SharedPhysX::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
#ifndef PHYSX3_DISABLE_SHARED_APPLY_ACTION
	PxRigidBody* RigidBody = GetRigidBody(handle);
	if(RigidBody)
	{
		//### local space/world space confusion in this
		PxForceMode::Enum mode;
		if(action_type==PINT_ACTION_FORCE)
			mode = PxForceMode::eFORCE;
		else if(action_type==PINT_ACTION_IMPULSE)
			mode = PxForceMode::eIMPULSE;
		else ASSERT(0);

//		printf("Picking force magnitude: %f\n", action.Magnitude());
		PxRigidBodyExt::addForceAtPos(*RigidBody, ToPxVec3(action), ToPxVec3(pos), mode);
	}
#endif
}*/

void SharedPhysX::AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos)
{
#ifndef PHYSX3_DISABLE_SHARED_APPLY_ACTION
	PxRigidBody* RigidBody = GetRigidBody(handle);
	if(RigidBody)
	{
		PxRigidBodyExt::addForceAtPos(*RigidBody, ToPxVec3(world_impulse), ToPxVec3(world_pos), PxForceMode::eIMPULSE);
	}
#endif
}

void SharedPhysX::AddLocalTorque(PintObjectHandle handle, const Point& local_torque)
{
#ifndef PHYSX3_DISABLE_SHARED_APPLY_ACTION
	if(mParams.mNbSubsteps>1)
	{
		mLocalTorques.push_back(LocalTorque(handle, local_torque));
	}
	else
	{
		PxRigidBody* RigidBody = GetRigidBody(handle);
		if(RigidBody)
		{
			const PxVec3 GlobalTorque = RigidBody->getGlobalPose().rotate(ToPxVec3(local_torque));
	//		RigidBody->addTorque(GlobalTorque, PxForceMode::eFORCE, true);
			RigidBody->addTorque(GlobalTorque, PxForceMode::eACCELERATION, true);
		}
	}
#endif
}

Point SharedPhysX::GetAngularVelocity(PintObjectHandle handle)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(RigidBody)
	{
		const PxTransform Pose = RigidBody->getGlobalPose();

		const PxVec3 LocalAngularVelocity = Pose.rotateInv(RigidBody->getAngularVelocity());

		return ToPoint(LocalAngularVelocity);
	}
	return Point(0.0f, 0.0f, 0.0f);
}

void SharedPhysX::SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(RigidBody)
	{
		const PxTransform Pose = RigidBody->getGlobalPose();

		const PxVec3 GlobalAngularVelocity = Pose.rotate(ToPxVec3(angular_velocity));

		RigidBody->setAngularVelocity(GlobalAngularVelocity);
	}
}

float SharedPhysX::GetMass(PintObjectHandle handle)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(!RigidBody)
		return 0.0f;
	return RigidBody->getMass();
}

Point SharedPhysX::GetLocalInertia(PintObjectHandle handle)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(!RigidBody)
		return Point(0.0f, 0.0f, 0.0f);
	return ToPoint(RigidBody->getMassSpaceInertiaTensor());
}

udword SharedPhysX::GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)
{
//	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	PxRigidActor* RigidActor = (PxRigidActor*)handle;
	return RigidActor->getShapes((PxShape**)shapes, 3);
}

void SharedPhysX::SetLocalRot(PintObjectHandle handle, const Quat& q)
{
//	PxShape* Shape = GetShapeFromHandle(handle);
	PxShape* Shape = (PxShape*)handle;
	PxTransform lp = Shape->getLocalPose();
	lp.q = ToPxQuat(q);
	Shape->setLocalPose(lp);
}

bool SharedPhysX::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	PxRigidActor* Actor = GetActorFromHandle(handle);
	if(!Actor)
		return false;

	ASSERT(Actor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC);

	PxRigidDynamic* Kine = static_cast<PxRigidDynamic*>(Actor);
	Kine->setKinematicTarget(PxTransform(ToPxVec3(pos)));
	return true;
}

bool SharedPhysX::SetKinematicPose(PintObjectHandle handle, const PR& pr)
{
	PxRigidActor* Actor = GetActorFromHandle(handle);
	if(!Actor)
		return false;

	ASSERT(Actor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC);

	PxRigidDynamic* Kine = static_cast<PxRigidDynamic*>(Actor);
	Kine->setKinematicTarget(PxTransform(ToPxVec3(pr.mPos), ToPxQuat(pr.mRot)));
	return true;
}

udword SharedPhysX::CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)
{
	// TODO: is this ok??
	PxConvexMesh* ConvexMesh = CreateConvexMesh(desc.mVerts, desc.mNbVerts, PxConvexFlag::eCOMPUTE_CONVEX, desc.mRenderer);
	const udword CurrentSize = mConvexObjects.size();
	mConvexObjects.push_back(ConvexMesh);
	return CurrentSize;
}

PxMaterial* SharedPhysX::CreateMaterial(const PINT_MATERIAL_CREATE& desc)
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
	if(mParams.mDisableStrongFriction)
		M->setFlags(PxMaterialFlag::eDISABLE_STRONG_FRICTION);
//	M->setFrictionCombineMode(PxCombineMode::eMIN);
//	M->setRestitutionCombineMode(PxCombineMode::eMIN);
	PxCombineMode::Enum defMode = M->getFrictionCombineMode();
	mMaterials.push_back(M);
	return M;
}

PxConvexMesh* SharedPhysX::CreateConvexMesh(const Point* verts, udword vertCount, PxConvexFlags flags, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mPhysics);

	if(mParams.mShareMeshData && renderer)
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

	PxConvexMesh* NewConvex;
#ifdef PHYSX_SUPPORT_INSERTION_CALLBACK
	if(1)
	{
		NewConvex = mCooking->createConvexMesh(ConvexDesc, mPhysics->getPhysicsInsertionCallback());
	}
	else
#endif
	{
		MemoryOutputStream buf;
		if(!mCooking->cookConvexMesh(ConvexDesc, buf))
			return null;

		MemoryInputData input(buf.getData(), buf.getSize());
		NewConvex = mPhysics->createConvexMesh(input);
	//	printf("3.4 convex: %d vertices\n", NewConvex->getNbVertices());
	}

	if(renderer)
		mConvexes.push_back(ConvexRender(NewConvex, renderer));

	return NewConvex;
}

PintObjectHandle SharedPhysX::CreateAggregate(udword max_size, bool enable_self_collision)
{
	ASSERT(mPhysics);
	// TODO: where are these released?
	PxAggregate* Aggregate = mPhysics->createAggregate(max_size, enable_self_collision);
	return Aggregate;
}

bool SharedPhysX::AddToAggregate(PintObjectHandle object, PintObjectHandle aggregate)
{
	PxRigidActor* Actor = GetActorFromHandle(object);
	if(!Actor)
		return false;

	PxAggregate* Aggregate = (PxAggregate*)aggregate;
	return Aggregate->addActor(*Actor);
}

bool SharedPhysX::AddAggregateToScene(PintObjectHandle aggregate)
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
#ifdef IS_PHYSX_3_2
			if(!(RigidDynamic->getRigidDynamicFlags() & PxRigidDynamicFlag::eKINEMATIC))
#else
			if(!(RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
#endif
				SetupSleeping(RigidDynamic, mParams.mEnableSleeping);
		}
	}
	return true;
}

extern PEEL_PhysX3_AllocatorCallback* gDefaultAllocator;

PxTriangleMesh* SharedPhysX::CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer)
{
	ASSERT(mCooking);
	ASSERT(mPhysics);

	if(mParams.mShareMeshData && renderer)
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

//	gDefaultAllocator->mLog = true;
//	printf("gDefaultAllocator->mCurrentMemory: %d\n", gDefaultAllocator->mCurrentMemory);
//	printf("gDefaultAllocator->mNbAllocs: %d\n", gDefaultAllocator->mNbAllocs);

	PxTriangleMesh* NewMesh;
#ifdef PHYSX_SUPPORT_INSERTION_CALLBACK
	if(1)
	{
//		udword NbAllocs = gDefaultAllocator->mTotalNbAllocs;
		NewMesh = mCooking->createTriangleMesh(MeshDesc, mPhysics->getPhysicsInsertionCallback());
//		NbAllocs = gDefaultAllocator->mTotalNbAllocs - NbAllocs;
//		printf("NbAllocs: %d\n", NbAllocs);
	}
	else
#endif
	{
//		printf("PhysX 3.3: cooking mesh: %d verts, %d faces\n", surface.mNbVerts, surface.mNbFaces);

//		MemoryOutputStream buf(gDefaultAllocator);
		MemoryOutputStream buf;
		if(!mCooking->cookTriangleMesh(MeshDesc, buf))
			return null;
//	printf("gDefaultAllocator->mCurrentMemory: %d\n", gDefaultAllocator->mCurrentMemory);
//	printf("gDefaultAllocator->mNbAllocs: %d\n", gDefaultAllocator->mNbAllocs);
//	gDefaultAllocator->mLog = false;

//		printf("PhysX 3.3: creating mesh... ");

		MemoryInputData input(buf.getData(), buf.getSize());
//udword MemBefore = gDefaultAllocator->mCurrentMemory;
		NewMesh = mPhysics->createTriangleMesh(input);
//udword MemAfter = gDefaultAllocator->mCurrentMemory;
//printf("PhysX 3.3 mesh memory: %d Kb\n", (MemAfter - MemBefore)/1024);

//	printf("Done.\n");
	}

//	printf("gDefaultAllocator->mCurrentMemory: %d\n", gDefaultAllocator->mCurrentMemory);
//	printf("gDefaultAllocator->mNbAllocs: %d\n", gDefaultAllocator->mNbAllocs);
//	gDefaultAllocator->mLog = false;

	if(renderer)
		mMeshes.push_back(MeshRender(NewMesh, renderer));

	return NewMesh;
}

PintObjectHandle SharedPhysX::CreateArticulation(const PINT_ARTICULATION_CREATE&)
{
	if(mParams.mDisableArticulations)
		return null;

	PxArticulation* Articulation = mPhysics->createArticulation();
	Articulation->setSleepThreshold(mParams.mSleepThreshold);

#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
	// Stabilization can create artefacts on jointed objects so we just disable it
	Articulation->setStabilizationThreshold(0.0f);
#endif

	if(0)
	{
		PxU32 minPositionIters, minVelocityIters;
		Articulation->getSolverIterationCounts(minPositionIters, minVelocityIters);
		printf("minPositionIters: %d\n", minPositionIters);
		printf("minVelocityIters: %d\n", minVelocityIters);
	}
	Articulation->setSolverIterationCounts(mParams.mSolverIterationCountPos, mParams.mSolverIterationCountVel);
	
	// Projection
	Articulation->setMaxProjectionIterations(mParams.mMaxProjectionIterations);
	Articulation->setSeparationTolerance(mParams.mSeparationTolerance);

	//
	Articulation->setExternalDriveIterations(mParams.mExternalDriveIterations);
	Articulation->setInternalDriveIterations(mParams.mInternalDriveIterations);
	return Articulation;
}

bool SharedPhysX::AddArticulationToScene(PintObjectHandle articulation)
{
	PxArticulation* Articulation = (PxArticulation*)articulation;

	// TODO: hack to detect that adding the articulation failed on the GPU. We should revisit this and return bool or something.
	const udword NbErrors = gNbErrors;
	mScene->addArticulation(*Articulation);
	if(gNbErrors!=NbErrors)
		return false;

	SetupSleeping(Articulation, mParams.mEnableSleeping);
	return true;
}

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
	if(bc.mEnableSwingLimit)
		j->setSwingLimit(bc.mSwingYLimit, bc.mSwingZLimit);

	j->setTwistLimitEnabled(bc.mEnableTwistLimit);
	if(bc.mEnableTwistLimit)
		j->setTwistLimit(bc.mTwistLowerLimit, bc.mTwistUpperLimit);

	if(bc.mUseMotor)
	{
		if(bc.mMotor.mExternalCompliance!=0.0f)
			j->setExternalCompliance(bc.mMotor.mExternalCompliance);
		if(bc.mMotor.mInternalCompliance!=0.0f)
			j->setInternalCompliance(bc.mMotor.mInternalCompliance);
		j->setDamping(bc.mMotor.mDamping);
#ifdef IS_PHYSX_3_2
		j->setSpring(bc.mMotor.mStiffness);
#else
		j->setStiffness(bc.mMotor.mStiffness);
#endif
		if(!bc.mMotor.mTargetVelocity.IsNotUsed())
			j->setTargetVelocity(ToPxVec3(bc.mMotor.mTargetVelocity));
		if(!bc.mMotor.mTargetOrientation.IsNotUsed())
			j->setTargetOrientation(ToPxQuat(bc.mMotor.mTargetOrientation));
	}
}

PintObjectHandle SharedPhysX::CreateArticulatedObject(const PINT_OBJECT_CREATE& oc, const PINT_ARTICULATED_BODY_CREATE& bc, PintObjectHandle articulation)
{
	PxArticulation* Articulation = (PxArticulation*)articulation;

	// Note that this already creates the joint between the objects!
	PintObjectHandle h = CreateArticulationLink(Articulation, (PxArticulationLink*)bc.mParent, *this, oc);

	// ...so we setup the joint data immediately
	PxArticulationJoint* joint = ((PxArticulationLink*)h)->getInboundJoint();
	if(joint)	// Will be null for root link
	{
//		PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f, 0.0f, 1.0f));
		const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(bc.mX));

		joint->setParentPose(PxTransform(ToPxVec3(bc.mLocalPivot0), q));
		joint->setChildPose(PxTransform(ToPxVec3(bc.mLocalPivot1), q));
		setupJoint(joint, bc);
	}
	return h;
}

void SharedPhysX::SetArticulatedMotor(PintObjectHandle handle, const PINT_ARTICULATED_MOTOR_CREATE& motor)
{
	PxRigidBody* RigidBody = PhysX3::GetRigidBody(handle);
	if(RigidBody)
	{
		PxArticulationLink* actor = static_cast<PxArticulationLink*>(RigidBody);
		PxArticulationJoint* j = actor->getInboundJoint();
		if(j)
		{
			if(motor.mExternalCompliance!=0.0f)
				j->setExternalCompliance(motor.mExternalCompliance);
			if(motor.mInternalCompliance!=0.0f)
				j->setInternalCompliance(motor.mInternalCompliance);
			j->setDamping(motor.mDamping);
#ifdef IS_PHYSX_3_2
			j->setSpring(motor.mStiffness);
#else
			j->setStiffness(motor.mStiffness);
#endif
			j->setTargetVelocity(ToPxVec3(motor.mTargetVelocity));
		}
	}
}

// TODO: refactor with CreateJoint
PintObjectHandle SharedPhysX::CreateArticulationLink(PxArticulation* articulation, PxArticulationLink* parent, Pint& pint, const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

//	ASSERT(mPhysics);
//	ASSERT(mScene);

	const PxTransform pose(ToPxVec3(desc.mPosition), ToPxQuat(desc.mRotation));

	PxArticulationLink* actor = articulation->createLink(parent, pose);

/*	PxRigidActor* actor;
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
	}*/

	CreateShapes(desc, actor);

	if(actor)
		SetupArticulationLink(*actor, desc);

	// Removed since doesn't work with shared shapes
//	PxSetGroup(*actor, desc.mCollisionGroup);

/*	if(desc.mAddToWorld)
	{
		mScene->addActor(*actor);

		if(rigidDynamic && !desc.mKinematic)
			SetupSleeping(rigidDynamic, mParams.mEnableSleeping);
	}*/
	return CreateHandle(actor);
}

udword SharedPhysX::BatchRaycastAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxRaycastHit Hit;
	while(nb--)
	{
		const bool b = raycastAny(mScene, ToPxVec3(raycasts->mOrigin), ToPxVec3(raycasts->mDir), raycasts->mMaxDist, Hit, PF);
		NbHits += b;
		dest->mHit = b;
		raycasts++;
		dest++;
	}
	return NbHits;
}

udword SharedPhysX::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxOverlapHit Hit;
	while(nb--)
	{
#ifdef IS_PHYSX_3_2
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());
#else
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat(PxIdentity));
#endif

		if(overlapAny(mScene, PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hit, PF))
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

udword SharedPhysX::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxOverlapHit Hits[4096];
	while(nb--)
	{
#ifdef IS_PHYSX_3_2
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());
#else
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat(PxIdentity));
#endif

		PxI32 Nb = overlapMultiple(mScene, PxSphereGeometry(overlaps->mSphere.mRadius), Pose, Hits, 4096, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword SharedPhysX::BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxOverlapHit Hit;
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = overlaps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(overlaps->mBox.mCenter), ToPxQuat(Q));

		if(overlapAny(mScene, PxBoxGeometry(ToPxVec3(overlaps->mBox.mExtents)), Pose, Hit, PF))
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

udword SharedPhysX::BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxOverlapHit Hits[4096];
	while(nb--)
	{
		// ### add this as a helper
		const Quat Q = overlaps->mBox.mRot;	// ### SIGH
		const PxTransform Pose(ToPxVec3(overlaps->mBox.mCenter), ToPxQuat(Q));

		PxI32 Nb = overlapMultiple(mScene, PxBoxGeometry(ToPxVec3(overlaps->mBox.mExtents)), Pose, Hits, 4096, PF);
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

udword SharedPhysX::BatchCapsuleOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)
{
	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

	udword NbHits = 0;
	PxOverlapHit Hit;
	while(nb--)
	{
		// ### refactor this!
		const Point Center = (overlaps->mCapsule.mP0 + overlaps->mCapsule.mP1)*0.5f;
		Point CapsuleAxis = overlaps->mCapsule.mP1 - overlaps->mCapsule.mP0;
		const float M = CapsuleAxis.Magnitude();
		CapsuleAxis /= M;
		const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), ToPxVec3(CapsuleAxis));

		const PxTransform Pose(ToPxVec3(Center), q);

		if(overlapAny(mScene, PxCapsuleGeometry(overlaps->mCapsule.mRadius, M*0.5f), Pose, Hit, PF))
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

udword SharedPhysX::BatchCapsuleOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)
{
/*	ASSERT(mScene);

	const PxQueryFilterData PF = GetSQFilterData();

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

udword SharedPhysX::FindTriangles_MeshSphereOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)
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

#ifdef IS_PHYSX_3_2
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape);
#else
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);
#endif
	PxU32 Results[8192];
	udword NbTouchedTriangles = 0;
	const PxU32 startIndex = 0;
	while(nb--)
	{
#ifdef IS_PHYSX_3_2
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat::createIdentity());
#else
		const PxTransform Pose(ToPxVec3(overlaps->mSphere.mCenter), PxQuat(PxIdentity));
#endif
		bool Overflow = false;
		PxU32 Nb = PxMeshQuery::findOverlapTriangleMesh(PxSphereGeometry(overlaps->mSphere.mRadius), Pose, meshGeom, meshPose, Results, 8192, startIndex, Overflow);
		ASSERT(!Overflow);

		NbTouchedTriangles += Nb;
		overlaps++;
	}
	return NbTouchedTriangles;
}

udword SharedPhysX::FindTriangles_MeshBoxOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps)
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

#ifdef IS_PHYSX_3_2
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape);
#else
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);
#endif

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

udword SharedPhysX::FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps)
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

#ifdef IS_PHYSX_3_2
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape);
#else
	const PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape, *RigidActor);
#endif

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

static	const	bool	gDumpSceneBoundsEachFrame		= false;
static			bool	gVisualizeMBPRegions			= false;

void SharedPhysX::Render(PintRender& renderer)
{
	if(mScene)
	{
		AABB GlobalBounds;
		GlobalBounds.SetEmpty();

#ifdef IS_PHYSX_3_2
		const PxActorTypeSelectionFlags selectionFlags = PxActorTypeSelectionFlag::eRIGID_STATIC | PxActorTypeSelectionFlag::eRIGID_DYNAMIC;
#else
		const PxActorTypeFlags selectionFlags = PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC;
#endif
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
/*								const PxQuat q = PxShortestRotation(PxVec3(0.0f, 1.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
								Quat QQ = IcePose.mRot * ToQuat(q);

								shapeRenderer->Render(PR(IcePose.mPos, QQ));*/

								PR CapsuleTransform;
								PhysX3::ComputeCapsuleTransform(CapsuleTransform, IcePose);
								shapeRenderer->Render(CapsuleTransform);
							}
							else
							{
								shapeRenderer->Render(IcePose);
							}
						}
						// Reactivated for internal XPs
						else
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
						}
					}
				}
			}
		}

		const PxU32 NbArticulations = mScene->getNbArticulations();
		for(PxU32 i=0;i<NbArticulations;i++)
		{
			PxArticulation* Articulation;
			mScene->getArticulations(&Articulation, 1, i);
			const PxU32 NbLinks = Articulation->getNbLinks();
			PxArticulationLink* Links[256];
			PxU32 Nb = Articulation->getLinks(Links, 256);
			for(PxU32 jj=0;jj<NbLinks;jj++)
			{
				PxRigidActor* rigidActor = Links[jj];

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
/*								const PxQuat q = PxShortestRotation(PxVec3(0.0f, 1.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
								Quat QQ = IcePose.mRot * ToQuat(q);

								shapeRenderer->Render(PR(IcePose.mPos, QQ));*/
								PR CapsuleTransform;
								PhysX3::ComputeCapsuleTransform(CapsuleTransform, IcePose);
								shapeRenderer->Render(CapsuleTransform);
							}
							else
							{
								shapeRenderer->Render(IcePose);
							}
						}
						// Reactivated for internal XPs
						else
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
								PxCapsuleGeometry geometry;
								bool status = shape->getCapsuleGeometry(geometry);
								ASSERT(status);

								// ### precompute
/*								const PxQuat q = PxShortestRotation(PxVec3(0.0f, 1.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
								Quat QQ = IcePose.mRot * ToQuat(q);

								renderer.DrawCapsule(geometry.radius, geometry.halfHeight*2.0f, PR(IcePose.mPos, QQ));*/
								PR CapsuleTransform;
								PhysX3::ComputeCapsuleTransform(CapsuleTransform, IcePose);
								renderer.DrawCapsule(geometry.radius, geometry.halfHeight*2.0f, CapsuleTransform);
							}
							else if(geomType==PxGeometryType::eCONVEXMESH)
							{
								ASSERT(0);
							}
							else ASSERT(0);
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

#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
		if(gVisualizeMBPRegions && mParams.mBroadPhaseType == PxBroadPhaseType::eMBP)
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
#endif
	}
}

///////////////////////////////////////////////////////////////////////////////

enum PhysXGUIElement
{
	PHYSX_GUI_MAIN,
	//
	PHYSX_GUI_ENABLE_SLEEPING,
	PHYSX_GUI_ENABLE_SQ,
	PHYSX_GUI_ENABLE_CCD,
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
	PHYSX_GUI_ENABLE_ANGULAR_CCD,
#endif
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	PHYSX_GUI_ENABLE_RAYCAST_CCD,
	PHYSX_GUI_ENABLE_RAYCAST_CCD_DYNA_DYNA,
#endif
	PHYSX_GUI_SQ_FILTER_OUT,
	PHYSX_GUI_SQ_INITIAL_OVERLAP,
//	PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES,
	PHYSX_GUI_SQ_PRECISE_SWEEPS,
	PHYSX_GUI_SQ_REBUILD_RATE_HINT,
	PHYSX_GUI_SHARE_MESH_DATA,
	PHYSX_GUI_SHARE_SHAPES,
#ifdef PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
	PHYSX_GUI_TIGHT_CONVEX_BOUNDS,
#endif
	PHYSX_GUI_PCM,
	PHYSX_GUI_ADAPTIVE_FORCE,
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
	PHYSX_GUI_STABILIZATION,
#endif
#ifdef PHYSX_SUPPORT_SSE_FLAG
	PHYSX_GUI_ENABLE_SSE,
#endif
	PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS,
	PHYSX_GUI_ENABLE_CONTACT_CACHE,
	PHYSX_GUI_FLUSH_SIMULATION,
	PHYSX_GUI_DISABLE_STRONG_FRICTION,
	PHYSX_GUI_ENABLE_ONE_DIR_FRICTION,
	PHYSX_GUI_ENABLE_TWO_DIR_FRICTION,
	PHYSX_GUI_USE_PVD,
	PHYSX_GUI_USE_FULL_PVD_CONNECTION,
//	PHYSX_GUI_DRAW_MBP_REGIONS,
#ifdef PHYSX_SUPPORT_GPU
	PHYSX_GUI_USE_GPU,
#endif
	//
	PHYSX_GUI_NB_THREADS,
	PHYSX_GUI_STATIC_PRUNER,
	PHYSX_GUI_DYNAMIC_PRUNER,
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	PHYSX_GUI_BROAD_PHASE,
#endif
	PHYSX_GUI_SCRATCH_BUFFER,
	//
	PHYSX_GUI_MAX_NB_CCD_PASSES,
	PHYSX_GUI_NB_SOLVER_ITER_POS,
	PHYSX_GUI_NB_SOLVER_ITER_VEL,
	PHYSX_GUI_LINEAR_DAMPING,
	PHYSX_GUI_ANGULAR_DAMPING,
	PHYSX_GUI_MAX_ANGULAR_VELOCITY,
	PHYSX_GUI_MAX_DEPEN_VELOCITY,
//	PHYSX_GUI_NUM_16K_CONTACT_DATA_BLOCKS,
	PHYSX_GUI_GLOBAL_BOX_SIZE,
	PHYSX_GUI_DEFAULT_FRICTION,
	PHYSX_GUI_CONTACT_OFFSET,
	PHYSX_GUI_REST_OFFSET,
#ifdef PHYSX_SUPPORT_SUBSTEPS
	PHYSX_GUI_NB_SUBSTEPS,
	PHYSX_GUI_CONFIG_DESC,
	PHYSX_GUI_CONFIG_COMBO_BOX,
#endif
	PHYSX_GUI_SLEEP_THRESHOLD,
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	PHYSX_GUI_MBP_SUBDIV_LEVEL,
	PHYSX_GUI_MBP_RANGE,
#endif
	//
	PHYSX_GUI_USE_D6_JOINT,
#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
	PHYSX_GUI_DISABLE_PREPROCESSING,
#endif
#ifndef IS_PHYSX_3_2
	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
	PHYSX_GUI_ENABLE_JOINT_32_COMPATIBILITY,
	#endif
#endif
	PHYSX_GUI_ENABLE_JOINT_PROJECTION,
	PHYSX_GUI_PROJECTION_LINEAR_TOLERANCE,
	PHYSX_GUI_PROJECTION_ANGULAR_TOLERANCE,
	PHYSX_GUI_INVERSE_INERTIA_SCALE,
	PHYSX_GUI_INVERSE_MASS_SCALE,
#ifdef PHYSX_SUPPORT_ARTICULATIONS
	PHYSX_GUI_DISABLE_ARTICULATIONS,
	PHYSX_GUI_MAX_PROJECTION_ITERATIONS,
	PHYSX_GUI_SEPARATION_TOLERANCE,
	PHYSX_GUI_INTERNAL_DRIVE_ITERATIONS,
	PHYSX_GUI_EXTERNAL_DRIVE_ITERATIONS,
#endif
	// Cooking
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
	PHYSX_GUI_MESH_COOKING_HINT,
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
	PHYSX_GUI_MID_PHASE,
#endif
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
	PHYSX_GUI_GAUSSMAP_LIMIT,
#endif
#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
	PHYSX_GUI_PRECOMPUTE_ACTIVE_EDGES,
#endif
	//
	PHYSX_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

EditableParams::EditableParams() :
	// Main
	mNbThreads					(0),
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	mScratchSize				(0),
#endif
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	mBroadPhaseType				(PxBroadPhaseType::eSAP),
	mMBPSubdivLevel				(4),
	mMBPRange					(1000.0f),
#endif
	mUseCCD						(false),
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
	mUseAngularCCD				(false),
#endif
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
	mUseRaycastCCD				(false),
	mUseRaycastCCD_DynaDyna		(false),
#endif
	mShareMeshData				(true),
	mShareShapes				(true),
#ifdef PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
	mUseTightConvexBounds		(true),
#endif
	mPCM						(true),
#ifdef PHYSX_SUPPORT_SSE_FLAG
	mEnableSSE					(true),
#endif
	mEnableActiveTransforms		(false),
	mEnableContactCache			(true),
	mFlushSimulation			(false),
#ifdef PINT_SUPPORT_PVD	// Defined in project's properties
	mUsePVD						(true),
#else
	mUsePVD						(false),
#endif
	mUseFullPvdConnection		(true),
#ifdef PHYSX_SUPPORT_GPU
	mUseGPU						(false),
#endif
	//mGlobalBoxSize			(10000.0f),
	mDefaultFriction			(0.5f),
	//mContactOffset			(0.002f),
	mContactOffset				(0.02f),
	mRestOffset					(0.0f),
#ifdef PHYSX_SUPPORT_SUBSTEPS
	mNbSubsteps					(1),
#endif
	// Dynamics
	mEnableSleeping				(false),
	mDisableStrongFriction		(false),
	mEnableOneDirFriction		(false),
	mEnableTwoDirFriction		(false),
	mAdaptiveForce				(false),
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
	mStabilization				(true),
#endif
#ifndef IS_PHYSX_3_2
	mMaxNbCCDPasses				(1),
#endif
	mSolverIterationCountPos	(4),
	mSolverIterationCountVel	(1),
	mLinearDamping				(0.1f),
	mAngularDamping				(0.05f),
	mMaxAngularVelocity			(100.0f),
#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
	mMaxDepenVelocity			(3.0f),
#endif
	mSleepThreshold				(0.05f),
	// Scene queries
	mStaticPruner				(PxPruningStructureType::eDYNAMIC_AABB_TREE),
	mDynamicPruner				(PxPruningStructureType::eDYNAMIC_AABB_TREE),
	mSQDynamicRebuildRateHint	(100),
	mSQFlag						(true),
	mSQFilterOutAllShapes		(false),
	mSQInitialOverlap			(false),
	//mSQManualFlushUpdates		(true),
	mSQPreciseSweeps			(false),
	// Joints
	mEnableJointProjection		(false),
	mUseD6Joint					(false),
#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
	mDisablePreprocessing		(false),
#endif
#ifndef IS_PHYSX_3_2
	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
	mEnableJoint32Compatibility	(false),
	#endif
#endif
	mProjectionLinearTolerance	(0.1f),
	mProjectionAngularTolerance	(180.0f),
#ifndef IS_PHYSX_3_2
	mInverseInertiaScale		(1.0f),
	mInverseMassScale			(1.0f),
#endif
#ifdef PHYSX_SUPPORT_ARTICULATIONS
	// Articulations
	mDisableArticulations		(false),
	mMaxProjectionIterations	(16),
	mSeparationTolerance		(0.001f),
	mExternalDriveIterations	(4),
	mInternalDriveIterations	(4),
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
	mMidPhaseType				(PxMeshMidPhase::eBVH34),
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
	mMeshCookingHint			(PxMeshCookingHint::eSIM_PERFORMANCE),
#endif
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
	mGaussMapLimit				(32),
#endif
#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
	mPrecomputeActiveEdges		(true),
#endif
	mLast						(true)
{
}

static EditableParams gParams;

const EditableParams& PhysX3::GetEditableParams()
{
	return gParams;
}

#define MAX_NB_DEBUG_VIZ_PARAMS	32

	struct PhysXUI : public Allocateable
	{
						PhysXUI(UICallback& callback, udword nb_debug_viz_params, bool* debug_viz_params, const char** debug_viz_names);
						~PhysXUI();

		UICallback&		mCallback;

		udword			mNbDebugVizParams;
		bool*			mDebugVizParams;
		const char**	mDebugVizNames;

		Container*		mPhysXGUI;
		IceComboBox*	mComboBox_NbThreads;
		IceComboBox*	mComboBox_StaticPruner;
		IceComboBox*	mComboBox_DynamicPruner;
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
		IceComboBox*	mComboBox_ScratchSize;
#endif
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
		IceComboBox*	mComboBox_BroadPhase;
		IceEditBox*		mEditBox_MBPSubdivLevel;
		IceEditBox*		mEditBox_MBPRange;
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
		IceComboBox*	mComboBox_MidPhase;
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
		IceComboBox*	mComboBox_MeshCookingHint;
#endif
		IceEditBox*		mEditBox_ProjectionLinearTolerance;
		IceEditBox*		mEditBox_ProjectionAngularTolerance;
		IceEditBox*		mEditBox_InverseInertiaScale;
		IceEditBox*		mEditBox_InverseMassScale;
		IceEditBox*		mEditBox_MaxProjectionIterations;
		IceEditBox*		mEditBox_SeparationTolerance;
		IceEditBox*		mEditBox_ExternalDriveIterations;
		IceEditBox*		mEditBox_InternalDriveIterations;
		IceEditBox*		mEditBox_MaxNbCCDPasses;
		IceEditBox*		mEditBox_SolverIterPos;
		IceEditBox*		mEditBox_SolverIterVel;
		IceEditBox*		mEditBox_LinearDamping;
		IceEditBox*		mEditBox_AngularDamping;
		IceEditBox*		mEditBox_MaxAngularVelocity;
		IceEditBox*		mEditBox_MaxDepenVelocity;
		//IceEditBox*	mEditBox_GlobalBoxSize;
		IceEditBox*		mEditBox_DefaultFriction;
		IceEditBox*		mEditBox_ContactOffset;
		IceEditBox*		mEditBox_RestOffset;
#ifdef PHYSX_SUPPORT_SUBSTEPS
		IceEditBox*		mEditBox_NbSubsteps;
		IceEditBox*		mEditBox_ConfigDesc;
		IceComboBox*	mComboBox_Config;
#endif
		IceEditBox*		mEditBox_SleepThreshold;
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
		IceEditBox*		mEditBox_GaussMapLimit;
#endif
		IceCheckBox*	mCheckBox_TwoDirFriction;
		IceCheckBox*	mCheckBox_Sleeping;
		IceCheckBox*	mCheckBox_PVD;
		IceCheckBox*	mCheckBox_FullPVD;
		IceCheckBox*	mCheckBox_SQ_FilterOutAllShapes;
		IceCheckBox*	mCheckBox_SQ_InitialOverlap;
		//IceCheckBox*	mCheckBox_SQ_ManualFlushUpdates;
		IceCheckBox*	mCheckBox_SQ_PreciseSweeps;
		IceEditBox*		mEditBox_SQ_RebuildRateHint;
		//IceCheckBox*	mCheckBox_DrawMBPRegions;
		IceCheckBox*	mCheckBox_DebugVis[MAX_NB_DEBUG_VIZ_PARAMS];
	};

PhysXUI::PhysXUI(UICallback& callback, udword nb_debug_viz_params, bool* debug_viz_params, const char** debug_viz_names) :
	mCallback							(callback),
	mNbDebugVizParams					(nb_debug_viz_params),
	mDebugVizParams						(debug_viz_params),
	mDebugVizNames						(debug_viz_names),
	mPhysXGUI							(null),
	mComboBox_NbThreads					(null),
	mComboBox_StaticPruner				(null),
	mComboBox_DynamicPruner				(null),
#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	mComboBox_ScratchSize				(null),
#endif
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	mComboBox_BroadPhase				(null),
	mEditBox_MBPSubdivLevel				(null),
	mEditBox_MBPRange					(null),
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
	mComboBox_MidPhase					(null),
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
	mComboBox_MeshCookingHint			(null),
#endif
	mEditBox_ProjectionLinearTolerance	(null),
	mEditBox_ProjectionAngularTolerance	(null),
	mEditBox_InverseInertiaScale		(null),
	mEditBox_InverseMassScale			(null),
	mEditBox_MaxProjectionIterations	(null),
	mEditBox_SeparationTolerance		(null),
	mEditBox_ExternalDriveIterations	(null),
	mEditBox_InternalDriveIterations	(null),
	mEditBox_MaxNbCCDPasses				(null),
	mEditBox_SolverIterPos				(null),
	mEditBox_SolverIterVel				(null),
	mEditBox_LinearDamping				(null),
	mEditBox_AngularDamping				(null),
	mEditBox_MaxAngularVelocity			(null),
	mEditBox_MaxDepenVelocity			(null),
	//mEditBox_GlobalBoxSize			(null),
	mEditBox_DefaultFriction			(null),
	mEditBox_ContactOffset				(null),
	mEditBox_RestOffset					(null),
#ifdef PHYSX_SUPPORT_SUBSTEPS
	mEditBox_NbSubsteps					(null),
	mEditBox_ConfigDesc					(null),
	mComboBox_Config					(null),
#endif
	mEditBox_SleepThreshold				(null),
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
	mEditBox_GaussMapLimit				(null),
#endif
	mCheckBox_TwoDirFriction			(null),
	mCheckBox_Sleeping					(null),
	mCheckBox_PVD						(null),
	mCheckBox_FullPVD					(null),
	mCheckBox_SQ_FilterOutAllShapes		(null),
	mCheckBox_SQ_InitialOverlap			(null),
	//mCheckBox_SQ_ManualFlushUpdates	(null),
	mCheckBox_SQ_PreciseSweeps			(null),
	mEditBox_SQ_RebuildRateHint			(null)
	//mCheckBox_DrawMBPRegions			(null),
{
	ASSERT(nb_debug_viz_params<MAX_NB_DEBUG_VIZ_PARAMS);
	for(udword i=0;i<MAX_NB_DEBUG_VIZ_PARAMS;i++)
		mCheckBox_DebugVis[i] = null;
}

PhysXUI::~PhysXUI()
{
	Common_CloseGUI(mPhysXGUI);
}

static PhysXUI* gPhysXUI = null;

static udword gNbThreadsToIndex[] = { 0, 0, 1, 2, 3 };
static udword gIndexToNbThreads[] = { 0, 2, 3, 4 };

void PhysX3::GetOptionsFromGUI(const char* test_name)
{
	if(!gPhysXUI)
		return;

	if(gPhysXUI->mComboBox_NbThreads)
	{
		const udword Index = gPhysXUI->mComboBox_NbThreads->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToNbThreads)/sizeof(gIndexToNbThreads[0]));
		gParams.mNbThreads = gIndexToNbThreads[Index];
	}

	if(gPhysXUI->mComboBox_StaticPruner)
	{
		const udword Index = gPhysXUI->mComboBox_StaticPruner->GetSelectedIndex();
		gParams.mStaticPruner = PxPruningStructureType::Enum(Index);
	}

	if(gPhysXUI->mComboBox_DynamicPruner)
	{
		const udword Index = gPhysXUI->mComboBox_DynamicPruner->GetSelectedIndex();
		gParams.mDynamicPruner = PxPruningStructureType::Enum(Index);
	}

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
	if(gPhysXUI->mComboBox_ScratchSize)
	{
		const udword Index = gPhysXUI->mComboBox_ScratchSize->GetSelectedIndex();
		gParams.mScratchSize = Index;
	}
#endif

#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	if(gPhysXUI->mComboBox_BroadPhase)
	{
		const udword Index = gPhysXUI->mComboBox_BroadPhase->GetSelectedIndex();
		gParams.mBroadPhaseType = PxBroadPhaseType::Enum(Index);
	}
#endif

#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
	if(gPhysXUI->mComboBox_MidPhase)
	{
		const udword Index = gPhysXUI->mComboBox_MidPhase->GetSelectedIndex();
		gParams.mMidPhaseType = PxMeshMidPhase::Enum(Index);
	}
#endif

#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
	if(gPhysXUI->mComboBox_MeshCookingHint)
	{
		const udword Index = gPhysXUI->mComboBox_MeshCookingHint->GetSelectedIndex();
		gParams.mMeshCookingHint = PxMeshCookingHint::Enum(Index);
	}
#endif

#ifndef IS_PHYSX_3_2
	Common_GetFromEditBox(gParams.mInverseInertiaScale, gPhysXUI->mEditBox_InverseInertiaScale, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mInverseMassScale, gPhysXUI->mEditBox_InverseMassScale, 0.0f, FLT_MAX);
#endif
#ifdef PHYSX_SUPPORT_ARTICULATIONS
	Common_GetFromEditBox(gParams.mMaxProjectionIterations, gPhysXUI->mEditBox_MaxProjectionIterations);
	Common_GetFromEditBox(gParams.mSeparationTolerance, gPhysXUI->mEditBox_SeparationTolerance, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mInternalDriveIterations, gPhysXUI->mEditBox_InternalDriveIterations);
	Common_GetFromEditBox(gParams.mExternalDriveIterations, gPhysXUI->mEditBox_ExternalDriveIterations);
#endif
	Common_GetFromEditBox(gParams.mProjectionLinearTolerance, gPhysXUI->mEditBox_ProjectionLinearTolerance, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mProjectionAngularTolerance, gPhysXUI->mEditBox_ProjectionAngularTolerance, 0.0f, FLT_MAX);
#ifndef IS_PHYSX_3_2
	Common_GetFromEditBox(gParams.mMaxNbCCDPasses, gPhysXUI->mEditBox_MaxNbCCDPasses);
#endif
	Common_GetFromEditBox(gParams.mSolverIterationCountPos, gPhysXUI->mEditBox_SolverIterPos);
	Common_GetFromEditBox(gParams.mSolverIterationCountVel, gPhysXUI->mEditBox_SolverIterVel);
	Common_GetFromEditBox(gParams.mLinearDamping, gPhysXUI->mEditBox_LinearDamping, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mAngularDamping, gPhysXUI->mEditBox_AngularDamping, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mMaxAngularVelocity, gPhysXUI->mEditBox_MaxAngularVelocity, 0.0f, FLT_MAX);
#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
	Common_GetFromEditBox(gParams.mMaxDepenVelocity, gPhysXUI->mEditBox_MaxDepenVelocity, 0.0f, FLT_MAX);
#endif
	Common_GetFromEditBox(gParams.mDefaultFriction, gPhysXUI->mEditBox_DefaultFriction, 0.0f, FLT_MAX);
	Common_GetFromEditBox(gParams.mContactOffset, gPhysXUI->mEditBox_ContactOffset, -FLT_MAX, FLT_MAX);
	Common_GetFromEditBox(gParams.mRestOffset, gPhysXUI->mEditBox_RestOffset, -FLT_MAX, FLT_MAX);
#ifdef PHYSX_SUPPORT_SUBSTEPS
	Common_GetFromEditBox(gParams.mNbSubsteps, gPhysXUI->mEditBox_NbSubsteps);
#endif
	Common_GetFromEditBox(gParams.mSleepThreshold, gPhysXUI->mEditBox_SleepThreshold, 0.0f, FLT_MAX);
#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	Common_GetFromEditBox(gParams.mMBPSubdivLevel, gPhysXUI->mEditBox_MBPSubdivLevel);
	Common_GetFromEditBox(gParams.mMBPRange, gPhysXUI->mEditBox_MBPRange, 0.0f, FLT_MAX);
#endif
	Common_GetFromEditBox(gParams.mSQDynamicRebuildRateHint, gPhysXUI->mEditBox_SQ_RebuildRateHint);
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
	Common_GetFromEditBox(gParams.mGaussMapLimit, gPhysXUI->mEditBox_GaussMapLimit);
#endif

/*	if(gPhysXUI->mEditBox_GlobalBoxSize)
	{
		float tmp;
		bool status = gPhysXUI->mEditBox_GlobalBoxSize->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gGlobalBoxSize = tmp;
	}*/


	EditableParams& EP = const_cast<EditableParams&>(PhysX3::GetEditableParams());

	if(test_name && (strcmp(test_name, "Vehicle2")==0 || strcmp(test_name, "VehicleTest")==0 ))
	{
#ifdef PHYSX_SUPPORT_SUBSTEPS
		if(EP.mNbSubsteps<4)
		{
			printf("WARNING: hardcoding settings for WIP test!\n");
			gParams.mNbSubsteps = 4;
		}
#endif
	}

}

#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
	// ### would be easier to use a callback here
	class BPComboBox : public IceComboBox
	{
		public:
								BPComboBox(const ComboBoxDesc& desc) : IceComboBox(desc)	{}
		virtual	void			OnComboBoxEvent(ComboBoxEvent event)
		{
			if(event==CBE_SELECTION_CHANGED)
			{
				if(gPhysXUI->mEditBox_MBPSubdivLevel)
				{
					const udword Index = GetSelectedIndex();
					if(PxBroadPhaseType::Enum(Index)==PxBroadPhaseType::eSAP)
						gPhysXUI->mEditBox_MBPSubdivLevel->SetEnabled(false);
					else
						gPhysXUI->mEditBox_MBPSubdivLevel->SetEnabled(true);
				}

				if(gPhysXUI->mEditBox_MBPRange)
				{
					const udword Index = GetSelectedIndex();
					if(PxBroadPhaseType::Enum(Index)==PxBroadPhaseType::eSAP)
						gPhysXUI->mEditBox_MBPRange->SetEnabled(false);
					else
						gPhysXUI->mEditBox_MBPRange->SetEnabled(true);
				}
			}
		}
	};
#endif

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword NB_DEBUG_VIZ_PARAMS = gPhysXUI->mNbDebugVizParams;
	bool* gDebugVizParams = gPhysXUI->mDebugVizParams;
	const char** gDebugVizNames = gPhysXUI->mDebugVizNames;

	const udword id = check_box.GetID();
	switch(id)
	{
#ifdef PHYSX_SUPPORT_ARTICULATIONS
		case PHYSX_GUI_DISABLE_ARTICULATIONS:
			gParams.mDisableArticulations = checked;
			break;
#endif
		case PHYSX_GUI_USE_D6_JOINT:
			gParams.mUseD6Joint = checked;
			break;
#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
		case PHYSX_GUI_DISABLE_PREPROCESSING:
			gParams.mDisablePreprocessing = checked;
			break;
#endif
#ifndef IS_PHYSX_3_2
	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
		case PHYSX_GUI_ENABLE_JOINT_32_COMPATIBILITY:
			gParams.mEnableJoint32Compatibility = checked;
			break;
	#endif
#endif
		case PHYSX_GUI_ENABLE_JOINT_PROJECTION:
			gParams.mEnableJointProjection = checked;
			if(gPhysXUI->mEditBox_ProjectionLinearTolerance)
				gPhysXUI->mEditBox_ProjectionLinearTolerance->SetEnabled(checked);
			if(gPhysXUI->mEditBox_ProjectionAngularTolerance)
				gPhysXUI->mEditBox_ProjectionAngularTolerance->SetEnabled(checked);
			break;
		case PHYSX_GUI_ENABLE_SLEEPING:
			gParams.mEnableSleeping = checked;
			break;
		case PHYSX_GUI_ENABLE_SQ:
			gParams.mSQFlag = checked;
			if(gPhysXUI->mCheckBox_SQ_FilterOutAllShapes)
				gPhysXUI->mCheckBox_SQ_FilterOutAllShapes->SetEnabled(checked);
			if(gPhysXUI->mCheckBox_SQ_InitialOverlap)
				gPhysXUI->mCheckBox_SQ_InitialOverlap->SetEnabled(checked);
//			if(gPhysXUI->mCheckBox_SQ_ManualFlushUpdates)
//				gPhysXUI->mCheckBox_SQ_ManualFlushUpdates->SetEnabled(checked);
			if(gPhysXUI->mCheckBox_SQ_PreciseSweeps)
				gPhysXUI->mCheckBox_SQ_PreciseSweeps->SetEnabled(checked);
			if(gPhysXUI->mEditBox_SQ_RebuildRateHint)
				gPhysXUI->mEditBox_SQ_RebuildRateHint->SetEnabled(checked);
			if(gPhysXUI->mComboBox_StaticPruner)
				gPhysXUI->mComboBox_StaticPruner->SetEnabled(checked);
			if(gPhysXUI->mComboBox_DynamicPruner)
				gPhysXUI->mComboBox_DynamicPruner->SetEnabled(checked);
			break;
		case PHYSX_GUI_ENABLE_CCD:
			gParams.mUseCCD = checked;
			break;
#ifdef PHYSX_SUPPORT_ANGULAR_CCD
		case PHYSX_GUI_ENABLE_ANGULAR_CCD:
			gParams.mUseAngularCCD = checked;
			break;
#endif
#ifdef PHYSX_SUPPORT_RAYCAST_CCD
		case PHYSX_GUI_ENABLE_RAYCAST_CCD:
			gParams.mUseRaycastCCD = checked;
			break;
		case PHYSX_GUI_ENABLE_RAYCAST_CCD_DYNA_DYNA:
			gParams.mUseRaycastCCD_DynaDyna = checked;
			break;
#endif
		case PHYSX_GUI_SQ_FILTER_OUT:
			gParams.mSQFilterOutAllShapes = checked;
			break;
		case PHYSX_GUI_SQ_INITIAL_OVERLAP:
			gParams.mSQInitialOverlap = checked;
			break;
/*		case PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES:
			gParams.mSQManualFlushUpdates = checked;
			break;*/
		case PHYSX_GUI_SQ_PRECISE_SWEEPS:
			gParams.mSQPreciseSweeps = checked;
			break;
		case PHYSX_GUI_SHARE_MESH_DATA:
			gParams.mShareMeshData = checked;
			break;
		case PHYSX_GUI_SHARE_SHAPES:
			gParams.mShareShapes = checked;
			break;
#ifdef PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
		case PHYSX_GUI_TIGHT_CONVEX_BOUNDS:
			gParams.mUseTightConvexBounds = checked;
			break;
#endif
		case PHYSX_GUI_PCM:
			gParams.mPCM = checked;
			break;
		case PHYSX_GUI_ADAPTIVE_FORCE:
			gParams.mAdaptiveForce = checked;
			break;
#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
		case PHYSX_GUI_STABILIZATION:
			gParams.mStabilization = checked;
			break;
#endif
#ifdef PHYSX_SUPPORT_SSE_FLAG
		case PHYSX_GUI_ENABLE_SSE:
			gParams.mEnableSSE = checked;
			break;
#endif
		case PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS:
			gParams.mEnableActiveTransforms = checked;
			break;
		case PHYSX_GUI_ENABLE_CONTACT_CACHE:
			gParams.mEnableContactCache = checked;
			break;
		case PHYSX_GUI_FLUSH_SIMULATION:
			gParams.mFlushSimulation = checked;
			break;
		case PHYSX_GUI_DISABLE_STRONG_FRICTION:
			gParams.mDisableStrongFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_ONE_DIR_FRICTION:
			gParams.mEnableOneDirFriction = checked;
			break;
		case PHYSX_GUI_ENABLE_TWO_DIR_FRICTION:
			gParams.mEnableTwoDirFriction = checked;
			break;
		case PHYSX_GUI_USE_PVD:
			gParams.mUsePVD = checked;
			if(gPhysXUI->mCheckBox_FullPVD)
				gPhysXUI->mCheckBox_FullPVD->SetEnabled(checked);
			break;
		case PHYSX_GUI_USE_FULL_PVD_CONNECTION:
			gParams.mUseFullPvdConnection = checked;
			break;
#ifdef PHYSX_SUPPORT_GPU
		case PHYSX_GUI_USE_GPU:
			gParams.mUseGPU = checked;
			break;
#endif
//		case PHYSX_GUI_DRAW_MBP_REGIONS:
//			gVisualizeMBPRegions = checked;
//			break;
#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
		case PHYSX_GUI_PRECOMPUTE_ACTIVE_EDGES:
			gParams.mPrecomputeActiveEdges = checked;
			break;
#endif
		case PHYSX_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gPhysXUI->mCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>PHYSX_GUI_ENABLE_DEBUG_VIZ && id<PHYSX_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-PHYSX_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	gPhysXUI->mCallback.UIModificationCallback();
}

IceWindow* PhysX3::InitSharedGUI(IceWidget* parent, PintGUIHelper& helper, UICallback& callback, udword nb_debug_viz_params, bool* debug_viz_params, const char** debug_viz_names)
{
	ASSERT(!gPhysXUI);
	gPhysXUI = ICE_NEW(PhysXUI)(callback, nb_debug_viz_params, debug_viz_params, debug_viz_names);

	IceWindow* Main_ = helper.CreateMainWindow(gPhysXUI->mPhysXGUI, parent, PHYSX_GUI_MAIN, "PhysX3 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	const sdword YStart = 4;
	sdword y = YStart;

	const sdword OffsetX = 90;
	const sdword LabelOffsetY = 2;

	const sdword EditBoxX = 100;
	const sdword EditBoxWidth = 60;
	const sdword LabelWidth = 100;
	const udword CheckBoxWidth = 190;

	// Tab control
	enum TabIndex
	{
		TAB_MAIN,
		TAB_DYNAMICS,
		TAB_JOINTS,
		TAB_SCENE_QUERIES,
		TAB_COOKING,
		TAB_DEBUG_VIZ,
		TAB_COUNT,
	};
	IceWindow* Tabs[TAB_COUNT];
	{
		TabControlDesc TCD;
		TCD.mParent	= Main_;
		TCD.mX		= 0;
		TCD.mY		= 0;
//		TCD.mWidth	= MainWidth - WD.mX - BorderSize;
//		TCD.mHeight	= MainHeight - BorderSize*2;
		TCD.mWidth	= 500;
		TCD.mHeight	= 410;
		IceTabControl* TabControl = ICE_NEW(IceTabControl)(TCD);
		gPhysXUI->mPhysXGUI->Add(udword(TabControl));

		for(udword i=0;i<TAB_COUNT;i++)
		{
			WindowDesc WD;
			WD.mParent	= Main_;
			WD.mX		= 0;
			WD.mY		= 0;
			WD.mWidth	= 500;
			WD.mHeight	= 410;
			WD.mLabel	= "Tab";
			WD.mType	= WINDOW_DIALOG;
			IceWindow* Tab = ICE_NEW(IceWindow)(WD);
			gPhysXUI->mPhysXGUI->Add(udword(Tab));
			Tab->SetVisible(true);
			Tabs[i] = Tab;
		}
		TabControl->Add(Tabs[TAB_MAIN], "Main");
		TabControl->Add(Tabs[TAB_DYNAMICS], "Dynamics");
		TabControl->Add(Tabs[TAB_JOINTS], "Joints");
		TabControl->Add(Tabs[TAB_SCENE_QUERIES], "Scene queries");
		TabControl->Add(Tabs[TAB_COOKING], "Cooking");
		TabControl->Add(Tabs[TAB_DEBUG_VIZ], "Debug vis.");

		// TAB_MAIN
		{
			IceWindow* TabWindow = Tabs[TAB_MAIN];
			sdword y = YStart;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Num threads:", gPhysXUI->mPhysXGUI);
			ComboBoxDesc CBBD;
			CBBD.mID		= PHYSX_GUI_NB_THREADS;
			CBBD.mParent	= TabWindow;
			CBBD.mX			= 4+OffsetX;
			CBBD.mY			= y;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;
			CBBD.mLabel		= "Num threads";
			gPhysXUI->mComboBox_NbThreads = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_NbThreads));
			gPhysXUI->mComboBox_NbThreads->Add("Single threaded");
			gPhysXUI->mComboBox_NbThreads->Add("1 main + 2 worker threads");
			gPhysXUI->mComboBox_NbThreads->Add("1 main + 3 worker threads");
			gPhysXUI->mComboBox_NbThreads->Add("1 main + 4 worker threads");
			ASSERT(gParams.mNbThreads<sizeof(gNbThreadsToIndex)/sizeof(gNbThreadsToIndex[0]));
			gPhysXUI->mComboBox_NbThreads->Select(gNbThreadsToIndex[gParams.mNbThreads]);
			gPhysXUI->mComboBox_NbThreads->SetVisible(true);
			y += YStep;

#ifdef PHYSX_SUPPORT_SCRATCH_BUFFER
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Scratch buffer:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_SCRATCH_BUFFER;
			CBBD.mY			= y;
			CBBD.mLabel		= "Scratch buffer";
			gPhysXUI->mComboBox_ScratchSize = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_ScratchSize));
			gPhysXUI->mComboBox_ScratchSize->Add("Disabled");
			gPhysXUI->mComboBox_ScratchSize->Add("32 Kb");
			gPhysXUI->mComboBox_ScratchSize->Add("128 Kb");
			gPhysXUI->mComboBox_ScratchSize->Add("256 Kb");
			gPhysXUI->mComboBox_ScratchSize->Add("512 Kb");
			gPhysXUI->mComboBox_ScratchSize->Add("1024 Kb");
			gPhysXUI->mComboBox_ScratchSize->Add("2048 Kb");
			gPhysXUI->mComboBox_ScratchSize->Select(gParams.mScratchSize);
			gPhysXUI->mComboBox_ScratchSize->SetVisible(true);
			y += YStep;
#endif

#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Broad phase:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_BROAD_PHASE;
			CBBD.mY			= y;
			CBBD.mLabel		= "Broad phase";
			gPhysXUI->mComboBox_BroadPhase = ICE_NEW(BPComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_BroadPhase));
			gPhysXUI->mComboBox_BroadPhase->Add("eSAP");
			gPhysXUI->mComboBox_BroadPhase->Add("eMBP");
			gPhysXUI->mComboBox_BroadPhase->Select(gParams.mBroadPhaseType);
			gPhysXUI->mComboBox_BroadPhase->SetVisible(true);
			y += YStep;
#endif
			y += YStep;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gPhysXUI->mPhysXGUI, gParams.mUseCCD, gCheckBoxCallback);
			y += YStepCB;

#ifdef PHYSX_SUPPORT_ANGULAR_CCD
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_ANGULAR_CCD, 4, y, CheckBoxWidth, 20, "Enable angular CCD", gPhysXUI->mPhysXGUI, gParams.mUseAngularCCD, gCheckBoxCallback);
			y += YStepCB;
#endif

#ifdef PHYSX_SUPPORT_RAYCAST_CCD
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_RAYCAST_CCD, 4, y, CheckBoxWidth, 20, "Enable raycast CCD", gPhysXUI->mPhysXGUI, gParams.mUseRaycastCCD, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_RAYCAST_CCD_DYNA_DYNA, 4, y, CheckBoxWidth, 20, "Enable raycast CCD for dynamics", gPhysXUI->mPhysXGUI, gParams.mUseRaycastCCD_DynaDyna, gCheckBoxCallback);
			y += YStepCB;
#endif

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_SHARE_MESH_DATA, 4, y, CheckBoxWidth, 20, "Share mesh data", gPhysXUI->mPhysXGUI, gParams.mShareMeshData, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_SHARE_SHAPES, 4, y, CheckBoxWidth, 20, "Share shapes", gPhysXUI->mPhysXGUI, gParams.mShareShapes, gCheckBoxCallback);
			y += YStepCB;

#ifdef PHYSX_SUPPORT_TIGHT_CONVEX_BOUNDS
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_TIGHT_CONVEX_BOUNDS, 4, y, CheckBoxWidth, 20, "Tight convex bounds", gPhysXUI->mPhysXGUI, gParams.mUseTightConvexBounds, gCheckBoxCallback);
			y += YStepCB;
#endif
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_PCM, 4, y, CheckBoxWidth, 20, "Enable PCM", gPhysXUI->mPhysXGUI, gParams.mPCM, gCheckBoxCallback);
			y += YStepCB;

#ifdef PHYSX_SUPPORT_SSE_FLAG
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_SSE, 4, y, CheckBoxWidth, 20, "Enable SSE", gPhysXUI->mPhysXGUI, gParams.mEnableSSE, gCheckBoxCallback);
			y += YStepCB;
#endif
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_ACTIVE_TRANSFORMS, 4, y, CheckBoxWidth, 20, "Enable active transforms", gPhysXUI->mPhysXGUI, gParams.mEnableActiveTransforms, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_CONTACT_CACHE, 4, y, CheckBoxWidth, 20, "Enable contact cache", gPhysXUI->mPhysXGUI, gParams.mEnableContactCache, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_FLUSH_SIMULATION, 4, y, CheckBoxWidth, 20, "Flush simulation buffers", gPhysXUI->mPhysXGUI, gParams.mFlushSimulation, gCheckBoxCallback);
			y += YStepCB;

			gPhysXUI->mCheckBox_PVD = helper.CreateCheckBox(TabWindow, PHYSX_GUI_USE_PVD, 4, y, CheckBoxWidth, 20, "Use PVD", gPhysXUI->mPhysXGUI, gParams.mUsePVD, gCheckBoxCallback);
			y += YStepCB;

			gPhysXUI->mCheckBox_FullPVD = helper.CreateCheckBox(TabWindow, PHYSX_GUI_USE_FULL_PVD_CONNECTION, 4, y, CheckBoxWidth, 20, "Full connection", gPhysXUI->mPhysXGUI, gParams.mUseFullPvdConnection, gCheckBoxCallback);
	#ifdef PINT_SUPPORT_PVD	// Defined in project's properties
			gPhysXUI->mCheckBox_FullPVD->SetEnabled(gUsePVD);
	#else
			gPhysXUI->mCheckBox_PVD->SetEnabled(false);
			gPhysXUI->mCheckBox_FullPVD->SetEnabled(false);
	#endif
			y += YStepCB;

#ifdef PHYSX_SUPPORT_GPU
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_USE_GPU, 4, y, CheckBoxWidth, 20, "Use GPU", gPhysXUI->mPhysXGUI, gParams.mUseGPU, gCheckBoxCallback);
			y += YStepCB;
#endif
			y += YStepCB;

			y = 100;
			const udword x2 = 250;
//			const udword x2 = 4;


//		helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "World bounds size:", gPhysXUI->mPhysXGUI);
//		gPhysXUI->mEditBox_GlobalBoxSize = helper.CreateEditBox(TabWindow, PHYSX_GUI_GLOBAL_BOX_SIZE, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gGlobalBoxSize), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
//		y += YStep;

			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "Default friction coeff:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_DefaultFriction = helper.CreateEditBox(TabWindow, PHYSX_GUI_DEFAULT_FRICTION, x2+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mDefaultFriction), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "Contact offset:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_ContactOffset = helper.CreateEditBox(TabWindow, PHYSX_GUI_CONTACT_OFFSET, x2+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mContactOffset), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT, null);
			y += YStep;

			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "Rest offset:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_RestOffset = helper.CreateEditBox(TabWindow, PHYSX_GUI_REST_OFFSET, x2+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mRestOffset), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT, null);
			y += YStep;

#ifdef PHYSX_SUPPORT_SUBSTEPS
			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "Nb substeps:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_NbSubsteps = helper.CreateEditBox(TabWindow, PHYSX_GUI_NB_SUBSTEPS, x2+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mNbSubsteps), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;
#endif

#ifdef PHYSX_SUPPORT_PX_BROADPHASE_TYPE
			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "MBP subdiv level:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_MBPSubdivLevel = helper.CreateEditBox(TabWindow, PHYSX_GUI_MBP_SUBDIV_LEVEL, x2+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mMBPSubdivLevel), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, x2, y+LabelOffsetY, LabelWidth, 20, "MBP range:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_MBPRange = helper.CreateEditBox(TabWindow, PHYSX_GUI_MBP_RANGE, x2+EditBoxX, y, EditBoxWidth, 20, _F("%f", gParams.mMBPRange), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			if(gParams.mBroadPhaseType==PxBroadPhaseType::eSAP)
			{
				gPhysXUI->mEditBox_MBPSubdivLevel->SetEnabled(false);
				gPhysXUI->mEditBox_MBPRange->SetEnabled(false);
			}
			else
			{
				gPhysXUI->mEditBox_MBPSubdivLevel->SetEnabled(true);
				gPhysXUI->mEditBox_MBPRange->SetEnabled(true);
			}
#endif

#ifdef PHYSX_SUPPORT_SUBSTEPS
			const udword x3 = 200;
			y += YStep;

			static const char* ConfigDesc0 = "4 velocity solver iterations, single-threaded,\nno substeps, sleeping disabled.";
			static const char* ConfigDesc1 = "4 velocity solver iterations, 4 threads,\nno substeps, sleeping enabled.";
			static const char* ConfigDesc2 = "8 velocity solver iterations, single-threaded,\n2 substeps, sleeping disabled.";

			class ConfigComboBox : public IceComboBox
			{
				public:
								ConfigComboBox(const ComboBoxDesc& desc) : IceComboBox(desc)	{}

				virtual	void	OnComboBoxEvent(ComboBoxEvent event)
								{
									if(event==CBE_SELECTION_CHANGED)
									{
										const udword SelectedIndex = GetSelectedIndex();
										if(SelectedIndex==0)
										{
											gPhysXUI->mEditBox_NbSubsteps->SetText("1");
											gPhysXUI->mEditBox_SolverIterPos->SetText("4");
											gPhysXUI->mComboBox_NbThreads->Select(gNbThreadsToIndex[0]);
											gParams.mEnableSleeping = false;
											gPhysXUI->mEditBox_ConfigDesc->SetMultilineText(ConfigDesc0);
										}
										else if(SelectedIndex==1)
										{
											gPhysXUI->mEditBox_NbSubsteps->SetText("1");
											gPhysXUI->mEditBox_SolverIterPos->SetText("4");
											gPhysXUI->mComboBox_NbThreads->Select(gNbThreadsToIndex[4]);
											gParams.mEnableSleeping = true;
											gPhysXUI->mEditBox_ConfigDesc->SetMultilineText(ConfigDesc1);
										}
										else if(SelectedIndex==2)
										{
											gPhysXUI->mEditBox_NbSubsteps->SetText("2");
											gPhysXUI->mEditBox_SolverIterPos->SetText("8");
											gPhysXUI->mComboBox_NbThreads->Select(gNbThreadsToIndex[0]);
											gParams.mEnableSleeping = false;
											gPhysXUI->mEditBox_ConfigDesc->SetMultilineText(ConfigDesc2);
										}
										gPhysXUI->mCheckBox_Sleeping->SetChecked(gParams.mEnableSleeping);
									}
								}
			};

			{
			ComboBoxDesc CBBD;
			CBBD.mParent	= TabWindow;
			CBBD.mX			= x3;
			CBBD.mWidth		= 250;
			CBBD.mHeight	= 20;
			CBBD.mID		= PHYSX_GUI_CONFIG_COMBO_BOX;
			CBBD.mY			= y;
			CBBD.mLabel		= "Configuration";
			gPhysXUI->mComboBox_Config = ICE_NEW(ConfigComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_Config));
			gPhysXUI->mComboBox_Config->Add("Configure for comparison tests (default)");
			gPhysXUI->mComboBox_Config->Add("Configure for better performance");
			gPhysXUI->mComboBox_Config->Add("Configure for better sim quality");
			gPhysXUI->mComboBox_Config->Select(0);
			gPhysXUI->mComboBox_Config->SetVisible(true);
			y += YStep;
			}

			gPhysXUI->mEditBox_ConfigDesc = helper.CreateEditBox(TabWindow, PHYSX_GUI_CONFIG_DESC, x3, y, 250, 60, "", gPhysXUI->mPhysXGUI, EDITBOX_TEXT, null);
			gPhysXUI->mEditBox_ConfigDesc->SetMultilineText(ConfigDesc0);
			gPhysXUI->mEditBox_ConfigDesc->SetReadOnly(true);
#endif
		}

		// TAB_DYNAMICS
		{
			IceWindow* TabWindow = Tabs[TAB_DYNAMICS];
			sdword y = YStart;

			gPhysXUI->mCheckBox_Sleeping = helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gPhysXUI->mPhysXGUI, gParams.mEnableSleeping, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_DISABLE_STRONG_FRICTION, 4, y, CheckBoxWidth, 20, "Disable strong friction", gPhysXUI->mPhysXGUI, gParams.mDisableStrongFriction, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_ONE_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable one dir. friction", gPhysXUI->mPhysXGUI, gParams.mEnableOneDirFriction, gCheckBoxCallback);
			y += YStepCB;

			gPhysXUI->mCheckBox_TwoDirFriction = helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_TWO_DIR_FRICTION, 4, y, CheckBoxWidth, 20, "Enable two dir. friction", gPhysXUI->mPhysXGUI, gParams.mEnableTwoDirFriction, gCheckBoxCallback);
			y += YStepCB;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ADAPTIVE_FORCE, 4, y, CheckBoxWidth, 20, "Adaptive force", gPhysXUI->mPhysXGUI, gParams.mAdaptiveForce, gCheckBoxCallback);
			y += YStepCB;

#ifdef PHYSX_SUPPORT_STABILIZATION_FLAG
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_STABILIZATION, 4, y, CheckBoxWidth, 20, "Stabilization", gPhysXUI->mPhysXGUI, gParams.mStabilization, gCheckBoxCallback);
			y += YStepCB;
#endif
			y += YStep;

#ifndef IS_PHYSX_3_2
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max nb CCD passes:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_MaxNbCCDPasses = helper.CreateEditBox(TabWindow, PHYSX_GUI_MAX_NB_CCD_PASSES, 4+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mMaxNbCCDPasses), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;
#endif
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Solver iter pos:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_SolverIterPos = helper.CreateEditBox(TabWindow, PHYSX_GUI_NB_SOLVER_ITER_POS, 4+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mSolverIterationCountPos), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Solver iter vel:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_SolverIterVel = helper.CreateEditBox(TabWindow, PHYSX_GUI_NB_SOLVER_ITER_VEL, 4+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mSolverIterationCountVel), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Linear damping:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_LinearDamping = helper.CreateEditBox(TabWindow, PHYSX_GUI_LINEAR_DAMPING, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mLinearDamping), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Angular damping:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_AngularDamping = helper.CreateEditBox(TabWindow, PHYSX_GUI_ANGULAR_DAMPING, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mAngularDamping), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max angular velocity:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_MaxAngularVelocity = helper.CreateEditBox(TabWindow, PHYSX_GUI_MAX_ANGULAR_VELOCITY, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mMaxAngularVelocity), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

#ifdef PHYSX_SUPPORT_MAX_DEPEN_VELOCITY
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max depen. velocity:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_MaxDepenVelocity = helper.CreateEditBox(TabWindow, PHYSX_GUI_MAX_DEPEN_VELOCITY, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mMaxDepenVelocity), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;
#endif

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Sleep threshold:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_SleepThreshold = helper.CreateEditBox(TabWindow, PHYSX_GUI_REST_OFFSET, 4+EditBoxX, y, EditBoxWidth, 20, helper.Convert(gParams.mSleepThreshold), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;
		}

		// TAB_JOINTS
		{
			IceWindow* TabWindow = Tabs[TAB_JOINTS];
			sdword y = YStart;

			const sdword xj = 4;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_USE_D6_JOINT, xj, y, CheckBoxWidth, 20, "Use D6 joint if possible", gPhysXUI->mPhysXGUI, gParams.mUseD6Joint, gCheckBoxCallback);
			y += YStepCB;

#ifdef PHYSX_SUPPORT_DISABLE_PREPROCESSING
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_DISABLE_PREPROCESSING, xj, y, CheckBoxWidth, 20, "Disable preprocessing", gPhysXUI->mPhysXGUI, gParams.mDisablePreprocessing, gCheckBoxCallback);
			y += YStepCB;
#endif
#ifndef IS_PHYSX_3_2
	#ifndef PHYSX_REMOVE_JOINT_32_COMPATIBILITY
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_JOINT_32_COMPATIBILITY, xj, y, CheckBoxWidth, 20, "Enable 3.2 compatibility", gPhysXUI->mPhysXGUI, gParams.mEnableJoint32Compatibility, gCheckBoxCallback);
			y += YStepCB;
	#endif
#endif
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_JOINT_PROJECTION, xj, y, CheckBoxWidth, 20, "Enable joint projection", gPhysXUI->mPhysXGUI, gParams.mEnableJointProjection, gCheckBoxCallback);
//			y += YStepCB;

			y += YStep;

			const sdword LabelWidthJ = 150;
			helper.CreateLabel(TabWindow, xj, y+LabelOffsetY, LabelWidthJ, 20, "Projection linear tolerance:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_ProjectionLinearTolerance = helper.CreateEditBox(TabWindow, PHYSX_GUI_PROJECTION_LINEAR_TOLERANCE, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%f", gParams.mProjectionLinearTolerance), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			gPhysXUI->mEditBox_ProjectionLinearTolerance->SetEnabled(gParams.mEnableJointProjection);
			y += YStep;

			helper.CreateLabel(TabWindow, xj, y+LabelOffsetY, LabelWidthJ, 20, "Projection angular tolerance:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_ProjectionAngularTolerance = helper.CreateEditBox(TabWindow, PHYSX_GUI_PROJECTION_ANGULAR_TOLERANCE, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%f", gParams.mProjectionAngularTolerance), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			gPhysXUI->mEditBox_ProjectionAngularTolerance->SetEnabled(gParams.mEnableJointProjection);
			y += YStep;

#ifndef IS_PHYSX_3_2
			helper.CreateLabel(TabWindow, xj, y+LabelOffsetY, LabelWidthJ, 20, "Inverse inertia scale:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_InverseInertiaScale = helper.CreateEditBox(TabWindow, PHYSX_GUI_INVERSE_INERTIA_SCALE, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%f", gParams.mInverseInertiaScale), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(TabWindow, xj, y+LabelOffsetY, LabelWidthJ, 20, "Inverse mass scale:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_InverseMassScale = helper.CreateEditBox(TabWindow, PHYSX_GUI_INVERSE_MASS_SCALE, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%f", gParams.mInverseMassScale), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;
#endif
			y += YStep;

#ifdef PHYSX_SUPPORT_ARTICULATIONS
			{
				sdword YSaved = y;

				y += 20;
				const sdword xa = xj + 10;

				helper.CreateCheckBox(TabWindow, PHYSX_GUI_DISABLE_ARTICULATIONS, xa, y, CheckBoxWidth, 20, "Disable articulations", gPhysXUI->mPhysXGUI, gParams.mDisableArticulations, gCheckBoxCallback);
				y += YStepCB;
				y += YStepCB;

				helper.CreateLabel(TabWindow, xa, y+LabelOffsetY, LabelWidthJ, 20, "Max Projection Iterations:", gPhysXUI->mPhysXGUI);
				gPhysXUI->mEditBox_MaxProjectionIterations = helper.CreateEditBox(TabWindow, PHYSX_GUI_MAX_PROJECTION_ITERATIONS, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%d", gParams.mMaxProjectionIterations), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
				y += YStep;

				helper.CreateLabel(TabWindow, xa, y+LabelOffsetY, LabelWidthJ, 20, "Separation tolerance:", gPhysXUI->mPhysXGUI);
				gPhysXUI->mEditBox_SeparationTolerance = helper.CreateEditBox(TabWindow, PHYSX_GUI_SEPARATION_TOLERANCE, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%f", gParams.mSeparationTolerance), gPhysXUI->mPhysXGUI, EDITBOX_FLOAT_POSITIVE, null);
				y += YStep;

				helper.CreateLabel(TabWindow, xa, y+LabelOffsetY, LabelWidthJ, 20, "Internal drive iterations:", gPhysXUI->mPhysXGUI);
				gPhysXUI->mEditBox_InternalDriveIterations = helper.CreateEditBox(TabWindow, PHYSX_GUI_INTERNAL_DRIVE_ITERATIONS, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%d", gParams.mInternalDriveIterations), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
				y += YStep;

				helper.CreateLabel(TabWindow, xa, y+LabelOffsetY, LabelWidthJ, 20, "External drive iterations:", gPhysXUI->mPhysXGUI);
				gPhysXUI->mEditBox_ExternalDriveIterations = helper.CreateEditBox(TabWindow, PHYSX_GUI_EXTERNAL_DRIVE_ITERATIONS, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%d", gParams.mExternalDriveIterations), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
				y += YStep;

				{
					EditBoxDesc EBD;
					EBD.mParent		= TabWindow;
					EBD.mX			= xj;
					EBD.mY			= YSaved;
					EBD.mWidth		= 250;
					EBD.mHeight		= 150;
					EBD.mLabel		= "============ Articulations ============";
					EBD.mFilter		= EDITBOX_TEXT;
					EBD.mType		= EDITBOX_READ_ONLY;
					IceEditBox* EB = ICE_NEW(IceEditBox)(EBD);
					EB->SetVisible(true);
					gPhysXUI->mPhysXGUI->Add(udword(EB));
				}
			}
#endif
		}

		// TAB_SCENE_QUERIES
		{
			IceWindow* TabWindow = Tabs[TAB_SCENE_QUERIES];
			sdword y = YStart;

			helper.CreateCheckBox(TabWindow, PHYSX_GUI_ENABLE_SQ, 4, y, CheckBoxWidth, 20, "Enable scene queries", gPhysXUI->mPhysXGUI, gParams.mSQFlag, gCheckBoxCallback);
			y += YStepCB;

			gPhysXUI->mCheckBox_SQ_FilterOutAllShapes = helper.CreateCheckBox(TabWindow, PHYSX_GUI_SQ_FILTER_OUT, 4, y, CheckBoxWidth, 20, "Filter out all shapes (DEBUG)", gPhysXUI->mPhysXGUI, gParams.mSQFilterOutAllShapes, gCheckBoxCallback);
			gPhysXUI->mCheckBox_SQ_FilterOutAllShapes->SetEnabled(gParams.mSQFlag);
			y += YStepCB;

			gPhysXUI->mCheckBox_SQ_InitialOverlap = helper.CreateCheckBox(TabWindow, PHYSX_GUI_SQ_INITIAL_OVERLAP, 4, y, CheckBoxWidth, 20, "eINITIAL_OVERLAP flag (sweeps)", gPhysXUI->mPhysXGUI, gParams.mSQInitialOverlap, gCheckBoxCallback);
			gPhysXUI->mCheckBox_SQ_InitialOverlap->SetEnabled(gParams.mSQFlag);
			y += YStepCB;

	/*		gPhysXUI->mCheckBox_SQ_ManualFlushUpdates = helper.CreateCheckBox(TabWindow, PHYSX_GUI_SQ_MANUAL_FLUSH_UPDATES, 4, y, CheckBoxWidth, 20, "Manual flush updates", gPhysXUI->mPhysXGUI, gParams.mSQManualFlushUpdates, gCheckBoxCallback);
			gPhysXUI->mCheckBox_SQ_ManualFlushUpdates->SetEnabled(gParams.mSQFlag);
			y += YStepCB;*/

			gPhysXUI->mCheckBox_SQ_PreciseSweeps = helper.CreateCheckBox(TabWindow, PHYSX_GUI_SQ_PRECISE_SWEEPS, 4, y, CheckBoxWidth, 20, "Precise sweeps", gPhysXUI->mPhysXGUI, gParams.mSQPreciseSweeps, gCheckBoxCallback);
			gPhysXUI->mCheckBox_SQ_PreciseSweeps->SetEnabled(gParams.mSQFlag);
			y += YStepCB;

			y += YStepCB;
			const sdword xj = 4;
			const sdword LabelWidthJ = 150;
			helper.CreateLabel(TabWindow, xj, y+LabelOffsetY, LabelWidthJ, 20, "Tree rebuild rate hint:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_SQ_RebuildRateHint = helper.CreateEditBox(TabWindow, PHYSX_GUI_SQ_REBUILD_RATE_HINT, xj+LabelWidthJ, y, EditBoxWidth, 20, _F("%d", gParams.mSQDynamicRebuildRateHint), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			gPhysXUI->mEditBox_SQ_RebuildRateHint->SetEnabled(gParams.mSQFlag);
			y += YStep;

			y += YStep;

			ComboBoxDesc CBBD;
			CBBD.mParent	= TabWindow;
			CBBD.mX			= 4+OffsetX;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Static pruner:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_STATIC_PRUNER;
			CBBD.mY			= y;
			CBBD.mLabel		= "Static pruner";
			gPhysXUI->mComboBox_StaticPruner = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_StaticPruner));
			gPhysXUI->mComboBox_StaticPruner->Add("eNONE");
			gPhysXUI->mComboBox_StaticPruner->Add("eDYNAMIC_AABB_TREE");
			gPhysXUI->mComboBox_StaticPruner->Add("eSTATIC_AABB_TREE");
			gPhysXUI->mComboBox_StaticPruner->Select(gParams.mStaticPruner);
			gPhysXUI->mComboBox_StaticPruner->SetVisible(true);
			gPhysXUI->mComboBox_StaticPruner->SetEnabled(gParams.mSQFlag);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Dynamic pruner:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_DYNAMIC_PRUNER;
			CBBD.mY			= y;
			CBBD.mLabel		= "Dynamic pruner";
			gPhysXUI->mComboBox_DynamicPruner = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_DynamicPruner));
			gPhysXUI->mComboBox_DynamicPruner->Add("eNONE");
			gPhysXUI->mComboBox_DynamicPruner->Add("eDYNAMIC_AABB_TREE");
			gPhysXUI->mComboBox_DynamicPruner->Select(gParams.mDynamicPruner);
			gPhysXUI->mComboBox_DynamicPruner->SetVisible(true);
			gPhysXUI->mComboBox_DynamicPruner->SetEnabled(gParams.mSQFlag);
			y += YStep;
		}

		// TAB_COOKING
		{
			IceWindow* TabWindow = Tabs[TAB_COOKING];
			sdword y = YStart;

			ComboBoxDesc CBBD;
			CBBD.mParent	= TabWindow;
			CBBD.mX			= 4+OffsetX;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;

#ifdef PHYSX_SUPPORT_PX_MESH_COOKING_HINT
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Mesh cooking hint:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_MESH_COOKING_HINT;
			CBBD.mY			= y;
			CBBD.mLabel		= "Mesh cooking hint";
			gPhysXUI->mComboBox_MeshCookingHint = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_MeshCookingHint));
			gPhysXUI->mComboBox_MeshCookingHint->Add("eSIM_PERFORMANCE");
			gPhysXUI->mComboBox_MeshCookingHint->Add("eCOOKING_PERFORMANCE");
			gPhysXUI->mComboBox_MeshCookingHint->Select(gParams.mMeshCookingHint);
			gPhysXUI->mComboBox_MeshCookingHint->SetVisible(true);
			y += YStep;
#endif
#ifdef PHYSX_SUPPORT_PX_MESH_MIDPHASE
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "Midphase:", gPhysXUI->mPhysXGUI);
			CBBD.mID		= PHYSX_GUI_MID_PHASE;
			CBBD.mY			= y;
			CBBD.mLabel		= "Midphase";
			gPhysXUI->mComboBox_MidPhase = ICE_NEW(IceComboBox)(CBBD);
			gPhysXUI->mPhysXGUI->Add(udword(gPhysXUI->mComboBox_MidPhase));
			gPhysXUI->mComboBox_MidPhase->Add("eBVH33");
			gPhysXUI->mComboBox_MidPhase->Add("eBVH34");
			gPhysXUI->mComboBox_MidPhase->Select(gParams.mMidPhaseType);
			gPhysXUI->mComboBox_MidPhase->SetVisible(true);
			y += YStep;
#endif
#ifdef PHYSX_SUPPORT_USER_DEFINED_GAUSSMAP_LIMIT
			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, 90, 20, "GaussMap limit:", gPhysXUI->mPhysXGUI);
			gPhysXUI->mEditBox_GaussMapLimit = helper.CreateEditBox(TabWindow, PHYSX_GUI_GAUSSMAP_LIMIT, 4+EditBoxX, y, EditBoxWidth, 20, _F("%d", gParams.mGaussMapLimit), gPhysXUI->mPhysXGUI, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;
#endif
#ifdef PHYSX_SUPPORT_DISABLE_ACTIVE_EDGES_PRECOMPUTE
			helper.CreateCheckBox(TabWindow, PHYSX_GUI_PRECOMPUTE_ACTIVE_EDGES, 4, y, CheckBoxWidth, 20, "Precompute active edges", gPhysXUI->mPhysXGUI, gParams.mPrecomputeActiveEdges, gCheckBoxCallback);
			y += YStepCB;
#endif
		}

		// TAB_DEBUG_VIZ
		{
			const udword NB_DEBUG_VIZ_PARAMS = gPhysXUI->mNbDebugVizParams;
			bool* gDebugVizParams = gPhysXUI->mDebugVizParams;
			const char** gDebugVizNames = gPhysXUI->mDebugVizNames;

			IceWindow* TabWindow = Tabs[TAB_DEBUG_VIZ];
			sdword LastY = Common_CreateDebugVizUI(TabWindow, 10, YStart, gCheckBoxCallback, PHYSX_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gPhysXUI->mCheckBox_DebugVis, gPhysXUI->mPhysXGUI);
		}
	}
//	gPhysXUI->mCheckBox_DrawMBPRegions = helper.CreateCheckBox(Main, PHYSX_GUI_DRAW_MBP_REGIONS, 290+10, LastY, CheckBoxWidth, 20, "Draw MBP regions", gPhysXUI->mPhysXGUI, gVisualizeMBPRegions, gCheckBoxCallback);
	return Main_;
}

void PhysX3::CloseSharedGUI()
{
	DELETESINGLE(gPhysXUI);
}
