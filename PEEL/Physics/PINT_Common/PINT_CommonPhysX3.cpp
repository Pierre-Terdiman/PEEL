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
#include "PINT_CommonPhysX3.h"

///////////////////////////////////////////////////////////////////////////////

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

void PEEL_PhysX3_ErrorCallback::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
	printf(message);
}

///////////////////////////////////////////////////////////////////////////////

PEEL_PhysX3_AllocatorCallback::PEEL_PhysX3_AllocatorCallback() :
	mNbAllocs		(0),
	mCurrentMemory	(0)
{
}

PEEL_PhysX3_AllocatorCallback::~PEEL_PhysX3_AllocatorCallback()
{
	if(mNbAllocs)
		printf("PhysX 3: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
}

void* PEEL_PhysX3_AllocatorCallback::allocate(size_t size, const char* typeName, const char* filename, int line)
{
//	printf("%d | %s | %s\n", size, typeName, filename);
//return _aligned_malloc(size, 16);
/*	if(size>8192)
	{
		int stop=1;
	}*/
	char* memory = (char*)_aligned_malloc(size+16, 16);
	Header* H = (Header*)memory;
	H->mMagic = 0x12345678;
	H->mSize = size;
	mNbAllocs++;
	mCurrentMemory+=size;
	return memory + 16;
}

void PEEL_PhysX3_AllocatorCallback::deallocate(void* ptr)
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

///////////////////////////////////////////////////////////////////////////////

PR PhysX3::GetWorldTransform(PintObjectHandle handle)
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
#ifdef SUPPORT_SHARED_SHAPES
		ASSERT(Shape->getActor());
		Pose = PxShapeExt::getGlobalPose(*Shape, *Shape->getActor());
#else
		Pose = PxShapeExt::getGlobalPose(*Shape);
#endif
	}

	return PR(ToPoint(Pose.p), ToQuat(Pose.q));
}

///////////////////////////////////////////////////////////////////////////////

#ifndef PHYSX3_DISABLE_SHARED_APPLY_ACTION
void PhysX3::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	PxRigidActor* RigidActor = GetActorFromHandle(handle);
	if(!RigidActor)
	{
		PxShape* Shape = GetShapeFromHandle(handle);
		ASSERT(Shape);
#ifdef SUPPORT_SHARED_SHAPES
		RigidActor = Shape->getActor();
#else
		RigidActor = &Shape->getActor();
#endif
	}

	if(RigidActor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
	{
		PxRigidDynamic* RigidDynamic = static_cast<PxRigidDynamic*>(RigidActor);
		if(!(RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
		{
			PxForceMode::Enum mode;
			if(action_type==PINT_ACTION_FORCE)
				mode = PxForceMode::eFORCE;
			else if(action_type==PINT_ACTION_IMPULSE)
				mode = PxForceMode::eIMPULSE;
			else ASSERT(0);

			PxRigidBodyExt::addForceAtPos(*RigidDynamic, ToPxVec3(action), ToPxVec3(pos), mode);
		}
	}
}
#endif
