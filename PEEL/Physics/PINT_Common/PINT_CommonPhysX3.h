///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_COMMON_PHYSX3_H
#define PINT_COMMON_PHYSX3_H

	#define SAFE_RELEASE(x)	if(x) { x->release(); x = null; }

	inline_ Point	ToPoint(const PxVec3& p)	{ return Point(p.x, p.y, p.z);				}
	inline_ Quat	ToQuat(const PxQuat& q)		{ return Quat(q.w, q.x, q.y, q.z);			}
	inline_ PxVec3	ToPxVec3(const Point& p)	{ return PxVec3(p.x, p.y, p.z);				}
	inline_ PxQuat	ToPxQuat(const Quat& q)		{ return PxQuat(q.p.x, q.p.y, q.p.z, q.w);	}

	inline_	PintObjectHandle CreateHandle(PxRigidActor* actor)
	{
		const size_t binary = size_t(actor);
		ASSERT(!(binary&1));
		return PintObjectHandle(binary);
	}

	inline_	PintObjectHandle CreateHandle(PxShape* shape)
	{
		const size_t binary = size_t(shape);
		ASSERT(!(binary&1));
		return PintObjectHandle(binary|1);
	}

	inline_	PxRigidActor* GetActorFromHandle(PintObjectHandle handle)
	{
		const size_t binary = size_t(handle);
		return (binary & 1) ? null : (PxRigidActor*)binary;
	}

	inline_	PxShape* GetShapeFromHandle(PintObjectHandle handle)
	{
		const size_t binary = size_t(handle);
		return (binary & 1) ? (PxShape*)(binary&~1) : null;
	}

	class MemoryOutputStream : public PxOutputStream
	{
	public:
						MemoryOutputStream();
	virtual				~MemoryOutputStream();

			PxU32		write(const void* src, PxU32 count);

			PxU32		getSize()	const	{	return mSize; }
			PxU8*		getData()	const	{	return mData; }
	private:
			PxU8*		mData;
			PxU32		mSize;
			PxU32		mCapacity;
	};

	class MemoryInputData : public PxInputData
	{
	public:
						MemoryInputData(PxU8* data, PxU32 length);

			PxU32		read(void* dest, PxU32 count);
			PxU32		getLength() const;
			void		seek(PxU32 pos);
			PxU32		tell() const;

	private:
			PxU32		mSize;
			const PxU8*	mData;
			PxU32		mPos;
	};

	class PEEL_PhysX3_ErrorCallback : public PxErrorCallback
	{
	public:
				PEEL_PhysX3_ErrorCallback()		{}
		virtual	~PEEL_PhysX3_ErrorCallback()	{}

		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line);
	};

	class PEEL_PhysX3_AllocatorCallback : public PxAllocatorCallback
	{
	public:
		struct Header
		{
			udword	mMagic;
			udword	mSize;
		};

				PEEL_PhysX3_AllocatorCallback();
		virtual	~PEEL_PhysX3_AllocatorCallback();

		virtual	void*	allocate(size_t size, const char* typeName, const char* filename, int line);
		virtual	void	deallocate(void* ptr);

				udword	mNbAllocs;
				udword	mCurrentMemory;
	};

	namespace PhysX3
	{
		PR		GetWorldTransform(PintObjectHandle handle);
		void	ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
	}

#endif