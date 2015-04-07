///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "..\PINT_Common\PINT_Common.h"

///////////////////////////////////////////////////////////////////////////////

static			udword									gNbThreads				= 0;
static	const	bool									gUseCustomMemory		= true;
static			hkpRigidBodyCinfo::SolverDeactivation	gSolverDeactivation		= hkpRigidBodyCinfo::SOLVER_DEACTIVATION_MEDIUM;
static			hkpWorldCinfo::ContactPointGeneration	gContactPointGeneration	= hkpWorldCinfo::CONTACT_POINT_REJECT_MANY;
static			bool									gEnableSleeping			= false;
static			bool									gEnableKDTree			= true;
static			float									gLinearDamping			= 0.1f;
static			float									gAngularDamping			= 0.05f;
static			float									gGlobalBoxSize			= 10000.0f;
static			float									gMaxLinearVelocity		= 10000.0f;
static			bool									gShareShapes			= true;
static			udword									gSolverIterationCount	= 4;
static			bool									gUseVDB					= false;
static			bool									gUseCCD					= false;

///////////////////////////////////////////////////////////////////////////////

#include "..\PINT_Common\PINT_CommonHavok.h"

///////////////////////////////////////////////////////////////////////////////

/*void* hkSystemMalloc(int size, int align)
{
	return 0;
}
void hkSystemFree(void* p)
{
}*/


static void HK_CALL errorReport(const char* msg, void*)
{
	printf("HAVOK message: %s", msg);
}

Havok::Havok() :
	mThreadMemory	(null),
	mStackBuffer	(null),
	mThreadPool		(null),
	mJobQueue		(null),
	mPhysicsWorld	(null),
	mContext		(null),
	mVdb			(null)
{
}

Havok::~Havok()
{
	ASSERT(!mVdb);
	ASSERT(!mContext);
	ASSERT(!mPhysicsWorld);
	ASSERT(!mJobQueue);
	ASSERT(!mThreadPool);
	ASSERT(!mStackBuffer);
	ASSERT(!mThreadMemory);
}

void Havok::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportKinematics				= false;
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
	caps.mSupportConvexSweeps			= true;
	caps.mSupportSphereOverlaps			= true;
	caps.mSupportBoxOverlaps			= false;
	caps.mSupportCapsuleOverlaps		= false;
	caps.mSupportConvexOverlaps			= false;
}

// given an unaligned pointer, round it up to align and
// store the offset just before the returned pointer.
static inline void* hkMemoryRoundUp(void* pvoid, int align=16)
{
  char* p = reinterpret_cast<char*>(pvoid);
  char* aligned = reinterpret_cast<char*>(
    HK_NEXT_MULTIPLE_OF( align, reinterpret_cast<int>(p+1)) );
  reinterpret_cast<int*>(aligned)[-1] = (int)(aligned - p);
  return aligned;
}

// given a pointer from hkMemoryRoundUp, recover the original pointer.
static inline void* hkMemoryRoundDown(void* p)
{
  int offset = reinterpret_cast<int*>(p)[-1];
  return static_cast<char*>(p) - offset;
}

class CustomMemory : public hkMemory
{
  public:

		struct Header
		{
			udword	mMagic;
			udword	mSize;
		};

		CustomMemory() :
			mNbAllocs		(0),
			mCurrentMemory	(0)
		{
		}
		~CustomMemory()
		{
			if(mNbAllocs)
				printf("Havok: %d leaks found (%d bytes)\n", mNbAllocs, mCurrentMemory);
		}

		void* __allocate(size_t size)
		{
			char* memory = (char*)_aligned_malloc(size+16, 16);
			Header* H = (Header*)memory;
			H->mMagic = 0x12345678;
			H->mSize = size;
			mNbAllocs++;
			mCurrentMemory+=size;
			return memory + 16;
		}

		void __deallocate(void* ptr)
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


			/// Allocate nblocks of nbytes. This is equivalent to nblocks calls to
			/// allocateChunk but is more lock friendly.
		virtual void allocateChunkBatch(void** blocksOut, int nblocks, int nbytes, HK_MEMORY_CLASS cl)
		{
			for(int i=0;i<nblocks;i++)
//				blocksOut[i] = _aligned_malloc(nbytes, 16);
				blocksOut[i] = __allocate(nbytes);
		}

			/// Deallocate nblocks of nbytes. This is equivalent to nblocks calls to
			/// deallocateChunk but is more lock friendly.
		virtual void deallocateChunkBatch(void** blocksOut, int nblocks, int nbytes, HK_MEMORY_CLASS cl)
		{
			for(int i=0;i<nblocks;i++)
//				_aligned_free(blocksOut[i]);
				__deallocate(blocksOut[i]);
		}

			/// Allocates a fixed size piece of memory, aligned to 16 bytes.
			/// Allocations of 8 bytes or less are aligned to 8 bytes.
			/// Note that the size is not stored and must be supplied when deallocating.
		virtual void* allocateChunk( int nbytes, HK_MEMORY_CLASS cl)
		{
//			return _aligned_malloc(nbytes, 16);
			return __allocate(nbytes);
		}

			/// Deallocates a piece of memory that has been allocated by allocateChunk
		virtual void deallocateChunk( void* ptr, int nbytes, HK_MEMORY_CLASS cl )
		{
//			_aligned_free(ptr);
			__deallocate(ptr);
		}

			/// Allocates a block of size nbytes of fast runtime memory
			/// If a matching block is available this function will return it.
			/// If no matching block could be found this function will allocate a new block and issue a warning,
			/// add it to its managed array of blocks and return a pointer to it.
			///
			/// General info on fast runtime blocks:
			/// Regular stack memory is not working properly in a multi-threaded environment. To cater to the need for
			/// fast memory allocations during simulation, the concept of dedicated runtime memory blocks (working
			/// as a stack memory replacement) has been devised.
			/// Runtime block functions provide access to very fast memory allocation by internally
			/// storing an array of preallocated memory blocks that are re-used on the fly as soon
			/// as they have been declared available again by explicitly deallocating them. New
			/// allocations are only performed if no available memory block of matching size could
			/// be found in the internal array of managed block.
			/// Internally these runtime blocks are only used inside the simulation (i.e. during a call to
			/// stepDeltaTime() or stepProcessMt()) and can therefore be used freely outside by the user.
		virtual void* allocateRuntimeBlock(int nbytes, HK_MEMORY_CLASS cl)
		{
//			return _aligned_malloc(nbytes, 16);
			return __allocate(nbytes);
		}

			/// Deallocates a block of fast runtime memory that has been allocated by allocateRuntimeBlock()
			/// For more information on runtime blocks see allocateRuntimeBlock()
		virtual void deallocateRuntimeBlock(void* ptr, int nbytes, HK_MEMORY_CLASS cl)
		{
//			_aligned_free(ptr);
			__deallocate(ptr);
		}

			/// Preallocates a fast runtime memory block of size nbytes
			/// For more information on runtime blocks see allocateRuntimeBlock()
		virtual void preAllocateRuntimeBlock(int nbytes, HK_MEMORY_CLASS cl)
		{
//			ASSERT(0);
		}

			/// This allows the user to provide his own block(s) of memory for fast runtime memory
			/// Note that memory blocks that have been provided by the user will not be deallocated by a call to
			/// freeRuntimeBlocks(). The user has to take care of deallocation himself.
			/// For more information on runtime blocks see allocateRuntimeBlock()
		virtual void provideRuntimeBlock(void*, int nbytes, HK_MEMORY_CLASS cl)
		{
//			ASSERT(0);
		}

			/// Deallocate all fast runtime memory blocks that are still allocated and that have NOT been provided
			/// by the user (by using provideRuntimeBlock())
			/// For more information on runtime blocks see allocateRuntimeBlock()
		virtual void freeRuntimeBlocks()
		{
//			ASSERT(0);
		}

			/// Prints some statistics to the specified console.
		virtual void printStatistics(hkOstream* c)
		{
			ASSERT(0);
		}

			/// Find the memory rounded up by the memory allocator for a requested memory allocation
		virtual int getAllocatedSize( int size )
		{
			ASSERT(0);
			return 0;
		}

            /// Work out memory statistics. This function tries to work out fairly accurately details about memory usage -
            /// this functions performance may not be great depending on the underlying implementation.
            /// See hkMemoryStatisticcs for details of the information returned.
            /// NOTE! That if you have a memory limit set (via setMemoryLimit) the values returned will reflect that limit
            /// you must have the limit set to zero if you want to find out the 'true' memory statistics.
        virtual void calculateStatistics(hkMemoryStatistics& stats)
		{
			ASSERT(0);
		}

            /// There are two styles of limit in the memory allocator, a soft limit and a hard limit.
            /// If the soft limit is exceeded it will _NOT_ stop an allocation being performed if it can be performed.
            /// The way to test to see if the soft limit has been exceeded is to call hasMemoryAvailable, which will
            /// return false if the soft limit has been passed.
            /// The other type of limit is a 'hard' limit. The hard limit cannot be passed - if you try and allocate
            /// past the hard limit allocation will fail and HK_NULL will be returned. Passing the hard limit is also
            /// what will produce calls to the hkLimitedMemoryListener. There is no listener mechanism around the
            /// soft limit, for performance reasons.
            /// Allow the setting of a memory usage limit (in bytes). Depending on the implementation the limit
            /// may or may not be honored. Look at the docs on specific implementations to see if memory limiting
            /// is supported. If limiting is not supported the method will have no effect. If the memory requested was
            /// unfeasibly low (for example more than that amount of memory has already been allocated) the
            /// actual setting does not change. Check getMemoryLimit to see if the change has been accepted.
            /// Setting the limit to 0, will set the limit to as large as possible
        virtual void setMemoryHardLimit(hk_size_t maxMemory) { }
            /// Returns what the limit is set to (in bytes). If there is no limit the method returns 0.
        virtual hk_size_t getMemoryHardLimit() { return 0; }
            /// set a soft memory limit. Setting 0 will mean there is no soft limit.
            /// See setMemoryHardLimit for a description of the difference between a soft and hard limit
        virtual void setMemorySoftLimit(hk_size_t maxMemory) { }

            /// Returns what the soft memory limit is. 0 means there is no soft memory limit.
        virtual hk_size_t getMemorySoftLimit() { return 0; }

            /// Set the limited memory listener. May not be implemented on all implementations - you can check
            /// by doing a get, if it is HK_NULL and you tried to set it to something, support for the listener isn't implemented
            /// on this platform.
			/// If the listener is set, it becomes the listeners responsibility to try and make memory avaialable. One way to make
			/// memory available is to call a 'garbageCollect' which will combine small free blocks to make larger allocations
			/// possible.
        virtual void setLimitedMemoryListener(hkLimitedMemoryListener* listener) {}
            /// Get the limited memory listener. Not all memory systems support listeners.
        virtual hkLimitedMemoryListener* getLimitedMemoryListener() { return HK_NULL; }

            /// Returns true if an amount of memory of at least 'size' is available. Why not just have a method that returns
            /// the amount of memory available? Because with some allocators its meaningless - for when you have virtual
            /// memory, and because the implementation can decide broadly what 'enough memory' means - as it has internal
            /// knowledge about how memory works. This method is designed to be fast so can be called rapidly if needs be,
            /// but recommended use would be to call infrequently and check that larger allocations are available.
            /// Passing in size as zero well return true if some 'nominal' amount of memory required for normal function
            /// is available.
        virtual hkBool hasMemoryAvailable(hk_size_t size) { return true; }

			/// Some implementations (the DebugMemory for instance) can do checks on the current
			/// allocated pool. If the implementation does not support the check it will just return true.
		virtual hkBool isOk() const
		{
			return true;
		}

			/// Using the hkDebugMemory implementation you can use this function to search for the originally allocated memory block
			/// within which the supplied memory block is located. This function returns the originally allocated memory block's base
			/// address. It will assert if it cannot find a matching block.
		virtual void* findBaseAddress(void* p, int nbytes) { return HK_NULL; }

			/// Using the hkDebugMemory you can explicitly lock allocated memory. Upon freeing a memory block this lock counter will be
			/// checked and the debug memory will assert if there is still a lock left.
		virtual void lockBlock(void* p) {}

			/// Unlock memory in hkDebugMemory.
		virtual void unlockBlock(void* p) {}

            /// Traverses all of the memory used by the subsystem (allocated and free). The order that allocations
            /// are returned is undefined. Not all memory allocators can walk memory - if an implementation cannot
            /// it will return HK_FAILURE from this method.
        virtual hkResult walkMemory(hkMemoryWalkCallback callback,void* param) { return HK_FAILURE; }


/*    void lock(); // platform specific thread lock
    void unlock(); // platform specific thread unlock

    inline void* _allocateChunk(int blockSize)
    {
      return hkMemoryRoundUp(::new char[blockSize+16]);
    }

    inline void* _deallocateChunk(void* p, int blockSize)
    {
      ::delete [] static_cast<char*>( hkMemoryRoundDown(p) );
    }

    virtual void* allocateChunkBatch(void** ptrs, int numBlocks, int blockSize)
    {
      lock();
      for( int i = 0; i < numBlocks; ++i )
      {
        ptrs[i] = _allocateChunk(blockSize);
      }
      unlock();
    }

    virtual void deallocateChunkBatch(void** ptrs, int numBlocks, int blockSize)
    {
      lock();
      for( int i = 0; i < numBlocks; ++i )
      {
        ptrs[i] = _deallocateChunk(ptrs[i],blockSize);
      }
      unlock();
    }

    virtual void* allocateChunk(int nbytes, HK_MEMORY_CLASS cl)
    {
      lock();
      void* p = _allocateChunk(nbytes);
      unlock();
      return p;
    }

    virtual void deallocateChunk(void* p, int nbytes, HK_MEMORY_CLASS )
    {
      if(p)
      {
        lock();
        _deallocateChunk(p, nbytes);
        unlock();
      }
    }

    virtual void printStatistics(hkOstream* c) { }*/
};

static CustomMemory* gMemoryManager = null;

void Havok::Init(const PINT_WORLD_CREATE& desc)
{
	// Initialize the base system including our memory system
	hkMemory* memoryManager = null;
	if(gUseCustomMemory)
	{
		CustomMemory* CM = new CustomMemory();
		gMemoryManager = CM;
		memoryManager = CM;
	}
	else
	{
		hkPoolMemory* PM = new hkPoolMemory();
		memoryManager = PM;
	}

	mThreadMemory = new hkThreadMemory(memoryManager);
	hkBaseSystem::init(memoryManager, mThreadMemory, errorReport);
	memoryManager->removeReference();

	// We now initialize the stack area to 100k (fast temporary memory to be used by the engine).
	if(1)
	{
		const int stackSize = 0x100000;
//		const int stackSize = 32000;
		mStackBuffer = hkAllocate<char>(stackSize, HK_MEMORY_CLASS_BASE);
		hkThreadMemory::getInstance().setStackArea(mStackBuffer, stackSize);
	}

	if(gNbThreads>=2)
	{
		//
		// Initialize the multi-threading classes, hkJobQueue, and hkJobThreadPool
		//

		// They can be used for all Havok multithreading tasks. In this exmaple we only show how to use
		// them for physics, but you can reference other multithreading demos in the demo framework
		// to see how to multithread other products. The model of usage is the same as for physics.
		// The hkThreadpool has a specified number of threads that can run Havok jobs.  These can work
		// alongside the main thread to perform any Havok multi-threadable computations.
		// The model for running Havok tasks in Spus and in auxilary threads is identical.  It is encapsulated in the
		// class hkJobThreadPool.  On PLAYSTATION(R)3 we initialize the SPU version of this class, which is simply a SPURS taskset.
		// On other multi-threaded platforms we initialize the CPU version of this class, hkCpuJobThreadPool, which creates a pool of threads
		// that run in exactly the same way.  On the PLAYSTATION(R)3 we could also create a hkCpuJobThreadPool.  However, it is only
		// necessary (and advisable) to use one Havok PPU thread for maximum efficiency. In this case we simply use this main thread
		// for this purpose, and so do not create a hkCpuJobThreadPool.

		// We can cap the number of threads used - here we use the maximum for whatever multithreaded platform we are running on. This variable is
		// set in the following code sections.
		int totalNumThreadsUsed = gNbThreads;

		// Get the number of physical threads available on the system
/*		hkHardwareInfo hwInfo;
		hkGetHardwareInfo(hwInfo);
		totalNumThreadsUsed = hwInfo.m_numThreads;*/

		// We use one less than this for our thread pool, because we must also use this thread for our simulation
		hkCpuJobThreadPoolCinfo threadPoolCinfo;
		threadPoolCinfo.m_numThreads = totalNumThreadsUsed - 1;

		// This line enables timers collection, by allocating 200 Kb per thread.  If you leave this at its default (0),
		// timer collection will not be enabled.
		threadPoolCinfo.m_timerBufferPerThreadAllocation = 200000;
		mThreadPool = new hkCpuJobThreadPool( threadPoolCinfo );

		// We also need to create a Job queue. This job queue will be used by all Havok modules to run multithreaded work.
		// Here we only use it for physics.
		hkJobQueueCinfo info;
		info.m_jobQueueHwSetup.m_numCpuThreads = totalNumThreadsUsed;
		mJobQueue = new hkJobQueue(info);

		//
		// Enable monitors for this thread.
		//

		// Monitors have been enabled for thread pool threads already (see above comment).
		hkMonitorStream::getInstance().resize(200000);
	}

	//
	// <PHYSICS-ONLY>: Create the physics world.
	// At this point you would initialize any other Havok modules you are using.
	//
	{
		// The world cinfo contains global simulation parameters, including gravity, solver settings etc.
		hkpWorldCinfo worldInfo;

		worldInfo.m_useKdTree = gEnableKDTree;
		worldInfo.m_contactPointGeneration = gContactPointGeneration;

		if(desc.mGlobalBounds.IsValid())
		{
			Point m, M;
			desc.mGlobalBounds.GetMin(m);
			desc.mGlobalBounds.GetMax(M);

			worldInfo.m_broadPhaseWorldAabb = hkAabb(ToHkVector4(m), ToHkVector4(M));
		}
		else
		{
			worldInfo.m_broadPhaseWorldAabb = hkAabb(hkVector4(-gGlobalBoxSize, -gGlobalBoxSize, -gGlobalBoxSize), hkVector4(gGlobalBoxSize, gGlobalBoxSize, gGlobalBoxSize));
		}

		worldInfo.m_gravity = ToHkVector4(desc.mGravity);

		worldInfo.m_solverIterations = gSolverIterationCount;

		// Set the simulation type of the world to multi-threaded.
		if(gNbThreads>=2)
		{
			worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;
		}
		else
		{
			if(gUseCCD)
			{
				worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_CONTINUOUS;
				worldInfo.m_sizeOfToiEventQueue = 1024;
			}
			else
				worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_DISCRETE;
		}

		// Flag objects that fall "out of the world" to be automatically removed - just necessary for this physics scene
		worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_REMOVE_ENTITY;

		worldInfo.m_enableDeactivation = gEnableSleeping;

		mPhysicsWorld = new hkpWorld(worldInfo);

		// Disable deactivation, so that you can view timers in the VDB. This should not be done in your game.
		mPhysicsWorld->m_wantDeactivation = gEnableSleeping;


		// When the simulation type is SIMULATION_TYPE_MULTITHREADED, in the debug build, the sdk performs checks
		// to make sure only one thread is modifying the world at once to prevent multithreaded bugs. Each thread
		// must call markForRead / markForWrite before it modifies the world to enable these checks.
		mPhysicsWorld->markForWrite();


		// Register all collision agents, even though only box - box will be used in this particular example.
		// It's important to register collision agents before adding any entities to the world.
		hkpAgentRegisterUtil::registerAllAgents( mPhysicsWorld->getCollisionDispatcher() );

		// We need to register all modules we will be running multi-threaded with the job queue
		if(mJobQueue)
			mPhysicsWorld->registerWithJobQueue(mJobQueue);

		// Now we have finished modifying the world, release our write marker.
		mPhysicsWorld->unmarkForWrite();
	}

	if(gUseVDB)
	{
		//
		// Initialize the VDB
		//
		hkArray<hkProcessContext*> contexts;

		// <PHYSICS-ONLY>: Register physics specific visual debugger processes
		// By default the VDB will show debug points and lines, however some products such as physics and cloth have additional viewers
		// that can show geometries etc and can be enabled and disabled by the VDB app.
		{
			mPhysicsWorld->markForWrite();

			// The visual debugger so we can connect remotely to the simulation
			// The context must exist beyond the use of the VDB instance, and you can make
			// whatever contexts you like for your own viewer types.
			mContext = new hkpPhysicsContext();
			hkpPhysicsContext::registerAllPhysicsProcesses(); // all the physics viewers
			mContext->addWorld(mPhysicsWorld); // add the physics world so the viewers can see it
			contexts.pushBack(mContext);

			// Now we have finished modifying the world, release our write marker.
			mPhysicsWorld->unmarkForWrite();
		}

		mVdb = new hkVisualDebugger(contexts);
		mVdb->serve();
	}
}

void Havok::SetGravity(const Point& gravity)
{
	ASSERT(mPhysicsWorld);
	mPhysicsWorld->setGravity(ToHkVector4(gravity));
}

void Havok::Close()
{
/*	if(0)
	{
		udword Size = mRigidBodies.size();
		for(udword i=0;i<Size;i++)
		{
			hkpRigidBody* rb = mRigidBodies[i];
			DELETESINGLE(rb);
		}
	}*/

	// <PHYSICS-ONLY>: cleanup physics
	{
		mPhysicsWorld->markForWrite();
		mPhysicsWorld->removeReference();
		mPhysicsWorld = null;
	}

	{
		if(mVdb)
		{
			mVdb->removeReference();
			mVdb = null;
		}

		// Contexts are not reference counted at the base class level by the VDB as
		// they are just interfaces really. So only delete the context after you have
		// finished using the VDB.
		if(mContext)
		{
			mContext->removeReference();
			mContext = null;
		}
	}

	{
		DELETESINGLE(mJobQueue);

		//
		// Clean up the thread pool
		//

		if(mThreadPool)
		{
			mThreadPool->removeReference();
			mThreadPool = null;
		}
	}

	// Deallocate stack area
	mThreadMemory->setStackArea(0, 0);
	hkDeallocate(mStackBuffer);
	mStackBuffer = null;

	mThreadMemory->removeReference();
	mThreadMemory = null;

	// Quit base system
	hkBaseSystem::quit();

//	DELETESINGLE(gMemoryManager);
	gMemoryManager = null;
}

udword Havok::Update(float timestep)
{
	// <PHYSICS-ONLY>:
	// Step the physics world. This single call steps using this thread and all threads
	// in the threadPool. For other products you add jobs, call process all jobs and wait for completion.
	// See the multithreading chapter in the user guide for details
	if(gNbThreads>=2)
	{
		mPhysicsWorld->stepMultithreaded(mJobQueue, mThreadPool, timestep);
	}
	else
	{
		mPhysicsWorld->stepDeltaTime(timestep);
	}

	// Step the visual debugger. We first synchronize the timer data
	if(mContext)
		mContext->syncTimers(mThreadPool);
	if(mVdb)
		mVdb->step();

	// Clear accumulated timer data in this thread and all slave threads
	hkMonitorStream::getInstance().reset();
	if(mThreadPool)
		mThreadPool->clearTimerData();

	if(gMemoryManager)
		return gMemoryManager->mCurrentMemory;

	return 0;
}

static void DrawLeafShape(PintRender& renderer, const hkpShape* shape, const PR& pose)
{
	ASSERT(shape->getUserData());
	if(shape->getUserData())
	{
		PintShapeRenderer* psr = (PintShapeRenderer*)shape->getUserData();
		psr->Render(pose);
//		return;
	}

/*	const hkpShapeType Type = shape->getType();

	if(Type==HK_SHAPE_SPHERE)
	{
		const hkpSphereShape* SphereShape = static_cast<const hkpSphereShape*>(shape);
		const float Radius = SphereShape->getRadius();
		renderer.DrawSphere(Radius, pose);
	}
	else if(Type==HK_SHAPE_BOX)
	{
		const hkpBoxShape* BoxShape = static_cast<const hkpBoxShape*>(shape);
		const hkVector4& Extents = BoxShape->getHalfExtents();
		renderer.DrawBox(ToPoint(Extents), pose);
	}
	else if(Type==HK_SHAPE_CONVEX_VERTICES)
	{
		const hkpConvexVerticesShape* ConvexShape = static_cast<const hkpConvexVerticesShape*>(shape);
		ASSERT(0);
	}
	else ASSERT(0);*/
}

Point Havok::GetMainColor()
{
	return Point(0.0f, 1.0f, 1.0f);
}

void Havok::Render(PintRender& renderer)
{
	const udword Size = mRigidBodies.GetNbEntries();
	for(udword i=0;i<Size;i++)
	{
		hkpRigidBody* rigidBody = (hkpRigidBody*)mRigidBodies[i];

		rigidBody->markForRead();

		const hkTransform& BodyPose = rigidBody->getTransform();

		const hkpCollidable* collidable = rigidBody->getCollidable();
		const hkpShape* shape = collidable->getShape();
		const hkpShapeType type = shape->getType();
		if(type==HK_SHAPE_LIST)
		{
			const hkpListShape* listShape = static_cast<const hkpListShape*>(shape);
			const int NbShapes = listShape->getNumChildShapes();
			for(int j=0;j<NbShapes;j++)
			{
				const hkpShape* ChildShape = listShape->getChildShapeInl(j);
				const hkpShapeType ChildShapeType = ChildShape->getType();
				if(ChildShapeType==HK_SHAPE_CONVEX_TRANSFORM)
				{
					const hkpConvexTransformShape* ConvexTransformShape = static_cast<const hkpConvexTransformShape*>(ChildShape);
					const hkpConvexShape* LeafShape = ConvexTransformShape->getChildShape();
					const hkTransform& LeafTransform = ConvexTransformShape->getTransform();

					hkTransform Combo;
					Combo.setMul(BodyPose, LeafTransform);

					DrawLeafShape(renderer, LeafShape, ToPR(Combo));
				}
				else ASSERT(0);
			}
		}
		else
		{
			DrawLeafShape(renderer, shape, ToPR(BodyPose));
		}

		rigidBody->unmarkForRead();
	}
}

hkpRigidBody* Havok::CreateObject(const hkpRigidBodyCinfo& info, const PINT_OBJECT_CREATE& create, hkpShape* shape)
{
	hkpRigidBody* rigidBody = new hkpRigidBody(info);

	if(create.mAddToWorld)
	{
		mPhysicsWorld->markForWrite();
		mPhysicsWorld->addEntity(rigidBody);
		mPhysicsWorld->unmarkForWrite();
		rigidBody->removeReference();
	}

	shape->removeReference();

	if(info.m_motionType != hkpMotion::MOTION_FIXED)
	{
		rigidBody->markForWrite();
		rigidBody->setMaxLinearVelocity(gMaxLinearVelocity);
		rigidBody->setLinearVelocity(ToHkVector4(create.mLinearVelocity));
		rigidBody->setAngularVelocity(ToHkVector4(create.mAngularVelocity));
		rigidBody->unmarkForWrite();
	}

	mRigidBodies.Add(udword(rigidBody));

//	hkReal pd = rigidBody->getAllowedPenetrationDepth();

	return rigidBody;
}

bool Havok::ReleaseObject(PintObjectHandle handle)
{
	hkpRigidBody* rigidBody = (hkpRigidBody*)handle;
	mRigidBodies.Delete(udword(handle));
		mPhysicsWorld->markForWrite();
//int test0 = rigidBody->getReferenceCount();
		hkBool Status = mPhysicsWorld->removeEntity(rigidBody);
//int test1 = rigidBody->getReferenceCount();
		ASSERT(Status);
		mPhysicsWorld->unmarkForWrite();
//	DELETESINGLE(rigidBody);
//rigidBody->removeReference();
//int test2 = rigidBody->getReferenceCount();
	return true;
}

static void FillRigidBodyInfo(hkpRigidBodyCinfo& info, const PINT_OBJECT_CREATE& desc, hkpShape* shape)
{
	info.m_shape				= shape;
	info.m_position				= ToHkVector4(desc.mPosition);
	info.m_rotation				= ToHkQuaternion(desc.mRotation);
	info.m_solverDeactivation	= gSolverDeactivation;
	info.m_collisionFilterInfo	= hkpGroupFilter::calcFilterInfo(desc.mCollisionGroup);
	if(desc.mMass!=0.0f)
	{
		info.m_linearDamping	= gLinearDamping;
		info.m_angularDamping	= gAngularDamping;

		if(gUseCCD)
			info.m_qualityType		= HK_COLLIDABLE_QUALITY_BULLET;
//			info.m_qualityType		= HK_COLLIDABLE_QUALITY_CRITICAL;
	}
	else
	{
		info.m_qualityType		= HK_COLLIDABLE_QUALITY_FIXED;
		info.m_motionType		= hkpMotion::MOTION_FIXED;
	}
}

PintObjectHandle Havok::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;
	if(NbShapes>1)
		return CreateCompoundObject(desc);

	if(desc.mShapes->mType==PINT_SHAPE_SPHERE)
	{
		const PINT_SPHERE_CREATE* SphereCreate = static_cast<const PINT_SPHERE_CREATE*>(desc.mShapes);

		hkpSphereShape* shape = FindSphereShape(gShareShapes, mSphereShapes, *SphereCreate);

		hkpRigidBodyCinfo info;
		if(desc.mMass!=0.0f)
		{
			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeSphereVolumeMassProperties(SphereCreate->mRadius, desc.mMass, massProperties);

			info.m_mass				= massProperties.m_mass;
			info.m_centerOfMass		= massProperties.m_centerOfMass;
			info.m_inertiaTensor	= massProperties.m_inertiaTensor;
			info.m_motionType		= hkpMotion::MOTION_SPHERE_INERTIA;
		//	info.m_motionType		= hkpMotion::MOTION_BOX_INERTIA;
		}
		FillRigidBodyInfo(info, desc, shape);

		if(SphereCreate->mMaterial)
		{
			info.m_friction		= SphereCreate->mMaterial->mDynamicFriction;
			info.m_restitution	= SphereCreate->mMaterial->mRestitution;
		}
		else
		{
			info.m_friction		= 0.5f;
			info.m_restitution	= 0.0f;
		}

		return CreateObject(info, desc, shape);
	}
	else if(desc.mShapes->mType==PINT_SHAPE_BOX)
	{
		const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(desc.mShapes);

		const hkVector4 halfExtents = ToHkVector4(BoxCreate->mExtents);

		hkpBoxShape* shape = FindBoxShape(gShareShapes, mBoxShapes, *BoxCreate);

		hkpRigidBodyCinfo info;
//		info.m_allowedPenetrationDepth = 1.0f;
		if(desc.mMass!=0.0f)
		{
			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, desc.mMass, massProperties);

			info.m_mass				= massProperties.m_mass;
			info.m_centerOfMass		= massProperties.m_centerOfMass;
			info.m_inertiaTensor	= massProperties.m_inertiaTensor;
			info.m_motionType		= hkpMotion::MOTION_BOX_INERTIA;
//			info.m_motionType		= hkpMotion::MOTION_SPHERE_INERTIA;
		}
		FillRigidBodyInfo(info, desc, shape);

		if(BoxCreate->mMaterial)
		{
			info.m_friction		= BoxCreate->mMaterial->mDynamicFriction;
			info.m_restitution	= BoxCreate->mMaterial->mRestitution;
		}
		else
		{
			info.m_friction		= 0.5f;
			info.m_restitution	= 0.0f;
		}

		return CreateObject(info, desc, shape);
	}
	else if(desc.mShapes->mType==PINT_SHAPE_CAPSULE)
	{
		const PINT_CAPSULE_CREATE* CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE*>(desc.mShapes);

		hkpCapsuleShape* shape = FindCapsuleShape(gShareShapes, mCapsuleShapes, *CapsuleCreate);

		hkpRigidBodyCinfo info;
//		info.m_allowedPenetrationDepth = 1.0f;
		if(desc.mMass!=0.0f)
		{
			const hkVector4 vectorA(0.0f, CapsuleCreate->mHalfHeight, 0.0f);
			const hkVector4 vectorB(0.0f, -CapsuleCreate->mHalfHeight, 0.0f);

			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeCapsuleVolumeMassProperties(vectorA, vectorB, CapsuleCreate->mRadius, desc.mMass, massProperties);

			info.m_mass				= massProperties.m_mass;
			info.m_centerOfMass		= massProperties.m_centerOfMass;
			info.m_inertiaTensor	= massProperties.m_inertiaTensor;
			info.m_motionType		= hkpMotion::MOTION_DYNAMIC;
		}
		FillRigidBodyInfo(info, desc, shape);

		if(CapsuleCreate->mMaterial)
		{
			info.m_friction		= CapsuleCreate->mMaterial->mDynamicFriction;
			info.m_restitution	= CapsuleCreate->mMaterial->mRestitution;
		}
		else
		{
			info.m_friction		= 0.5f;
			info.m_restitution	= 0.0f;
		}

		return CreateObject(info, desc, shape);
	}
	else if(desc.mShapes->mType==PINT_SHAPE_CONVEX)
	{
		const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(desc.mShapes);

		hkStridedVertices StridedVerts;
		StridedVerts.m_numVertices	= ConvexCreate->mNbVerts;
		StridedVerts.m_striding		= sizeof(Point);
		StridedVerts.m_vertices		= &ConvexCreate->mVerts->x;

		hkpConvexVerticesShape* shape = FindConvexShape(gShareShapes, mConvexShapes, *ConvexCreate, StridedVerts);

		hkpRigidBodyCinfo info;
		if(desc.mMass!=0.0f)
		{
			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeConvexHullMassProperties(StridedVerts, 0.0f, massProperties);

			info.m_mass				= massProperties.m_mass;
			info.m_centerOfMass		= massProperties.m_centerOfMass;
			info.m_inertiaTensor	= massProperties.m_inertiaTensor;
			info.m_motionType		= hkpMotion::MOTION_DYNAMIC;
		}
		FillRigidBodyInfo(info, desc, shape);

		if(ConvexCreate->mMaterial)
		{
			info.m_friction		= ConvexCreate->mMaterial->mDynamicFriction;
			info.m_restitution	= ConvexCreate->mMaterial->mRestitution;
		}
		else
		{
			info.m_friction		= 0.5f;
			info.m_restitution	= 0.0f;
		}

		return CreateObject(info, desc, shape);
	}
	else if(desc.mShapes->mType==PINT_SHAPE_MESH)
	{
		const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(desc.mShapes);

//		hkpShape* shape = CreateMeshShape(*MeshCreate, gMeshFormat, mMeshes, MeshCreate->mRenderer);

		// ### share meshes?

		hkpExtendedMeshShape* m_mesh = new hkpExtendedMeshShape;
//		hkpStorageExtendedMeshShape* m_mesh = new hkpStorageExtendedMeshShape;

		// it is common to have a landscape with 0 convex radius (for each triangle)
		// and all moving objects with non zero radius.
//		m_mesh->setRadius(0.0f);

		{
			hkpExtendedMeshShape::TrianglesSubpart part;

			part.m_vertexBase			= &MeshCreate->mSurface.mVerts->x;
			part.m_vertexStriding		= sizeof(Point);
			part.m_numVertices			= MeshCreate->mSurface.mNbVerts;

			part.m_indexBase			= MeshCreate->mSurface.mDFaces;
			part.m_indexStriding		= sizeof(udword)*3;
			part.m_numTriangleShapes	= MeshCreate->mSurface.mNbFaces;
			part.m_stridingType			= hkpExtendedMeshShape::INDICES_INT32;

			m_mesh->addTrianglesSubpart(part);
		}

		hkpMoppCompilerInput mci;
		hkpMoppCode* m_code = hkpMoppUtility::buildCode(m_mesh, mci);

		hkpMoppBvTreeShape* shape = new hkpMoppBvTreeShape(m_mesh, m_code);

		m_code->removeReference();
		m_mesh->removeReference();

//		hkResult res = m_mesh->computeWeldingInfo(shape, hkpWeldingUtility::WELDING_TYPE_CLOCKWISE);
		hkResult res = m_mesh->computeWeldingInfo(shape, hkpWeldingUtility::WELDING_TYPE_ANTICLOCKWISE);

		if(MeshCreate->mRenderer)
			shape->setUserData(hkUlong(MeshCreate->mRenderer));

		hkpRigidBodyCinfo info;
		if(desc.mMass!=0.0f)
		{
			ASSERT(0);
		}
		FillRigidBodyInfo(info, desc, shape);

		if(MeshCreate->mMaterial)
		{
			info.m_friction		= MeshCreate->mMaterial->mDynamicFriction;
			info.m_restitution	= MeshCreate->mMaterial->mRestitution;
		}
		else
		{
			info.m_friction		= 0.5f;
			info.m_restitution	= 0.0f;
		}

		return CreateObject(info, desc, shape);
	}
	else ASSERT(0);
	return null;
}

PintObjectHandle Havok::CreateCompoundObject(const PINT_OBJECT_CREATE& desc)
{
	// We'll basically have to create a 'List' Shape  (ie. a hkpListShape) in order to have many
	// shapes in the same body. Each element of the list will be a (transformed) hkpShape, ie.
	// a hkpTransformShape (which basically is a (geometry,transformation) pair).
	// The hkpListShape constructor needs a pointer to an array of hkShapes, so we create an array here, and push
	// back the hkTransformShapes as we create them.
	hkInplaceArray<hkpShape*, 32> shapeArray;
	hkInplaceArray<hkpShape*, 32> refs;

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
		ASSERT(!CurrentShape->mMaterial);

		hkTransform LocalPose;
		LocalPose.setTranslation(ToHkVector4(CurrentShape->mLocalPos));
		LocalPose.setRotation(ToHkQuaternion(CurrentShape->mLocalRot));

		if(CurrentShape->mType==PINT_SHAPE_SPHERE)
		{
			const PINT_SPHERE_CREATE* SphereCreate = static_cast<const PINT_SPHERE_CREATE*>(CurrentShape);

			hkpSphereShape* shape = FindSphereShape(gShareShapes, mSphereShapes, *SphereCreate);
			refs.pushBack(shape);

//			sphere->addReference();
			hkpConvexTransformShape* sphereTransform = new hkpConvexTransformShape(shape, LocalPose);
			ASSERT(sphereTransform);
			shapeArray.pushBack(sphereTransform);
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(CurrentShape);

//			const hkVector4 halfExtents(ToHkVector4(BoxCreate->mExtents));

			hkpBoxShape* shape = FindBoxShape(gShareShapes, mBoxShapes, *BoxCreate);
			refs.pushBack(shape);

//			sphere->addReference();
			hkpConvexTransformShape* boxTransform = new hkpConvexTransformShape(shape, LocalPose);
			ASSERT(boxTransform);
			shapeArray.pushBack(boxTransform);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CAPSULE)
		{
			const PINT_CAPSULE_CREATE* CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE*>(CurrentShape);

			hkpCapsuleShape* shape = FindCapsuleShape(gShareShapes, mCapsuleShapes, *CapsuleCreate);
			refs.pushBack(shape);

//			sphere->addReference();
			hkpConvexTransformShape* capsuleTransform = new hkpConvexTransformShape(shape, LocalPose);
			ASSERT(capsuleTransform);
			shapeArray.pushBack(capsuleTransform);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);

			hkStridedVertices StridedVerts;
			StridedVerts.m_numVertices	= ConvexCreate->mNbVerts;
			StridedVerts.m_striding		= sizeof(Point);
			StridedVerts.m_vertices		= &ConvexCreate->mVerts->x;

			hkpConvexVerticesShape* shape = FindConvexShape(gShareShapes, mConvexShapes, *ConvexCreate, StridedVerts);
			refs.pushBack(shape);

//			sphere->addReference();
			hkpConvexTransformShape* convexTransform = new hkpConvexTransformShape(shape, LocalPose);
			ASSERT(convexTransform);
			shapeArray.pushBack(convexTransform);
		}
		else ASSERT(0);

		CurrentShape = CurrentShape->mNext;
	}


	// Now we can create the compound body as a hkpListShape

	hkpListShape* listShape;
	{
		listShape = new hkpListShape(shapeArray.begin(), shapeArray.getSize());
		for(int i=0; i<refs.getSize(); ++i)
			refs[i]->removeReference();

		for(int i=0; i<shapeArray.getSize(); ++i)
			shapeArray[i]->removeReference();
	}

	// Create the rigid body
	hkpRigidBodyCinfo info;
	if(desc.mMass!=0.0f)
	{
		// #######
		hkpMassProperties massProperties;
		hkpInertiaTensorComputer::computeBoxVolumeMassProperties(hkVector4(1.0f, 1.0f, 1.0f), desc.mMass, massProperties);

		info.m_mass				= massProperties.m_mass;
		info.m_centerOfMass		= massProperties.m_centerOfMass;
		info.m_inertiaTensor	= massProperties.m_inertiaTensor;
		info.m_motionType		= hkpMotion::MOTION_DYNAMIC;
	}
	else
	{
		info.m_restitution		= 0.0f;
	}
	FillRigidBodyInfo(info, desc, listShape);

	return CreateObject(info, desc, listShape);
}

/*
#include <Common/Internal/ConvexHull/hkGeometryUtility.h>
hkpRigidBody* CreateRandomConvexGeometricFromBox(const hkVector4& size, const hkReal mass, const hkVector4& position, const int numVertices, BasicRandom& rnd)
{
	// generate a random convex geometry
	hkArray<hkVector4> vertices;
	{
		hkVector4 halfExtents; halfExtents.setMul4( 0.5f, size );
		vertices.reserve( numVertices );
		for( int i = 0; i < numVertices; i++ )
		{
			hkVector4 xyz;
			{
				xyz(0) = rnd.randomFloat();
				xyz(1) = rnd.randomFloat();
				xyz(2) = rnd.randomFloat();
				xyz(3) = 0.0f;
			}
			xyz.normalize3();
			xyz(0) *= halfExtents(0);
			xyz(1) *= halfExtents(1);
			xyz(2) *= halfExtents(2);

			vertices.pushBack( xyz );
		}
	}

	// convert it to a convex vertices shape
	hkpConvexVerticesShape* cvs = new hkpConvexVerticesShape(vertices);
	
	hkpRigidBodyCinfo convexInfo;

	convexInfo.m_shape = cvs;
	if(mass != 0.0f)
	{
		convexInfo.m_mass = mass;
		hkpInertiaTensorComputer::setShapeVolumeMassProperties(convexInfo.m_shape, convexInfo.m_mass, convexInfo);
		convexInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
	}
	else
	{
		convexInfo.m_motionType = hkpMotion::MOTION_FIXED;
	}

	convexInfo.m_rotation.setIdentity();
	convexInfo.m_position = position;

	hkpRigidBody* convexRigidBody = new hkpRigidBody(convexInfo);

	cvs->removeReference();

	return convexRigidBody;
}*/

PintJointHandle Havok::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mPhysicsWorld);

	hkpRigidBody* body0 = (hkpRigidBody*)desc.mObject0;
	hkpRigidBody* body1 = (hkpRigidBody*)desc.mObject1;

	hkpConstraintInstance* constraint = null;
	hkpConstraintData* data = null;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			if(1)
			{
				hkpBallAndSocketConstraintData* bsc = new hkpBallAndSocketConstraintData();
				bsc->setInBodySpace(ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1));

				constraint = new hkpConstraintInstance(body0, body1, bsc);
				data = bsc;
			}
			else
			{
/*				hkpBallSocketChainData* chainData = new hkpBallSocketChainData();
				hkpConstraintChainInstance* chainInstance = new hkpConstraintChainInstance( chainData );

				chainInstance->addEntity( entities[0] );
				for (int e = 1; e < entities.getSize(); e++)
				{
					chainData->addConstraintInfoInBodySpace( hkVector4(elemHalfSize, 0.0f, 0.0f), hkVector4( -elemHalfSize, 0.0f, 0.0f) );
					chainInstance->addEntity( entities[e] );
				}
				chainData->removeReference();

				m_world->addConstraint(chainInstance);
				chainInstance->removeReference();*/
			}
		}
		break;

		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			if(jc.mMinLimitAngle!=MIN_FLOAT || jc.mMaxLimitAngle!=MAX_FLOAT)
			{
				hkpLimitedHingeConstraintData* lhc = new hkpLimitedHingeConstraintData();

				if(!jc.mGlobalAnchor.IsNotUsed() && !jc.mGlobalAxis.IsNotUsed())
				{
					lhc->setInWorldSpace(	body0->getTransform(), body1->getTransform(),
											ToHkVector4(jc.mGlobalAnchor), ToHkVector4(jc.mGlobalAxis));
				}
				else
				{
					Point Right0, Up0;	ComputeBasis(jc.mLocalAxis0, Right0, Up0);
					Point Right1, Up1;	ComputeBasis(jc.mLocalAxis1, Right1, Up1);

					lhc->setInBodySpace(ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1),
										ToHkVector4(jc.mLocalAxis0), ToHkVector4(jc.mLocalAxis1),
										ToHkVector4(Right0), ToHkVector4(Right1));
				}
				lhc->setMinAngularLimit(jc.mMinLimitAngle);
				lhc->setMaxAngularLimit(jc.mMaxLimitAngle);

				constraint = new hkpConstraintInstance(body0, body1, lhc);
				data = lhc;
			}
			else
			{
				hkpHingeConstraintData* hc = new hkpHingeConstraintData();

				if(!jc.mGlobalAnchor.IsNotUsed() && !jc.mGlobalAxis.IsNotUsed())
				{
					hc->setInWorldSpace(body0->getTransform(), body1->getTransform(),
										ToHkVector4(jc.mGlobalAnchor), ToHkVector4(jc.mGlobalAxis));
				}
				else
				{
					hc->setInBodySpace(ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1), ToHkVector4(jc.mLocalAxis0), ToHkVector4(jc.mLocalAxis1));
				}

				constraint = new hkpConstraintInstance(body0, body1, hc);
				data = hc;
			}
		}
		break;

		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			if(0)	// Emulating fixed joint with limited hinge
			{
				hkpLimitedHingeConstraintData* lhc = new hkpLimitedHingeConstraintData();

				const Point LocalAxis(1,0,0);
				const Point PerpAxis(0,1,0);
				lhc->setInBodySpace(ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1), ToHkVector4(LocalAxis), ToHkVector4(LocalAxis), ToHkVector4(PerpAxis), ToHkVector4(PerpAxis));
				lhc->setMinAngularLimit(0.0f);
				lhc->setMaxAngularLimit(0.0f);
				
				constraint = new hkpConstraintInstance(body0, body1, lhc);
				data = lhc;
			}

			if(1)	// Emulating fixed joint with prismatic
			{
				hkpPrismaticConstraintData* pris = new hkpPrismaticConstraintData(); 

				const Point LocalAxis(1,0,0);
				const Point PerpAxis(0,1,0);
				pris->setInBodySpace(ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1), ToHkVector4(LocalAxis), ToHkVector4(LocalAxis), ToHkVector4(PerpAxis), ToHkVector4(PerpAxis));
				pris->setMaxLinearLimit(0.0f);
				pris->setMinLinearLimit(0.0f);
				
				constraint = new hkpConstraintInstance(body0, body1, pris);
				data = pris;
			}
		}
		break;

		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			hkpPrismaticConstraintData* pris = new hkpPrismaticConstraintData(); 

			Point Right0, Up0;	ComputeBasis(jc.mLocalAxis0, Right0, Up0);
			Point Right1, Up1;	ComputeBasis(jc.mLocalAxis1, Right1, Up1);

			pris->setInBodySpace(	ToHkVector4(jc.mLocalPivot0), ToHkVector4(jc.mLocalPivot1),
									ToHkVector4(jc.mLocalAxis0), ToHkVector4(jc.mLocalAxis1),
									ToHkVector4(Right0), ToHkVector4(Right1));
//			pris->setMaxLinearLimit(10.0f);
//			pris->setMinLinearLimit(-10.0f);
			
			constraint = new hkpConstraintInstance(body0, body1, pris);
			data = pris;
		}
		break;
	}

	if(constraint)
	{
		mPhysicsWorld->addConstraint(constraint); 	
		constraint->removeReference();
	}
	if(data)
		data->removeReference();

	return constraint;
}

void* Havok::CreatePhantom(const AABB& box)
{
	return null;
}

udword Havok::BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**)
{
	return 0;
}

void Havok::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	hkpGroupFilter* filter = new hkpGroupFilter();
	for(udword i=0;i<nb_groups;i++)
		filter->disableCollisionsBetween(groups[i].mGroup0, groups[i].mGroup1);
	mPhysicsWorld->setCollisionFilter(filter);
	filter->removeReference();
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const hkpWorldRayCastOutput& result, const Point& origin, const Point& dir, float max_dist)
{
	const hkpCollidable* touched = result.m_rootCollidable;
	hit.mObject			= (hkpRigidBody*)touched->getOwner();	// ###
	hit.mImpact			= origin + dir * result.m_hitFraction * max_dist;
	hit.mNormal			= ToPoint(result.m_normal);
//	hit.mDistance		= origin.Distance(hit.mImpact);
	hit.mDistance		= result.m_hitFraction * max_dist;
	hit.mTriangleIndex	= INVALID_ID;
}

udword Havok::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mPhysicsWorld);

	hkpWorldRayCastOutput output;
	udword NbHits = 0;
	while(nb--)
	{
		hkpWorldRayCastInput input;
		input.m_from	= ToHkVector4(raycasts->mOrigin);
		input.m_to		= ToHkVector4(raycasts->mOrigin + raycasts->mDir*raycasts->mMaxDist);

		output.reset();
		mPhysicsWorld->castRay(input, output);

		if(output.m_rootCollidable)
		{
			NbHits++;
			FillResultStruct(*dest, output, raycasts->mOrigin, raycasts->mDir, raycasts->mMaxDist);
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

void Havok::PrintMemoryStatistics()
{
	hkOstream hkcout("d:\\HavokMemStats.txt");
	hkMemory::getInstance().printStatistics(&hkcout);
}




#include <Physics/Dynamics/Constraint/Chain/hkpConstraintChainInstance.h>
#include <Physics/Dynamics/Constraint/Chain/BallSocket/hkpBallSocketChainData.h>
#include <Physics/Dynamics/Constraint/Chain/StiffSpring/hkpStiffSpringChainData.h>
#include <Physics/Dynamics/Constraint/Chain/Powered/hkpPoweredChainData.h>

namespace
{
	enum ChainType
	{
		BALL_AND_SOCKET,
		STIFF_SPRING
	};
}

struct SwingingRopeVariant
{
	const char*	 m_name;
	ChainType m_type;
	hkInt32 m_numBodies;
	hkReal m_tau;
	hkReal m_gravity;
	const char* m_details;
};


static const SwingingRopeVariant g_variants[] =
{
	{ "Ball and Socket chain. Length==3",                  BALL_AND_SOCKET,   3, 0.6f, 9.81f,	"" },
	{ "Ball and Socket chain. Length==20",                 BALL_AND_SOCKET,  20, 0.6f, 9.81f,	"" },
	{ "Ball and Socket chain. Length==50",                 BALL_AND_SOCKET,  50, 0.6f, 9.81f,	"" },
	{ "Ball and Socket chain. Length==100",                BALL_AND_SOCKET, 100, 0.6f, 9.81f,	"" },
	{ "Ball and Socket chain. Length==100, triple gravity",BALL_AND_SOCKET, 100, 0.6f, 30.0f,	"" },
	{ "Ball and Socket chain. Length==200, tau==0.60",     BALL_AND_SOCKET, 200, 0.6f, 9.81f,	"" },
	{ "Ball and Socket chain. Length==200, tau==0.02",     BALL_AND_SOCKET, 200, 0.02f, 9.81f,	"" },
	{ "Stiff spring chain.    Length==100, tau==0.60",     STIFF_SPRING,    100, 0.6f, 9.81f,	"" },
	{ "Stiff spring chain.    Length==100, tau==0.10",     STIFF_SPRING,    100, 0.1f, 9.81f,	"" }
};

void Havok::TestNewFeature()
{
	const float Altitude = 100.0f;

	const SwingingRopeVariant& variant = g_variants[2];
//	const SwingingRopeVariant& variant = g_variants[7];
//	const SwingingRopeVariant& variant = g_variants[1];
//	const SwingingRopeVariant& variant = g_variants[3];

		// Horizontal distance between the attachment points of the strings.
	const hkReal offset = 50.0f;

	//
	// Create the world
	//
	{
/*		hkpWorldCinfo info;
		info.setBroadPhaseWorldSize( 100000.0f );
		info.m_gravity.set(0.0f, 0.0f, -variant.m_gravity);
		info.m_solverTau = variant.m_tau;
		mPhysicsWorld = new hkpWorld( info );*/
		mPhysicsWorld->lock();

//		mPhysicsWorld->m_wantDeactivation = false;
	}


	{
		hkpRigidBodyCinfo info;
		info.m_shape = new hkpBoxShape( hkVector4(0.5, 0.5f, 0.5f), 0.01f );
		hkpInertiaTensorComputer::setShapeVolumeMassProperties(info.m_shape, 10.0f, info);
		info.m_mass = 1.0f;
//		info.m_linearVelocity(2) = -10.0f;


		//
		// Construct string of independent bilateral constraints
		//
		{
			hkpConstraintData* data;
			{
				hkpBallAndSocketConstraintData* bsData = new hkpBallAndSocketConstraintData();
//				bsData->setInBodySpace(hkVector4::getZero(), hkVector4( -1.5f, 0.0f, -0.3f));
				bsData->setInBodySpace(hkVector4::getZero(), hkVector4( -1.5f, -0.3f, 0.0f));
				data = bsData;
			}

			hkpRigidBody* prevBody = HK_NULL;
			for (int b = 0; b < variant.m_numBodies; b++)
			{
				info.m_position.set(1.5f * hkReal(b) - offset / 2.0f, Altitude + 0.3f * hkReal(b), 0.0f);
				info.m_motionType = b ? hkpMotion::MOTION_DYNAMIC : hkpMotion::MOTION_FIXED;

				hkpRigidBody* body = new hkpRigidBody(info);
				mPhysicsWorld->addEntity(body);
//				HK_SET_OBJECT_COLOR( hkUlong(body->getCollidable()), 0xffff0000 );

				mRigidBodies.Add(udword(body));

				if (prevBody)
				{
					// add constraint
					hkpConstraintInstance* instance = new hkpConstraintInstance(prevBody, body, data);
					mPhysicsWorld->addConstraint( instance );
					instance->removeReference();
				}

				prevBody = body;
				// we remove reference, but we know one is still kept by mPhysicsWorld
				body->removeReference();
			}
			data->removeReference();
		}


		//
		// Construct constraint chain
		//
		{
			hkArray<hkpEntity*> entities;

			for (int b = 0; b < variant.m_numBodies; b++)
			{
				info.m_position.set(1.5f * hkReal(b) + offset / 2.0f, Altitude + 0.3f * hkReal(b), 2.0f);
				info.m_motionType = b ? hkpMotion::MOTION_DYNAMIC : hkpMotion::MOTION_FIXED;

				hkpRigidBody* body = new hkpRigidBody(info);
				mPhysicsWorld->addEntity(body);
//				HK_SET_OBJECT_COLOR( hkUlong(body->getCollidable()), 0xff00ff00 );

				mRigidBodies.Add(udword(body));

				entities.pushBack(body);
				// we know, a reference is kept by the world
				body->removeReference();
			}
	
			{
				hkpConstraintChainInstance* chainInstance = HK_NULL;

				if (variant.m_type == BALL_AND_SOCKET)
				{
					hkpBallSocketChainData* chainData = new hkpBallSocketChainData();
					chainInstance = new hkpConstraintChainInstance( chainData );

					chainInstance->addEntity( entities[0] );
					for (int e = 1; e < entities.getSize(); e++)
					{
//						chainData->addConstraintInfoInBodySpace( hkVector4::getZero(), hkVector4( -1.5f, 0.0f, -0.3f) );
						chainData->addConstraintInfoInBodySpace( hkVector4::getZero(), hkVector4( -1.5f, -0.3f, 0.0f) );
						chainInstance->addEntity( entities[e] );
					}

					chainData->m_tau = variant.m_tau;
					chainData->removeReference();
				}
				else if(variant.m_type == STIFF_SPRING)
				{
					hkpStiffSpringChainData* chainData = new hkpStiffSpringChainData();
					chainInstance = new hkpConstraintChainInstance( chainData );

					chainInstance->addEntity( entities[0] );
					for (int e = 1; e < entities.getSize(); e++)
					{
//						chainData->addConstraintInfoInBodySpace( hkVector4::getZero(), hkVector4( -1.0f, 0.0f, -0.2f), 0.55f );
						chainData->addConstraintInfoInBodySpace( hkVector4::getZero(), hkVector4( -1.0f, -0.2f, 0.0f), 0.55f );
						chainInstance->addEntity( entities[e] );
					}

					chainData->m_tau = variant.m_tau;
					chainData->removeReference();
				}

				mPhysicsWorld->addConstraint( chainInstance );
				chainInstance->removeReference();
			}

		}

		info.m_shape->removeReference();

	}

	mPhysicsWorld->unlock();

}

/*

#include <Demos/demos.h>
#include <Demos/Physics/Api/Constraints/Chain/SwingingRope/SwingingRopeDemo.h>
#include <Demos/DemoCommon/Utilities/GameUtils/GameUtils.h>

#include <Physics/Dynamics/Constraint/ConstraintKit/hkpGenericConstraintData.h>

// We will need these collision agents
#include <Physics/Collide/Agent/ConvexAgent/BoxBox/hkpBoxBoxAgent.h>


#include <Physics/Utilities/VisualDebugger/Viewer/Dynamics/hkpConstraintViewer.h>
#include <Physics/Utilities/VisualDebugger/Viewer/Dynamics/hkpRigidBodyCentreOfMassViewer.h>

#include <Common/Visualize/hkDebugDisplay.h>

#include <Physics/Dynamics/Constraint/Motor/Position/hkpPositionConstraintMotor.h>

#include <Physics/Utilities/Constraint/Bilateral/hkpConstraintUtils.h>
*/

////////////////////////////////////////////////////////////////////////////////

static Havok* gHavok = null;
static void gHavok_GetOptionsFromGUI();

Havok* GetHavok()
{
	return gHavok;
}

void Havok_Close()
{
	if(gHavok)
	{
		gHavok->Close();
		delete gHavok;
		gHavok = null;
	}
}

void Havok_Init(const PINT_WORLD_CREATE& desc)
{
	gHavok_GetOptionsFromGUI();

	ASSERT(!gHavok);
	gHavok = ICE_NEW(Havok);
	gHavok->Init(desc);
}

/*
static void createBrickWall( hkpWorld* world, int height, int length, const hkVector4& position, hkReal gapWidth, hkpConvexShape* box, hkVector4Parameter halfExtents )
{
	hkVector4 posx = position;
	// do a raycast to place the wall
	{
		hkpWorldRayCastInput ray;
		ray.m_from = posx;
		ray.m_to = posx;

		ray.m_from(1) += 20.0f;
		ray.m_to(1)   -= 20.0f;

		hkpWorldRayCastOutput result;
		world->castRay( ray, result );
		posx.setInterpolate4( ray.m_from, ray.m_to, result.m_hitFraction );
	}
	// move the start point
	posx(0) -= ( gapWidth + 2.0f * halfExtents(0) ) * length * 0.5f;
	posx(1) -= halfExtents(1) + box->getRadius();

	hkArray<hkpEntity*> entitiesToAdd;

	for ( int x = 0; x < length; x ++ )		// along the ground
	{
		hkVector4 pos = posx;
		for( int ii = 0; ii < height; ii++ )
		{
			pos(1) += (halfExtents(1) + box->getRadius())* 2.0f;

			hkpRigidBodyCinfo boxInfo;
			boxInfo.m_mass = 10.0f;
			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, boxInfo.m_mass, massProperties);

			boxInfo.m_mass = massProperties.m_mass;
			boxInfo.m_centerOfMass = massProperties.m_centerOfMass;
			boxInfo.m_inertiaTensor = massProperties.m_inertiaTensor;
			boxInfo.m_solverDeactivation = boxInfo.SOLVER_DEACTIVATION_MEDIUM;
			boxInfo.m_shape = box;
			//boxInfo.m_qualityType = HK_COLLIDABLE_QUALITY_DEBRIS;
			boxInfo.m_restitution = 0.0f;

			boxInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;

			{
				boxInfo.m_position = pos;
				hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
				world->addEntity( boxRigidBody );
				boxRigidBody->removeReference();
			}

			pos(1) += (halfExtents(1) + box->getRadius())* 2.0f;
			pos(0) += halfExtents(0) * 0.6f;
			{
				boxInfo.m_position = pos;
				hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
				entitiesToAdd.pushBack(boxRigidBody);
			}
			pos(0) -= halfExtents(0) * 0.6f;
		}
		posx(0) += halfExtents(0)* 2.0f + gapWidth;
	}
	world->addEntityBatch( entitiesToAdd.begin(), entitiesToAdd.getSize());

	for (int i=0; i < entitiesToAdd.getSize(); i++){ entitiesToAdd[i]->removeReference(); }
}

static void setupPhysics(hkpWorld* physicsWorld)
{
	//
	//  Create the ground box
	//
	{
		hkVector4 groundRadii(70.0f, 2.0f, 140.0f);
		hkpConvexShape* shape = new hkpBoxShape(groundRadii , 0);

		hkpRigidBodyCinfo ci;
		ci.m_shape			= shape;
		ci.m_motionType		= hkpMotion::MOTION_FIXED;
		ci.m_position		= hkVector4( 0.0f, -2.0f, 0.0f );
		ci.m_qualityType	= HK_COLLIDABLE_QUALITY_FIXED;

		physicsWorld->addEntity(new hkpRigidBody(ci))->removeReference();
		shape->removeReference();
	}

	hkVector4 groundPos( 0.0f, 0.0f, 0.0f );
	hkVector4 posy = groundPos;

	//
	// Create the walls
	//

	int wallHeight = 8;
	int wallWidth  = 8;
	int numWalls = 6;
	hkVector4 boxSize( 1.0f, 0.5f, 0.5f);
	hkpBoxShape* box = new hkpBoxShape( boxSize , 0 );
	box->setRadius( 0.0f );

	hkReal deltaZ = 25.0f;
	posy(2) = -deltaZ * numWalls * 0.5f;

	for ( int y = 0; y < numWalls; y ++ )			// first wall
	{
		createBrickWall( physicsWorld, wallHeight, wallWidth, posy, 0.2f, box, boxSize );
		posy(2) += deltaZ;
	}
	box->removeReference();

	//
	// Create a ball moving towards the walls
	//

	const hkReal radius = 1.5f;
	const hkReal sphereMass = 150.0f;

	hkVector4 relPos( 0.0f,radius + 0.0f, 50.0f );

	hkpRigidBodyCinfo info;
	hkpMassProperties massProperties;
	hkpInertiaTensorComputer::computeSphereVolumeMassProperties(radius, sphereMass, massProperties);

	info.m_mass = massProperties.m_mass;
	info.m_centerOfMass  = massProperties.m_centerOfMass;
	info.m_inertiaTensor = massProperties.m_inertiaTensor;
	info.m_shape = new hkpSphereShape( radius );
	info.m_position.setAdd4(posy, relPos );
	info.m_motionType  = hkpMotion::MOTION_BOX_INERTIA;

	info.m_qualityType = HK_COLLIDABLE_QUALITY_BULLET;


	hkpRigidBody* sphereRigidBody = new hkpRigidBody( info );
	g_ball = sphereRigidBody;

	physicsWorld->addEntity( sphereRigidBody );
	sphereRigidBody->removeReference();
	info.m_shape->removeReference();

	hkVector4 vel(  0.0f,4.9f, -100.0f );
	sphereRigidBody->setLinearVelocity( vel );
}
*/

///////////////////////////////////////////////////////////////////////////////

static Container*	gHavokGUI = null;
static IceComboBox*	gComboBox_NbThreads = null;
static IceComboBox*	gComboBox_SolverDeactivation = null;
static IceComboBox*	gComboBox_ContactPointGeneration = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_GlobalBoxSize = null;

enum HavokGUIElement
{
	HAVOK_GUI_MAIN,
	//
	HAVOK_GUI_ENABLE_SLEEPING,
	HAVOK_GUI_ENABLE_SQ,
	HAVOK_GUI_ENABLE_CCD,
	HAVOK_GUI_SHARE_SHAPES,
	HAVOK_GUI_USE_VDB,
	//
	HAVOK_GUI_NB_THREADS,
	HAVOK_GUI_SOLVER_DEACTIVATION,
	HAVOK_GUI_CONTACT_POINT_GENERATION,
	//
	HAVOK_GUI_SOLVER_ITER,
	HAVOK_GUI_LINEAR_DAMPING,
	HAVOK_GUI_ANGULAR_DAMPING,
	HAVOK_GUI_GLOBAL_BOX_SIZE,
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	switch(check_box.GetID())
	{
		case HAVOK_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case HAVOK_GUI_ENABLE_SQ:
			gEnableKDTree = checked;
			break;
		case HAVOK_GUI_ENABLE_CCD:
			gUseCCD = checked;
			break;
		case HAVOK_GUI_SHARE_SHAPES:
			gShareShapes = checked;
			break;
		case HAVOK_GUI_USE_VDB:
			gUseVDB = checked;
			break;
	}
}

static udword gSolverDeactivationToIndex[] = { 0, 0, 1, 2, 3, 4 };
static hkpRigidBodyCinfo::SolverDeactivation gIndexToSolverDeactivation[] = {
												hkpRigidBodyCinfo::SOLVER_DEACTIVATION_OFF,
												hkpRigidBodyCinfo::SOLVER_DEACTIVATION_LOW,
												hkpRigidBodyCinfo::SOLVER_DEACTIVATION_MEDIUM,
												hkpRigidBodyCinfo::SOLVER_DEACTIVATION_HIGH,
												hkpRigidBodyCinfo::SOLVER_DEACTIVATION_MAX	};

static udword gNbThreadsToIndex[] = { 0, 0, 1, 2, 3 };
static udword gIndexToNbThreads[] = { 0, 2, 3, 4 };

static void gHavok_GetOptionsFromGUI()
{
	if(gComboBox_NbThreads)
	{
		const udword Index = gComboBox_NbThreads->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToNbThreads)/sizeof(gIndexToNbThreads[0]));
		gNbThreads = gIndexToNbThreads[Index];
	}

	if(gComboBox_SolverDeactivation)
	{
		const udword Index = gComboBox_SolverDeactivation->GetSelectedIndex();
		ASSERT(Index<sizeof(gIndexToSolverDeactivation)/sizeof(gIndexToSolverDeactivation[0]));
		gSolverDeactivation = gIndexToSolverDeactivation[Index];
	}

	if(gComboBox_ContactPointGeneration)
	{
		const udword Index = gComboBox_ContactPointGeneration->GetSelectedIndex();
		gContactPointGeneration = hkpWorldCinfo::ContactPointGeneration(Index);
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

IceWindow* Havok_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gHavokGUI, parent, HAVOK_GUI_MAIN, "Havok 6.6.0 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, HAVOK_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gHavokGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, HAVOK_GUI_ENABLE_SQ, 4, y, CheckBoxWidth, 20, "Enable scene queries (KD tree)", gHavokGUI, gEnableKDTree, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, HAVOK_GUI_ENABLE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gHavokGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, HAVOK_GUI_SHARE_SHAPES, 4, y, CheckBoxWidth, 20, "Share shapes", gHavokGUI, gShareShapes, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, HAVOK_GUI_USE_VDB, 4, y, CheckBoxWidth, 20, "Use VDB", gHavokGUI, gUseVDB, gCheckBoxCallback);
		y += YStepCB;
	}

	y += YStep;

	const sdword LabelOffsetY = 2;
	const sdword OffsetX = 90;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Solver deact.:", gHavokGUI);
		ComboBoxDesc CBBD;
		CBBD.mID		= HAVOK_GUI_SOLVER_DEACTIVATION;
		CBBD.mParent	= Main;
		CBBD.mX			= 4+OffsetX;
		CBBD.mY			= y;
		CBBD.mWidth		= 200;
		CBBD.mHeight	= 20;
		CBBD.mLabel		= "Solver deactivation";
		gComboBox_SolverDeactivation = ICE_NEW(IceComboBox)(CBBD);
		gHavokGUI->Add(udword(gComboBox_SolverDeactivation));
		gComboBox_SolverDeactivation->Add("SOLVER_DEACTIVATION_OFF");
		gComboBox_SolverDeactivation->Add("SOLVER_DEACTIVATION_LOW");
		gComboBox_SolverDeactivation->Add("SOLVER_DEACTIVATION_MEDIUM");
		gComboBox_SolverDeactivation->Add("SOLVER_DEACTIVATION_HIGH");
		gComboBox_SolverDeactivation->Add("SOLVER_DEACTIVATION_MAX");
		ASSERT(gSolverDeactivation<sizeof(gSolverDeactivationToIndex)/sizeof(gSolverDeactivationToIndex[0]));
		gComboBox_SolverDeactivation->Select(gSolverDeactivationToIndex[gSolverDeactivation]);
		gComboBox_SolverDeactivation->SetVisible(true);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Ctc pt gen:", gHavokGUI);
		CBBD.mID		= HAVOK_GUI_CONTACT_POINT_GENERATION;
		CBBD.mY			= y;
		CBBD.mLabel		= "Contact point generation";
		gComboBox_ContactPointGeneration = ICE_NEW(IceComboBox)(CBBD);
		gHavokGUI->Add(udword(gComboBox_ContactPointGeneration));
		gComboBox_ContactPointGeneration->Add("CONTACT_POINT_ACCEPT_ALWAYS");
		gComboBox_ContactPointGeneration->Add("CONTACT_POINT_REJECT_DUBIOUS");
		gComboBox_ContactPointGeneration->Add("CONTACT_POINT_REJECT_MANY");
		gComboBox_ContactPointGeneration->Select(gContactPointGeneration);
		gComboBox_ContactPointGeneration->SetVisible(true);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num threads:", gHavokGUI);
		CBBD.mID		= HAVOK_GUI_NB_THREADS;
		CBBD.mY			= y;
		CBBD.mLabel		= "Num threads";
		gComboBox_NbThreads = ICE_NEW(IceComboBox)(CBBD);
		gHavokGUI->Add(udword(gComboBox_NbThreads));
		gComboBox_NbThreads->Add("Single threaded");
		gComboBox_NbThreads->Add("2 threads");
		gComboBox_NbThreads->Add("3 threads");
		gComboBox_NbThreads->Add("4 threads");
		ASSERT(gNbThreads<sizeof(gNbThreadsToIndex)/sizeof(gNbThreadsToIndex[0]));
		gComboBox_NbThreads->Select(gNbThreadsToIndex[gNbThreads]);
		gComboBox_NbThreads->SetVisible(true);
		y += YStep;
	}

	y += YStep;

	const sdword EditBoxWidth = 60;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num solver iter:", gHavokGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, HAVOK_GUI_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gHavokGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gHavokGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, HAVOK_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gHavokGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gHavokGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, HAVOK_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gHavokGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "World bounds size:", gHavokGUI);
		gEditBox_GlobalBoxSize = helper.CreateEditBox(Main, HAVOK_GUI_GLOBAL_BOX_SIZE, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gGlobalBoxSize), gHavokGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;
	}

	y += YStep;
	return Main;
}

void Havok_CloseGUI()
{
	Common_CloseGUI(gHavokGUI);

	gComboBox_NbThreads = null;
	gComboBox_SolverDeactivation = null;
	gComboBox_ContactPointGeneration = null;
	gEditBox_SolverIter = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	gEditBox_GlobalBoxSize = null;
}

///////////////////////////////////////////////////////////////////////////////

class HavokPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Havok_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Havok_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Havok_Init(desc);						}
	virtual	void		Close()												{ Havok_Close();						}
	virtual	Pint*		GetPint()											{ return GetHavok();					}
};
static HavokPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
