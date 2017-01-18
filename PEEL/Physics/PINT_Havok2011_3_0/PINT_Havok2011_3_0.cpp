///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "..\PINT_Common\PINT_Common.h"
#include "..\PINT_Common\PINT_CommonHavok.h"

///////////////////////////////////////////////////////////////////////////////

static void HK_CALL errorReport(const char* msg, void*)
{
	printf("HAVOK message: %s", msg);
}

Havok::Havok(const EditableParams& params) :
	SharedHavok		(params),
	mThreadPool		(null)
{
}

Havok::~Havok()
{
	ASSERT(!mThreadPool);
}

void Havok::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportCylinders				= true;
	caps.mSupportMassForInertia			= true;
	caps.mSupportKinematics				= true;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= true;
	caps.mSupportMeshes					= true;
#ifdef SUPPORT_HAVOK_ARTICULATION
//	caps.mSupportArticulations			= true;
	caps.mSupportArticulations			= false;	// Currently disabled because support is incomplete and it crashes in some scenes
#endif
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
}

#ifdef REMOVED
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
#endif

void Havok::Init(const PINT_WORLD_CREATE& desc)
{
/*	// Initialize the base system including our memory system
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
	}*/


//	mThreadMemory = new hkThreadMemory(memoryManager);
	mMemoryRouter = hkMemoryInitUtil::initDefault( hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(mParams.mSolverBufferSize * 1024));

//	hkBaseSystem::init(memoryManager, mThreadMemory, errorReport);
	hkBaseSystem::init( mMemoryRouter, errorReport );

//	memoryManager->removeReference();

	// We now initialize the stack area to 100k (fast temporary memory to be used by the engine).
/*	if(1)
	{
		const int stackSize = 0x100000;
//		const int stackSize = 32000;
		mStackBuffer = hkAllocate<char>(stackSize, HK_MEMORY_CLASS_BASE);
		hkThreadMemory::getInstance().setStackArea(mStackBuffer, stackSize);
	}*/

	if(mParams.mNbThreads>=2)
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
		int totalNumThreadsUsed = mParams.mNbThreads;

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
/*	{
		// The world cinfo contains global simulation parameters, including gravity, solver settings etc.
		hkpWorldCinfo worldInfo;

		if(desc.mGlobalBounds.IsValid())
		{
			Point m, M;
			desc.mGlobalBounds.GetMin(m);
			desc.mGlobalBounds.GetMax(M);

			worldInfo.m_broadPhaseWorldAabb = hkAabb(ToHkVector4(m), ToHkVector4(M));
		}
		else
		{
			const float Size = mParams.mGlobalBoxSize;
			worldInfo.m_broadPhaseWorldAabb = hkAabb(hkVector4(-Size, -Size, -Size), hkVector4(Size, Size, Size));
		}
		worldInfo.m_broadPhaseType = mParams.mBroadPhaseType;
		worldInfo.m_contactPointGeneration = mParams.mContactPointGeneration;

		worldInfo.m_gravity = ToHkVector4(desc.mGravity);

		worldInfo.m_solverIterations = mParams.mSolverIterationCount;

		// Set the simulation type of the world to multi-threaded.
		if(mParams.mNbThreads>=2)
		{
			worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;
		}
		else
		{
			if(mParams.mUseCCD)
			{
				worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_CONTINUOUS;
				worldInfo.m_sizeOfToiEventQueue = 1024;
			}
			else
				worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_DISCRETE;
		}

		// Flag objects that fall "out of the world" to be automatically removed - just necessary for this physics scene
		worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_REMOVE_ENTITY;

		worldInfo.m_enableDeactivation = mParams.mEnableSleeping;

		worldInfo.m_collisionTolerance = mParams.mCollisionTolerance;

		mPhysicsWorld = new hkpWorld(worldInfo);

		// Disable deactivation, so that you can view timers in the VDB. This should not be done in your game.
		mPhysicsWorld->m_wantDeactivation = mParams.mEnableSleeping;


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
	}*/
	mPhysicsWorld = SetupWorld(desc, mJobQueue);

	if(mParams.mUseVDB)
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

	{
		mPhysicsWorld->lock();
		const udword Size = mPhantoms.GetNbEntries();
		for(udword i=0;i<Size;i++)
		{
			hkpAabbPhantom* phantom = (hkpAabbPhantom*)mPhantoms[i];
			phantom->removeReference();
		}
		mPhysicsWorld->unlock();
	}

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
//	mThreadMemory->setStackArea(0, 0);
	hkDeallocate(mStackBuffer);
	mStackBuffer = null;

//	mThreadMemory->removeReference();
//	mThreadMemory = null;
	mMemoryRouter = null;

	// Quit base system
	hkBaseSystem::quit();

//	DELETESINGLE(gMemoryManager);
//	gMemoryManager = null;
	hkMemoryInitUtil::quit();
}

udword Havok::Update(float timestep)
{
	// <PHYSICS-ONLY>:
	// Step the physics world. This single call steps using this thread and all threads
	// in the threadPool. For other products you add jobs, call process all jobs and wait for completion.
	// See the multithreading chapter in the user guide for details
	if(mParams.mNbThreads>=2)
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

//	if(gMemoryManager)
//		return gMemoryManager->mCurrentMemory;

/*	if(mMemoryRouter)
	{
		hkMemoryAllocator::MemoryStatistics Stats;

		udword TotalSize = 0;

//		TotalSize += mMemoryRouter->stack().getAllocatedSize();

		mMemoryRouter->temp().getMemoryStatistics(Stats);
		TotalSize += Stats.m_allocated;
		Stats.m_allocated = 0;

		mMemoryRouter->heap().getMemoryStatistics(Stats);
		TotalSize += Stats.m_allocated;
		Stats.m_allocated = 0;

		mMemoryRouter->debug().getMemoryStatistics(Stats);
		TotalSize += Stats.m_allocated;
		Stats.m_allocated = 0;

		mMemoryRouter->solver().getMemoryStatistics(Stats);
		TotalSize += Stats.m_allocated;
		Stats.m_allocated = 0;

		return TotalSize;
	}*/

	return Havok_GetAllocatedMemory();
}

Point Havok::GetMainColor()
{
	return HAVOK_MAIN_COLOR;
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

void* Havok::CreatePhantom(const AABB& box)
{
	Point Min, Max;
	box.GetMin(Min);
	box.GetMax(Max);

	hkAabb info;
	info.m_min	= ToHkVector4(Min);
	info.m_max	= ToHkVector4(Max);
	hkpAabbPhantom* phantom = new hkpAabbPhantom(info);

	mPhysicsWorld->lock();
	mPhysicsWorld->addPhantom(phantom);
	mPhysicsWorld->unlock();

	mPhantoms.Add(udword(phantom));

	return phantom;
}

udword Havok::BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void** phantoms)
{
	ASSERT(phantoms);
	hkpAabbPhantom** ph = (hkpAabbPhantom**)phantoms;

	udword NbHits = 0;
	while(nb--)
	{
		hkpWorldRayCastInput input;
		input.m_from	= ToHkVector4(raycasts->mOrigin);
		input.m_to		= ToHkVector4(raycasts->mOrigin + raycasts->mDir*raycasts->mMaxDist);

		hkpAabbPhantom* phantom = *ph++;

		hkpWorldRayCastOutput output;
		phantom->castRay(input, output);

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

void Havok::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	hkpGroupFilter* filter = new hkpGroupFilter();
	for(udword i=0;i<nb_groups;i++)
		filter->disableCollisionsBetween(groups[i].mGroup0, groups[i].mGroup1);
	mPhysicsWorld->setCollisionFilter(filter);
	filter->removeReference();
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
//	hkOstream hkcout("d:\\HavokMemStats.txt");
//	hkMemory::getInstance().printStatistics(&hkcout);
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

	const float Radius = 1.0f;
	{
		hkpRigidBodyCinfo info;
		info.m_shape = new hkpSphereShape(Radius);
//		info.m_shape = new hkpBoxShape( hkVector4(0.5, 0.5f, 0.5f), 0.01f );
//		hkpInertiaTensorComputer::setShapeVolumeMassProperties(info.m_shape, 10.0f, info);
		hkpInertiaTensorComputer::setShapeVolumeMassProperties(info.m_shape, 1.0f, info);
		info.m_mass = 1.0f;
//		info.m_linearVelocity(2) = -10.0f;


		//
		// Construct string of independent bilateral constraints
		//
		if(0)
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
//				info.m_position.set(1.5f * hkReal(b) + offset / 2.0f, Altitude + 0.3f * hkReal(b), 2.0f);
				info.m_position.set(float(b)*(Radius+0.5f)*2.0f, 0.0f, 0.0f);
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
//						chainData->addConstraintInfoInBodySpace( hkVector4::getZero(), hkVector4( -1.5f, -0.3f, 0.0f) );
						chainData->addConstraintInfoInBodySpace( hkVector4( Radius+0.5f, 0.0f, 0.0f), hkVector4( -Radius-0.5f, 0.0f, 0.0f) );
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
//static void gHavok_GetOptionsFromGUI();

class MyUICallback : public UICallback
{
	public:
	virtual	void			UIModificationCallback()
	{
//		if(gHavok)
//			gHavok->UpdateFromUI();
	}

}gUICallback;

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
//	gHavok_GetOptionsFromGUI();
	Havok_::GetOptionsFromGUI();

	ASSERT(!gHavok);
	gHavok = ICE_NEW(Havok)(Havok_::GetEditableParams());
	gHavok->Init(desc);
}

///////////////////////////////////////////////////////////////////////////////

IceWindow* Havok_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	return Havok_::InitSharedGUI(parent, helper, gUICallback);
}

void Havok_CloseGUI()
{
	Havok_::CloseSharedGUI();
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
