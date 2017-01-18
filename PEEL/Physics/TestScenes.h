///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef TEST_SCENES_H
#define TEST_SCENES_H

#include "Common.h"
#include "Pint.h"
#include "SurfaceManager.h"
#include "CameraManager.h"

	class Pint;
	class PintRender;
	class PINT_WORLD_CREATE;
	class GLFontRenderer;

	enum TestCategory
	{
		CATEGORY_UNDEFINED,
		CATEGORY_API,
		CATEGORY_BEHAVIOR,
		CATEGORY_CONTACT_GENERATION,
		CATEGORY_JOINTS,
		CATEGORY_ARTICULATIONS,
		CATEGORY_PERFORMANCE,
		CATEGORY_KINEMATICS,
		CATEGORY_CCD,
		CATEGORY_VEHICLES,
		CATEGORY_RAYCAST,
		CATEGORY_SWEEP,
		CATEGORY_OVERLAP,
		CATEGORY_STATIC_SCENE,
		CATEGORY_WIP,
	};

	enum TestFlags
	{
		TEST_FLAGS_DEFAULT			= 0,
		TEST_FLAGS_USE_CURSOR_KEYS	= (1<<0),
	};

	class PhysicsTest
	{
		public:
								PhysicsTest() : mMustResetTest(false)	{}
		virtual					~PhysicsTest()							{}

		virtual	const char*		GetName()						const	= 0;
		virtual	const char*		GetDescription()				const	= 0;
		virtual	TestCategory	GetCategory()					const	= 0;

		virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)	= 0;

		virtual	bool			CommonSetup()							= 0;
		virtual	void			CommonRelease()							= 0;
		virtual	void			CommonUpdate(float dt)					= 0;
		virtual	void			CommonRender(PintRender&)				= 0;

		virtual	bool			Init(Pint& pint)						= 0;
		virtual	void			Close(Pint& pint)						= 0;
		virtual	udword			Update(Pint& pint, float dt)			= 0;

		// Defines if the system should profile the PhysicsTest::Update() function.
		// If false, the system profiles the physics engine's update function.
		// If true, the system profiles the test's update function.
		// Regular tests return false. "SQ tests" return true.
		//
		// When returning true, the generic UI can actually be used to define what the
		// system should profile (see "SQ Profiling Mode" in the user manual).
		virtual	bool			ProfileUpdate()							= 0;

		// Some tests use private customer assets that can't be included in the public version.
		// These tests are unfortunately disabled in that case.
		virtual	bool			IsPrivate()						const	{ return false;	}

		// Experimental support for per-test controls
		virtual	udword			GetFlags()									const	{ return TEST_FLAGS_DEFAULT;	}
//		virtual	bool			MotionCallback(int x, int y)						{ return false;	}
//		virtual	bool			MouseCallback(int button, int state, int x, int y)	{ return false;	}
		virtual	bool			SpecialKeyCallback(int key, int x, int y, bool down){ return false;	}
//		virtual	bool			KeyboardCallback(unsigned char key, int x, int y)	{ return false;	}

		// Configurable tests can have their own dialog. They are created and deleted in these dedicated functions.
		// That way we can reset the scene without deleting the dialog window (thus keeping current settings alive).
		virtual	void			InitUI(PintGUIHelper& helper)			{}
		virtual	void			CloseUI()								{}
		// Returns optional sub-name for presets (will be used in saved Excel files)
		virtual	const char*		GetSubName()					const	{ return null; }

		// Let tests draw some information on screen if needed. Experimental design.
		virtual	float			DrawDebugText(Pint& pint, GLFontRenderer& renderer, float y, float text_scale)	{ return y;	}
		virtual	void			DrawDebugInfo(Pint& pint, PintRender& render)	{}

		// This is used to let the per-test GUI (from "configurable tests") reset the test after users have
		// tweaked the test parameters.
				bool			mMustResetTest;
	};

	void			InitTests();
	udword			GetNbTests();
	PhysicsTest*	GetTest(udword i);


	class TestBase : public SurfaceManager, /*public ObjectsManager,*/ public PhysicsTest
	{
		public:
										TestBase();
		virtual							~TestBase();

		// PhysicsTest
		virtual	void					GetSceneParams(PINT_WORLD_CREATE& params);

		virtual	bool					CommonSetup();
		virtual	void					CommonRelease();
		virtual	void					CommonUpdate(float dt);
		virtual	void					CommonRender(PintRender&);

		virtual	bool					Init(Pint& pint);
		virtual	void					Close(Pint& pint){}
		virtual	udword					Update(Pint& pint, float dt);

		virtual	bool					ProfileUpdate();

		virtual	void					CloseUI();
		//~PhysicsTest

		virtual	bool					Setup(Pint& pint, const PintCaps& caps)	= 0;

				void					RegisterAABB(const AABB& aabb);
				void					RenderAllAABBs(PintRender& renderer);
		inline_	udword					GetNbAABBs()	const	{ return mAABBs.GetNbEntries()/(sizeof(AABB)/sizeof(udword));	}
		inline_	const AABB*				GetAABBs()		const	{ return (const AABB*)mAABBs.GetEntries();						}

				void					RegisterRaycast(const Point& origin, const Point& dir, float max_dist, void* phantom_handle=null);
				udword					GetNbRegisteredRaycasts()	const;
				PintRaycastData*		GetRegisteredRaycasts()		const;
				udword					GetNbRegisteredPhantoms()	const;
				void**					GetRegisteredPhantoms()		const;
				void					UnregisterAllRaycasts();

				void					RegisterBoxSweep(const OBB& box, const Point& dir, float max_dist);
				udword					GetNbRegisteredBoxSweeps()	const;
				PintBoxSweepData*		GetRegisteredBoxSweeps()	const;
				void					UnregisterAllBoxSweeps();

				void					RegisterSphereSweep(const Sphere& sphere, const Point& dir, float max_dist);
				udword					GetNbRegisteredSphereSweeps()	const;
				PintSphereSweepData*	GetRegisteredSphereSweeps()		const;
				void					UnregisterAllSphereSweeps();

				void					RegisterCapsuleSweep(const LSS& capsule, const Point& dir, float max_dist);
				udword					GetNbRegisteredCapsuleSweeps()	const;
				PintCapsuleSweepData*	GetRegisteredCapsuleSweeps()	const;
				void					UnregisterAllCapsuleSweeps();

				void					RegisterConvexSweep(const udword convex_object_index, PintShapeRenderer* renderer, const PR& pr, const Point& dir, float max_dist);
				udword					GetNbRegisteredConvexSweeps()	const;
				PintConvexSweepData*	GetRegisteredConvexSweeps()		const;
				void					UnregisterAllConvexSweeps();

				void					RegisterSphereOverlap(const Sphere& sphere);
				udword					GetNbRegisteredSphereOverlaps()	const;
				PintSphereOverlapData*	GetRegisteredSphereOverlaps()	const;
				void					UnregisterAllSphereOverlaps();

				void					RegisterBoxOverlap(const OBB& box);
				udword					GetNbRegisteredBoxOverlaps()	const;
				PintBoxOverlapData*		GetRegisteredBoxOverlaps()		const;
				void					UnregisterAllBoxOverlaps();

				void					RegisterCapsuleOverlap(const LSS& capsule);
				udword					GetNbRegisteredCapsuleOverlaps()	const;
				PintCapsuleOverlapData*	GetRegisteredCapsuleOverlaps()		const;
				void					UnregisterAllCapsuleOverlaps();

				void					RegisterRenderer(PintShapeRenderer* renderer);
				udword					GetNbRegisteredRenderers()			const;
				PintShapeRenderer**		GetRegisteredRenderers()			const;

		inline_	Container*				GetUIElements()
										{
											return &mUIElems;
										}
		inline_	void					RegisterUIElement(IceWidget* widget)
										{
											mUIElems.Add(udword(widget));
										}
				void					AddResetButton(IceWindow* parent, sdword x, sdword y, sdword width);

				CameraManager			mCameraManager;

				PINT_MATERIAL_CREATE	mHighFrictionMaterial;
				PINT_MATERIAL_CREATE	mZeroFrictionMaterial;

				void*					mRepX;				// Built-in support for one RepX scene
				float					mCurrentTime;
				Container				mAABBs;
				Container				mRaycastData;
				Container				mPhantomData;
				Container				mBoxSweepData;
				Container				mSphereSweepData;
				Container				mCapsuleSweepData;
				Container				mConvexSweepData;
				Container				mSphereOverlapData;
				Container				mBoxOverlapData;
				Container				mCapsuleOverlapData;
				Container				mRenderers;
		private:
				Container				mUIElems;
		public:
				bool					mCreateDefaultEnvironment;
	};

	#define START_TEST(name, category, desc)												\
		class name : public TestBase														\
		{																					\
			public:																			\
									name()						{						}	\
			virtual					~name()						{						}	\
			virtual	const char*		GetName()			const	{ return #name;			}	\
			virtual	const char*		GetDescription()	const	{ return desc;			}	\
			virtual	TestCategory	GetCategory()		const	{ return category;		}

	#define START_SQ_TEST(name, category, desc)												\
		class name : public TestBase														\
		{																					\
			public:																			\
									name()						{						}	\
			virtual					~name()						{						}	\
			virtual	const char*		GetName()			const	{ return #name;			}	\
			virtual	const char*		GetDescription()	const	{ return desc;			}	\
			virtual	TestCategory	GetCategory()		const	{ return category;		}	\
			virtual	bool			ProfileUpdate()				{ return true;			}

	#define START_SQ_RAYCAST_TEST(name, category, desc)										\
		class name : public TestBase														\
		{																					\
			public:																			\
									name()						{						}	\
			virtual					~name()						{						}	\
			virtual	const char*		GetName()			const	{ return #name;			}	\
			virtual	const char*		GetDescription()	const	{ return desc;			}	\
			virtual	TestCategory	GetCategory()		const	{ return category;		}	\
			virtual	bool			ProfileUpdate()				{ return true;			}	\
			virtual	udword			Update(Pint& pint, float dt){ return DoBatchRaycasts(*this, pint);	}

	#define END_TEST(name)	}name;

#endif