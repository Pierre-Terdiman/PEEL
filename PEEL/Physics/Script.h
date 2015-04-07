///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef SCRIPT_H
#define SCRIPT_H

	class PhysicsTest;
	struct ParseContext;

	struct AutomatedTest
	{
		PhysicsTest*	mTest;
		udword			mNbFrames;
	};

	class AutomatedTests : public Allocateable
	{
		public:
							AutomatedTests(const ParseContext&);
							~AutomatedTests();

			bool			IsValid()			const;
			AutomatedTest*	GetCurrentTest()	const;
			AutomatedTest*	SelectNextTest();

			Container		mTests;
			udword			mDefaultNbFrames;
			udword			mIndex;
			bool			mRendering;
			bool			mRandomizeOrder;
			bool			mTrashCache;
	};

	AutomatedTests* GetAutomatedTests();
	void ReleaseAutomatedTests();

	void ExecuteScript(const char* filename);

#endif
