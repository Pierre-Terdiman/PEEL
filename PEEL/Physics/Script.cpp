///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Script.h"
#include "TestScenes.h"

static AutomatedTests* gAutomatedTests = null;

AutomatedTests* GetAutomatedTests()
{
	return gAutomatedTests;
}

void ReleaseAutomatedTests()
{
	DELETESINGLE(gAutomatedTests);
}

struct ParseContext
{
	ParseContext() :
		mNbFrames		(0),
		mRendering		(false),
		mRandomizeOrder	(false),
		mTrashCache		(false)
	{
	}

	Container	mTests;
	udword		mNbFrames;
	bool		mRendering;
	bool		mRandomizeOrder;
	bool		mTrashCache;
};

AutomatedTests::AutomatedTests(const ParseContext& ctx) :
	mTests			(ctx.mTests),
	mDefaultNbFrames(ctx.mNbFrames),
	mIndex			(0),
	mRendering		(ctx.mRendering),
	mRandomizeOrder	(ctx.mRandomizeOrder),
	mTrashCache		(ctx.mTrashCache)
{
}

AutomatedTests::~AutomatedTests()
{
}

bool AutomatedTests::IsValid() const
{
	return mDefaultNbFrames!=0 && mTests.GetNbEntries()!=0;
}

AutomatedTest* AutomatedTests::GetCurrentTest() const
{
	if(!mTests.GetNbEntries())
		return null;

/*	if(mIndex>=mTests.GetNbEntries())
		return null;

	return (PhysicsTest*)mTests.GetEntry(mIndex);*/

	const udword N = sizeof(AutomatedTest)/sizeof(udword);
	const udword NbTests = mTests.GetNbEntries()/N;
	if(mIndex>=NbTests)
		return null;

	AutomatedTest* AT = (AutomatedTest*)mTests.GetEntries();
	return AT + mIndex;
}

AutomatedTest* AutomatedTests::SelectNextTest()
{
	mIndex++;
	return GetCurrentTest();
}

///////////////////////////////////////////////////////////////////////////////

static PhysicsTest* FindTest(const char* name)
{
	udword NbTests = GetNbTests();
	for(udword i=0;i<NbTests;i++)
	{
		PhysicsTest* CurrentTest = GetTest(i);
		if(strcmp(CurrentTest->GetName(), name)==0)
			return CurrentTest;
	}
	return null;
}

static bool gParseCallback(const char* command, const ParameterBlock& pb, udword context, void* user_data, const ParameterBlock* cmd)
{
	ParseContext* Context = (ParseContext*)user_data;

	if(pb.GetNbParams()>=2 && pb[0]=="Test")
	{
		const char* TestName = pb[1];
		PhysicsTest* Test = FindTest(TestName);
		if(!Test)
			printf(_F("Unknown test in script:\n%s\n", TestName));
		else
		{
			AutomatedTest* AT = (AutomatedTest*)Context->mTests.Reserve(sizeof(AutomatedTest)/sizeof(udword));
			AT->mTest = Test;
			if(pb.GetNbParams()==3)
				AT->mNbFrames = (sdword)pb[2];
			else
				AT->mNbFrames = 0;
		}
	}
	else if(pb.GetNbParams()==2 && pb[0]=="NbFrames")
	{
		Context->mNbFrames = (sdword)pb[1];
	}
	else if(pb.GetNbParams()==2 && pb[0]=="Rendering")
	{
		if(pb[1]=="true")
			Context->mRendering = true;
		else if(pb[1]=="false")
			Context->mRendering = false;
	}
	else if(pb.GetNbParams()==2 && pb[0]=="RandomizeOrder")
	{
		if(pb[1]=="true")
			Context->mRandomizeOrder = true;
		else if(pb[1]=="false")
			Context->mRandomizeOrder = false;
	}
	else if(pb.GetNbParams()==2 && pb[0]=="TrashCache")
	{
		if(pb[1]=="true")
			Context->mTrashCache = true;
		else if(pb[1]=="false")
			Context->mTrashCache = false;
	}
	else
	{
		printf(_F("Unknown command in script:\n%s\n", command));
	}
	return true;
}

void ExecuteScript(const char* filename)
{
	ASSERT(filename);

	ReleaseAutomatedTests();

	ScriptFile Parser;
//	Parser.Enable(BFC_MAKE_LOWER_CASE);
	Parser.Enable(BFC_REMOVE_TABS);
	Parser.Disable(BFC_REMOVE_SEMICOLON);
	Parser.Enable(BFC_DISCARD_COMMENTS);
	Parser.Disable(BFC_DISCARD_UNKNOWNCMDS);
	Parser.Disable(BFC_DISCARD_INVALIDCMDS);
	Parser.Disable(BFC_DISCARD_GLOBALCMDS);

	ParseContext Context;

	Parser.SetUserData(&Context);
	Parser.SetParseCallback(gParseCallback);
	if(Parser.Execute(filename))
	{
		if(Context.mNbFrames && Context.mTests.GetNbEntries())
		{
			gAutomatedTests = ICE_NEW(AutomatedTests)(Context);
		}
	}
}