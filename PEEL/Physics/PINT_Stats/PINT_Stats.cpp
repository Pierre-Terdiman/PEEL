///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_Stats.h"
#include "..\PINT_Common\PINT_Common.h"

///////////////////////////////////////////////////////////////////////////////

static	bool	gDrawSceneBounds = false;

Stats::Stats() :
	mNbStatics			(0),
	mNbDynamics			(0),
	mNbStaticCompounds	(0),
	mNbDynamicCompounds	(0),
	mNbJoints			(0),
	mNbBoxShapes		(0),
	mNbSphereShapes		(0),
	mNbCapsuleShapes	(0),
	mNbConvexShapes		(0),
	mNbMeshShapes		(0),
	mTotalNbVerts		(0),
	mTotalNbTris		(0)
{
}

static void UpdateStatsUI(const Stats& stats, const PINT_WORLD_CREATE* create);

StatsPint::StatsPint() : mUpdateUI(false)
{
}

StatsPint::~StatsPint()
{
}

void StatsPint::GetCaps(PintCaps& caps) const
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

void StatsPint::Init(const PINT_WORLD_CREATE& desc)
{
	mCreate = desc;
	mUpdateUI = true;
}

void StatsPint::SetGravity(const Point& gravity)
{
}

void StatsPint::Close()
{
}

udword StatsPint::Update(float dt)
{
	if(mUpdateUI)
	{
		mUpdateUI = false;
		UpdateStatsUI(mStats, &mCreate);
	}
	return 0;
}

Point StatsPint::GetMainColor()
{
	return Point(1.0f, 1.0f, 1.0f);
}

void StatsPint::Render(PintRender& renderer)
{
	if(gDrawSceneBounds)
	{
		if(mCreate.mGlobalBounds.IsValid())
		{
			renderer.DrawWirefameAABB(mCreate.mGlobalBounds, Point(1.0f, 0.0f, 0.0f));
		}
	}
}

PintObjectHandle StatsPint::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	const udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	if(desc.mMass==0.0f)
	{
		mStats.mNbStatics++;
		if(NbShapes>1)
			mStats.mNbStaticCompounds++;
	}
	else
	{
		mStats.mNbDynamics++;
		if(NbShapes>1)
			mStats.mNbDynamicCompounds++;
	}

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
		if(CurrentShape->mType==PINT_SHAPE_SPHERE)
		{
			mStats.mNbSphereShapes++;
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			mStats.mNbBoxShapes++;
		}
		else if(CurrentShape->mType==PINT_SHAPE_CAPSULE)
		{
			mStats.mNbCapsuleShapes++;
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			mStats.mNbConvexShapes++;
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
			const PINT_MESH_CREATE* MeshCreate = static_cast<const PINT_MESH_CREATE*>(CurrentShape);
			mStats.mNbMeshShapes++;
			mStats.mTotalNbVerts += MeshCreate->mSurface.mNbVerts;
			mStats.mTotalNbTris += MeshCreate->mSurface.mNbFaces;
		}
		else ASSERT(0);

		CurrentShape = CurrentShape->mNext;
	}
	mUpdateUI = true;
	return null;
}

bool StatsPint::ReleaseObject(PintObjectHandle handle)
{
	return false;
}

PintJointHandle StatsPint::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	mStats.mNbJoints++;
	mUpdateUI = true;
	return null;
}

void StatsPint::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
}

void* StatsPint::CreatePhantom(const AABB& box)
{
	return null;
}

udword StatsPint::BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void** phantoms)
{
	return 0;
}

udword StatsPint::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	return 0;
}

udword StatsPint::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	return 0;
}

udword StatsPint::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	return 0;
}

udword StatsPint::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	return 0;
}

udword StatsPint::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	return 0;
}

udword StatsPint::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	return 0;
}

PR StatsPint::GetWorldTransform(PintObjectHandle handle)
{
	return PR();
}

void StatsPint::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
}

udword StatsPint::GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)
{
	return 0;
}

void StatsPint::SetLocalRot(PintObjectHandle handle, const Quat& q)
{
}

bool StatsPint::GetConvexData(PintObjectHandle handle, PINT_CONVEX_CREATE& data)
{
	return false;
}


static StatsPint* gStats = null;

void Stats_Init(const PINT_WORLD_CREATE& desc)
{
	ASSERT(!gStats);
	gStats = ICE_NEW(StatsPint);
	gStats->Init(desc);
}

void Stats_Close()
{
	if(gStats)
	{
		gStats->Close();
		delete gStats;
		gStats = null;
	}
}

StatsPint* GetStats()
{
	return gStats;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gStatsGUI = null;
static IceEditBox*	gEditBox_Stats = null;
static IceEditBox*	gEditBox_Notes = null;
static IceCheckBox*	gCheckBox_DrawSceneBounds = null;

static void UpdateStatsUI(const Stats& stats, const PINT_WORLD_CREATE* create)
{
	ASSERT(gEditBox_Stats);
	{
		const udword TotalNbActors = stats.mNbStatics + stats.mNbDynamics;
		const udword TotalNbShapes = stats.mNbBoxShapes + stats.mNbSphereShapes + stats.mNbCapsuleShapes + stats.mNbConvexShapes + stats.mNbMeshShapes;

		CustomArray CA;
		CA.StoreASCII(_F("%d actors:\n=======\n", TotalNbActors));
		CA.StoreASCII(_F("%d static actors:\n", stats.mNbStatics));
		CA.StoreASCII(_F("    %d singles\n", stats.mNbStatics - stats.mNbStaticCompounds));
		CA.StoreASCII(_F("    %d compounds\n", stats.mNbStaticCompounds));
		CA.StoreASCII(_F("%d dynamic actors:\n", stats.mNbDynamics));
		CA.StoreASCII(_F("    %d singles\n", stats.mNbDynamics - stats.mNbDynamicCompounds));
		CA.StoreASCII(_F("    %d compounds\n", stats.mNbDynamicCompounds));
		CA.StoreASCII("\nJoints:\n=====\n");
		CA.StoreASCII(_F("%d joints\n", stats.mNbJoints));
		CA.StoreASCII(_F("\n%d shapes:\n========\n", TotalNbShapes));
		CA.StoreASCII(_F("%d box shapes\n", stats.mNbBoxShapes));
		CA.StoreASCII(_F("%d sphere shapes\n", stats.mNbSphereShapes));
		CA.StoreASCII(_F("%d capsule shapes\n", stats.mNbCapsuleShapes));
		CA.StoreASCII(_F("%d convex shapes\n", stats.mNbConvexShapes));
		CA.StoreASCII(_F("%d mesh shapes:\n", stats.mNbMeshShapes));
		CA.StoreASCII(_F("    %d total nb verts\n", stats.mTotalNbVerts));
		CA.StoreASCII(_F("    %d total nb tris\n", stats.mTotalNbTris));
		CA.Store(ubyte(0));

		gEditBox_Stats->SetText((const char*)CA.Collapse());
	}

	ASSERT(gEditBox_Notes);
	if(create)
	{
		CustomArray CA;

		if(!create->mGlobalBounds.IsValid())
		{
			CA.StoreASCII("Global bounds not set.\n");
			gCheckBox_DrawSceneBounds->SetEnabled(false);
			gCheckBox_DrawSceneBounds->SetChecked(false);
		}
		else
		{
			gCheckBox_DrawSceneBounds->SetEnabled(true);
		}
		CA.Store(ubyte(0));

		gEditBox_Notes->SetText((const char*)CA.Collapse());
	}
}

enum StatsGUIElement
{
	STATS_GUI_MAIN,
	//
	STATS_GUI_EDIT_BOX_STATS,
	STATS_GUI_EDIT_BOX_NOTES,
	//
	STATS_GUI_DRAW_SCENE_BOUNDS,
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case STATS_GUI_DRAW_SCENE_BOUNDS:
			gDrawSceneBounds = checked;
			break;
	}

//	if(gPhysX)
//		gPhysX->UpdateFromUI();
}

static const char* gTooltip_DrawSceneBounds		= "Visualize scene bounds passed by each test to plug-in's init function";

IceWindow* Stats_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gStatsGUI, parent, STATS_GUI_MAIN, "Stats options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	const sdword OffsetX = 180;
	const sdword LabelOffsetY = 2;

	{
//		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Summary:", gStatsGUI);
		gEditBox_Stats = helper.CreateEditBox(Main, STATS_GUI_EDIT_BOX_STATS, 4+OffsetX, y, 300, 300, "", gStatsGUI, EDITBOX_TEXT, null);
		gEditBox_Stats->SetReadOnly(true);

//		y += YStep;
	}

	{
//		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Summary:", gStatsGUI);
		gEditBox_Notes = helper.CreateEditBox(Main, STATS_GUI_EDIT_BOX_NOTES, 4, y, 160, 100, "", gStatsGUI, EDITBOX_TEXT, null);
		gEditBox_Notes->SetReadOnly(true);

//		y += YStep;
	}

	y = 150;
	{
		const udword CheckBoxWidth = 150;

		gCheckBox_DrawSceneBounds = helper.CreateCheckBox(Main, STATS_GUI_DRAW_SCENE_BOUNDS, 4, y, CheckBoxWidth, 20, "Draw scene bounds", gStatsGUI, gDrawSceneBounds, gCheckBoxCallback, gTooltip_DrawSceneBounds);
		y += YStepCB;
	}

	Stats s;
	UpdateStatsUI(s, null);

	return Main;
}

void Stats_CloseGUI()
{
	Common_CloseGUI(gStatsGUI);
	gEditBox_Stats = null;
	gEditBox_Notes = null;
	gCheckBox_DrawSceneBounds = null;
}

///////////////////////////////////////////////////////////////////////////////

class StatsPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Stats_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Stats_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Stats_Init(desc);						}
	virtual	void		Close()												{ Stats_Close();						}
	virtual	Pint*		GetPint()											{ return GetStats();					}
};
static StatsPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
