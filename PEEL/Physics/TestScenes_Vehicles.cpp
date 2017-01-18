///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Render.h"
#include "Camera.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "Cylinder.h"
#include "ProceduralTrack.h"
#include "Loader_Bin.h"
#include "GUI_Helpers.h"

///////////////////////////////////////////////////////////////////////////////

	struct VehicleSDKData : public Allocateable
	{
		VehicleSDKData()
		{
			mVehicle = null;
			mChassis = null;
		}
		PintObjectHandle	mVehicle;
		PintObjectHandle	mChassis;
	};

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_VehicleSDK = "(Configurable test) - Basic test for the Vehicle SDK. Mainly based on the PhysX API & plugin for now. \
Use the arrow keys to drive the vehicle (the render window must have the focus). Switch between raycast-based and sweep-based vehicles in the \
PhysX UI options panel, for versions that expose/support both. Use the X key (wireframe overlay rendering) to better visualize the ground if needed. \
This is a work-in-progress, many parameters are hardcoded, not fine-tuned, not exposed to the UI, etc.";

extern String* gRoot;
static void gButtonCallback(IceButton& button, void* user_data);

class VehicleSDK : public TestBase
{
	public:
							VehicleSDK() :
								mCheckBox_DriveVehicle				(null),
								mEditBox_WheelRadius				(null),
								mEditBox_WheelWidth					(null),
								mEditBox_WheelCoeffX				(null),
								mEditBox_WheelCoeffZ				(null),
								mEditBox_WheelMass					(null),
								mEditBox_WheelMaxBrakeTorqueFront	(null),
								mEditBox_WheelMaxBrakeTorqueRear	(null),
								mEditBox_WheelMaxSteerFront			(null),
								mEditBox_WheelMaxSteerRear			(null),
								mEditBox_TireFrictionMultiplier		(null),
								mEditBox_ChassisX					(null),
								mEditBox_ChassisY					(null),
								mEditBox_ChassisZ					(null),
								mEditBox_ChassisMass				(null),
								mEditBox_ChassisMOICoeffY			(null),
								mEditBox_ChassisCMOffsetY			(null),
								mEditBox_ChassisCMOffsetZ			(null),
								mEditBox_ForceApplicationCMOffsetY	(null),
								mEditBox_EnginePeakTorque			(null),
								mEditBox_EngineMaxOmega				(null),
								mEditBox_SuspMaxCompression			(null),
								mEditBox_SuspMaxDroop				(null),
								mEditBox_SuspSpringStrength			(null),
								mEditBox_SuspSpringDamperRate		(null),
								mEditBox_SuspCamberAngleAtRest		(null),
								mEditBox_SuspCamberAngleAtMaxCompr	(null),
								mEditBox_SuspCamberAngleAtMaxDroop	(null),
								mEditBox_GearsSwitchTime			(null),
								mEditBox_ClutchStrength				(null),
								mComboBox_Level						(null),
								mComboBox_Differential				(null)
														{}
	virtual					~VehicleSDK()				{}
	virtual	const char*		GetName()			const	{ return "VehicleSDK";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_VehicleSDK;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_VEHICLES;	}

	PINT_VEHICLE_INPUT	mInput;
	IceCheckBox*		mCheckBox_DriveVehicle;
	IceEditBox*			mEditBox_WheelRadius;
	IceEditBox*			mEditBox_WheelWidth;
	IceEditBox*			mEditBox_WheelCoeffX;
	IceEditBox*			mEditBox_WheelCoeffZ;
	IceEditBox*			mEditBox_WheelMass;
	IceEditBox*			mEditBox_WheelMaxBrakeTorqueFront;
	IceEditBox*			mEditBox_WheelMaxBrakeTorqueRear;
	IceEditBox*			mEditBox_WheelMaxSteerFront;
	IceEditBox*			mEditBox_WheelMaxSteerRear;
	IceEditBox*			mEditBox_TireFrictionMultiplier;
	IceEditBox*			mEditBox_ChassisX;
	IceEditBox*			mEditBox_ChassisY;
	IceEditBox*			mEditBox_ChassisZ;
	IceEditBox*			mEditBox_ChassisMass;
	IceEditBox*			mEditBox_ChassisMOICoeffY;
	IceEditBox*			mEditBox_ChassisCMOffsetY;
	IceEditBox*			mEditBox_ChassisCMOffsetZ;
	IceEditBox*			mEditBox_ForceApplicationCMOffsetY;
	IceEditBox*			mEditBox_EnginePeakTorque;
	IceEditBox*			mEditBox_EngineMaxOmega;
	IceEditBox*			mEditBox_SuspMaxCompression;
	IceEditBox*			mEditBox_SuspMaxDroop;
	IceEditBox*			mEditBox_SuspSpringStrength;
	IceEditBox*			mEditBox_SuspSpringDamperRate;
	IceEditBox*			mEditBox_SuspCamberAngleAtRest;
	IceEditBox*			mEditBox_SuspCamberAngleAtMaxCompr;
	IceEditBox*			mEditBox_SuspCamberAngleAtMaxDroop;
	IceEditBox*			mEditBox_GearsSwitchTime;
	IceEditBox*			mEditBox_ClutchStrength;
	IceComboBox*		mComboBox_Level;
	IceComboBox*		mComboBox_Differential;

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 500;
		WD.mLabel	= "VehicleSDK";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 180;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			mCheckBox_DriveVehicle = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Drive vehicle", UIElems, true, null, null);
			y += YStep;
		}
		{
			ComboBoxDesc CBBD;
			CBBD.mID		= 0;
			CBBD.mParent	= UI;
			CBBD.mX			= 4;
			CBBD.mY			= y;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;
			CBBD.mLabel		= "Level";
			IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);
			RegisterUIElement(CB);

			CB->Add("Flat");
			CB->Add("Flat with box bumps");
			CB->Add("Terrain");
			CB->Add("Race track");

			CB->Select(0);
			CB->SetVisible(true);
			CB->SetEnabled(true);

			mComboBox_Level = CB;
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);

		{
			y += YStep;
			ButtonDesc BD;
			BD.mUserData	= this;
//			BD.mStyle		= ;
			BD.mCallback	= gButtonCallback;
			BD.mID			= 0;
			BD.mParent		= UI;
			BD.mX			= 4;
			BD.mY			= y;
			BD.mWidth		= 300-16;
			BD.mHeight		= 20;
			BD.mLabel		= "Save car data";
			IceButton* B = ICE_NEW(IceButton)(BD);
			RegisterUIElement(B);
			B->SetVisible(true);
			y += YStep;

			BD.mID			= 1;
			BD.mY			= y;
			BD.mLabel		= "Load car data";
			B = ICE_NEW(IceButton)(BD);
			RegisterUIElement(B);
			B->SetVisible(true);
		}


		// Tab control
		enum TabIndex
		{
			TAB_CHASSIS,
			TAB_WHEEL_TIRES,
			TAB_ENGINE_GEARBOX,
			TAB_SUSPENSION,
			TAB_DIFFERENTIAL,
			TAB_STEERING,
			TAB_COUNT,
		};
		IceWindow* Tabs[TAB_COUNT];
		{
			TabControlDesc TCD;
			TCD.mParent	= UI;
			TCD.mX		= 4;
			TCD.mY		= 0;
			TCD.mY		= y + 50;
			TCD.mWidth	= WD.mWidth - 16;
			TCD.mHeight	= 300;
			IceTabControl* TabControl = ICE_NEW(IceTabControl)(TCD);
			RegisterUIElement(TabControl);

			for(udword i=0;i<TAB_COUNT;i++)
			{
				WindowDesc WD;
				WD.mParent	= UI;
				WD.mX		= 0;
				WD.mY		= 0;
				WD.mWidth	= WD.mWidth;
				WD.mHeight	= TCD.mHeight;
				WD.mLabel	= "Tab";
				WD.mType	= WINDOW_DIALOG;
				IceWindow* Tab = ICE_NEW(IceWindow)(WD);
				RegisterUIElement(Tab);
				Tab->SetVisible(true);
				Tabs[i] = Tab;
			}
			TabControl->Add(Tabs[TAB_CHASSIS], "Chassis");
			TabControl->Add(Tabs[TAB_WHEEL_TIRES], "Wheels/tires");
			TabControl->Add(Tabs[TAB_ENGINE_GEARBOX], "Engine/gearbox");
			TabControl->Add(Tabs[TAB_SUSPENSION], "Suspension");
			TabControl->Add(Tabs[TAB_DIFFERENTIAL], "Differential");
			TabControl->Add(Tabs[TAB_STEERING], "Steering");
		}

		// TAB_CHASSIS
		{
			IceWindow* TabWindow = Tabs[TAB_CHASSIS];
			y = 4;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Width:", UIElems);
			mEditBox_ChassisX = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Height:", UIElems);
			mEditBox_ChassisY = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Length:", UIElems);
			mEditBox_ChassisZ = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Mass:", UIElems);
			mEditBox_ChassisMass = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1500.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "MOI Y coeff:", UIElems);
			mEditBox_ChassisMOICoeffY = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.8", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "CM Y offset:", UIElems);
			mEditBox_ChassisCMOffsetY = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.65", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "CM Z offset:", UIElems);
			mEditBox_ChassisCMOffsetZ = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.25", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Force app. CM Y offset (~body roll):", UIElems);
			mEditBox_ForceApplicationCMOffsetY = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.3", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;
		}

		// TAB_WHEEL_TIRES
		{
			IceWindow* TabWindow = Tabs[TAB_WHEEL_TIRES];
			y = 4;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Radius:", UIElems);
			mEditBox_WheelRadius = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Width:", UIElems);
			mEditBox_WheelWidth = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.3", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Coeff X:", UIElems);
			mEditBox_WheelCoeffX = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.85", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Coeff Z:", UIElems);
			mEditBox_WheelCoeffZ = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.85", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Mass:", UIElems);
			mEditBox_WheelMass = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "20.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max brake torque front:", UIElems);
			mEditBox_WheelMaxBrakeTorqueFront = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "150.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max brake torque rear:", UIElems);
			mEditBox_WheelMaxBrakeTorqueRear = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1500.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max steer front:", UIElems);
			mEditBox_WheelMaxSteerFront = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.047", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max steer rear:", UIElems);
			mEditBox_WheelMaxSteerRear = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Tire friction multiplier (~grip):", UIElems);
			mEditBox_TireFrictionMultiplier = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.1", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;
		}

		// TAB_ENGINE_GEARBOX
		{
			IceWindow* TabWindow = Tabs[TAB_ENGINE_GEARBOX];
			y = 4;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Engine peak torque:", UIElems);
			mEditBox_EnginePeakTorque = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1000.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Engine max omega:", UIElems);
			mEditBox_EngineMaxOmega = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "1000.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Gears switch time:", UIElems);
			mEditBox_GearsSwitchTime = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Clutch strength:", UIElems);
			mEditBox_ClutchStrength = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "10.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			IceEditBox* tmp = helper.CreateEditBox(TabWindow, 0, 4, y+20, 250, 100, "", UIElems, EDITBOX_TEXT, null);
			tmp->SetMultilineText("Most engine & gearbox params are currently\nhardcoded and not exposed to the UI.");
			tmp->SetReadOnly(true);
		}

		// TAB_SUSPENSION
		{
			IceWindow* TabWindow = Tabs[TAB_SUSPENSION];
			y = 4;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max compression:", UIElems);
			mEditBox_SuspMaxCompression = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.3", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Max droop:", UIElems);
			mEditBox_SuspMaxDroop = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.1", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Spring strength:", UIElems);
			mEditBox_SuspSpringStrength = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "35000.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Spring damper rate:", UIElems);
			mEditBox_SuspSpringDamperRate = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "4500.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Camber angle at rest:", UIElems);
			mEditBox_SuspCamberAngleAtRest = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Camber angle at max compression:", UIElems);
			mEditBox_SuspCamberAngleAtMaxCompr = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.01", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Camber angle at max droop:", UIElems);
			mEditBox_SuspCamberAngleAtMaxDroop = helper.CreateEditBox(TabWindow, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.01", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;
		}

		// TAB_DIFFERENTIAL
		{
			IceWindow* TabWindow = Tabs[TAB_DIFFERENTIAL];
			y = 4;

			helper.CreateLabel(TabWindow, 4, y+LabelOffsetY, LabelWidth, 20, "Differential:", UIElems);
			{
				ComboBoxDesc CBBD;
				CBBD.mID		= 0;
				CBBD.mParent	= TabWindow;
				CBBD.mX			= 64;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "Differential";
				IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);
				RegisterUIElement(CB);

				CB->Add("4WD limited slip diff");
				CB->Add("FWD limited slip diff");
				CB->Add("RWD limited slip diff");
				CB->Add("4WD open diff");
				CB->Add("FWD open diff");
				CB->Add("RWD open diff");

				CB->Select(0);
				CB->SetVisible(true);

				mComboBox_Differential = CB;
			}
		}

		// TAB_STEERING
		{
			IceWindow* TabWindow = Tabs[TAB_STEERING];
			y = 4;

			if(0)
			{
				GraphDesc Desc;
				Desc.mParent			= TabWindow;
				Desc.mX					= 4;
				Desc.mY					= y;
				Desc.mWidth				= 250;
				Desc.mHeight			= 200;
				Desc.mLabel				= "GW";
				Desc.mType				= WINDOW_DIALOG;
				Desc.mGraphStyle		= GRAPH_ANTIALIASED;
				Desc.mEnablePopupMenu	= false;

				GraphWindow* GW = ICE_NEW(GraphWindow)(Desc);
				RegisterUIElement(GW);

#define APP_TO_GW(x)	-(x)

				GW->SetKey(0, 0.0f/120.0f, APP_TO_GW(0.75f));

				Keyframe k;
				k.mX = 5.0f/120.0f;		k.mY = APP_TO_GW(0.75f);	GW->CreateKey(k);
				k.mX = 30.0f/120.0f;	k.mY = APP_TO_GW(0.125f);	GW->CreateKey(k);
				k.mX = 120.0f/120.0f;	k.mY = APP_TO_GW(0.1f);		GW->CreateKey(k);
				GW->SetVisible(true);
			}


		}
	}

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(4.89f, 3.40f, 4.62f), Point(-0.71f, -0.31f, -0.63f));
	}

	virtual void	Close(Pint& pint)
	{
		VehicleSDKData* UserData = (VehicleSDKData*)pint.mUserData;
		DELETESINGLE(UserData);
		pint.mUserData = null;

		TestBase::Close(pint);
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		const udword Index = mComboBox_Level->GetSelectedIndex();
		if(Index==0)
		{
			mCreateDefaultEnvironment = false;
		}
		else if(Index==1)
		{
			mCreateDefaultEnvironment = false;
		}
		else if(Index==2)
		{
			LoadMeshesFromFile_(*this, "Terrain.bin", null, false, 0);
			mCreateDefaultEnvironment = false;
		}
		else if(Index==3)
		{
			mCreateDefaultEnvironment = false;
			IndexedSurface* IS;
			{
				IS = CreateManagedSurface();

				RaceTrack RT;
				RT.Build();

				IS->Init(RT.mNbTris, RT.mNbVerts, RT.mVerts, (const IndexedTriangle*)RT.mIndices);

				Point Center;
				IS->GetFace(0)->Center(IS->GetVerts(), Center);

				IS->Translate(-Center);
			}
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportVehicles)
			return false;

		CreateMeshesFromRegisteredSurfaces(pint, caps, *this, &mHighFrictionMaterial);

		// We currently have this hardcoded in the PhysX plugin so we must define the car accordingly:
		//const PxVec3 forward(0.0f, 0.0f, 1.0f);

		const float WheelRadius = GetFromEditBox(0.5f, mEditBox_WheelRadius, 0.0f, MAX_FLOAT);
		const float WheelWidth = GetFromEditBox(0.3f, mEditBox_WheelWidth, 0.0f, MAX_FLOAT);
		const float CoeffX = GetFromEditBox(0.85f, mEditBox_WheelCoeffX, 0.0f, MAX_FLOAT);
		const float CoeffZ = GetFromEditBox(0.85f, mEditBox_WheelCoeffZ, 0.0f, MAX_FLOAT);

		// Forward is along Z so the chassis must be along Z as well
		Point ChassisExtents;
		ChassisExtents.x = GetFromEditBox(1.0f, mEditBox_ChassisX, 0.0f, MAX_FLOAT);
		ChassisExtents.y = GetFromEditBox(0.5f, mEditBox_ChassisY, 0.0f, MAX_FLOAT);
		ChassisExtents.z = GetFromEditBox(1.5f, mEditBox_ChassisZ, 0.0f, MAX_FLOAT);

		const udword NbPts = 60;
//		const udword NbPts = 10;
		// Forward is along Z so the cylinder axis must be along X
		const CylinderMesh Cylinder(NbPts, WheelRadius, WheelWidth*0.5f, ORIENTATION_YZ);
		const udword TotalNbVerts = Cylinder.mNbVerts;
		const Point* Verts = Cylinder.mVerts;

		PINT_VEHICLE_CREATE VehicleDesc;
		VehicleDesc.mStartPose.mPos				= Point(0.0f, 3.0f, 0.0f);
		VehicleDesc.mDifferential				= PintVehicleDifferential(mComboBox_Differential->GetSelectedIndex());
		VehicleDesc.mChassisMass				= GetFromEditBox(1500.0f,	mEditBox_ChassisMass,				0.0f, MAX_FLOAT);
		VehicleDesc.mChassisMOICoeffY			= GetFromEditBox(0.8f,		mEditBox_ChassisMOICoeffY,			0.0f, MAX_FLOAT);
		VehicleDesc.mChassisCMOffsetY			= GetFromEditBox(0.65f,		mEditBox_ChassisCMOffsetY,			MIN_FLOAT, MAX_FLOAT);
		VehicleDesc.mChassisCMOffsetZ			= GetFromEditBox(0.25f,		mEditBox_ChassisCMOffsetZ,			MIN_FLOAT, MAX_FLOAT);
		VehicleDesc.mForceApplicationCMOffsetY	= GetFromEditBox(0.3f,		mEditBox_ForceApplicationCMOffsetY,	0.0f, MAX_FLOAT);
		VehicleDesc.mWheelMass					= GetFromEditBox(20.0f,		mEditBox_WheelMass,					0.0f, MAX_FLOAT);
		VehicleDesc.mWheelMaxBrakeTorqueFront	= GetFromEditBox(150.0f,	mEditBox_WheelMaxBrakeTorqueFront,	0.0f, MAX_FLOAT);
		VehicleDesc.mWheelMaxBrakeTorqueRear	= GetFromEditBox(1500.0f,	mEditBox_WheelMaxBrakeTorqueRear,	0.0f, MAX_FLOAT);
		VehicleDesc.mWheelMaxSteerFront			= GetFromEditBox(PI/3.0f,	mEditBox_WheelMaxSteerFront,		0.0f, MAX_FLOAT);
		VehicleDesc.mWheelMaxSteerRear			= GetFromEditBox(0.0f,		mEditBox_WheelMaxSteerRear,			0.0f, MAX_FLOAT);
		VehicleDesc.mTireFrictionMultiplier		= GetFromEditBox(1.1f,		mEditBox_TireFrictionMultiplier,	0.0f, MAX_FLOAT);
		VehicleDesc.mEnginePeakTorque			= GetFromEditBox(1000.0f,	mEditBox_EnginePeakTorque,			0.0f, MAX_FLOAT);
		VehicleDesc.mEngineMaxOmega				= GetFromEditBox(1000.0f,	mEditBox_EngineMaxOmega,			0.0f, MAX_FLOAT);
		VehicleDesc.mSuspMaxCompression			= GetFromEditBox(0.3f,		mEditBox_SuspMaxCompression,		0.0f, MAX_FLOAT);
		VehicleDesc.mSuspMaxDroop				= GetFromEditBox(0.1f,		mEditBox_SuspMaxDroop,				0.0f, MAX_FLOAT);
		VehicleDesc.mSuspSpringStrength			= GetFromEditBox(35000.0f,	mEditBox_SuspSpringStrength,		0.0f, MAX_FLOAT);
		VehicleDesc.mSuspSpringDamperRate		= GetFromEditBox(4500.0f,	mEditBox_SuspSpringDamperRate,		0.0f, MAX_FLOAT);
		VehicleDesc.mSuspCamberAngleAtRest		= GetFromEditBox(0.0f,		mEditBox_SuspCamberAngleAtRest,		0.0f, MAX_FLOAT);
		VehicleDesc.mSuspCamberAngleAtMaxCompr	= GetFromEditBox(0.01f,		mEditBox_SuspCamberAngleAtMaxCompr,	0.0f, MAX_FLOAT);
		VehicleDesc.mSuspCamberAngleAtMaxDroop	= GetFromEditBox(0.01f,		mEditBox_SuspCamberAngleAtMaxDroop,	0.0f, MAX_FLOAT);
		VehicleDesc.mGearsSwitchTime			= GetFromEditBox(0.5f,		mEditBox_GearsSwitchTime,			0.0f, MAX_FLOAT);
		VehicleDesc.mClutchStrength				= GetFromEditBox(10.0f,		mEditBox_ClutchStrength,			0.0f, MAX_FLOAT);

		if(0)
		{
			Matrix3x3 Rot;
//			Rot.RotZ(HALFPI);
			Rot.RotZ(60.0f*DEGTORAD);
			Rot.RotZ(70.0f*DEGTORAD);
			VehicleDesc.mStartPose.mRot = Rot;
		}

		VehicleDesc.mWheel.mNbVerts		= TotalNbVerts;
		VehicleDesc.mWheel.mVerts		= Verts;
		VehicleDesc.mWheel.mRenderer	= CreateConvexRenderer(VehicleDesc.mWheel.mNbVerts, VehicleDesc.mWheel.mVerts);

		// We must create the wheels in this order:
		// eFRONT_LEFT=0,
		// eFRONT_RIGHT,
		// eREAR_LEFT,
		// eREAR_RIGHT
		// Forward is along Z so front is +z, left is +x

		VehicleDesc.mWheelOffset[0] = Point( ChassisExtents.x*CoeffX, -WheelRadius,  ChassisExtents.z*CoeffZ);
		VehicleDesc.mWheelOffset[1] = Point(-ChassisExtents.x*CoeffX, -WheelRadius,  ChassisExtents.z*CoeffZ);
		VehicleDesc.mWheelOffset[2] = Point( ChassisExtents.x*CoeffX, -WheelRadius, -ChassisExtents.z*CoeffZ);
		VehicleDesc.mWheelOffset[3] = Point(-ChassisExtents.x*CoeffX, -WheelRadius, -ChassisExtents.z*CoeffZ);

		AABB ChassisBox;
		ChassisBox.SetCenterExtents(Point(0.0f, 0.0f, 0.0f), ChassisExtents);
		Point ChassisPts[8];
		ChassisBox.ComputePoints(ChassisPts);

		VehicleDesc.mChassis.mNbVerts	= 8;
		VehicleDesc.mChassis.mVerts		= ChassisPts;
		VehicleDesc.mChassis.mRenderer	= CreateConvexRenderer(VehicleDesc.mChassis.mNbVerts, VehicleDesc.mChassis.mVerts);

		VehicleSDKData* VehicleData = ICE_NEW(VehicleSDKData);
		pint.mUserData = VehicleData;

		PintVehicleData VD;
		const PintObjectHandle VehicleHandle = pint.CreateVehicle(VD, VehicleDesc);
		VehicleData->mChassis = VD.mChassis;
		VehicleData->mVehicle = VehicleHandle;

		const udword Index = mComboBox_Level->GetSelectedIndex();
		if(Index==0)
		{
			//#####
			const float Altitude = 0.0f;
			CreatePlanarMesh(*this, pint, Altitude, 200, 0.5f, null);
		}
		else if(Index==1)
		{
			//#####
			const float Altitude = 0.0f;
			CreatePlanarMesh(*this, pint, Altitude, 200, 0.5f, null);

			const float Spacing = 10.0f;
		//	const float Size = 0.3f;
			const float Size = 0.2f;
//			const float Size = 0.5f;
		//	const float Size = 1.0f;
			for(udword i=0;i<100;i++)
			{
				PINT_BOX_CREATE BoxDesc;
				BoxDesc.mExtents	= Point(100.0f, Size, Size);
//				BoxDesc.mExtents	= Point(10.0f, Size, Size);
//				BoxDesc.mExtents	= Point(10.0f, Size, 4.0f);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
		//		BoxDesc.mMaterial	= material;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition	= Point(0.0f, 0.0f, float(i)*Spacing);
				ObjectDesc.mShapes		= &BoxDesc;
				ObjectDesc.mMass		= 0.0f;

				pint.CreateObject(ObjectDesc);
			}
			for(udword i=0;i<100;i++)
			{
				PINT_BOX_CREATE BoxDesc;
//				BoxDesc.mExtents	= Point(Size, Size, 100.0f);
				BoxDesc.mExtents	= Point(Size, Size, 10.0f);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
		//		BoxDesc.mMaterial	= material;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition	= Point(float(i)*Spacing, 0.0f, 0.0f);
				ObjectDesc.mShapes		= &BoxDesc;
				ObjectDesc.mMass		= 0.0f;
				pint.CreateObject(ObjectDesc);
			}

			{
				Matrix3x3 Rot;
				Rot.RotX(25.0f*DEGTORAD);

				PINT_BOX_CREATE BoxDesc(10.0f, 1.0f, 10.0f);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition	= Point(5.0f, 0.0f, -100.0f);
				ObjectDesc.mRotation	= Rot;
				ObjectDesc.mShapes		= &BoxDesc;
				ObjectDesc.mMass		= 0.0f;
				pint.CreateObject(ObjectDesc);

/*
					Rot.RotX(-25.0f*DEGTORAD);

					ObjectDesc.mPosition	= Point(5.0f, 0.0f, -200.0f);
					ObjectDesc.mRotation	= Rot;
					pint.CreateObject(ObjectDesc);*/
			}
		}
		return true;
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		const VehicleSDKData* UserData = (const VehicleSDKData*)pint.mUserData;

		const PintObjectHandle VehicleHandle = UserData->mVehicle;
		if(!VehicleHandle)
			return 0;

		if(mCheckBox_DriveVehicle->IsChecked())
		{
			pint.SetVehicleInput(VehicleHandle, mInput);

			// Camera
			if(1 && UserData->mChassis)
			{
				// Won't work well more more than 1 engine at a time of course
				const PR Pose = pint.GetWorldTransform(UserData->mChassis);
		//		printf("%f\n", Pose.mPos.y);
		//		const Point CamPos = GetCameraPos();
		//		const Point Dir = (Pose.mPos - CamPos).Normalize();
		//		SetCamera(CamPos, Dir);

				const Matrix3x3 M(Pose.mRot);
	//			Point D = M[0];
				Point D = M[2];
				D.y = 0.0f;
				D.Normalize();

	float	mCameraUpOffset			= 3.0f;
	float	mCameraDistToTarget		= 8.0f;

				const Point Target = Pose.mPos;
				const Point CamPos = Pose.mPos + Point(0.0f, mCameraUpOffset, 0.0f) - D*mCameraDistToTarget;
		//		const Point CamPos = Pose.mPos + Point(0.0f, mCameraUpOffset, 0.0f) - Point(1.0f, 0.0f, 0.0f)*mCameraDistToTarget;

				const Point Dir = (Target - CamPos).Normalize();

	//			const float Sharpness = 0.02f;
				const float Sharpness = 0.2f;
				static Point FilteredPos(0.0f, 0.0f, 0.0f);
				static Point FilteredDir(0.0f, 0.0f, 0.0f);
				FeedbackFilter(CamPos.x, FilteredPos.x, Sharpness);
				FeedbackFilter(CamPos.y, FilteredPos.y, Sharpness);
				FeedbackFilter(CamPos.z, FilteredPos.z, Sharpness);
				FeedbackFilter(Dir.x, FilteredDir.x, Sharpness);
				FeedbackFilter(Dir.y, FilteredDir.y, Sharpness);
				FeedbackFilter(Dir.z, FilteredDir.z, Sharpness);

				Point Tmp = FilteredDir;
				Tmp.Normalize();

		//			SetCamera(CamPos, Dir);
				SetCamera(FilteredPos, Tmp);
			}
		}
		return TestBase::Update(pint, dt);
	}

	virtual udword	GetFlags() const
	{
		return mCheckBox_DriveVehicle->IsChecked() ? TEST_FLAGS_USE_CURSOR_KEYS : TEST_FLAGS_DEFAULT;
	}

	virtual bool	SpecialKeyCallback(int key, int x, int y, bool down)
	{
		if(mCheckBox_DriveVehicle->IsChecked())
		{
			switch(key)
			{
				case GLUT_KEY_UP:
				{
					mInput.mAccelerate = down;
				}
				break;
				case GLUT_KEY_DOWN:
				{
					mInput.mBrake = down;
				}
				break;
				case GLUT_KEY_LEFT:
				{
					mInput.mLeft = down;
				}
				break;
				case GLUT_KEY_RIGHT:
				{
					mInput.mRight = down;
				}
				break;
			}
		}
		return TestBase::SpecialKeyCallback(key, x, y, down);
	}

	void	SaveCarData(const char* filename)	const
	{
		FILE* fp = fopen(filename, "wb");
		if(!fp)
		{
			printf("OOPS.... fopen failed!\n");
			return;
		}

		const float WheelRadius = GetFromEditBox(0.0f, mEditBox_WheelRadius, 0.0f, MAX_FLOAT);
		const float WheelWidth = GetFromEditBox(0.0f, mEditBox_WheelWidth, 0.0f, MAX_FLOAT);
		const float WheelMass = GetFromEditBox(0.0f, mEditBox_WheelMass, 0.0f, MAX_FLOAT);
		const float WheelCoeffX = GetFromEditBox(0.0f, mEditBox_WheelCoeffX, 0.0f, MAX_FLOAT);
		const float WheelCoeffZ = GetFromEditBox(0.0f, mEditBox_WheelCoeffZ, 0.0f, MAX_FLOAT);
		const float WheelMaxBrakeTorqueFront = GetFromEditBox(0.0f, mEditBox_WheelMaxBrakeTorqueFront, 0.0f, MAX_FLOAT);
		const float WheelMaxBrakeTorqueRear = GetFromEditBox(0.0f, mEditBox_WheelMaxBrakeTorqueRear, 0.0f, MAX_FLOAT);
		const float WheelMaxSteerFront = GetFromEditBox(0.0f, mEditBox_WheelMaxSteerFront, 0.0f, MAX_FLOAT);
		const float WheelMaxSteerRear = GetFromEditBox(0.0f, mEditBox_WheelMaxSteerRear, 0.0f, MAX_FLOAT);

		Point ChassisExtents;
		ChassisExtents.x = GetFromEditBox(0.0f, mEditBox_ChassisX, 0.0f, MAX_FLOAT);
		ChassisExtents.y = GetFromEditBox(0.0f, mEditBox_ChassisY, 0.0f, MAX_FLOAT);
		ChassisExtents.z = GetFromEditBox(0.0f, mEditBox_ChassisZ, 0.0f, MAX_FLOAT);
		const float ChassisMass					= GetFromEditBox(0.0f,		mEditBox_ChassisMass,				0.0f, MAX_FLOAT);
		const float ChassisMOICoeffY			= GetFromEditBox(0.0f,		mEditBox_ChassisMOICoeffY,			0.0f, MAX_FLOAT);
		const float ChassisCMOffsetY			= GetFromEditBox(0.0f,		mEditBox_ChassisCMOffsetY,			MIN_FLOAT, MAX_FLOAT);
		const float ChassisCMOffsetZ			= GetFromEditBox(0.0f,		mEditBox_ChassisCMOffsetZ,			MIN_FLOAT, MAX_FLOAT);
		const float ForceApplicationCMOffsetY	= GetFromEditBox(0.0f,		mEditBox_ForceApplicationCMOffsetY,	0.0f, MAX_FLOAT);

		const float TireFrictionMultiplier		= GetFromEditBox(0.0f,		mEditBox_TireFrictionMultiplier,	0.0f, MAX_FLOAT);
		const float EnginePeakTorque			= GetFromEditBox(0.0f,		mEditBox_EnginePeakTorque,			0.0f, MAX_FLOAT);
		const float EngineMaxOmega				= GetFromEditBox(0.0f,		mEditBox_EngineMaxOmega,			0.0f, MAX_FLOAT);
		const float SuspMaxCompression			= GetFromEditBox(0.0f,		mEditBox_SuspMaxCompression,		0.0f, MAX_FLOAT);
		const float SuspMaxDroop				= GetFromEditBox(0.0f,		mEditBox_SuspMaxDroop,				0.0f, MAX_FLOAT);
		const float SuspSpringStrength			= GetFromEditBox(0.0f,		mEditBox_SuspSpringStrength,		0.0f, MAX_FLOAT);
		const float SuspSpringDamperRate		= GetFromEditBox(0.0f,		mEditBox_SuspSpringDamperRate,		0.0f, MAX_FLOAT);
		const float SuspCamberAngleAtRest		= GetFromEditBox(0.0f,		mEditBox_SuspCamberAngleAtRest,		0.0f, MAX_FLOAT);
		const float SuspCamberAngleAtMaxCompr	= GetFromEditBox(0.0f,		mEditBox_SuspCamberAngleAtMaxCompr,	0.0f, MAX_FLOAT);
		const float SuspCamberAngleAtMaxDroop	= GetFromEditBox(0.0f,		mEditBox_SuspCamberAngleAtMaxDroop,	0.0f, MAX_FLOAT);
		const float GearsSwitchTime				= GetFromEditBox(0.0f,		mEditBox_GearsSwitchTime,			0.0f, MAX_FLOAT);
		const float ClutchStrength				= GetFromEditBox(0.0f,		mEditBox_ClutchStrength,			0.0f, MAX_FLOAT);

		const udword Differential				= mComboBox_Differential->GetSelectedIndex();

		const udword Version = 1;
		CustomArray CA;
		CA.Store(udword('PVF!'))
			.Store(udword(Version))
		// Wheels
			.Store(WheelRadius)
			.Store(WheelWidth)
			.Store(WheelMass)
			.Store(WheelCoeffX)
			.Store(WheelCoeffZ)
			.Store(WheelMaxBrakeTorqueFront)
			.Store(WheelMaxBrakeTorqueRear)
			.Store(WheelMaxSteerFront)
			.Store(WheelMaxSteerRear)
		// Chassis
			.Store(ChassisExtents.x)
			.Store(ChassisExtents.y)
			.Store(ChassisExtents.z)
			.Store(ChassisMass)
			.Store(ChassisMOICoeffY)
			.Store(ChassisCMOffsetY)
			.Store(ChassisCMOffsetZ)
			.Store(ForceApplicationCMOffsetY)
		//
			.Store(TireFrictionMultiplier)
			.Store(EnginePeakTorque)
			.Store(EngineMaxOmega)
			.Store(SuspMaxCompression)
			.Store(SuspMaxDroop)
			.Store(SuspSpringStrength)
			.Store(SuspSpringDamperRate)
			.Store(SuspCamberAngleAtRest)
			.Store(SuspCamberAngleAtMaxCompr)
			.Store(SuspCamberAngleAtMaxDroop)
			.Store(GearsSwitchTime)
			.Store(ClutchStrength)
			.Store(Differential);

		CA.ExportToDisk(fp);

		fclose(fp);
	}

	void	LoadCarData(const char* filename)
	{
		IceFile F(filename);
		if(!F.IsValid())
		{
			printf("OOPS.... fopen failed!\n");
			return;
		}

		const udword Header = F.LoadDword();
		const udword Version = F.LoadDword();
		const float WheelRadius = F.LoadFloat();
		const float WheelWidth = F.LoadFloat();
		const float WheelMass = F.LoadFloat();
		const float WheelCoeffX = F.LoadFloat();
		const float WheelCoeffZ = F.LoadFloat();
		const float WheelMaxBrakeTorqueFront = F.LoadFloat();
		const float WheelMaxBrakeTorqueRear = F.LoadFloat();
		const float WheelMaxSteerFront = F.LoadFloat();
		const float WheelMaxSteerRear = F.LoadFloat();
		const float ChassisExtents_x = F.LoadFloat();
		const float ChassisExtents_y = F.LoadFloat();
		const float ChassisExtents_z = F.LoadFloat();
		const float ChassisMass = F.LoadFloat();
		const float ChassisMOICoeffY = F.LoadFloat();
		const float ChassisCMOffsetY = F.LoadFloat();
		const float ChassisCMOffsetZ = F.LoadFloat();
		const float ForceApplicationCMOffsetY = F.LoadFloat();
		const float TireFrictionMultiplier = F.LoadFloat();
		const float EnginePeakTorque = F.LoadFloat();
		const float EngineMaxOmega = F.LoadFloat();
		const float SuspMaxCompression = F.LoadFloat();
		const float SuspMaxDroop = F.LoadFloat();
		const float SuspSpringStrength = F.LoadFloat();
		const float SuspSpringDamperRate = F.LoadFloat();
		const float SuspCamberAngleAtRest = F.LoadFloat();
		const float SuspCamberAngleAtMaxCompr = F.LoadFloat();
		const float SuspCamberAngleAtMaxDroop = F.LoadFloat();
		const float GearsSwitchTime = F.LoadFloat();
		const float ClutchStrength = F.LoadFloat();
		const udword Differential = F.LoadDword();

		mEditBox_WheelRadius->SetText(_F("%.2f", WheelRadius));
		mEditBox_WheelWidth->SetText(_F("%.2f", WheelWidth));
		mEditBox_WheelMass->SetText(_F("%.2f", WheelMass));
		mEditBox_WheelCoeffX->SetText(_F("%.2f", WheelCoeffX));
		mEditBox_WheelCoeffZ->SetText(_F("%.2f", WheelCoeffZ));
		mEditBox_WheelMaxBrakeTorqueFront->SetText(_F("%.2f", WheelMaxBrakeTorqueFront));
		mEditBox_WheelMaxBrakeTorqueRear->SetText(_F("%.2f", WheelMaxBrakeTorqueRear));
		mEditBox_WheelMaxSteerFront->SetText(_F("%.2f", WheelMaxSteerFront));
		mEditBox_WheelMaxSteerRear->SetText(_F("%.2f", WheelMaxSteerRear));
		mEditBox_ChassisX->SetText(_F("%.2f", ChassisExtents_x));
		mEditBox_ChassisY->SetText(_F("%.2f", ChassisExtents_y));
		mEditBox_ChassisZ->SetText(_F("%.2f", ChassisExtents_z));
		mEditBox_ChassisMass->SetText(_F("%.2f", ChassisMass));
		mEditBox_ChassisMOICoeffY->SetText(_F("%.2f", ChassisMOICoeffY));
		mEditBox_ChassisCMOffsetY->SetText(_F("%.2f", ChassisCMOffsetY));
		mEditBox_ChassisCMOffsetZ->SetText(_F("%.2f", ChassisCMOffsetZ));
		mEditBox_ForceApplicationCMOffsetY->SetText(_F("%.2f", ForceApplicationCMOffsetY));
		mEditBox_TireFrictionMultiplier->SetText(_F("%.2f", TireFrictionMultiplier));
		mEditBox_EnginePeakTorque->SetText(_F("%.2f", EnginePeakTorque));
		mEditBox_EngineMaxOmega->SetText(_F("%.2f", EngineMaxOmega));
		mEditBox_SuspMaxCompression->SetText(_F("%.2f", SuspMaxCompression));
		mEditBox_SuspMaxDroop->SetText(_F("%.2f", SuspMaxDroop));
		mEditBox_SuspSpringStrength->SetText(_F("%.2f", SuspSpringStrength));
		mEditBox_SuspSpringDamperRate->SetText(_F("%.2f", SuspSpringDamperRate));
		mEditBox_SuspCamberAngleAtRest->SetText(_F("%.2f", SuspCamberAngleAtRest));
		mEditBox_SuspCamberAngleAtMaxCompr->SetText(_F("%.2f", SuspCamberAngleAtMaxCompr));
		mEditBox_SuspCamberAngleAtMaxDroop->SetText(_F("%.2f", SuspCamberAngleAtMaxDroop));
		mEditBox_GearsSwitchTime->SetText(_F("%.2f", GearsSwitchTime));
		mEditBox_ClutchStrength->SetText(_F("%.2f", ClutchStrength));
		mComboBox_Differential->Select(Differential);
		mMustResetTest = true;
	}

}VehicleSDK_;

static void gButtonCallback(IceButton& button, void* user_data)
{
	if(gRoot)
	{
		FILESELECTCREATE Create;
		Create.mFilter			= "PEEL vehicle files (*.pvf)|*.pvf|All Files (*.*)|*.*||";
		Create.mFileName		= "DefaultCar.pvf";
		Create.mInitialDir		= *gRoot;
		Create.mCaptionTitle	= "Select file";
		Create.mDefExt			= "pvf";

		String Filename;
		if(button.GetID()==0)
		{
			if(!FileselectSave(Create, Filename, true))
				return;

			VehicleSDK* T = (VehicleSDK*)button.GetUserData();
			T->SaveCarData(Filename);
		}
		else
		{
			if(!FileselectOpenSingle(Create, Filename))
				return;

			VehicleSDK* T = (VehicleSDK*)button.GetUserData();
			T->LoadCarData(Filename);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

