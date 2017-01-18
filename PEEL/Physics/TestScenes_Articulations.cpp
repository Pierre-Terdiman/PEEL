///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Camera.h"
#include "Cylinder.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "Loader_Bin.h"
#include "ProceduralTrack.h"
#include "MyConvex.h"
#include "GUI_Helpers.h"

///////////////////////////////////////////////////////////////////////////////

	enum ChainDataUIIndex
	{
		CHAIN_DATA_NB_SPHERES,
		CHAIN_DATA_RADIUS,
		CHAIN_DATA_GAP,
		CHAIN_DATA_MASS,
		CHAIN_DATA_CENTERED_PIVOTS,
		CHAIN_DATA_USE_DISTANCE_CONSTAINTS,
		CHAIN_DATA_USE_LARGER_INERTIA,
		CHAIN_DATA_ATTACH_HEAVY_BOX,
		CHAIN_DATA_USE_ARTICULATION,
		CHAIN_DATA_FIRST_SPHERE_IS_STATIC,
	};

	struct ChainData
	{
		ChainData() :
			mEditBox_NbSimCalls			(null),
			mEditBox_NbSpheres			(null),
			mEditBox_Radius				(null),
			mEditBox_Gap				(null),
			mEditBox_Mass				(null),
			mCenteredPivots				(false),
			mUseExtraDistanceConstraints(false),
			mUseLargerInertia			(false),
			mAttachHeavyBox				(false),
			mUseArticulation			(false),
			mFirstSphereIsStatic		(false)
			{}
		IceEditBox*		mEditBox_NbSimCalls;
		IceEditBox*		mEditBox_NbSpheres;
		IceEditBox*		mEditBox_Radius;
		IceEditBox*		mEditBox_Gap;
		IceEditBox*		mEditBox_Mass;
		bool			mCenteredPivots;
		bool			mUseExtraDistanceConstraints;
		bool			mUseLargerInertia;
		bool			mAttachHeavyBox;
		bool			mUseArticulation;
		bool			mFirstSphereIsStatic;
	};

static const char* gDesc_ConfigurableSphericalChain = "(Configurable test) - Spherical chain. This test is similar to the StableSphericalChain test. \
It demonstrates various ways to make a rope/chain more robust, and lets you play with the settings to see the effects on stability. It also supports articulations. \
There are some additional options like the 'inverse inertia scale' settings in PhysX that are not exposed in the test UI itself, and must/can be tweaked \
in the per-engine UI.";

START_TEST(ConfigurableSphericalChain, CATEGORY_ARTICULATIONS, gDesc_ConfigurableSphericalChain)

	ChainData	mData;

	static void CheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
	{
		ChainData* CD = (ChainData*)user_data;

		switch(check_box.GetID())
		{
			case CHAIN_DATA_CENTERED_PIVOTS:
				CD->mCenteredPivots = checked;
				break;
			case CHAIN_DATA_USE_DISTANCE_CONSTAINTS:
				CD->mUseExtraDistanceConstraints = checked;
				break;
			case CHAIN_DATA_USE_LARGER_INERTIA:
				CD->mUseLargerInertia = checked;
				break;
			case CHAIN_DATA_ATTACH_HEAVY_BOX:
				CD->mAttachHeavyBox = checked;
				break;
			case CHAIN_DATA_USE_ARTICULATION:
				CD->mUseArticulation = checked;
				break;
			case CHAIN_DATA_FIRST_SPHERE_IS_STATIC:
				CD->mFirstSphereIsStatic = checked;
				break;
		}
	}

	virtual	void	InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 450;
		WD.mHeight	= 320;
		WD.mLabel	= "ConfigurableSphericalChain";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		const sdword EditBoxWidth = 60;
		const sdword LabelSize = 240;

		{
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelSize, 20, "Nb sim calls/frame:", UIElems);
			mData.mEditBox_NbSimCalls = helper.CreateEditBox(UI, 1, 4+LabelSize, y, EditBoxWidth, 20, "4", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelSize, 20, "Nb spheres (more spheres is less stable):", UIElems);
			mData.mEditBox_NbSpheres = helper.CreateEditBox(UI, CHAIN_DATA_NB_SPHERES, 4+LabelSize, y, EditBoxWidth, 20, "64", UIElems, EDITBOX_INTEGER_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelSize, 20, "Sphere radius (larger radius is more stable):", UIElems);
			mData.mEditBox_Radius = helper.CreateEditBox(UI, CHAIN_DATA_RADIUS, 4+LabelSize, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelSize, 20, "Sphere mass:", UIElems);
			mData.mEditBox_Mass = helper.CreateEditBox(UI, CHAIN_DATA_MASS, 4+LabelSize, y, EditBoxWidth, 20, "1.0", UIElems, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelSize, 20, "Gap between spheres:", UIElems);
			mData.mEditBox_Gap = helper.CreateEditBox(UI, CHAIN_DATA_GAP, 4+LabelSize, y, EditBoxWidth, 20, "0.0", UIElems, EDITBOX_FLOAT_POSITIVE, null);
			y += YStep;
		}

		{
			y += YStep;

			IceCheckBox* CB;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_CENTERED_PIVOTS, 4, y, 400, 20, "Place pivots at sphere centers (more stable).", UIElems, mData.mCenteredPivots, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_USE_DISTANCE_CONSTAINTS, 4, y, 400, 20, "Use extra distance constraints between spheres (more stable)", UIElems, mData.mUseExtraDistanceConstraints, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_USE_LARGER_INERTIA, 4, y, 400, 20, "Make the inertia tensor artificially larger (can be more stable)", UIElems, mData.mUseLargerInertia, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_ATTACH_HEAVY_BOX, 4, y, 400, 20, "Attach heavy box (100X sphere mass) to last sphere (less stable)", UIElems, mData.mAttachHeavyBox, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_USE_ARTICULATION, 4, y, 400, 20, "Use dedicated articulation feature (more stable)", UIElems, mData.mUseArticulation, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;

			CB = helper.CreateCheckBox(UI, CHAIN_DATA_FIRST_SPHERE_IS_STATIC, 4, y, 400, 20, "Make first sphere static (else attach it to static anchor)", UIElems, mData.mFirstSphereIsStatic, ConfigurableSphericalChain::CheckBoxCallback, null);
			CB->SetUserData(&mData);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 450-16);
	}

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.92f, -37.17f, 88.77f), Point(-0.13f, 0.07f, -0.99f));
		desc.mCamera[1] = CameraPose(Point(1.04f, -1.68f, 6.24f), Point(-0.20f, -0.09f, -0.98f));
		desc.mNbSimulateCallsPerFrame = GetFromEditBox(4, mData.mEditBox_NbSimCalls);
//		desc.mTimestep = (1.0f/60.0f)/desc.mNbSimulateCallsPerFrame;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportSphericalJoints)
			return false;
		if(mData.mUseExtraDistanceConstraints && !caps.mSupportDistanceJoints)
			return false;
		if(mData.mUseArticulation && !caps.mSupportArticulations)
			return false;
		if(mData.mUseLargerInertia && !caps.mSupportMassForInertia)
			return false;

		// Filtering is used to disable collisions between two jointed objects.
		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG(1, 2);
			pint.SetDisabledGroups(1, &DG);
		}

		const udword NbSpheres = GetFromEditBox(0, mData.mEditBox_NbSpheres);
		if(!NbSpheres)
			return true;

		const float Radius = GetFromEditBox(0.5f, mData.mEditBox_Radius, 0.0f, MAX_FLOAT);
		const float Mass = GetFromEditBox(1.0f, mData.mEditBox_Mass, 0.0f, MAX_FLOAT);
		const float Gap = GetFromEditBox(0.5f, mData.mEditBox_Gap, 0.0f, MAX_FLOAT);

		const Point InitPos(0.0f, 0.0f, 0.0f);
		Point Pos = InitPos;
		PintObjectHandle* Handles = new PintObjectHandle[NbSpheres];
		Point* Positions = ICE_NEW(Point)[NbSpheres];

		PintObjectHandle Articulation = mData.mUseArticulation ? pint.CreateArticulation(PINT_ARTICULATION_CREATE()) : null;

		udword GroupBit = 0;
		{
			PINT_SPHERE_CREATE SphereDesc(Radius);
			SphereDesc.mRenderer	= CreateSphereRenderer(Radius);

			const Point Offset((Radius+Gap)*2.0f, 0.0f, 0.0f);

			Point LocalPivot0, LocalPivot1;
			if(mData.mCenteredPivots)
			{
				LocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				LocalPivot1	= Point(-(Radius+Gap)*2.0f, 0.0f, 0.0f);
			}
			else
			{
				LocalPivot0	= Point(Radius+Gap, 0.0f, 0.0f);
				LocalPivot1	= Point(-Radius-Gap, 0.0f, 0.0f);
			}

			for(udword i=0;i<NbSpheres;i++)
			{
				Positions[i] = Pos;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes		= &SphereDesc;
				ObjectDesc.mMass		= Mass;
				if(mData.mFirstSphereIsStatic && i==0)
					ObjectDesc.mMass	= 0.0f;

				if(mData.mUseLargerInertia)
					ObjectDesc.mMassForInertia = Mass*10.0f;
				ObjectDesc.mPosition	= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;

				if(Articulation)
				{
					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					ArticulatedDesc.mParent = i ? Handles[i-1] : null;
					ArticulatedDesc.mLocalPivot0 = LocalPivot0;
					ArticulatedDesc.mLocalPivot1 = LocalPivot1;
					Handles[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
				}
				else
					Handles[i] = CreatePintObject(pint, ObjectDesc);

				Pos += Offset;
			}

			if(!Articulation)
			{
				for(udword i=0;i<NbSpheres-1;i++)
				{
					PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i], Handles[i+1], LocalPivot0, LocalPivot1));
					ASSERT(JointHandle);
				}
			}

			if(mData.mUseExtraDistanceConstraints)
			{
				for(udword i=0;i<NbSpheres;i++)
				{
					if(i+2<NbSpheres)
					{
						PINT_DISTANCE_JOINT_CREATE Desc;
						Desc.mObject0		= Handles[i];
						Desc.mObject1		= Handles[i+2];
						Desc.mMaxDistance	= Positions[i].Distance(Positions[i+2]);
						PintJointHandle JointHandle = pint.CreateJoint(Desc);
						ASSERT(JointHandle);
					}
				}
/*				for(udword i=0;i<NbSpheres-1;i++)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+1];
					Desc.mMaxDistance	= Positions[i].Distance(Positions[i+1]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}*/
			}
		}
		if(Articulation)
			pint.AddArticulationToScene(Articulation);

		// Attach first sphere to static world.
		if(!mData.mFirstSphereIsStatic)
		{
			PINT_BOX_CREATE BoxDesc(0.1f, 0.1f, 0.1f);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition		= InitPos;
			ObjectDesc.mCollisionGroup	= 2;
			PintObjectHandle h = pint.CreateObject(ObjectDesc);

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(h, Handles[0], Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
			ASSERT(JointHandle);
		}

		// Attach heavy box to last sphere
		if(mData.mAttachHeavyBox)
		{
			PintObjectHandle HeavyBox;
			const Point BoxExtents(Radius*10.0f, Radius*10.0f, Radius*10.0f);
			{
				PINT_BOX_CREATE BoxDesc(BoxExtents);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxExtents);

				Pos.x += BoxExtents.x - Radius;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= Mass * 100.0f;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
				HeavyBox = CreatePintObject(pint, ObjectDesc);
			}

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[NbSpheres-1], HeavyBox, Point(Radius, 0.0f, 0.0f), Point(-BoxExtents.x, 0.0f, 0.0f)));
			ASSERT(JointHandle);
		}

		DELETEARRAY(Positions);
		DELETEARRAY(Handles);

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(ConfigurableSphericalChain)

///////////////////////////////////////////////////////////////////////////////

	struct ConfigurableArticulationDefault
	{
				ConfigurableArticulationDefault() :
				mNbSimCallsPerRenderFrame	(1),
				mNbLinks					(0),
				mCapsuleRadius				(0.0f),
				mCapsuleMass				(0.0f),
				mCapsuleMassForInertia		(0.0f),
				mBoxMass					(0.0f),
				mInitialAngularVelocity		(0.0f),
				mUseArticulations			(false),
				mUseExtraDistanceConstraints(false)
				{}
		udword	mNbSimCallsPerRenderFrame;
		udword	mNbLinks;
		float	mCapsuleRadius;
		float	mCapsuleMass;
		float	mCapsuleMassForInertia;
		float	mBoxMass;
		float	mInitialAngularVelocity;
		bool	mUseArticulations;
		bool	mUseExtraDistanceConstraints;
	};

	class ConfigurableArticulationTest : public TestBase
	{
		protected:
				ConfigurableArticulationDefault	mValues;
				IceCheckBox*	mCheckBox_Articulations;
				IceCheckBox*	mCheckBox_DistanceConstraints;
				IceCheckBox*	mCheckBox_OverlappingLinks;
				IceEditBox*		mEditBox_NbSimCalls;
				IceEditBox*		mEditBox_NbLinks;
				IceEditBox*		mEditBox_CapsuleMass;
				IceEditBox*		mEditBox_CapsuleMassForInertia;
				IceEditBox*		mEditBox_CapsuleRadius;
				IceEditBox*		mEditBox_BoxMass;
				IceEditBox*		mEditBox_InitialAngularVel;
		public:
								ConfigurableArticulationTest() :
									mCheckBox_Articulations			(null),
									mCheckBox_DistanceConstraints	(null),
									mCheckBox_OverlappingLinks		(null),
									mEditBox_NbSimCalls				(null),
									mEditBox_NbLinks				(null),
									mEditBox_CapsuleMass			(null),
									mEditBox_CapsuleMassForInertia	(null),
									mEditBox_CapsuleRadius			(null),
									mEditBox_BoxMass				(null),
									mEditBox_InitialAngularVel		(null)
																	{}
		virtual					~ConfigurableArticulationTest()		{}

		virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
		{
			TestBase::GetSceneParams(desc);
			desc.mNbSimulateCallsPerFrame = GetFromEditBox(mValues.mNbSimCallsPerRenderFrame, mEditBox_NbSimCalls);
		}

		virtual	void			InitUI(PintGUIHelper& helper)
		{
			WindowDesc WD;
			WD.mParent	= null;
			WD.mX		= 50;
			WD.mY		= 50;
			WD.mWidth	= 300;
			WD.mHeight	= 280;
			WD.mLabel	= GetName();
			WD.mType	= WINDOW_DIALOG;
			IceWindow* UI = ICE_NEW(IceWindow)(WD);
			RegisterUIElement(UI);
			UI->SetVisible(true);

			Container* UIElems = GetUIElements();

			const sdword EditBoxWidth = 60;
			const sdword LabelWidth = 140;
			const sdword OffsetX = LabelWidth + 10;
			const sdword LabelOffsetY = 2;
			const sdword YStep = 20;
			sdword y = 10;
			{
				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Nb sim calls/frame:", UIElems);
				mEditBox_NbSimCalls = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", mValues.mNbSimCallsPerRenderFrame), UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
				y += YStep;

				mCheckBox_Articulations = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Use articulations", UIElems, mValues.mUseArticulations, null, null);
				y += YStep;

				mCheckBox_DistanceConstraints = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Use extra distance constraints", UIElems, mValues.mUseExtraDistanceConstraints, null, null);
				y += YStep;

	//			mCheckBox_OverlappingLinks = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Create overlapping capsules", UIElems, false, null, null);
	//			y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Nb links:", UIElems);
				mEditBox_NbLinks = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", mValues.mNbLinks), UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
				y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Capsule radius:", UIElems);
				mEditBox_CapsuleRadius = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%.2f", mValues.mCapsuleRadius), UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
				y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Capsule mass:", UIElems);
				mEditBox_CapsuleMass = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%.2f", mValues.mCapsuleMass), UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
				y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Capsule mass for inertia:", UIElems);
				mEditBox_CapsuleMassForInertia = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%.2f", mValues.mCapsuleMassForInertia), UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
				y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Box mass:", UIElems);
				mEditBox_BoxMass = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%.2f", mValues.mBoxMass), UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
				y += YStep;

				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Initial angular velocity:", UIElems);
				mEditBox_InitialAngularVel = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, _F("%.2f", mValues.mInitialAngularVelocity), UIElems, EDITBOX_FLOAT, null, null);
				y += YStep;
			}

			y += YStep;
			AddResetButton(UI, 4, y, 300-16);
		}
	};

///////////////////////////////////////////////////////////////////////////////

static void AttachLink(PintObjectHandle h, Pint& pint, const Point& pos, float radius, float half_height)
{
	PINT_BOX_CREATE BoxDesc(0.01f, 0.01f, 0.01f);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mPosition	= pos;
	ObjectDesc.mShapes		= &BoxDesc;
	ObjectDesc.mMass		= 0.0f;
//	ObjectDesc.mCollisionGroup	= 1 + GroupBit;

	PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(pint.CreateObject(ObjectDesc), h, Point(0.0f, 0.0f, 0.0f), Point(0.0f, -half_height - radius, 0.0f)));
	ASSERT(JointHandle);
}

static void CreateChain(Pint& pint, bool use_articulations, bool use_overlapping_links, bool use_distance_constraints,
						const Point& p0, const Point& p1, udword nb_links, float radius, float mass, float mass_for_inertia,
						PintObjectHandle heavy_object, const Point& local_pivot0)
{
	PintObjectHandle Handles[MAX_LINKS];
	float HalfHeight;
	{
		PintObjectHandle Articulation = use_articulations ? pint.CreateArticulation(PINT_ARTICULATION_CREATE()) : null;
		CreateCapsuleRope2(pint, HalfHeight, Articulation, null, p0, p1, nb_links, radius, mass, mass_for_inertia, use_overlapping_links, use_distance_constraints, Handles);
		if(Articulation)
			pint.AddArticulationToScene(Articulation);
	}
	AttachLink(Handles[0], pint, p0, radius, HalfHeight);

	PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(heavy_object, Handles[nb_links-1], local_pivot0, Point(0.0f, HalfHeight+radius, 0.0f)));
	ASSERT(JointHandle);
}

static const char* gDesc_SwingSet = "(Configurable test) - a test inspired by a swing set. We all did that on the playground as kids!... This is not \
really a swing set though: the capsules each have a mass of 1 and the box has a mass of 2000.";
// Repro for regression where 2.8.4 behaves MUCH better than 3.x
class SwingSet : public ConfigurableArticulationTest
{
	public:
							SwingSet()
							{
								mValues.mNbSimCallsPerRenderFrame		= 4;
								mValues.mNbLinks						= 32;
								mValues.mCapsuleRadius					= 0.25f;
								mValues.mCapsuleMass					= 1.0f;
								mValues.mCapsuleMassForInertia			= 10.0;
								mValues.mBoxMass						= 2000.0;
								mValues.mInitialAngularVelocity			= 2.0;
								mValues.mUseArticulations				= true;
								mValues.mUseExtraDistanceConstraints	= true;
							}
	virtual					~SwingSet()					{									}
	virtual	const char*		GetName()			const	{ return "SwingSet";				}
	virtual	const char*		GetDescription()	const	{ return gDesc_SwingSet;			}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_ARTICULATIONS;	}

	virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		ConfigurableArticulationTest::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-28.59f, 36.09f, 99.17f), Point(0.42f, -0.02f, -0.91f));
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const bool UseArticulations = mCheckBox_Articulations ? mCheckBox_Articulations->IsChecked() : mValues.mUseArticulations;
		const bool UseDistanceConstraints = mCheckBox_DistanceConstraints ? mCheckBox_DistanceConstraints->IsChecked() : mValues.mUseExtraDistanceConstraints;
		const bool UseOverlappingLinks = mCheckBox_OverlappingLinks ? mCheckBox_OverlappingLinks->IsChecked() : false;
		const float AngularVel = GetFromEditBox(mValues.mInitialAngularVelocity, mEditBox_InitialAngularVel, MIN_FLOAT, MAX_FLOAT);
		const float BoxMass = GetFromEditBox(mValues.mBoxMass, mEditBox_BoxMass, 0.0f, MAX_FLOAT);
		const float CapsuleMass = GetFromEditBox(mValues.mCapsuleMass, mEditBox_CapsuleMass, 0.0f, MAX_FLOAT);
		const float CapsuleMassForInertia = GetFromEditBox(mValues.mCapsuleMassForInertia, mEditBox_CapsuleMassForInertia, 0.0f, MAX_FLOAT);
		const float CapsuleRadius = GetFromEditBox(mValues.mCapsuleRadius, mEditBox_CapsuleRadius, 0.0f, MAX_FLOAT);
		const udword NbLinks = GetFromEditBox(mValues.mNbLinks, mEditBox_NbLinks);
		if(UseArticulations && !caps.mSupportArticulations)
			return false;
		if(UseDistanceConstraints && !caps.mSupportDistanceJoints)
			return false;

		if(CapsuleMass!=CapsuleMassForInertia && !caps.mSupportMassForInertia)
			printf(_F("WARNING: %s doesn't support 'mass for inertia', feature is ignored.\n", pint.GetName()));

//		const float Scale = 0.2f;
		const float Scale = 1.0f;
		const float x = 0.0f;
		const float y = 0.0f;
		const float z = 0.0f;
		const float h = 100.0f * Scale;
		const float ex = 10.0f * Scale;
		const float ey = 4.0f * Scale;
		const float ez = 4.0f * Scale;
		const float Radius = CapsuleRadius * Scale;

		PINT_BOX_CREATE BoxDesc(ex, ey, ez);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mPosition		= Point(0.0f, y - BoxDesc.mExtents.y, 0.0f);
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= BoxMass;
	//	ObjectDesc.mCollisionGroup	= 1 + GroupBit;
		ObjectDesc.mAngularVelocity	= Point(0.0f, AngularVel, 0.0f);
		const PintObjectHandle HeavyObject = pint.CreateObject(ObjectDesc);

		const bool b0 = UseArticulations;
		const bool b1 = UseOverlappingLinks;
		const bool b2 = UseDistanceConstraints;

		CreateChain(pint, b0, b1, b2, Point(x-ex, y+h, z-ez), Point(x-ex, y, z-ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, HeavyObject, Point(-ex, BoxDesc.mExtents.y, -ez));
		CreateChain(pint, b0, b1, b2, Point(x+ex, y+h, z-ez), Point(x+ex, y, z-ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, HeavyObject, Point(ex, BoxDesc.mExtents.y, -ez));
		CreateChain(pint, b0, b1, b2, Point(x-ex, y+h, z+ez), Point(x-ex, y, z+ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, HeavyObject, Point(-ex, BoxDesc.mExtents.y, ez));
		CreateChain(pint, b0, b1, b2, Point(x+ex, y+h, z+ez), Point(x+ex, y, z+ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, HeavyObject, Point(ex, BoxDesc.mExtents.y, ez));

		mCreateDefaultEnvironment = false;
		return true;
	}

}SwingSet;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Crane = "(Configurable test) - Crane test.";
class Crane : public ConfigurableArticulationTest
{
	public:
							Crane()
							{
								mValues.mNbSimCallsPerRenderFrame		= 1;
								mValues.mNbLinks						= 11;
								mValues.mCapsuleRadius					= 0.1f;
								mValues.mCapsuleMass					= 1.0f;
								mValues.mCapsuleMassForInertia			= 10.0;
								mValues.mBoxMass						= 1000.0;
								mValues.mInitialAngularVelocity			= 0.0;
								mValues.mUseArticulations				= true;
								mValues.mUseExtraDistanceConstraints	= true;
							}
	virtual					~Crane()					{									}
	virtual	const char*		GetName()			const	{ return "Crane";					}
	virtual	const char*		GetDescription()	const	{ return gDesc_Crane;				}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_ARTICULATIONS;	}

	virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		ConfigurableArticulationTest::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(-13.92f, 32.40f, 87.53f), Point(0.20f, 0.01f, -0.98f));
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const float x = 0.0f;
		const float y = 0.0f;
		const float z = 0.0f;
		const float h = 50.0f;
		const float ex = 10.0f;
		const float ey = 4.0f;
		const float ez = 4.0f;
		const float ex2 = 0.0f;
		const float ez2 = 0.0f;

		PintObjectHandle Handles[MAX_LINKS];
		PintCollisionGroup CollisionGroups[MAX_LINKS];
		float HalfHeight;

		const bool UseArticulations = mCheckBox_Articulations ? mCheckBox_Articulations->IsChecked() : mValues.mUseArticulations;
		const bool UseDistanceConstraints = mCheckBox_DistanceConstraints ? mCheckBox_DistanceConstraints->IsChecked() : mValues.mUseExtraDistanceConstraints;
		const bool UseOverlappingLinks = mCheckBox_OverlappingLinks ? mCheckBox_OverlappingLinks->IsChecked() : false;
		const float AngularVel = GetFromEditBox(mValues.mInitialAngularVelocity, mEditBox_InitialAngularVel, MIN_FLOAT, MAX_FLOAT);
		const float BoxMass = GetFromEditBox(mValues.mBoxMass, mEditBox_BoxMass, 0.0f, MAX_FLOAT);
		const float CapsuleMass = GetFromEditBox(mValues.mCapsuleMass, mEditBox_CapsuleMass, 0.0f, MAX_FLOAT);
		const float CapsuleMassForInertia = GetFromEditBox(mValues.mCapsuleMassForInertia, mEditBox_CapsuleMassForInertia, 0.0f, MAX_FLOAT);
		const float Radius = GetFromEditBox(mValues.mCapsuleRadius, mEditBox_CapsuleRadius, 0.0f, MAX_FLOAT);
		const udword NbLinks = GetFromEditBox(mValues.mNbLinks, mEditBox_NbLinks);

		if(UseArticulations && !caps.mSupportArticulations)
			return false;

		if(UseDistanceConstraints && !caps.mSupportDistanceJoints)
			return false;

		if(CapsuleMass!=CapsuleMassForInertia && !caps.mSupportMassForInertia)
			printf(_F("WARNING: %s doesn't support 'mass for inertia', feature is ignored.\n", pint.GetName()));

		if(1)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			if(0)
			{
				// We need to do two separate things here:
				//
				// 1) disable collisions between elements N and N+1. That's because some engines
				// do not disable collisions between jointed objects automatically, so we have to
				// do it explicitly (as in other PEEL scenes).
				//
				// 2) we need to disable collisions at the point where the 5 different chains are
				// connected. The elements there are not all connected by joints, but they will be
				// overlapping eachother constantly. And contacts there will make things jitter.
				//
				// We're going to use the following collision groups:
				//
				//            31212121212
				// 21212121213
				//            31212121212
				//
				// - the regular chain elements have groups 1 and 2
				// - the elements at the junction are in group 3
				// - CD between 1 and 2 is disabled, as usual
				// - CD within group 3 is disabled
				// - CD between 1/2 and 3 is disabled
			}
			else
			{
				// However since the ropes cannot actually touch eachother in this test (contrary to
				// the swing set scene), we can do something simpler: just disable CD between all links...
				for(udword i=0;i<NbLinks;i++)
					CollisionGroups[i] = 1;
				const PintDisabledGroups DG(1, 1);
				pint.SetDisabledGroups(1, &DG);
			}
		}

		PINT_BOX_CREATE BoxDesc(ex, ey, ez);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mPosition		= Point(0.0f, y - BoxDesc.mExtents.y, 0.0f);
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= BoxMass;
		ObjectDesc.mCollisionGroup	= 1;
		ObjectDesc.mAngularVelocity = Point(0.0f, AngularVel, 0.0f);
		PintObjectHandle HeavyObject = pint.CreateObject(ObjectDesc);

		PintObjectHandle Articulation = UseArticulations ? pint.CreateArticulation(PINT_ARTICULATION_CREATE()) : null;

		const Point BasePos(x, y+h, z);
		const Point TopPos(x, y+h+h, z);
		CreateCapsuleRope2(pint, HalfHeight, Articulation, null, TopPos, BasePos, NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, false, UseDistanceConstraints, Handles, CollisionGroups);
		AttachLink(Handles[0], pint, TopPos, Radius, HalfHeight);

		PintObjectHandle Parent = Handles[NbLinks-1];

		const float Offset = 1.0f;

		Point RootPos[4];
		PintObjectHandle Root[4];
		{
			RootPos[0] = Point(x-ex2, y+h, z-ez2);
			CreateCapsuleRope2(pint, HalfHeight, Articulation, Parent, RootPos[0], Point(x-ex, y, z-ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, false, UseDistanceConstraints, Handles, CollisionGroups);
			Root[0] = Handles[0];

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(HeavyObject, Handles[NbLinks-1], Point(-ex*Offset, BoxDesc.mExtents.y, -ez*Offset), Point(0.0f, HalfHeight+Radius, 0.0f)));
			ASSERT(JointHandle);
		}

		{
			RootPos[1] = Point(x+ex2, y+h, z-ez2);
			CreateCapsuleRope2(pint, HalfHeight, Articulation, Parent, RootPos[1], Point(x+ex, y, z-ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, false, UseDistanceConstraints, Handles, CollisionGroups);
			Root[1] = Handles[0];

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(HeavyObject, Handles[NbLinks-1], Point(ex*Offset, BoxDesc.mExtents.y, -ez*Offset), Point(0.0f, HalfHeight+Radius, 0.0f)));
			ASSERT(JointHandle);
		}

		{
			RootPos[2] = Point(x-ex2, y+h, z+ez2);
			CreateCapsuleRope2(pint, HalfHeight, Articulation, Parent, RootPos[2], Point(x-ex, y, z+ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, false, UseDistanceConstraints, Handles, CollisionGroups);
			Root[2] = Handles[0];

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(HeavyObject, Handles[NbLinks-1], Point(-ex*Offset, BoxDesc.mExtents.y, ez*Offset), Point(0.0f, HalfHeight+Radius, 0.0f)));
			ASSERT(JointHandle);
		}

		{
			RootPos[3] = Point(x+ex2, y+h, z+ez2);
			CreateCapsuleRope2(pint, HalfHeight, Articulation, Parent, RootPos[3], Point(x+ex, y, z+ez), NbLinks, Radius, CapsuleMass, CapsuleMassForInertia, false, UseDistanceConstraints, Handles, CollisionGroups);
			Root[3] = Handles[0];

			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(HeavyObject, Handles[NbLinks-1], Point(ex*Offset, BoxDesc.mExtents.y, ez*Offset), Point(0.0f, HalfHeight+Radius, 0.0f)));
			ASSERT(JointHandle);
		}

/*		for(udword j=0;j<4;j++)
		{
			for(udword i=j+1;i<4;i++)
			{
				PINT_DISTANCE_JOINT_CREATE Desc;
				Desc.mObject0	= Root[j];
				Desc.mObject1	= Root[i];
				Desc.mDistance	= RootPos[j].Distance(RootPos[i]);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}*/
		if(Articulation)
			pint.AddArticulationToScene(Articulation);

		mCreateDefaultEnvironment = false;
		return true;
	}

}Crane;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ArticulationDriveTest = "(Configurable test) - Articulation drive test. This one is mainly to get a feeling for the articulation drive \
parameters in PhysX. Pick and drag the rotating object with the mouse to see how it reacts to external forces. This test also shows how to setup the articulation \
to emulate a hinge joint.";

	enum ArticulationDriveType
	{
		DRIVE_VELOCITY,
		DRIVE_POSITION,
	};

	struct WheelTestData
	{
		WheelTestData() :
			mEditBox_TargetVelocity		(null),
			mEditBox_TargetPosition		(null),
			mEditBox_Stiffness			(null),
			mEditBox_Damping			(null),
			mEditBox_InternalCompliance	(null),
			mEditBox_ExternalCompliance	(null),
			mComboBox_DriveType			(null),
			mDriveType					(DRIVE_VELOCITY)
			{}
		IceEditBox*				mEditBox_TargetVelocity;
		IceEditBox*				mEditBox_TargetPosition;
		IceEditBox*				mEditBox_Stiffness;
		IceEditBox*				mEditBox_Damping;
		IceEditBox*				mEditBox_InternalCompliance;
		IceEditBox*				mEditBox_ExternalCompliance;
		IceComboBox*			mComboBox_DriveType;
		ArticulationDriveType	mDriveType;
	};

	class DriveComboBox : public IceComboBox
	{
		public:
						DriveComboBox(const ComboBoxDesc& desc, WheelTestData* data) : IceComboBox(desc), mData(data)	{}

		virtual	void	OnComboBoxEvent(ComboBoxEvent event)
						{
							if(event==CBE_SELECTION_CHANGED)
							{
								mData->mDriveType = ArticulationDriveType(GetSelectedIndex());
								if(mData->mDriveType==DRIVE_VELOCITY)
								{
									mData->mEditBox_TargetVelocity->SetEnabled(true);
									mData->mEditBox_TargetPosition->SetEnabled(false);
									mData->mEditBox_TargetVelocity->SetLabel("2");
									mData->mEditBox_TargetPosition->SetLabel("0");
									mData->mEditBox_Stiffness->SetLabel("0");
									mData->mEditBox_Damping->SetLabel("1000");
									mData->mEditBox_InternalCompliance->SetLabel("1");
									mData->mEditBox_ExternalCompliance->SetLabel("1");
								}
								else
								{
									mData->mEditBox_TargetVelocity->SetEnabled(false);
									mData->mEditBox_TargetPosition->SetEnabled(true);
									mData->mEditBox_TargetVelocity->SetLabel("0");
									mData->mEditBox_TargetPosition->SetLabel("45");
									mData->mEditBox_Stiffness->SetLabel("1000");
									mData->mEditBox_Damping->SetLabel("200");
									mData->mEditBox_InternalCompliance->SetLabel("1");
									mData->mEditBox_ExternalCompliance->SetLabel("1");
								}
							}
						}

				WheelTestData*	mData;
	};

START_TEST(ArticulationDriveTest, CATEGORY_ARTICULATIONS, gDesc_ArticulationDriveTest)

	WheelTestData	mData;

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		udword ID = 0;

		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 250;
		WD.mLabel	= "Articulation drive";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword OffsetX = 120;
		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 120;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			y += YStep;
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Drive type:", UIElems);

			ComboBoxDesc CBBD;
			CBBD.mID		= ID++;
			CBBD.mParent	= UI;
			CBBD.mX			= 4+OffsetX;
			CBBD.mY			= y;
			CBBD.mWidth		= 150;
			CBBD.mHeight	= 20;
			CBBD.mLabel		= "Drive type";
			mData.mComboBox_DriveType = ICE_NEW(DriveComboBox)(CBBD, &mData);
			RegisterUIElement(mData.mComboBox_DriveType);
			mData.mComboBox_DriveType->Add("Velocity");
			mData.mComboBox_DriveType->Add("Position");
			mData.mComboBox_DriveType->Select(mData.mDriveType);
			mData.mComboBox_DriveType->SetVisible(true);
			y += YStep;
		}

		{
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Target velocity:", UIElems);
			mData.mEditBox_TargetVelocity = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "2", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Target position:", UIElems);
			mData.mEditBox_TargetPosition = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "0", UIElems, EDITBOX_FLOAT, null, null);
			mData.mEditBox_TargetPosition->SetEnabled(false);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Stiffness:", UIElems);
			mData.mEditBox_Stiffness = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "0", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Damping:", UIElems);
			mData.mEditBox_Damping = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "1000", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Internal compliance:", UIElems);
			mData.mEditBox_InternalCompliance = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "External compliance:", UIElems);
			mData.mEditBox_ExternalCompliance = helper.CreateEditBox(UI, ID++, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);
	}

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(2.73f, 10.47f, 9.51f), Point(-0.36f, -0.43f, -0.83f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportArticulations || !caps.mSupportRigidBodySimulation)
			return false;

		// This test doesn't work without articulation
		PintObjectHandle Articulation = pint.CreateArticulation(PINT_ARTICULATION_CREATE());
		if(!Articulation)
			return false;

		const float Altitude = 5.0f;
		const Point Extents(1.0f, 1.0f, 1.0f);
		const Point ArticulationPos(0.0f, Altitude + Extents.y, 0.0f);

		//

		const float TargetVelocity = GetFromEditBox(0.0f, mData.mEditBox_TargetVelocity, MIN_FLOAT, MAX_FLOAT);
		const float TargetPosition = GetFromEditBox(0.0f, mData.mEditBox_TargetPosition, MIN_FLOAT, MAX_FLOAT);
		const float Stiffness = GetFromEditBox(0.0f, mData.mEditBox_Stiffness, MIN_FLOAT, MAX_FLOAT);
		const float Damping = GetFromEditBox(0.0f, mData.mEditBox_Damping, MIN_FLOAT, MAX_FLOAT);
		const float InternalCompliance = GetFromEditBox(0.0f, mData.mEditBox_InternalCompliance, MIN_FLOAT, MAX_FLOAT);
		const float ExternalCompliance = GetFromEditBox(0.0f, mData.mEditBox_ExternalCompliance, MIN_FLOAT, MAX_FLOAT);

		//
		PintObjectHandle ArticulatedObjectRoot;
		{
			PINT_BOX_CREATE BoxDesc(Extents);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 1.0f;
			ObjectDesc.mPosition	= ArticulationPos;

			//
			PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;

			ArticulatedObjectRoot = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
		}

		//

		//
		if(1)
		{
			PINT_FIXED_JOINT_CREATE fjc;
			fjc.mObject0 = null;
			fjc.mObject1 = ArticulatedObjectRoot;
			fjc.mLocalPivot0 = ArticulationPos;
			fjc.mLocalPivot1 = Point(0.0f, 0.0f, 0.0f);
			PintJointHandle j1 = pint.CreateJoint(fjc);
		}
		if(0)
		{
			PintJointHandle j1 = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(null, ArticulatedObjectRoot, ArticulationPos, Point(0.0f, 0.0f, 0.0f)));
		}
		if(0)
		{
			PINT_HINGE_JOINT_CREATE fjc;
			fjc.mObject0 = null;
			fjc.mObject1 = ArticulatedObjectRoot;
			fjc.mLocalPivot0 = ArticulationPos;
			fjc.mLocalPivot1 = Point(0.0f, 0.0f, 0.0f);
			fjc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
			fjc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
			PintJointHandle j1 = pint.CreateJoint(fjc);
		}
		//

			PINT_BOX_CREATE BoxDesc(2.0f, 2.0f, 0.5f);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 1.0f;
			ObjectDesc.mPosition	= ArticulationPos;

		ObjectDesc.mPosition = ArticulationPos;
		PintObjectHandle Parent = ArticulatedObjectRoot;
		{
			PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
			ArticulatedDesc.mParent = Parent;
			ArticulatedDesc.mLocalPivot0 = Point(0.0f, 0.0f, 0.0f);
			ArticulatedDesc.mLocalPivot1 = Point(0.0f, 0.0f, 0.0f);
			// We want a hinge. We will use the twist axis as the hinge axis. The twist axis is
			// defined by the "X" axis in PhysX, and we can remap it here.
			ArticulatedDesc.mX = Point(0.0f, 0.0f, 1.0f);
//			ArticulatedDesc.mX = Point(1.0f, 0.0f, 0.0f);
//			ArticulatedDesc.mEnableTwistLimit = true;
			ArticulatedDesc.mEnableTwistLimit = false;
			ArticulatedDesc.mTwistLowerLimit = -0.001f;
			ArticulatedDesc.mTwistUpperLimit = 0.001f;
			ArticulatedDesc.mEnableSwingLimit = true;
			ArticulatedDesc.mSwingYLimit = 0.001f;
			ArticulatedDesc.mSwingZLimit = 0.001f;
//			ArticulatedDesc.mSwingZLimit = FLT_MAX;

			ArticulatedDesc.mUseMotor = true;
			ArticulatedDesc.mMotor.mExternalCompliance = ExternalCompliance;
			ArticulatedDesc.mMotor.mInternalCompliance = InternalCompliance;
			ArticulatedDesc.mMotor.mDamping = Damping;
			ArticulatedDesc.mMotor.mStiffness = Stiffness;
			// So this is a bit confusing here. The target velocity is defined in "parent constraint frame". In the
			// PEEL plugin we use the same constraint frame for the two bodies linked by an articulation joint. So
			// in our case this velocity is defined in the rotating object's frame. We remapped its X axis to Z above,
			// so the desired angular velocity is now around X. Another way to see it is just that we want to rotate
			// around the twist axis, and the twist axis is always X from the joint's local point of view.
			// I suspect the Y and Z values of the target vel would just be around the Y and Z swing axes.
			ArticulatedDesc.mMotor.mTargetVelocity = Point(TargetVelocity, 0.0f, 0.0f);
//			ArticulatedDesc.mMotor.mTargetVelocity = Point(TargetVelocity, TargetVelocity, TargetVelocity);

			const AngleAxis AA(TargetPosition*DEGTORAD, 1.0f, 0.0f, 0.0f);
			Quat Rot = AA;
			Rot.Normalize();

			ArticulatedDesc.mMotor.mTargetOrientation = Rot;

			Parent = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
		}
		pint.AddArticulationToScene(Articulation);

		return true;
	}

	virtual	udword	Update(Pint& pint, float dt)
	{
		PintObjectHandle Obj = pint.mUserData;
//		pint.SetAngularVelocity(Obj, Point(0.0f, 0.0f, 0.0f));

		return TestBase::Update(pint, dt);
	}

END_TEST(ArticulationDriveTest)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ArticulatedChain = "Articulated chain. The chain is made of 30 capsules. Each capsule has a mass of 1. \
The large box at the end of the chain has a mass of 100. This kind of scene is challenging for iterative solvers \
(that most physics engines use by default). On the other hand some engines like PhysX also support dedicated solutions/solvers \
for articulated bodies, and they can handle this test case out of the box.";
class ArticulatedChain : public ConfigurableArticulationTest
{
	public:
							ArticulatedChain()
							{
								mValues.mNbSimCallsPerRenderFrame		= 1;
								mValues.mNbLinks						= 30;
								mValues.mCapsuleRadius					= 0.5f;
								mValues.mCapsuleMass					= 1.0f;
								mValues.mCapsuleMassForInertia			= 1.0;
								mValues.mBoxMass						= 100.0;
								mValues.mInitialAngularVelocity			= 0.0;
								mValues.mUseArticulations				= true;
								mValues.mUseExtraDistanceConstraints	= false;
							}

	virtual					~ArticulatedChain()			{									}
	virtual	const char*		GetName()			const	{ return "ArticulatedChain";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_ArticulatedChain;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_ARTICULATIONS;	}

	virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		ConfigurableArticulationTest::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.46f, 27.81f, 16.09f), Point(0.00f, -0.39f, -0.92f));
	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		ConfigurableArticulationTest::InitUI(helper);
		mEditBox_InitialAngularVel->SetEnabled(false);
		mEditBox_NbLinks->SetEnabled(false);
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG[2] = { PintDisabledGroups(1, 2), PintDisabledGroups(3, 3)	};
			pint.SetDisabledGroups(2, DG);
		}

		const bool UseArticulations = mCheckBox_Articulations ? mCheckBox_Articulations->IsChecked() : mValues.mUseArticulations;
		const bool UseDistanceConstraints = mCheckBox_DistanceConstraints ? mCheckBox_DistanceConstraints->IsChecked() : mValues.mUseExtraDistanceConstraints;
//		const bool UseOverlappingLinks = mCheckBox_OverlappingLinks ? mCheckBox_OverlappingLinks->IsChecked() : false;
//		const float AngularVel = GetFromEditBox(mValues.mInitialAngularVelocity, mEditBox_InitialAngularVel, MIN_FLOAT, MAX_FLOAT);
		const float BoxMass = GetFromEditBox(mValues.mBoxMass, mEditBox_BoxMass, 0.0f, MAX_FLOAT);
		const float CapsuleMass = GetFromEditBox(mValues.mCapsuleMass, mEditBox_CapsuleMass, 0.0f, MAX_FLOAT);
		const float CapsuleMassForInertia = GetFromEditBox(mValues.mCapsuleMassForInertia, mEditBox_CapsuleMassForInertia, 0.0f, MAX_FLOAT);
		const float CapsuleRadius = GetFromEditBox(mValues.mCapsuleRadius, mEditBox_CapsuleRadius, 0.0f, MAX_FLOAT);
//		const udword NbLinks = GetFromEditBox(mValues.mNbLinks, mEditBox_NbLinks);

		if(UseArticulations && !caps.mSupportArticulations)
			return false;

		if(UseDistanceConstraints && !caps.mSupportDistanceJoints)
			return false;

		if(CapsuleMass!=CapsuleMassForInertia && !caps.mSupportMassForInertia)
			printf(_F("WARNING: %s doesn't support 'mass for inertia', feature is ignored.\n", pint.GetName()));

		PintObjectHandle Articulation = UseArticulations ? pint.CreateArticulation(PINT_ARTICULATION_CREATE()) : null;

		const float Scale = 0.25f*0.5f;
		const float Radius = CapsuleRadius*Scale;
		const float HalfHeight = 2.0f*Scale;
		const udword NbCapsules = 30;
		const Point Dir(1.0f, 0.0f, 0.0f);
		const Point Extents = Dir * (Radius + HalfHeight);
//		const Point Extents = Dir * (HalfHeight);
		const Point PosOffset = Dir * 0.0f;

		Matrix3x3 m;
		m.RotZ(degToRad(90.0f));

		PINT_CAPSULE_CREATE CapsuleDesc(Radius, HalfHeight);
		CapsuleDesc.mLocalRot	= m;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(Radius, HalfHeight*2.0f);

		PintObjectHandle Handles[NbCapsules];
		Point Positions[NbCapsules];

		const Point BoxExtents(10.0f*Scale, 10.0f*Scale, 10.0f*Scale);
//		const float InitY = (5.0f + float(NbCapsules)*(Radius+HalfHeight)*2.0f + BoxExtents.y)*Scale;
		const float InitY = 185.0f*Scale;
		Point Pos(0.0f, InitY, 0.0f);

		Positions[0] = Pos;
		udword GroupBit = 0;

		// Creates a spherical attachment point for the rope, to make the scene more visually pleasing
		if(1)
		{
			PINT_SPHERE_CREATE SphereDesc(2.0f*Scale);
			SphereDesc.mRenderer = CreateSphereRenderer(SphereDesc.mRadius);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &SphereDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition		= Pos - Point(2.0f*Scale, 0.0f, 0.0f);
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
			
			PintObjectHandle h = pint.CreateObject(ObjectDesc);
		}

		// Creates a static box obstacle. The rope will collide with it and rotate around it.
		if(1)
		{
			PINT_BOX_CREATE BoxDesc(10.0f*Scale, 3.0f*Scale, 20.0f*Scale);
			BoxDesc.mRenderer = CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Point(50.0f*Scale, InitY-10.0f*Scale, 0.0f);

			PintObjectHandle h = pint.CreateObject(ObjectDesc);
		}

		{
			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &CapsuleDesc;
			ObjectDesc.mMass			= CapsuleMass;
			ObjectDesc.mMassForInertia	= CapsuleMassForInertia;
			ObjectDesc.mPosition		= Pos;
			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;

			if(Articulation)
			{
				PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
				Handles[0] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
//				Handles[0] = pint.CreateArticulatedObject(articulation, null, *this, ObjectDesc);
			}
			else
			{
				Handles[0] = CreatePintObject(pint, ObjectDesc);
			}

			if(0)
			{
				// Make it "static"...
//				PINT_SPHERICAL_JOINT_CREATE Desc;
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mLocalAxis0 = Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1 = Point(0.0f, 0.0f, 1.0f);
				Desc.mObject0		= null;
				Desc.mObject1		= Handles[0];
				Desc.mLocalPivot0	= Pos + Point(0.0f, 0.0f, 0.0f);
//				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Pos + Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			else
			{
				PINT_BOX_CREATE BoxDesc(0.1f*Scale, 0.1f*Scale, 0.1f*Scale);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= 0.0f;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;
				PintObjectHandle h = pint.CreateObject(ObjectDesc);

				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(h, Handles[0], Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
				ASSERT(JointHandle);
			}
		}
		Pos += (PosOffset + Extents)*2.0f;

		for(udword i=1;i<NbCapsules-1;i++)
		{
			Positions[i] = Pos;
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &CapsuleDesc;
				ObjectDesc.mMass			= CapsuleMass;
				ObjectDesc.mMassForInertia	= CapsuleMassForInertia;
				ObjectDesc.mPosition		= Pos;
				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
//				Handles[i] = CreateArticulationLink(articulation, (PxArticulationLink*)Handles[i-1], *this, ObjectDesc);
				if(Articulation)
				{
					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					ArticulatedDesc.mParent = Handles[i-1];
					ArticulatedDesc.mLocalPivot0 = Extents + PosOffset;
					ArticulatedDesc.mLocalPivot1 = -Extents - PosOffset;
					Handles[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
				}
				else
				{
					Handles[i] = CreatePintObject(pint, ObjectDesc);

/*					PINT_HINGE_JOINT_CREATE Desc;
					Desc.mLocalAxis0 = Point(0.0f, 0.0f, 1.0f);
					Desc.mLocalAxis1 = Point(0.0f, 0.0f, 1.0f);
					Desc.mObject0		= Handles[i-1];
					Desc.mObject1		= Handles[i];
					Desc.mLocalPivot0	= Extents + PosOffset;
					Desc.mLocalPivot1	= -Extents - PosOffset;*/

					const Point Offset = Extents + PosOffset;
					PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i-1], Handles[i], Offset, -Offset));
					ASSERT(JointHandle);
				}
				if(0)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i-1];
					Desc.mObject1		= Handles[i];
					Desc.mMaxDistance	= Positions[i-1].Distance(Positions[i]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
			Pos += (PosOffset + Extents)*2.0f;
		}

		{
			const udword i=NbCapsules-1;
			PINT_BOX_CREATE BoxDesc(BoxExtents);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxExtents);

			//###
			Pos -= (PosOffset + Extents);
			Pos.x += BoxExtents.x;
//			Pos.x += BoxExtents.x - Radius;

			Positions[i] = Pos;
//			Handles[i] = CreateDynamicObject(pint, &SphereDesc, Pos);
			{
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= BoxMass;
				ObjectDesc.mPosition		= Pos;
//				ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
//				Handles[i] = CreateArticulationLink(articulation, (PxArticulationLink*)Handles[i-1], *this, ObjectDesc);
				if(Articulation)
				{
					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					ArticulatedDesc.mParent = Handles[i-1];
					ArticulatedDesc.mLocalPivot0 = Extents + PosOffset;
					ArticulatedDesc.mLocalPivot1 = Point(-BoxExtents.x, 0.0f, 0.0f);
					Handles[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
//					printf("Big Mass: %f\n", ((PxArticulationLink*)Handles[i])->getMass());

/*				PxArticulationJoint* joint = ((PxArticulationLink*)Handles[i])->getInboundJoint();
				if(joint)
				{
					joint->setParentPose(PxTransform(ToPxVec3(Extents + PosOffset)));
					joint->setChildPose(PxTransform(PxVec3(-BoxExtents.x, 0.0f, 0.0f)));
					setupJoint(joint);
				}*/
				}
				else
				{
					Handles[i] = CreatePintObject(pint, ObjectDesc);

					PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i-1], Handles[i], Extents + PosOffset, Point(-BoxExtents.x, 0.0f, 0.0f)));
					ASSERT(JointHandle);
				}

				if(0)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[0];
					Desc.mObject1		= Handles[i];
					Desc.mMaxDistance	= Positions[i].Distance(Positions[0]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}

			}
			Pos += (PosOffset + Extents)*2.0f;
		}
		if(Articulation)
			pint.AddArticulationToScene(Articulation);

		// Distance constraints
		if(UseDistanceConstraints)
		{
			const float Slop = 1.0f;
//			const float Slop = 1.025f;
			if(0)
			{
				udword i=NbCapsules-2;
				PINT_DISTANCE_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[0];
				Desc.mObject1		= Handles[i];
				Desc.mMaxDistance	= Positions[i].Distance(Positions[0])*Slop;
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}

			if(1)
			{
				for(udword i=0;i<NbCapsules;i++)
				{
					if(i+2<NbCapsules)
					{
						PINT_DISTANCE_JOINT_CREATE Desc;
						Desc.mObject0		= Handles[i];
						Desc.mObject1		= Handles[i+2];
						Desc.mMaxDistance	= Positions[i].Distance(Positions[i+2])*Slop;
						PintJointHandle JointHandle = pint.CreateJoint(Desc);
						ASSERT(JointHandle);
					}
				}
			}

			if(0)
			{
				for(udword i=1;i<NbCapsules;i++)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[0];
					Desc.mObject1		= Handles[i];
					Desc.mMaxDistance	= Positions[0].Distance(Positions[i])*Slop;
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}

		// Rotating plank
		if(0)
		{
			PINT_BOX_CREATE BoxDesc(2.0f*Scale, 4.0f*Scale, 20.0f*Scale);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= 1.0f;
//			ObjectDesc.mMass			= 10000.0f;
//			ObjectDesc.mPosition		= Point(0.0f*Scale, -50.0f*Scale, -15.0f*Scale);
			ObjectDesc.mPosition		= Point(0.0f*Scale, InitY - float(NbCapsules)*(Radius+HalfHeight)*2.0f - BoxExtents.y, -15.0f*Scale);
			
			ObjectDesc.mCollisionGroup	= 3;
			PintObjectHandle h = pint.CreateObject(ObjectDesc);

			if(0)
			{
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
				Desc.mObject0		= null;
				Desc.mObject1		= h;
				Desc.mLocalPivot0	= ObjectDesc.mPosition;
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			else
			{
				PINT_BOX_CREATE BoxDesc(0.1f*Scale, 0.1f*Scale, 0.1f*Scale);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
				ObjectDesc.mShapes	= &BoxDesc;
				ObjectDesc.mMass	= 0.0f;
				PintObjectHandle h2 = pint.CreateObject(ObjectDesc);

				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mLocalAxis0 = Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1 = Point(0.0f, 1.0f, 0.0f);
				Desc.mObject0		= h2;
				Desc.mObject1		= h;
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}

		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(ArticulatedChain)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ConnectedArticulations = "The PhysX articulations are limited to 64 elements per articulation. This test shows how to \
bypass that limit for a rope, by connecting multiple articulations (3 in this case) with regular joints. This test also shows that using overlapping capsules \
for the chain is better for collision detection. You should be able to pick the end of the rope with the mouse and with some practice actually make a knot.";

START_TEST(ConnectedArticulations, CATEGORY_ARTICULATIONS, gDesc_ConnectedArticulations)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(45.47f, 4.24f, 4.65f), Point(-0.84f, -0.23f, -0.49f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportArticulations || !caps.mSupportRigidBodySimulation)
			return false;

/*		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG[2] = { PintDisabledGroups(1, 2), PintDisabledGroups(3, 3)	};
			pint.SetDisabledGroups(2, DG);
		}*/

		const udword NbArticulations = 3;
		PintObjectHandle Articulations[64];
		for(udword i=0;i<NbArticulations;i++)
		{
			Articulations[i] = pint.CreateArticulation(PINT_ARTICULATION_CREATE());
			// This test doesn't work without articulations
			if(!Articulations[i])
				return false;
		}

		const float Scale = 0.25f*0.5f*0.5f;
//		const float Scale = 0.25f*0.5f*0.1f;
//		const float Scale = 0.25f*0.5f*0.25f;

//		const float Radius = 0.5f*Scale;
		const float Radius = 1.0f*Scale;
//		const float Radius = 2.0f*Scale;
		const float HalfHeight = 2.0f*Scale;
//		const float HalfHeight = 1.0f*Scale;
//		const udword NbCapsules = 20;
//		const udword NbCapsules = 40;
		const udword NbCapsules = 64;
//		const udword NbCapsules = 100;
//		const udword NbCapsules = 200;
//		const udword NbCapsules = 500;
		const Point Dir(1.0f, 0.0f, 0.0f);
//		const Point Extents = Dir * (Radius + HalfHeight);
		const Point Extents = Dir * (HalfHeight);
		const Point PosOffset = Dir * 0.0f;
		const bool AttachToWorld = true;

		Matrix3x3 m;
		m.RotZ(degToRad(90.0f));

		PINT_CAPSULE_CREATE CapsuleDesc(Radius, HalfHeight);
		CapsuleDesc.mLocalRot	= m;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(Radius, HalfHeight*2.0f);

		PintObjectHandle Handles[NbCapsules*NbArticulations];
		Point Positions[NbCapsules*NbArticulations];

		const Point BoxExtents(10.0f*Scale, 10.0f*Scale, 10.0f*Scale);
//		const float InitY = (5.0f + float(NbCapsules)*(Radius+HalfHeight)*2.0f + BoxExtents.y)*Scale;
		const float InitY = 185.0f*Scale;
		Point Pos(0.0f, InitY, 0.0f);

		for(udword j=0;j<NbArticulations;j++)
		{
			PintObjectHandle* H = Handles + j*NbCapsules;
			Point* P = Positions + j*NbCapsules;

			PintObjectHandle Articulation = Articulations[j];
			for(udword i=0;i<NbCapsules;i++)
			{
				{
					PINT_OBJECT_CREATE ObjectDesc;
					ObjectDesc.mShapes		= &CapsuleDesc;
					ObjectDesc.mMass		= 1.0f;
					ObjectDesc.mPosition	= Pos;

					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					if(i)
						ArticulatedDesc.mParent = H[i-1];
					ArticulatedDesc.mLocalPivot0 = Extents + PosOffset;
					ArticulatedDesc.mLocalPivot1 = -Extents - PosOffset;
					H[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
					P[i] = Pos;
				}
				Pos += (PosOffset + Extents)*2.0f;
			}
			pint.AddArticulationToScene(Articulation);
		}

		for(udword j=0;j<NbArticulations-1;j++)
		{
			const PintObjectHandle* H = Handles + j*NbCapsules;
			const Point* P = Positions + j*NbCapsules;

			const Point Offset = Extents + PosOffset;
			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(H[NbCapsules-1], H[NbCapsules], Offset, -Offset));
			ASSERT(JointHandle);

			if(1)
			{
				for(udword i=0;i<NbCapsules;i++)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					const udword Index0 = NbCapsules-i-1;
					const udword Index1 = NbCapsules+i;
					Desc.mObject0		= H[Index0];
					Desc.mObject1		= H[Index1];
					Desc.mMaxDistance	= P[Index0].Distance(P[Index1]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}

		if(AttachToWorld)
		{
			PINT_BOX_CREATE BoxDesc(0.1f*Scale, 0.1f*Scale, 0.1f*Scale);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Point(0.0f, InitY, 0.0f);
			PintObjectHandle h = pint.CreateObject(ObjectDesc);

			if(0)
			{
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= h;
				Desc.mObject1		= Handles[0];
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			else
			{
				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(h, Handles[0], Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
				ASSERT(JointHandle);
			}
		}

//		mCreateDefaultEnvironment = false;
		return true;
	}

END_TEST(ConnectedArticulations)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_Winch = "Grab that handle and rotate it with the mouse for some winch action.";

START_TEST(Winch, CATEGORY_ARTICULATIONS, gDesc_Winch)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(4.11f, 12.58f, 9.86f), Point(-0.46f, -0.26f, -0.85f));
		desc.mCamera[1] = CameraPose(Point(-3.80f, 12.55f, 11.81f), Point(0.37f, -0.29f, -0.88f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportArticulations || !caps.mSupportRigidBodySimulation)
			return false;

/*		const bool UseFiltering = true;
		if(UseFiltering)
		{
			if(!caps.mSupportCollisionGroups)
				return false;

			const PintDisabledGroups DG[2] = { PintDisabledGroups(1, 2), PintDisabledGroups(3, 3)	};
			pint.SetDisabledGroups(2, DG);
		}*/

		const udword NbArticulations = 3;
		PintObjectHandle Articulations[64];
		for(udword i=0;i<NbArticulations;i++)
		{
			Articulations[i] = pint.CreateArticulation(PINT_ARTICULATION_CREATE());
			// This test doesn't work without articulations
			if(!Articulations[i])
				return false;
		}

		const float Scale = 0.25f*0.5f*0.5f;
//		const float Scale = 0.25f*0.5f*0.1f;
//		const float Scale = 0.25f*0.5f*0.25f;

//		const float Radius = 0.5f*Scale;
		const float Radius = 1.0f*Scale;
//		const float Radius = 2.0f*Scale;
		const float HalfHeight = 2.0f*Scale;
		const udword NbCapsules = 64;
		const Point Dir(1.0f, 0.0f, 0.0f);
//		const Point Extents = Dir * (Radius + HalfHeight);
		const Point Extents = Dir * (HalfHeight);
		const Point PosOffset = Dir * 0.0f;
		const bool AttachToWorld = true;

		const Point BoxExtents(10.0f*Scale, 10.0f*Scale, 10.0f*Scale);
//		const float InitY = (5.0f + float(NbCapsules)*(Radius+HalfHeight)*2.0f + BoxExtents.y)*Scale;
		const float InitY = 185.0f*Scale;
		Point Pos(0.0f, InitY, 0.0f);

		Matrix3x3 m;
		m.RotZ(degToRad(90.0f));

//		{
//			PINT_CAPSULE_CREATE CapsuleDesc2(HalfHeight*2.0f, 4.0f);
			PINT_CAPSULE_CREATE CapsuleDesc2(HalfHeight, 4.0f);
			CapsuleDesc2.mRenderer = CreateCapsuleRenderer(CapsuleDesc2.mRadius, CapsuleDesc2.mHalfHeight*2.0f);

			Matrix3x3 m2;
			m2.RotX(degToRad(90.0f));
			CapsuleDesc2.mLocalRot	= m2;


			PINT_CAPSULE_CREATE CapsuleDesc3(HalfHeight, 2.0f);
			CapsuleDesc3.mRenderer	= CreateCapsuleRenderer(CapsuleDesc3.mRadius, CapsuleDesc3.mHalfHeight*2.0f);
			CapsuleDesc3.mLocalPos	= Point(0.0f, 0.0f, CapsuleDesc2.mHalfHeight);
			CapsuleDesc3.mLocalRot	= m;
			CapsuleDesc2.mNext		= &CapsuleDesc3;

/*
			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &CapsuleDesc2;
			ObjectDesc.mMass		= 10.0f;
			ObjectDesc.mPosition	= Pos;
//			ObjectDesc.mRotation	= m2;
//			PintObjectHandle h = pint.CreateObject(ObjectDesc);*/
//		}

		PINT_CAPSULE_CREATE CapsuleDesc(Radius, HalfHeight);
		CapsuleDesc.mLocalRot	= m;
		CapsuleDesc.mRenderer	= CreateCapsuleRenderer(Radius, HalfHeight*2.0f);

		PintObjectHandle Handles[NbCapsules*NbArticulations];
		Point Positions[NbCapsules*NbArticulations];

		for(udword j=0;j<NbArticulations;j++)
		{
			PintObjectHandle* H = Handles + j*NbCapsules;
			Point* P = Positions + j*NbCapsules;

			PintObjectHandle Articulation = Articulations[j];
			for(udword i=0;i<NbCapsules;i++)
			{
				{
					PINT_OBJECT_CREATE ObjectDesc;
					if(i || j)
					{
						ObjectDesc.mShapes		= &CapsuleDesc;
						ObjectDesc.mMass		= 1.0f;
						ObjectDesc.mPosition	= Pos;
					}
					else
					{
						ObjectDesc.mShapes		= &CapsuleDesc2;
						ObjectDesc.mMass		= 20.0f;
						ObjectDesc.mPosition	= Pos;
//						ObjectDesc.mRotation	= m2;
//						ObjectDesc.mAngularVelocity	= Point(0.0f, 0.0f, 100.0f);
					}

					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					if(i)
						ArticulatedDesc.mParent = H[i-1];
//					if(i||j)
					ArticulatedDesc.mLocalPivot0 = Extents + PosOffset;
//					else
//					ArticulatedDesc.mLocalPivot0 = (Dir * CapsuleDesc2.mRadius) + PosOffset;
					ArticulatedDesc.mLocalPivot1 = -Extents - PosOffset;
					H[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
					P[i] = Pos;
				}
//				if(i||j)
				Pos += (PosOffset + Extents)*2.0f;
//				else
//				Pos += (PosOffset + (Dir * (CapsuleDesc2.mRadius)))*2.0f;
			}
			pint.AddArticulationToScene(Articulation);
		}

		for(udword j=0;j<NbArticulations-1;j++)
		{
			const PintObjectHandle* H = Handles + j*NbCapsules;
			const Point* P = Positions + j*NbCapsules;

			const Point Offset = Extents + PosOffset;
			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(H[NbCapsules-1], H[NbCapsules], Offset, -Offset));
			ASSERT(JointHandle);

			if(1)
			{
				for(udword i=0;i<NbCapsules;i++)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					const udword Index0 = NbCapsules-i-1;
					const udword Index1 = NbCapsules+i;
					Desc.mObject0		= H[Index0];
					Desc.mObject1		= H[Index1];
					Desc.mMaxDistance	= P[Index0].Distance(P[Index1]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}

		if(AttachToWorld)
		{
			PINT_BOX_CREATE BoxDesc(0.1f*Scale, 0.1f*Scale, 0.1f*Scale);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxDesc;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Point(0.0f, InitY, 0.0f);
			PintObjectHandle h = pint.CreateObject(ObjectDesc);

			if(1)
			{
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= h;
				Desc.mObject1		= Handles[0];
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			else
			{
				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(h, Handles[0], Point(0.0f, 0.0f, 0.0f), Point(0.0f, 0.0f, 0.0f)));
				ASSERT(JointHandle);
			}

//			pint.SetAngularVelocity(Handles[0], Point(0.0f, 0.0f, 100.0f));
			pint.mUserData = Handles[0];
		}

//		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual	udword	Update(Pint& pint, float dt)
	{
/*		PintObjectHandle h = PintObjectHandle(pint.mUserData);
		if(h)
			pint.SetAngularVelocity(h, Point(0.0f, 0.0f, 4.0f));*/
		return TestBase::Update(pint, dt);
	}

END_TEST(Winch)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ArticulatedCaterpillarTrack = "Articulated caterpillar track. This test demonstrates how to overcome two \
apparent limits of the PhysX articulations, namely how to do a closed loop and hinge joints with them.";
START_TEST(ArticulatedCaterpillarTrack, CATEGORY_ARTICULATIONS, gDesc_ArticulatedCaterpillarTrack )

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(0.37f, 8.80f, 11.93f), Point(-0.03f, -0.38f, -0.93f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportArticulations || !caps.mSupportRigidBodySimulation)
			return false;

		// This test doesn't work without articulation
		PintObjectHandle Articulation = pint.CreateArticulation(PINT_ARTICULATION_CREATE());
		if(!Articulation)
			return false;

		const float Radius = 4.0f;
		const udword NbLinks = 64;
		const float D = PI*Radius/float(NbLinks);
		const float Altitude = 5.0f;
//		const Point Extents(D*0.8f, 0.1f, 1.0f);
		const Point Extents(D, 0.1f, 1.0f);
		const Point ArticulationPos(0.0f, Altitude + Extents.y, 0.0f);

		// Main plank
		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			// Smaller inside box to catch the gear
			PINT_BOX_CREATE BoxDesc2(Extents.x*0.1f, Extents.y, Extents.z*0.5f);
			BoxDesc2.mRenderer	= CreateBoxRenderer(BoxDesc2.mExtents);
			BoxDesc2.mLocalPos	= Point(0.0f, -Extents.y - BoxDesc2.mExtents.y, 0.0f);
			BoxDesc.mNext		= &BoxDesc2;

			// Left & right upper boxes (will touch the ground)
			PINT_BOX_CREATE BoxDesc3(Extents.x, 0.2f, 0.1f);
			BoxDesc3.mRenderer	= CreateBoxRenderer(BoxDesc3.mExtents);
			BoxDesc3.mLocalPos	= Point(0.0f, (BoxDesc3.mExtents.y - Extents.y), Extents.z + BoxDesc3.mExtents.z);
			BoxDesc2.mNext		= &BoxDesc3;

			PINT_BOX_CREATE BoxDesc4(BoxDesc3.mExtents);
			BoxDesc4.mRenderer	= BoxDesc3.mRenderer;
			BoxDesc4.mLocalPos	= Point(0.0f, BoxDesc3.mLocalPos.y, -BoxDesc3.mLocalPos.z);
			BoxDesc3.mNext		= &BoxDesc4;

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= ArticulationPos;

		//
		PintObjectHandle Objects[128];
		PintObjectHandle Parent = null;
		for(udword i=0;i<NbLinks;i++)
		{
			const float Coeff = float(i)/float(NbLinks);
			const float Angle = Coeff*TWOPI;
			const float x = sinf(Angle) * Radius;
			const float y = cosf(Angle) * Radius;

			ObjectDesc.mPosition = Point(x, y+Altitude, 0.0f);

			Matrix3x3 Rot;
			Rot.RotZ(-Angle);
			ObjectDesc.mRotation = Rot;

			PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
			if(Parent)
			{
				ArticulatedDesc.mParent = Parent;
				ArticulatedDesc.mLocalPivot0 = Point(D, 0.0f, 0.0f);
				ArticulatedDesc.mLocalPivot1 = Point(-D, 0.0f, 0.0f);
			}

			if(0)
			{
				// This way is similar to what we did in the ArticulationDrive test.
				ArticulatedDesc.mX = Point(0.0f, 0.0f, 1.0f);
				ArticulatedDesc.mEnableTwistLimit = false;
				ArticulatedDesc.mTwistLowerLimit = -0.001f;
				ArticulatedDesc.mTwistUpperLimit = 0.001f;
				ArticulatedDesc.mEnableSwingLimit = true;
				ArticulatedDesc.mSwingYLimit = 0.001f;
				ArticulatedDesc.mSwingZLimit = 0.001f;
			}
			else
			{
				const float Epsilon = 0.0001f;
				// We simply emulate a hinge joint using the joints limits.
				ArticulatedDesc.mEnableTwistLimit = true;
				ArticulatedDesc.mTwistLowerLimit = -Epsilon;
				ArticulatedDesc.mTwistUpperLimit = Epsilon;
				ArticulatedDesc.mEnableSwingLimit = true;
//				ArticulatedDesc.mSwingYLimit = Epsilon;
				ArticulatedDesc.mSwingZLimit = Epsilon;
//				ArticulatedDesc.mSwingYLimit = FLT_MAX;
//				ArticulatedDesc.mSwingYLimit = PI - Epsilon;
				ArticulatedDesc.mSwingYLimit = PI/2.0f;
			}
			Parent = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
			Objects[i] = Parent;
		}
		pint.AddArticulationToScene(Articulation);

		// We use a regular hinge joint to close the loop. It could be made more robust by using
		// extra distance constraints on top of this, as seen in the ConnectedArticulations test.
		// But this test shows that the basic & obvious approach also works.
		if(1)
		{
			PINT_HINGE_JOINT_CREATE Desc;
			Desc.mObject0		= Objects[0];
			Desc.mObject1		= Objects[NbLinks-1];
			Desc.mLocalPivot0	= Point(-D, 0.0f, 0.0f);
			Desc.mLocalPivot1	= Point(D, 0.0f, 0.0f);
			Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
			Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
		return true;
	}

END_TEST(ArticulatedCaterpillarTrack)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ArticulatedVehicle = "(Configurable test) - A basic articulated vehicle with large mass ratios between the jointed parts. \
This test uses an articulation for the car, showing that it can do more than just ropes. It is a basic vehicle without suspension or anything, \
just to show that this setup does work in PhysX - provided you use the right feature. You can also make it work a lot better with regular joints, \
by using more homogeneous masses for the car parts and/or increasing the solver iteration counts and/or creating the constraints several times, etc. \
Since this test uses regular rigid bodies (as opposed to the dedicated PhysX vehicle SDK), the behavior also depends on which friction model is \
selected in the PhysX UI panel.";
class ArticulatedVehicle : public TestBase
{
			IceCheckBox*		mCheckBox_Articulations;
			IceEditBox*			mEditBox_Multiplier;
			IceEditBox*			mEditBox_GroundFriction;
			IceEditBox*			mEditBox_WheelFriction;
			IceEditBox*			mEditBox_ChassisMass;
			IceEditBox*			mEditBox_WheelMass;
			IceEditBox*			mEditBox_StructMass;
			IceEditBox*			mEditBox_WheelRadius;
			IceEditBox*			mEditBox_WheelWidth;
			IceEditBox*			mEditBox_WheelTess;
			IceEditBox*			mEditBox_ChassisX;
			IceEditBox*			mEditBox_ChassisY;
			IceEditBox*			mEditBox_ChassisZ;
			IceEditBox*			mEditBox_COMLocalOffset;
			IceEditBox*			mEditBox_AxleRadius;
			IceEditBox*			mEditBox_AxleWidth;

			IndexedSurface*		mSurface;
			PintShapeRenderer*	mRenderer;

	public:
							ArticulatedVehicle() :
								mCheckBox_Articulations	(null),
								mEditBox_Multiplier		(null),
								mEditBox_GroundFriction	(null),
								mEditBox_WheelFriction	(null),
								mEditBox_ChassisMass	(null),
								mEditBox_WheelMass		(null),
								mEditBox_StructMass		(null),
								mEditBox_WheelRadius	(null),
								mEditBox_WheelWidth		(null),
								mEditBox_WheelTess		(null),
								mEditBox_ChassisX		(null),
								mEditBox_ChassisY		(null),
								mEditBox_ChassisZ		(null),
								mEditBox_COMLocalOffset	(null),
								mEditBox_AxleRadius		(null),
								mEditBox_AxleWidth		(null),
								mSurface				(null),
								mRenderer				(null)
							{
							}

	virtual					~ArticulatedVehicle()		{									}
	virtual	const char*		GetName()			const	{ return "ArticulatedVehicle";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_ArticulatedVehicle;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_ARTICULATIONS;	}

	virtual	void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(6.34f, 3.77f, 5.14f), Point(-0.69f, -0.41f, -0.60f));
	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 400;
		WD.mLabel	= "ArticulatedVehicle";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 120;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			mCheckBox_Articulations = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Use articulations", UIElems, true, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Constraint multiplier:", UIElems);
			mEditBox_Multiplier = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Ground friction:", UIElems);
			mEditBox_GroundFriction = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.1", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Wheel friction:", UIElems);
			mEditBox_WheelFriction = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Chassis mass:", UIElems);
			mEditBox_ChassisMass = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "40.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Wheel mass:", UIElems);
			mEditBox_WheelMass = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "15.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Struct mass:", UIElems);
			mEditBox_StructMass = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.0", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Wheel radius:", UIElems);
			mEditBox_WheelRadius = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Wheel width:", UIElems);
			mEditBox_WheelWidth = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.3", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Wheel tessellation:", UIElems);
			mEditBox_WheelTess = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "60", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Chassis size X:", UIElems);
			mEditBox_ChassisX = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Chassis size Y:", UIElems);
			mEditBox_ChassisY = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.5", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Chassis size Z:", UIElems);
			mEditBox_ChassisZ = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.7", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "COM local offset (Y):", UIElems);
			mEditBox_COMLocalOffset = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "-1.0", UIElems, EDITBOX_FLOAT, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Axle radius:", UIElems);
			mEditBox_AxleRadius = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "0.1", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Axle width:", UIElems);
			mEditBox_AxleWidth = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "1.1", UIElems, EDITBOX_FLOAT_POSITIVE, null, null);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);
	}

	virtual	bool			CommonSetup()
	{
		const udword Nb = 100;
		const float Scale = 0.1f;
		mSurface = CreateManagedSurface();
		bool status = mSurface->MakePlane(Nb, Nb);
		ASSERT(status);
		mSurface->Scale(Point(Scale, 1.0f, Scale));
		mSurface->Flip();

		mRenderer = CreateMeshRenderer(mSurface->GetSurfaceInterface());

		mCreateDefaultEnvironment = false;

		return true;
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation || !caps.mSupportMeshes)
			return false;

		const bool UseArticulations = mCheckBox_Articulations ? mCheckBox_Articulations->IsChecked() : true;
		if(UseArticulations && !caps.mSupportArticulations)
			return false;

		if(!UseArticulations && !caps.mSupportHingeJoints)
			return false;

		if(!caps.mSupportCollisionGroups)
			return false;

		// We just disable collisions between all parts of the car for now. That's because some engines don't automatically disable
		// collisions between jointed objects. We could do that better but the focus of this test is the stability of the vehicle,
		// and we don't really need proper collision filtering setup to evaluate this.
		const PintDisabledGroups DG(1, 1);
		pint.SetDisabledGroups(1, &DG);
		PintCollisionGroup GroupChassis = 1;
		PintCollisionGroup GroupWheel = 1;
		PintCollisionGroup GroupStruct = 1;

		// The following code will create the vehicle with or without articulations
		PintObjectHandle Articulation = UseArticulations ? pint.CreateArticulation(PINT_ARTICULATION_CREATE()) : null;

		const udword NbPts = GetFromEditBox(60, mEditBox_WheelTess);
		const udword Multiplier = GetFromEditBox(1, mEditBox_Multiplier);
		const float GroundFriction = GetFromEditBox(0.1f, mEditBox_GroundFriction, 0.0f, MAX_FLOAT);
		const float WheelFriction = GetFromEditBox(1.0f, mEditBox_WheelFriction, 0.0f, MAX_FLOAT);
		const float ChassisMass = GetFromEditBox(40.0f, mEditBox_ChassisMass, 0.0f, MAX_FLOAT);
		const float WheelMass = GetFromEditBox(15.0f, mEditBox_WheelMass, 0.0f, MAX_FLOAT);
		const float StructMass = GetFromEditBox(1.0f, mEditBox_StructMass, 0.0f, MAX_FLOAT);
		const float WheelRadius = GetFromEditBox(0.5f, mEditBox_WheelRadius, 0.0f, MAX_FLOAT);
		const float WheelWidth = GetFromEditBox(0.3f, mEditBox_WheelWidth, 0.0f, MAX_FLOAT);
		const float ChassisX = GetFromEditBox(1.5f, mEditBox_ChassisX, 0.0f, MAX_FLOAT);
		const float ChassisY = GetFromEditBox(0.5f, mEditBox_ChassisY, 0.0f, MAX_FLOAT);
		const float ChassisZ = GetFromEditBox(1.0f, mEditBox_ChassisZ, 0.0f, MAX_FLOAT);
		const float COMLocalOffset = GetFromEditBox(-1.0f, mEditBox_COMLocalOffset, MIN_FLOAT, MAX_FLOAT);
		const float AxleRadius = GetFromEditBox(0.1f, mEditBox_AxleRadius, 0.0f, MAX_FLOAT);
		const float AxleWidth = GetFromEditBox(1.0f, mEditBox_AxleWidth, 0.0f, MAX_FLOAT);

		PINT_MATERIAL_CREATE GroundMaterial;
		GroundMaterial.mStaticFriction	= GroundFriction;
		GroundMaterial.mDynamicFriction	= GroundFriction;
		GroundMaterial.mRestitution		= 0.0f;

		PINT_MATERIAL_CREATE WheelMaterial;
		WheelMaterial.mStaticFriction	= WheelFriction;
		WheelMaterial.mDynamicFriction	= WheelFriction;
		WheelMaterial.mRestitution		= 0.0f;

		// Creates the ground mesh
		{
			PINT_MESH_CREATE MeshDesc;
			MeshDesc.mSurface	= mSurface->GetSurfaceInterface();
			MeshDesc.mRenderer	= mRenderer;
			MeshDesc.mMaterial	= &GroundMaterial;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &MeshDesc;
			ObjectDesc.mPosition	= Point(0.0f, 0.0f, 0.0f);
			ObjectDesc.mMass		= 0.0f;
			PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
			ASSERT(Handle);
		}

//		const float y = 4.0f;	// ###
		const float y = ChassisY + WheelRadius;

		// When not using articulations we'll batch the joints to create them all at once in the end,
		// which allows us to easily create them multiple times for improved stability.
		PINT_HINGE_JOINT_CREATE Hinges[8];
		udword NbHinges = 0;

		// Chassis
		const Point Chassis(ChassisX, ChassisY, ChassisZ);
		const Point ChassisPos(0.0f, y + Chassis.y, 0.0f);
		PintObjectHandle ChassisObject;
		{
			// ### Note that the chassis is going to use the scene's default material & friction value
			PINT_BOX_CREATE BoxDesc;
			BoxDesc.mExtents	= Chassis;
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mPosition		= ChassisPos;
			ObjectDesc.mShapes			= &BoxDesc;
			ObjectDesc.mMass			= ChassisMass;
			ObjectDesc.mCollisionGroup	= GroupChassis;
			ObjectDesc.mCOMLocalOffset	= Point(0.0f, COMLocalOffset, 0.0f);

			if(Articulation)
			{
				PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
				ChassisObject = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
			}
			else
				ChassisObject = pint.CreateObject(ObjectDesc);

/*			if(0)
			{
				BoxDesc.mExtents	= Point(Chassis.x, 0.5f, Chassis.z*0.6f);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition	= CarPos + Point(0.0f, Chassis.y+BoxDesc.mExtents.y, 0.0f);
				ObjectDesc.mShapes		= &BoxDesc;
				ObjectDesc.mMass		= 4.0f;
				PintObjectHandle h = pint.CreateObject(ObjectDesc);

				PINT_PRISMATIC_JOINT_CREATE Desc;
				Desc.mObject0		= h;
				Desc.mObject1		= ChassisObject;
				Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
			}*/
		}

		// Rear wheels + rear axle. We put them all in one rigid compound object, which means they will all be
		// connected without the need for joints, and the collision between these pieces will be naturally
		// filtered out.
		//###TODO: share this data and the wheel renderer
		const CylinderMesh Cylinder(NbPts, WheelRadius, WheelWidth*0.5f);
		const udword NbWheelVerts = Cylinder.mNbVerts;
		const Point* WheelVerts = Cylinder.mVerts;
		PintShapeRenderer* WheelRenderer = CreateConvexRenderer(NbWheelVerts, WheelVerts);

		const Point Axle(AxleRadius, AxleRadius, AxleWidth);
		{
			const Point RearAxleOffset(-Chassis.x, -Chassis.y - Axle.y, 0.0f);	// delta between chassis center and rear axle center.
			const float RearWheelMass = StructMass+WheelMass*2.0f;	// 1 axle + 2 wheels all put in the same compound object
			const bool UseSphereWheels = false;

			PINT_BOX_CREATE BoxDesc(Axle);
			BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

			const Point AxlePos = ChassisPos + RearAxleOffset;
			PINT_OBJECT_CREATE AxleDesc;
			AxleDesc.mPosition			= AxlePos;
			AxleDesc.mShapes			= &BoxDesc;
			AxleDesc.mMass				= RearWheelMass;
			AxleDesc.mCollisionGroup	= GroupStruct;	//### Technically we'd need 3 groups here

			PINT_CONVEX_CREATE WheelDesc;
			PINT_CONVEX_CREATE WheelDesc2;
			PINT_SPHERE_CREATE SphereWheelDesc;
			PINT_SPHERE_CREATE SphereWheelDesc2;
			//###TODO: this uses the per-shape material feature (and it shows why it's useful) but some engines
			// don't support this. Check what happens for them.
			WheelDesc.mMaterial			= &WheelMaterial;
			WheelDesc2.mMaterial		= &WheelMaterial;
			SphereWheelDesc.mMaterial	= &WheelMaterial;
			SphereWheelDesc2.mMaterial	= &WheelMaterial;

			// The wheels will be located relative to the compound's position, i.e. AxlePos.
			// We put the wheel centers exactly on the left & right sides of the axle box.
			const Point RearWheelOffset(0.0f, 0.0f, Axle.z);

			if(UseSphereWheels)
			{
				SphereWheelDesc.mRadius		= WheelRadius;
				SphereWheelDesc.mRenderer	= CreateSphereRenderer(WheelRadius);
				SphereWheelDesc.mLocalPos	= RearWheelOffset;
				BoxDesc.mNext = &SphereWheelDesc;

				SphereWheelDesc2.mRadius	= WheelRadius;
				SphereWheelDesc2.mRenderer	= CreateSphereRenderer(WheelRadius);
				SphereWheelDesc2.mLocalPos	= -RearWheelOffset;
				SphereWheelDesc.mNext = &SphereWheelDesc2;
			}
			else
			{
				WheelDesc.mNbVerts		= NbWheelVerts;
				WheelDesc.mVerts		= WheelVerts;
				WheelDesc.mRenderer		= WheelRenderer;
				WheelDesc.mLocalPos		= RearWheelOffset;
				BoxDesc.mNext = &WheelDesc;

				WheelDesc2.mNbVerts		= NbWheelVerts;
				WheelDesc2.mVerts		= WheelVerts;
				WheelDesc2.mRenderer	= WheelRenderer;
				WheelDesc2.mLocalPos	= -RearWheelOffset;
				WheelDesc.mNext = &WheelDesc2;
			}

			// Now we attach the compound object to the chassis. We want a hinge between the two objects.
			// Collisions between the chassis and the rear compound will be disabled automatically in engines
			// that disable collision between jointed objects by default. For other engines we'd need to use
			// explicit collision filters here.
			PintObjectHandle RearAxleObject;
			if(Articulation)
			{
				PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
				ArticulatedDesc.mParent				= ChassisObject;
				ArticulatedDesc.mLocalPivot0		= RearAxleOffset;
				ArticulatedDesc.mLocalPivot1		= Point(0.0f, 0.0f, 0.0f);
				ArticulatedDesc.mX					= Point(0.0f, 0.0f, 1.0f);
				ArticulatedDesc.mEnableTwistLimit	= false;
				ArticulatedDesc.mTwistLowerLimit	= -0.01f;
				ArticulatedDesc.mTwistUpperLimit	= 0.01f;
				ArticulatedDesc.mEnableSwingLimit	= true;
				ArticulatedDesc.mSwingYLimit		= 0.001f;//PI/6.0f;
				ArticulatedDesc.mSwingZLimit		= 0.001f;//PI/6.0f;
				RearAxleObject = pint.CreateArticulatedObject(AxleDesc, ArticulatedDesc, Articulation);
			}
			else
			{
				RearAxleObject = pint.CreateObject(AxleDesc);

				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= ChassisObject;
				Desc.mObject1		= RearAxleObject;
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalPivot0	= RearAxleOffset;
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
//				PintJointHandle JointHandle = pint.CreateJoint(Desc);
//				ASSERT(JointHandle);
				Hinges[NbHinges++] = Desc;
			}
		}

		// Front wheels + front axle. The "FrontAxleObject" here will be cut into two smaller parts, but otherwise it is the
		// same as the rear axle (same size & location compared to the chassis).
		Point WheelPt[2];
		Point WheelGlobalPt[2];
		PintObjectHandle FrontAxleObject[2];
		// We want Chassis.z + FrontAxle.z = AxleWidth to make sure the front & rear wheels are aligned.
		// i.e. FrontAxle.z = AxleWidth - Chassis.z
		const Point FrontAxle = Point(AxleRadius, AxleRadius, AxleWidth - Chassis.z);	//### Object to whom the wheel is directly attached

		const float z2 = FrontAxle.z*0.25f;
		for(udword i=0;i<2;i++)
		{
			const float Sign = i ? -1.0f : 1.0f;
			// delta between chassis center and front axle(s) center:
			// *  +Chassis.x to put the objects' centers exactly at the front of the chassis (similar to the rear axle, which was -Chassis.x)
			// *  "-Chassis.y - Axle.y" is exactly the same as for the rear axle
			// *  Now things are different for Z, i.e. along the axle. For the rear wheels we only had one compound object for the whole thing
			//    so the Z offset was 0.0f. But for the front wheels we have 2 compounds, created left & right in such a way that the centers
			//    of the object are exactly located on the sides of the chassis (hence +/- Chassis.z). Note that the compound's position will be
			//    the center of the "FrontAxle" box.
			const Point FrontAxleOffset = Point(Chassis.x, -Chassis.y - Axle.y, Sign*Chassis.z);
			const Point AxlePos = ChassisPos + FrontAxleOffset;

			{
				// So that's the "front axle" box, i.e. the object similar to the rear axle but which got cut in two pieces.
				PINT_BOX_CREATE BoxDesc;
				BoxDesc.mExtents	= FrontAxle;
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				// And then that one is the extra (new) box that doesn't exist for the rear axle. It goes back along -x towards
				// the rear axle, and we'll attach some kind of sway bar to these guys later.
				PINT_BOX_CREATE BoxDesc2;
				// 0.3 is the length of the piece towards the rear axle
				// 0.5 is some coeff<1 so that the sway bar apparatus doesn't touch the chassis
				// z2 is arbitrary, it has to be <1 and there's some restriction with the wheel width here
				BoxDesc2.mExtents	= Point(0.3f, FrontAxle.y*0.5f, z2);
				BoxDesc2.mRenderer	= CreateBoxRenderer(BoxDesc2.mExtents);
				// We move it so that it aligns perfectly with the front axle object.
				// "-BoxDesc.mExtents.x - BoxDesc2.mExtents.x" aligns it perfectly along x, and as we said, towards the rear axle.
				// "FrontAxle.z*0.5f" = z2 = BoxDesc.mExtents.z so we align it with the front axle but let the objects overlap on that axis.
				BoxDesc2.mLocalPos	= Point(-BoxDesc.mExtents.x - BoxDesc2.mExtents.x, 0.0f, -Sign*z2);
				BoxDesc.mNext = &BoxDesc2;

				// So far these two boxes are in the same rigid body compound so collisions will be disabled between them.
				// This time we cannot put the wheels in the same compound, since they will need to rotate freely around Z,
				// and the front axles + sway bar thing cannot do so. So we'll attach the wheels to these compounds separately.
				// For now we add these compounds to our articulation. We want them to rotate around Y (i.e. to steer!). That's
				// another hinge.

/*				PINT_CONVEX_CREATE WheelDesc;
				WheelDesc.mNbVerts	= TotalNbVerts;
				WheelDesc.mVerts	= Verts;
				WheelDesc.mRenderer	= CreateConvexRenderer(WheelDesc.mNbVerts, WheelDesc.mVerts);
				WheelDesc.mLocalPos	= Point(0.0f, 0.0f, Coeff*FrontAxle.z);
				BoxDesc.mNext = &WheelDesc;*/

				PINT_OBJECT_CREATE AxleDesc;
				AxleDesc.mPosition			= AxlePos;
				AxleDesc.mShapes			= &BoxDesc;
				AxleDesc.mMass				= StructMass;
				AxleDesc.mCollisionGroup	= GroupStruct;

				// We attach these to the chassis, so collisions between the chassis and the compound will be disabled by
				// default in most engines.
				if(Articulation)
				{
//				const float Limit = PI/4.0f;
				const float Limit = PI/6.0f;
//				const float Limit = 0.0f;
					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					ArticulatedDesc.mParent = ChassisObject;
					ArticulatedDesc.mLocalPivot0 = FrontAxleOffset;
					ArticulatedDesc.mLocalPivot1 = Point(0.0f, 0.0f, 0.0f);
/*					ArticulatedDesc.mX = Point(0.0f, 1.0f, 0.0f);
					ArticulatedDesc.mEnableTwistLimit = true;
					ArticulatedDesc.mTwistLowerLimit = -Limit;
					ArticulatedDesc.mTwistUpperLimit = Limit;
					ArticulatedDesc.mEnableSwingLimit = true;
					ArticulatedDesc.mSwingYLimit = 0.001f;//PI/6.0f;
					ArticulatedDesc.mSwingZLimit = 0.001f;//PI/6.0f;*/
					ArticulatedDesc.mX = Point(1.0f, 0.0f, 0.0f);
					ArticulatedDesc.mEnableTwistLimit = true;
					ArticulatedDesc.mTwistLowerLimit = -0.001f;
					ArticulatedDesc.mTwistUpperLimit = 0.001f;
					ArticulatedDesc.mEnableSwingLimit = true;
					ArticulatedDesc.mSwingYLimit = 0.001f;
					ArticulatedDesc.mSwingZLimit = Limit;
					FrontAxleObject[i] = pint.CreateArticulatedObject(AxleDesc, ArticulatedDesc, Articulation);
				}
				else
				{
					FrontAxleObject[i] = pint.CreateObject(AxleDesc);

					//### Creating this before or after the wheel joint will produce different artefacts/failures
					if(1 && !Articulation)
					{
						const float Limit = PI/6.0f;
		//				const float Limit = 0.0f;
						PINT_HINGE_JOINT_CREATE Desc;
						Desc.mObject0		= ChassisObject;
						Desc.mObject1		= FrontAxleObject[i];
						Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
						Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
						Desc.mLocalPivot0	= FrontAxleOffset;
						Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
						Desc.mMinLimitAngle	= -Limit;
						Desc.mMaxLimitAngle	= Limit;
						Hinges[NbHinges++] = Desc;
					}
				}

				WheelPt[i] = Point(-BoxDesc.mExtents.x - BoxDesc2.mExtents.x*2.0f, 0.0f, -Sign*z2);
				WheelGlobalPt[i] = AxlePos + WheelPt[i];
			}

			// Create & attach front wheels to the previous compound
			if(1)
			{
				PINT_CONVEX_CREATE WheelDesc(NbWheelVerts, WheelVerts);
				WheelDesc.mRenderer	= WheelRenderer;
				WheelDesc.mMaterial	= &WheelMaterial;

				// AxlePos is the center of the "FrontAxle" object, so we just put the wheel's center on the
				// left or right side of it.
				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition		= AxlePos + Point(0.0f, 0.0f, Sign*FrontAxle.z);
				ObjectDesc.mMass			= WheelMass;
				ObjectDesc.mShapes			= &WheelDesc;
				ObjectDesc.mCollisionGroup	= GroupWheel;
//				ObjectDesc.mAngularVelocity	= Point(0.0, 0.0f, -100.0f);

				// We simply attach this to the previous front-axle compound, using an hinge to let the wheel rotate freely.
				if(Articulation)
				{
					PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
					ArticulatedDesc.mParent = FrontAxleObject[i];
					ArticulatedDesc.mLocalPivot0 = Point(0.0f, 0.0f, Sign*FrontAxle.z);
					ArticulatedDesc.mLocalPivot1 = Point(0.0f, 0.0f, 0.0f);
					ArticulatedDesc.mX = Point(0.0f, 0.0f, 1.0f);
					ArticulatedDesc.mEnableTwistLimit = false;
					ArticulatedDesc.mTwistLowerLimit = -0.01f;
					ArticulatedDesc.mTwistUpperLimit = 0.01f;
					ArticulatedDesc.mEnableSwingLimit = true;
					ArticulatedDesc.mSwingYLimit = 0.001f;//PI/6.0f;
					ArticulatedDesc.mSwingZLimit = 0.001f;//PI/6.0f;
					PintObjectHandle WheelObject = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
				}
				else
				{
					PintObjectHandle WheelObject = pint.CreateObject(ObjectDesc);

					PINT_HINGE_JOINT_CREATE Desc;
					Desc.mObject0		= WheelObject;
					Desc.mObject1		= FrontAxleObject[i];
					Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
					Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
					Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
					Desc.mLocalPivot1	= Point(0.0f, 0.0f, Sign*FrontAxle.z);
					Hinges[NbHinges++] = Desc;
				}
			}

			if(0 && !Articulation)
			{
				const float Limit = PI/6.0f;
//				const float Limit = 0.0f;
				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= ChassisObject;
				Desc.mObject1		= FrontAxleObject[i];
				Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalPivot0	= FrontAxleOffset;
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, 0.0f);
				Desc.mMinLimitAngle	= -Limit;
				Desc.mMaxLimitAngle	= Limit;
				Hinges[NbHinges++] = Desc;
			}
		}

		// Finally, we just connect the two "front axle" compounds with a rod, to make sure the front wheels
		// always have the same orientation.
		if(1)
		{
			const float Length = WheelGlobalPt[0].Distance(WheelGlobalPt[1]);
			if(0)
			{
				PINT_DISTANCE_JOINT_CREATE Desc;
				Desc.mObject0		= FrontAxleObject[0];
				Desc.mObject1		= FrontAxleObject[1];
				Desc.mLocalPivot0	= WheelPt[0];
				Desc.mLocalPivot1	= WheelPt[1];
				Desc.mMinDistance	= Length;
				Desc.mMaxDistance	= Length;
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			else
			{
				PINT_BOX_CREATE BoxDesc;
				BoxDesc.mExtents	= Point(0.05f, 0.05f, Length*0.5f/* - z2*/);
				BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mPosition		= (WheelGlobalPt[0] + WheelGlobalPt[1])*0.5f;
				ObjectDesc.mShapes			= &BoxDesc;
				ObjectDesc.mMass			= StructMass;
				ObjectDesc.mCollisionGroup	= GroupStruct;
				PintObjectHandle RodObject = pint.CreateObject(ObjectDesc);

				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mLocalAxis0	= Point(0.0f, 1.0f, 0.0f);
				Desc.mLocalAxis1	= Point(0.0f, 1.0f, 0.0f);

				Desc.mObject0		= FrontAxleObject[0];
				Desc.mObject1		= RodObject;
				Desc.mLocalPivot0	= WheelPt[0];
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, BoxDesc.mExtents.z);
				Hinges[NbHinges++] = Desc;

				Desc.mObject0		= FrontAxleObject[1];
				Desc.mObject1		= RodObject;
				Desc.mLocalPivot0	= WheelPt[1];
				Desc.mLocalPivot1	= Point(0.0f, 0.0f, -BoxDesc.mExtents.z);
				Hinges[NbHinges++] = Desc;
			}
		}

		ASSERT(NbHinges<8);
		for(udword j=0;j<Multiplier;j++)
		{
			for(udword i=0;i<NbHinges;i++)
			{
				PintJointHandle h = pint.CreateJoint(Hinges[i]);
				ASSERT(h);
			}
		}

		if(Articulation)
			pint.AddArticulationToScene(Articulation);
		return true;
	}

END_TEST(ArticulatedVehicle)

///////////////////////////////////////////////////////////////////////////////
