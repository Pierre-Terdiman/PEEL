///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_STATS_H
#define PINT_STATS_H

#include "..\Pint.h"

	class Stats
	{
		public:
									Stats();

				udword				mNbStatics;
				udword				mNbDynamics;
				udword				mNbStaticCompounds;
				udword				mNbDynamicCompounds;
				udword				mNbJoints;
				udword				mNbBoxShapes;
				udword				mNbSphereShapes;
				udword				mNbCapsuleShapes;
				udword				mNbConvexShapes;
				udword				mNbMeshShapes;
				udword				mTotalNbVerts;
				udword				mTotalNbTris;
	};

	class StatsPint : public Pint
	{
		public:
									StatsPint();
		virtual						~StatsPint();

		// Pint
		virtual	const char*			GetName()				const	{ return "Scene stats";	}
		virtual	void				GetCaps(PintCaps& caps)	const;
		virtual	void				Init(const PINT_WORLD_CREATE& desc);
		virtual	void				SetGravity(const Point& gravity);
		virtual	void				Close();
		virtual	udword				Update(float dt);
		virtual	Point				GetMainColor();
		virtual	void				Render(PintRender& renderer);

		virtual	void				SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle	CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool				ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle		CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	void*				CreatePhantom(const AABB& box);
		virtual	udword				BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**);

		virtual	udword				BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword				BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword				BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword				BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		virtual	udword				BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword				BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);

		virtual	PR					GetWorldTransform(PintObjectHandle handle);
		virtual	void				ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);

		// XP
		virtual	udword				GetShapes(PintObjectHandle* shapes, PintObjectHandle handle);
		virtual	void				SetLocalRot(PintObjectHandle handle, const Quat& q);
		virtual	bool				GetConvexData(PintObjectHandle handle, PINT_CONVEX_CREATE& data);
		virtual	udword				GetFlags()				const	{ return 0;	}
		//~Pint

		private:
				PINT_WORLD_CREATE	mCreate;
				Stats				mStats;
				bool				mUpdateUI;
	};

	IceWindow*	Stats_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		Stats_CloseGUI();
	void		Stats_Init(const PINT_WORLD_CREATE& desc);
	void		Stats_Close();
	StatsPint*	GetStats();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif