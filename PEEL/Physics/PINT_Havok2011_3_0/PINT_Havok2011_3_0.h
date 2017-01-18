///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_HAVOK_H
#define PINT_HAVOK_H

#include "..\Pint.h"
#include "..\PINT_Common\PINT_CommonHavok.h"

	class hkMemoryRouter;
	class hkJobThreadPool;
	class hkJobQueue;
	class hkpWorld;
	class hkpPhysicsContext;
	class hkVisualDebugger;
	class hkpRigidBody;
	class hkpShape;
	class hkpSphereShape;
	class hkpBoxShape;
	class hkpCapsuleShape;
	class hkpConvexVerticesShape;
	class hkpRigidBodyCinfo;
	struct HavokMeshRender;
	struct InternalCapsuleShape;

	class Havok : public SharedHavok
	{
		public:
														Havok(const EditableParams& params);
		virtual											~Havok();

		// Pint
		virtual	const char*								GetName()				const	{ return "Havok 2011_3_0";	}
		virtual	void									GetCaps(PintCaps& caps)	const;
		virtual	void									Init(const PINT_WORLD_CREATE& desc);
		virtual	void									SetGravity(const Point& gravity);
		virtual	void									Close();
		virtual	udword									Update(float dt);
		virtual	Point									GetMainColor();

		virtual	void									SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);

		virtual	void*									CreatePhantom(const AABB& box);
		virtual	udword									BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**);

		virtual	udword									BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword									BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword									BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword									BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		virtual	udword									BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword									BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);

		virtual	PR										GetWorldTransform(PintObjectHandle handle);
		virtual	void									SetWorldTransform(PintObjectHandle handle, const PR& pose);

//		virtual	void									ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void									AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);

		virtual	Point									GetAngularVelocity(PintObjectHandle handle);
		virtual	void									SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity);

		virtual	float									GetMass(PintObjectHandle handle);
		virtual	Point									GetLocalInertia(PintObjectHandle handle);

		virtual	void									TestNewFeature();

		virtual	bool									SetKinematicPose(PintObjectHandle handle, const Point& pos);
		virtual	bool									SetKinematicPose(PintObjectHandle handle, const PR& pr);

		virtual	udword									CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc);
		virtual	udword									BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps);
		//~Pint

				void									PrintMemoryStatistics();

				hkJobThreadPool*						mThreadPool;
	};

	IceWindow*	Havok_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		Havok_CloseGUI();
	void		Havok_Init(const PINT_WORLD_CREATE& desc);
	void		Havok_Close();
	Havok*		GetHavok();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif