///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_PHYSX32_H
#define PINT_PHYSX32_H

#include "..\Pint.h"
#include "..\PINT_Common\PINT_CommonPhysX3.h"

	class PhysX : public SharedPhysX
	{
		public:
										PhysX(const EditableParams& params);
		virtual							~PhysX();

		// Pint
		virtual	const char*				GetName()				const	{ return "PhysX 3.2";	}
		virtual	void					GetCaps(PintCaps& caps)	const;
		virtual	void					Init(const PINT_WORLD_CREATE& desc);
		virtual	void					Close();
		virtual	udword					Update(float dt);
		virtual	Point					GetMainColor();

//		virtual	PintObjectHandle		CreateObject(const PINT_OBJECT_CREATE& desc);

		//
		virtual	udword					BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		//
		virtual	udword					BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword					BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword					BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		//

//		virtual	void					ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void					AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
		virtual	void					AddLocalTorque(PintObjectHandle handle, const Point& local_torque);
		//~Pint

				void					UpdateFromUI();
		private:
				PxProfileZoneManager*	mProfileZoneManager;
				void					CreateShapes(const PINT_OBJECT_CREATE& desc, PxRigidActor* actor);
	};

	IceWindow*	PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		PhysX_CloseGUI();
	void		PhysX_Init(const PINT_WORLD_CREATE& desc);
	void		PhysX_Close();
	PhysX*		GetPhysX();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif