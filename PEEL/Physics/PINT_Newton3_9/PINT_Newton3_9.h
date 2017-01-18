///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_NEWTON_H
#define PINT_NEWTON_H

#include "..\Pint.h"
#include "Newton.h"
#include <vector>

typedef std::vector< NewtonBody * > NewtonBodyVector;

	class NewtonPint : public Pint
	{
		public:
									NewtonPint();
		virtual						~NewtonPint();

		// Pint
		virtual	const char*			GetName() const;
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

		virtual	udword				BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword				BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword				BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword				BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);

		virtual	PR					GetWorldTransform(PintObjectHandle handle);
		virtual	void				SetWorldTransform(PintObjectHandle handle, const PR& pose);

//		virtual	void				ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void				AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
//		virtual	void				AddLocalTorque(PintObjectHandle handle, const Point& local_torque);

		virtual	Point				GetAngularVelocity(PintObjectHandle handle);
		virtual	void				SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity);

		virtual	float				GetMass(PintObjectHandle handle);
		virtual	Point				GetLocalInertia(PintObjectHandle handle);

		//~Pint
		private:
		void						Render(PintRender& renderer, NewtonCollision* const collsion, const IceMaths::Matrix4x4& worldPose) const;
		NewtonCollision*			CreateShape(const PINT_OBJECT_CREATE& desc) const;
		NewtonCollision*			CreateShape(NewtonWorld* const world, const PINT_SHAPE_CREATE* const shape, const PintShapeRenderer* const renderer, int groupID) const;

		static dFloat				GenericRayCast(const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const normal, int* const collisionID, void* const userData, dFloat intersetParam);

		static int					OnAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
		static	void				ApplyForceAndTorqueCallback(const NewtonBody* const body, dFloat timestep, int threadIndex);

		static  void				DuplicateCollisionShape(const NewtonWorld* const newtonWorld, NewtonCollision* const newCollision, const NewtonCollision* const sourceCollision);
		static  void				DestroyCollisionShape(const NewtonWorld* const newtonWorld, const NewtonCollision* const collision);
		
		udword						mGroupMasks[32];
		NewtonWorld*				mWorld;
		NewtonBodyVector			mBodies;
		Point						mGlobalGravity;
	};

	IceWindow*	Newton_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		Newton_CloseGUI();
	void		Newton_Init(const PINT_WORLD_CREATE& desc);
	void		Newton_Close();
	NewtonPint*	GetNewton();

	

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif