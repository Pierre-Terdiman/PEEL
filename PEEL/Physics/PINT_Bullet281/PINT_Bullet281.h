///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_BULLET_H
#define PINT_BULLET_H

#include "..\Pint.h"
#include "btBulletDynamicsCommon.h"

	class Bullet : public Pint
	{
		public:
														Bullet();
		virtual											~Bullet();

		// Pint
		virtual	const char*								GetName()				const	{ return "Bullet 2.81";	}
		virtual	void									GetCaps(PintCaps& caps)	const;
		virtual	void									Init(const PINT_WORLD_CREATE& desc);
		virtual	void									SetGravity(const Point& gravity);
		virtual	void									Close();
		virtual	udword									Update(float dt);
		virtual	Point									GetMainColor();
		virtual	void									Render(PintRender& renderer);

		virtual	void									SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle						CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool									ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle							CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	udword									BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword									BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword									BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword									BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);

		virtual	PR										GetWorldTransform(PintObjectHandle handle);
		virtual	void									SetWorldTransform(PintObjectHandle handle, const PR& pose);

//		virtual	void									ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void									AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
		virtual	void									AddLocalTorque(PintObjectHandle handle, const Point& local_torque);

		virtual	Point									GetAngularVelocity(PintObjectHandle handle);
		virtual	void									SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity);

		virtual	float									GetMass(PintObjectHandle handle);
		virtual	Point									GetLocalInertia(PintObjectHandle handle);

		virtual	udword									CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc);
		virtual	udword									BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps);
		//~Pint

		private:
				btDefaultCollisionConfiguration*		mCollisionConfiguration;
				btCollisionDispatcher*					mDispatcher;
				btBroadphaseInterface*					mBroadPhase;
				btSequentialImpulseConstraintSolver*	mSolver;
				btDiscreteDynamicsWorld*				mDynamicsWorld;
				udword									mGroupMasks[32];

				struct InternalBoxShape
				{
					btBoxShape*	mShape;
					Point		mExtents;	// Necessary for sharing, as Bullet doesn't store the un-scaled extents
				};

				struct InternalMeshShape
				{
					btBvhTriangleMeshShape*		mShape;
					btTriangleIndexVertexArray*	mMeshData;
					btTriangleInfoMap*			mTriangleInfoMap;
				};

				std::vector<btTypedConstraint*>			mConstraints;
				std::vector<btCollisionShape*>			mCollisionShapes;
				std::vector<btSphereShape*>				mSphereShapes;
				std::vector<InternalBoxShape>			mBoxShapes;
				std::vector<btCapsuleShape*>			mCapsuleShapes;
				std::vector<btConvexHullShape*>			mConvexShapes;
				std::vector<InternalMeshShape>			mMeshShapes;

				std::vector<btConvexHullShape*>			mConvexObjects;

				btCollisionShape*						CreateBulletShape(const PINT_SHAPE_CREATE& desc);
				btSphereShape*							FindSphereShape(const PINT_SPHERE_CREATE& create);
				btBoxShape*								FindBoxShape(const PINT_BOX_CREATE& create);
				btCapsuleShape*							FindCapsuleShape(const PINT_CAPSULE_CREATE& create);
				btConvexHullShape*						FindConvexShape(const PINT_CONVEX_CREATE& create);
	};

	IceWindow*	Bullet_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		Bullet_CloseGUI();
	void		Bullet_Init(const PINT_WORLD_CREATE& desc);
	void		Bullet_Close();
	Bullet*		GetBullet();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif