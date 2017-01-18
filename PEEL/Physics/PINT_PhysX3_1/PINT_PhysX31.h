///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_PHYSX31_H
#define PINT_PHYSX31_H

#include "..\Pint.h"

	class PhysX : public Pint
	{
		public:
											PhysX();
		virtual								~PhysX();

		// Pint
		virtual	const char*					GetName()				const	{ return "PhysX 3.1.1";	}
		virtual	void						GetCaps(PintCaps& caps)	const;
		virtual	void						Init(const PINT_WORLD_CREATE& desc);
		virtual	void						SetGravity(const Point& gravity);
		virtual	void						Close();
		virtual	udword						Update(float dt);
		virtual	Point						GetMainColor();
		virtual	void						Render(PintRender& renderer);

		virtual	void						SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle			CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool						ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle				CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	udword						BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword						BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword						BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword						BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		virtual	udword						BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword						BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);

		virtual	PR							GetWorldTransform(PintObjectHandle handle);

//		virtual	void						ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
		virtual	void						AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
		virtual	void						AddLocalTorque(PintObjectHandle handle, const Point& local_torque);

//		virtual	float						GetMass(PintObjectHandle handle);
//		virtual	Point						GetLocalInertia(PintObjectHandle handle);

		virtual	udword						GetShapes(PintObjectHandle* shapes, PintObjectHandle handle);
		virtual	void						SetLocalRot(PintObjectHandle handle, const Quat& q);
		//~Pint

				void						UpdateFromUI();
		private:
				PxFoundation*				mFoundation;
//				PxProfileZoneManager*		mProfileZoneManager;
				PxPhysics*					mPhysics;
				PxCooking*					mCooking;
				PxScene*					mScene;
				PxMaterial*					mDefaultMaterial;

				struct ConvexRender
				{
					PxConvexMesh*			mConvexMesh;
					PintShapeRenderer*		mRenderer;
				};

				struct MeshRender
				{
					PxTriangleMesh*			mTriangleMesh;
					PintShapeRenderer*		mRenderer;
				};

				std::vector<PxMaterial*>	mMaterials;
				std::vector<ConvexRender>	mConvexes;
				std::vector<MeshRender>		mMeshes;
				PxMaterial*					CreateMaterial(const PINT_MATERIAL_CREATE& desc);
				PxConvexMesh*				CreateConvexMesh(const Point* verts, udword vertCount, PxConvexFlags flags, PintShapeRenderer* renderer);
				PxTriangleMesh*				CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer);

				void*						mScratchPad;
	};

	IceWindow*	PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		PhysX_CloseGUI();
	void		PhysX_Init(const PINT_WORLD_CREATE& desc);
	void		PhysX_Close();
	PhysX*		GetPhysX();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif