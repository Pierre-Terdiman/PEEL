///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_PHYSX284_H
#define PINT_PHYSX284_H

#include "..\Pint.h"

	class NxPhysicsSDK;
	class NxScene;
	class NxMaterial;
	class NxCookingInterface;
	class NxConvexMesh;
	class NxTriangleMesh;

	class PhysX284 : public Pint
	{
		public:
													PhysX284();
		virtual										~PhysX284();

		// Pint
		virtual	const char*							GetName()				const	{ return "PhysX 2.8.4";	}
		virtual	void								GetCaps(PintCaps& caps)	const;
		virtual	void								Init(const PINT_WORLD_CREATE& desc);
		virtual	void								SetGravity(const Point& gravity);
		virtual	void								Close();
		virtual	udword								Update(float dt);
		virtual	Point								GetMainColor();
		virtual	void								Render(PintRender& renderer);

		virtual	void								SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle					CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool								ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle						CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	udword								BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword								BatchRaycastAny				(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts);
		virtual	udword								BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword								BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword								BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		virtual	udword								BatchSphereOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword								BatchSphereOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword								BatchBoxOverlapAny			(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps);
		virtual	udword								BatchBoxOverlapObjects		(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps);
		virtual	udword								BatchCapsuleOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps);
		virtual	udword								BatchCapsuleOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps);

		virtual	PR									GetWorldTransform(PintObjectHandle handle);
		virtual	void								ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);

		virtual	bool								SetKinematicPose(PintObjectHandle handle, const Point& pos);
		//~Pint

				void								UpdateFromUI();
		private:
				NxPhysicsSDK*						mSDK;
				NxScene*							mScene;
				NxCookingInterface*					mCooking;

				std::vector<NxMaterial*>			mMaterials;
				NxMaterial*							CreateMaterial(const PINT_MATERIAL_CREATE& desc);

				struct ConvexRender
				{
					NxConvexMesh*			mConvexMesh;
					PintShapeRenderer*		mRenderer;
				};

				struct MeshRender
				{
					NxTriangleMesh*			mTriangleMesh;
					PintShapeRenderer*		mRenderer;
				};

				std::vector<ConvexRender>			mConvexes;
				std::vector<MeshRender>				mMeshes;
				NxConvexMesh*						CreateConvexMesh(const Point* verts, udword vertCount, udword flags, PintShapeRenderer* renderer);
				NxTriangleMesh*						CreateTriangleMesh(const SurfaceInterface& surface, PintShapeRenderer* renderer);
	};

	IceWindow*	PhysX284_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		PhysX284_CloseGUI();
	void		PhysX284_Init(const PINT_WORLD_CREATE& desc);
	void		PhysX284_Close();
	PhysX284*	GetPhysX284();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif