///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_PHYSX33_H
#define PINT_PHYSX33_H

#include "..\Pint.h"
#include "..\PINT_Common\PINT_CommonPhysX3.h"
#include "..\PINT_Common\PINT_CommonPhysX3_Vehicles.h"

	class PhysX : public SharedPhysX_Vehicles
	{
		public:
													PhysX(const EditableParams& params);
		virtual										~PhysX();

		// Pint
#ifdef PINT_SUPPORT_PVD	// Defined in project's properties
		virtual	const char*							GetName()				const	{ return "PhysX 3.3.0 (PVD)";	}
#else
		virtual	const char*							GetName()				const	{ return "PhysX 3.3.0";	}
#endif
		virtual	void								GetCaps(PintCaps& caps)	const;
		virtual	void								Init(const PINT_WORLD_CREATE& desc);
		virtual	void								Close();
		virtual	udword								Update(float dt);
		virtual	Point								GetMainColor();

		virtual	void*								CreatePhantom(const AABB& box);
		virtual	udword								BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**);

		//
		virtual	udword								BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		//
		virtual	udword								BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword								BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword								BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		//
		//~Pint

				void								UpdateFromUI();
		private:
				struct InternalShape
				{
					InternalShape(PxShape* shape, const PxMaterial* material, const PxTransform& local_pose, PxU16 collision_group) :
						mShape			(shape),
						mMaterial		(material),
						mLocalPose		(local_pose),
						mCollisionGroup	(collision_group)
					{
					}
					PxShape*						mShape;
					const PxMaterial*				mMaterial;
					PxTransform						mLocalPose;
					PxU16							mCollisionGroup;

					inline_ bool Compare(const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group)	const
					{
						if(		mMaterial==&material
							&&	mLocalPose.p.x==local_pose.p.x
							&&	mLocalPose.p.y==local_pose.p.y
							&&	mLocalPose.p.z==local_pose.p.z
							&&	mLocalPose.q.x==local_pose.q.x
							&&	mLocalPose.q.y==local_pose.q.y
							&&	mLocalPose.q.z==local_pose.q.z
							&&	mLocalPose.q.w==local_pose.q.w
							&&	mCollisionGroup==collision_group
							)
							return true;
						return false;
					}
				};

				struct InternalSphereShape : InternalShape
				{
					InternalSphereShape(float radius, PxShape* shape, const PxMaterial* material, const PxTransform& local_pose, PxU16 collision_group) :
						mRadius			(radius),
						InternalShape	(shape, material, local_pose, collision_group)
					{
					}
					float							mRadius;
				};

				struct InternalBoxShape : InternalShape
				{
					InternalBoxShape(const PxVec3& extents, PxShape* shape, const PxMaterial* material, const PxTransform& local_pose, PxU16 collision_group) :
						mExtents		(extents),
						InternalShape	(shape, material, local_pose, collision_group)
					{
					}
					PxVec3							mExtents;
				};

				struct InternalCapsuleShape : InternalShape
				{
					InternalCapsuleShape(float radius, float half_height, PxShape* shape, const PxMaterial* material, const PxTransform& local_pose, PxU16 collision_group) :
						mRadius			(radius),
						mHalfHeight		(half_height),
						InternalShape	(shape, material, local_pose, collision_group)
					{
					}
					float							mRadius;
					float							mHalfHeight;
				};

				struct InternalConvexShape : InternalShape
				{
					InternalConvexShape(PxConvexMesh* convex_mesh, PxShape* shape, const PxMaterial* material, const PxTransform& local_pose, PxU16 collision_group) :
						mConvexMesh		(convex_mesh),
						InternalShape	(shape, material, local_pose, collision_group)
					{
					}
					PxConvexMesh*					mConvexMesh;
				};

				PxProfileZoneManager*				mProfileZoneManager;

				std::vector<InternalSphereShape>	mSphereShapes;
				std::vector<InternalBoxShape>		mBoxShapes;
				std::vector<InternalCapsuleShape>	mCapsuleShapes;
				std::vector<InternalConvexShape>	mConvexShapes;

				PxShape*							CreateSphereShape	(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxSphereGeometry& geometry,		const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group);
				PxShape*							CreateBoxShape		(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxBoxGeometry& geometry,			const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group);
				PxShape*							CreateCapsuleShape	(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxCapsuleGeometry& geometry,		const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group);
				PxShape*							CreateConvexShape	(const PINT_SHAPE_CREATE* create, PxRigidActor* actor, const PxConvexMeshGeometry& geometry,	const PxMaterial& material, const PxTransform& local_pose, PxU16 collision_group);
				void								CreateShapes(const PINT_OBJECT_CREATE& desc, PxRigidActor* actor);
	};

	IceWindow*	PhysX_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		PhysX_CloseGUI();
	void		PhysX_Init(const PINT_WORLD_CREATE& desc);
	void		PhysX_Close();
	PhysX*		GetPhysX();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif