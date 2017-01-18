///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_OPCODE13_H
#define PINT_OPCODE13_H

#include "..\Pint.h"

	class OpcodeMesh : public Allocateable
	{
		public:
									OpcodeMesh();
									~OpcodeMesh();

				IndexedSurface		mSurface;
				Model				mModel;
				MeshInterface		mMeshInterface;
				PintShapeRenderer*	mRenderer;
	};

	class OpcodeActor : public Allocateable
	{
		public:
									OpcodeActor();
									~OpcodeActor();

				OpcodeMesh*			mMesh;
				Matrix4x4			mMeshTM;
	};

	class Opcode13Pint : public Pint
	{
		public:
									Opcode13Pint();
		virtual						~Opcode13Pint();

		// Pint
		virtual	const char*			GetName()				const	{ return "Opcode 1.3";	}
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

		//
		virtual	udword				BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword				BatchRaycastAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts);
		//
		virtual	udword				BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword				BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword				BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);
		//
		virtual	udword				BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword				BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword				BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps);
		virtual	udword				BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps);
		//
		virtual	udword				FindTriangles_MeshSphereOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps);

		virtual	PintSQThreadContext	CreateSQThreadContext();
		virtual	void				ReleaseSQThreadContext(PintSQThreadContext);
		//~Pint

		private:
				Container			mMeshes;
				Container			mActors;
				Container			mWorldBoxes;
				AABBTree*			mSceneTree;

				struct SQThreadContext : public Allocateable
				{
					Container		mBoxIndices;
				};
	};

	IceWindow*		Opcode_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void			Opcode_CloseGUI();
	void			Opcode_Init(const PINT_WORLD_CREATE& desc);
	void			Opcode_Close();
	Opcode13Pint*	GetOpcode();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif