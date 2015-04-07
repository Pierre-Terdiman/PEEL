///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_NOVODEX_H
#define PINT_NOVODEX_H

#include "..\Pint.h"
#include "NxArray.h"

	class NxPhysicsSDK;
	class NxScene;
	class NxMaterial;
	class NxTriangleMesh;
	class NxShape;

	class NovodeX : public Pint
	{
		public:
												NovodeX();
		virtual									~NovodeX();

		// Pint
		virtual	const char*						GetName()				const	{ return "NovodeX 2.1.1";	}
		virtual	void							GetCaps(PintCaps& caps)	const;
		virtual	void							Init(const PINT_WORLD_CREATE& desc);
		virtual	void							SetGravity(const Point& gravity);
		virtual	void							Close();
		virtual	udword							Update(float dt);
		virtual	Point							GetMainColor();
		virtual	void							Render(PintRender& renderer);

		virtual	void							SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle				CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool							ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle					CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	udword							BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword							BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
		virtual	udword							BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);

		virtual	PR								GetWorldTransform(PintObjectHandle handle);
		virtual	void							ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);

		virtual	bool							SetKinematicPose(PintObjectHandle handle, const Point& pos);
		//~Pint

				void							UpdateFromUI();
		private:
				NxPhysicsSDK*					mSDK;
				NxScene*						mScene;
				NxArraySDK<NxShape*>*			mTouchedShapes;

				std::vector<NxMaterial*>		mMaterials;
				std::vector<NxTriangleMesh*>	mMeshes;
				udword							CreateMaterial(const PINT_MATERIAL_CREATE& desc);
	};

	IceWindow*	NovodeX_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		NovodeX_CloseGUI();
	void		NovodeX_Init(const PINT_WORLD_CREATE& desc);
	void		NovodeX_Close();
	NovodeX*	GetNovodeX();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif