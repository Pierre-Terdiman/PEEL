///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_SKELETON_H
#define PINT_SKELETON_H

#include "..\Pint.h"

	class SkeletonPint : public Pint
	{
		public:
									SkeletonPint();
		virtual						~SkeletonPint();

		// Pint
		virtual	const char*			GetName()				const	{ return "Skeleton";	}
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
		//~Pint

		private:
	};

	IceWindow*		Skeleton_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void			Skeleton_CloseGUI();
	void			Skeleton_Init(const PINT_WORLD_CREATE& desc);
	void			Skeleton_Close();
	SkeletonPint*	GetSkeleton();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif