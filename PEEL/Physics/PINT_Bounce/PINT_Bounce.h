///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_BOUNCE_H
#define PINT_BOUNCE_H

#include "..\Pint.h"

	class BouncePint : public Pint
	{
		public:
									BouncePint();
		virtual						~BouncePint();

		// Pint
		virtual	const char*			GetName()				const	{ return "Bounce";	}
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
		//~Pint

		private:
	};

	IceWindow*		Bounce_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void			Bounce_CloseGUI();
	void			Bounce_Init(const PINT_WORLD_CREATE& desc);
	void			Bounce_Close();
	BouncePint*		GetBounce();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif