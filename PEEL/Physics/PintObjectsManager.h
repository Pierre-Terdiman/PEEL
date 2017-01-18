///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_OBJECTS_MANAGER_H
#define PINT_OBJECTS_MANAGER_H

#include "Pint.h"

//### we can't store the objects in the test class since we have N Pint-dependent versions of them
// it's the same problem as for phantoms, and what gave birth to PintSQ. Maybe rename "PintSQ" to "PintHelper"?
	class ObjectsManager
	{
		public:
									ObjectsManager();
		virtual						~ObjectsManager();

				void				Init(Pint* owner);

				udword				GetNbObjects()		const;
				PintObjectHandle	GetObject(udword i)	const;
				void				AddObject(PintObjectHandle object);
				void				Reset();
		private:
				Pint*				mOwner;
				Container			mObjects;
	};

	inline_ PintObjectHandle CreatePintObject(Pint& pint, const PINT_OBJECT_CREATE& desc)
	{
		PintObjectHandle handle = pint.CreateObject(desc);
		pint.mOMHelper->AddObject(handle);
		return handle;
	}

#endif