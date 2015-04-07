///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef SURFACE_MANAGER_H
#define SURFACE_MANAGER_H

	class SurfaceManager
	{
		public:
								SurfaceManager();
		virtual					~SurfaceManager();

				IndexedSurface*	CreateManagedSurface();
				void			ReleaseManagedSurfaces();

		inline_	udword			GetNbSurfaces()				const			{ return mSurfaces.GetNbEntries();					}
		inline_	IndexedSurface*	GetFirstSurface()			const			{ return (IndexedSurface*)mSurfaces.GetEntry(0);	}
		inline_	IndexedSurface*	GetSurface(udword i)		const			{ return (IndexedSurface*)mSurfaces.GetEntry(i);	}

		inline_	void			SetGlobalBounds(const AABB& global_bounds)	{ mGlobalBounds = global_bounds;	}

		inline_	void			GetGlobalBounds(Point& center, Point& extents)
								{
									mGlobalBounds.GetCenter(center);
									mGlobalBounds.GetExtents(extents);
								}
		private:
				AABB			mGlobalBounds;
				Container		mSurfaces;
	};

#endif
