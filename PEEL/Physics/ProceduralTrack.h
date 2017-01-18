///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PROCEDURAL_TRACK_H
#define PROCEDURAL_TRACK_H

	class RaceTrack
	{
		public:
						RaceTrack();
						~RaceTrack();

			bool		Build();

			udword		mNbVerts;
			udword		mNbTris;
			Point*		mVerts;
			udword*		mIndices;
	};

#endif
