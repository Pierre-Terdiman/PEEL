///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef MY_CONVEX_H
#define MY_CONVEX_H

	// Support for old convex files from optimized SAT demo

	class MyPoly : public Allocateable
	{
		public:
					MyPoly() : mNbVerts(0), mIndices(NULL)
					{
					}
					~MyPoly()
					{
						DELETEARRAY(mIndices);
					}

		short		mNbVerts;
		char*		mIndices;
		float		mPlane[4];
		float		mMin;
	};

	class MyConvex : public Allocateable
	{
		public:
								MyConvex();
								~MyConvex();

				bool			LoadFromFile(const char* filename);
				bool			LoadFile(udword i);
				void			Release();

				int				mNbVerts;
				Point*			mVerts;
				int				mNbPolys;
				MyPoly*			mPolys;
	};

#endif
