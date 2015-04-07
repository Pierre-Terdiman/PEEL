///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef FILE_FINDER_H
#define FILE_FINDER_H

	// ### TODO: move this back to ICE
	class FileFinder2 : public Allocateable
	{
		public:
									FileFinder2(const char* mask);
				bool				FindNext();
		inline_	bool				IsValid()	const	{ return (mHandle!=(void*)-1);	}

		public:
				CHAR				mFile[MAX_PATH];
				udword				mAttribs;
				udword				mSize;
		private:
				WIN32_FIND_DATAA	mFindData;
				HANDLE				mHandle;
		inline_	void				ProcessFile();
	};

	typedef bool (*FileFinderCallback)(const char* filename, const char* path, void* user_data);

	struct FindFilesParams
	{
		FindFilesParams() :
			mMask				(null)
		{
			mDir[0] = 0;
		}
		const char*	mMask;
		CHAR		mDir[MAX_PATH];
	};

	void _FindFiles(FindFilesParams& params, FileFinderCallback callback, void* user_data);

#endif
