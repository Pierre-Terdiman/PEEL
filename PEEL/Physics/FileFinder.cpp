///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "FileFinder.h"

inline_ void FileFinder2::ProcessFile()
{
	memcpy(mFile, mFindData.cFileName, MAX_PATH);
	mSize		= mFindData.nFileSizeLow;
	mAttribs	= 0;
	mAttribs	|= (mFindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)	? FINDFILE_DIR		: 0;
	mAttribs	|= (mFindData.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)		? FINDFILE_HIDDEN	: 0;
	mAttribs	|= (mFindData.dwFileAttributes & FILE_ATTRIBUTE_NORMAL)		? FINDFILE_NORMAL	: 0;
	mAttribs	|= (mFindData.dwFileAttributes & FILE_ATTRIBUTE_READONLY)	? FINDFILE_READONLY	: 0;
}

FileFinder2::FileFinder2(const char* mask)
{
	mHandle = FindFirstFileA(mask, &mFindData);

	// Did we find a file ?
	if(!IsValid())
	{
		mFile[0]	= 0;
		mSize		= 0;
		mAttribs	= 0;
	}
	else
	{
		ProcessFile();
	}
}

bool FileFinder2::FindNext()
{
	if(!FindNextFileA(mHandle, &mFindData))
	{
		mHandle = (void*)-1;
		FindClose(mHandle);
		return false;
	}
	ProcessFile();
	return true;
}

void _FindFiles(FindFilesParams& params, FileFinderCallback callback, void* user_data)
{
	CHAR CurrentMask[MAX_PATH];
	strcpy(CurrentMask, params.mDir);
	strcat(CurrentMask, params.mMask);

	FileFinder2 FF(CurrentMask);
	while(FF.IsValid())
	{
		if(FF.mAttribs & FINDFILE_HIDDEN)	// Skip hidden files & dirs
			continue;

		if(FF.mAttribs & FINDFILE_DIR)
		{
			// Skip "." and ".." directories
			if(FF.mFile[0]=='.' && ((FF.mFile[1]==0) || (FF.mFile[1]=='.' && FF.mFile[2]==0)))
			{
				// Invalid dir
			}
			else
			{
				const udword Offset = strlen(params.mDir);
				strcat(params.mDir, FF.mFile);
				strcat(params.mDir, "/");
//				printf("%s\n", params.mDir);

				_FindFiles(params, callback, user_data);
				params.mDir[Offset]=0;
			}
		}
		else
		{
			(callback)(FF.mFile, params.mDir, user_data);
		}

		FF.FindNext();
	}
}

