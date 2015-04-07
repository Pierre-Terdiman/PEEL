///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SurfaceManager.h"

SurfaceManager::SurfaceManager()
{
	mGlobalBounds.SetEmpty();
}

SurfaceManager::~SurfaceManager()
{
}

IndexedSurface* SurfaceManager::CreateManagedSurface()
{
	// Some physics engines don't copy the mesh data, so we need to keep it around for the lifetime of the test.
	IndexedSurface* IS = new IndexedSurface;
	mSurfaces.Add(udword(IS));
	return IS;
}

void SurfaceManager::ReleaseManagedSurfaces()
{
	udword Nb = mSurfaces.GetNbEntries();
	for(udword i=0;i<Nb;i++)
	{
		IndexedSurface* IS = reinterpret_cast<IndexedSurface*>(mSurfaces.GetEntry(i));
		DELETESINGLE(IS);
	}
	mSurfaces.Empty();
}
