///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PintObjectsManager.h"

///////////////////////////////////////////////////////////////////////////////

static udword gEntrySize = sizeof(PintObjectHandle)/sizeof(udword);

ObjectsManager::ObjectsManager() : mOwner(null)
{
}

ObjectsManager::~ObjectsManager()
{
	Reset();
}

void ObjectsManager::Init(Pint* owner)
{
	Reset();
	mOwner	= owner;
	owner->mOMHelper = this;
}

udword ObjectsManager::GetNbObjects() const
{
	return mObjects.GetNbEntries()/gEntrySize; 
}

PintObjectHandle ObjectsManager::GetObject(udword i) const
{
	PintObjectHandle* Entries = (PintObjectHandle*)mObjects.GetEntries();
	return Entries[i];
}

void ObjectsManager::AddObject(PintObjectHandle object)
{
	PintObjectHandle* Memory = (PintObjectHandle*)mObjects.Reserve(gEntrySize);
	*Memory = object;
}

void ObjectsManager::Reset()
{
	mOwner = null;
	mObjects.Empty();
}


