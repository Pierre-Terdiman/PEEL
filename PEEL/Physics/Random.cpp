///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Random.h"

void UnitRandomPt(Point& v, BasicRandom& rnd)
{
	v.x = rnd.RandomFloat();
	v.y = rnd.RandomFloat();
	v.z = rnd.RandomFloat();
	v.Normalize();
}

void UnitRandomQuat(Quat& v, BasicRandom& rnd)
{
	v.p.x = rnd.RandomFloat();
	v.p.y = rnd.RandomFloat();
	v.p.z = rnd.RandomFloat();
	v.w = rnd.RandomFloat();
	v.Normalize();
}

