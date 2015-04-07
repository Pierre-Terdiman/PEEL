///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_H
#define COMMON_H

#define PEEL_PUBLIC_BUILD

	inline_ float	degToRad(float a)
	{
		return (float)0.01745329251994329547 * a;
	}

	Quat ShortestRotation(const Point& v0, const Point& v1);

	const char* FindPEELFile(const char* filename);

#endif
