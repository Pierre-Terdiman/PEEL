/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_DRAW_H
#define B3_DRAW_H

#include <bounce/common/math/math.h>
#include <bounce/collision/shapes/aabb.h>

// Color channels used by the debug draw interface.
struct b3Color 
{
	b3Color() { }
	
	b3Color(scalar R, scalar G, scalar B, scalar A = scalar(1)) : r(R),	g(G), b(B),	a(A) { }
	
	scalar r, g, b, a;
};

// Color pallete commonly used by the debug draw interface.
extern const b3Color b3Color_black;
extern const b3Color b3Color_white;
extern const b3Color b3Color_red;
extern const b3Color b3Color_green;
extern const b3Color b3Color_blue;
extern const b3Color b3Color_yellow;
extern const b3Color b3Color_pink;
extern const b3Color b3Color_gray;

// Implement this interface and set to a world so it can draw the physics entities.
class b3Draw 
{
public :
	// Bit flags to tell the world what needs to be draw.
	enum b3Flags
	{
		e_shapesFlag = 0x0001,
		e_centerOfMassesFlag = 0x0002,
		e_jointsFlag = 0x0004,
		e_contactPointsFlag = 0x0008,
		e_contactNormalsFlag = 0x0010,
		e_contactTangentsFlag = 0x0020,
		e_contactPolygonsFlag = 0x0040,
		e_aabbsFlag = 0x0080,
	};

	b3Draw()
	{
		m_flags = 0;
	}
	
	~b3Draw()
	{

	}

	void SetFlags(u32 flags);
	void AppendFlags(u32 flags);
	
	// Draw a point.
	virtual void DrawPoint(const b3Vec3& p, scalar size, const b3Color& color) = 0;

	// Draw a line segment.
	virtual void DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color) = 0;

	// Draw a triangle with vertices ordered CCW.
	virtual void DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color) = 0;

	// Draw a solid triangle with vertices ordered CCW.
	virtual void DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color) = 0;

	// Draw a polygon with vertices ordered CCW.
	virtual void DrawPolygon(const b3Vec3* vertices, u32 count, const b3Color& color) = 0;
	
	// Draw a solid polygon with vertices ordered CCW.
	virtual void DrawSolidPolygon(const b3Vec3& normal, const b3Vec3* vertices, u32 count, const b3Color& color) = 0;

	// Draw a circle with center, normal, and radius.
	virtual void DrawCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color) = 0;
	
	// Draw a solid circle with center, normal, and radius.
	virtual void DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color) = 0;

	// Draw a plane with center, normal and radius.
	virtual void DrawPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color) = 0;

	// Draw a solid plane with center, normal and radius.
	virtual void DrawSolidPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color) = 0;

	// Draw a sphere with center, and radius.
	virtual void DrawSphere(const b3Vec3& center, scalar radius, const b3Color& color) = 0;

	// Draw a solid sphere with center, radius, and rotation.
	virtual void DrawSolidSphere(const b3Vec3& center, scalar radius, const b3Quat& rotation, const b3Color& color) = 0;
	
	// Draw a capsule with segment, and radius.
	virtual void DrawCapsule(const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Color& color) = 0;

	// Draw a solid capsule with segment, radius, and rotation.
	virtual void DrawSolidCapsule(const b3Vec3& p1, const b3Vec3& p2, scalar radius, const b3Quat& rotation, const b3Color& color) = 0;

	// Draw a AABB.
	virtual void DrawAABB(const b3AABB& aabb, const b3Color& color) = 0;

	// Draw a transform.
	virtual void DrawTransform(const b3Transform& xf) = 0;

	// Debug draw flags.
	u32 m_flags;
};

// The debug drawer interface used by Bounce. 
// You should set this to an implementation 
// before calling any function that uses the interface.
extern b3Draw* b3Draw_draw;

inline void b3Draw::SetFlags(u32 flags)
{
	m_flags = flags;
}

inline void b3Draw::AppendFlags(u32 flags)
{
	m_flags |= flags;
}

#endif
