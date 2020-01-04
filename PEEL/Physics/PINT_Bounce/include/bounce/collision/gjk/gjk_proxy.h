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

#ifndef B3_GJK_PROXY_H
#define B3_GJK_PROXY_H

#include <bounce/common/math/vec3.h>

// A GJK proxy encapsulates any convex hull to be used by the GJK.
struct b3GJKProxy
{
	const b3Vec3* vertices; // vertices in this proxy
	u32 vertexCount; // number of vertices
	scalar radius; // proxy radius
	b3Vec3 vertexBuffer[3]; // vertex buffer for convenience

	// Get the number of vertices in this proxy.
	u32 GetVertexCount() const;

	// Read an indexed vertex from this proxy.
	const b3Vec3& GetVertex(u32 index) const;

	// Get the support vertex index in a given direction.
	u32 GetSupportIndex(const b3Vec3& direction) const;

	// Convenience function.
	// Get the support vertex in a given direction.
	const b3Vec3& GetSupportVertex(const b3Vec3& direction) const;
};

inline u32 b3GJKProxy::GetVertexCount() const
{
	return vertexCount;
}

inline const b3Vec3& b3GJKProxy::GetVertex(u32 index) const
{
	B3_ASSERT(0 <= index && index < vertexCount);
	return vertices[index];
}

inline u32 b3GJKProxy::GetSupportIndex(const b3Vec3& d) const
{
	u32 maxIndex = 0;
	scalar maxProjection = b3Dot(d, vertices[maxIndex]);
	for (u32 i = 1; i < vertexCount; ++i)
	{
		scalar projection = b3Dot(d, vertices[i]);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline const b3Vec3& b3GJKProxy::GetSupportVertex(const b3Vec3& d) const
{
	u32 index = GetSupportIndex(d);
	return vertices[index];
}

#endif