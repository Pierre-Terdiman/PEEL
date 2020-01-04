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

#ifndef B3_TRIANGLE_SHAPE_H
#define B3_TRIANGLE_SHAPE_H

#include <bounce/dynamics/shapes/shape.h>

// A triangle shape.
class b3TriangleShape : public b3Shape
{
public:
	b3TriangleShape();
	~b3TriangleShape();

	// Set this triangle as an isolated triangle.
	void Set(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3);

	void Clone(const b3TriangleShape& other);

	void ComputeMass(b3MassData* data, scalar density) const;

	void ComputeAABB(b3AABB* aabb, const b3Transform& xf) const;

	bool TestSphere(const b3Sphere& sphere, const b3Transform& xf) const;

	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const;

	// The triangle vertices.
	b3Vec3 m_vertex1, m_vertex2, m_vertex3;

	// Optional edge wing vertices. 
	// These are used for smooth edge collision.
	bool m_hasE1Vertex, m_hasE2Vertex, m_hasE3Vertex;
	b3Vec3 m_e1Vertex, m_e2Vertex, m_e3Vertex;
};

#endif