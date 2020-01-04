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

#ifndef B3_MESH_H
#define B3_MESH_H

#include <bounce/common/geometry.h>
#include <bounce/collision/trees/static_tree.h>

#define B3_NULL_VERTEX B3_MAX_U32

// Mesh triangle.
struct b3MeshTriangle
{
	// Write an indexed vertex to this triangle.
	u32& GetVertex(u32 i) { return (&v1)[i]; }

	// Read an indexed vertex from this triangle.
	u32 GetVertex(u32 i) const { return (&v1)[i]; }

	// The triangle vertices in the mesh.
	u32 v1, v2, v3;
};

// Mesh triangle adjacency.
// This is used for smooth edge collision.
struct b3MeshTriangleWings
{
	// Write an indexed edge wing vertex to this triangle.
	u32& GetVertex(u32 i) { return (&u1)[i]; }

	// Read an indexed edge wing vertex from this triangle.
	u32 GetVertex(u32 i) const { return (&u1)[i]; }

	// The wing vertex of each edge in this triangle.
	// An edge is a boundary if its wing vertex is set to B3_NULL_VERTEX.
	u32 u1, u2, u3;
};

struct b3Mesh 
{
	u32 vertexCount;
	b3Vec3* vertices;
	u32 triangleCount;
	b3MeshTriangle* triangles;
	b3MeshTriangleWings* triangleWings;

	b3StaticTree tree;

	b3Mesh();
	~b3Mesh();

	// Build the static AABB tree. 
	void BuildTree();

	// Build mesh adjacency.
	// This won't work properly if there are non-manifold edges.
	void BuildAdjacency();

	const b3Vec3& GetVertex(u32 index) const;
	const b3MeshTriangle* GetTriangle(u32 index) const;
	const b3MeshTriangleWings* GetTriangleWings(u32 index) const;
	b3AABB GetTriangleAABB(u32 index) const;
	
	u32 GetSize() const;

	void Scale(const b3Vec3& scale);
	void Rotate(const b3Quat& rotation);
	void Translate(const b3Vec3& translation);

	// Scale -> Rotate -> Translate
	void Transform(const b3Transform& xf, const b3Vec3& scale);
};

inline const b3Vec3& b3Mesh::GetVertex(u32 index) const
{
	return vertices[index];
}

inline const b3MeshTriangle* b3Mesh::GetTriangle(u32 index) const
{
	return triangles + index;
}

inline const b3MeshTriangleWings* b3Mesh::GetTriangleWings(u32 index) const
{
	return triangleWings + index;
}

inline b3AABB b3Mesh::GetTriangleAABB(u32 index) const
{
	const b3MeshTriangle* triangle = triangles + index;
	
	b3Vec3 v1 = vertices[triangle->v1];
	b3Vec3 v2 = vertices[triangle->v2];
	b3Vec3 v3 = vertices[triangle->v3];

	b3AABB aabb;
	aabb.lowerBound = b3Min(v1, b3Min(v2, v3));
	aabb.upperBound = b3Max(v1, b3Max(v2, v3));

	return aabb;
}

inline u32 b3Mesh::GetSize() const
{
	u32 size = 0;
	size += sizeof(b3Mesh);
	size += sizeof(b3Vec3) * vertexCount;
	size += sizeof(b3MeshTriangle) * triangleCount;
	size += sizeof(b3MeshTriangleWings) * triangleCount;
	size += tree.GetSize();
	return size;
}

#endif