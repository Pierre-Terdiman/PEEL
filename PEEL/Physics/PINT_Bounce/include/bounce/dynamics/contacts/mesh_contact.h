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

#ifndef B3_MESH_CONTACT_H
#define B3_MESH_CONTACT_H

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/manifold.h>
#include <bounce/dynamics/contacts/collide/collide.h>
#include <bounce/collision/shapes/aabb.h>

// This structure helps replicate the convex contact per convex-triangle pair scenario, 
// but efficiently. There is no need to store a manifold here since they're reduced 
// by the cluster algorithm.
struct b3TriangleCache
{
	u32 index; // triangle index
	b3ConvexCache cache;
};

class b3MeshContact;

// Links for the world mesh contact link list.
struct b3MeshContactLink
{
	b3MeshContact* m_c;
	b3MeshContactLink* m_prev;
	b3MeshContactLink* m_next;
};

class b3MeshContact : public b3Contact
{
public:
	static b3Contact* Create(b3Shape* shapeA, b3Shape* shapeB, b3BlockPool* allocator);
	static void Destroy(b3Contact* contact, b3BlockPool* allocator);

	b3MeshContact(b3Shape* shapeA, b3Shape* shapeB);
	~b3MeshContact();

	bool TestOverlap();

	void Collide();

	void SynchronizeShapes();

	bool MoveAABB(const b3AABB& aabb, const b3Vec3& displacement);

	void FindNewPairs();

	// Static tree callback. There is no midphase. 
	bool Report(u32 proxyId);

	// Did the AABB move significantly?
	bool m_aabbBMoved;

	// The AABB B relative to shape A's origin.
	b3AABB m_aabbB; 
	
	// Triangles potentially overlapping with the first shape.
	u32 m_triangleCapacity;
	b3TriangleCache* m_triangles;
	u32 m_triangleCount;

	// Contact manifolds.
	b3Manifold m_stackManifolds[B3_MAX_MANIFOLDS];

	// Link to the world mesh contact list.
	b3MeshContactLink m_link;
};

#endif