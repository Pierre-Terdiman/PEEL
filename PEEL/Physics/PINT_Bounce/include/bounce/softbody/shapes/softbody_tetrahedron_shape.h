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

#ifndef B3_CLOTH_TETRAHEDRON_SHAPE_H
#define B3_CLOTH_TETRAHEDRON_SHAPE_H

#include <bounce/softbody/shapes/softbody_shape.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3SoftBodyParticle;

struct b3SoftBodyTetrahedronShapeDef : public b3SoftBodyShapeDef
{
	b3SoftBodyParticle* p1;
	b3SoftBodyParticle* p2;
	b3SoftBodyParticle* p3;
	b3SoftBodyParticle* p4;

	b3Vec3 v1, v2, v3, v4;
};

// Tetrahedron shape
class b3SoftBodyTetrahedronShape : public b3SoftBodyShape
{
public:
	// Return the particle 1.
	b3SoftBodyParticle* GetParticle1() { return m_p1; }
	const b3SoftBodyParticle* GetParticle1() const { return m_p1; }

	// Return the particle 2.
	b3SoftBodyParticle* GetParticle2() { return m_p2; }
	const b3SoftBodyParticle* GetParticle2() const { return m_p2; }

	// Return the particle 3.
	b3SoftBodyParticle* GetParticle3() { return m_p3; }
	const b3SoftBodyParticle* GetParticle3() const { return m_p3; }

	// Return the particle 4.
	b3SoftBodyParticle* GetParticle4() { return m_p4; }
	const b3SoftBodyParticle* GetParticle4() const { return m_p4; }
	
	// Return the next tetrahedron shape in the body list of shapes.
	b3SoftBodyTetrahedronShape* GetNext() { return m_next; }
	const b3SoftBodyTetrahedronShape* GetNext() const { return m_next; }
private:
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3List<b3SoftBodyTetrahedronShape>;

	b3SoftBodyTetrahedronShape(const b3SoftBodyTetrahedronShapeDef& def, b3SoftBody* body);
	~b3SoftBodyTetrahedronShape();

	// Compute AABB
	b3AABB ComputeAABB() const;

	// Synchronize the AABB
	void Synchronize(const b3Vec3& displacement);

	// Touch proxy
	void TouchProxy();

	// Destroy shape contacts
	void DestroyContacts();

	// Particles
	b3SoftBodyParticle* m_p1;
	b3SoftBodyParticle* m_p2;
	b3SoftBodyParticle* m_p3;
	b3SoftBodyParticle* m_p4;

	// Rest volume. Used for computing the mass of the particles.
	scalar m_volume;

	// Links to the body tetrahedron shape list.
	b3SoftBodyTetrahedronShape* m_prev;
	b3SoftBodyTetrahedronShape* m_next;
};

#endif