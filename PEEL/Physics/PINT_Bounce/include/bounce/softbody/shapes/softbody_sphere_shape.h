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

#ifndef B3_SOFTBODY_SPHERE_SHAPE_H
#define B3_SOFTBODY_SPHERE_SHAPE_H

#include <bounce/softbody/shapes/softbody_shape.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3SoftBodyParticle;

struct b3SoftBodySphereShapeDef : public b3SoftBodyShapeDef
{
	b3SoftBodyParticle* p;
};

// Sphere shape.
class b3SoftBodySphereShape : public b3SoftBodyShape
{
public:
	// Return the particle.
	b3SoftBodyParticle* GetParticle() { return m_p; }
	const b3SoftBodyParticle* GetParticle() const { return m_p; }

	// Return the next sphere in the body.
	b3SoftBodySphereShape* GetNext() { return m_next; };
	const b3SoftBodySphereShape* GetNext() const { return m_next; };
private:
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySphereAndTriangleContact;
	friend class b3SoftBodySphereAndShapeContact;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyContactSolver;
	friend class b3List<b3SoftBodySphereShape>;

	b3SoftBodySphereShape(const b3SoftBodySphereShapeDef& def, b3SoftBody* body);
	~b3SoftBodySphereShape();

	// Compute AABB
	b3AABB ComputeAABB() const;

	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Touch AABB proxy
	void TouchProxy();

	// Destroy contacts
	void DestroyContacts();

	// Particle
	b3SoftBodyParticle* m_p;

	// Links to the body sphere shape list.
	b3SoftBodySphereShape* m_prev;
	b3SoftBodySphereShape* m_next;
};

#endif