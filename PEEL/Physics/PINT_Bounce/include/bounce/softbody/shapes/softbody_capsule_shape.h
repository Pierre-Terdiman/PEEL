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

#ifndef B3_SOFTBODY_CAPSULE_SHAPE_H
#define B3_SOFTBODY_CAPSULE_SHAPE_H

#include <bounce/softbody/shapes/softbody_shape.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3SoftBodyParticle;

struct b3SoftBodyCapsuleShapeDef : b3SoftBodyShapeDef
{
	b3SoftBodyParticle* p1;
	b3SoftBodyParticle* p2;
};

// Capsule shape.
class b3SoftBodyCapsuleShape : public b3SoftBodyShape
{
public:
	// Return the particle 1.
	b3SoftBodyParticle* GetParticle1() { return m_p1; }
	const b3SoftBodyParticle* GetParticle1() const { return m_p1; }

	// Return the particle 2.
	b3SoftBodyParticle* GetParticle2() { return m_p2; }
	const b3SoftBodyParticle* GetParticle2() const { return m_p2; }

	// Return the next capsule in the body capsule list.
	b3SoftBodyCapsuleShape* GetNext() { return m_next; }
	const b3SoftBodyCapsuleShape* GetNext() const { return m_next; }
private:
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3SoftBodyShearForce;
	friend class b3SoftBodyStretchForce;
	friend class b3SoftBodyMouseForce;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySphereAndTriangleContact;
	friend class b3SoftBodyCapsuleAndCapsuleContact;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyContactSolver;
	friend class b3List<b3SoftBodyCapsuleShape>;

	b3SoftBodyCapsuleShape(const b3SoftBodyCapsuleShapeDef& def, b3SoftBody* body);
	~b3SoftBodyCapsuleShape();

	// Compute AABB
	b3AABB ComputeAABB() const;
	
	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Touch proxy
	void TouchProxy();

	// Destroy contacts
	void DestroyContacts();

	// Particles
	b3SoftBodyParticle* m_p1;
	b3SoftBodyParticle* m_p2;

	// Links to the body capsule shape list.
	b3SoftBodyCapsuleShape* m_prev;
	b3SoftBodyCapsuleShape* m_next;
};

#endif