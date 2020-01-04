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

#ifndef B3_SOFTBODY_SPHERE_AND_TRIANGLE_CONTACT_H
#define B3_SOFTBODY_SPHERE_AND_TRIANGLE_CONTACT_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>

class b3SoftBodySphereShape;
class b3SoftBodyTriangleShape;

// Contact between sphere and a triangle
class b3SoftBodySphereAndTriangleContact
{
public:
private:
	friend class b3List<b3SoftBodySphereAndTriangleContact>;
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3SoftBodySphereShape;
	friend class b3SoftBodyTriangleShape;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodyContactSolver;

	b3SoftBodySphereAndTriangleContact() { }
	~b3SoftBodySphereAndTriangleContact() { }

	void Update();

	b3SoftBodySphereShape* m_s1;
	b3SoftBodyTriangleShape* m_s2;

	bool m_active;

	scalar m_w1, m_w2, m_w3;
	
	b3Vec3 m_normal1;
	scalar m_normalImpulse;
	
	b3Vec3 m_tangent1;
	b3Vec3 m_tangent2;
	b3Vec2 m_tangentImpulse;

	b3SoftBodySphereAndTriangleContact* m_prev;
	b3SoftBodySphereAndTriangleContact* m_next;
};

#endif