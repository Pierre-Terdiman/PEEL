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

#ifndef B3_SOFTBODY_FORCE_H
#define B3_SOFTBODY_FORCE_H

#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>

class b3SoftBodyParticle;

struct b3SparseForceSolverData;

// Force types
enum b3SoftBodyForceType
{
	e_softBodyStretchForce,
	e_softBodyShearForce,
	e_softBodySpringForce,
	e_softBodyMouseForce,
	e_softBodyTriangleElementForce,
	e_softBodyTetrahedronElementForce,
};

struct b3SoftBodyForceDef
{
	b3SoftBodyForceDef()
	{
		meshIndex = B3_MAX_U32;
	}
	
	b3SoftBodyForceType type;
	u32 meshIndex;
};

// A force acts on a set of particles.
class b3SoftBodyForce
{
public:
	// Get the force type.
	b3SoftBodyForceType GetType() const;

	// Has this force a given particle?
	virtual bool HasParticle(const b3SoftBodyParticle* particle) const = 0;

	// Get the next force in the body force list.
	const b3SoftBodyForce* GetNext() const;
	b3SoftBodyForce* GetNext();

	// Get the mesh feature index.
	u32 GetMeshIndex() const { return m_meshIndex; }
protected:
	friend class b3List<b3SoftBodyForce>;
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3SoftBodyForceSolver;
	friend class b3SoftBodyForceModel;

	// Factory create and destroy.
	static b3SoftBodyForce* Create(const b3SoftBodyForceDef* def);
	static void Destroy(b3SoftBodyForce* f);

	b3SoftBodyForce() { }
	virtual ~b3SoftBodyForce() { }

	virtual void ComputeForces(const b3SparseForceSolverData* data) = 0;

	b3SoftBodyForceType m_type;
	u32 m_meshIndex;

	b3SoftBodyForce* m_prev;
	b3SoftBodyForce* m_next;
};

inline b3SoftBodyForceType b3SoftBodyForce::GetType() const
{
	return m_type;
}

inline const b3SoftBodyForce* b3SoftBodyForce::GetNext() const
{
	return m_next;
}

inline b3SoftBodyForce* b3SoftBodyForce::GetNext()
{
	return m_next;
}

#endif