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

#ifndef B3_SOFTBODY_SHEAR_FORCE_H
#define B3_SOFTBODY_SHEAR_FORCE_H

#include <bounce/softbody/forces/softbody_force.h>

// Shear force definition.
// This requires defining the (u, v) coordinates 
// of the triangle and some parameters.
struct b3SoftBodyShearForceDef : public b3SoftBodyForceDef
{
	b3SoftBodyShearForceDef()
	{
		type = e_softBodyShearForce;
		u1 = scalar(1);
		v1 = scalar(0);
		u2 = scalar(-1);
		v2 = scalar(0);
		u3 = scalar(0);
		v3 = scalar(0);
		shearingStiffness = scalar(0);
		dampingStiffness = scalar(0);
	}
	
	// Initialize this definition from rest positions
	void Initialize(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3);
	
	// Particle 1
	b3SoftBodyParticle* p1;

	// Particle 2
	b3SoftBodyParticle* p2;

	// Particle 3
	b3SoftBodyParticle* p3;

	// (u, v) coordinates for vertex 1 in the rest state
	scalar u1, v1;
	
	// (u, v) coordinates for vertex 2 in the rest state
	scalar u2, v2;
	
	// (u, v) coordinates for vertex 3 in the rest state
	scalar u3, v3;

	// Shearing stiffness
	scalar shearingStiffness;

	// Damping stiffness
	scalar dampingStiffness;
};

// Shear force acting on a triangle.
class b3SoftBodyShearForce : public b3SoftBodyForce
{
public:
	// Has this force a given particle?
	bool HasParticle(const b3SoftBodyParticle* particle) const;

	// Get the particle 1.
	const b3SoftBodyParticle* GetParticle1() const { return m_p1; }
	b3SoftBodyParticle* GetParticle1() { return m_p1; }

	// Get the particle 2.
	const b3SoftBodyParticle* GetParticle2() const { return m_p2; }
	b3SoftBodyParticle* GetParticle2() { return m_p2; }
	
	// Get the particle 3.
	const b3SoftBodyParticle* GetParticle3() const { return m_p3; }
	b3SoftBodyParticle* GetParticle3() { return m_p3; }

	// Get the shearing stiffness.
	scalar GetShearingStiffness() const;

	// Get the damping stiffness.
	scalar GetDampingStiffness() const;

	// Get the force acting on particle 1.
	b3Vec3 GetActionForce1() const;

	// Get the force acting on particle 2.
	b3Vec3 GetActionForce2() const;

	// Get the force acting on particle 3.
	b3Vec3 GetActionForce3() const;
private:
	friend class b3SoftBodyForce;

	b3SoftBodyShearForce(const b3SoftBodyShearForceDef* def);
	~b3SoftBodyShearForce();

	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3SoftBodyParticle* m_p1;

	// Particle 2
	b3SoftBodyParticle* m_p2;

	// Particle 3
	b3SoftBodyParticle* m_p3;
	
	// Area
	scalar m_alpha;

	// inverse duv matrix
	scalar m_du1, m_dv1;
	scalar m_du2, m_dv2;
	scalar m_inv_det;

	// dwudx, dwvdx
	b3Vec3 m_dwudx, m_dwvdx;

	// Shearing stiffness
	scalar m_ks;

	// Damping stiffness
	scalar m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3;
};

inline scalar b3SoftBodyShearForce::GetShearingStiffness() const
{
	return m_ks;
}

inline scalar b3SoftBodyShearForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3SoftBodyShearForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3SoftBodyShearForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3SoftBodyShearForce::GetActionForce3() const
{
	return m_f3;
}

#endif