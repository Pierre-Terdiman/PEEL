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

#ifndef B3_SOFTBODY_SPRING_FORCE_H
#define B3_SOFTBODY_SPRING_FORCE_H

#include <bounce/softbody/forces/softbody_force.h>

// Spring force definition.
// This requires defining two particles, the 
// natural spring rest length, and the spring parameters.
struct b3SoftBodySpringForceDef : public b3SoftBodyForceDef
{
	b3SoftBodySpringForceDef()
	{
		type = e_softBodySpringForce;
		restLength = scalar(0);
		stiffness = scalar(0);
		dampingStiffness = scalar(0);
	}

	// Initialize this definition from particles and stiffnesses.
	void Initialize(b3SoftBodyParticle* particle1, b3SoftBodyParticle* particle2, scalar structuralStiffness, scalar dampingStiffness);

	// Particle 1
	b3SoftBodyParticle* p1;

	// Particle 2
	b3SoftBodyParticle* p2;

	// Rest length
	scalar restLength;

	// Spring stiffness
	scalar stiffness;

	// Damping stiffness
	scalar dampingStiffness;
};

// A spring force acting on two particles to maintain them at a desired distance.
class b3SoftBodySpringForce : public b3SoftBodyForce
{
public:
	// Does this force contain a given particle?
	bool HasParticle(const b3SoftBodyParticle* particle) const;

	// Get the particle 1.
	const b3SoftBodyParticle* GetParticle1() const { return m_p1; }
	b3SoftBodyParticle* GetParticle1() { return m_p1; }

	// Get the particle 2.
	const b3SoftBodyParticle* GetParticle2() const { return m_p2; }
	b3SoftBodyParticle* GetParticle2() { return m_p2; }

	// Get the spring natural rest length.
	scalar GetRestLenght() const;

	// Get the spring stiffness.
	scalar GetStiffness() const;

	// Get the damping stiffness.
	scalar GetDampingStiffness() const;

	// Get the force acting on particle 1.
	b3Vec3 GetActionForce() const;

	// Get the force acting on particle 2.
	b3Vec3 GetReactionForce() const;
private:
	friend class b3SoftBodyForce;
	
	b3SoftBodySpringForce(const b3SoftBodySpringForceDef* def);
	~b3SoftBodySpringForce();

	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3SoftBodyParticle* m_p1;

	// Particle 2
	b3SoftBodyParticle* m_p2;

	// Spring natural rest length
	scalar m_L0;

	// Spring stiffness
	scalar m_ks;

	// Damping stiffness
	scalar m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2;
};

inline scalar b3SoftBodySpringForce::GetRestLenght() const
{
	return m_L0;
}

inline scalar b3SoftBodySpringForce::GetStiffness() const
{
	return m_ks;
}

inline scalar b3SoftBodySpringForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3SoftBodySpringForce::GetActionForce() const
{
	return m_f1;
}

inline b3Vec3 b3SoftBodySpringForce::GetReactionForce() const
{
	return m_f2;
}

#endif