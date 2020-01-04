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

#ifndef B3_SOFTBODY_TETRAHEDRON_ELEMENT_FORCE_H
#define B3_SOFTBODY_TETRAHEDRON_ELEMENT_FORCE_H

#include <bounce/softbody/forces/softbody_force.h>
#include <bounce/common/math/quat.h>

class b3SoftBody;

struct b3SparseForceSolverData;

// Tetrahedral element force definition.
// This requires defining the element in the rest state and its material parameters.
struct b3SoftBodyTetrahedronElementForceDef : public b3SoftBodyForceDef
{
	b3SoftBodyTetrahedronElementForceDef()
	{
		type = e_softBodyTetrahedronElementForce;
		youngModulus = scalar(500);
		poissonRatio = scalar(0.3);
		stiffnessDamping = scalar(0);
		elasticStrainYield = B3_MAX_SCALAR;
		creepRate = scalar(0);
		maxPlasticStrain = scalar(0);
	}

	// Particle 1
	b3SoftBodyParticle* p1;

	// Particle 2
	b3SoftBodyParticle* p2;

	// Particle 3
	b3SoftBodyParticle* p3;

	// Particle 4
	b3SoftBodyParticle* p4;
	
	// Vertex 1 in rest state
	b3Vec3 v1;
	
	// Vertex 2 in rest state
	b3Vec3 v2;
	
	// Vertex 3 in rest state
	b3Vec3 v3;
	
	// Vertex 4 in rest state
	b3Vec3 v4;

	// Material Young modulus in [0, inf]
	// Units are 1e3N/m^2
	scalar youngModulus;

	// Material Poisson ratio in [0, 0.5]
	// This is a dimensionless value
	scalar poissonRatio;

	// Coefficient of stiffness damping.
	// You can tune this value if you see vibrations in your simulation.
	scalar stiffnessDamping;

	// Material elastic strain yield in [0, inf]
	// This is a dimensionless value.
	// Set to inf to disable plasticity.
	scalar elasticStrainYield;

	// Material creep rate in [0, 1 / dt] 
	// Units are Hz
	// This is usually set to the simulation frequency 
	scalar creepRate;

	// Material maximum plastic strain in [0, inf]
	// This is a dimensionless value
	scalar maxPlasticStrain;
};

// Soft body tetrahedron element
class b3SoftBodyTetrahedronElementForce : public b3SoftBodyForce
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

	// Get the particle 4.
	const b3SoftBodyParticle* GetParticle4() const { return m_p4; }
	b3SoftBodyParticle* GetParticle4() { return m_p4; }

	// Set the material Young Modulus of elasticity in [0, inf].
	void SetYoungModulus(scalar E);
	
	// Get the material Young Modulus of elasticity in [0, inf].
	scalar GetYoungModulus() const;

	// Set the Poisson's ratio in [0, 0.5].
	void SetPoissonRatio(scalar nu);

	// Get the Poisson's ratio in [0, 0.5].
	scalar GetPoissonRatio() const;
	
	// Set the coefficient of stiffness damping.
	void SetStiffnessDamping(scalar damping);

	// Get the coefficient of stiffness damping.
	scalar GetStiffnessDamping() const;

	// Set the elastic strain yield in range [0, inf].
	// Set this value to inf to disable plasticity.
	void SetElasticStrainYield(scalar yield);

	// Get the elastic strain yield in range [0, inf].
	scalar GetElasticStrainYield() const;

	// Set the material creep rate in hertz.
	// Plasticity must be enabled for this value to take effect.
	void SetCreepRate(scalar hz);

	// Get the material creep rate in hertz.
	scalar GetCreepRate() const;

	// Set the material maximum plastic strain in the range [0, inf].
	// Plasticity must be enabled for this value to take effect.
	void SetMaxPlasticStrain(scalar max);

	// Get the material maximum plastic strain in the range [0, inf].
	scalar GetMaxPlasticStrain() const;
private:
	friend class b3SoftBodyForce;
	
	b3SoftBodyTetrahedronElementForce(const b3SoftBodyTetrahedronElementForceDef* def);
	~b3SoftBodyTetrahedronElementForce();

	// This resets the finite element data.
	void ResetElementData();

	// Compute element forces.
	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3SoftBodyParticle* m_p1;
	
	// Particle 2
	b3SoftBodyParticle* m_p2;
	
	// Particle 3
	b3SoftBodyParticle* m_p3;
	
	// Particle 4
	b3SoftBodyParticle* m_p4;

	// Reference tetrahedron
	b3Vec3 m_x1, m_x2, m_x3, m_x4;

	// Reference volume
	scalar m_V;

	// Elasticity
	scalar m_E;
	scalar m_nu;

	// Stiffness damping
	scalar m_stiffnessDamping;

	// Plasticity
	scalar m_c_yield;
	scalar m_c_creep;
	scalar m_c_max;
	scalar m_epsilon_plastic[6]; // 6 x 1

	// Solver shared
	b3Mat33 m_invE; // 3 x 3
	b3Quat m_q; // 3 x 3
	b3Mat33* m_Kp[16]; // 12 x 12
	b3Mat33 m_K[16]; // 12 x 12
	scalar m_B[72]; // 6 x 12
	scalar m_P[72]; // V * BT * E -> 12 x 6
};

inline void b3SoftBodyTetrahedronElementForce::SetYoungModulus(scalar E)
{
	B3_ASSERT(E > scalar(0));
	if (E != m_E)
	{
		m_E = E;
		ResetElementData();
	}
}

inline scalar b3SoftBodyTetrahedronElementForce::GetYoungModulus() const
{
	return m_E;
}

inline void b3SoftBodyTetrahedronElementForce::SetPoissonRatio(scalar nu)
{
	B3_ASSERT(nu >= scalar(0) && nu <= scalar(0.5));
	if (nu != m_nu)
	{
		m_nu = nu;
		ResetElementData();
	}
}

inline scalar b3SoftBodyTetrahedronElementForce::GetPoissonRatio() const
{
	return m_nu;
}

inline void b3SoftBodyTetrahedronElementForce::SetStiffnessDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_stiffnessDamping = damping;
}

inline scalar b3SoftBodyTetrahedronElementForce::GetStiffnessDamping() const
{
	return m_stiffnessDamping;
}

inline void b3SoftBodyTetrahedronElementForce::SetElasticStrainYield(scalar yield)
{
	B3_ASSERT(yield >= scalar(0));
	m_c_yield = yield;
}

inline scalar b3SoftBodyTetrahedronElementForce::GetElasticStrainYield() const
{
	return m_c_yield;
}

inline void b3SoftBodyTetrahedronElementForce::SetCreepRate(scalar hz)
{
	B3_ASSERT(hz >= scalar(0));
	m_c_creep = hz;
}

inline scalar b3SoftBodyTetrahedronElementForce::GetCreepRate() const
{
	return m_c_creep;
}

inline void b3SoftBodyTetrahedronElementForce::SetMaxPlasticStrain(scalar max)
{
	B3_ASSERT(max >= scalar(0));
	m_c_max = max;
}

inline scalar b3SoftBodyTetrahedronElementForce::GetMaxPlasticStrain() const
{
	return m_c_max;
}

#endif