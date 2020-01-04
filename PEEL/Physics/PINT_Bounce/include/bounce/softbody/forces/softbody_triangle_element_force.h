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

#ifndef B3_SOFTBODY_TRIANGLE_ELEMENT_FORCE_H
#define B3_SOFTBODY_TRIANGLE_ELEMENT_FORCE_H

#include <bounce/softbody/forces/softbody_force.h>
#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

// Triangular element force definition.
// This requires defining the element in the rest state and its material parameters.
struct b3SoftBodyTriangleElementForceDef : public b3SoftBodyForceDef
{
	b3SoftBodyTriangleElementForceDef()
	{
		type = e_softBodyTriangleElementForce;
		youngModulusX = scalar(100);
		youngModulusY = scalar(100);
		shearModulus = scalar(50);
		poissonRationXY = scalar(0.2);
		poissonRationYX = scalar(0.2);
		stiffnessDamping = scalar(0);
	}

	// Particle 1
	b3SoftBodyParticle* p1;

	// Particle 2
	b3SoftBodyParticle* p2;

	// Particle 3
	b3SoftBodyParticle* p3;

	// Vertex 1 in rest state
	b3Vec3 v1;
	
	// Vertex 2 in rest state
	b3Vec3 v2;
	
	// Vertex 3 in rest state
	b3Vec3 v3;

	// Young Modulus in x direction (weft)
	scalar youngModulusX;

	// Young Modulus in y direction (warp)
	scalar youngModulusY;

	// Shear Modulus 
	scalar shearModulus;

	// x, y Poisson's Ratio 
	scalar poissonRationXY;

	// y, x Poisson's Ratio
	scalar poissonRationYX;

	// Coefficient of stiffness damping.
	// You can tune this value if you see vibrations in your simulation.
	scalar stiffnessDamping;
};

// Element force acting on a triangle.
class b3SoftBodyTriangleElementForce : public b3SoftBodyForce
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

	// Set the coefficient of stiffness damping for this element.
	void SetStiffnessDamping(scalar damping);

	// Get the coefficient of stiffness damping for this element.
	scalar GetStiffnessDamping() const;
private:
	friend class b3SoftBodyForce;

	b3SoftBodyTriangleElementForce(const b3SoftBodyTriangleElementForceDef* def);
	~b3SoftBodyTriangleElementForce();

	// Reset element data.
	void ResetElementData();

	// Compute element forces.
	void ComputeForces(const b3SparseForceSolverData* data);

	// Particle 1
	b3SoftBodyParticle* m_p1;

	// Particle 2
	b3SoftBodyParticle* m_p2;

	// Particle 3
	b3SoftBodyParticle* m_p3;

	// Rest triangle vertices in 3D
	b3Vec3 m_v1, m_v2, m_v3;
	
	// Rest triangle vertices in 2D
	b3Vec2 m_x1, m_x2, m_x3;

	// Rest triangle area in 2D
	scalar m_A;

	// Initial inverse deformation in 2D
	b3Mat22 m_invS;

	// Young Modulus in 2D
	scalar m_E_x, m_E_y, m_E_s;

	// Poisson Ratio in 2D
	scalar m_nu_xy, m_nu_yx;

	// Elasticity tensor
	// This is a 3x3 matrix
	b3Mat33 m_C;

	// Shape functions derivative matrix
	// This is a 3x6 matrix
	scalar m_B[18];

	// Blocked stiffness matrix
	// This is a 6x6 matrix
	b3Mat22 m_K[9];

	// Coefficient of stiffness damping.
	scalar m_stiffnessDamping;
};

inline void b3SoftBodyTriangleElementForce::SetStiffnessDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_stiffnessDamping = damping;
}

inline scalar b3SoftBodyTriangleElementForce::GetStiffnessDamping() const
{
	return m_stiffnessDamping;
}

#endif