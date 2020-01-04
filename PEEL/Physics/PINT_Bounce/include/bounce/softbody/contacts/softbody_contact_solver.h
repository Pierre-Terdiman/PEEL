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

#ifndef B3_SOFTBODY_CONTACT_SOLVER_H
#define B3_SOFTBODY_CONTACT_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/softbody/softbody_time_step.h>

class b3StackAllocator;

class b3SoftBodyParticle;

class b3SoftBodySphereAndShapeContact;
class b3SoftBodySphereAndTriangleContact;
class b3SoftBodyCapsuleAndCapsuleContact;

struct b3SoftBodySolverShapeContactVelocityConstraint
{
	u32 indexA;
	scalar invMassA;

	scalar friction;

	b3Vec3 normal;
	scalar normalMass;
	scalar normalImpulse;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	scalar tangentMass;
	b3Vec2 tangentImpulse;
};

struct b3SoftBodySolverShapeContactPositionConstraint
{
	u32 indexA;
	scalar invMassA;
	scalar radiusA;

	scalar radiusB;
	
	b3Vec3 normal;
	b3Vec3 pointB;
};

struct b3SoftBodySolverTriangleContactVelocityConstraint
{
	u32 indexA;
	scalar invMassA;

	u32 indexB;
	scalar invMassB;
	u32 indexC;
	scalar invMassC;
	u32 indexD;
	scalar invMassD;
	
	scalar wB, wC, wD;

	b3Vec3 normal;
	scalar normalMass;
	scalar normalImpulse;

	scalar friction;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;
};

struct b3SoftBodySolverTriangleContactPositionConstraint
{
	u32 indexA;
	scalar invMassA;
	scalar radiusA;

	u32 indexB;
	scalar invMassB;
	u32 indexC;
	scalar invMassC;
	u32 indexD;
	scalar invMassD;
	scalar triangleRadius;

	scalar wB, wC, wD;
};

struct b3SoftBodySolverCapsuleContactVelocityConstraint
{
	u32 indexA;
	scalar invMassA;
	u32 indexB;
	scalar invMassB;

	u32 indexC;
	scalar invMassC;
	u32 indexD;
	scalar invMassD;

	scalar wA, wB, wC, wD;

	b3Vec3 normal;
	scalar normalMass;
	scalar normalImpulse;

	scalar friction;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;
};

struct b3SoftBodySolverCapsuleContactPositionConstraint
{
	u32 indexA;
	scalar invMassA;
	u32 indexB;
	scalar invMassB;
	scalar radiusA;

	u32 indexC;
	scalar invMassC;
	u32 indexD;
	scalar invMassD;
	scalar radiusB;

	scalar wA, wB, wC, wD;
};

struct b3SoftBodyContactSolverDef
{
	b3SoftBodyTimeStep step;
	b3StackAllocator* allocator;
	
	b3Vec3* positions;
	b3Vec3* velocities;
	
	u32 shapeContactCount;
	b3SoftBodySphereAndShapeContact** shapeContacts;

	u32 triangleContactCount;
	b3SoftBodySphereAndTriangleContact** triangleContacts;
	
	u32 capsuleContactCount;
	b3SoftBodyCapsuleAndCapsuleContact** capsuleContacts;
};

inline scalar b3MixFriction(scalar u1, scalar u2)
{
	return b3Sqrt(u1 * u2);
}

class b3SoftBodyContactSolver
{
public:
	b3SoftBodyContactSolver(const b3SoftBodyContactSolverDef& def);
	~b3SoftBodyContactSolver();

	void InitializeShapeContactConstraints();
	void InitializeTriangleContactConstraints();
	void InitializeCapsuleContactConstraints();

	void WarmStartShapeContactConstraints();
	void WarmStartTriangleContactConstraints();
	void WarmStartCapsuleContactConstraints();

	void SolveShapeContactVelocityConstraints();
	void SolveTriangleContactVelocityConstraints();
	void SolveCapsuleContactVelocityConstraints();

	void StoreImpulses();

	bool SolveShapeContactPositionConstraints();
	bool SolveTriangleContactPositionConstraints();
	bool SolveCapsuleContactPositionConstraints();
protected:
	b3SoftBodyTimeStep m_step;

	b3StackAllocator* m_allocator;

	b3Vec3* m_positions;
	b3Vec3* m_velocities;

	u32 m_shapeContactCount;
	b3SoftBodySphereAndShapeContact** m_shapeContacts;
	b3SoftBodySolverShapeContactVelocityConstraint* m_shapeVelocityConstraints;
	b3SoftBodySolverShapeContactPositionConstraint* m_shapePositionConstraints;

	u32 m_triangleContactCount;
	b3SoftBodySphereAndTriangleContact** m_triangleContacts;
	b3SoftBodySolverTriangleContactVelocityConstraint* m_triangleVelocityConstraints;
	b3SoftBodySolverTriangleContactPositionConstraint* m_trianglePositionConstraints;
	
	u32 m_capsuleContactCount;
	b3SoftBodyCapsuleAndCapsuleContact** m_capsuleContacts;
	b3SoftBodySolverCapsuleContactVelocityConstraint* m_capsuleVelocityConstraints;
	b3SoftBodySolverCapsuleContactPositionConstraint* m_capsulePositionConstraints;
};

#endif