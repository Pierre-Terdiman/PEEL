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

#ifndef B3_SOFTBODY_H
#define B3_SOFTBODY_H

#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/list.h>
#include <bounce/softbody/softbody_contact_manager.h>

struct b3SoftBodyParticleDef;
class b3SoftBodyParticle;

struct b3SoftBodyForceDef;
class b3SoftBodyForce;

struct b3SoftBodySphereShapeDef;
class b3SoftBodySphereShape;

struct b3SoftBodyCapsuleShapeDef;
class b3SoftBodyCapsuleShape;

struct b3SoftBodyTriangleShapeDef;
class b3SoftBodyTriangleShape;

struct b3SoftBodyTetrahedronShapeDef;
class b3SoftBodyTetrahedronShape;

struct b3SoftBodyWorldShapeDef;
class b3SoftBodyWorldShape;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3SoftBodyTimeStep;

struct b3SoftBodyRayCastSingleOutput
{
	b3SoftBodyTriangleShape* triangle;
	scalar fraction;
	b3Vec3 normal;
};

// A soft body represents a deformable object as a collection of particles.
// Particles may be connected with each other by forces.
class b3SoftBody
{
public:
	b3SoftBody();
	virtual ~b3SoftBody();

	// Create a particle.
	b3SoftBodyParticle* CreateParticle(const b3SoftBodyParticleDef& def);

	// Destroy a given particle.
	void DestroyParticle(b3SoftBodyParticle* particle);

	// Return the list of particles in this body.
	const b3List<b3SoftBodyParticle>& GetParticleList() const;

	// Create a force.
	b3SoftBodyForce* CreateForce(const b3SoftBodyForceDef& def);

	// Destroy a given force.
	void DestroyForce(b3SoftBodyForce* force);

	// Return the list of forces in this body.
	const b3List<b3SoftBodyForce>& GetForceList() const;

	// Create a sphere shape.
	b3SoftBodySphereShape* CreateSphereShape(const b3SoftBodySphereShapeDef& def);

	// Destroy a given sphere shape.
	void DestroySphereShape(b3SoftBodySphereShape* shape);
	
	// Return the list of sphere shapes in this body.
	const b3List<b3SoftBodySphereShape>& GetSphereShapeList() const;
	
	// Create a capsule shape.
	b3SoftBodyCapsuleShape* CreateCapsuleShape(const b3SoftBodyCapsuleShapeDef& def);

	// Destroy a given capsule shape.
	void DestroyCapsuleShape(b3SoftBodyCapsuleShape* shape);

	// Return the list of capsule shapes in this body.
	const b3List<b3SoftBodyCapsuleShape>& GetCapsuleShapeList() const;

	// Create a triangle shape.
	b3SoftBodyTriangleShape* CreateTriangleShape(const b3SoftBodyTriangleShapeDef& def);

	// Destroy a given triangle shape.
	void DestroyTriangleShape(b3SoftBodyTriangleShape* shape);

	// Return the list of triangle shapes in this body.
	const b3List<b3SoftBodyTriangleShape>& GetTriangleShapeList() const;

	// Create a tetrahedron shape.
	b3SoftBodyTetrahedronShape* CreateTetrahedronShape(const b3SoftBodyTetrahedronShapeDef& def);

	// Destroy a given tetrahedron shape.
	void DestroyTetrahedronShape(b3SoftBodyTetrahedronShape* shape);

	// Return the list of tetrahedron shapes in this body.
	const b3List<b3SoftBodyTetrahedronShape>& GetTetrahedronShapeList() const;

	// Create a new world shape.
	b3SoftBodyWorldShape* CreateWorldShape(const b3SoftBodyWorldShapeDef& def);

	// Destroy a given world shape.
	void DestroyWorldShape(b3SoftBodyWorldShape* shape);

	// Return the list of world shapes in this body.
	const b3List<b3SoftBodyWorldShape>& GetWorldShapeList() const;

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Perform a time step. 
	void Step(scalar dt, u32 velocityIterations, u32 positionIterations, u32 forceIterations, u32 forceSubIterations);

	// Perform a ray cast with the body.
	bool RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Return the kinetic energy in this system.
	scalar GetEnergy() const;

	// Debug draw the body entities.
	void Draw() const;
protected:
	friend class b3SoftBodyParticle;
	friend class b3SoftBodySphereShape;
	friend class b3SoftBodyCapsuleShape;
	friend class b3SoftBodyTriangleShape;
	friend class b3SoftBodyTetrahedronShape;
	friend class b3SoftBodyWorldShape;
	friend class b3SoftBodySpringForce;
	friend class b3SoftBodyStretchForce;
	friend class b3SoftBodyShearForce;
	friend class b3SoftBodyMouseForce;
	friend class b3SoftBodyContactManager;
	
	// Rest the mass data of the body.
	void ResetMass();

	// Solve
	void Solve(const b3SoftBodyTimeStep& step);

	// Enable or disable self-collision.
	void EnableSelfCollision(bool flag);

	// Is self-collision enabled?
	bool IsSelfCollisionEnabled() const;

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Pool of particles
	b3BlockPool m_particleBlocks;

	// List of particles
	b3List<b3SoftBodyParticle> m_particleList;

	// List of forces
	b3List<b3SoftBodyForce> m_forceList;

	// List of sphere shapes
	b3List<b3SoftBodySphereShape> m_sphereShapeList;
	
	// List of capsule shapes
	b3List<b3SoftBodyCapsuleShape> m_capsuleShapeList;
	
	// List of triangle shapes
	b3List<b3SoftBodyTriangleShape> m_triangleShapeList;
	
	// List of tetrahedron shapes
	b3List<b3SoftBodyTetrahedronShape> m_tetrahedronShapeList;
	
	// List of world shapes
	b3List<b3SoftBodyWorldShape> m_worldShapeList;

	// Contact manager
	b3SoftBodyContactManager m_contactManager;

	// Used to compute the time step ratio to 
	// support variable time steps.
	scalar m_inv_dt0;

	// Self-collision activation flag
	bool m_enableSelfCollision;
};

inline void b3SoftBody::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3SoftBody::GetGravity() const
{
	return m_gravity;
}

inline bool b3SoftBody::IsSelfCollisionEnabled() const
{
	return m_enableSelfCollision;
}

inline const b3List<b3SoftBodyForce>& b3SoftBody::GetForceList() const
{
	return m_forceList;
}

inline const b3List<b3SoftBodyParticle>& b3SoftBody::GetParticleList() const
{
	return m_particleList;
}

inline const b3List<b3SoftBodySphereShape>& b3SoftBody::GetSphereShapeList() const
{
	return m_sphereShapeList;
}

inline const b3List<b3SoftBodyCapsuleShape>& b3SoftBody::GetCapsuleShapeList() const
{
	return m_capsuleShapeList;
}

inline const b3List<b3SoftBodyTriangleShape>& b3SoftBody::GetTriangleShapeList() const
{
	return m_triangleShapeList;
}

inline const b3List<b3SoftBodyTetrahedronShape>& b3SoftBody::GetTetrahedronShapeList() const
{
	return m_tetrahedronShapeList;
}

inline const b3List<b3SoftBodyWorldShape>& b3SoftBody::GetWorldShapeList() const
{
	return m_worldShapeList;
}

#endif