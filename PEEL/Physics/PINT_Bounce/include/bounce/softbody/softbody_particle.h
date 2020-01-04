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

#ifndef B3_SOFTBODY_PARTICLE_H
#define B3_SOFTBODY_PARTICLE_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec3.h>

class b3SoftBody;

struct b3SparseForceSolverData;

// Static particle: Zero mass. Can be moved manually.
// Kinematic particle: Zero mass. Non-zero velocity, can be moved by the solver.
// Dynamic particle: Non-zero mass. Non-zero velocity determined by force, can be moved by the solver.
enum b3SoftBodyParticleType
{
	e_staticSoftBodyParticle,
	e_kinematicSoftBodyParticle,
	e_dynamicSoftBodyParticle
};

// Particle definition.
struct b3SoftBodyParticleDef
{
	b3SoftBodyParticleDef()
	{
		type = e_staticSoftBodyParticle;
		position.SetZero();
		velocity.SetZero();
		massDamping = scalar(0);
		meshIndex = B3_MAX_U32;
		userData = nullptr;
	}

	// The particle type.
	// Types: static, kinematic, and dynamic.
	b3SoftBodyParticleType type;

	// Initial position of the particle.
	b3Vec3 position;

	// Initial velocity of the particle.
	b3Vec3 velocity;

	// Coefficient of mass damping of the particle.
	scalar massDamping;

	// Optional vertex index in the mesh.
	u32 meshIndex;

	// User data pointer to anything.
	void* userData;
};

// A particle.
class b3SoftBodyParticle
{
public:
	// Set the particle type.
	void SetType(b3SoftBodyParticleType type);

	// Get the particle type.
	b3SoftBodyParticleType GetType() const;

	// Set the particle position. 
	// If the particle is dynamic changing the position directly might lead 
	// to physically incorrect simulation behaviour.
	void SetPosition(const b3Vec3& position);

	// Get the particle position.
	const b3Vec3& GetPosition() const;

	// Set the particle velocity.
	void SetVelocity(const b3Vec3& velocity);

	// Get the particle velocity.
	const b3Vec3& GetVelocity() const;

	// Get the particle mass.
	scalar GetMass() const;

	// Get the applied force.
	const b3Vec3& GetForce() const;

	// Apply a force.
	void ApplyForce(const b3Vec3& force);

	// Get the applied translation.
	const b3Vec3& GetTranslation() const;

	// Apply a translation.
	void ApplyTranslation(const b3Vec3& translation);

	// Set the mass coefficient of damping.
	void SetMassDamping(scalar massDamping);

	// Get the mass coefficient of damping.
	scalar GetMassDamping() const;

	// Get the mesh index.
	u32 GetMeshIndex() const;
	
	// Set the user data.
	void SetUserData(void* userData);

	// Get the user data.
	const void* GetUserData() const;
	void* GetUserData();

	// Get the next particle in the body list of particles.
	b3SoftBodyParticle* GetNext();
	const b3SoftBodyParticle* GetNext() const;
private:
	friend class b3List<b3SoftBodyParticle>;
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyForceSolver;
	friend class b3SoftBodyForceModel;
	friend class b3SoftBodySphereShape;
	friend class b3SoftBodyCapsuleShape;
	friend class b3SoftBodyTriangleShape;
	friend class b3SoftBodyTetrahedronShape;
	friend class b3SoftBodySphereAndShapeContact;
	friend class b3SoftBodySphereAndTriangleContact;
	friend class b3SoftBodyCapsuleAndCapsuleContact;
	friend class b3SoftBodyContactSolver;
	friend class b3SoftBodyForce;
	friend class b3SoftBodyStretchForce;
	friend class b3SoftBodyShearForce;
	friend class b3SoftBodySpringForce;
	friend class b3SoftBodyMouseForce;
	friend class b3SoftBodyTriangleElementForce;
	friend class b3SoftBodyTetrahedronElementForce;

	b3SoftBodyParticle(const b3SoftBodyParticleDef& def, b3SoftBody* body);
	~b3SoftBodyParticle();

	// Synchronize shapes
	void SynchronizeShapes();

	// Touch shape proxies
	void TouchProxies();

	// Destroy shapes.
	void DestroyShapes();

	// Destroy forces.
	void DestroyForces();

	// Destroy contacts.
	void DestroyContacts();
	
	// Compute forces due to particle.
	void ComputeForces(const b3SparseForceSolverData* data);

	// Type
	b3SoftBodyParticleType m_type;

	// Position
	b3Vec3 m_position;

	// Velocity
	b3Vec3 m_velocity;

	// Applied external force
	b3Vec3 m_force;

	// Applied translation
	b3Vec3 m_translation;

	// Mass
	scalar m_mass;

	// Inverse mass
	scalar m_invMass;

	// Coefficient of mass damping.
	scalar m_massDamping;

	// Mesh index. 
	u32 m_meshIndex;

	// Solver temp identifier
	u32 m_solverId;

	// User data
	void* m_userData;

	// Body
	b3SoftBody* m_body;

	// Links to the body particle list.
	b3SoftBodyParticle* m_prev;
	b3SoftBodyParticle* m_next;
};

inline b3SoftBodyParticleType b3SoftBodyParticle::GetType() const
{
	return m_type;
}

inline void b3SoftBodyParticle::SetPosition(const b3Vec3& position)
{
	m_position = position;
	m_translation.SetZero();
	SynchronizeShapes();
}

inline const b3Vec3& b3SoftBodyParticle::GetPosition() const
{
	return m_position;
}

inline void b3SoftBodyParticle::SetVelocity(const b3Vec3& velocity)
{
	if (m_type == e_staticSoftBodyParticle)
	{
		return;
	}
	m_velocity = velocity;
}

inline const b3Vec3& b3SoftBodyParticle::GetVelocity() const
{
	return m_velocity;
}

inline scalar b3SoftBodyParticle::GetMass() const
{
	return m_mass;
}

inline const b3Vec3& b3SoftBodyParticle::GetForce() const
{
	return m_force;
}

inline void b3SoftBodyParticle::ApplyForce(const b3Vec3& force)
{
	if (m_type != e_dynamicSoftBodyParticle)
	{
		return;
	}
	m_force += force;
}

inline const b3Vec3& b3SoftBodyParticle::GetTranslation() const
{
	return m_translation;
}

inline void b3SoftBodyParticle::ApplyTranslation(const b3Vec3& translation)
{
	m_translation += translation;
}

inline void b3SoftBodyParticle::SetMassDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_massDamping = damping;
}

inline scalar b3SoftBodyParticle::GetMassDamping() const
{
	return m_massDamping;
}

inline u32 b3SoftBodyParticle::GetMeshIndex() const
{
	return m_meshIndex;
}

inline void b3SoftBodyParticle::SetUserData(void* userData)
{
	m_userData = userData;
}

inline const void* b3SoftBodyParticle::GetUserData() const
{
	return m_userData;
}

inline void* b3SoftBodyParticle::GetUserData()
{
	return m_userData;
}

inline b3SoftBodyParticle* b3SoftBodyParticle::GetNext()
{
	return m_next;
}

inline const b3SoftBodyParticle* b3SoftBodyParticle::GetNext() const
{
	return m_next;
}

#endif