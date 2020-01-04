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

#ifndef B3_SHAPE_H
#define B3_SHAPE_H

#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>
#include <bounce/collision/collision.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/collision/shapes/sphere.h>

struct b3ContactEdge;

class b3Body;
class b3Shape;

// Shape types. 
// Types currently supported are spheres, capsules, 
// triangles, convex hulls, and triangle meshes.
enum b3ShapeType
{
	e_sphereShape,
	e_capsuleShape,
	e_triangleShape,
	e_hullShape,
	e_meshShape,
	e_maxShapes
};

// Shape definition. This requires providing a shape pointer that will 
// be cloned and some physics-related parameters. 
// As the shape will be cloned internally you can create shapes in the stack.
struct b3ShapeDef 
{
	// Default constructor fills in some default parameters.
	b3ShapeDef() 
	{
		shape = nullptr;
		userData = nullptr;
		isSensor = false;
		density = scalar(0);
		friction = scalar(0.3);
		restitution = scalar(0);
	}

	// The shape pointer. The shape will be cloned.
	// Therefore, you can create shapes in the stack and they can be temporary.
	const b3Shape* shape;
	
	// User data. It typically stores a pointer to a model mesh or a game entity.
	void* userData;
	
	// Is the shape a sensor? 
	// Sensor shapes aren't able to respond to collisions.
	// However, sensor shapes are reported to the contact callbacks.
	bool isSensor;
	
	// Density in kg/m^3 (kilogram per cubic metre).
	scalar density;
	
	// Coefficient of restitution in the range [0, 1].
	scalar restitution;
	
	// Coefficient of friction in the range [0, 1].
	scalar friction;
};

// This structure stores the mass-related data of a shape.
struct b3MassData 
{
	// The position of the shape's centroid relative to the shape's origin. 
	b3Vec3 center;

	// The mass of the shape, typically in kg.
	scalar mass;

	// The rotational inertia of the shape about the local origin.
	b3Mat33 I;
};

// This class extends a collision shape by containing simulation-related parameters 
// such as density, coefficients of friction and restitution.
class b3Shape
{
public:
	// Default constructor.
	b3Shape();
	
	// Default destructor does nothing.
	virtual ~b3Shape() { }

	// Get the shape type.
	b3ShapeType GetType() const;

	// Get the parent body of this shape.
	const b3Body* GetBody() const;
	b3Body* GetBody();

	// Calculate the mass data for this shape given the shape density.
	virtual void ComputeMass(b3MassData* data, scalar density) const = 0;

	// Compute the shape world AABB.
	virtual void ComputeAABB(b3AABB* aabb, const b3Transform& xf) const = 0;

	// Test if a sphere is contained inside this shape.
	virtual bool TestSphere(const b3Sphere& sphere, const b3Transform& xf) const = 0;

	// Compute the ray intersection point, normal of surface, and fraction.
	virtual bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const = 0;
	
	// Set if this shape is a sensor.
	void SetSensor(bool bit);

	// Is this shape a sensor?
	bool IsSensor() const;

	// Get the shape density.
	scalar GetDensity() const;

	// Set the shape density.
	void SetDensity(scalar density);

	// Get the shape coefficient of restitution.
	scalar GetRestitution() const;

	// Set the shape coefficient of restitution.
	// This is a value in the range [0, 1].
	void SetRestitution(scalar restitution);

	// Get the shape coefficient of friction.
	scalar GetFriction() const;
	
	// Set the shape coefficient of friction.
	// This is a value in the range [0, 1].
	void SetFriction(scalar friction);

	// Get the user data associated with this shape.
	void* GetUserData() const;

	// Set the user data associated with this shape.
	void SetUserData(void* data);

	// Get broadphase AABB
	const b3AABB& GetAABB() const;

	// Get the list of contacts that contains this body.
	const b3List<b3ContactEdge>& GetContactList() const;
	b3List<b3ContactEdge>& GetContactList();

	// Dump this shape to the log file.
	void Dump(u32 bodyIndex) const;

	// Get the next shape in the body shape list.
	const b3Shape* GetNext() const;
	b3Shape* GetNext();

	// This function is used internally.
	void SetShape(b3Shape* shape);

	scalar m_radius;
protected:
	friend class b3World;
	friend class b3Body;
	friend class b3Contact;
	friend class b3ContactManager;
	friend class b3MeshShape;
	friend class b3MeshContact;
	friend class b3ContactSolver;
	friend class b3List<b3Shape>;

	static b3Shape* Create(const b3ShapeDef& def);
	static void Destroy(b3Shape* shape);

	// Convenience function.
	// Destroy the contacts associated with this shape.
	void DestroyContacts();
	
	b3ShapeType m_type;
	bool m_isSensor;
	void* m_userData;
	scalar m_density;
	scalar m_restitution;
	scalar m_friction;
	u32 m_broadPhaseID;

	// Contact edges for this shape contact graph.
	b3List<b3ContactEdge> m_contactEdges;
	
	// The parent body of this shape.
	b3Body* m_body;
	
	// Links to the body shape list.
	b3Shape* m_prev;
	b3Shape* m_next;
};

inline b3ShapeType b3Shape::GetType() const 
{ 
	return m_type; 
}

inline scalar b3Shape::GetDensity() const 
{ 
	return m_density; 
}

inline void b3Shape::SetDensity(scalar density) 
{ 
	m_density = density; 
}

inline scalar b3Shape::GetRestitution() const 
{ 
	return m_restitution; 
}

inline void b3Shape::SetRestitution(scalar restitution) 
{ 
	m_restitution = restitution; 
}

inline scalar b3Shape::GetFriction() const 
{
	return m_friction;
}

inline void b3Shape::SetFriction(scalar friction) 
{
	m_friction = friction;
}

inline bool b3Shape::IsSensor() const 
{
	return m_isSensor;
}

inline void* b3Shape::GetUserData() const 
{ 
	return m_userData; 
}

inline void b3Shape::SetUserData(void* data) 
{ 
	m_userData = data; 
}

inline const b3Body* b3Shape::GetBody() const 
{
	return m_body;
}

inline b3Body* b3Shape::GetBody() 
{
	return m_body;
}

inline const b3List<b3ContactEdge>& b3Shape::GetContactList() const
{
	return m_contactEdges;
}

inline b3List<b3ContactEdge>& b3Shape::GetContactList()
{
	return m_contactEdges;
}

inline const b3Shape* b3Shape::GetNext() const
{
	return m_next;
}

inline b3Shape* b3Shape::GetNext()
{
	return m_next;
}

#endif
