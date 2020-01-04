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

#ifndef B3_SOFTBODY_WORLD_SHAPE_H
#define B3_SOFTBODY_WORLD_SHAPE_H

#include <bounce/softbody/shapes/softbody_shape_proxy.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3SoftBody;

struct b3Hull;
class b3SDF;

enum b3SoftBodyWorldShapeType
{
	e_softBodySphereWorldShape,
	e_softBodyCapsuleWorldShape,
	e_softBodyHullWorldShape,
	e_softBodySDFWorldShape,
};

// Sphere contact manifold. 
// c2 = center
// separation = dot(c2 - c1, normal) - r1 - r2
struct b3SoftBodySphereManifold
{
	b3Vec3 point; // contact point on the shape
	b3Vec3 normal; // contact normal on the shape towards the sphere
};

class b3SoftBodyWorldShape;

// Collision shape definition.
// The given shape will be cloned and can be a temporary object.
struct b3SoftBodyWorldShapeDef
{
	b3SoftBodyWorldShapeDef()
	{
		shape = nullptr;
		friction = scalar(0);
	}

	// Shape to be cloned.
	b3SoftBodyWorldShape* shape;
	
	// Coefficient of friction in the range [0, 1].
	scalar friction;
};

// Collision shape in static environment.
class b3SoftBodyWorldShape : public b3SoftBodyShapeBase
{
public:
	// Default ctor.
	b3SoftBodyWorldShape();
	
	// Default dtor.
	virtual ~b3SoftBodyWorldShape() { }
	
	// Return the shape type.
	b3SoftBodyWorldShapeType GetShapeType() const { return m_worldShapeType; }
	
	// Set the coefficient of friction of this shape.
	void SetFriction(scalar friction) { m_friction = friction; }

	// Get the coefficient of friction of this shape.
	// This represents both static and dynamic friction.
	scalar GetFriction() const { return m_friction; }

	// Compute AABB.
	virtual b3AABB ComputeAABB() const = 0;

	// Generate the contact manifold for a given sphere.
	// Return true if the given sphere is colliding with this shape, false otherwise.
	virtual bool CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const = 0;

	// Draw this shape.
	virtual void Draw() const = 0;

	// Return the next world shape in the body list of world shapes.
	b3SoftBodyWorldShape* GetNext() { return m_next; }
	const b3SoftBodyWorldShape* GetNext() const { return m_next; }
	
	// Shape radius.
	scalar m_radius;
protected:
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySphereAndShapeContact;
	friend class b3SoftBodyContactSolver;
	friend class b3List<b3SoftBodyWorldShape>;
	
	// Factory.
	static b3SoftBodyWorldShape* Create(const b3SoftBodyWorldShapeDef& def);
	static void Destroy(b3SoftBodyWorldShape* shape);

	// Synchronize AABB.
	void Synchronize(const b3Vec3& displacement);

	// Destroy contacts.
	void DestroyContacts();

	// The shape type.
	b3SoftBodyWorldShapeType m_worldShapeType;

	// Body
	b3SoftBody* m_body;

	// Coefficient of friction.
	scalar m_friction;

	// Cloth list links.
	b3SoftBodyWorldShape* m_prev;
	b3SoftBodyWorldShape* m_next;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

// Sphere collision shape.
class b3SoftBodySphereWorldShape : public b3SoftBodyWorldShape
{
public:
	b3SoftBodySphereWorldShape();
	~b3SoftBodySphereWorldShape();
	
	void Clone(const b3SoftBodySphereWorldShape& other);
	
	b3AABB ComputeAABB() const;

	bool CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const;
	
	void Draw() const;

	// Center
	b3Vec3 m_center;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

// Capsule collision shape.
class b3SoftBodyCapsuleWorldShape : public b3SoftBodyWorldShape
{
public:
	b3SoftBodyCapsuleWorldShape();
	~b3SoftBodyCapsuleWorldShape();

	void Clone(const b3SoftBodyCapsuleWorldShape& other);

	b3AABB ComputeAABB() const;

	bool CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const;

	void Draw() const;

	// Centers
	b3Vec3 m_center1, m_center2;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

// Convex hull collision shape.
class b3SoftBodyHullWorldShape : public b3SoftBodyWorldShape
{
public:
	b3SoftBodyHullWorldShape();
	~b3SoftBodyHullWorldShape();

	void Clone(const b3SoftBodyHullWorldShape& other);

	b3AABB ComputeAABB() const;

	bool CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const;

	void Draw() const;	

	const b3Hull* m_hull;
	b3Transform m_xf;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

// SDF collision shape.
class b3SoftBodySDFWorldShape : public b3SoftBodyWorldShape
{
public:
	b3SoftBodySDFWorldShape();
	~b3SoftBodySDFWorldShape();

	void Clone(const b3SoftBodySDFWorldShape& other);

	b3AABB ComputeAABB() const;

	bool CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const;

	void Draw() const;

	const b3SDF* m_sdf;
	b3Transform m_xf;
	b3Vec3 m_scale;
};

#endif