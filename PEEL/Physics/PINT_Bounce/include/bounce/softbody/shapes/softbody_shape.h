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

#ifndef B3_SOFTBODY_SHAPE_H
#define B3_SOFTBODY_SHAPE_H

#include <bounce/softbody/shapes/softbody_shape_proxy.h>

class b3SoftBody;

// Cloth shape type
enum b3SoftBodyShapeType
{
	e_softBodySphereShape,
	e_softBodyCapsuleShape,
	e_softBodyTriangleShape,
	e_softBodyTetrahedronShape,
	e_maxSoftBodyShapes
};

// Shape definition.
struct b3SoftBodyShapeDef
{
	b3SoftBodyShapeDef()
	{
		radius = scalar(0);
		friction = scalar(0);
		density = scalar(0);
		meshIndex = B3_MAX_U32;
	}

	// Type.
	b3SoftBodyShapeType type;
	
	// Skin.
	scalar radius;

	// Coefficient of friction.
	scalar friction;

	// Density.
	scalar density;
	
	// Feature index into mesh.
	u32 meshIndex;
};

// This is an internal shape.
class b3SoftBodyShape : public b3SoftBodyShapeBase
{
public:
	// Default ctor.
	b3SoftBodyShape() { m_baseType = e_softBodyShapeBase; }

	// Virtual dtor.
	virtual ~b3SoftBodyShape() { }
	
	// Get the shape type.
	b3SoftBodyShapeType GetShapeType() const;
	
	// Get the body.
	b3SoftBody* GetBody();
	const b3SoftBody* GetBody() const;

	// Get the shape radius.
	scalar GetRadius() const;

	// Get the shape density.
	scalar GetDensity() const;

	// Set the coefficient of friction of this shape.
	void SetFriction(scalar friction);

	// Get the coefficient of friction of this shape.
	// This represents both static and dynamic friction.
	scalar GetFriction() const;
protected:
	friend class b3SoftBody;
	friend class b3SoftBodyParticle;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodyContactSolver;

	// Type
	b3SoftBodyShapeType m_shapeType;

	// Body
	b3SoftBody* m_body;

	// Radius
	scalar m_radius;

	// Coefficient of friction
	scalar m_friction;

	// Density
	scalar m_density;

	// Mesh index
	u32 m_meshIndex;
};

inline b3SoftBodyShapeType b3SoftBodyShape::GetShapeType() const
{
	return m_shapeType;
}

inline b3SoftBody* b3SoftBodyShape::GetBody()
{
	return m_body;
}

inline const b3SoftBody* b3SoftBodyShape::GetBody() const
{
	return m_body;
}

inline scalar b3SoftBodyShape::GetRadius() const
{
	return m_radius;
}

inline void b3SoftBodyShape::SetFriction(scalar friction)
{
	m_friction = friction;
}

inline scalar b3SoftBodyShape::GetFriction() const
{
	return m_friction;
}

inline scalar b3SoftBodyShape::GetDensity() const
{
	return m_density;
}

#endif