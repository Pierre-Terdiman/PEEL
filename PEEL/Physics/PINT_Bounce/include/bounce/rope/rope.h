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

#ifndef B3_ROPE_H
#define B3_ROPE_H

#include <bounce/common/math/transform.h>

struct b3RopeBody;

// Rope definition. 
// This definition requires passing an array of point masses 
// and some simulation parameters.
// The arrays will not be used internally after the rope creation. 
// Therefore, you can create the arrays on the stack.
struct b3RopeDef
{
	b3RopeDef()
	{
		count = 0;
		vertices = nullptr;
		masses = nullptr;
		linearDamping = scalar(0.6);
		angularDamping = scalar(0.6);
	}

	// Number of point masses.
	u32 count;

	// Position of each point mass.
	b3Vec3* vertices;

	// Mass of each point mass in kg.
	scalar* masses;

	// Linear coefficient of damping.
	scalar linearDamping;

	// Angular coefficient of damping.
	scalar angularDamping;
};

// This class represents a rope as a chain of links connected to a base link.
// The motion of a child link is determined by the motion of its parent link.
// You can take arbitrary time steps and the joints won't break apart because the 
// simulation is performed using reduced coordinates. 
// However, taking large time steps may blow up the simulation.
// Suggestion: Tune the coefficients of damping and take smaller time steps for improving stability.
class b3Rope
{
public:
	// Construct this rope from a definition.
	b3Rope(const b3RopeDef& def);

	// Rope destructor.
	~b3Rope();

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	const b3Vec3& GetGravity() const;

	// Set the position of the base link.
	void SetPosition(const b3Vec3& position);

	// Get the position of the base link.
	const b3Vec3& GetPosition() const;

	// Set the linear velocity of the base link.
	void SetLinearVelocity(const b3Vec3& linearVelocity);
	
	// Get the linear velocity of the base link.
	b3Vec3 GetLinearVelocity() const;

	// Set the angular velocity of the base link.
	void SetAngularVelocity(const b3Vec3& angularVelocity);
	
	// Get the angular velocity of the base link.
	b3Vec3 GetAngularVelocity() const;

	// Get the number of links.
	u32 GetLinkCount() const;

	// Get the link transform given the link index.
	const b3Transform& GetLinkTransform(u32 index) const;

	// Perform a time-step.
	void Step(scalar dt);

	// Debug draw the links using their transforms.
	void Draw() const;
private:
	// Acceleration of gravity.
	b3Vec3 m_gravity;

	// Linear coefficient of damping.
	scalar m_linearDamping;
	
	// Angular coefficient of damping.
	scalar m_angularDamping;

	// Links. There is no separate base link.
	u32 m_linkCount;
	b3RopeBody* m_links;	
};

inline void b3Rope::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline const b3Vec3& b3Rope::GetGravity() const
{
	return m_gravity;
}

inline u32 b3Rope::GetLinkCount() const
{
	return m_linkCount;
}

#endif