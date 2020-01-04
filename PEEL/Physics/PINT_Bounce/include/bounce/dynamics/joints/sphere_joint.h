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

#ifndef B3_SPHERE_JOINT_H
#define B3_SPHERE_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Spherical joint definition.
// This is also known as ball-in-socked joint.
struct b3SphereJointDef : public b3JointDef
{
	b3SphereJointDef()
	{
		type = b3JointType::e_sphereJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
	}

	// Initialize this definition from bodies and world anchor point.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor);

	// The anchor point relative to body A's origin
	b3Vec3 localAnchorA;

	// The anchor point relative to body B's origin
	b3Vec3 localAnchorB;
};

// A sphere joint constrains the bodies to rotate relative to each 
// other about a specified anchor point.
class b3SphereJoint : public b3Joint
{
public:
	// Get the anchor point on body A in world coordinates.
	b3Vec3 GetAnchorA() const;
	
	// Get the anchor point on body B in world coordinates.
	b3Vec3 GetAnchorB() const;
	
	// Get the local anchor point relative to body A's origin.
	const b3Vec3& GetLocalAnchorA() const
	{
		return m_localAnchorA;
	}

	// Get the local anchor point relative to body B's origin.
	const b3Vec3& GetLocalAnchorB() const
	{
		return m_localAnchorB;
	}

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3SphereJoint(const b3SphereJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	scalar m_mA;
	scalar m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;
	
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_mass;
	b3Vec3 m_impulse;
};

#endif