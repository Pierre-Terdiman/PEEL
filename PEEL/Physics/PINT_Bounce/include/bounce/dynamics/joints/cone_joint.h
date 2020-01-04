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

#ifndef B3_CONE_JOINT_H
#define B3_CONE_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Cone joint definition.
struct b3ConeJointDef : public b3JointDef
{
	b3ConeJointDef()
	{
		type = e_coneJoint;
		localAnchorA.SetZero();
		localRotationA.SetIdentity();
		localAnchorB.SetZero();
		localRotationB.SetIdentity();
		referenceRotation.SetIdentity();
		enableConeLimit = false;
		coneAngle = scalar(0);
		enableTwistLimit = false;
		lowerAngle = scalar(0);
		upperAngle = scalar(0);
	}

	// Initialize this definition from bodies, cone axis, anchor point, and full cone angle in radians.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& axis, const b3Vec3& anchor, scalar coneAngle);

	// The joint anchor relative to body A's origin.
	b3Vec3 localAnchorA;

	// The joint frame relative to body A's frame.
	b3Quat localRotationA;

	// The joint frame relative to body B's origin.
	b3Vec3 localAnchorB;

	// The joint frame relative to body B's frame.
	b3Quat localRotationB;
	
	// Rotation from A to B in reference state
	b3Quat referenceRotation;

	// Enable cone angle limit.
	bool enableConeLimit;

	// The full cone angle in radians.
	scalar coneAngle;

	// Enable the twist limit.
	bool enableTwistLimit;

	// The lower twist angle in radians 
	scalar lowerAngle;

	// The upper twist angle in radians 
	scalar upperAngle;
};

// This joint constrains the bodies to share a common point (cone tip). 
// It also constrains the relative rotation about an axis perpendicular 
// to the cone axis.
// You can limit the relative rotation with a cone angle limit. 
// You can limit the relative rotation around the cone axis with a 
// twist limit.
// This joint was designed to create structures such as ragdolls.
class b3ConeJoint : public b3Joint
{
public:
	// Set the joint limit enabled.
	void SetEnableConeLimit(bool bit);

	// Is the joint cone limit enabled?
	bool IsConeLimitEnabled() const;

	// Set the cone angle in radians.
	void SetConeAngle(scalar angle);

	// Get the cone angle in radians.
	scalar GetConeAngle() const;

	// Set the joint twist limit enabled.
	void SetEnableTwistLimit(bool bit);

	// Is the joint twist limit enabled?
	bool IsTwistLimitEnabled() const;

	// Set/get the twist angles in radians.
	void SetTwistLimits(scalar lowerAngle, scalar upperAngle);
	scalar GetTwistLowerAngle() const;
	scalar GetTwistUpperAngle() const;

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3Body;
	friend class b3World;
	friend class b3Island;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3ConeJoint(const b3ConeJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Quat m_localRotationA;
	b3Vec3 m_localAnchorB;
	b3Quat m_localRotationB;
	b3Quat m_referenceRotation;

	scalar m_coneAngle;
	bool m_enableConeLimit;

	scalar m_lowerAngle;
	scalar m_upperAngle;
	bool m_enableTwistLimit;

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
	
	// Linear
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_mass;
	b3Vec3 m_impulse;

	// Cone limit
	b3Vec3 m_coneAxis;
	scalar m_coneMass;
	scalar m_coneImpulse;
	b3LimitState m_coneState;

	// Twist limit
	b3Vec3 m_twistAxis;
	scalar m_twistMass;
	scalar m_twistImpulse;
	b3LimitState m_twistState;
};

#endif