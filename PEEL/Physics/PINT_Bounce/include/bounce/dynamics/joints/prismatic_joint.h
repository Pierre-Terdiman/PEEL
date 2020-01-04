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

#ifndef B3_PRISMATIC_JOINT_H
#define B3_PRISMATIC_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Prismatic joint definition. 
// This requires defining an axis of motion and an anchor point. 
// The definition uses local anchor points and a local axis so that the initial configuration
// can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space. Using local
// anchors and a local axis helps when saving and loading a game.
struct b3PrismaticJointDef : public b3JointDef
{
	b3PrismaticJointDef()
	{
		type = e_prismaticJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(scalar(1), scalar(0), scalar(0));
		referenceRotation.SetIdentity();
		enableLimit = false;
		lowerTranslation = scalar(0);
		upperTranslation = scalar(0);
		enableMotor = false;
		maxMotorForce = scalar(0);
		motorSpeed = scalar(0);
	}

	// Initialize the bodies, anchors, axis, and reference angle using the world
	// anchor and unit world axis.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor, const b3Vec3& axis);

	// The local anchor point relative to bodyA's origin.
	b3Vec3 localAnchorA;

	// The local anchor point relative to bodyB's origin.
	b3Vec3 localAnchorB;

	// The local translation unit axis in body A's frame.
	b3Vec3 localAxisA;

	// The constrained angle between the bodies.
	b3Quat referenceRotation;

	// Enable/disable the joint limit.
	bool enableLimit;

	// The lower translation limit, usually in meters.
	scalar lowerTranslation;

	// The upper translation limit, usually in meters.
	scalar upperTranslation;

	// Enable/disable the joint motor.
	bool enableMotor;

	// The maximum motor torque, usually in N-m.
	scalar maxMotorForce;

	// The desired motor speed in radians per second.
	scalar motorSpeed;
};

// A prismatic joint. 
// This joint provides translation (one degree of freedom)
// along an axis fixed in body A. 
// Relative rotation is prevented. 
// You can use a joint limit to restrict the range of motion 
// and a joint motor to drive the motion or to model joint friction.
class b3PrismaticJoint : public b3Joint
{
public:
	b3Vec3 GetAnchorA() const;
	b3Vec3 GetAnchorB() const;

	// The local anchor point relative to body A's origin.
	const b3Vec3& GetLocalAnchorA() const { return m_localAnchorA; }

	// The local anchor point relative to body B's origin.
	const b3Vec3& GetLocalAnchorB() const { return m_localAnchorB; }

	// The local joint axis relative to bodyA.
	const b3Vec3& GetLocalAxisA() const { return m_localXAxisA; }

	// Get the reference rotation.
	const b3Quat& GetReferenceRotation() const { return m_referenceRotation; }

	// Get the current joint translation, usually in meters.
	scalar GetJointTranslation() const;

	// Get the current joint translation speed, usually in meters per second.
	scalar GetJointSpeed() const;

	// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	// Get the lower joint limit, usually in meters.
	scalar GetLowerLimit() const;

	// Get the upper joint limit, usually in meters.
	scalar GetUpperLimit() const;

	// Set the joint limits, usually in meters.
	void SetLimits(scalar lower, scalar upper);

	// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	// Set the motor speed, usually in meters per second.
	void SetMotorSpeed(scalar speed);

	// Get the motor speed, usually in meters per second.
	scalar GetMotorSpeed() const;

	// Set the maximum motor force, usually in N.
	void SetMaxMotorForce(scalar force);
	
	// Get the maximum motor  force, usually in N.
	scalar GetMaxMotorForce() const { return m_maxMotorForce; }

	// Draw this joint
	void Draw() const;
protected:
	friend class b3Joint;

	b3PrismaticJoint(const b3PrismaticJointDef* def);

	void InitializeConstraints(const b3SolverData* data);
	void WarmStart(const b3SolverData* data);
	void SolveVelocityConstraints(const b3SolverData* data);
	bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;
	b3Vec3 m_localXAxisA;
	b3Vec3 m_localYAxisA;
	b3Vec3 m_localZAxisA;
	b3Quat m_referenceRotation;
	b3Vec2 m_linearImpulse;
	b3Vec3 m_angularImpulse;
	scalar m_limitImpulse;
	scalar m_motorImpulse;
	scalar m_lowerTranslation;
	scalar m_upperTranslation;
	scalar m_maxMotorForce;
	scalar m_motorSpeed;
	bool m_enableLimit;
	bool m_enableMotor;
	b3LimitState m_limitState;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;
	scalar m_mA;
	scalar m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	
	b3Vec3 m_axis;
	b3Vec3 m_a1, m_a2;
	scalar m_motorMass;

	b3Vec3 m_perp1;
	b3Vec3 m_s1, m_s2;

	b3Vec3 m_perp2;
	b3Vec3 m_s3, m_s4;

	b3Mat22 m_linearMass;

	b3Mat33 m_angularMass;
};

inline scalar b3PrismaticJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

#endif