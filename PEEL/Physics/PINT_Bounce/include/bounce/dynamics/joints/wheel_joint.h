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

#ifndef B3_WHEEL_JOINT_H
#define B3_WHEEL_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Wheel joint definition. 
// This requires defining two axis and an anchor point.
// The anchor point usually coincides with the wheel position.
// The definition uses local anchor points and a local axes so that 
// the initial configuration can violate the constraint slightly. 
// The joint translation is zero when the local anchor points coincide in world space. 
// Using local anchors and a local axes helps when saving and loading a game.
struct b3WheelJointDef : public b3JointDef
{
	b3WheelJointDef()
	{
		type = e_wheelJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(scalar(0), scalar(1), scalar(0));
		localAxisB.Set(scalar(1), scalar(0), scalar(0));
		enableMotor = false;
		maxMotorTorque = scalar(0);
		motorSpeed = scalar(0);
		frequencyHz = scalar(4);
		dampingRatio = scalar(0.7);
	}

	// Initialize the bodies, anchors, axis, and reference angle using the world
	// anchor and world axes.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor, const b3Vec3& axisA, const b3Vec3& axisB);

	// The local anchor point relative to bodyA's origin.
	b3Vec3 localAnchorA;

	// The local anchor point relative to bodyB's origin.
	b3Vec3 localAnchorB;

	// The local translation axis in body A.
	b3Vec3 localAxisA;

	// The local rotation axis in body B.
	b3Vec3 localAxisB;
	
	// Enable/disable the joint motor.
	bool enableMotor;

	// The maximum motor torque, usually in N-m.
	scalar maxMotorTorque;

	// The desired motor velocity in radians per second.
	scalar motorSpeed;

	// Suspension frequency, zero indicates no suspension
	scalar frequencyHz;

	// Suspension damping ratio, one indicates critical damping
	scalar dampingRatio;
};

// A wheel joint. 
// This joint will keep the motion of the anchor point along a translation 
// axis fixed in body A. 
// This joint maintains the initial angle between an axis on 
// body A and another axis on body B. 
// The anchor point usually coincides with the wheel position.
// You can control the spring parameters along the translation axis on body A.
// You can control the desired angular motor speed along the axis on body B.
// This joint is designed for vehicle suspensions.
class b3WheelJoint : public b3Joint
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

	// Get the current joint translation, usually in meters.
	scalar GetJointTranslation() const;

	// Get the current joint linear speed, usually in meters per second.
	scalar GetJointLinearSpeed() const;

	// Get the current joint rotation quaternion.
	b3Quat GetJointRotation() const;

	// Get the current joint angular speed in radians per second.
	scalar GetJointAngularSpeed() const;

	// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	// Set the motor angular speed, usually in radians per second.
	void SetMotorSpeed(scalar speed);

	// Get the motor angular speed, usually in radians per second.
	scalar GetMotorSpeed() const;

	// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(scalar torque);
	scalar GetMaxMotorTorque() const;

	// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	void SetSpringFrequencyHz(scalar hz);
	scalar GetSpringFrequencyHz() const;

	// Set/Get the spring damping ratio
	void SetSpringDampingRatio(scalar ratio);
	scalar GetSpringDampingRatio() const;

	// Draw this joint.
	void Draw() const;
protected:
	friend class b3Joint;
	
	b3WheelJoint(const b3WheelJointDef* def);

	void InitializeConstraints(const b3SolverData* data);
	void WarmStart(const b3SolverData* data);
	void SolveVelocityConstraints(const b3SolverData* data);
	bool SolvePositionConstraints(const b3SolverData* data);

	scalar m_frequencyHz;
	scalar m_dampingRatio;

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;

	b3Vec3 m_localXAxisA;
	b3Vec3 m_localYAxisA;
	b3Vec3 m_localZAxisA;
	
	b3Vec3 m_localXAxisB;

	scalar m_referenceCosine;
	scalar m_referenceAngle;

	b3Vec2 m_linearImpulse;
	scalar m_motorImpulse;
	scalar m_springImpulse;

	b3Vec3 m_u;
	scalar m_angularMass;
	scalar m_angularImpulse;

	scalar m_maxMotorTorque;
	scalar m_motorSpeed;
	bool m_enableMotor;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	scalar m_mA;
	scalar m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;

	b3Vec3 m_a1, m_a2;

	b3Vec3 m_perp1;
	b3Vec3 m_s1, m_s2;

	b3Vec3 m_perp2;
	b3Vec3 m_s3, m_s4;

	b3Mat22 m_linearMass;

	b3Vec3 m_axisA;
	b3Vec3 m_axisB;

	scalar m_mass;
	scalar m_motorMass;
	scalar m_springMass;

	scalar m_bias;
	scalar m_gamma;
};

inline scalar b3WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline scalar b3WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

inline void b3WheelJoint::SetSpringFrequencyHz(scalar hz)
{
	m_frequencyHz = hz;
}

inline scalar b3WheelJoint::GetSpringFrequencyHz() const
{
	return m_frequencyHz;
}

inline void b3WheelJoint::SetSpringDampingRatio(scalar ratio)
{
	m_dampingRatio = ratio;
}

inline scalar b3WheelJoint::GetSpringDampingRatio() const
{
	return m_dampingRatio;
}

#endif