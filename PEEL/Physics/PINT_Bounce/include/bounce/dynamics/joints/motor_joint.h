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

#ifndef B3_MOTOR_JOINT_H
#define B3_MOTOR_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Motor joint definition.
struct b3MotorJointDef : public b3JointDef
{
	b3MotorJointDef()
	{
		type = e_motorJoint;
		linearOffset.SetZero();
		angularOffset.SetIdentity();
		maxForce = scalar(0);
		maxTorque = scalar(0);
		correctionFactor = 0.2f;
	}

	// Initialize the bodies and offsets using the current transforms.
	void Initialize(b3Body* bodyA, b3Body* bodyB);

	// Position of body B minus the position of body A, relative body A's origin, in meters.
	b3Vec3 linearOffset;

	// The body B rotation minus body A rotation.
	b3Quat angularOffset;

	// Maximum motor force in N.
	scalar maxForce;

	// Maximum torque in N-m.
	scalar maxTorque;

	// Position correction factor in the range [0, 1].
	scalar correctionFactor;
};

// A motor joint can be used to control the relative movement between two bodies. 
// Commonly it is used to control the motion of a dynamic body with respect 
// to a static body.
class b3MotorJoint : public b3Joint
{
public:
	b3Vec3 GetAnchorA() const;
	b3Vec3 GetAnchorB() const;

	// Set the target linear offset, in frame A, in meters.
	void SetLinearOffset(const b3Vec3& linearOffset);
	
	// Get the target linear offset, in frame A, in meters.
	const b3Vec3& GetLinearOffset() const;

	// Set the target angular offset, in frame A, in meters.
	void SetAngularOffset(const b3Quat& angularOffset);

	// Get the target angular offset, in frame A, in meters.
	const b3Quat& GetAngularOffset() const;
	
	// Set the maximum friction force in N.
	void SetMaxForce(scalar force);

	// Get the maximum friction force in N.
	scalar GetMaxForce() const;

	// Set the maximum friction torque in N-m.
	void SetMaxTorque(scalar force);

	// Get the maximum friction torque in N-m.
	scalar GetMaxTorque() const;
	
	// Set the position correction factor in the range [0, 1].
	void SetCorrectionFactor(scalar factor);

	// Get the position correction factor in the range [0, 1].
	scalar GetCorrectionFactor() const;

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3MotorJoint(const b3MotorJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_linearOffset;
	b3Quat m_angularOffset;
	b3Vec3 m_linearImpulse;
	b3Vec3 m_angularImpulse;
	scalar m_correctionFactor;
	scalar m_maxForce;
	scalar m_maxTorque;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	scalar m_mA;
	scalar m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	b3Vec3 m_linearError;
	b3Vec3 m_angularVelocity;
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_linearMass;
	b3Mat33 m_angularMass;
};

#endif