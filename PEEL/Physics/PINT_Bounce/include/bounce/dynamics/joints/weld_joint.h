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

#ifndef B3_WELD_JOINT_H
#define B3_WELD_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Weld joint (or lock joint).
struct b3WeldJointDef : public b3JointDef
{
	b3WeldJointDef()
	{
		type = e_weldJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		referenceRotation.SetIdentity();
		frequencyHz = scalar(0);
		dampingRatio = scalar(0);
	}

	// Initialize this definition from bodies and world anchor point.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor);

	// The joint anchor relative body A's origin.
	b3Vec3 localAnchorA;

	// The joint anchor relative body B's origin.
	b3Vec3 localAnchorB;

	// The initial relative rotation from body A to body B.
	b3Quat referenceRotation;
	
	// The mass-spring-damper frequency in Hertz. 
	// Disable softness with a value of 0.
	// Rotation only.
	scalar frequencyHz;

	// The damping ratio. 
	// 0 = no damping. 
	// 1 = critical damping.
	scalar dampingRatio;
};

// A weld joint removes the relative movement between two bodies. 
// You need to specify the relative rotation and the local anchor points. 
class b3WeldJoint : public b3Joint
{
public:
	// Set the joint anchor point in world coordinates.
	// Calling this function will set the target relative rotation
	// from body A to body B using the current body rotations.
	void SetAnchor(const b3Vec3& anchor);

	// Set the target relative rotation from body A to body B.
	void SetReferenceRotation(const b3Quat& referenceRotation);

	// Get the anchor point on body A in world coordinates.
	b3Vec3 GetAnchorA() const;

	// Get the anchor point on body B in world coordinates.
	b3Vec3 GetAnchorB() const;

	// Get the local anchor point relative to bodyA's origin.
	const b3Vec3& GetLocalAnchorA() const
	{
		return m_localAnchorA;
	}

	// Get the local anchor point relative to bodyB's origin.
	const b3Vec3& GetLocalAnchorB() const
	{
		return m_localAnchorB;
	}
	
	// Get the reference orientation.
	const b3Quat& GetReferenceRotation() const
	{
		return m_referenceRotation;
	}
	
	// Set the frequency in Hz.
	void SetFrequency(scalar hz)
	{
		m_frequencyHz = hz;
	}

	// Get the frequency in Hz.
	scalar GetFrequency() const
	{
		return m_frequencyHz;
	}

	// Set the damping ratio.
	void SetDampingRatio(scalar ratio)
	{
		m_dampingRatio = ratio;
	}

	// Get the damping ratio.
	scalar GetDampingRatio() const
	{
		return m_dampingRatio;
	}

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3WeldJoint(const b3WeldJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;
	b3Quat m_referenceRotation;
	scalar m_frequencyHz;
	scalar m_dampingRatio;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	scalar m_mA;
	scalar m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;

	// Linear
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Vec3 m_linearImpulse;
	b3Mat33 m_linearMass;

	// Angular
	b3Vec3 m_angularImpulse;
	b3Mat33 m_angularMass;
	b3Mat33 m_gamma;
	b3Vec3 m_bias;
};

#endif