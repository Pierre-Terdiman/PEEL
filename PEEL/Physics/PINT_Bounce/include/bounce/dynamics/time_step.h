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

#ifndef B3_TIME_STEP_H
#define B3_TIME_STEP_H

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>

struct b3Position
{
	b3Vec3 x;
	b3Quat q;
};

struct b3Velocity
{
	b3Vec3 v;
	b3Vec3 w;
};

struct b3SolverData
{
	b3Position* positions;
	b3Velocity* velocities;
	b3Mat33* invInertias;
	scalar dt;
	scalar invdt;
};

enum b3LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

// Return the Steiner's matrix given the displacement vector from the old 
// center of rotation to the new center of rotation.
// The result equals to transpose( skew(v) ) * skew(v) or diagonal(v^2) - outer(v, v)
inline b3Mat33 b3Steiner(const b3Vec3& v)
{
	scalar xx = v.x * v.x;
	scalar yy = v.y * v.y;
	scalar zz = v.z * v.z;

	b3Mat33 S;
	
	S.x.x = yy + zz;
	S.x.y = -v.x * v.y;
	S.x.z = -v.x * v.z;

	S.y.x = S.x.y;
	S.y.y = xx + zz;
	S.y.z = -v.y * v.z;

	S.z.x = S.x.z;
	S.z.y = S.y.z;
	S.z.z = xx + yy;

	return S;
}

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Mat33& rotation)
{
	return rotation * inertia * b3Transpose(rotation);
}

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Quat& rotation)
{
	b3Mat33 R = rotation.GetRotationMatrix();
	
	return R * inertia * b3Transpose(R);
}

// Compute the time derivative of an orientation given
// the angular velocity of the rotating frame represented by the orientation.
inline b3Quat b3Derivative(const b3Quat& orientation, const b3Vec3& velocity)
{
	b3Quat w(velocity.x, velocity.y, velocity.z, scalar(0));
	return scalar(0.5) * w * orientation;
}

// Integrate an orientation over a time step given
// the current orientation, angular velocity of the rotating frame
// represented by the orientation, and the time step dt.
inline b3Quat b3Integrate(const b3Quat& orientation, const b3Vec3& omega, scalar dt)
{
	// "Practical Parameterization of Rotations Using the Exponential Map", Grassia
	scalar h = dt;

	scalar x = b3Length(omega);

	const scalar kTol = scalar(10.0e-4);

	b3Vec3 qv;
	if (scalar(0.5) * h * x < kTol)
	{
		// Use first three terms of Taylor expansion of sin(h * x / 2)
		
		// f'(0) / 1! * x = h / 2 * x
		// f''(0) / 2! * x^2 = 0 * 2 / x ^ 2 = 0
		// f'''(0) / 3! * x^3 = -x^3 * h^3 / 48

		// Sum up, divide by x, and simplify (expand)
		// s = h / 2 - (h * h * h) * x * x / 48
		const scalar kInv48 = scalar(1) / scalar(48);
		
		scalar s = scalar(0.5) * h - kInv48 * (h * h * h) * x * x;

		qv = s * omega;
	}
	else
	{
		scalar s = sin(scalar(0.5) * h * x) / x;

		qv = s * omega;
	}

	b3Quat q;
	q.v = qv;
	q.s = cos(scalar(0.5) * h * x);

	b3Quat q1 = q * orientation;
	q1.Normalize();
	return q1;
}

#endif