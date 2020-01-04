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

#ifndef B3_TRANSFORM_H
#define B3_TRANSFORM_H

#include <bounce/common/math/mat33.h>
#include <bounce/common/math/mat44.h>
#include <bounce/common/math/quat.h>

// A transform represents a rigid frame. 
// It has a translation representing a position 
// and a rotation quaternion representing an orientation 
// relative to some reference frame.
struct b3Transform 
{
	// Default ctor does nothing for performance.
	b3Transform() { }

	// Set this transform from a translation vector and a rotation quaternion.
	b3Transform(const b3Vec3& _translation, const b3Quat& _rotation) : translation(_translation), rotation(_rotation) { }

	// Set this transform to the identity transform.
	void SetIdentity() 
	{
		translation.SetZero();
		rotation.SetIdentity();
	}

	b3Vec3 translation;
	b3Quat rotation;
};

// Identity transformation
extern const b3Transform b3Transform_identity;

// Convert a transform to a 4-by-4 transformation matrix. 
inline b3Mat44 b3TransformMat44(const b3Transform& T)
{
	b3Vec3 t = T.translation;
	b3Mat33 R = b3QuatMat33(T.rotation);

	return b3Mat44(
		b3Vec4(R.x.x, R.x.y, R.x.z, scalar(0)),
		b3Vec4(R.y.x, R.y.y, R.y.z, scalar(0)),
		b3Vec4(R.z.x, R.z.y, R.z.z, scalar(0)),
		b3Vec4(t.x, t.y, t.z, scalar(1)));
}

// Multiply a transform times a vector.
inline b3Vec3 b3Mul(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T.rotation, v) + T.translation;
}

// Multiply a transform times another transform.
inline b3Transform b3Mul(const b3Transform& A, const b3Transform& B)
{
	// [A y][B x] = [AB Ax+y]
	// [0 1][0 1]   [0  1   ]
	b3Transform C;
	C.rotation = b3Mul(A.rotation, B.rotation);
	C.translation = b3Mul(A.rotation, B.translation) + A.translation;
	return C;
}

// Multiply the transpose of one transform (inverse 
// transform) times another transform (composed transform).
inline b3Transform b3MulT(const b3Transform& A, const b3Transform& B) 
{
	//[A^-1  -A^-1*y][B x] = [A^-1*B A^-1(x-y)]
	//[0      1     ][0 1]   [0      1        ]
	b3Transform C;
	C.rotation = b3MulC(A.rotation, B.rotation);
	C.translation = b3MulC(A.rotation, B.translation - A.translation);
	return C;
}

// Multiply the transpose of a transform times a vector.
// If the transform represents a frame then this transforms
// the vector from one frame to another (inverse transform).
inline b3Vec3 b3MulT(const b3Transform& A, const b3Vec3& v)
{
	//[A^-1  -A^-1*y][x] = A^-1*x - A^-1*y = A^-1 * (x - y)
	//[0     1      ][1]   
	return b3MulC(A.rotation, v - A.translation);
}

// Inverse transform.
inline b3Transform b3Inverse(const b3Transform& T)
{
	b3Transform B;
	B.rotation = b3Conjugate(T.rotation);
	B.translation = b3MulC(T.rotation, -T.translation);
	return B;
}

// Multiply a transform times a vector. If the transform 
// represents a frame this returns the vector in terms 
// of the frame.
inline b3Vec3 operator*(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T, v);
}

// Multiply a transform times another transform (composed transform).
inline b3Transform operator*(const b3Transform& A, const b3Transform& B)
{
	return b3Mul(A, B);
}

// Motion proxy for TOI computation.
struct b3Sweep
{
	// Get this sweep transform at a given time between [0, 1]
	b3Transform GetTransform(scalar t) const;

	b3Vec3 localCenter; // local center

	b3Quat orientation0; // last orientation
	b3Vec3 worldCenter0; // last world center
	
	scalar t0; // last fraction between [0, 1]

	b3Quat orientation; // world orientation
	b3Vec3 worldCenter; // world center
};

inline b3Transform b3Sweep::GetTransform(scalar t) const
{
	b3Vec3 c = (scalar(1) - t) * worldCenter0 + t * worldCenter;

	b3Quat q1 = orientation0;
	b3Quat q2 = orientation;

	if (b3Dot(q1, q2) < scalar(0))
	{
		q1 = -q1;
	}

	b3Quat q = (scalar(1) - t) * q1 + t * q2;
	q.Normalize();

	b3Transform xf;
	xf.translation = c - b3Mul(q, localCenter);
	xf.rotation = q;
	
	return xf;
}

#endif