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

#ifndef B3_GEOMETRY_H
#define B3_GEOMETRY_H

#include <bounce/common/math/math.h>
#include <bounce/common/math/transform.h>

// A ray in finite form.
// R(t) = O + t * n.
struct b3Ray3
{
	// Return the begin point of this ray.
	b3Vec3 A() const
	{
		return origin;
	}
	
	// Return the end point.
	b3Vec3 B() const
	{
		return origin + fraction * direction;
	}

	b3Vec3 origin;
	b3Vec3 direction;
	scalar fraction;
};

// A plane in constant normal form.
// dot(n, p) - d = 0.
struct b3Plane
{
	// Does nothing for performance.
	b3Plane() { }
	
	// Set this plane from a normal and a signed distance from its origin.
	b3Plane(const b3Vec3& _normal, scalar _offset) 
	{
		normal = _normal;
		offset = _offset;
	}
	
	// Set this plane from a normal and a point on the plane.
	b3Plane(const b3Vec3& _normal, const b3Vec3& _point)
	{ 
		normal = _normal;
		offset = b3Dot(_normal, _point);
	}

	// Compute this plane from three non-colinear points.
	b3Plane(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
	{
		b3Vec3 N = b3Cross(B - A, C - A);
		normal = b3Normalize(N);
		offset = b3Dot(normal, A);
	}

	b3Vec3 normal;
	scalar offset;
};

// Transform a plane by a given frame.
inline b3Plane b3Mul(const b3Transform& T, const b3Plane& plane)
{
	b3Vec3 normal = b3Mul(T.rotation, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, T.translation));
}

// Transform a plane by a given frame.
inline b3Plane operator*(const b3Transform& T, const b3Plane& plane)
{
	return b3Mul(T, plane);
}

// Compute the distance between a point and a plane.
inline scalar b3Distance(const b3Vec3& P, const b3Plane& plane)
{
	return b3Dot(plane.normal, P) - plane.offset;
}

// Project a point onto a normal plane.
inline b3Vec3 b3ClosestPointOnPlane(const b3Vec3& P, const b3Plane& plane)
{
	scalar fraction = b3Distance(P, plane);
	return P - fraction * plane.normal;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v) 
// with respect to a segment AB.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(scalar out[3], 
	const b3Vec3& A, const b3Vec3& B, 
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	
	scalar divisor = b3Dot(AB, AB);
	
	out[0] = b3Dot(QB, AB);
	out[1] = -b3Dot(QA, AB);
	out[2] = divisor;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w) 
// with respect to a triangle ABC.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(scalar out[4],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	//scalar divisor = b3Dot(AB_x_AC, AB_x_AC);

	out[0] = b3Dot(QB_x_QC, AB_x_AC);
	out[1] = b3Dot(QC_x_QA, AB_x_AC);
	out[2] = b3Dot(QA_x_QB, AB_x_AC);
	out[3] = out[0] + out[1] + out[2];
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w, x) 
// with respect to a tetrahedron ABCD.
// The last output value is the (positive) divisor.
inline void b3BarycentricCoordinates(scalar out[5],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;
	b3Vec3 AD = D - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;
	b3Vec3 QD = D - Q;

	scalar divisor = b3Det(AB, AC, AD);
	scalar sign = b3Sign(divisor);

	out[0] = sign * b3Det(QB, QC, QD);
	out[1] = sign * b3Det(QA, QD, QC);
	out[2] = sign * b3Det(QA, QB, QD);
	out[3] = sign * b3Det(QA, QC, QB);
	out[4] = sign * divisor;
}

// Project a point onto a segment AB.
inline b3Vec3 b3ClosestPointOnSegment(const b3Vec3& P, const b3Vec3& A, const b3Vec3& B)
{
	scalar wAB[3];
	b3BarycentricCoordinates(wAB, A, B, P);

	if (wAB[1] <= scalar(0))
	{
		return A;
	}

	if (wAB[0] <= scalar(0))
	{
		return B;
	}

	scalar s = scalar(1) / wAB[2];
	scalar wA = s * wAB[0];
	scalar wB = s * wAB[1];
	return wA * A + wB * B;
}

#endif
