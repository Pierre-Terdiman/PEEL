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

#ifndef B3_MAT33_H
#define B3_MAT33_H

#include <bounce/common/math/vec3.h>

// A 3-by-3 matrix stored in column-major order.
struct b3Mat33 
{
	// Does nothing for performance.
	b3Mat33() { }

	// Set this matrix from three column vectors.
	b3Mat33(const b3Vec3& _x, const b3Vec3& _y, const b3Vec3& _z) :	x(_x), y(_y), z(_z) { }

	// Read an indexed column vector from this matrix.
	const b3Vec3& operator[](u32 i) const 
	{
		return (&x)[i];
	}
	
	// Write an indexed column vector to this matrix.
	b3Vec3& operator[](u32 i) 
	{
		return (&x)[i];
	}

	// Read an indexed element from this matrix.
	scalar operator()(u32 i, u32 j) const
	{
		return (&x.x)[i + 3 * j];
	}

	// Write an indexed element to this matrix.
	scalar& operator()(u32 i, u32 j) 
	{
		return (&x.x)[i + 3 * j];
	}
	
	// Add a matrix to this matrix.
	void operator+=(const b3Mat33& B)
	{
		x += B.x;
		y += B.y;
		z += B.z;
	}

	// Subtract a matrix from this matrix.
	void operator-=(const b3Mat33& B)
	{
		x -= B.x;
		y -= B.y;
		z -= B.z;
	}
	
	// Set this matrix to the zero matrix.
	void SetZero() 
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
	}

	// Set this matrix to the identity matrix.
	void SetIdentity() 
	{
		x.Set(scalar(1), scalar(0), scalar(0));
		y.Set(scalar(0), scalar(1), scalar(0));
		z.Set(scalar(0), scalar(0), scalar(1));
	}

	// Solve Ax = b. 
	// It doesn't compute the inverse. 
	// Therefore, is more efficient.
	// Returns the zero vector if the matrix is singular.
	b3Vec3 Solve(const b3Vec3& b) const;

	b3Vec3 x, y, z;
};

// Zero matrix
extern const b3Mat33 b3Mat33_zero;

// Identity matrix
extern const b3Mat33 b3Mat33_identity;

// Add two matrices.
inline b3Mat33 operator+(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A.x + B.x, A.y + B.y, A.z + B.z);
}

// Subtract two matrices.
inline b3Mat33 operator-(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A.x - B.x, A.y - B.y, A.z - B.z);
}

// Multiply a scalar times a matrix.
inline b3Mat33 operator*(scalar s, const b3Mat33& A) 
{
	return b3Mat33(s * A.x, s * A.y, s * A.z);
}

// Negate a matrix.
inline b3Mat33 operator-(const b3Mat33& A)
{
	return scalar(-1) * A;
}

// Multiply a matrix times a vector.
inline b3Vec3 operator*(const b3Mat33& A, const b3Vec3& v) 
{
	return v.x * A.x + v.y * A.y + v.z * A.z;
}

// Multiply two matrices.
inline b3Mat33 operator*(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A * B.x, A * B.y, A * B.z);
}

// Multiply a matrix times a vector. If the matrix 
// represents a rotation this transforms the vector 
// from one frame to another.
inline b3Vec3 b3Mul(const b3Mat33& A, const b3Vec3& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z;
}

// Multiply two matrices.
inline b3Mat33 b3Mul(const b3Mat33& A, const b3Mat33& B)
{
	return b3Mat33( b3Mul(A, B.x), b3Mul(A, B.y), b3Mul(A, B.z));
}

// Multiply the transpose of a matrix times a vector.
inline b3Vec3 b3MulT(const b3Mat33& A, const b3Vec3& v) 
{
	return b3Vec3(b3Dot(A.x, v), b3Dot(A.y, v), b3Dot(A.z, v));
}

// Multiply the transpose of a matrix times another matrix.
inline b3Mat33 b3MulT(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(
		b3Vec3(b3Dot(A.x, B.x), b3Dot(A.y, B.x), b3Dot(A.z, B.x)),
		b3Vec3(b3Dot(A.x, B.y), b3Dot(A.y, B.y), b3Dot(A.z, B.y)),
		b3Vec3(b3Dot(A.x, B.z), b3Dot(A.y, B.z), b3Dot(A.z, B.z)));
}

// Return the absolute matrix of a given matrix.
inline b3Mat33 b3Abs(const b3Mat33& A)
{
	return b3Mat33(b3Abs(A.x), b3Abs(A.y), b3Abs(A.z));
}

// Transpose a matrix.
inline b3Mat33 b3Transpose(const b3Mat33& A) 
{
	return b3Mat33(
		b3Vec3(A.x.x, A.y.x, A.z.x),
		b3Vec3(A.x.y, A.y.y, A.z.y),
		b3Vec3(A.x.z, A.y.z, A.z.z)
		);
}

// Uniform diagonal matrix.
inline b3Mat33 b3Mat33Diagonal(scalar s) 
{
	return b3Mat33(
		b3Vec3(s, scalar(0), scalar(0)),
		b3Vec3(scalar(0), s, scalar(0)),
		b3Vec3(scalar(0), scalar(0), s));
}

// Uniform or non-uniform diagonal matrix.
inline b3Mat33 b3Mat33Diagonal(scalar x, scalar y, scalar z)
{
	return b3Mat33(
		b3Vec3(x, scalar(0), scalar(0)),
		b3Vec3(scalar(0), y, scalar(0)),
		b3Vec3(scalar(0), scalar(0), z));
}

// Return the cofactor matrix of a given matrix.
inline b3Mat33 b3CofactorMatrix(const b3Mat33& A)
{
	// Cofactor matrix using cross products.
	//
	// c = cross(a, b) = [i  j  k ] = i * minor(i) - j * minor(j) + k * minor(k)
	//                   [a1 a2 a3]
	//                   [b1 b2 b3]
	//
	// Therefore,
	//
	// c1 = minor(i)
	// c2 = -minor(j)
	// c3 = minor(k)
	//
	// A = [a11 a12 a13]
	//     [a21 a22 a23]
	//     [a31 a32 a33]
	// 
	// Properties used:
	// 
	// |A| = |A^T|
	// 
	// |a b| = -|c a|
	// |c d|    |d b| 
	// 
	// cx  = cross(y, z) = [i   j   k  ]
	//                     [a12 a22 a32]
	//                     [a13 a23 a33]
	// 
	// c1 = |a22 a32| = |a22 a23|
	//      |a23 a33|   |a32 a33|
	// 
	// c2 = -|a12 a32| = -|a12 a13|
	//       |a13 a33|    |a32 a33|
	// 
	// c3 = |a12 a22| = |a12 a13|
	//      |a13 a23|   |a22 a23|
	//
	// cy = cross(z, x) = [i   j   k  ]
	//                    [a13 a23 a33]
	//                    [a11 a21 a31]
	// 
	// c1 = |a23 a33| = -|a21 a23| 
	//      |a21 a31|    |a31 a33|
	//
	// c2 = -|a13 a33| = -(-|a11 a13|) = |a11 a13|
	//       |a11 a31|	  ( |a31 a33|)   |a31 a33|
	// 
	// c2 = |a13 a23| = -|a11 a13|
	//      |a11 a21|    |a21 a23|
	// 
	// cz = cross(x, y) = [i   j   k  ]
	//                    [a11 a21 a31]
	//                    [a12 a22 a32]
	// 
	// c1 = |a21 a31| = |a21 a22|
	//      |a22 a32|   |a31 a32|
	//
	// c2 = -|a11 a31| = -|a11 a12|
	//       |a12 a32|    |a31 a32|
	//
	// c3 = |a11 a21| = |a11 a12|
	//      |a12 a22|   |a21 a22|
	//
	b3Mat33 C;
	C.x = b3Cross(A.y, A.z);
	C.y = b3Cross(A.z, A.x);
	C.z = b3Cross(A.x, A.y);
	return C;
}

// Return the adjugate of a matrix.
inline b3Mat33 b3Adjugate(const b3Mat33& A)
{
	return b3Transpose(b3CofactorMatrix(A));
}

// Invert a matrix.
// If the matrix is singular this 
// returns the zero matrix.
b3Mat33 b3Inverse(const b3Mat33& A);

// Invert a symmetric matrix.
// If the matrix is singular this 
// returns the zero matrix.
b3Mat33 b3SymInverse(const b3Mat33& A);

// Return a skew (anti-symmetric) matrix for a vector such that 
// skew(v) * w = cross(v, w).
inline b3Mat33 b3Skew(const b3Vec3& v) 
{
	return b3Mat33(
		b3Vec3(scalar(0), v.z, -v.y),
		b3Vec3(-v.z, scalar(0), v.x),
		b3Vec3(v.y, -v.x, scalar(0)));
}

// Compute the outer product of two vectors.
// The result is a matrix A = a * b^T.
inline b3Mat33 b3Outer(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Mat33(b.x * a, b.y * a, b.z * a);
}

// Compute an orthogonal basis given one of its vectors.
// The vector must be normalized.
inline void b3ComputeBasis(const b3Vec3& a, b3Vec3& b, b3Vec3& c)
{
	// https://box2d.org/2014/02/computing-a-basis/, (Erin)
	// Suppose vector a has all equal components and is a unit vector : a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3). 
	// This means that at least one component of a unit vector must be greater or equal to s.
	static const scalar sqrt_inv3 = b3Sqrt(scalar(1) / scalar(3));

	if (b3Abs(a.x) >= sqrt_inv3)
	{
		b.Set(a.y, -a.x, scalar(0));
	}
	else
	{
		b.Set(scalar(0), a.z, -a.y);
	}
	
	b.Normalize();
	c = b3Cross(a, b);
}

#endif