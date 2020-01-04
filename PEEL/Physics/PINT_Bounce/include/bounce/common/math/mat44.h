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

#ifndef B3_MAT_44_H
#define B3_MAT_44_H

#include <bounce/common/math/vec4.h>

// A 4-by-4 matrix stored in column-major order.
struct b3Mat44
{
	// Does nothing for performance.
	b3Mat44() { }

	// Set this matrix from four column vectors.
	b3Mat44(const b3Vec4& _x, const b3Vec4& _y, const b3Vec4& _z, const b3Vec4& _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}
	// Read an indexed column vector from this matrix.
	const b3Vec4& operator[](u32 i) const
	{
		return (&x)[i];
	}

	// Write an indexed column vector to this matrix.
	b3Vec4& operator[](u32 i)
	{
		return (&x)[i];
	}

	// Read an indexed element from this matrix.
	scalar operator()(u32 i, u32 j) const
	{
		return (&x.x)[i + 4 * j];
	}

	// Write an indexed element to this matrix.
	scalar& operator()(u32 i, u32 j)
	{
		return (&x.x)[i + 4 * j];
	}

	// Add a matrix to this matrix.
	void operator+=(const b3Mat44& B)
	{
		x += B.x;
		y += B.y;
		z += B.z;
		w += B.w;
	}

	// Subtract a matrix from this matrix.
	void operator-=(const b3Mat44& B)
	{
		x -= B.x;
		y -= B.y;
		z -= B.z;
		w -= B.w;
	}

	// Set this matrix to the zero matrix.
	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
		w.SetZero();
	}

	// Set this matrix to the identity matrix.
	void SetIdentity()
	{
		x.Set(scalar(1), scalar(0), scalar(0), scalar(0));
		y.Set(scalar(0), scalar(1), scalar(0), scalar(0));
		z.Set(scalar(0), scalar(0), scalar(1), scalar(0));
		w.Set(scalar(0), scalar(0), scalar(0), scalar(1));
	}

	b3Vec4 x, y, z, w;
};

// Zero matrix
extern const b3Mat44 b3Mat44_zero;

// Identity matrix
extern const b3Mat44 b3Mat44_identity;

// Add two matrices.
inline b3Mat44 operator+(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(A.x + B.x, A.y + B.y, A.z + B.z, A.w + B.w);
}

// Subtract two matrices.
inline b3Mat44 operator-(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(A.x - B.x, A.y - B.y, A.z - B.z, A.w - B.w);
}

// Multiply a scalar times a matrix.
inline b3Mat44 operator*(scalar s, const b3Mat44& A)
{
	return b3Mat44(s * A.x, s * A.y, s * A.z, s * A.w);
}

// Negate a matrix.
inline b3Mat44 operator-(const b3Mat44& A)
{
	return scalar(-1) * A;
}

// Multiply a matrix times a vector.
inline b3Vec4 operator*(const b3Mat44& A, const b3Vec4& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z + v.w * A.w;
}

// Multiply two matrices.
inline b3Mat44 operator*(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(A * B.x, A * B.y, A * B.z, A * B.w);
}

// Multiply a matrix times a vector. If the matrix 
// represents a rotation this transforms the vector 
// from one frame to another.
inline b3Vec4 b3Mul(const b3Mat44& A, const b3Vec4& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z + v.w * A.w;
}

// Multiply two matrices.
inline b3Mat44 b3Mul(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(b3Mul(A, B.x), b3Mul(A, B.y), b3Mul(A, B.z), b3Mul(A, B.w));
}

// Multiply the transpose of a matrix times a vector.
inline b3Vec4 b3MulT(const b3Mat44& A, const b3Vec4& v)
{
	return b3Vec4(b3Dot(A.x, v), b3Dot(A.y, v), b3Dot(A.z, v), b3Dot(A.w, v));
}

// Multiply the transpose of a matrix times another matrix.
inline b3Mat44 b3MulT(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(
		b3Vec4(b3Dot(A.x, B.x), b3Dot(A.y, B.x), b3Dot(A.z, B.x), b3Dot(A.w, B.x)),
		b3Vec4(b3Dot(A.x, B.y), b3Dot(A.y, B.y), b3Dot(A.z, B.y), b3Dot(A.w, B.y)),
		b3Vec4(b3Dot(A.x, B.z), b3Dot(A.y, B.z), b3Dot(A.z, B.z), b3Dot(A.w, B.z)),
		b3Vec4(b3Dot(A.x, B.w), b3Dot(A.y, B.w), b3Dot(A.z, B.w), b3Dot(A.w, B.w)));
}

// Return the absolute matrix of a given matrix.
inline b3Mat44 b3Abs(const b3Mat44& A)
{
	return b3Mat44(b3Abs(A.x), b3Abs(A.y), b3Abs(A.z), b3Abs(A.w));
}

// Transpose a matrix.
inline b3Mat44 b3Transpose(const b3Mat44& A)
{
	return b3Mat44(
		b3Vec4(A.x.x, A.y.x, A.z.x, A.w.x),
		b3Vec4(A.x.y, A.y.y, A.z.y, A.w.y),
		b3Vec4(A.x.z, A.y.z, A.z.z, A.w.z),
		b3Vec4(A.x.w, A.y.w, A.z.w, A.w.w)
	);
}

// Uniform diagonal matrix.
inline b3Mat44 b3Mat44Diagonal(scalar s)
{
	return b3Mat44(
		b3Vec4(s, scalar(0), scalar(0), scalar(0)),
		b3Vec4(scalar(0), s, scalar(0), scalar(0)),
		b3Vec4(scalar(0), scalar(0), s, scalar(0)),
		b3Vec4(scalar(0), scalar(0), scalar(0), s));
}

// Uniform or non-uniform diagonal matrix.
inline b3Mat44 b3Mat44Diagonal(scalar x, scalar y, scalar z, scalar w)
{
	return b3Mat44(
		b3Vec4(x, scalar(0), scalar(0), scalar(0)),
		b3Vec4(scalar(0), y, scalar(0), scalar(0)),
		b3Vec4(scalar(0), scalar(0), z, scalar(0)),
		b3Vec4(scalar(0), scalar(0), scalar(0), w));
}

// Invert a matrix.
// If the matrix is singular this 
// returns the zero matrix.
b3Mat44 b3Inverse(const b3Mat44& A);

// Compute the outer product of two vectors.
// The result is a matrix A = a * b^T.
inline b3Mat44 b3Outer(const b3Vec4& a, const b3Vec4& b)
{
	return b3Mat44(b.x * a, b.y * a, b.z * a, b.w * a);
}

#endif