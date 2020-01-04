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

#ifndef B3_SPATIAL_H
#define B3_SPATIAL_H

#include <bounce/common/math/mat33.h>

// A 6-by-1 motion vector.
struct b3MotionVec
{
	b3MotionVec() { }

	b3MotionVec(const b3Vec3& _w, const b3Vec3& _v)
	{
		w = _w;
		v = _v;
	}

	void SetZero()
	{
		w.SetZero();
		v.SetZero();
	}

	void operator+=(const b3MotionVec& b)
	{
		w += b.w;
		v += b.v;
	}

	void operator-=(const b3MotionVec& b)
	{
		w -= b.w;
		v -= b.v;
	}
	
	b3Vec3 w, v;
};

// a + b
inline b3MotionVec operator+(const b3MotionVec& a, const b3MotionVec& b)
{
	return b3MotionVec(a.w + b.w, a.v + b.v);
}

// a - b
inline b3MotionVec operator-(const b3MotionVec& a, const b3MotionVec& b)
{
	return b3MotionVec(a.w - b.w, a.v - b.v);
}

// -a 
inline b3MotionVec operator-(const b3MotionVec& a)
{
	return b3MotionVec(-a.w, -a.v);
}

// a * s
inline b3MotionVec operator*(const b3MotionVec& a, scalar s)
{
	return b3MotionVec(s * a.w, s * a.v);
}

// s * a
inline b3MotionVec operator*(scalar s, const b3MotionVec& a)
{
	return b3MotionVec(s * a.w, s * a.v);
}

// a / s
inline b3MotionVec operator/(const b3MotionVec& a, scalar s)
{
	return b3MotionVec(a.w / s, a.v / s);
}

// a x b
// [wx  0][w2] = [wx * w2 + 0 * v2] =  [wx * w2]
// [vx wx][v2]   [vx * w2 + wx * v2]   [vx * w2 + wx * v2]
inline b3MotionVec b3Cross(const b3MotionVec& a, const b3MotionVec& b)
{
	b3MotionVec result;
	result.w = b3Cross(a.w, b.w);
	result.v = b3Cross(a.v, b.w) + b3Cross(a.w, b.v);
	return result;
}

// A 6-by-1 force vector.
struct b3SoftBodyForceVec
{
	b3SoftBodyForceVec() { }

	b3SoftBodyForceVec(const b3Vec3& _n, const b3Vec3& _f)
	{
		n = _n;
		f = _f;
	}
	
	void SetZero()
	{
		n.SetZero();
		f.SetZero();
	}

	void operator-=(const b3SoftBodyForceVec& v)
	{
		n -= v.n;
		f -= v.f;
	}

	void operator+=(const b3SoftBodyForceVec& v)
	{
		n += v.n;
		f += v.f;
	}

	b3Vec3 n, f;
};

// a + b
inline b3SoftBodyForceVec operator+(const b3SoftBodyForceVec& a, const b3SoftBodyForceVec& b)
{
	return b3SoftBodyForceVec(a.n + b.n, a.f + b.f);
}

// a - b
inline b3SoftBodyForceVec operator-(const b3SoftBodyForceVec& a, const b3SoftBodyForceVec& b)
{
	return b3SoftBodyForceVec(a.n - b.n, a.f - b.f);
}

// -a
inline b3SoftBodyForceVec operator-(const b3SoftBodyForceVec& a)
{
	return b3SoftBodyForceVec(-a.n, -a.f);
}

// a * s
inline b3SoftBodyForceVec operator*(const b3SoftBodyForceVec& a, scalar s)
{
	return b3SoftBodyForceVec(s * a.n, s * a.f);
}

// s * a
inline b3SoftBodyForceVec operator*(scalar s, const b3SoftBodyForceVec& a)
{
	return b3SoftBodyForceVec(s * a.n, s * a.f);
}

// a / s
inline b3SoftBodyForceVec operator/(const b3SoftBodyForceVec& a, scalar s)
{
	return b3SoftBodyForceVec(a.n / s, a.f / s);
}

// a^T = [a.b^T, a.a^T]
// a^T * b = a.b * b.a + a.a * b.b
inline scalar b3Dot(const b3MotionVec& a, const b3SoftBodyForceVec& b)
{
	return b3Dot(a.v, b.n) + b3Dot(a.w, b.f);
}

// A 6-by-6 spatial inertia matrix stored as a block matrix.
// A, B, C, D are the 3-by-3 matrices. D is not stored  
// because it's defined as D = A^T.
struct b3SpInertia
{
	b3SpInertia() { }

	void SetZero()
	{
		A.SetZero();
		B.SetZero();
		C.SetZero();
	}

	// Set this matrix from mass and rotational inertia 
	// about the local center of mass (zero vector).
	void SetLocalInertia(scalar m, const b3Mat33& I)
	{
		A.SetZero();
		B = b3Mat33Diagonal(m);
		C = I;
	}

	void operator-=(const b3SpInertia& M)
	{
		A -= M.A;
		B -= M.B;
		C -= M.C;
	}

	void operator+=(const b3SpInertia& M)
	{
		A += M.A;
		B += M.B;
		C += M.C;
	}
	
	// Solve Ax = b.
	b3MotionVec Solve(const b3SoftBodyForceVec& b) const;

	b3Mat33 A, B, C;
};

inline b3MotionVec b3SpInertia::Solve(const b3SoftBodyForceVec& b) const
{
	// Numerical Recipes, p. 77
	// Block matrix inversion:
	// https://en.wikipedia.org/wiki/Block_matrix#Block_matrix_inversion
	b3Mat33 invA_A, invA_B, invA_C, invA_D;
	
	b3Mat33 D = b3Transpose(A);
	b3Mat33 NinvB = -b3Inverse(B);
		
	invA_B = b3Inverse(D * NinvB * A + C);
	invA_A = invA_B * D * NinvB;
	invA_D = b3Transpose(invA_A);

	b3Mat33 T = A * invA_A;
	T[0][0] -= scalar(1);
	T[1][1] -= scalar(1);
	T[2][2] -= scalar(1);

	invA_C = NinvB * T;

	b3MotionVec x;
	x.w = invA_A * b.n + invA_B * b.f;
	x.v = invA_C * b.n + invA_D * b.f;
	return x;
}

// M * v
inline b3SoftBodyForceVec operator*(const b3SpInertia& M, const b3MotionVec& v)
{
	b3SoftBodyForceVec result;
	result.n = M.A * v.w + M.B * v.v;
	result.f = M.C * v.w + b3MulT(M.A, v.v);
	return result;
}

// a * b^T
inline b3SpInertia b3Outer(const b3SoftBodyForceVec& a, const b3SoftBodyForceVec& b)
{
	b3SpInertia result;
	result.A = b3Outer(a.n, b.f);
	result.B = b3Outer(a.n, b.n);
	result.C = b3Outer(a.f, b.f);
	return result;
}

// A spatial transformation matrix. This is a 
// 6-by-6 matrix, but we represent it efficiently 
// with a rotation matrix and a translation vector.
struct b3SpTransform
{
	b3SpTransform() { }

	b3SpTransform(const b3Mat33& _E, const b3Vec3& _r)
	{
		E = _E;
		r = _r;
	}

	void SetIdentity()
	{
		E.SetIdentity();
		r.SetZero();
	}

	b3Mat33 E;
	b3Vec3 r;
};

// X * v
inline b3MotionVec b3Mul(const b3SpTransform& X, const b3MotionVec& v)
{
	b3MotionVec result;
	result.w = X.E * v.w;
	result.v = -b3Cross(X.r, X.E * v.w) + X.E * v.v;
	return result;
}

// X^-1 * v
inline b3MotionVec b3MulT(const b3SpTransform& X, const b3MotionVec& v)
{
	b3MotionVec result;
	result.w = b3MulT(X.E, v.w);
	result.v = b3MulT(X.E, v.v + b3Cross(X.r, v.w));
	return result;
}

// X * v
inline b3SoftBodyForceVec b3Mul(const b3SpTransform& X, const b3SoftBodyForceVec& v)
{
	b3SoftBodyForceVec result;
	result.n = X.E * v.n;
	result.f = -b3Cross(X.r, X.E * v.n) + X.E * v.f;
	return result;
}

// X^-1 * v
inline b3SoftBodyForceVec b3MulT(const b3SpTransform& X, const b3SoftBodyForceVec& v)
{
	b3SoftBodyForceVec result;
	result.n = b3MulT(X.E, v.n);
	result.f = b3MulT(X.E, v.f + b3Cross(X.r, v.n));
	return result;
}

// X^-1 * I
inline b3SpInertia b3MulT(const b3SpTransform& X, const b3SpInertia& I)
{
	b3Mat33 E = X.E;
	b3Mat33 ET = b3Transpose(X.E);
	b3Mat33 rx = b3Skew(X.r);

	b3SpInertia result;
	result.A = ET * (I.A - I.B * rx) * E;
	result.B = ET * I.B * E;
	result.C = ET * (rx * (I.A - I.B * rx) + I.C - b3Transpose(I.A) * rx) * E;
	return result;
}

#endif