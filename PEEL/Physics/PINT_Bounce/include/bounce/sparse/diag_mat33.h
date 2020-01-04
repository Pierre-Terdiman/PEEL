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

#ifndef B3_DIAG_MAT_33_H
#define B3_DIAG_MAT_33_H

#include <bounce/common/math/mat33.h>
#include <bounce/sparse/dense_vec3.h>

// Diagonal matrix storing only the diagonal elements of the 
// original matrix.
struct b3DiagMat33
{
	b3DiagMat33(u32 _n)
	{
		n = _n;
		v = (b3Mat33*)b3Alloc(n * sizeof(b3Mat33));
	}

	b3DiagMat33(const b3DiagMat33& _v)
	{
		n = _v.n;
		v = (b3Mat33*)b3Alloc(n * sizeof(b3Mat33));

		Copy(_v);
	}

	~b3DiagMat33()
	{
		b3Free(v);
	}

	const b3Mat33& operator[](u32 i) const
	{
		B3_ASSERT(i < n);
		return v[i];
	}

	b3Mat33& operator[](u32 i)
	{
		B3_ASSERT(i < n);
		return v[i];
	}

	b3DiagMat33& operator=(const b3DiagMat33& _v)
	{
		if (_v.v == v)
		{
			return *this;
		}

		if (n == _v.n)
		{
			Copy(_v);
			return *this;
		}

		b3Free(v);

		n = _v.n;
		v = (b3Mat33*)b3Alloc(n * sizeof(b3Mat33));

		Copy(_v);

		return *this;
	}

	void Copy(const b3DiagMat33& _v)
	{
		B3_ASSERT(n == _v.n);
		memcpy(v, _v.v, n * sizeof(b3Mat33));
	}

	void SetZero()
	{
		for (u32 i = 0; i < n; ++i)
		{
			v[i].SetZero();
		}
	}

	void SetIdentity()
	{
		for (u32 i = 0; i < n; ++i)
		{
			v[i].SetIdentity();
		}
	}

	b3Mat33* v;
	u32 n;
};

inline void b3Add(b3DiagMat33& out, const b3DiagMat33& a, const b3DiagMat33& b)
{
	B3_ASSERT(out.n == a.n && a.n == b.n);

	for (u32 i = 0; i < a.n; ++i)
	{
		out[i] = a[i] + b[i];
	}
}

inline void b3Sub(b3DiagMat33& out, const b3DiagMat33& a, const b3DiagMat33& b)
{
	B3_ASSERT(out.n == a.n && a.n == b.n);

	for (u32 i = 0; i < a.n; ++i)
	{
		out[i] = a[i] - b[i];
	}
}

inline void b3Mul(b3DiagMat33& out, scalar a, const b3DiagMat33& b)
{
	B3_ASSERT(out.n == b.n);

	for (u32 i = 0; i < b.n; ++i)
	{
		out[i] = a * b[i];
	}
}

inline void b3Mul(b3DenseVec3& out, const b3DiagMat33& a, const b3DenseVec3& b)
{
	B3_ASSERT(out.n == a.n && a.n == b.n);

	for (u32 i = 0; i < b.n; ++i)
	{
		out[i] = a[i] * b[i];
	}
}

inline void b3Negate(b3DiagMat33& out, const b3DiagMat33& v)
{
	b3Mul(out, scalar(-1), v);
}

inline b3DiagMat33 operator+(const b3DiagMat33& a, const b3DiagMat33& b)
{
	b3DiagMat33 result(a.n);
	b3Add(result, a, b);
	return result;
}

inline b3DiagMat33 operator-(const b3DiagMat33& a, const b3DiagMat33& b)
{
	b3DiagMat33 result(a.n);
	b3Sub(result, a, b);
	return result;
}

inline b3DiagMat33 operator*(scalar a, const b3DiagMat33& b)
{
	b3DiagMat33 result(b.n);
	b3Mul(result, a, b);
	return result;
}

inline b3DenseVec3 operator*(const b3DiagMat33& a, const b3DenseVec3& b)
{
	b3DenseVec3 result(b.n);
	b3Mul(result, a, b);
	return result;
}

#endif