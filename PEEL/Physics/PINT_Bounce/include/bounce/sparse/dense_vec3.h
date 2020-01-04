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

#ifndef B3_DENSE_VEC_3_H
#define B3_DENSE_VEC_3_H

#include <bounce/common/math/vec3.h>

struct b3DenseVec3
{
	b3DenseVec3(u32 _n)
	{
		n = _n;
		v = (b3Vec3*)b3Alloc(n * sizeof(b3Vec3));
	}

	b3DenseVec3(const b3DenseVec3& _v)
	{
		n = _v.n;
		v = (b3Vec3*)b3Alloc(n * sizeof(b3Vec3));

		Copy(_v);
	}

	~b3DenseVec3()
	{
		b3Free(v);
	}

	const b3Vec3& operator[](u32 i) const
	{
		B3_ASSERT(i < n);
		return v[i];
	}

	b3Vec3& operator[](u32 i)
	{
		B3_ASSERT(i < n);
		return v[i];
	}

	b3DenseVec3& operator=(const b3DenseVec3& _v)
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
		v = (b3Vec3*)b3Alloc(n * sizeof(b3Vec3));
		
		Copy(_v);

		return *this;
	}

	void Copy(const b3DenseVec3& _v)
	{
		B3_ASSERT(n == _v.n);
		memcpy(v, _v.v, n * sizeof(b3Vec3));
	}

	void SetZero()
	{
		for (u32 i = 0; i < n; ++i)
		{
			v[i].SetZero();
		}
	}

	scalar& GetComponent(u32 i)
	{
		B3_ASSERT(i < 3 * n);
		u32 i0 = i / 3;
		b3Vec3& iv = v[i0];
		u32 ii = i - 3 * i0;
		return iv[ii];
	}

	scalar GetComponent(u32 i) const
	{
		B3_ASSERT(i < 3 * n);
		u32 i0 = i / 3;
		const b3Vec3& iv = v[i0];
		u32 ii = i - 3 * i0;
		return iv[ii];
	}

	b3Vec3* v;
	u32 n;
};

inline void b3Add(b3DenseVec3& out, const b3DenseVec3& a, const b3DenseVec3& b)
{
	B3_ASSERT(out.n == a.n && a.n == b.n);

	for (u32 i = 0; i < a.n; ++i)
	{
		out[i] = a[i] + b[i];
	}
}

inline void b3Sub(b3DenseVec3& out, const b3DenseVec3& a, const b3DenseVec3& b)
{
	B3_ASSERT(out.n == a.n && a.n == b.n);

	for (u32 i = 0; i < a.n; ++i)
	{
		out[i] = a[i] - b[i];
	}
}

inline void b3Mul(b3DenseVec3& out, scalar a, const b3DenseVec3& b)
{
	B3_ASSERT(out.n == b.n);

	for (u32 i = 0; i < b.n; ++i)
	{
		out[i] = a * b[i];
	}
}

inline void b3Negate(b3DenseVec3& out, const b3DenseVec3& v)
{
	b3Mul(out, scalar(-1), v);
}

inline scalar b3Dot(const b3DenseVec3& a, const b3DenseVec3& b)
{
	B3_ASSERT(a.n == b.n);

	scalar result(0);

	for (u32 i = 0; i < a.n; ++i)
	{
		result += b3Dot(a[i], b[i]);
	}
	
	return result;
}

inline scalar b3LengthSquared(const b3DenseVec3& v)
{
	scalar result(0);

	for (u32 i = 0; i < v.n; ++i)
	{
		result += b3LengthSquared(v[i]);
	}

	return result;
}

inline scalar b3Length(const b3DenseVec3& v)
{
	return b3Sqrt(b3LengthSquared(v));
}

inline b3DenseVec3 operator+(const b3DenseVec3& a, const b3DenseVec3& b)
{
	b3DenseVec3 result(a.n);
	b3Add(result, a, b);
	return result;
}

inline b3DenseVec3 operator-(const b3DenseVec3& a, const b3DenseVec3& b)
{
	b3DenseVec3 result(a.n);
	b3Sub(result, a, b);
	return result;
}

inline b3DenseVec3 operator*(scalar a, const b3DenseVec3& b)
{
	b3DenseVec3 result(b.n);
	b3Mul(result, a, b);
	return result;
}

inline b3DenseVec3 operator-(const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Negate(result, v);
	return result;
}

#endif