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

#ifndef B3_MAT_H
#define B3_MAT_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/mat44.h>

// This header contain implementations for some small rectangular 
// matrices. 
// Some operations assume vectors are row or column vectors.

struct b3Mat23
{
	b3Mat23() { }

	b3Mat23(const b3Vec2& _x, const b3Vec2& _y, const b3Vec2& _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
	}

	b3Vec2 x, y, z;
};

struct b3Mat24
{
	b3Mat24() { }

	b3Mat24(const b3Vec2& _x, const b3Vec2& _y, const b3Vec2& _z, const b3Vec2& _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
		w.SetZero();
	}

	b3Vec2 x, y, z, w;
};

struct b3Mat32
{
	b3Mat32() { }

	b3Mat32(const b3Vec3& _x, const b3Vec3& _y)
	{
		x = _x;
		y = _y;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
	}

	b3Vec3 x, y;
};

struct b3Mat34
{
	b3Mat34() { }

	b3Mat34(const b3Vec3& _x, const b3Vec3& _y, const b3Vec3& _z, const b3Vec3& _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
		w.SetZero();
	}

	b3Vec3 x, y, z, w;
};

struct b3Mat43
{
	b3Mat43() { }

	b3Mat43(const b3Vec4& _x, const b3Vec4& _y, const b3Vec4& _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
	}

	b3Vec4 x, y, z;
};

// a * 3x4 = 3x4
inline b3Mat34 operator*(scalar s, const b3Mat34& A)
{
	return b3Mat34(s * A.x, s * A.y, s * A.z, s * A.w);
}

// 4x3 * 3x1 = 4x1
inline b3Vec4 operator*(const b3Mat43& A, const b3Vec3& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z;
}

// 3x4 * 4x1 = 3x1
inline b3Vec3 operator*(const b3Mat34& A, const b3Vec4& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z + v.w * A.w;
}

// 1x3 * 3x1 = 1x1
inline scalar operator*(const b3Vec3& A, const b3Vec3& B)
{
	return A.x * B.x + A.y * B.y + A.z * B.z;
}

// 1x4 * 4x1 = 1x1
inline scalar operator*(const b3Vec4& A, const b3Vec4& B)
{
	return A.x * B.x + A.y * B.y + A.z * B.z + A.w * B.w;
}

// 1x3 * 3x3 = 1x3
inline b3Vec3 operator*(const b3Vec3& A, const b3Mat33& B)
{
	return b3Vec3(A * B.x, A * B.y, A * B.z);
}

// 1x4 * 4x4 = 1x4
inline b3Vec4 operator*(const b3Vec4& A, const b3Mat44& B)
{
	return b3Vec4(A * B.x, A * B.y, A * B.z, A * B.w);
}

// 1x4 * 4x3 = 1x3
inline b3Vec3 operator*(const b3Vec4& A, const b3Mat43& B)
{
	return b3Vec3(A * B.x, A * B.y, A * B.z);
}

// 3x2 * 2x1 = 3x1
inline b3Vec3 operator*(const b3Mat32& A, const b3Vec2& B)
{
	return B.x * A.x + B.y * A.y;
}

// 2x3 * 3x1 = 2x1
inline b3Vec2 operator*(const b3Mat23& A, const b3Vec3& B)
{
	return B.x * A.x + B.y * A.y + B.z * A.z;
}

// 2x3 * 3x2 = 2x2
inline b3Mat22 operator*(const b3Mat23& A, const b3Mat32& B)
{
	return b3Mat22(A * B.x, A * B.y);
}

// 2x3 * 3x3 = 2x3
inline b3Mat23 operator*(const b3Mat23& A, const b3Mat33& B)
{
	return b3Mat23(A * B.x, A * B.y, A * B.z);
}

// 3x4 * 4x3 = 3x3
inline b3Mat33 operator*(const b3Mat34& A, const b3Mat43& B)
{
	return b3Mat33(A * B.x, A * B.y, A * B.z);
}

// 3x4 * 4x4 = 3x3
inline b3Mat34 operator*(const b3Mat34& A, const b3Mat44& B)
{
	return b3Mat34(A * B.x, A * B.y, A * B.z, A * B.w);
}

// 2x4 * 4x1 = 2x1
inline b3Vec2 operator*(const b3Mat24& A, const b3Vec4& B)
{
	return B.x * A.x + B.y * A.y + B.z * A.z + B.w * A.w;
}

// 2x4 * 4x3 = 4x3
inline b3Mat23 operator*(const b3Mat24& A, const b3Mat43& B)
{
	return b3Mat23(A * B.x, A * B.y, A * B.z);
}

// 2x4 * 4x4 = 2x4
inline b3Mat24 operator*(const b3Mat24& A, const b3Mat44& B)
{
	return b3Mat24(A * B.x, A * B.y, A * B.z, A * B.w);
}

// 4x4 * 4x3 = 4x3
inline b3Mat43 operator*(const b3Mat44& A, const b3Mat43& B)
{
	return b3Mat43(A * B.x, A * B.y, A * B.z);
}

inline b3Mat23 b3Transpose(const b3Mat32& A)
{
	b3Mat23 result;
	result.x = b3Vec2(A.x.x, A.y.x);
	result.y = b3Vec2(A.x.y, A.y.y);
	result.z = b3Vec2(A.x.z, A.y.z);
	return result;
}

inline b3Mat32 b3Transpose(const b3Mat23& A)
{
	b3Mat32 result;
	result.x = b3Vec3(A.x.x, A.y.x, A.z.x);
	result.y = b3Vec3(A.x.y, A.y.y, A.z.y);
	return result;
}

inline b3Mat34 b3Transpose(const b3Mat43& A)
{
	b3Mat34 result;
	result.x = b3Vec3(A.x.x, A.y.x, A.z.x);
	result.y = b3Vec3(A.x.y, A.y.y, A.z.y);
	result.z = b3Vec3(A.x.z, A.y.z, A.z.z);
	result.w = b3Vec3(A.x.w, A.y.w, A.z.w);
	return result;
}

inline b3Mat43 b3Transpose(const b3Mat34& A)
{
	b3Mat43 result;
	result.x = b3Vec4(A.x.x, A.y.x, A.z.x, A.w.x);
	result.y = b3Vec4(A.x.y, A.y.y, A.z.y, A.w.y);
	result.z = b3Vec4(A.x.z, A.y.z, A.z.z, A.w.z);
	return result;
}

#endif