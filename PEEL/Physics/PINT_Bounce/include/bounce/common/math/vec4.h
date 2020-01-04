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

#ifndef B3_VEC4_H
#define B3_VEC4_H

#include <bounce/common/math/math.h>

// A 4D column vector.
struct b3Vec4
{
	// Does nothing for performance.
	b3Vec4() { }

	// Set this vector from four components.
	b3Vec4(scalar _x, scalar _y, scalar _z, scalar _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	// Read an indexed component from this vector.
	scalar operator[](u32 i) const
	{
		return (&x)[i];
	}

	// Write an indexed component to this vector.
	scalar& operator[](u32 i)
	{
		return (&x)[i];
	}

	// Add a vector to this vector.
	void operator+=(const b3Vec4& b)
	{
		x += b.x;
		y += b.y;
		z += b.z;
		w += b.w;
	}

	// Subtract a vector from this vector.
	void operator-=(const b3Vec4& b)
	{
		x -= b.x;
		y -= b.y;
		z -= b.z;
		w -= b.w;
	}

	// Scale this vector.
	void operator*=(scalar s)
	{
		x *= s;
		y *= s;
		z *= s;
		w *= s;
	}

	// Inverse scale this vector.
	void operator/=(scalar a)
	{
		scalar s = scalar(1) / a;
		x *= s;
		y *= s;
		z *= s;
		w *= s;
	}

	// Set this vector to the zero vector.
	void SetZero()
	{
		x = y = z = w = scalar(0);
	}

	// Set this vector from four components.
	void Set(scalar _x, scalar _y, scalar _z, scalar _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	// Convert this vector to the unit vector. Return the length.
	scalar Normalize()
	{
		scalar length = b3Sqrt(x * x + y * y + z * z + w * w);
		if (length > B3_EPSILON)
		{
			scalar s = scalar(1) / length;
			x *= s;
			y *= s;
			z *= s;
			w *= s;
		}
		return length;
	}

	scalar x, y, z, w;
};

// Zero vector
extern const b3Vec4 b3Vec4_zero;

// Compute the sum of two vectors.
inline b3Vec4 operator+(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

// Compute the subtraction of two vectors.
inline b3Vec4 operator-(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

// Compute a scalar-vector product.
inline b3Vec4 operator*(scalar s, const b3Vec4& v)
{
	return b3Vec4(s * v.x, s * v.y, s * v.z, s * v.w);
}

// Compute a scalar-vector product.
inline b3Vec4 operator*(const b3Vec4& v, scalar s)
{
	return b3Vec4(s * v.x, s * v.y, s * v.z, s * v.w);
}

// Inverse multiply a scalar-vector.
inline b3Vec4 operator/(const b3Vec4& v, scalar s)
{
	return b3Vec4(v.x / s, v.y / s, v.z / s, v.w / s);
}

// Compute the dot product of two vectors.
inline scalar b3Dot(const b3Vec4& a, const b3Vec4& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Compute the length of a vector.
inline scalar b3Length(const b3Vec4& v)
{
	return b3Sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

// Compute the squared length of a vector.
inline scalar b3LengthSquared(const b3Vec4& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w; 
}

// Compute the normalized vector of a vector.
inline b3Vec4 b3Normalize(const b3Vec4& v)
{
	scalar length = b3Length(v);
	if (length > B3_EPSILON)
	{
		scalar s = scalar(1) / length;
		return s * v;
	}
	return v;
}

// Compute the minimum vector between two vectors component-wise.
inline b3Vec4 b3Min(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(b3Min(a.x, b.x), b3Min(a.y, b.y), b3Min(a.z, b.z), b3Min(a.w, b.w));
}

// Compute the maximum vector between two vectors component-wise.
inline b3Vec4 b3Max(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(b3Max(a.x, b.x), b3Max(a.y, b.y), b3Max(a.z, b.z), b3Max(a.w, b.w));
}

// Compute the absolute vector of a given vector.
inline b3Vec4 b3Abs(const b3Vec4& v)
{
	return b3Vec4(b3Abs(v.x), b3Abs(v.y), b3Abs(v.z), b3Abs(v.w));
}

// Multiply two vectors component-wise.
inline b3Vec4 b3Mul(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

// Divide two vectors component-wise.
inline b3Vec4 b3Div(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

#endif