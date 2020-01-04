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

#ifndef B3_VEC3_H
#define B3_VEC3_H

#include <bounce/common/math/math.h>

// A 3D column vector.
struct b3Vec3 
{
	// Does nothing for performance.
	b3Vec3() { }

	// Set this vector from three components.
	b3Vec3(scalar _x, scalar _y, scalar _z) : x(_x), y(_y), z(_z) { }

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
	void operator+=(const b3Vec3& b) 
	{
		x += b.x;
		y += b.y;
		z += b.z;
	}

	// Subtract a vector from this vector.
	void operator-=(const b3Vec3& b) 
	{
		x -= b.x;
		y -= b.y;
		z -= b.z;
	}

	// Scale this vector.
	void operator*=(scalar s) 
	{
		x *= s;
		y *= s;
		z *= s;
	}
	
	// Inverse scale this vector.
	void operator/=(scalar a)
	{
		scalar s = scalar(1) / a;
		x *= s;
		y *= s;
		z *= s;
	}

	// Set this vector to the zero vector.
	void SetZero() 
	{
		x = y = z = scalar(0);
	}

	// Set this vector from three components.
	void Set(scalar _x, scalar _y, scalar _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	// Convert this vector to the unit vector. Return the length.
	scalar Normalize()
	{
		scalar length = b3Sqrt(x * x + y * y + z * z);
		if (length > B3_EPSILON)
		{
			scalar s = scalar(1) / length;
			x *= s;
			y *= s;
			z *= s;
		}
		return length;
	}

	scalar x, y, z;
};

// Zero vector
extern const b3Vec3 b3Vec3_zero;

// Right vector
extern const b3Vec3 b3Vec3_x;

// Up vector 
extern const b3Vec3 b3Vec3_y;

// Forward vector 
extern const b3Vec3 b3Vec3_z;

// Negate a vector.
inline b3Vec3 operator-(const b3Vec3& v) 
{
	return b3Vec3(-v.x, -v.y, -v.z);
}

// Compute the sum of two vectors.
inline b3Vec3 operator+(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

// Compute the subtraction of two vectors.
inline b3Vec3 operator-(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Compute a scalar-vector product.
inline b3Vec3 operator*(scalar s, const b3Vec3& v) 
{
	return b3Vec3(s * v.x, s * v.y, s * v.z);
}

// Compute a scalar-vector product.
inline b3Vec3 operator*(const b3Vec3& v, scalar s)
{
	return s * v;
}

// Inverse multiply a scalar-vector.
inline b3Vec3 operator/(const b3Vec3& v, scalar s)
{
	return b3Vec3(v.x / s, v.y / s, v.z / s);
}

// Compute the dot product of two vectors.
inline scalar b3Dot(const b3Vec3& a, const b3Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Compute the cross product of two vectors.
inline b3Vec3 b3Cross(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Compute the determinant of a matrix whose columns are three given vectors.
// Useful property: det(a, b, c) = det(c, a, b) = det(b, c, a).
inline scalar b3Det(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c)
{
	return b3Dot(a, b3Cross(b, c));
}

// Compute the length of a vector.
inline scalar b3Length(const b3Vec3& v) 
{
	return b3Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Compute the squared length of a vector.
inline scalar b3LengthSquared(const b3Vec3& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Compute the normalized vector of a vector.
inline b3Vec3 b3Normalize(const b3Vec3& v)
{
	scalar length = b3Length(v);
	if (length > B3_EPSILON)
	{
		scalar s = scalar(1) / length;
		return s * v;
	}
	return v;
}

// Compute the euclidean distance between two points.
inline scalar b3Distance(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Length(a - b);
}

// Compute the squared euclidean distance between two points.
inline scalar b3DistanceSquared(const b3Vec3& a, const b3Vec3& b)
{
	return b3LengthSquared(a - b);
}

// Compute the minimum vector between two vectors component-wise.
inline b3Vec3 b3Min(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(b3Min(a.x, b.x), b3Min(a.y, b.y), b3Min(a.z, b.z));
}

// Compute the maximum vector between two vectors component-wise.
inline b3Vec3 b3Max(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(b3Max(a.x, b.x), b3Max(a.y, b.y), b3Max(a.z, b.z));
}

// Compute the absolute vector of a given vector.
inline b3Vec3 b3Abs(const b3Vec3& v)
{
	return b3Vec3(b3Abs(v.x), b3Abs(v.y), b3Abs(v.z));
}

// Multiply two vectors component-wise.
inline b3Vec3 b3Mul(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Divide two vectors component-wise.
inline b3Vec3 b3Div(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.x / b.x, a.y / b.y, a.z / b.z);
}

// Compute a perpendicular unit vector to a given unit vector.
inline b3Vec3 b3Perp(const b3Vec3& v)
{
	static const scalar sqrt_inv3 = b3Sqrt(scalar(1) / scalar(3));

	b3Vec3 u;
	if (b3Abs(v.x) >= sqrt_inv3)
	{
		u.Set(v.y, -v.x, scalar(0));
	}
	else
	{
		u.Set(scalar(0), v.z, -v.y);
	}

	u.Normalize();
	return u;
}

#endif