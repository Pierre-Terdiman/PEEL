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

#ifndef B3_VEC2_H
#define B3_VEC2_H

#include <bounce/common/math/math.h>

// A 2D column vector.
struct b3Vec2
{
	// Does nothing for performance.
	b3Vec2() { }

	// Set this vector from two components.
	b3Vec2(scalar _x, scalar _y) : x(_x), y(_y) { }

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
	void operator+=(const b3Vec2& b)
	{
		x += b.x;
		y += b.y;
	}

	// Subtract a vector from this vector.
	void operator-=(const b3Vec2& b)
	{
		x -= b.x;
		y -= b.y;
	}

	// Scale this vector.
	void operator*=(scalar s)
	{
		x *= s;
		y *= s;
	}

	// Inverse scale this vector.
	void operator/=(scalar s)
	{
		x /= s;
		y /= s;
	}

	// Set this vector to the zero vector.
	void SetZero()
	{
		x = y = scalar(0);
	}

	// Set this vector from two components.
	void Set(scalar _x, scalar _y)
	{
		x = _x;
		y = _y;
	}

	// Convert this vector to the unit vector. Return the lenght.
	scalar Normalize()
	{
		scalar lenght = b3Sqrt(x * x + y * y);
		if (lenght > B3_EPSILON)
		{
			x /= lenght;
			y /= lenght;
		}
		return lenght;
	}

	scalar x, y;
};

// Zero vector 
extern const b3Vec2 b3Vec2_zero;

// Right vector 
extern const b3Vec2 b3Vec2_x;

// Up vector 
extern const b3Vec2 b3Vec2_y;

// Negate a vector.
inline b3Vec2 operator-(const b3Vec2& v)
{
	return b3Vec2(-v.x, -v.y);
}

// Compute the sum of two vectors.
inline b3Vec2 operator+(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x + b.x, a.y + b.y);
}

// Compute the subtraction of two vectors.
inline b3Vec2 operator-(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x - b.x, a.y - b.y);
}

// Compute a scalar-vector product.
inline b3Vec2 operator*(scalar s, const b3Vec2& v)
{
	return b3Vec2(s * v.x, s * v.y);
}

// Compute a scalar-vector product.
inline b3Vec2 operator*(const b3Vec2& v, scalar s)
{
	return s * v;
}

// Inverse multiply a scalar-vector.
inline b3Vec2 operator/(const b3Vec2& v, scalar s)
{
	return b3Vec2(v.x / s, v.y / s);
}

// Compute the dot product of two vectors.
inline scalar b3Dot(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

// Compute the cross product of two vectors. 
// In 2D this results in a scalar.
inline scalar b3Cross(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

// Compute the cross product of a vector and a scalar. 
// In 2D this results in a vector.
inline b3Vec2 b3Cross(const b3Vec2& a, scalar s)
{
	return b3Vec2(s * a.y, -s * a.x);
}

// Compute the cross product of a scalar and a vector. 
// In 2D this results in a vector.
inline b3Vec2 b2Cross(scalar s, const b3Vec2& a)
{
	return b3Vec2(-s * a.y, s * a.x);
}

// Compute the determinant of a matrix whose columns are two given vectors.
inline scalar b3Det(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

// Compute the length of a vector.
inline scalar b3Length(const b3Vec2& v)
{
	return b3Sqrt(v.x * v.x + v.y * v.y);
}

// Compute the squared length of a vector.
inline scalar b3LengthSquared(const b3Vec2& v)
{
	return v.x * v.x + v.y * v.y;
}

// Compute the normalized vector of a vector.
inline b3Vec2 b3Normalize(const b3Vec2& v)
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
inline scalar b3Distance(const b3Vec2& a, const b3Vec2& b)
{
	return b3Length(a - b);
}

// Compute the squared euclidean distance between two points.
inline scalar b3DistanceSquared(const b3Vec2& a, const b3Vec2& b)
{
	return b3LengthSquared(a - b);
}

// Compute the minimum vector between two vectors component-wise.
inline b3Vec2 b3Min(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(b3Min(a.x, b.x), b3Min(a.y, b.y));
}

// Compute the maximum vector between two vectors component-wise.
inline b3Vec2 b3Max(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(b3Max(a.x, b.x), b3Max(a.y, b.y));
}

// Compute the absolute vector of a given vector component-wise.
inline b3Vec2 b3Abs(const b3Vec2& v)
{
	return b3Vec2(b3Abs(v.x), b3Abs(v.y));
}

// Multiply two vectors component-wise.
inline b3Vec2 b3Mul(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x * b.x, a.y * b.y);
}

// Divide two vectors component-wise.
inline b3Vec2 b3Div(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x / b.x, a.y / b.y);
}

// Compute a perpendicular vector to a given vector (CCW).
inline b3Vec2 b3Perp(const b3Vec2& v)
{
	return b3Vec2(-v.y, v.x);
}

#endif