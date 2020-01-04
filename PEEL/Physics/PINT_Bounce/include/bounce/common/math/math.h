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

#ifndef B3_MATH_H
#define B3_MATH_H

#include <cmath>
#include <cstdlib> // For abs() with integral types
#include <bounce/common/settings.h>

inline bool b3IsInf(scalar x)
{
//	return std::isinf(x);
	return false;
}

inline bool b3IsNaN(scalar x)
{
//	return std::isnan(x);
	return false;
}

inline bool b3IsValid(scalar fx)
{
//	return std::isfinite(fx);
	return false;
}

inline scalar b3Sqrt(scalar x) 
{
	return std::sqrt(x);
}

template <class T>
inline T b3Abs(T x) 
{
	return std::abs(x);
}

template <class T>
inline T b3Min(T a, T b) 
{
	return a < b ? a : b;
}

template <class T>
inline T b3Max(T a, T b) 
{
	return a > b ? a : b;
}

template <class T>
inline T b3Clamp(T a, T low, T high) 
{
	return b3Max(low, b3Min(a, high));
}

template <class T>
inline void b3Swap(T& a, T& b) 
{
	T tmp = a;
	a = b;
	b = tmp;
}

template <class T>
inline T b3Sign(T x)
{
	return T(T(0) < x) - T(T(x) < T(0));
}

template <class T>
inline u32 b3UniqueCount(const T* V, u32 N)
{
	u32 count = 0;
	for (u32 i = 0; i < N; ++i) 
	{
		u32 j;
		for (j = 0; j < N; ++j)
		{
			if (V[i] == V[j])
			{
				break;
			}
		}		
		if (i == j)
		{
			++count;
		}
	}
	return count;
}

// Log a matrix stored in column-major order.
inline void b3Log(const scalar* A, u32 AM, u32 AN)
{
	for (u32 i = 0; i < AM; ++i)
	{
		b3Log("[");
		for (u32 j = 0; j < AN; ++j)
		{
			if (j == 0)
			{
				b3Log("%f", A[i + AM * j]);
			}
			else
			{
				b3Log("\t%f", A[i + AM * j]);
			}
		}
		b3Log("]\n");
	}
}

// Multiply two matrices stored in column-major order.
// C = A * B
inline void b3Mul(scalar* C, const scalar* A, u32 AM, u32 AN, const scalar* B, u32 BM, u32 BN)
{
	B3_ASSERT(AN == BM);

	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < BN; ++j)
		{
			C[i + AM * j] = scalar(0);

			for (u32 k = 0; k < AN; ++k)
			{
				C[i + AM * j] += A[i + AM * k] * B[k + BM * j];
			}
		}
	}
}

// Return the transpose of a given matrix stored in column-major order.
// B = A^T
inline void b3Transpose(scalar* B, const scalar* A, u32 AM, u32 AN)
{
	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < AN; ++j)
		{
			B[j + AN * i] = A[i + AM * j];
		}
	}
}

// Return the original matrix with the given row and column ignored.
// The row and column parameters must be zero based indices to the element.
// The given matrix must be stored in column-major order.
inline void b3SubMatrix(scalar* out, const scalar* A, u32 AM, u32 AN, u32 row, u32 col)
{
	B3_ASSERT(row < AM);
	B3_ASSERT(col < AN);
	
	u32 count = 0;
	
	for (u32 j = 0; j < AN; ++j)
	{
		for (u32 i = 0; i < AM; ++i)
		{
			if (i != row && j != col)
			{
				out[count++] = A[i + AM * j];
			}
		}
	}
}

// Return the lenght of a given vector.
// ||v||
inline scalar b3Length(const scalar* v, u32 n)
{
	scalar result(0);
	for (u32 i = 0; i < n; ++i)
	{
		result += v[i] * v[i];
	}
	return b3Sqrt(result);
}

#endif