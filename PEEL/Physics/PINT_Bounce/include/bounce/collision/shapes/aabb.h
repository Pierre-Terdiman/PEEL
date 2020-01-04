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

#ifndef B3_AABB_H
#define B3_AABB_H

#include <bounce/common/math/transform.h>
#include <bounce/collision/collision.h>

// A min-max representation of a three-dimensional AABB.
struct b3AABB
{
	// Constructor. Does nothing for performance. 
	b3AABB() { }

	// Get the support vertex in a given direction.
	b3Vec3 GetSupportVertex(const b3Vec3& direction) const
	{
		b3Vec3 support;
		support.x = direction.x < scalar(0) ? lowerBound.x : upperBound.x;
		support.y = direction.y < scalar(0) ? lowerBound.y : upperBound.y;
		support.z = direction.z < scalar(0) ? lowerBound.z : upperBound.z;
		return support;
	}

	// Set this AABB from a list of points.
	void Set(const b3Vec3* points, u32 count)
	{
		lowerBound = upperBound = points[0];
		for (u32 i = 1; i < count; ++i)
		{
			lowerBound = b3Min(lowerBound, points[i]);
			upperBound = b3Max(upperBound, points[i]);
		}
	}

	// Set this AABB from a list of points and a transform.
	void Set(const b3Vec3* points, u32 count, const b3Transform& xf)
	{
		lowerBound = upperBound = b3Mul(xf, points[0]);
		for (u32 i = 1; i < count; ++i)
		{
			b3Vec3 v = b3Mul(xf, points[i]);
			lowerBound = b3Min(lowerBound, v);
			upperBound = b3Max(upperBound, v);
		}
	}

	// Set this AABB from a list of points, a local scale, and a transform.
	void Set(const b3Vec3* points, u32 count, const b3Vec3& scale, const b3Transform& xf)
	{
		lowerBound = upperBound = b3Mul(xf, b3Mul(scale, points[0]));
		for (u32 i = 1; i < count; ++i)
		{
			b3Vec3 v = b3Mul(xf, b3Mul(scale, points[i]));
			lowerBound = b3Min(lowerBound, v);
			upperBound = b3Max(upperBound, v);
		}
	}
	
	// Set this AABB from a center point and a radius vector.
	void Set(const b3Vec3& center, const b3Vec3& r)
	{
		lowerBound = center - r;
		upperBound = center + r;
	}

	// Set this AABB from a center point and a radius value.
	void Set(const b3Vec3& center, scalar radius)
	{
		b3Vec3 r(radius, radius, radius);	
		Set(center, r);
	}

	// Extend this AABB by a radius value.
	void Extend(scalar s) 
	{
		b3Vec3 r(s, s, s);
		lowerBound -= r;
		upperBound += r;
	}
	
	// Extend this AABB by a radius vector.
	void Extend(const b3Vec3& r)
	{
		lowerBound -= r;
		upperBound += r;
	}

	// Get the center of this AABB.
	b3Vec3 GetCenter() const 
	{
		return  scalar(0.5) * (lowerBound + upperBound);
	}

	// Get the half-extents of this AABB.
	b3Vec3 GetExtents() const
	{
		return scalar(0.5) * (upperBound - lowerBound);
	}

	// Get the width of this AABB.
	scalar GetWidth() const 
	{
		return upperBound.x - lowerBound.x;
	}

	// Get the height of this AABB.
	scalar GetHeight() const 
	{
		return upperBound.y - lowerBound.y;
	}

	// Get the depth of this AABB.
	scalar GetDepth() const 
	{
		return upperBound.z - lowerBound.z;
	}

	// Get the volume of this AABB.
	scalar GetVolume() const 
	{
		b3Vec3 d = upperBound - lowerBound;
		return d.x * d.y * d.z;
	}

	// Get the surface area of this AABB.
	scalar GetSurfaceArea() const 
	{
		b3Vec3 d = upperBound - lowerBound;
		return scalar(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
	}

	// Get the index of the longest axis of this AABB.
	u32 GetLongestAxisIndex() const
	{
		b3Vec3 d = upperBound - lowerBound;
		
		scalar maxValue = d.x;
		u32 maxIndex = 0;
		
		if (d.y > maxValue)
		{
			maxValue = d.y;
			maxIndex = 1;
		}
		
		if (d.z > maxValue)
		{
			maxValue = d.z;
			maxIndex = 2;
		}
		
		return maxIndex;
	}

	// Test if this AABB contains a point.
	bool Contains(const b3Vec3& point) const
	{
		return	lowerBound.x <= point.x && point.x <= upperBound.x &&
				lowerBound.y <= point.y && point.y <= upperBound.y &&
				lowerBound.z <= point.z && point.z <= upperBound.z;
	}

	// Test if this AABB contains another AABB.
	bool Contains(const b3AABB& aabb) const
	{
		return Contains(aabb.lowerBound) && Contains(aabb.upperBound);
	}

	// Perform a ray-cast against this AABB.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
	{
		b3Vec3 p1 = input.p1;
		b3Vec3 p2 = input.p2;
		b3Vec3 d = p2 - p1;

		b3Vec3 normals[6];
		normals[0].Set(scalar(-1), scalar(0), scalar(0));
		normals[1].Set(scalar(1), scalar(0), scalar(0));
		normals[2].Set(scalar(0), scalar(-1), scalar(0));
		normals[3].Set(scalar(0), scalar(1), scalar(0));
		normals[4].Set(scalar(0), scalar(0), scalar(-1));
		normals[5].Set(scalar(0), scalar(0), scalar(1));

		u32 index = B3_MAX_U32;

		scalar lower = scalar(0);
		scalar upper = input.maxFraction;

		u32 planeIndex = 0;
		
		for (u32 i = 0; i < 3; ++i)
		{
			scalar numerators[2], denominators[2];

			numerators[0] = p1[i] - lowerBound[i];
			numerators[1] = upperBound[i] - p1[i];

			denominators[0] = -d[i];
			denominators[1] = d[i];
			
			for (u32 j = 0; j < 2; ++j)
			{
				scalar numerator = numerators[j];
				scalar denominator = denominators[j];
				
				if (denominator == scalar(0))
				{
					// s is parallel to this half-space.
					if (numerator < scalar(0))
					{
						// s is outside of this half-space.
						// dot(n, p1) and dot(n, p2) < 0.
						return false;
					}
				}
				else
				{
					if (denominator < scalar(0))
					{
						// s enters this half-space.
						if (numerator < lower * denominator)
						{
							// Increase lower.
							lower = numerator / denominator;
							index = planeIndex;
						}
					}
					else
					{
						// s exits the half-space.	
						if (numerator < upper * denominator)
						{
							// Decrease upper.
							upper = numerator / denominator;
						}
					}
					// Exit if intersection becomes empty.
					if (upper < lower)
					{
						return false;
					}
				}

				++planeIndex;
			}
		}

		B3_ASSERT(lower >= scalar(0) && lower <= input.maxFraction);
		
		if (index != B3_MAX_U32)
		{
			output->fraction = lower;
			output->normal = normals[index];
			return true;
		}

		return false;
	}

	b3Vec3 lowerBound; // lower vertex
	b3Vec3 upperBound; // upper vertex
};

// Compute an AABB that encloses two AABBs.
inline b3AABB b3Combine(const b3AABB& a, const b3AABB& b) 
{
	b3AABB aabb;
	aabb.lowerBound = b3Min(a.lowerBound, b.lowerBound);
	aabb.upperBound = b3Max(a.upperBound, b.upperBound);
	return aabb;
}

// Test if two AABBs are overlapping.
inline bool b3TestOverlap(const b3AABB& a, const b3AABB& b) 
{
	return (a.lowerBound.x <= b.upperBound.x) && (a.lowerBound.y <= b.upperBound.y) &&	(a.lowerBound.z <= b.upperBound.z) &&
		   (a.upperBound.x >= b.lowerBound.x) && (a.upperBound.y >= b.lowerBound.y) &&	(a.upperBound.z >= b.lowerBound.z);
}

// Transform a AABB by a given frame.
inline b3AABB b3TransformAABB(const b3AABB& local_aabb, const b3Transform& xf)
{
	b3Vec3 local_center = local_aabb.GetCenter();
	b3Vec3 local_radius = local_aabb.GetExtents();

	b3Vec3 center = xf * local_center;

	b3Mat33 rotation = b3Abs(b3QuatMat33(xf.rotation));
	
	b3Vec3 radius;
	radius.x = b3Dot(local_radius, rotation.x);
	radius.y = b3Dot(local_radius, rotation.y);
	radius.z = b3Dot(local_radius, rotation.z);

	b3AABB aabb;
	aabb.Set(center, radius);

	return aabb;
}

// Transform a AABB given a local scale by a given frame.
inline b3AABB b3TransformAABB(const b3AABB& local_aabb, const b3Vec3& local_scale, const b3Transform& xf)
{
	b3Vec3 local_center = local_aabb.GetCenter();
	b3Vec3 local_radius = b3Mul(b3Abs(local_scale), local_aabb.GetExtents());

	b3Vec3 center = xf * local_center;

	b3Mat33 rotation = b3Abs(b3QuatMat33(xf.rotation));

	b3Vec3 radius;
	radius.x = b3Dot(local_radius, rotation.x);
	radius.y = b3Dot(local_radius, rotation.y);
	radius.z = b3Dot(local_radius, rotation.z);

	b3AABB aabb;
	aabb.Set(center, radius);

	return aabb;
}

// Scale an AABB by a scale.
// The scale can be non-uniform and negative.
inline b3AABB b3ScaleAABB(const b3AABB& aabb, const b3Vec3& scale)
{
	b3AABB scaled_aabb;

	scaled_aabb.lowerBound.x = scale.x > scalar(0) ? scale.x * aabb.lowerBound.x : scale.x * aabb.upperBound.x;
	scaled_aabb.lowerBound.y = scale.y > scalar(0) ? scale.y * aabb.lowerBound.y : scale.y * aabb.upperBound.y;
	scaled_aabb.lowerBound.z = scale.z > scalar(0) ? scale.z * aabb.lowerBound.z : scale.z * aabb.upperBound.z;

	scaled_aabb.upperBound.x = scale.x > scalar(0) ? scale.x * aabb.upperBound.x : scale.x * aabb.lowerBound.x;
	scaled_aabb.upperBound.y = scale.y > scalar(0) ? scale.y * aabb.upperBound.y : scale.y * aabb.lowerBound.y;
	scaled_aabb.upperBound.z = scale.z > scalar(0) ? scale.z * aabb.upperBound.z : scale.z * aabb.lowerBound.z;

	return scaled_aabb;
}

#endif