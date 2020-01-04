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

#ifndef B3_MANIFOLD_H
#define B3_MANIFOLD_H

#include <bounce/common/math/vec2.h>
#include <bounce/dynamics/contacts/collide/clip.h>

#define B3_NULL_TRIANGLE (0xFFFFFFFF)

// A contact manifold point.
struct b3ManifoldPointKey
{
	bool operator==(const b3ManifoldPointKey& other) const
	{
		return triangleKey == other.triangleKey &&
			key1 == other.key1 && key2 == other.key2;
	}

	u32 triangleKey; 
	u64 key1; 
	u64 key2; 
};

inline b3ManifoldPointKey b3MakeKey(const b3FeaturePair& featurePair)
{
	struct b3Key128
	{
		u64 key1;
		u64 key2;
	};

	union b3FeaturePairKey
	{
		b3FeaturePair pair;
		b3Key128 key;
	};

	b3FeaturePairKey fpkey;
	fpkey.pair = featurePair;

	b3ManifoldPointKey key;
	key.triangleKey = B3_NULL_TRIANGLE;
	key.key1 = fpkey.key.key1;
	key.key2 = fpkey.key.key2;
	return key;
}

// A contact manifold point.
struct b3ManifoldPoint
{
	b3Vec3 localNormal1; // local normal on the first shape 
	b3Vec3 localPoint1; // local point on the first shape without its radius
	b3Vec3 localPoint2; // local point on the other shape without its radius

	b3ManifoldPointKey key; // point identifier

	scalar normalImpulse; // normal impulse
	u64 persistCount; // number of time steps this point is persisting

	b3FeaturePair featurePair; // internal use
	bool edgeContact; // internal use
};

// A contact manifold is a group of contact points with similar contact normal.
struct b3Manifold
{
	// Clear the manifold.
	// Initialize impulses arbitrarily for warm starting.
	void Initialize();
	
	// Initialize impulses for warm starting from the old manifold.
	void Initialize(const b3Manifold& old);
	
	b3ManifoldPoint points[B3_MAX_MANIFOLD_POINTS]; // manifold points
	u32 pointCount; // number of manifold points

	b3Vec2 tangentImpulse;
	scalar motorImpulse;
	
	scalar motorSpeed; // target angular speed along the normal
	scalar tangentSpeed1; // target speed along the first tangent
	scalar tangentSpeed2; // target speed along the second tangent
};

struct b3WorldManifoldPoint
{
	void Initialize(const b3ManifoldPoint* p, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB);
	
	b3Vec3 point;
	b3Vec3 normal;
	scalar separation;
};

struct b3WorldManifold
{
	void Initialize(const b3Manifold* m, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB);

	u32 pointCount; 
	b3WorldManifoldPoint points[B3_MAX_MANIFOLD_POINTS]; 

	b3Vec3 center;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
};

#endif