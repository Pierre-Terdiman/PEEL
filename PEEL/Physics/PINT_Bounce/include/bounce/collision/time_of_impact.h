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

#ifndef B3_TIME_OF_IMPACT_H
#define B3_TIME_OF_IMPACT_H

#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/math/transform.h>

// Input parameters for b3TimeOfImpact
struct b3TOIInput
{
	b3GJKProxy proxyA;
	b3GJKProxy proxyB;
	b3Sweep sweepA;
	b3Sweep sweepB;
	scalar tMax; // sweep interval in [0, 1]
};

// Output parameters of b3TimeOfImpact
struct b3TOIOutput
{
	enum State
	{
		e_unknown,
		e_failed,
		e_overlapped,
		e_touching,
		e_separated
	};

	State state;
	scalar t;
	u32 iterations;
};

// Compute the time of impact between two shapes.
// This is represented as a fraction between [0, tMax].
// Use b3GJK to compute the contact point and normal at the time of impact.
b3TOIOutput b3TimeOfImpact(const b3TOIInput& input);

// Compute the time of impact between two shapes.
// This is represented as a fraction between [0, 1].
// You must supply the linear displacements of each shape.
// Use b3GJK to compute the contact point and normal at the time of impact.
b3TOIOutput b3TimeOfImpact(const b3Transform& xf1, const b3GJKProxy& proxy1, const b3Vec3& d1,
	const b3Transform& xf2, const b3GJKProxy& proxy2, const b3Vec3& d2, 
	u32 maxIterations = 20);

// Compute the time of impact between two AABBs.
// This is represented as a fraction between [0, 1].
// You must supply the linear displacements of each AABB center.
b3TOIOutput b3TimeOfImpact(const b3AABB& aabb1, const b3Vec3& d1, const b3AABB& aabb2, const b3Vec3& d2);

#endif