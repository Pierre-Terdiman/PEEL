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

#ifndef B3_HULL_AND_EDGE_SAT_H
#define B3_HULL_AND_EDGE_SAT_H

#include <bounce/collision/sat/sat.h>

struct b3Capsule;

///////////////////////////////////////////////////////////////////////////////////////////////////

scalar b3ProjectEdge(const b3Capsule* hull, const b3Plane& plane);

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Capsule* hull2);

///////////////////////////////////////////////////////////////////////////////////////////////////

scalar b3ProjectEdge(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C2);

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Capsule* hull2);

#endif