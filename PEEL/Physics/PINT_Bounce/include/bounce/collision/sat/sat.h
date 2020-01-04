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

#ifndef B3_SAT_H
#define B3_SAT_H

#include <bounce/common/geometry.h>

struct b3Hull;

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3FaceQuery
{
	u32 index;
	scalar separation;
};

scalar b3Project(const b3Hull* hull, const b3Plane& plane);

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2);

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3EdgeQuery
{
	u32 index1;
	u32 index2;
	scalar separation;
};

bool b3IsMinkowskiFace(const b3Vec3& A, const b3Vec3& B, const b3Vec3& B_x_A, const b3Vec3& C, const b3Vec3& D, const b3Vec3& D_x_C);

scalar b3Project(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C1);

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2);


///////////////////////////////////////////////////////////////////////////////////////////////////

enum b3SATCacheType
{
	e_separation,
	e_overlap,
	e_empty,
};

enum b3SATFeatureType
{
	e_edge1, // an edge on hull 1 and an edge on hull 2
	e_face1, // a face on hull 1 and a vertex/edge/face on hull 2
	e_face2, // a face on hull 2 and a vertex/edge/face on hull 1
};

// A cached feature pair is used to improve the performance 
// of the SAT when called more than once. 
struct b3SATFeaturePair
{
	b3SATCacheType state; // sat result
	b3SATFeatureType type; // feature pair type
	u32 index1; // feature index on hull 1
	u32 index2; // feature index on hull 2
};

inline b3SATFeaturePair b3MakeFeaturePair(b3SATCacheType state, b3SATFeatureType type, u32 index1, u32 index2)
{
	b3SATFeaturePair pair;
	pair.state = state;
	pair.type = type;
	pair.index1 = index1;
	pair.index2 = index2;
	return pair;
}

struct b3FeatureCache
{
	// Read the current state of the cache.
	// Return e_unkown if neither a separation or penetration was detected.
	b3SATCacheType ReadState(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);

	b3SATCacheType ReadEdge(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);

	b3SATCacheType ReadFace(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);
	
	b3SATFeaturePair m_featurePair;
};

#endif
