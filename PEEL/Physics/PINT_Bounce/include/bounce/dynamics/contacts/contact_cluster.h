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

#ifndef B3_CONTACT_CLUSTER_H
#define B3_CONTACT_CLUSTER_H

#include <bounce/common/geometry.h>
#include <bounce/common/template/array.h>
#include <bounce/dynamics/contacts/manifold.h>

// Used for contact cluster reduction.
struct b3ClusterPolygonVertex
{
	b3Vec3 position; // point on the cluster plane
	u32 clipIndex; // where did this vertex came from 
};

// Used for contact cluster reduction.
typedef b3Array<b3ClusterPolygonVertex> b3ClusterPolygon;

// Sort a convex polygon such that the polygon normal points to a given normal.
void b3SortPolygon(b3ClusterPolygon& pOut, 
	const b3ClusterPolygon& pIn, const b3Vec3& pNormal);

// Reduce a set of contact points to a quad (approximate convex polygon).
// All points must lie in a common plane and an initial point must be given.
void b3ReducePolygon(b3ClusterPolygon& pOut, 
	const b3ClusterPolygon& pIn, const b3Vec3& pNormal, u32 initialPoint);

#define B3_NULL_CLUSTER (0xFFFFFFFF)

// An observation represents a contact normal.	
struct b3Observation
{
	b3Vec3 point; 
	u32 cluster; 
	u32 manifold;
	u32 manifoldPoint;
};

// A group of contact normals pointing to a similar direction.
struct b3Cluster
{
	b3Vec3 centroid;
};

// This is used for simplifying contact manifolds.
// It performs k-means for grouping contact points by their normals.
// It also performs contact point reduction.
class b3ClusterSolver
{
public:
	b3ClusterSolver();
	~b3ClusterSolver();

	// Pushes an observation. This is usually a contact point.
	void AddObservation(const b3Observation& observation);

	// Number of k-means iterations.
	u32 GetIterations() const;

	// Return the observations.
	const b3Array<b3Observation>& GetObservations() const;

	// Return the set of clusters. 
	const b3Array<b3Cluster>& GetClusters() const;

	// Perform clustering on a set of manifolds.
	void Run(b3Manifold mOut[3], u32& numOut,
	const b3Manifold* mIn, u32 numIn,
		const b3Transform& xfA, scalar radiusA, const b3Transform& xfB, scalar radiusB);

	// Run the cluster algorithm.
	void Solve();
private:
	// Cluster initialization.
	void InitializeClusters();
	
	// Adds a new cluster. This merges similar clusters.
	void AddCluster(const b3Vec3& centroid);

	// Find the cluster closest to the given point.
	u32 FindCluster(const b3Vec3& point) const;

	// Number of k-means iterations.
	u32 m_iterations;

	// Observations.
	b3StackArray<b3Observation, 256> m_observations;
	
	// Clusters.
	b3StackArray<b3Cluster, 256> m_clusters;
};

inline void b3ClusterSolver::AddObservation(const b3Observation& observation)
{
	m_observations.PushBack(observation);
}

inline u32 b3ClusterSolver::GetIterations() const
{
	return m_iterations;
}

inline const b3Array<b3Observation>& b3ClusterSolver::GetObservations() const
{
	return m_observations;
}

inline const b3Array<b3Cluster>& b3ClusterSolver::GetClusters() const
{
	return m_clusters;
}

#endif