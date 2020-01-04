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

#ifndef B3_TRIANGLE_HULL_H
#define B3_TRIANGLE_HULL_H

#include <bounce/collision/shapes/hull.h>

struct b3TriangleHull : public b3Hull
{
	b3Vec3 triangleVertices[3];
	b3HalfEdge triangleEdges[6];
	b3Face triangleFaces[2];
	b3Plane trianglePlanes[2];

	b3TriangleHull() { }

	b3TriangleHull(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
	{
		Set(A, B, C);
	}

	void Set(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
	{
		centroid = (A + B + C) / scalar(3);

		triangleVertices[0] = A;
		triangleVertices[1] = B;
		triangleVertices[2] = C;

		// Each edge must be followed by its twin.
		triangleEdges[0] = b3MakeEdge(0, 1, 0, 4, 2); // Face 0 - Edge 0
		triangleEdges[2] = b3MakeEdge(1, 3, 0, 0, 4); // Face 0 - Edge 1
		triangleEdges[4] = b3MakeEdge(2, 5, 0, 2, 0); // Face 0 - Edge 2
		
		triangleEdges[1] = b3MakeEdge(1, 0, 1, 3, 5); // Face 1 - Edge 0
		triangleEdges[3] = b3MakeEdge(2, 2, 1, 5, 1); // Face 1 - Edge 1
		triangleEdges[5] = b3MakeEdge(0, 4, 1, 1, 3); // Face 1 - Edge 2
				
		triangleFaces[0].edge = 0;
		triangleFaces[1].edge = 1;
		
		b3Vec3 N = b3Cross(B - A, C - A);
		N = b3Normalize(N);

		trianglePlanes[0] = b3Plane(N, centroid);
		trianglePlanes[1] = b3Plane(-N, centroid);
		
		vertices = triangleVertices;
		vertexCount = 3;
		edges = triangleEdges;
		edgeCount = 6;
		faces = triangleFaces;
		planes = trianglePlanes;
		faceCount = 2;
	}
};

#endif