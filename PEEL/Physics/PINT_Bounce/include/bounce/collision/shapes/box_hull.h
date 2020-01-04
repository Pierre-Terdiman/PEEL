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

#ifndef B3_BOX_HULL_H
#define B3_BOX_HULL_H

#include <bounce/collision/shapes/hull.h>

struct b3BoxHull : public b3Hull
{
	b3Vec3 boxVertices[8];
	b3HalfEdge boxEdges[24];
	b3Face boxFaces[6];
	b3Plane boxPlanes[6];

	// Does nothing for performance.
	b3BoxHull() { }

	// Construct this box from three extents and centered at the origin.
	b3BoxHull(scalar ex, scalar ey, scalar ez)
	{
		SetExtents(ex, ey, ez);
	}

	// Set this box to the unit box centered at the origin.
	void SetIdentity()
	{
		boxVertices[0] = b3Vec3(-1, -1, 1);
		boxVertices[1] = b3Vec3(1, -1, -1);
		boxVertices[2] = b3Vec3(1, 1, 1);
		boxVertices[3] = b3Vec3(-1, 1, -1);
		boxVertices[4] = b3Vec3(1, 1, -1);
		boxVertices[5] = b3Vec3(-1, 1, 1);
		boxVertices[6] = b3Vec3(1, -1, 1);
		boxVertices[7] = b3Vec3(-1, -1, -1);

		boxEdges[0] = b3MakeEdge(0, 1, 0, 6, 2);
		boxEdges[1] = b3MakeEdge(5, 0, 1, 12, 8);
		boxEdges[2] = b3MakeEdge(5, 3, 0, 0, 4);
		boxEdges[3] = b3MakeEdge(3, 2, 5, 19, 13);
		boxEdges[4] = b3MakeEdge(3, 5, 0, 2, 6);
		boxEdges[5] = b3MakeEdge(7, 4, 3, 15, 18);
		boxEdges[6] = b3MakeEdge(7, 7, 0, 4, 0);
		boxEdges[7] = b3MakeEdge(0, 6, 2, 9, 14);
		boxEdges[8] = b3MakeEdge(0, 9, 1, 1, 10);
		boxEdges[9] = b3MakeEdge(6, 8, 2, 16, 7);
		boxEdges[10] = b3MakeEdge(6, 11, 1, 8, 12);
		boxEdges[11] = b3MakeEdge(2, 10, 4, 22, 17);
		boxEdges[12] = b3MakeEdge(2, 13, 1, 10, 1);
		boxEdges[13] = b3MakeEdge(5, 12, 5, 3, 23);
		boxEdges[14] = b3MakeEdge(7, 15, 2, 7, 16);
		boxEdges[15] = b3MakeEdge(1, 14, 3, 20, 5);
		boxEdges[16] = b3MakeEdge(1, 17, 2, 14, 9);
		boxEdges[17] = b3MakeEdge(6, 16, 4, 11, 21);
		boxEdges[18] = b3MakeEdge(3, 19, 3, 5, 20);
		boxEdges[19] = b3MakeEdge(4, 18, 5, 23, 3);
		boxEdges[20] = b3MakeEdge(4, 21, 3, 18, 15);
		boxEdges[21] = b3MakeEdge(1, 20, 4, 17, 22);
		boxEdges[22] = b3MakeEdge(4, 23, 4, 21, 11);
		boxEdges[23] = b3MakeEdge(2, 22, 5, 13, 19);

		boxFaces[0].edge = 0;
		boxFaces[1].edge = 8;
		boxFaces[2].edge = 7;
		boxFaces[3].edge = 15;
		boxFaces[4].edge = 21;
		boxFaces[5].edge = 23;

		boxPlanes[0].normal = b3Vec3(-1, 0, 0);
		boxPlanes[0].offset = 1;
		
		boxPlanes[1].normal = b3Vec3(0, 0, 1);
		boxPlanes[1].offset = 1;
		
		boxPlanes[2].normal = b3Vec3(0, -1, 0);
		boxPlanes[2].offset = 1;
		
		boxPlanes[3].normal = b3Vec3(0, 0, -1);
		boxPlanes[3].offset = 1;
		
		boxPlanes[4].normal = b3Vec3(1, 0, 0);
		boxPlanes[4].offset = 1;
		
		boxPlanes[5].normal = b3Vec3(0, 1, 0);
		boxPlanes[5].offset = 1;

		centroid = b3Vec3(0, 0, 0);
		vertices = boxVertices;
		vertexCount = 8;
		edges = boxEdges;
		edgeCount = 24;
		faces = boxFaces;
		planes = boxPlanes;
		faceCount = 6;

		Validate();
	}

	// Set this box from three extents and centered and aligned at the origin.
	void SetExtents(scalar ex, scalar ey, scalar ez)
	{
		SetIdentity();

		b3Vec3 scale(ex, ey, ez);
		Scale(scale);
	}
};

extern const b3BoxHull b3BoxHull_identity;

#endif