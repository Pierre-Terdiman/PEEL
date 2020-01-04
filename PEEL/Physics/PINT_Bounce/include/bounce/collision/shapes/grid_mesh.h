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

#ifndef B3_GRID_MESH_H
#define B3_GRID_MESH_H

#include <bounce/collision/shapes/mesh.h>

// A (H + 1) x (W + 1) grid mesh stored in row-major order.
// v(i, j) = i * (W + 1) + j
template<u32 H = 1, u32 W = 1>
struct b3GridMesh : public b3Mesh
{
	b3Vec3 gridVertices[(H + 1) * (W + 1)];
	b3MeshTriangle gridTriangles[2 * H * W];

	// Set this grid to a W (width) per H (height) dimensioned grid centered at the origin and aligned
	// with the world x-z axes.
	b3GridMesh()
	{
		vertexCount = 0;
		for (u32 j = 0; j <= W; ++j)
		{
			for (u32 i = 0; i <= H; ++i)
			{
				u32 vertex = GetVertex(i, j);
				gridVertices[vertex].Set(scalar(j), 0, scalar(i));
				++vertexCount;
			}
		}

		B3_ASSERT(vertexCount == (H + 1) * (W + 1));

		b3Vec3 translation;
		translation.x = scalar(-0.5) * scalar(W);
		translation.y = 0;
		translation.z = scalar(-0.5) * scalar(H);

		for (u32 i = 0; i < vertexCount; ++i)
		{
			gridVertices[i] += translation;
		}

		triangleCount = 0;
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				// 1*|----|*4
				//   |----|
				// 2*|----|*3
				u32 v1 = GetVertex(i, j);
				u32 v2 = GetVertex(i + 1, j);
				u32 v3 = GetVertex(i + 1, j + 1);
				u32 v4 = GetVertex(i, j + 1);

				b3MeshTriangle* t1 = gridTriangles + triangleCount++;
				t1->v1 = v1;
				t1->v2 = v2;
				t1->v3 = v3;

				b3MeshTriangle* t2 = gridTriangles + triangleCount++;
				t2->v1 = v3;
				t2->v2 = v4;
				t2->v3 = v1;
			}
		}

		B3_ASSERT(triangleCount == 2 * H * W);

		vertices = gridVertices;
		triangles = gridTriangles;
	}

	u32 GetVertex(u32 i, u32 j)
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};

#endif