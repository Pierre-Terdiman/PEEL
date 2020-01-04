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

#ifndef B3_CONE_HULL_H
#define B3_CONE_HULL_H

#include <bounce/collision/shapes/hull.h>

// A cone with 20 segments.
struct b3ConeHull : public b3Hull
{
	b3Vec3 coneVertices[21];
	b3HalfEdge coneEdges[80];
	b3Face coneFaces[21];
	b3Plane conePlanes[21];

	// Does nothing for performance.
	b3ConeHull()
	{

	}

	// Construct this cone from radius and y extent centered at the origin.
	// The cone will have radius and extent set to 1;
	b3ConeHull(scalar radius, scalar ey)
	{
		SetExtents(radius, ey);
	}

	// Set this cone to the unit cylinder centered at the origin.
	void SetIdentity()
	{
		coneVertices[0] = b3Vec3(-0.95105665922164916992, -1.00000000000000000000, -0.30901667475700378418);
		coneVertices[1] = b3Vec3(-0.95105648040771484375, -1.00000000000000000000, 0.30901741981506347656);
		coneVertices[2] = b3Vec3(0.30901747941970825195, -1.00000000000000000000, 0.95105648040771484375);
		coneVertices[3] = b3Vec3(0.95105648040771484375, -1.00000000000000000000, -0.30901703238487243652);
		coneVertices[4] = b3Vec3(-0.58778500556945800781, -1.00000000000000000000, 0.80901730060577392578);
		coneVertices[5] = b3Vec3(-0.30901724100112915039, -1.00000000000000000000, -0.95105648040771484375);
		coneVertices[6] = b3Vec3(0.30901682376861572266, -1.00000000000000000000, -0.95105654001235961914);
		coneVertices[7] = b3Vec3(0.80901741981506347656, -1.00000000000000000000, 0.58778494596481323242);
		coneVertices[8] = b3Vec3(-0.80901724100112915039, -1.00000000000000000000, -0.58778500556945800781);
		coneVertices[9] = b3Vec3(0.95105683803558349609, -1.00000000000000000000, 0.30901655554771423340);
		coneVertices[10] = b3Vec3(-0.30901664495468139648, -1.00000000000000000000, 0.95105671882629394531);
		coneVertices[11] = b3Vec3(0.80901694297790527344, -1.00000000000000000000, -0.58778530359268188477);
		coneVertices[12] = b3Vec3(0.58778578042984008789, -1.00000000000000000000, 0.80901682376861572266);
		coneVertices[13] = b3Vec3(0.58778512477874755859, -1.00000000000000000000, -0.80901706218719482422);
		coneVertices[14] = b3Vec3(-0.80901682376861572266, -1.00000000000000000000, 0.58778566122055053711);
		coneVertices[15] = b3Vec3(-0.58778554201126098633, -1.00000000000000000000, -0.80901688337326049805);
		coneVertices[16] = b3Vec3(0.00000044982880353928, -1.00000000000000000000, 1.00000011920928955078);
		coneVertices[17] = b3Vec3(-0.00000022072345018387, -1.00000000000000000000, -1.00000000000000000000);
		coneVertices[18] = b3Vec3(-1.00000011920928955078, -1.00000000000000000000, 0.00000039115548133850);
		coneVertices[19] = b3Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000);
		coneVertices[20] = b3Vec3(1.00000000000000000000, -1.00000000000000000000, 0.00000000000000000000);

		coneEdges[0] = b3MakeEdge(0, 1, 0, 4, 2);
		coneEdges[1] = b3MakeEdge(19, 0, 1, 8, 6);
		coneEdges[2] = b3MakeEdge(19, 3, 0, 0, 4);
		coneEdges[3] = b3MakeEdge(8, 2, 17, 11, 71);
		coneEdges[4] = b3MakeEdge(8, 5, 0, 2, 0);
		coneEdges[5] = b3MakeEdge(0, 4, 2, 7, 10);
		coneEdges[6] = b3MakeEdge(0, 7, 1, 1, 8);
		coneEdges[7] = b3MakeEdge(18, 6, 2, 44, 5);
		coneEdges[8] = b3MakeEdge(18, 9, 1, 6, 1);
		coneEdges[9] = b3MakeEdge(19, 8, 3, 46, 45);
		coneEdges[10] = b3MakeEdge(8, 11, 2, 5, 12);
		coneEdges[11] = b3MakeEdge(15, 10, 17, 71, 3);
		coneEdges[12] = b3MakeEdge(15, 13, 2, 10, 14);
		coneEdges[13] = b3MakeEdge(5, 12, 12, 67, 70);
		coneEdges[14] = b3MakeEdge(5, 15, 2, 12, 16);
		coneEdges[15] = b3MakeEdge(17, 14, 11, 68, 66);
		coneEdges[16] = b3MakeEdge(17, 17, 2, 14, 18);
		coneEdges[17] = b3MakeEdge(6, 16, 14, 73, 69);
		coneEdges[18] = b3MakeEdge(6, 19, 2, 16, 20);
		coneEdges[19] = b3MakeEdge(13, 18, 13, 74, 72);
		coneEdges[20] = b3MakeEdge(13, 21, 2, 18, 22);
		coneEdges[21] = b3MakeEdge(11, 20, 20, 61, 75);
		coneEdges[22] = b3MakeEdge(11, 23, 2, 20, 24);
		coneEdges[23] = b3MakeEdge(3, 22, 8, 57, 60);
		coneEdges[24] = b3MakeEdge(3, 25, 2, 22, 26);
		coneEdges[25] = b3MakeEdge(20, 24, 7, 58, 56);
		coneEdges[26] = b3MakeEdge(20, 27, 2, 24, 28);
		coneEdges[27] = b3MakeEdge(9, 26, 18, 79, 59);
		coneEdges[28] = b3MakeEdge(9, 29, 2, 26, 30);
		coneEdges[29] = b3MakeEdge(7, 28, 16, 77, 78);
		coneEdges[30] = b3MakeEdge(7, 31, 2, 28, 32);
		coneEdges[31] = b3MakeEdge(12, 30, 15, 55, 76);
		coneEdges[32] = b3MakeEdge(12, 33, 2, 30, 34);
		coneEdges[33] = b3MakeEdge(2, 32, 6, 51, 54);
		coneEdges[34] = b3MakeEdge(2, 35, 2, 32, 36);
		coneEdges[35] = b3MakeEdge(16, 34, 5, 52, 50);
		coneEdges[36] = b3MakeEdge(16, 37, 2, 34, 38);
		coneEdges[37] = b3MakeEdge(10, 36, 19, 65, 53);
		coneEdges[38] = b3MakeEdge(10, 39, 2, 36, 40);
		coneEdges[39] = b3MakeEdge(4, 38, 10, 63, 64);
		coneEdges[40] = b3MakeEdge(4, 41, 2, 38, 42);
		coneEdges[41] = b3MakeEdge(14, 40, 9, 49, 62);
		coneEdges[42] = b3MakeEdge(14, 43, 2, 40, 44);
		coneEdges[43] = b3MakeEdge(1, 42, 4, 47, 48);
		coneEdges[44] = b3MakeEdge(1, 45, 2, 42, 7);
		coneEdges[45] = b3MakeEdge(18, 44, 3, 9, 46);
		coneEdges[46] = b3MakeEdge(1, 47, 3, 45, 9);
		coneEdges[47] = b3MakeEdge(19, 46, 4, 48, 43);
		coneEdges[48] = b3MakeEdge(14, 49, 4, 43, 47);
		coneEdges[49] = b3MakeEdge(19, 48, 9, 62, 41);
		coneEdges[50] = b3MakeEdge(2, 51, 5, 35, 52);
		coneEdges[51] = b3MakeEdge(19, 50, 6, 54, 33);
		coneEdges[52] = b3MakeEdge(19, 53, 5, 50, 35);
		coneEdges[53] = b3MakeEdge(16, 52, 19, 37, 65);
		coneEdges[54] = b3MakeEdge(12, 55, 6, 33, 51);
		coneEdges[55] = b3MakeEdge(19, 54, 15, 76, 31);
		coneEdges[56] = b3MakeEdge(3, 57, 7, 25, 58);
		coneEdges[57] = b3MakeEdge(19, 56, 8, 60, 23);
		coneEdges[58] = b3MakeEdge(19, 59, 7, 56, 25);
		coneEdges[59] = b3MakeEdge(20, 58, 18, 27, 79);
		coneEdges[60] = b3MakeEdge(11, 61, 8, 23, 57);
		coneEdges[61] = b3MakeEdge(19, 60, 20, 75, 21);
		coneEdges[62] = b3MakeEdge(4, 63, 9, 41, 49);
		coneEdges[63] = b3MakeEdge(19, 62, 10, 64, 39);
		coneEdges[64] = b3MakeEdge(10, 65, 10, 39, 63);
		coneEdges[65] = b3MakeEdge(19, 64, 19, 53, 37);
		coneEdges[66] = b3MakeEdge(5, 67, 11, 15, 68);
		coneEdges[67] = b3MakeEdge(19, 66, 12, 70, 13);
		coneEdges[68] = b3MakeEdge(19, 69, 11, 66, 15);
		coneEdges[69] = b3MakeEdge(17, 68, 14, 17, 73);
		coneEdges[70] = b3MakeEdge(15, 71, 12, 13, 67);
		coneEdges[71] = b3MakeEdge(19, 70, 17, 3, 11);
		coneEdges[72] = b3MakeEdge(6, 73, 13, 19, 74);
		coneEdges[73] = b3MakeEdge(19, 72, 14, 69, 17);
		coneEdges[74] = b3MakeEdge(19, 75, 13, 72, 19);
		coneEdges[75] = b3MakeEdge(13, 74, 20, 21, 61);
		coneEdges[76] = b3MakeEdge(7, 77, 15, 31, 55);
		coneEdges[77] = b3MakeEdge(19, 76, 16, 78, 29);
		coneEdges[78] = b3MakeEdge(9, 79, 16, 29, 77);
		coneEdges[79] = b3MakeEdge(19, 78, 18, 59, 27);

		coneFaces[0].edge = 0;
		coneFaces[1].edge = 6;
		coneFaces[2].edge = 5;
		coneFaces[3].edge = 46;
		coneFaces[4].edge = 43;
		coneFaces[5].edge = 50;
		coneFaces[6].edge = 33;
		coneFaces[7].edge = 56;
		coneFaces[8].edge = 23;
		coneFaces[9].edge = 62;
		coneFaces[10].edge = 39;
		coneFaces[11].edge = 66;
		coneFaces[12].edge = 13;
		coneFaces[13].edge = 72;
		coneFaces[14].edge = 17;
		coneFaces[15].edge = 76;
		coneFaces[16].edge = 29;
		coneFaces[17].edge = 3;
		coneFaces[18].edge = 27;
		coneFaces[19].edge = 37;
		coneFaces[20].edge = 21;

		conePlanes[0].normal = b3Vec3(-0.79889804124832153320, 0.44279259443283081055, -0.40705847740173339844);
		conePlanes[0].offset = 0.44279259443283081055;
		conePlanes[1].normal = b3Vec3(-0.88558518886566162109, 0.44279265403747558594, -0.14026281237602233887);
		conePlanes[1].offset = 0.44279262423515319824;
		conePlanes[2].normal = b3Vec3(0.00000000000000000000, -1.00000000000000000000, 0.00000000000000000000);
		conePlanes[2].offset = 1.00000000000000000000;
		conePlanes[3].normal = b3Vec3(-0.88558501005172729492, 0.44279259443283081055, 0.14026330411434173584);
		conePlanes[3].offset = 0.44279262423515319824;
		conePlanes[4].normal = b3Vec3(-0.79889774322509765625, 0.44279259443283081055, 0.40705913305282592773);
		conePlanes[4].offset = 0.44279265403747558594;
		conePlanes[5].normal = b3Vec3(0.14026331901550292969, 0.44279259443283081055, 0.88558501005172729492);
		conePlanes[5].offset = 0.44279265403747558594;
		conePlanes[6].normal = b3Vec3(0.40705907344818115234, 0.44279259443283081055, 0.79889774322509765625);
		conePlanes[6].offset = 0.44279265403747558594;
		conePlanes[7].normal = b3Vec3(0.88558512926101684570, 0.44279256463050842285, -0.14026297628879547119);
		conePlanes[7].offset = 0.44279259443283081055;
		conePlanes[8].normal = b3Vec3(0.79889786243438720703, 0.44279253482818603516, -0.40705886483192443848);
		conePlanes[8].offset = 0.44279256463050842285;
		conePlanes[9].normal = b3Vec3(-0.63400870561599731445, 0.44279262423515319824, 0.63400918245315551758);
		conePlanes[9].offset = 0.44279256463050842285;
		conePlanes[10].normal = b3Vec3(-0.40705844759941101074, 0.44279259443283081055, 0.79889804124832153320);
		conePlanes[10].offset = 0.44279265403747558594;
		conePlanes[11].normal = b3Vec3(-0.14026300609111785889, 0.44279259443283081055, -0.88558506965637207031);
		conePlanes[11].offset = 0.44279253482818603516;
		conePlanes[12].normal = b3Vec3(-0.40705892443656921387, 0.44279259443283081055, -0.79889780282974243164);
		conePlanes[12].offset = 0.44279259443283081055;
		conePlanes[13].normal = b3Vec3(0.40705865621566772461, 0.44279253482818603516, -0.79889798164367675781);
		conePlanes[13].offset = 0.44279259443283081055;
		conePlanes[14].normal = b3Vec3(0.14026281237602233887, 0.44279256463050842285, -0.88558518886566162109);
		conePlanes[14].offset = 0.44279259443283081055;
		conePlanes[15].normal = b3Vec3(0.63400930166244506836, 0.44279262423515319824, 0.63400864601135253906);
		conePlanes[15].offset = 0.44279265403747558594;
		conePlanes[16].normal = b3Vec3(0.79889810085296630859, 0.44279265403747558594, 0.40705841779708862305);
		conePlanes[16].offset = 0.44279265403747558594;
		conePlanes[17].normal = b3Vec3(-0.63400924205780029297, 0.44279265403747558594, -0.63400876522064208984);
		conePlanes[17].offset = 0.44279265403747558594;
		conePlanes[18].normal = b3Vec3(0.88558530807495117188, 0.44279265403747558594, 0.14026220142841339111);
		conePlanes[18].offset = 0.44279268383979797363;
		conePlanes[19].normal = b3Vec3(-0.14026261866092681885, 0.44279259443283081055, 0.88558512926101684570);
		conePlanes[19].offset = 0.44279259443283081055;
		conePlanes[20].normal = b3Vec3(0.63400888442993164063, 0.44279256463050842285, -0.63400906324386596680);
		conePlanes[20].offset = 0.44279259443283081055;
		
		centroid = b3Vec3(-0.00000003081002830641, -0.50000011920928955078, 0.00000003992893837790);
		vertices = coneVertices;
		vertexCount = 21;
		edges = coneEdges;
		edgeCount = 80;
		faces = coneFaces;
		planes = conePlanes;
		faceCount = 21;

		Validate();
	}

	// Construct this cylinder from radius and y extent centered at the origin.
	void SetExtents(scalar radius, scalar ey)
	{
		SetIdentity();

		b3Vec3 scale(radius, ey, radius);
		Scale(scale);
	}
};

extern const b3ConeHull b3ConeHull_identity;

#endif