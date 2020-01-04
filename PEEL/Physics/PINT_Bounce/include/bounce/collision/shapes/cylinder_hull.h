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

#ifndef B3_CYLINDER_HULL_H
#define B3_CYLINDER_HULL_H

#include <bounce/collision/shapes/hull.h>

// A cylinder with 20 segments.
struct b3CylinderHull : public b3Hull
{
	b3Vec3 cylinderVertices[40];
	b3HalfEdge cylinderEdges[120];
	b3Face cylinderFaces[22];
	b3Plane cylinderPlanes[22];

	// Does nothing for performance.
	b3CylinderHull()
	{

	}

	// Construct this cylinder from radius and y extent centered at the origin.
	b3CylinderHull(scalar radius, scalar ey)
	{
		SetExtents(radius, ey);
	}

	// Set this cylinder to the unit cylinder centered at the origin.
	void SetIdentity()
	{
		cylinderVertices[0] = b3Vec3(0.80901741981506347656, 1, 0.58778494596481323242);
		cylinderVertices[1] = b3Vec3(-0.80901724100112915039, -1, -0.58778500556945800781);
		cylinderVertices[2] = b3Vec3(-0.30901724100112915039, 1, -0.95105648040771484375);
		cylinderVertices[3] = b3Vec3(0.30901747941970825195, 1, 0.95105648040771484375);
		cylinderVertices[4] = b3Vec3(0.95105648040771484375, 1, -0.30901703238487243652);
		cylinderVertices[5] = b3Vec3(-0.95105648040771484375, -1, 0.30901741981506347656);
		cylinderVertices[6] = b3Vec3(0.58778512477874755859, -1, -0.80901706218719482422);
		cylinderVertices[7] = b3Vec3(0.95105683803558349609, -1, 0.30901655554771423340);
		cylinderVertices[8] = b3Vec3(-0.58778500556945800781, 1, 0.80901730060577392578);
		cylinderVertices[9] = b3Vec3(0.30901747941970825195, -1, 0.95105648040771484375);
		cylinderVertices[10] = b3Vec3(-0.30901724100112915039, -1, -0.95105648040771484375);
		cylinderVertices[11] = b3Vec3(-0.95105648040771484375, 1, 0.30901741981506347656);
		cylinderVertices[12] = b3Vec3(0.95105648040771484375, -1, -0.30901703238487243652);
		cylinderVertices[13] = b3Vec3(0.58778512477874755859, 1, -0.80901706218719482422);
		cylinderVertices[14] = b3Vec3(-0.80901724100112915039, 1, -0.58778500556945800781);
		cylinderVertices[15] = b3Vec3(-0.58778500556945800781, -1, 0.80901730060577392578);
		cylinderVertices[16] = b3Vec3(-0.30901664495468139648, -1, 0.95105671882629394531);
		cylinderVertices[17] = b3Vec3(0.95105683803558349609, 1, 0.30901655554771423340);
		cylinderVertices[18] = b3Vec3(-0.95105665922164916992, 1, -0.30901667475700378418);
		cylinderVertices[19] = b3Vec3(0.30901682376861572266, 1, -0.95105654001235961914);
		cylinderVertices[20] = b3Vec3(-0.95105665922164916992, -1, -0.30901667475700378418);
		cylinderVertices[21] = b3Vec3(0.30901682376861572266, -1, -0.95105654001235961914);
		cylinderVertices[22] = b3Vec3(0.80901741981506347656, -1, 0.58778494596481323242);
		cylinderVertices[23] = b3Vec3(-0.30901664495468139648, 1, 0.95105671882629394531);
		cylinderVertices[24] = b3Vec3(-0.80901682376861572266, 1, 0.58778566122055053711);
		cylinderVertices[25] = b3Vec3(0.80901694297790527344, -1, -0.58778530359268188477);
		cylinderVertices[26] = b3Vec3(-0.58778554201126098633, -1, -0.80901688337326049805);
		cylinderVertices[27] = b3Vec3(0.58778578042984008789, -1, 0.80901682376861572266);
		cylinderVertices[28] = b3Vec3(0.58778578042984008789, 1, 0.80901682376861572266);
		cylinderVertices[29] = b3Vec3(-0.80901682376861572266, -1, 0.58778566122055053711);
		cylinderVertices[30] = b3Vec3(0.80901694297790527344, 1, -0.58778530359268188477);
		cylinderVertices[31] = b3Vec3(-0.58778554201126098633, 1, -0.80901688337326049805);
		cylinderVertices[32] = b3Vec3(1, 1, 0);
		cylinderVertices[33] = b3Vec3(0.00000044982880353928, -1, 1.00000011920928955078);
		cylinderVertices[34] = b3Vec3(-1.00000011920928955078, 1, 0.00000039115548133850);
		cylinderVertices[35] = b3Vec3(-0.00000022072345018387, -1, -1);
		cylinderVertices[36] = b3Vec3(0.00000044982880353928, 1, 1.00000011920928955078);
		cylinderVertices[37] = b3Vec3(-0.00000022072345018387, 1, -1);
		cylinderVertices[38] = b3Vec3(1, -1, 0);
		cylinderVertices[39] = b3Vec3(-1.00000011920928955078, -1, 0.00000039115548133850);

		cylinderEdges[0] = b3MakeEdge(0, 1, 0, 6, 2);
		cylinderEdges[1] = b3MakeEdge(22, 0, 1, 12, 8);
		cylinderEdges[2] = b3MakeEdge(22, 3, 0, 0, 4);
		cylinderEdges[3] = b3MakeEdge(7, 2, 4, 72, 13);
		cylinderEdges[4] = b3MakeEdge(7, 5, 0, 2, 6);
		cylinderEdges[5] = b3MakeEdge(17, 4, 16, 15, 73);
		cylinderEdges[6] = b3MakeEdge(17, 7, 0, 4, 0);
		cylinderEdges[7] = b3MakeEdge(0, 6, 2, 9, 14);
		cylinderEdges[8] = b3MakeEdge(0, 9, 1, 1, 10);
		cylinderEdges[9] = b3MakeEdge(28, 8, 2, 48, 7);
		cylinderEdges[10] = b3MakeEdge(28, 11, 1, 8, 12);
		cylinderEdges[11] = b3MakeEdge(27, 10, 9, 75, 49);
		cylinderEdges[12] = b3MakeEdge(27, 13, 1, 10, 1);
		cylinderEdges[13] = b3MakeEdge(22, 12, 4, 3, 74);
		cylinderEdges[14] = b3MakeEdge(17, 15, 2, 7, 16);
		cylinderEdges[15] = b3MakeEdge(32, 14, 16, 101, 5);
		cylinderEdges[16] = b3MakeEdge(32, 17, 2, 14, 18);
		cylinderEdges[17] = b3MakeEdge(4, 16, 10, 102, 100);
		cylinderEdges[18] = b3MakeEdge(4, 19, 2, 16, 20);
		cylinderEdges[19] = b3MakeEdge(30, 18, 11, 104, 103);
		cylinderEdges[20] = b3MakeEdge(30, 21, 2, 18, 22);
		cylinderEdges[21] = b3MakeEdge(13, 20, 15, 115, 105);
		cylinderEdges[22] = b3MakeEdge(13, 23, 2, 20, 24);
		cylinderEdges[23] = b3MakeEdge(19, 22, 14, 112, 114);
		cylinderEdges[24] = b3MakeEdge(19, 25, 2, 22, 26);
		cylinderEdges[25] = b3MakeEdge(37, 24, 21, 93, 113);
		cylinderEdges[26] = b3MakeEdge(37, 27, 2, 24, 28);
		cylinderEdges[27] = b3MakeEdge(2, 26, 6, 94, 92);
		cylinderEdges[28] = b3MakeEdge(2, 29, 2, 26, 30);
		cylinderEdges[29] = b3MakeEdge(31, 28, 7, 91, 95);
		cylinderEdges[30] = b3MakeEdge(31, 31, 2, 28, 32);
		cylinderEdges[31] = b3MakeEdge(14, 30, 5, 55, 90);
		cylinderEdges[32] = b3MakeEdge(14, 33, 2, 30, 34);
		cylinderEdges[33] = b3MakeEdge(18, 32, 3, 52, 54);
		cylinderEdges[34] = b3MakeEdge(18, 35, 2, 32, 36);
		cylinderEdges[35] = b3MakeEdge(34, 34, 20, 111, 53);
		cylinderEdges[36] = b3MakeEdge(34, 37, 2, 34, 38);
		cylinderEdges[37] = b3MakeEdge(11, 36, 13, 109, 110);
		cylinderEdges[38] = b3MakeEdge(11, 39, 2, 36, 40);
		cylinderEdges[39] = b3MakeEdge(24, 38, 12, 106, 108);
		cylinderEdges[40] = b3MakeEdge(24, 41, 2, 38, 42);
		cylinderEdges[41] = b3MakeEdge(8, 40, 17, 116, 107);
		cylinderEdges[42] = b3MakeEdge(8, 43, 2, 40, 44);
		cylinderEdges[43] = b3MakeEdge(23, 42, 18, 118, 117);
		cylinderEdges[44] = b3MakeEdge(23, 45, 2, 42, 46);
		cylinderEdges[45] = b3MakeEdge(36, 44, 19, 97, 119);
		cylinderEdges[46] = b3MakeEdge(36, 47, 2, 44, 48);
		cylinderEdges[47] = b3MakeEdge(3, 46, 8, 98, 96);
		cylinderEdges[48] = b3MakeEdge(3, 49, 2, 46, 9);
		cylinderEdges[49] = b3MakeEdge(28, 48, 9, 11, 99);
		cylinderEdges[50] = b3MakeEdge(1, 51, 3, 54, 52);
		cylinderEdges[51] = b3MakeEdge(20, 50, 4, 88, 56);
		cylinderEdges[52] = b3MakeEdge(20, 53, 3, 50, 33);
		cylinderEdges[53] = b3MakeEdge(18, 52, 20, 35, 89);
		cylinderEdges[54] = b3MakeEdge(14, 55, 3, 33, 50);
		cylinderEdges[55] = b3MakeEdge(1, 54, 5, 57, 31);
		cylinderEdges[56] = b3MakeEdge(1, 57, 4, 51, 58);
		cylinderEdges[57] = b3MakeEdge(26, 56, 5, 90, 55);
		cylinderEdges[58] = b3MakeEdge(26, 59, 4, 56, 60);
		cylinderEdges[59] = b3MakeEdge(10, 58, 7, 95, 91);
		cylinderEdges[60] = b3MakeEdge(10, 61, 4, 58, 62);
		cylinderEdges[61] = b3MakeEdge(35, 60, 6, 92, 94);
		cylinderEdges[62] = b3MakeEdge(35, 63, 4, 60, 64);
		cylinderEdges[63] = b3MakeEdge(21, 62, 21, 113, 93);
		cylinderEdges[64] = b3MakeEdge(21, 65, 4, 62, 66);
		cylinderEdges[65] = b3MakeEdge(6, 64, 14, 114, 112);
		cylinderEdges[66] = b3MakeEdge(6, 67, 4, 64, 68);
		cylinderEdges[67] = b3MakeEdge(25, 66, 15, 105, 115);
		cylinderEdges[68] = b3MakeEdge(25, 69, 4, 66, 70);
		cylinderEdges[69] = b3MakeEdge(12, 68, 11, 103, 104);
		cylinderEdges[70] = b3MakeEdge(12, 71, 4, 68, 72);
		cylinderEdges[71] = b3MakeEdge(38, 70, 10, 100, 102);
		cylinderEdges[72] = b3MakeEdge(38, 73, 4, 70, 3);
		cylinderEdges[73] = b3MakeEdge(7, 72, 16, 5, 101);
		cylinderEdges[74] = b3MakeEdge(27, 75, 4, 13, 76);
		cylinderEdges[75] = b3MakeEdge(9, 74, 9, 99, 11);
		cylinderEdges[76] = b3MakeEdge(9, 77, 4, 74, 78);
		cylinderEdges[77] = b3MakeEdge(33, 76, 8, 96, 98);
		cylinderEdges[78] = b3MakeEdge(33, 79, 4, 76, 80);
		cylinderEdges[79] = b3MakeEdge(16, 78, 19, 119, 97);
		cylinderEdges[80] = b3MakeEdge(16, 81, 4, 78, 82);
		cylinderEdges[81] = b3MakeEdge(15, 80, 18, 117, 118);
		cylinderEdges[82] = b3MakeEdge(15, 83, 4, 80, 84);
		cylinderEdges[83] = b3MakeEdge(29, 82, 17, 107, 116);
		cylinderEdges[84] = b3MakeEdge(29, 85, 4, 82, 86);
		cylinderEdges[85] = b3MakeEdge(5, 84, 12, 108, 106);
		cylinderEdges[86] = b3MakeEdge(5, 87, 4, 84, 88);
		cylinderEdges[87] = b3MakeEdge(39, 86, 13, 110, 109);
		cylinderEdges[88] = b3MakeEdge(39, 89, 4, 86, 51);
		cylinderEdges[89] = b3MakeEdge(20, 88, 20, 53, 111);
		cylinderEdges[90] = b3MakeEdge(31, 91, 5, 31, 57);
		cylinderEdges[91] = b3MakeEdge(26, 90, 7, 59, 29);
		cylinderEdges[92] = b3MakeEdge(37, 93, 6, 27, 61);
		cylinderEdges[93] = b3MakeEdge(35, 92, 21, 63, 25);
		cylinderEdges[94] = b3MakeEdge(10, 95, 6, 61, 27);
		cylinderEdges[95] = b3MakeEdge(2, 94, 7, 29, 59);
		cylinderEdges[96] = b3MakeEdge(36, 97, 8, 47, 77);
		cylinderEdges[97] = b3MakeEdge(33, 96, 19, 79, 45);
		cylinderEdges[98] = b3MakeEdge(9, 99, 8, 77, 47);
		cylinderEdges[99] = b3MakeEdge(3, 98, 9, 49, 75);
		cylinderEdges[100] = b3MakeEdge(32, 101, 10, 17, 71);
		cylinderEdges[101] = b3MakeEdge(38, 100, 16, 73, 15);
		cylinderEdges[102] = b3MakeEdge(12, 103, 10, 71, 17);
		cylinderEdges[103] = b3MakeEdge(4, 102, 11, 19, 69);
		cylinderEdges[104] = b3MakeEdge(25, 105, 11, 69, 19);
		cylinderEdges[105] = b3MakeEdge(30, 104, 15, 21, 67);
		cylinderEdges[106] = b3MakeEdge(29, 107, 12, 85, 39);
		cylinderEdges[107] = b3MakeEdge(24, 106, 17, 41, 83);
		cylinderEdges[108] = b3MakeEdge(11, 109, 12, 39, 85);
		cylinderEdges[109] = b3MakeEdge(5, 108, 13, 87, 37);
		cylinderEdges[110] = b3MakeEdge(34, 111, 13, 37, 87);
		cylinderEdges[111] = b3MakeEdge(39, 110, 20, 89, 35);
		cylinderEdges[112] = b3MakeEdge(21, 113, 14, 65, 23);
		cylinderEdges[113] = b3MakeEdge(19, 112, 21, 25, 63);
		cylinderEdges[114] = b3MakeEdge(13, 115, 14, 23, 65);
		cylinderEdges[115] = b3MakeEdge(6, 114, 15, 67, 21);
		cylinderEdges[116] = b3MakeEdge(15, 117, 17, 83, 41);
		cylinderEdges[117] = b3MakeEdge(8, 116, 18, 43, 81);
		cylinderEdges[118] = b3MakeEdge(16, 119, 18, 81, 43);
		cylinderEdges[119] = b3MakeEdge(23, 118, 19, 45, 79);

		cylinderFaces[0].edge = 0;
		cylinderFaces[1].edge = 8;
		cylinderFaces[2].edge = 7;
		cylinderFaces[3].edge = 50;
		cylinderFaces[4].edge = 56;
		cylinderFaces[5].edge = 55;
		cylinderFaces[6].edge = 27;
		cylinderFaces[7].edge = 95;
		cylinderFaces[8].edge = 47;
		cylinderFaces[9].edge = 99;
		cylinderFaces[10].edge = 17;
		cylinderFaces[11].edge = 103;
		cylinderFaces[12].edge = 85;
		cylinderFaces[13].edge = 109;
		cylinderFaces[14].edge = 65;
		cylinderFaces[15].edge = 115;
		cylinderFaces[16].edge = 73;
		cylinderFaces[17].edge = 41;
		cylinderFaces[18].edge = 117;
		cylinderFaces[19].edge = 79;
		cylinderFaces[20].edge = 53;
		cylinderFaces[21].edge = 113;

		cylinderPlanes[0].normal = b3Vec3(0.89100682735443115234, 0, 0.45399010181427001953);
		cylinderPlanes[0].offset = 0.98768866062164306641;
		
		cylinderPlanes[1].normal = b3Vec3(0.70710718631744384766, 0, 0.70710641145706176758);
		cylinderPlanes[1].offset = 0.98768854141235351563;
		
		cylinderPlanes[2].normal = b3Vec3(0, 1, 0);
		cylinderPlanes[2].offset = 1;
		
		cylinderPlanes[3].normal = b3Vec3(-0.89100670814514160156, 0, -0.45399013161659240723);
		cylinderPlanes[3].offset = 0.98768842220306396484;
		
		cylinderPlanes[4].normal = b3Vec3(0, -1, 0);
		cylinderPlanes[4].offset = 1;
		
		cylinderPlanes[5].normal = b3Vec3(-0.70710712671279907227, 0, -0.70710653066635131836);
		cylinderPlanes[5].offset = 0.98768848180770874023;
		
		cylinderPlanes[6].normal = b3Vec3(-0.15643458068370819092, 0, -0.98768836259841918945);
		cylinderPlanes[6].offset = 0.98768842220306396484;
		
		cylinderPlanes[7].normal = b3Vec3(-0.45399063825607299805, 0, -0.89100646972656250000);
		cylinderPlanes[7].offset = 0.98768842220306396484;
		
		cylinderPlanes[8].normal = b3Vec3(0.15643493831157684326, 0, 0.98768830299377441406);
		cylinderPlanes[8].offset = 0.98768848180770874023;
		
		cylinderPlanes[9].normal = b3Vec3(0.45399075746536254883, 0, 0.89100635051727294922);
		cylinderPlanes[9].offset = 0.98768836259841918945;
		
		cylinderPlanes[10].normal = b3Vec3(0.98768830299377441406, 0, -0.15643455088138580322);
		cylinderPlanes[10].offset = 0.98768830299377441406;
		
		cylinderPlanes[11].normal = b3Vec3(0.89100646972656250000, 0, -0.45399051904678344727);
		cylinderPlanes[11].offset = 0.98768830299377441406;
		
		cylinderPlanes[12].normal = b3Vec3(-0.89100635051727294922, 0, 0.45399084687232971191);
		cylinderPlanes[12].offset = 0.98768842220306396484;
		
		cylinderPlanes[13].normal = b3Vec3(-0.98768830299377441406, 0, 0.15643493831157684326);
		cylinderPlanes[13].offset = 0.98768842220306396484;
		
		cylinderPlanes[14].normal = b3Vec3(0.45399031043052673340, 0, -0.89100658893585205078);
		cylinderPlanes[14].offset = 0.98768830299377441406;
		
		cylinderPlanes[15].normal = b3Vec3(0.70710670948028564453, 0, -0.70710688829421997070);
		cylinderPlanes[15].offset = 0.98768830299377441406;
		
		cylinderPlanes[16].normal = b3Vec3(0.98768848180770874023, 0, 0.15643368661403656006);
		cylinderPlanes[16].offset = 0.98768854141235351563;
		
		cylinderPlanes[17].normal = b3Vec3(-0.70710653066635131836, 0, 0.70710712671279907227);
		cylinderPlanes[17].offset = 0.98768854141235351563;
		
		cylinderPlanes[18].normal = b3Vec3(-0.45399010181427001953, 0, 0.89100670814514160156);
		cylinderPlanes[18].offset = 0.98768842220306396484;
		
		cylinderPlanes[19].normal = b3Vec3(-0.15643416345119476318, 0, 0.98768842220306396484);
		cylinderPlanes[19].offset = 0.98768848180770874023;
		
		cylinderPlanes[20].normal = b3Vec3(-0.98768842220306396484, 0, -0.15643437206745147705);
		cylinderPlanes[20].offset = 0.98768848180770874023;
		
		cylinderPlanes[21].normal = b3Vec3(0.15643437206745147705, 0, -0.98768842220306396484);
		cylinderPlanes[21].offset = 0.98768836259841918945;

		centroid = b3Vec3(0, 0, 0);
		vertices = cylinderVertices;
		vertexCount = 40;
		edges = cylinderEdges;
		edgeCount = 120;
		faces = cylinderFaces;
		planes = cylinderPlanes;
		faceCount = 22;

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

extern const b3CylinderHull b3CylinderHull_identity;

#endif