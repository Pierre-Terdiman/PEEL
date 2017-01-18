///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "TestScenesHelpers.h"
#include "Render.h"
#include "Random.h"
#include "MyConvex.h"
#include "PintObjectsManager.h"
#include "TestScenes.h"
#include "ProgressBar.h"
#include "Cylinder.h"

static bool gRotateMeshes = false;

///////////////////////////////////////////////////////////////////////////////

const char* GetFile(const char* filename, udword& size)
{
	const char* Tmp0 = _F("../build/%s", filename);
	size = GetFileSize(Tmp0);
	if(size)
		return Tmp0;

	const char* Tmp1 = _F("./%s", filename);
	size = GetFileSize(Tmp1);
	if(size)
		return Tmp1;

	return null;
}

///////////////////////////////////////////////////////////////////////////////

void CreateSingleTriangleMesh(SurfaceManager& test, float scale, Triangle* tri, bool reverse_winding)
{
	ASSERT(!test.GetNbSurfaces());

	const udword NbFaces = 1;
	const udword NbVerts = 3;

	IndexedSurface* IS = test.CreateManagedSurface();
	bool Status = IS->Init(NbFaces, NbVerts);
	ASSERT(Status);

	Point* Verts = IS->GetVerts();
	Verts[0] = Point(0.0f, 0.0f, 0.0f);
	Verts[1] = Point(scale, 0.0f, 0.0f);
	Verts[2] = Point(scale, 0.0f, scale);
	const Point Center = (Verts[0]+Verts[1]+Verts[2])/3.0f;
	Verts[0] -= Center;
	Verts[1] -= Center;
	Verts[2] -= Center;
	
	if(tri)
	{
		tri->mVerts[0] =  Verts[0];
		tri->mVerts[1] =  Verts[1];
		tri->mVerts[2] =  Verts[2];
	}

	const IndexedTriangle Indices(0, reverse_winding ? 1 : 2, reverse_winding ? 2 : 1);
	IndexedTriangle* F = IS->GetFaces();
	*F = Indices;
}

///////////////////////////////////////////////////////////////////////////////

bool CreateBoxStack(Pint& pint, const PintCaps& caps, const udword nb_stacks, udword nb_base_boxes)
{
	if(!caps.mSupportRigidBodySimulation)
		return false;

	const float BoxExtent = 1.0f;

	PINT_BOX_CREATE BoxDesc(BoxExtent, BoxExtent, BoxExtent);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

	const udword NbStacks = nb_stacks;
	for(udword j=0;j<NbStacks;j++)
	{
		udword NbBoxes = nb_base_boxes;
		float BoxPosY = BoxExtent;
		while(NbBoxes)
		{
			for(udword i=0;i<NbBoxes;i++)
			{
		//		const float Coeff = float(i)/float(NbBoxes-1);
				const float Coeff = float(i) - float(NbBoxes)*0.5f;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes		= &BoxDesc;
		//		ObjectDesc.mPosition.x	= (Coeff-0.5f) * BoxExtent * 2.0f;
				ObjectDesc.mPosition.x	= Coeff * BoxExtent * 2.0f;
				ObjectDesc.mPosition.y	= BoxPosY;
				ObjectDesc.mPosition.z	= float(j) * BoxExtent * 4.0f;
				ObjectDesc.mMass		= 1.0f;
				PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
				ASSERT(Handle);
			}

			NbBoxes--;
			BoxPosY += BoxExtent*2.0f;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

void CreateBoxContainer(Pint& pint, float box_height, float box_side, float box_depth)
{
	PINT_BOX_CREATE BoxDesc0(box_side, box_height, box_depth);
	BoxDesc0.mRenderer		= CreateBoxRenderer(BoxDesc0.mExtents);

	PINT_BOX_CREATE BoxDesc1(box_depth, box_height, box_side);
	BoxDesc1.mRenderer		= CreateBoxRenderer(BoxDesc1.mExtents);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mMass		= 0.0f;
	ObjectDesc.mShapes		= &BoxDesc0;

	ObjectDesc.mPosition	= Point(-box_depth, box_height, 0.0f);
	CreatePintObject(pint, ObjectDesc);
	ObjectDesc.mPosition.x	= box_depth;
	CreatePintObject(pint, ObjectDesc);

	ObjectDesc.mShapes		= &BoxDesc1;
	ObjectDesc.mPosition	= Point(0.0f, box_height, -box_depth);
	CreatePintObject(pint, ObjectDesc);
	ObjectDesc.mPosition.z	= box_depth;
	CreatePintObject(pint, ObjectDesc);
}

///////////////////////////////////////////////////////////////////////////////

bool CreateSeaOfStaticBoxes(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude)
{
	BasicRandom Rnd(42);
	PINT_BOX_CREATE BoxCreate;
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)/float(nb_y-1)) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)/float(nb_x-1)) - 0.5f);

			Point Random;
			UnitRandomPt(Random, Rnd);
			const Point Extents = Random + Point(1.0f, 1.0f, 1.0f);

			BoxCreate.mExtents	= Extents;
			BoxCreate.mRenderer	= CreateBoxRenderer(Extents);

			UnitRandomPt(Random, Rnd);
			const Point Center = Random + Point(CoeffX * amplitude, altitude, CoeffY * amplitude);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &BoxCreate;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Center;
			CreatePintObject(pint, ObjectDesc);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool CreateSeaOfStaticSpheres(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude)
{
	BasicRandom Rnd(42);
	SRand(42);
	PINT_SPHERE_CREATE SphereCreate;
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)/float(nb_y-1)) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)/float(nb_x-1)) - 0.5f);

			Point Random;
			UnitRandomPt(Random, Rnd);
			const float Radius = 1.0f + UnitRandomFloat();

			SphereCreate.mRadius	= Radius;
			SphereCreate.mRenderer	= CreateSphereRenderer(Radius);

			UnitRandomPt(Random, Rnd);
			const Point Center = Random + Point(CoeffX * amplitude, altitude, CoeffY * amplitude);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &SphereCreate;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Center;
			CreatePintObject(pint, ObjectDesc);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool CreateSeaOfStaticCapsules(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude)
{
	BasicRandom Rnd(42);
	SRand(42);
	PINT_CAPSULE_CREATE CapsuleCreate;
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)/float(nb_y-1)) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)/float(nb_x-1)) - 0.5f);

			Point Random;
			UnitRandomPt(Random, Rnd);
			const float Radius = 1.0f + UnitRandomFloat();
			const float HalfHeight = 1.0f + UnitRandomFloat();

			CapsuleCreate.mRadius		= Radius;
			CapsuleCreate.mHalfHeight	= HalfHeight;
			CapsuleCreate.mRenderer		= CreateCapsuleRenderer(Radius, HalfHeight);

			UnitRandomPt(Random, Rnd);
			const Point Center = Random + Point(CoeffX * amplitude, altitude, CoeffY * amplitude);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &CapsuleCreate;
			ObjectDesc.mMass		= 0.0f;
			ObjectDesc.mPosition	= Center;
			CreatePintObject(pint, ObjectDesc);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool CreateSeaOfStaticConvexes(Pint& pint, const PintCaps& caps, udword nb_x, udword nb_y, float altitude)
{
	if(!caps.mSupportConvexes)
		return false;

	MyConvex C;
//	udword i=2;	// Small convexes
	udword i=4;	// 'Big' convexes
//	udword i=7;
//	udword i=13;
	C.LoadFile(i);

	PINT_CONVEX_CREATE ConvexCreate(C.mNbVerts, C.mVerts);
	ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

	const float Scale = 3.0f;
	for(udword y=0;y<nb_y;y++)
	{
		for(udword x=0;x<nb_x;x++)
		{
			const float xf = (float(x)-float(nb_x)*0.5f)*Scale;
			const float yf = (float(y)-float(nb_y)*0.5f)*Scale;

			const Point pos = Point(xf, altitude, yf);

			PintObjectHandle Handle = CreateStaticObject(pint, &ConvexCreate, pos);
			ASSERT(Handle);
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

void GenerateCirclePts(udword nb, Point* pts, float scale, float y)
{
	for(udword i=0;i<nb;i++)
	{
		const float Angle = TWOPI*float(i)/float(nb);
		pts[i].x = cosf(Angle)*scale;
		pts[i].y = y;
		pts[i].z = sinf(Angle)*scale;
	}
}

///////////////////////////////////////////////////////////////////////////////

udword GenerateConvex(Point* verts, udword nbInsideCirclePts, udword nbOutsideCirclePts, float scale0, float scale1, float z)
{
	const udword nbVerts = nbInsideCirclePts + nbOutsideCirclePts;
	GenerateCirclePts(nbInsideCirclePts, verts, scale0, 0.0f);
	GenerateCirclePts(nbOutsideCirclePts, verts+nbInsideCirclePts, scale1, z);
	return nbVerts;
}

///////////////////////////////////////////////////////////////////////////////

PintObjectHandle CreateSimpleObject(Pint& pint, const PINT_SHAPE_CREATE* shape, float mass, const Point& pos, const Quat* rot, const Point* linVel, const Point* angVel)
{
	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes			= shape;
	ObjectDesc.mMass			= mass;
	ObjectDesc.mPosition		= pos;
	if(rot)
		ObjectDesc.mRotation	= *rot;
	if(linVel)
		ObjectDesc.mLinearVelocity	= *linVel;
	if(angVel)
		ObjectDesc.mAngularVelocity	= *angVel;
	return CreatePintObject(pint, ObjectDesc);
}

///////////////////////////////////////////////////////////////////////////////

PintObjectHandle CreateDynamicObject(Pint& pint, const PINT_SHAPE_CREATE* shape, const Point& pos, const Quat* rot, const Point* linVel, const Point* angVel)
{
	return CreateSimpleObject(pint, shape, 1.0f, pos, rot, linVel, angVel);
}

///////////////////////////////////////////////////////////////////////////////

PintObjectHandle CreateStaticObject(Pint& pint, const PINT_SHAPE_CREATE* shape, const Point& pos, const Quat* rot)
{
	return CreateSimpleObject(pint, shape, 0.0f, pos, rot);
}

///////////////////////////////////////////////////////////////////////////////

PintObjectHandle CreateDynamicBox(Pint& pint, float size_x, float size_y, float size_z, const Point& pos, const Quat* rot, const PINT_MATERIAL_CREATE* material)
{
	PINT_BOX_CREATE BoxDesc;
	BoxDesc.mMaterial	= material;
	BoxDesc.mExtents	= Point(size_x, size_y, size_z);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

	return CreateDynamicObject(pint, &BoxDesc, pos, rot);
}

///////////////////////////////////////////////////////////////////////////////

bool CreateTestScene_ConvexStack_Generic(Pint& pint, udword NbX, udword NbY, udword NbLayers, udword NbInsideCirclePts, udword NbOutsideCirclePts)
{
	const udword TotalNbVerts = NbInsideCirclePts + NbOutsideCirclePts;
	ASSERT(TotalNbVerts<1024);

	Point Pts[1024];
	udword NbPts = GenerateConvex(Pts, NbInsideCirclePts, NbOutsideCirclePts, 2.0f, 3.0f, 2.0f);
	ASSERT(NbPts==TotalNbVerts);

	PINT_CONVEX_CREATE ConvexCreate(TotalNbVerts, Pts);
	ConvexCreate.mRenderer	= CreateConvexRenderer(TotalNbVerts, Pts);

	for(udword j=0;j<NbLayers;j++)
	{
		const float Scale = 8.0f;
		for(udword y=0;y<NbY;y++)
		{
			for(udword x=0;x<NbX;x++)
			{
				const float xf = (float(x)-float(NbX)*0.5f)*Scale;
				const float yf = (float(y)-float(NbY)*0.5f)*Scale;

				const Point pos = Point(xf, 0.0f + 2.0f * float(j), yf);

				PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
				ASSERT(Handle);
			}
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

void GenerateConvexPile(Pint& pint, udword NbX, udword NbY, udword NbLayers, float Amplitude, udword NbRandomPts)
{
	ASSERT(NbRandomPts<1024);
	Point Pts[1024];
	BasicRandom Rnd(42);
	for(udword i=0;i<NbRandomPts;i++)
	{
		UnitRandomPt(Pts[i], Rnd);
		Pts[i] *= Amplitude;
	}

	PINT_CONVEX_CREATE ConvexCreate(NbRandomPts, Pts);
	ConvexCreate.mRenderer	= CreateConvexRenderer(NbRandomPts, Pts);

	for(udword j=0;j<NbLayers;j++)
	{
		const float Scale = 8.0f;
		for(udword y=0;y<NbY;y++)
		{
			for(udword x=0;x<NbX;x++)
			{
				const float xf = (float(x)-float(NbX)*0.5f)*Scale;
				const float yf = (float(y)-float(NbY)*0.5f)*Scale;

				const Point pos = Point(xf, Amplitude + (Amplitude * 2.0f * float(j)), yf);

				PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, pos);
				ASSERT(Handle);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

bool CreateArrayOfDynamicConvexes(Pint& pint, const PINT_CONVEX_CREATE& convex_create, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point* offset, const Point* lin_vel)
{
	const float OneOverNbX = OneOverNb(nb_x);
	const float OneOverNbY = OneOverNb(nb_y);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_y);
			if(offset)
				Origin += *offset;

			PintObjectHandle Handle = CreateDynamicObject(pint, &convex_create, Origin, null, lin_vel);
			ASSERT(Handle);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool GenerateArrayOfShapes(Pint& pint, const PINT_SHAPE_CREATE* shape, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass, PintCollisionGroup group, const Point* offset)
{
	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes			= shape;
	ObjectDesc.mMass			= mass;
	ObjectDesc.mCollisionGroup	= group;

	const float OneOverNbX = OneOverNb(nb_x);
	const float OneOverNbY = OneOverNb(nb_y);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			Point Origin(CoeffX * scale_x, altitude, CoeffY * scale_z);
			if(offset)
				Origin += *offset;

			ObjectDesc.mPosition	= Origin;
			CreatePintObject(pint, ObjectDesc);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool GenerateArrayOfSpheres(Pint& pint, float radius, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass, PintCollisionGroup group, const Point* offset)
{
	PINT_SPHERE_CREATE SphereDesc(radius);
	SphereDesc.mRenderer	= CreateSphereRenderer(radius);

	return GenerateArrayOfShapes(pint, &SphereDesc, nb_x, nb_y, altitude, scale_x, scale_z, mass, group, offset);
}

///////////////////////////////////////////////////////////////////////////////

bool GenerateArrayOfBoxes(Pint& pint, const Point& extents, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass, PintCollisionGroup group, const Point* offset)
{
	PINT_BOX_CREATE BoxDesc(extents);
	BoxDesc.mRenderer	= CreateBoxRenderer(extents);

	return GenerateArrayOfShapes(pint, &BoxDesc, nb_x, nb_y, altitude, scale_x, scale_z, mass, group, offset);
}

///////////////////////////////////////////////////////////////////////////////

bool GenerateArrayOfCapsules(Pint& pint, float radius, float half_height, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass, PintCollisionGroup group, const Point* offset)
{
	PINT_CAPSULE_CREATE CapsuleDesc(radius, half_height);
	CapsuleDesc.mRenderer	= CreateCapsuleRenderer(radius, half_height*2.0f);

	return GenerateArrayOfShapes(pint, &CapsuleDesc, nb_x, nb_y, altitude, scale_x, scale_z, mass, group, offset);
}

///////////////////////////////////////////////////////////////////////////////

bool GenerateArrayOfConvexes(Pint& pint, const PintCaps& caps, bool is_static, float scale, udword nb_x, udword nb_y, udword convex_id)
{
	if(!caps.mSupportConvexes)
		return false;

	MyConvex C;
	const udword i=convex_id;
//	udword i=2;	// Small convexes
//	udword i=4;	// 'Big' convexes
//	udword i=7;
//	udword i=13;
	C.LoadFile(i);

	PINT_CONVEX_CREATE ConvexCreate(C.mNbVerts, C.mVerts);
	ConvexCreate.mRenderer	= CreateConvexRenderer(ConvexCreate.mNbVerts, ConvexCreate.mVerts);

	const float AltitudeC = 10.0f;
	const float OneOverNbX = OneOverNb(nb_x);
	const float OneOverNbY = OneOverNb(nb_y);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(CoeffX * scale, AltitudeC, CoeffY * scale);

			if(is_static)
			{
				PintObjectHandle Handle = CreateStaticObject(pint, &ConvexCreate, Origin);
				ASSERT(Handle);
			}
			else
			{
				PintObjectHandle Handle = CreateDynamicObject(pint, &ConvexCreate, Origin);
				ASSERT(Handle);
			}
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool GenerateArrayOfObjects(Pint& pint, const PintCaps& caps, PintShape type, udword type_data, udword nb_x, udword nb_y, float altitude, float scale, float mass)
{
	if(type==PINT_SHAPE_SPHERE)
	{
		const float Radius = 0.5f;
		return GenerateArrayOfSpheres(pint, Radius, nb_x, nb_y, altitude, scale, scale, mass);
	}
	else if(type==PINT_SHAPE_CAPSULE)
	{
		const float Radius = 0.5f;
		const float HalfHeight = 1.0f;
		return GenerateArrayOfCapsules(pint, Radius, HalfHeight, nb_x, nb_y, altitude, scale, scale, mass);
	}
	else if(type==PINT_SHAPE_BOX)
	{
		const float Radius = 0.5f;
		return GenerateArrayOfBoxes(pint, Point(Radius, Radius, Radius), nb_x, nb_y, altitude, scale, scale, mass);
	}
	else if(type==PINT_SHAPE_CONVEX)
	{
		if(!caps.mSupportConvexes)
			return false;
		return GenerateArrayOfConvexes(pint, caps, mass==0.0f, scale, nb_x, nb_y, type_data);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Dominos_Setup(Pint& pint, const PintCaps& caps, float friction)
{
	if(!caps.mSupportRigidBodySimulation)
		return false;

	const Point Extents(0.1f, 0.5f, 1.0f);

	PINT_MATERIAL_CREATE MatDesc;
	MatDesc.mStaticFriction		= 0.0f;
	MatDesc.mDynamicFriction	= friction;
	MatDesc.mRestitution		= 0.0f;

	PINT_BOX_CREATE BoxDesc(Extents);
	BoxDesc.mRenderer	= CreateBoxRenderer(Extents);
	BoxDesc.mMaterial	= &MatDesc;

	const udword Nb = 128;
	Point Pts[Nb];
	GenerateCirclePts(Nb, Pts, 10.0f, 0.0f);

	for(udword i=0;i<Nb;i++)
	{
		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= Pts[i] + Point(0.0f, Extents.z, 0.0f);

		const Point Dir = (Pts[(i+1)%Nb] - Pts[i]).Normalize();
		Point Right, Up;
		ComputeBasis(Dir, Right, Up);

		Matrix3x3 Rot;
		Rot[0] = Dir;
		Rot[1] = Right;
		Rot[2] = Up;
		ObjectDesc.mRotation = Rot;

		if(i==0)
		{
			ObjectDesc.mAngularVelocity = Point(10.0f, 0.0f, 0.0f);
		}

		CreatePintObject(pint, ObjectDesc);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Setup_PotPourri_Raycasts(Pint& pint, const PintCaps& caps, float mass, udword nb_layers, udword nb_x, udword nb_y)
{
	if(!caps.mSupportRaycasts)
		return false;

	const float BoxHeight = 4.0f;
	const float BoxSide = 1.0f;
	const float BoxDepth = 10.0f;

	const float SphereRadius = 0.5f;
	const float CapsuleRadius = 0.3f;
	const float HalfHeight = 0.5f;
	float yy = CapsuleRadius;
	BasicRandom Rnd(42);

	PINT_SPHERE_CREATE SphereDesc(SphereRadius);
	SphereDesc.mRenderer	= CreateSphereRenderer(SphereRadius);

	PINT_BOX_CREATE BoxDesc(CapsuleRadius, CapsuleRadius, CapsuleRadius);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

	PINT_CAPSULE_CREATE CapsuleDesc(CapsuleRadius, HalfHeight);
	CapsuleDesc.mRenderer	= CreateCapsuleRenderer(CapsuleRadius, HalfHeight*2.0f);

	for(udword k=0;k<nb_layers;k++)
	{
		for(udword y=0;y<nb_y;y++)
		{
			const float CoeffY = float(nb_y) * 0.5f * ((float(y)/float(nb_y-1)) - 0.5f);
			for(udword x=0;x<nb_x;x++)
			{
				const float CoeffX = float(nb_x) * 0.5f * ((float(x)/float(nb_x-1)) - 0.5f);

				const float RandomX = 4.0f * Rnd.RandomFloat();
				const float RandomY = 4.0f * Rnd.RandomFloat();
				const float RandomZ = 4.0f * Rnd.RandomFloat();

				const udword Index = Rnd.Randomize() % 3;

				PINT_OBJECT_CREATE ObjectDesc;
				if(Index==0)
					ObjectDesc.mShapes	= &SphereDesc;
				else if(Index==1)
					ObjectDesc.mShapes	= &BoxDesc;
				else if(Index==2)
					ObjectDesc.mShapes	= &CapsuleDesc;
				ObjectDesc.mMass		= mass;
				ObjectDesc.mPosition.x	= RandomX + CoeffX * (BoxDepth - SphereRadius - BoxSide*2.0f);
				ObjectDesc.mPosition.y	= RandomY + yy;
				ObjectDesc.mPosition.z	= RandomZ + CoeffY * (BoxDepth - SphereRadius - BoxSide*2.0f);

				UnitRandomQuat(ObjectDesc.mRotation, Rnd);

				CreatePintObject(pint, ObjectDesc);
			}
		}
		yy += HalfHeight*2.0f*4.0f;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Setup_PotPourri_Raycasts(TestBase& test, udword nb_rays, float max_dist)
{
	test.mCreateDefaultEnvironment = false;

	BasicRandom Rnd(42);
	for(udword i=0;i<nb_rays;i++)
	{
		Point Origin;
		UnitRandomPt(Origin, Rnd);
		Origin *= 30.0f;
		Origin.y += 30.0f;

		Point Dir;
		UnitRandomPt(Dir, Rnd);
		test.RegisterRaycast(Origin, Dir, max_dist);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static float gMaxEdgeLength2 = 5.0f * 5.0f;
//static float gMaxEdgeLength2 = 10.0f * 10.0f;
//static float gMaxEdgeLength2 = 50.0f * 50.0f;
//static float gMaxEdgeLength2 = 100.0f * 100.0f;
static void TessellateTriangle(udword& nb_new_tris, const Triangle& tr, Triangles& triangles)
{
	udword code;
	{
		const Point edge0 = tr.mVerts[0] - tr.mVerts[1];
		const Point edge1 = tr.mVerts[1] - tr.mVerts[2];
		const Point edge2 = tr.mVerts[2] - tr.mVerts[0];
		const bool split0 = edge0.SquareMagnitude()>gMaxEdgeLength2;
		const bool split1 = edge1.SquareMagnitude()>gMaxEdgeLength2;
		const bool split2 = edge2.SquareMagnitude()>gMaxEdgeLength2;
		code = (udword(split2)<<2)|(udword(split1)<<1)|udword(split0);
	}

	const Point m0 = (tr.mVerts[0] + tr.mVerts[1])*0.5f;
	const Point m1 = (tr.mVerts[1] + tr.mVerts[2])*0.5f;
	const Point m2 = (tr.mVerts[2] + tr.mVerts[0])*0.5f;

	switch(code)
	{
		case 0:     // 000: no split
		{
			triangles.AddTri(tr);
			nb_new_tris += 1;
		}
		break;
		case 1:     // 001: split edge0
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m0, tr.mVerts[2]),	triangles);
			TessellateTriangle(nb_new_tris, Triangle(m0, tr.mVerts[1], tr.mVerts[2]),	triangles);
		}
		break;
		case 2:     // 010: split edge1
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], tr.mVerts[1], m1),	triangles);
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m1, tr.mVerts[2]),	triangles);
		}
		break;
		case 3:     // 011: split edge0/edge1
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m0, m1),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m1, tr.mVerts[2]),	triangles);
			TessellateTriangle(nb_new_tris, Triangle(m0, tr.mVerts[1], m1),				triangles);
		}
		break;
		case 4:     // 100: split edge2
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], tr.mVerts[1], m2),	triangles);
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[1], tr.mVerts[2], m2),	triangles);
		}
		break;
		case 5:     // 101: split edge0/edge2
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m0, m2),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m0, tr.mVerts[1], m2),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m2, tr.mVerts[1], tr.mVerts[2]),	triangles);
		}
		break;
		case 6:     // 110: split edge1/edge2
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], tr.mVerts[1], m1),	triangles);
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m1, m2),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m2, m1, tr.mVerts[2]),				triangles);
		}
		break;
		case 7:     // 111: split edge0/edge1/edge2
		{
			TessellateTriangle(nb_new_tris, Triangle(tr.mVerts[0], m0, m2),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m0, tr.mVerts[1], m1),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m2, m1, tr.mVerts[2]),				triangles);
			TessellateTriangle(nb_new_tris, Triangle(m0, m1, m2),						triangles);
		}
		break;
	};
}

bool CreateMeshesFromRegisteredSurfaces(Pint& pint, const PintCaps& caps, const TestBase& test, const PINT_MATERIAL_CREATE* material)
{
	if(!caps.mSupportMeshes)
		return false;

	DWORD time = TimeGetTime();

	// Take a deep breath... and down the hatch
	const udword Nb = test.GetNbSurfaces();
	CreateProgressBar(Nb, _F("Loading... Please wait for: %s", pint.GetName()));

	const bool Regular = true;
	const bool MergeAll = false;
	const bool MergeGrid = false;
	const bool Tessellate = false;

	if(Tessellate)
	{
		for(udword i=0;i<Nb;i++)
		{
			SetProgress(i);

			const IndexedSurface* IS = test.GetSurface(i);

			Triangles NewTris;
			const Point* V = IS->GetVerts();
			udword TotalExtraTris = 0;
			for(udword j=0;j<IS->GetNbFaces();j++)
			{
				const IndexedTriangle* T = IS->GetFace(j);
				const Triangle Tr(V[T->mRef[0]], V[T->mRef[1]], V[T->mRef[2]]);
				udword NbNewTris = 0;
				TessellateTriangle(NbNewTris, Tr, NewTris);
				ASSERT(NbNewTris);
				TotalExtraTris += NbNewTris-1;
			}
			printf("TotalExtraTris: %d\n", TotalExtraTris);

			Permutation P(NewTris.GetNbTriangles()*3);
			P.Identity();
			const udword* Indices = P.GetList();

			PINT_MESH_CREATE MeshDesc;
//			MeshDesc.mSurface	= IS->GetSurfaceInterface();
			MeshDesc.mSurface.mNbFaces	= NewTris.GetNbTriangles();
			MeshDesc.mSurface.mDFaces	= Indices;
			MeshDesc.mSurface.mNbVerts	= NewTris.GetNbTriangles()*3;
			MeshDesc.mSurface.mVerts	= NewTris.GetTriangles()->mVerts;
			MeshDesc.mRenderer			= CreateMeshRenderer(MeshDesc.mSurface);
			MeshDesc.mMaterial			= material;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &MeshDesc;
			ObjectDesc.mPosition	= Point(0.0f, 0.0f, 0.0f);
			ObjectDesc.mMass		= 0.0f;
			CreatePintObject(pint, ObjectDesc);
		}
	}

	if(Regular)
	{
		for(udword i=0;i<Nb;i++)
		{
			SetProgress(i);

//			if(i!=5)
//				continue;

			const IndexedSurface* IS = test.GetSurface(i);

			PINT_MESH_CREATE MeshDesc;
			MeshDesc.mSurface	= IS->GetSurfaceInterface();
			MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);
			MeshDesc.mMaterial	= material;

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes		= &MeshDesc;
			ObjectDesc.mPosition	= Point(0.0f, 0.0f, 0.0f);
			if(gRotateMeshes)//###MEGADEBUG
			{
//				ObjectDesc.mPosition	= Point(10.2f, -2.0f, 3.15f);
				ObjectDesc.mRotation	= Quat(1.1f, 2.2f, 3.3f, 4.4f);
				ObjectDesc.mRotation.Normalize();

				Matrix3x3 RotX;
				RotX.RotX(HALFPI*0.5f);
				ObjectDesc.mRotation = RotX;
				ObjectDesc.mPosition = Point(0.1f, -0.2f, 0.3f);
			}
			ObjectDesc.mMass		= 0.0f;
			CreatePintObject(pint, ObjectDesc);
		}
	}
	
	if(MergeAll)
	{
		udword TotalNbVerts = 0;
		udword TotalNbTris = 0;
		for(udword i=0;i<Nb;i++)
		{
			const IndexedSurface* IS = test.GetSurface(i);
			TotalNbVerts += IS->GetNbVerts();
			TotalNbTris += IS->GetNbFaces();
		}

		IndexedSurface Merged;
		bool Status = Merged.Init(TotalNbTris, TotalNbVerts);
		ASSERT(Status);

		Point* V = Merged.GetVerts();
		IndexedTriangle* T = const_cast<IndexedTriangle*>(Merged.GetFaces());

		udword Offset = 0;
		for(udword i=0;i<Nb;i++)
		{
			SetProgress(i);
			const IndexedSurface* IS = test.GetSurface(i);
//			Merged.Merge(IS);

			const udword LocalNbVerts = IS->GetNbVerts();
			const Point* LocalV = IS->GetVerts();
			CopyMemory(V, LocalV, LocalNbVerts*sizeof(Point));
			V += LocalNbVerts;

			const udword LocalNbTris = IS->GetNbFaces();
			const IndexedTriangle* LocalT = IS->GetFaces();
			for(udword j=0;j<LocalNbTris;j++)
			{
				T[j].mRef[0] = LocalT[j].mRef[0] + Offset;
				T[j].mRef[1] = LocalT[j].mRef[1] + Offset;
				T[j].mRef[2] = LocalT[j].mRef[2] + Offset;
			}
			T += LocalNbTris;
			Offset += LocalNbVerts;
		}

		PINT_MESH_CREATE MeshDesc;
		MeshDesc.mSurface	= Merged.GetSurfaceInterface();
		MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);
		MeshDesc.mMaterial	= material;

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &MeshDesc;
		ObjectDesc.mPosition	= Point(0.0f, 0.0f, 0.0f);
		ObjectDesc.mMass		= 0.0f;
		CreatePintObject(pint, ObjectDesc);
	}

	if(MergeGrid)
	{
		AABB GlobalBounds;
		GlobalBounds.SetEmpty();

		udword TotalNbVerts = 0;
		udword TotalNbTris = 0;
		for(udword i=0;i<Nb;i++)
		{
			const IndexedSurface* IS = test.GetSurface(i);
			TotalNbVerts += IS->GetNbVerts();
			TotalNbTris += IS->GetNbFaces();

			for(udword j=0;j<IS->GetNbVerts();j++)
				GlobalBounds.Extend(*IS->GetVertex(j));
		}

		IndexedSurface Merged;
		bool Status = Merged.Init(TotalNbTris, TotalNbVerts);
		ASSERT(Status);

		Point* V = Merged.GetVerts();
		IndexedTriangle* T = const_cast<IndexedTriangle*>(Merged.GetFaces());

		udword Offset = 0;
		for(udword i=0;i<Nb;i++)
		{
			SetProgress(i);
			const IndexedSurface* IS = test.GetSurface(i);
//			Merged.Merge(IS);

			const udword LocalNbVerts = IS->GetNbVerts();
			const Point* LocalV = IS->GetVerts();
			CopyMemory(V, LocalV, LocalNbVerts*sizeof(Point));
			V += LocalNbVerts;

			const udword LocalNbTris = IS->GetNbFaces();
			const IndexedTriangle* LocalT = IS->GetFaces();
			for(udword j=0;j<LocalNbTris;j++)
			{
				T[j].mRef[0] = LocalT[j].mRef[0] + Offset;
				T[j].mRef[1] = LocalT[j].mRef[1] + Offset;
				T[j].mRef[2] = LocalT[j].mRef[2] + Offset;
			}
			T += LocalNbTris;
			Offset += LocalNbVerts;
		}

	Point Min, Max;
	GlobalBounds.GetMin(Min);
	GlobalBounds.GetMax(Max);
	Min -= Point(1.0f, 1.0f, 1.0f);
	Max += Point(1.0f, 1.0f, 1.0f);

	udword nb_subdiv = 16;
	const float dx = (Max.x - Min.x) / float(nb_subdiv);
	const float dy = (Max.y - Min.y) / float(nb_subdiv);
	const float dz = (Max.z - Min.z) / float(nb_subdiv);
	const udword NbCells = nb_subdiv*nb_subdiv*nb_subdiv;
	AABB* Cells = ICE_NEW(AABB)[NbCells];
//	udword* Counters = new udword[NbCells];
//	ZeroMemory(Counters, sizeof(udword)*NbCells);
	Container* CellTris = ICE_NEW(Container)[NbCells];
	Offset = 0;
	for(udword z=0;z<nb_subdiv;z++)
	{
		for(udword y=0;y<nb_subdiv;y++)
		{
			for(udword x=0;x<nb_subdiv;x++)
			{
				const Point CurrentMin(	Min.x + dx * float(x),
										Min.y + dy * float(y),
										Min.z + dz * float(z));
				const Point CurrentMax(	Min.x + dx * float(x+1),
										Min.y + dy * float(y+1),
										Min.z + dz * float(z+1));

				Cells[Offset].SetMinMax(CurrentMin, CurrentMax);
				Offset++;
			}
		}
	}


		V = Merged.GetVerts();
		T = const_cast<IndexedTriangle*>(Merged.GetFaces());

//		udword* Sorted = new udword[TotalNbTris];
		for(udword i=0;i<TotalNbTris;i++)
		{
			Point TriCenter;
			T[i].Center(V, TriCenter);

			udword CellIndex = INVALID_ID;
			for(udword j=0;j<NbCells;j++)
			{
				if(Cells[j].ContainsPoint(TriCenter))
				{
					CellIndex = j;
					break;
				}
			}
			ASSERT(CellIndex!=INVALID_ID);

//			Sorted[i] = CellIndex;
//			Counters[CellIndex]++;

			Container& C = CellTris[CellIndex];
			IndexedTriangle* Buffer = (IndexedTriangle*)C.Reserve(sizeof(IndexedTriangle)/sizeof(udword));
			*Buffer = T[i];
		}

		for(udword i=0;i<NbCells;i++)
		{
			const Container& C = CellTris[i];
			if(C.GetNbEntries())
			{
				udword NbTris = C.GetNbEntries()/(sizeof(IndexedTriangle)/sizeof(udword));
				if(NbTris==1)
					continue;

				IndexedSurface GridMesh;
				GridMesh.Init(NbTris, TotalNbVerts, Merged.GetVerts(), (const IndexedTriangle*)C.GetEntries());
				GridMesh.Optimize();

				PINT_MESH_CREATE MeshDesc;
				MeshDesc.mSurface	= GridMesh.GetSurfaceInterface();
				MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);
				MeshDesc.mMaterial	= material;

				PINT_OBJECT_CREATE ObjectDesc;
				ObjectDesc.mShapes		= &MeshDesc;
				ObjectDesc.mPosition	= Point(0.0f, 0.0f, 0.0f);
				ObjectDesc.mMass		= 0.0f;
				CreatePintObject(pint, ObjectDesc);
			}
		}


//		DELETEARRAY(Sorted);
	DELETEARRAY(CellTris);
//	DELETEARRAY(Counters);
	DELETEARRAY(Cells);
	}

	ReleaseProgressBar();

	time = TimeGetTime() - time;
	printf("Mesh creation time: %d (%s)\n", time, pint.GetName());

	return true;
}

///////////////////////////////////////////////////////////////////////////////

PintObjectHandle CreatePlanarMesh(TestBase& test, Pint& pint, float altitude, udword nb, float scale, const PINT_MATERIAL_CREATE* material)
{
	IndexedSurface* IS;
	if(test.GetNbSurfaces())
	{
		IS = test.GetFirstSurface();
	}
	else
	{
		IS = test.CreateManagedSurface();
		bool status = IS->MakePlane(nb, nb);
		ASSERT(status);
		IS->Scale(Point(scale, 1.0f, scale));
		IS->Flip();
	}

	PINT_MESH_CREATE MeshDesc;
	MeshDesc.mSurface	= IS->GetSurfaceInterface();
	MeshDesc.mRenderer	= CreateMeshRenderer(MeshDesc.mSurface);
	MeshDesc.mMaterial	= material;

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes		= &MeshDesc;
	ObjectDesc.mPosition	= Point(0.0f, altitude, 0.0f);
	ObjectDesc.mMass		= 0.0f;
	return CreatePintObject(pint, ObjectDesc);
}

///////////////////////////////////////////////////////////////////////////////

#ifdef REMOVED
static bool CreateRope(	Pint& pint, PintObjectHandle articulation, PintObjectHandle parent,
						const Point& p0, const Point& p1, udword nb_capsules, float capsule_radius, float capsule_mass,
						/*bool articulation,*/ bool overlapping_links, bool create_distance_constraints, PintObjectHandle* handles=null, float* half_height=null, Quat* qq=null)
{
	if(nb_capsules>MAX_LINKS)
		return false;

//	PintObjectHandle Articulation = articulation ? pint.CreateArticulation() : null;

	const Point Dir = (p1 - p0).Normalize();

	float Offset;
	float HalfHeight;
	Point Extents;
	if(!overlapping_links)
	{
		const float TotalLength = p0.Distance(p1);
		// <===TotalLength====>
		// (=*=)(=*=)(=*=)(=*=)
		// We divide TotalLength in N equal parts.
		const float LinkLength = TotalLength/float(nb_capsules);
		//    LinkLength = (Radius + HalfHeight)*2
		//<=> HalfHeight = (LinkLength/2) - Radius
		HalfHeight = LinkLength*0.5f - capsule_radius;
		if(HalfHeight<0.0f)
			return false;
		Offset = capsule_radius + HalfHeight;
		Extents = Dir * (capsule_radius + HalfHeight);
	}
	else
	{
/* TODO
		float TotalLength = p0.Distance(p1);
		// <====TotalLength====>
		// (=*=X=*=X=*=X=*=X=*=)
		// Remove start & end radius from total length:
		TotalLength -= capsule_radius*2.0f;
		if(TotalLength<0.0f)
			return false;
		// <===TotalLength===>
		// =*=X=*=X=*=X=*=X=*=

		//    LinkLength = (Radius + HalfHeight)*2
		//<=> HalfHeight = (LinkLength/2) - Radius
*/
	}

//	const Quat qXtoY = ShortestRotation(Point(1.0f, 0.0f, 0.0f), Point(0.0f, 1.0f, 0.0f));
	const Quat qToDir = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Dir);
	if(qq)
		*qq = qToDir;

	if(half_height)
		*half_height = HalfHeight;

	PINT_CAPSULE_CREATE CapsuleDesc(capsule_radius, HalfHeight);
//	CapsuleDesc.mLocalRot	= qXtoY;
	CapsuleDesc.mRenderer	= CreateCapsuleRenderer(capsule_radius, HalfHeight*2.0f);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes		= &CapsuleDesc;
	ObjectDesc.mMass		= capsule_mass;
	ObjectDesc.mRotation	= qToDir;

	PintObjectHandle Handles[MAX_LINKS];
	Point Positions[MAX_LINKS];

	Point Pos = p0 + Offset*Dir;

//	udword GroupBit = 0;	###
	for(udword i=0;i<nb_capsules;i++)
	{
		Positions[i] = Pos;
		{
			ObjectDesc.mPosition		= Pos;
//			ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;	###
			if(articulation)
			{
				PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
				ArticulatedDesc.mParent = i ? Handles[i-1] : parent;
				ArticulatedDesc.mLocalPivot0 = Extents;
				ArticulatedDesc.mLocalPivot1 = -Extents;
				Handles[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, articulation);
			}
			else
			{
				Handles[i] = CreatePintObject(pint, ObjectDesc);
				if(i)
				{
					PINT_SPHERICAL_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i-1];
					Desc.mObject1		= Handles[i];
					Desc.mLocalPivot0	= Extents;
					Desc.mLocalPivot1	= -Extents;
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
			if(handles)
				handles[i] = Handles[i];
		}
		Pos += Extents*2.0f;
	}

//	if(Articulation)
//		pint.AddArticulationToScene(Articulation);

	if(1)
	{
		if(0)
		{
			udword i=nb_capsules-2;
			PINT_DISTANCE_JOINT_CREATE Desc;
			Desc.mObject0		= Handles[0];
			Desc.mObject1		= Handles[i];
			Desc.mDistance		= Positions[i].Distance(Positions[0]);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}

		if(create_distance_constraints)
		{
			for(udword i=0;i<nb_capsules;i++)
			{
				if(i+2<nb_capsules)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+2];
					Desc.mDistance		= Positions[i].Distance(Positions[i+2]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}

		if(0)
		{
			for(udword i=1;i<nb_capsules;i++)
			{
				PINT_DISTANCE_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[0];
				Desc.mObject1		= Handles[i];
				Desc.mDistance		= Positions[0].Distance(Positions[i]);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
	}
	return true;
}

/*static bool CreateRope(Pint& pint, const Point& p0, const Point& p1, udword nb_capsules, float capsule_radius, float capsule_mass, bool articulation, bool overlapping_links, bool create_distance_constraints, PintObjectHandle* handles=null, float* half_height=null)
{
//	PintObjectHandle Articulation = articulation ? pint.CreateArticulation() : null;
	return CreateRope(pint, articulation, p0, p1, nb_capsules, capsule_radius, capsule_mass, articulation, overlapping_links, create_distance_constraints, handles, half_height);
//	if(Articulation)
//		pint.AddArticulationToScene(Articulation);
}*/
#endif

bool CreateCapsuleRope(	Pint& pint, PintObjectHandle articulation, PintObjectHandle parent,
						float capsule_radius, float half_height, float capsule_mass, float capsule_mass_for_inertia,
						const Point& capsule_dir, const Point& start_pos, udword nb_capsules,
						bool create_distance_constraints, PintObjectHandle* handles, const PintCollisionGroup* collision_groups)
{
	if(nb_capsules>MAX_LINKS)
		return false;

	const Quat qToDir = ShortestRotation(Point(0.0f, 1.0f, 0.0f), capsule_dir);

	PINT_CAPSULE_CREATE CapsuleDesc(capsule_radius, half_height);
	CapsuleDesc.mRenderer	= CreateCapsuleRenderer(capsule_radius, half_height*2.0f);

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes			= &CapsuleDesc;
	ObjectDesc.mMass			= capsule_mass;
	ObjectDesc.mMassForInertia	= capsule_mass_for_inertia;
	ObjectDesc.mRotation		= qToDir;

	Point CapsulePos = start_pos + capsule_dir*(capsule_radius+half_height);
	const Point CapsuleOffset(0.0f, capsule_radius+half_height, 0.0f);

#ifdef SETUP_ROPE_COLLISION_GROUPS
	udword GroupBit = 0;
#endif

	PintObjectHandle Handles[MAX_LINKS];
	Point Positions[MAX_LINKS];
	for(udword i=0;i<nb_capsules;i++)
	{
#ifdef SETUP_ROPE_COLLISION_GROUPS
		ObjectDesc.mCollisionGroup	= 1 + GroupBit;	GroupBit = 1 - GroupBit;
#endif
		if(collision_groups)
			ObjectDesc.mCollisionGroup = collision_groups[i];

		ObjectDesc.mPosition = Positions[i] = CapsulePos;
		CapsulePos += capsule_dir*(capsule_radius+half_height)*2.0f;
		if(articulation)
		{
			PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
			ArticulatedDesc.mParent = i ? Handles[i-1] : parent;
			ArticulatedDesc.mLocalPivot0 = CapsuleOffset;
			ArticulatedDesc.mLocalPivot1 = -CapsuleOffset;
			Handles[i] = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, articulation);
		}
		else
			Handles[i] = CreatePintObject(pint, ObjectDesc);
		if(handles)
			handles[i] = Handles[i];
	}

	if(!articulation)
	{
		if(parent)
		{
			PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(parent, Handles[0], CapsuleOffset, -CapsuleOffset));
			ASSERT(JointHandle);
		}

//		for(udword j=0;j<32;j++)
		{
			for(udword i=1;i<nb_capsules;i++)
			{
				PintJointHandle JointHandle = pint.CreateJoint(PINT_SPHERICAL_JOINT_CREATE(Handles[i-1], Handles[i], CapsuleOffset, -CapsuleOffset));
				ASSERT(JointHandle);
			}
		}
	}

	if(1)
	{
		if(0)
		{
			udword i=nb_capsules-2;
			PINT_DISTANCE_JOINT_CREATE Desc;
			Desc.mObject0		= Handles[0];
			Desc.mObject1		= Handles[i];
			Desc.mMaxDistance	= Positions[i].Distance(Positions[0]);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}

		if(create_distance_constraints)
		{
			for(udword i=0;i<nb_capsules;i++)
			{
				if(i+2<nb_capsules)
				{
					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Handles[i];
					Desc.mObject1		= Handles[i+2];
					Desc.mMaxDistance	= Positions[i].Distance(Positions[i+2]);
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}

		if(0)
		{
			for(udword i=1;i<nb_capsules;i++)
			{
				PINT_DISTANCE_JOINT_CREATE Desc;
				Desc.mObject0		= Handles[0];
				Desc.mObject1		= Handles[i];
				Desc.mMaxDistance	= Positions[0].Distance(Positions[i]);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
		}
	}
	return true;
}

bool CreateCapsuleRope2(Pint& pint, float& half_height, PintObjectHandle articulation, PintObjectHandle parent,
						const Point& p0, const Point& p1, udword nb_capsules,
						float capsule_radius, float capsule_mass, float capsule_mass_for_inertia,
						bool /*overlapping_links*/, bool create_distance_constraints, PintObjectHandle* handles, const PintCollisionGroup* collision_groups)
{
	const Point CapsuleDir = (p1 - p0).Normalize();

	float Offset;
	float HalfHeight;
//	Point Extents;
//	if(!overlapping_links)
	{
		const float TotalLength = p0.Distance(p1);
		// <===TotalLength====>
		// (=*=)(=*=)(=*=)(=*=)
		// We divide TotalLength in N equal parts.
		const float LinkLength = TotalLength/float(nb_capsules);
		//    LinkLength = (Radius + HalfHeight)*2
		//<=> HalfHeight = (LinkLength/2) - Radius
		HalfHeight = LinkLength*0.5f - capsule_radius;
		if(HalfHeight<0.0f)
			return false;
		Offset = capsule_radius + HalfHeight;
//		Extents = CapsuleDir * (capsule_radius + HalfHeight);
	}

	half_height = HalfHeight;

	const Point StartPos = p0;// + Extents;

	return CreateCapsuleRope(pint, articulation, parent, capsule_radius, HalfHeight, capsule_mass, capsule_mass_for_inertia, CapsuleDir, StartPos, nb_capsules, create_distance_constraints, handles, collision_groups);
}

///////////////////////////////////////////////////////////////////////////////

// This one is unfinished & old. Not properly tuned or optimized or anything, etc.
// TODO: refactor with new one in Bulldozer demo
void CreateCaterpillarTrack_OLD(Pint& pint, CaterpillarTrackObjects& objects, PINT_MATERIAL_CREATE* material, const Point& pos,
						 const CylinderMesh& mCylinder, PintShapeRenderer* cylinder_renderer,
						 const CylinderMesh& mCylinder2, PintShapeRenderer* cylinder_renderer2
						 )
{
	//### TODO: we really need an editor for all these params

	PINT_MATERIAL_CREATE FrictionlessMaterial;
	FrictionlessMaterial.mStaticFriction	= 0.0f;
	FrictionlessMaterial.mDynamicFriction	= 0.0f;
	FrictionlessMaterial.mRestitution		= 0.0f;

	const float Radius = 4.0f;
//	const float Altitude = 2.0f;
//	const float Altitude = pos.y;
	const float LinkMass = 1.0f;
	const float GearMass = 10.0f;
	const float SideMass = 4.0f;

	////

	Curve C;
//	const udword NbCurvePts = 4;
	const udword NbCurvePts = 1024;
//	const udword NbCurvePts = 64;
//	const udword NbCurvePts = 16;
	C.Init(NbCurvePts);
	C.mClosed = true;
	for(udword i=0;i<NbCurvePts;i++)
	{
		const float Coeff = float(i)/float(NbCurvePts);
		const float Angle = Coeff*TWOPI;
		const float x = sinf(Angle) * Radius;
		/*const*/ float y = cosf(Angle) * Radius * 0.5f;
		const float LimitP = 0.5f*Radius;
//		const float LimitP = 0.5f*Radius*0.5f;
		const float LimitN = 0.5f*Radius*0.5f;
		if(y>LimitP)
			y=LimitP;
		if(y<-LimitN)
			y=-LimitN;
		C.SetVertex(x, y/*+Altitude*/, 0.0f, i);
	}
	const float Length = C.ComputeLength();

	////

//	const udword NbLinks = 48;
//	const udword NbLinks = 32*2;
	const udword NbLinks = 32;	//
//	const udword NbLinks = 64;
//	const udword NbLinks = 16;
//	const udword NbLinks = 128;
//	const float D = PI*Radius/float(NbLinks);
	const float D = Length*0.5f/float(NbLinks);
//	const Point Extents(D*0.8f, 0.1f, 1.0f);
	const Point Extents(D, 0.1f, 1.0f);
//	const Point ArticulationPos(0.0f, Altitude + Extents.y, 0.0f);
//	const Point ArticulationPos = pos + Point(0.0f, Extents.y, 0.0f);

	//

	// Main plank
	PINT_BOX_CREATE BoxDesc(Extents);
	BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);
	BoxDesc.mMaterial	= material;
	BoxDesc.mMaterial	= &FrictionlessMaterial;

		// Smaller inside box to catch the gear
		PINT_BOX_CREATE BoxDesc2;
//		BoxDesc2.mExtents	= Point(Extents.x*0.2f, 0.15f, Extents.z*0.4f);
		BoxDesc2.mExtents	= Point(Extents.x*0.2f, 0.2f, Extents.z*0.4f);
		BoxDesc2.mRenderer	= CreateBoxRenderer(BoxDesc2.mExtents);
		BoxDesc2.mMaterial	= material;
		BoxDesc2.mMaterial	= &FrictionlessMaterial;		
		BoxDesc2.mLocalPos	= Point(0.0f, -Extents.y - BoxDesc2.mExtents.y, 0.0f);
		BoxDesc.mNext		= &BoxDesc2;

		// Left & right upper boxes (will touch the ground)
		PINT_BOX_CREATE BoxDesc3;
//		BoxDesc3.mExtents	= Point(Extents.x, 0.15f, 0.1f);
//		BoxDesc3.mExtents	= Point(Extents.x, 0.3f, 0.1f);
			BoxDesc3.mExtents	= Point(Extents.x, 0.4f, 0.1f);
//BoxDesc3.mExtents	= Point(Extents.x, 0.3f, 0.1f);
		BoxDesc3.mRenderer	= CreateBoxRenderer(BoxDesc3.mExtents);
		BoxDesc3.mMaterial	= material;
		BoxDesc3.mLocalPos	= Point(0.0f, BoxDesc3.mExtents.y - Extents.y, Extents.z + BoxDesc3.mExtents.z);
			BoxDesc3.mLocalPos	= Point(0.0f, 0.0f, Extents.z + BoxDesc3.mExtents.z);
//		BoxDesc3.mLocalPos	= Point(0.0f, -BoxDesc3.mExtents.y, Extents.z + BoxDesc3.mExtents.z);
//BoxDesc3.mLocalPos	= Point(0.0f, - BoxDesc3.mExtents.y + Extents.y, Extents.z + BoxDesc3.mExtents.z);
		BoxDesc2.mNext		= &BoxDesc3;

		PINT_BOX_CREATE BoxDesc4(BoxDesc3.mExtents);
		BoxDesc4.mRenderer	= BoxDesc3.mRenderer;
		BoxDesc4.mMaterial	= material;
		BoxDesc4.mLocalPos	= Point(0.0f, BoxDesc3.mLocalPos.y, -BoxDesc3.mLocalPos.z);
		BoxDesc3.mNext		= &BoxDesc4;

	PINT_OBJECT_CREATE ObjectDesc;
	ObjectDesc.mShapes		= &BoxDesc;
	ObjectDesc.mMass		= LinkMass;
//	ObjectDesc.mPosition	= ArticulationPos;
	ObjectDesc.mMassForInertia = LinkMass*10.0f;

	// There are multiple issues with articulations:
	// - they tend to emulate what a high solver iteration count would do with regular joints. And it turns out
	// the system explodes easily with regular joints & high counts when things are not properly setup (like not physically possible).
	// That's the case here with some settings that make the track impossible to actually wrap around the gears, etc.
	// So using articulations exposes that and explodes.
	// - then it looks like the default friction doesn't work well with this. It works much better with the other friction modes.......
	// Looks like a bug in PhysX we'll have to investigate & fix.
//	PintObjectHandle Articulation = pint.CreateArticulation(PINT_ARTICULATION_CREATE());
	PintObjectHandle Articulation = null;

	float Angles[128];
	for(udword i=0;i<NbLinks;i++)
	{
		const float Coeff = float(i)/float(NbLinks);
		Point pts[2];
		Point tmp;
		C.GetPoint(tmp, Coeff, pts);
		const float dx = pts[1].x - pts[0].x;
		const float dy = pts[1].y - pts[0].y;
		Angles[i] = atan2f(dy, dx);
	}

	PintObjectHandle Objects[128];
	Point Positions[128];
	PintObjectHandle Parent = null;
	for(udword i=0;i<NbLinks;i++)
	{
		const udword ii = (i)%NbLinks;

		const float Coeff = float(ii)/float(NbLinks);

		Point pts[2];
		C.GetPoint(ObjectDesc.mPosition, Coeff, pts);

		ObjectDesc.mPosition += pos;

		float Angle2;
		if(0)
		{
			const udword jj = (i+1)%NbLinks;
			const float NextCoeff = float(jj)/float(NbLinks);
			Point NextPt;
			C.GetPoint(NextPt, NextCoeff);
			const float dx = NextPt.x - ObjectDesc.mPosition.x;
			const float dy = NextPt.y - ObjectDesc.mPosition.y;
			Angle2 = atan2f(dy, dx);
		}
		if(1)
		{
			const float dx = pts[1].x - pts[0].x;
			const float dy = pts[1].y - pts[0].y;
			Angle2 = atan2f(dy, dx);
		}
		if(0)
		{
			const udword j = (i-1)%NbLinks;
			const udword k = (i+1)%NbLinks;
			Angle2 = (Angles[i] + Angles[j] + Angles[k])/3.0f;
		}

		Matrix3x3 Rot;
//		Rot.RotZ(-Angle);
		Rot.RotZ(Angle2);
		ObjectDesc.mRotation = Rot;

		if(Articulation)
		{
			PINT_ARTICULATED_BODY_CREATE ArticulatedDesc;
			if(Parent)
			{
				ArticulatedDesc.mParent = Parent;
				ArticulatedDesc.mLocalPivot0 = Point(D, 0.0f, 0.0f);
				ArticulatedDesc.mLocalPivot1 = Point(-D, 0.0f, 0.0f);
			}
			if(0)
			{
				ArticulatedDesc.mX = Point(0.0f, 0.0f, 1.0f);
				ArticulatedDesc.mEnableTwistLimit = false;
				ArticulatedDesc.mTwistLowerLimit = -0.001f;
				ArticulatedDesc.mTwistUpperLimit = 0.001f;
				ArticulatedDesc.mEnableSwingLimit = true;
				ArticulatedDesc.mSwingYLimit = 0.001f;
				ArticulatedDesc.mSwingZLimit = 0.001f;
			}
			else
			{
				ArticulatedDesc.mEnableTwistLimit = true;
				ArticulatedDesc.mTwistLowerLimit = -0.001f;
				ArticulatedDesc.mTwistUpperLimit = 0.001f;
				ArticulatedDesc.mEnableSwingLimit = true;
//				ArticulatedDesc.mSwingYLimit = 0.001f;
				ArticulatedDesc.mSwingZLimit = 0.001f;
//				ArticulatedDesc.mSwingYLimit = FLT_MAX;
				ArticulatedDesc.mSwingYLimit = PI - 0.001f;
			}
			Parent = pint.CreateArticulatedObject(ObjectDesc, ArticulatedDesc, Articulation);
		}
		else
		{
			Parent = pint.CreateObject(ObjectDesc);
		}
		Objects[i] = Parent;
		Positions[i] = ObjectDesc.mPosition;
	}
	if(Articulation)
		pint.AddArticulationToScene(Articulation);

	if(Articulation)
	{
		PINT_HINGE_JOINT_CREATE Desc;
		Desc.mObject0		= Objects[0];
		Desc.mObject1		= Objects[NbLinks-1];
		Desc.mLocalPivot0	= Point(-D, 0.0f, 0.0f);
		Desc.mLocalPivot1	= Point(D, 0.0f, 0.0f);
		Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
		Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
		PintJointHandle JointHandle = pint.CreateJoint(Desc);
		ASSERT(JointHandle);
	}
	else
	{
		//### Brute force to get something working. We should tune & optimize this later.
		const bool UseExtraDistanceConstraints = true;
		for(udword j=0;j<32;j++)
		{
			for(udword i=0;i<NbLinks;i++)
			{
				const udword ii = (i)%NbLinks;
				const udword jj = (i+1)%NbLinks;

				PINT_HINGE_JOINT_CREATE Desc;
				Desc.mObject0		= Objects[ii];
				Desc.mObject1		= Objects[jj];
				Desc.mLocalPivot0	= Point(D, 0.0f, 0.0f);
				Desc.mLocalPivot1	= Point(-D, 0.0f, 0.0f);
				Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
				Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
				PintJointHandle JointHandle = pint.CreateJoint(Desc);
				ASSERT(JointHandle);
			}
			if(UseExtraDistanceConstraints)
			{
				for(udword i=0;i<NbLinks;i++)
				{
					const udword ii = (i)%NbLinks;
					const udword jj = (i+2)%NbLinks;

					PINT_DISTANCE_JOINT_CREATE Desc;
					Desc.mObject0		= Objects[ii];
					Desc.mObject1		= Objects[jj];
	//				Desc.mMaxDistance	= Positions[ii].Distance(Positions[jj]);
					Desc.mMaxDistance	= Extents.x*2.0f*2.0f;
					PintJointHandle JointHandle = pint.CreateJoint(Desc);
					ASSERT(JointHandle);
				}
			}
		}
	}

	const float NbGears = 3;
	objects.mNbGears = NbGears;
//	const float x = Radius - mCylinder.mRadius - 0.005f;
//	const float x = Radius - mCylinder2.mRadius - 0.1f;
	const float x = Radius - mCylinder2.mRadius - 0.075f;
//	const float x = Radius - mCylinder2.mRadius - 0.05f;
	PintObjectHandle GearObject[3];
	const float xs[3] = { x, -x, 0.0f };
//	const float ys[3] = { 0.0f, 0.0f, 0.0f };
//	const float ys[3] = { 0.0f, 0.0f, 1.0f };
	const float ys[3] = { 0.0f, 0.0f, 0.9f };
//	const float ys[3] = { 0.0f, 0.0f, 1.2f };
//	const float ys[3] = { 0.0f, 0.0f, 1.1f };
//	const float ys[3] = { 0.0f, 0.0f, 1.5f };
	for(udword i=0;i<NbGears;i++)
	{
		PINT_CONVEX_CREATE ConvexCreate(mCylinder.mNbVerts, mCylinder.mVerts);
		ConvexCreate.mRenderer	= cylinder_renderer;
		ConvexCreate.mMaterial	= material;

		PINT_CONVEX_CREATE ConvexCreate2(mCylinder2.mNbVerts, mCylinder2.mVerts);
		ConvexCreate2.mRenderer	= cylinder_renderer2;
		ConvexCreate2.mMaterial	= material;
		ConvexCreate2.mLocalPos	= Point(0.0f, 0.0f, mCylinder.mHalfHeight+mCylinder2.mHalfHeight);
		ConvexCreate.mNext = &ConvexCreate2;

		PINT_CONVEX_CREATE ConvexCreate3 = ConvexCreate2;
		ConvexCreate3.mLocalPos	= Point(0.0f, 0.0f, -mCylinder.mHalfHeight-mCylinder2.mHalfHeight);
		ConvexCreate2.mNext = &ConvexCreate3;

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &ConvexCreate;
		ObjectDesc.mMass		= GearMass;
//		ObjectDesc.mPosition	= Point(xs[i], Altitude, 0.0f);
		ObjectDesc.mPosition	= pos + Point(xs[i], ys[i], 0.0f);
		GearObject[i] = CreatePintObject(pint, ObjectDesc);
		ASSERT(GearObject[i]);
		objects.mGears[i] = GearObject[i];
	}
	{
		const float Gap = 0.1f;
		PINT_BOX_CREATE SideBoxDesc(x, 0.3f, 0.1f);
		SideBoxDesc.mRenderer	= CreateBoxRenderer(SideBoxDesc.mExtents);
		const float z = mCylinder.mHalfHeight + mCylinder2.mHalfHeight*2.0f + SideBoxDesc.mExtents.z + Gap;
		SideBoxDesc.mLocalPos	= Point(0.0f, 0.0f, z);

		PINT_BOX_CREATE SideBoxDesc2 = SideBoxDesc;
		SideBoxDesc2.mLocalPos	= Point(0.0f, 0.0f, -z);
		SideBoxDesc.mNext = &SideBoxDesc2;

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &SideBoxDesc;
		ObjectDesc.mMass		= SideMass;
//		ObjectDesc.mPosition	= Point(0.0f, Altitude, 0.0f);
		ObjectDesc.mPosition	= pos;
		PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
		ASSERT(Handle);
		objects.mChassis = Handle;

		for(udword i=0;i<NbGears;i++)
		{
			PINT_HINGE_JOINT_CREATE Desc;
			Desc.mObject0		= GearObject[i];
			Desc.mObject1		= Handle;
			Desc.mLocalPivot0	= Point(0.0f, 0.0f, 0.0f);
			Desc.mLocalPivot1	= Point(xs[i], ys[i], 0.0f);
			Desc.mLocalAxis0	= Point(0.0f, 0.0f, 1.0f);
			Desc.mLocalAxis1	= Point(0.0f, 0.0f, 1.0f);
			PintJointHandle JointHandle = pint.CreateJoint(Desc);
			ASSERT(JointHandle);
		}
	}

	if(0)
	{
		PINT_CONVEX_CREATE ConvexCreate(mCylinder.mNbVerts, mCylinder.mVerts);
		ConvexCreate.mRenderer	= cylinder_renderer;
		ConvexCreate.mLocalPos	= Point(x, 0.0f, 0.0f);

		PINT_CONVEX_CREATE ConvexCreate2(mCylinder.mNbVerts, mCylinder.mVerts);
		ConvexCreate2.mRenderer	= cylinder_renderer;
		ConvexCreate2.mLocalPos	= Point(-x, 0.0f, 0.0f);
		ConvexCreate.mNext = &ConvexCreate2;

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &ConvexCreate;
		ObjectDesc.mMass		= 20.0f;
//		ObjectDesc.mPosition	= Point(0.0f, Altitude, 0.0f);
		ObjectDesc.mPosition	= pos;
		PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
		ASSERT(Handle);
	}
}

///////////////////////////////////////////////////////////////////////////////

bool ClampAngularVelocity(Pint& pint, PintObjectHandle wheel_parent, PintObjectHandle wheel, float max_velocity)
{
	const Point ParentAngularVelocity = pint.GetAngularVelocity(wheel_parent);

	const Point AngularVelocity = pint.GetAngularVelocity(wheel);
//	printf("Angular velocity %d: %f %f %f\n", i, AngularVelocity.x, AngularVelocity.y, AngularVelocity.z);

	Point RelativeVelocity = AngularVelocity - ParentAngularVelocity;

//	RelativeVelocity.x = RelativeVelocity.y = 0.0f;
	bool Clamped = false;
	if(RelativeVelocity.z>max_velocity)
	{
		RelativeVelocity.z = max_velocity;
		Clamped = true;
	}
	else if(RelativeVelocity.z<-max_velocity)
	{
		RelativeVelocity.z = -max_velocity;
		Clamped = true;
	}
	if(Clamped)
		pint.SetAngularVelocity(wheel, RelativeVelocity + ParentAngularVelocity);
	return Clamped;
}

///////////////////////////////////////////////////////////////////////////////

