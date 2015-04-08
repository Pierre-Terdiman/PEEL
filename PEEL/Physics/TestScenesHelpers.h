///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef TEST_SCENES_HELPERS_H
#define TEST_SCENES_HELPERS_H

#include "Pint.h"

//	static const float gValveScale = 0.01f;
	static const float gValveScale = 1.0f;

	static const float gSQMaxDist = 1000.0f;
//	static const float gSQMaxDist = 10.0f;

	class TestBase;
	class SurfaceManager;

	const char*			GetFile								(const char* filename, udword& size);

	void				GenerateCirclePts					(udword nb, Point* pts, float scale, float y);
	udword				GenerateConvex						(Point* verts, udword nbInsideCirclePts, udword nbOutsideCirclePts, float scale0, float scale1, float z);

	PintObjectHandle	CreateSimpleObject					(Pint& pint, const PINT_SHAPE_CREATE* shape, float mass, const Point& pos, const Quat* rot=null, const Point* linVel=null, const Point* angVel=null);
	PintObjectHandle	CreateDynamicObject					(Pint& pint, const PINT_SHAPE_CREATE* shape, const Point& pos, const Quat* rot=null, const Point* linVel=null, const Point* angVel=null);
	PintObjectHandle	CreateStaticObject					(Pint& pint, const PINT_SHAPE_CREATE* shape, const Point& pos, const Quat* rot=null);
	PintObjectHandle	CreateDynamicBox					(Pint& pint, float size_x, float size_y, float size_z, const Point& pos, const Quat* rot=null, const PINT_MATERIAL_CREATE* material=null);

	void				CreateSingleTriangleMesh			(SurfaceManager& test, float scale, Triangle* tri=null, bool reverse_winding=false);
	bool				Dominos_Setup						(Pint& pint, const PintCaps& caps, float friction);
	bool				CreateBoxStack						(Pint& pint, const PintCaps& caps, const udword nb_stacks, udword nb_base_boxes);
	void				CreateBoxContainer					(Pint& pint, float box_height, float box_side, float box_depth);
	bool				CreateSeaOfStaticBoxes				(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude);
	bool				CreateSeaOfStaticSpheres			(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude);
	bool				CreateSeaOfStaticCapsules			(Pint& pint, float amplitude, udword nb_x, udword nb_y, float altitude);
	bool				CreateSeaOfStaticConvexes			(Pint& pint, const PintCaps& caps, udword nb_x, udword nb_y, float altitude);

	bool				CreateTestScene_ConvexStack_Generic	(Pint& pint, udword NbX, udword NbY, udword NbLayers, udword NbInsideCirclePts, udword NbOutsideCirclePts);

	bool				CreateArrayOfDynamicConvexes		(Pint& pint, const PINT_CONVEX_CREATE& convex_create, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point* offset=null, const Point* lin_vel=null);
	bool				GenerateArrayOfSpheres				(Pint& pint, float radius, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass=1.0f, PintCollisionGroup group=0, const Point* offset=null);
	bool				GenerateArrayOfBoxes				(Pint& pint, const Point& extents, udword nb_x, udword nb_y, float altitude, float scale, float mass=1.0f, PintCollisionGroup group=0);
	bool				GenerateArrayOfCapsules				(Pint& pint, float radius, float half_height, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_z, float mass=1.0f, PintCollisionGroup group=0, const Point* offset=null);
	bool				GenerateArrayOfConvexes				(Pint& pint, const PintCaps& caps, bool is_static, float scale, udword nb_x, udword nb_y, udword convex_id);
	bool				GenerateArrayOfObjects				(Pint& pint, const PintCaps& caps, PintShape type, udword type_data, udword nb_x, udword nb_y, float altitude, float scale, float mass);

	void				GenerateConvexPile					(Pint& pint, udword NbX, udword NbY, udword NbLayers, float Amplitude, udword NbRandomPts);

	bool				Setup_PotPourri_Raycasts			(Pint& pint, const PintCaps& caps, float mass, udword nb_layers, udword nb_x, udword nb_y);
	bool				Setup_PotPourri_Raycasts			(TestBase& test, udword nb_rays, float max_dist);

	bool				CreateMeshesFromRegisteredSurfaces	(Pint& pint, const PintCaps& caps, const TestBase& test);

	void				CreatePlanarMesh					(TestBase& test, Pint& pint, float altitude);

	void				RegisterArrayOfRaycasts				(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float max_dist, const Point& offset);
	bool				GenerateArrayOfVerticalRaycasts		(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist);
	udword				DoBatchRaycasts						(TestBase& test, Pint& pint, bool use_phantoms = false);

	void				RegisterArrayOfBoxSweeps			(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, const Point& extents, const Point& offset, float max_dist);
	bool				GenerateArrayOfVerticalBoxSweeps	(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist);
	udword				DoBatchBoxSweeps					(TestBase& test, Pint& pint);
	void				UpdateBoxSweeps						(TestBase& test, float angle);

	void				RegisterArrayOfSphereSweeps			(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float radius, const Point& offset, float max_dist);
	bool				GenerateArrayOfVerticalSphereSweeps	(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist);
	udword				DoBatchSphereSweeps					(TestBase& test, Pint& pint);

	void				RegisterArrayOfCapsuleSweeps		(TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, float radius, float half_height, const Point& offset, float max_dist);
	bool				GenerateArrayOfVerticalCapsuleSweeps(TestBase& test, float scale, udword nb_x, udword nb_y, float max_dist);
	udword				DoBatchCapsuleSweeps				(TestBase& test, Pint& pint);
	void				UpdateCapsuleSweeps					(TestBase& test, float angle);

	udword				DoBatchConvexSweeps					(TestBase& test, Pint& pint);
	void				UpdateConvexSweeps					(TestBase& test, float angle);

	enum BatchOverlapMode
	{
		OVERLAP_ANY,
		OVERLAP_OBJECTS,
	};

	udword				DoBatchSphereOverlaps				(TestBase& test, Pint& pint, BatchOverlapMode mode);
	udword				DoBatchBoxOverlaps					(TestBase& test, Pint& pint, BatchOverlapMode mode);
	udword				DoBatchCapsuleOverlaps				(TestBase& test, Pint& pint, BatchOverlapMode mode);

	void				LoadRaysFile						(TestBase& test, const char* filename, bool only_rays, bool no_processing=false);

#endif