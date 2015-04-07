///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef RENDER_H
#define RENDER_H

	void SetupGLMatrix(const PR& pose);

	void DrawLine(const Point& p0, const Point& p1, const Point& color, float lineWidth=1.0f);
	void DrawCircle(udword nb_segments, const Matrix4x4& matrix, const Point& color, float radius, bool semi_circle=false);
	void DrawTriangle(const Point& p0, const Point& p1, const Point& p2, const Point& color);
//	void DrawPolygon(udword nb_pts, const Point* pts, const Point& normal, const Point& color);

	void DrawSphere(float radius, const PR& pose);
	void DrawSphereWireframe(float radius, const PR& pose, const Point& color);

	void DrawBox(const Point& extents, const PR& pose);

	void DrawCapsule(float r, float h, const PR& pose);
	void DrawCapsuleWireframe(float r, float h, const PR& pose, const Point& color);

	class PintShapeRenderer;
	PintShapeRenderer*	CreateSphereRenderer(float radius);
	PintShapeRenderer*	CreateCapsuleRenderer(float radius, float half_height);
	PintShapeRenderer*	CreateBoxRenderer(const Point& extents);
	PintShapeRenderer*	CreateConvexRenderer(udword nb_verts, const Point* verts);
	PintShapeRenderer*	CreateMeshRenderer(const SurfaceInterface& surface);
	void				ReleaseAllShapeRenderers();

	inline_ void DrawFrame(const Point& pt, float scale=1.0f)
	{
		DrawLine(pt, pt + Point(scale, 0.0f, 0.0f), Point(1.0f, 0.0f, 0.0f), 2.0f);
		DrawLine(pt, pt + Point(0.0f, scale, 0.0f), Point(0.0f, 1.0f, 0.0f), 2.0f);
		DrawLine(pt, pt + Point(0.0f, 0.0f, scale), Point(0.0f, 0.0f, 1.0f), 2.0f);
	}

#endif