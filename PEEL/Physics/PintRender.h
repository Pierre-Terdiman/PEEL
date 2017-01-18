///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_RENDER_H
#define PINT_RENDER_H

#include "Pint.h"

	class BasicRenderer : public PintRender
	{
		public:
						BasicRenderer() : mFrameNumber(0)	{}

		virtual	void	DrawLine						(const Point& p0, const Point& p1, const Point& color);
		virtual	void	DrawTriangle					(const Point& p0, const Point& p1, const Point& p2, const Point& color);

		virtual	void	DrawWirefameAABB				(const AABB& box, const Point& color);
		virtual	void	DrawWirefameOBB					(const OBB& box, const Point& color);

		virtual	void	DrawSphere						(float radius, const PR& pose);
		virtual	void	DrawBox							(const Point& extents, const PR& pose);
		virtual	void	DrawCapsule						(float radius, float height, const PR& pose);
		virtual	void	DrawCylinder					(float radius, float height, const PR& pose);

		virtual	void	DrawRaycastData					(udword nb, const PintRaycastData* raycast_data, const PintRaycastHit* hits, const Point& color);
		virtual	void	DrawRaycastAnyData				(udword nb, const PintRaycastData* raycast_data, const PintBooleanHit* hits, const Point& color);
		//
		virtual	void	DrawBoxSweepData				(udword nb, const PintBoxSweepData* box_sweep_data, const PintRaycastHit* hits, const Point& color);
		virtual	void	DrawSphereSweepData				(udword nb, const PintSphereSweepData* sphere_sweep_data, const PintRaycastHit* hits, const Point& color);
		virtual	void	DrawCapsuleSweepData			(udword nb, const PintCapsuleSweepData* capsule_sweep_data, const PintRaycastHit* hits, const Point& color);
		virtual	void	DrawConvexSweepData				(udword nb, const PintConvexSweepData* convex_sweep_data, const PintRaycastHit* hits, const Point& color);
		//
		virtual	void	DrawSphereOverlapAnyData		(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintBooleanHit* hits, const Point& color);
		virtual	void	DrawSphereOverlapObjectsData	(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintOverlapObjectHit* hits, const Point& color);
		virtual	void	DrawBoxOverlapAnyData			(udword nb, const PintBoxOverlapData* box_overlap_data, const PintBooleanHit* hits, const Point& color);
		virtual	void	DrawBoxOverlapObjectsData		(udword nb, const PintBoxOverlapData* box_overlap_data, const PintOverlapObjectHit* hits, const Point& color);
		virtual	void	DrawCapsuleOverlapAnyData		(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintBooleanHit* hits, const Point& color);
		virtual	void	DrawCapsuleOverlapObjectsData	(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintOverlapObjectHit* hits, const Point& color);

				udword	mFrameNumber;
	};

#endif