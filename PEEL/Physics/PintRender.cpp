///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PintRender.h"
#include "Render.h"

static inline_ void DrawImpact(const PintRaycastHit& hit)
{
	DrawFrame(hit.mImpact, 1.0f);
	::DrawLine(hit.mImpact, hit.mImpact + hit.mNormal, Point(1.0f, 1.0f, 1.0f), 2.0f);
}

static void DrawWireframeBox(const OBB& box, const Point& offset, const Point& color)
{
	Point Pts[8];
	box.ComputePoints(Pts);
	for(udword j=0;j<8;j++)
		Pts[j] += offset;

	const udword* Indices = box.GetEdges();
	for(udword j=0;j<12;j++)
	{
		const udword VRef0 = *Indices++;
		const udword VRef1 = *Indices++;
		::DrawLine(Pts[VRef0], Pts[VRef1], color);
	}
}

static void DrawWireframeSphere(const Sphere& sphere, const Point& offset, const Point& color)
{
	::DrawSphereWireframe(sphere.mRadius, PR(sphere.mCenter + offset, Quat(1.0f, 0.0f, 0.0f, 0.0f)), color);
}

static void DrawWireframeCapsule(float r, float h, const PR& pose, const Point& offset, const Point& color)
{
	::DrawCapsuleWireframe(r, h, PR(pose.mPos + offset, pose.mRot), color);
}

void BasicRenderer::DrawLine(const Point& p0, const Point& p1, const Point& color)
{
	::DrawLine(p0, p1, color);
}

void BasicRenderer::DrawTriangle(const Point& p0, const Point& p1, const Point& p2, const Point& color)
{
	::DrawTriangle(p0, p1, p2, color);
}

void BasicRenderer::DrawWirefameAABB(const AABB& box, const Point& color)
{
	Point Pts[8];
	box.ComputePoints(Pts);

	const udword* Edges = box.GetEdges();
	for(udword i=0;i<12;i++)
		::DrawLine(Pts[Edges[i*2]], Pts[Edges[i*2+1]], color);
}

void BasicRenderer::DrawWirefameOBB(const OBB& box, const Point& color)
{
	DrawWireframeBox(box, Point(0.0f, 0.0f, 0.0f), color);
}

void BasicRenderer::DrawSphere(float radius, const PR& pose)
{
	::DrawSphere(radius, pose);
}

void BasicRenderer::DrawBox(const Point& extents, const PR& pose)
{
	::DrawBox(extents, pose);
}

void BasicRenderer::DrawRaycastData(udword nb, const PintRaycastData* raycast_data, const PintRaycastHit* hits, const Point& color)
{
	if(!nb || !raycast_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		const Point& Origin = raycast_data[i].mOrigin;
		if(hits[i].mObject)
		{
			::DrawLine(Origin, hits[i].mImpact, color);
			DrawImpact(hits[i]);
		}
		else
		{
			const Point EndPoint = raycast_data[i].mOrigin + raycast_data[i].mDir * raycast_data[i].mMaxDist;
			::DrawLine(Origin, EndPoint, color);
		}
	}
}

void BasicRenderer::DrawRaycastAnyData(udword nb, const PintRaycastData* raycast_data, const PintBooleanHit* hits, const Point& color)
{
	if(!nb || !raycast_data || !hits)
		return;

	const Point Red(1.0f, 0.0f, 0.0f);
	const Point Green(0.0f, 1.0f, 0.0f);
	for(udword i=0;i<nb;i++)
	{
		const Point& Origin = raycast_data[i].mOrigin;
		const Point EndPoint = raycast_data[i].mOrigin + raycast_data[i].mDir * raycast_data[i].mMaxDist;
		if(hits[i].mHit)
			::DrawLine(Origin, EndPoint, Red);
		else
			::DrawLine(Origin, EndPoint, Green);
	}
}
		
void BasicRenderer::DrawBoxSweepData(udword nb, const PintBoxSweepData* box_sweep_data, const PintRaycastHit* hits, const Point& color)
{
	if(!nb || !box_sweep_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		// Draw solid box at initial position
		const Point& BoxStartPos = box_sweep_data[i].mBox.mCenter;
		const PR Pose(BoxStartPos, Quat(box_sweep_data[i].mBox.mRot));
		glColor4f(color.x, color.y, color.z, 1.0f);
		::DrawBox(box_sweep_data[i].mBox.mExtents, Pose);

		if(hits[i].mObject)
		{
			// Motion vector
			const Point Delta = hits[i].mDistance * box_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(BoxStartPos, BoxStartPos + Delta, color);

			// Draw wireframe box at impact position
			DrawWireframeBox(box_sweep_data[i].mBox, Delta, color);

			// Draw impact
			DrawImpact(hits[i]);
		}
		else
		{
			// Motion vector
			const Point Delta = box_sweep_data[i].mMaxDist * box_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(BoxStartPos, BoxStartPos + Delta, color);

			// Draw wireframe box at end position
			DrawWireframeBox(box_sweep_data[i].mBox, Delta, color);
		}
	}
}

void BasicRenderer::DrawSphereSweepData(udword nb, const PintSphereSweepData* sphere_sweep_data, const PintRaycastHit* hits, const Point& color)
{
	if(!nb || !sphere_sweep_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		// Draw solid sphere at initial position
		const Point& SphereStartPos = sphere_sweep_data[i].mSphere.mCenter;
		const PR Pose(SphereStartPos, Quat(1.0f, 0.0f, 0.0f, 0.0f));
		glColor4f(color.x, color.y, color.z, 1.0f);
		::DrawSphere(sphere_sweep_data[i].mSphere.mRadius, Pose);

		if(hits[i].mObject)
		{
			// Motion vector
			const Point Delta = hits[i].mDistance * sphere_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(SphereStartPos, SphereStartPos + Delta, color);

			// Draw wireframe sphere at impact position
			DrawWireframeSphere(sphere_sweep_data[i].mSphere, Delta, color);

			// Draw impact
			DrawImpact(hits[i]);
		}
		else
		{
			// Motion vector
			const Point Delta = sphere_sweep_data[i].mMaxDist * sphere_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(SphereStartPos, SphereStartPos + Delta, color);

			// Draw wireframe sphere at end position
			DrawWireframeSphere(sphere_sweep_data[i].mSphere, Delta, color);
		}
	}
}

void BasicRenderer::DrawCapsuleSweepData(udword nb, const PintCapsuleSweepData* capsule_sweep_data, const PintRaycastHit* hits, const Point& color)
{
	if(!nb || !capsule_sweep_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		// Draw solid capsule at initial position
		const LSS& Capsule = capsule_sweep_data[i].mCapsule;
		const Point CapsuleStartPos = (Capsule.mP0 + Capsule.mP1)*0.5f;

		Point CapsuleAxis = Capsule.mP1 - Capsule.mP0;
		const float M = CapsuleAxis.Magnitude();
		CapsuleAxis /= M;
		const Quat q = ShortestRotation(Point(1.0f, 0.0f, 0.0f), CapsuleAxis);

			const Quat qq = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
			Quat qqq = q * qq;

		const PR Pose(CapsuleStartPos, qqq);

		glColor4f(color.x, color.y, color.z, 1.0f);
//		::DrawCapsule(Capsule.mRadius, M*0.5f, Pose);
		::DrawCapsule(Capsule.mRadius, M, Pose);

		if(hits[i].mObject)
		{
			// Motion vector
			const Point Delta = hits[i].mDistance * capsule_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(CapsuleStartPos, CapsuleStartPos + Delta, color);

			// Draw wireframe capsule at impact position
//			DrawWireframeCapsule(Capsule.mRadius, M*0.5f, Pose, Delta, color);
			DrawWireframeCapsule(Capsule.mRadius, M, Pose, Delta, color);

			// Draw impact
			DrawImpact(hits[i]);
		}
		else
		{
			// Motion vector
			const Point Delta = capsule_sweep_data[i].mMaxDist * capsule_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(CapsuleStartPos, CapsuleStartPos + Delta, color);

			// Draw wireframe capsule at end position
//			DrawWireframeCapsule(Capsule.mRadius, M*0.5f, Pose, Delta, color);
			DrawWireframeCapsule(Capsule.mRadius, M, Pose, Delta, color);
		}
	}
}

void SetUserDefinedPolygonMode();
static void DrawWireframeConvex(PintShapeRenderer* renderer, const PR& pose, const Point& color)
{
	glColor4f(color.x, color.y, color.z, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		renderer->Render(pose);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	SetUserDefinedPolygonMode();
}

void BasicRenderer::DrawConvexSweepData(udword nb, const PintConvexSweepData* convex_sweep_data, const PintRaycastHit* hits, const Point& color)
{
	if(!nb || !convex_sweep_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		// Draw solid convex at initial position
		const Point& ConvexStartPos = convex_sweep_data[i].mTransform.mPos;

		glColor4f(color.x, color.y, color.z, 1.0f);
		convex_sweep_data[i].mRenderer->Render(convex_sweep_data[i].mTransform);

		if(hits[i].mObject)
		{
			// Motion vector
			const Point Delta = hits[i].mDistance * convex_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(ConvexStartPos, ConvexStartPos + Delta, color);

			// Draw wireframe convex at impact position
			PR EndPose = convex_sweep_data[i].mTransform;
			EndPose.mPos += Delta;
			DrawWireframeConvex(convex_sweep_data[i].mRenderer, EndPose, color);

			// Draw impact
			DrawImpact(hits[i]);
		}
		else
		{
			// Motion vector
			const Point Delta = convex_sweep_data[i].mMaxDist * convex_sweep_data[i].mDir;

			// Draw motion vector
			::DrawLine(ConvexStartPos, ConvexStartPos + Delta, color);

			// Draw wireframe convex at end position
			PR EndPose = convex_sweep_data[i].mTransform;
			EndPose.mPos += Delta;
			DrawWireframeConvex(convex_sweep_data[i].mRenderer, EndPose, color);
		}
	}
}

void BasicRenderer::DrawSphereOverlapAnyData(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintBooleanHit* hits, const Point& color)
{
	if(!nb || !sphere_overlap_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		const Point& SphereStartPos = sphere_overlap_data[i].mSphere.mCenter;
		const PR Pose(SphereStartPos, Quat(1.0f, 0.0f, 0.0f, 0.0f));
/*		glColor4f(color.x, color.y, color.z, 1.0f);
		::DrawLine(SphereStartPos, SphereStartPos + Point(0.0f, 100.0f, 0.0f), color);

		if(hits[i])
		{
			::DrawSphere(sphere_overlap_data[i].mSphere.mRadius, Pose);
		}
		else
		{
			DrawWireframeSphere(sphere_overlap_data[i].mSphere, Point(0.0f, 0.0f, 0.0f), color);
		}*/
		Point Color;
		if(hits[i].mHit)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			Color = Point(1.0f, 0.0f, 0.0f);
		}
		else
		{
			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			Color = Point(0.0f, 1.0f, 0.0f);
		}
//		if(mFrameNumber&16)
//			::DrawSphere(sphere_overlap_data[i].mSphere.mRadius, Pose);
//		else
			DrawWireframeSphere(sphere_overlap_data[i].mSphere, Point(0.0f, 0.0f, 0.0f), Color);
	}
}

void BasicRenderer::DrawSphereOverlapObjectsData(udword nb, const PintSphereOverlapData* sphere_overlap_data, const PintOverlapObjectHit* hits, const Point& color)
{
	if(!nb || !sphere_overlap_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		const Point& SphereStartPos = sphere_overlap_data[i].mSphere.mCenter;
		const PR Pose(SphereStartPos, Quat(1.0f, 0.0f, 0.0f, 0.0f));
/*		glColor4f(color.x, color.y, color.z, 1.0f);
		::DrawLine(SphereStartPos, SphereStartPos + Point(0.0f, 100.0f, 0.0f), color);

		if(hits[i])
		{
			::DrawSphere(sphere_overlap_data[i].mSphere.mRadius, Pose);
		}
		else
		{
			DrawWireframeSphere(sphere_overlap_data[i].mSphere, Point(0.0f, 0.0f, 0.0f), color);
		}*/

		Point Color;
		if(hits[i].mNbObjects)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			Color = Point(1.0f, 0.0f, 0.0f);
		}
		else
		{
			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			Color = Point(0.0f, 1.0f, 0.0f);
		}
//		if(mFrameNumber&16)
//			::DrawSphere(sphere_overlap_data[i].mSphere.mRadius, Pose);
//		else
			DrawWireframeSphere(sphere_overlap_data[i].mSphere, Point(0.0f, 0.0f, 0.0f), Color);
	}
}

void BasicRenderer::DrawBoxOverlapAnyData(udword nb, const PintBoxOverlapData* box_overlap_data, const PintBooleanHit* hits, const Point& color)
{
	if(!nb || !box_overlap_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		const Point& BoxStartPos = box_overlap_data[i].mBox.mCenter;
		const PR Pose(BoxStartPos, Quat(box_overlap_data[i].mBox.mRot));
/*		glColor4f(color.x, color.y, color.z, 1.0f);

		if(hits[i])
		{
			::DrawBox(box_overlap_data[i].mBox.mExtents, Pose);
		}
		else
		{
			DrawWireframeBox(box_overlap_data[i].mBox, Point(0.0f, 0.0f, 0.0f), color);
		}*/

		Point Color;
		if(hits[i].mHit)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			Color = Point(1.0f, 0.0f, 0.0f);
		}
		else
		{
			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			Color = Point(0.0f, 1.0f, 0.0f);
		}
		DrawWireframeBox(box_overlap_data[i].mBox, Point(0.0f, 0.0f, 0.0f), Color);
	}
}

void BasicRenderer::DrawBoxOverlapObjectsData(udword nb, const PintBoxOverlapData* box_overlap_data, const PintOverlapObjectHit* hits, const Point& color)
{
	if(!nb || !box_overlap_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
		const Point& BoxStartPos = box_overlap_data[i].mBox.mCenter;
		const PR Pose(BoxStartPos, Quat(box_overlap_data[i].mBox.mRot));
/*		glColor4f(color.x, color.y, color.z, 1.0f);

		if(hits[i])
		{
			::DrawBox(box_overlap_data[i].mBox.mExtents, Pose);
		}
		else
		{
			DrawWireframeBox(box_overlap_data[i].mBox, Point(0.0f, 0.0f, 0.0f), color);
		}*/
		Point Color;
		if(hits[i].mNbObjects)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			Color = Point(1.0f, 0.0f, 0.0f);
		}
		else
		{
			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			Color = Point(0.0f, 1.0f, 0.0f);
		}
		DrawWireframeBox(box_overlap_data[i].mBox, Point(0.0f, 0.0f, 0.0f), Color);
	}
}

void BasicRenderer::DrawCapsuleOverlapAnyData(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintBooleanHit* hits, const Point& color)
{
	if(!nb || !capsule_overlap_data || !hits)
		return;

	for(udword i=0;i<nb;i++)
	{
			// Draw solid capsule at initial position
			const LSS& Capsule = capsule_overlap_data[i].mCapsule;
			const Point CapsuleStartPos = (Capsule.mP0 + Capsule.mP1)*0.5f;

			Point CapsuleAxis = Capsule.mP1 - Capsule.mP0;
			const float M = CapsuleAxis.Magnitude();
			CapsuleAxis /= M;
			const Quat q = ShortestRotation(Point(1.0f, 0.0f, 0.0f), CapsuleAxis);

				const Quat qq = ShortestRotation(Point(0.0f, 1.0f, 0.0f), Point(1.0f, 0.0f, 0.0f));
				Quat qqq = q * qq;

			const PR Pose(CapsuleStartPos, qqq);

		Point Color;
		if(hits[i].mHit)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			Color = Point(1.0f, 0.0f, 0.0f);
		}
		else
		{
			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			Color = Point(0.0f, 1.0f, 0.0f);
		}
		DrawWireframeCapsule(Capsule.mRadius, M, Pose, Point(0.0f, 0.0f, 0.0f), Color);
	}
}

void BasicRenderer::DrawCapsuleOverlapObjectsData(udword nb, const PintCapsuleOverlapData* capsule_overlap_data, const PintOverlapObjectHit* hits, const Point& color)
{
}
