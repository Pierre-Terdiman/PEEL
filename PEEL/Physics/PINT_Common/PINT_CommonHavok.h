///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_COMMON_HAVOK_H
#define PINT_COMMON_HAVOK_H

	inline_ Point			ToPoint(const hkVector4& p)		{ return Point(p(0), p(1), p(2));											}
	inline_ Quat			ToQuat(const hkQuaternion& q)	{ return Quat(q(3), q(0), q(1), q(2));										}
	inline_ Quat			ToQuat(const hkRotation& q)		{ return ToQuat(hkQuaternion(q));											}
	inline_ hkVector4		ToHkVector4(const Point& p)		{ return hkVector4(p.x, p.y, p.z);											}
	inline_ hkQuaternion	ToHkQuaternion (const Quat& q)	{ return hkQuaternion(q.p.x, q.p.y, q.p.z, q.w);							}
	inline_ PR				ToPR(const hkTransform& pose)	{ return PR(ToPoint(pose.getTranslation()), ToQuat(pose.getRotation()));	}

// ### this is a bit bonkers!
/*#define HAVOK_RIGIDBODY_FROM_BOX(obb)							\
	const hkpBoxShape BoxShape(ToHkVector4(obb.mExtents), 0.0f);\
																\
	hkpRigidBodyCinfo ci;										\
	ci.m_shape		= &BoxShape;								\
	ci.m_motionType	= hkpMotion::MOTION_FIXED;					\
	ci.m_rotation	= ToHkQuaternion(Quat(obb.mRot));			\
	ci.m_position	= ToHkVector4(obb.mCenter);					\
	hkpRigidBody rigidBody(ci);*/

#define HAVOK_COLLIDABLE_FROM_BOX(obb)													\
	const hkpBoxShape BoxShape(ToHkVector4(obb.mExtents), 0.0f);						\
	const hkTransform Pose(ToHkQuaternion(Quat(obb.mRot)), ToHkVector4(obb.mCenter));	\
	const hkpCollidable Collidable(&BoxShape, &Pose);

// ### this is a bit bonkers!
/*#define HAVOK_RIGIDBODY_FROM_SPHERE(sphere)						\
	const hkpSphereShape SphereShape(sphere.mRadius);			\
																\
	hkpRigidBodyCinfo ci;										\
	ci.m_shape		= &SphereShape;								\
	ci.m_motionType	= hkpMotion::MOTION_FIXED;					\
	ci.m_rotation	= hkQuaternion(0.0f, 0.0f, 0.0f, 1.0f);		\
	ci.m_position	= ToHkVector4(sphere.mCenter);				\
	hkpRigidBody rigidBody(ci);*/

#define HAVOK_COLLIDABLE_FROM_SPHERE(sphere)													\
	const hkpSphereShape SphereShape(sphere.mRadius);											\
	const hkTransform Pose(hkQuaternion(0.0f, 0.0f, 0.0f, 1.0f), ToHkVector4(sphere.mCenter));	\
	const hkpCollidable Collidable(&SphereShape, &Pose);

// ### this is a bit bonkers!
/*#define HAVOK_RIGIDBODY_FROM_CAPSULE(capsule, center)																			\
	const hkpCapsuleShape CapsuleShape(ToHkVector4(capsule.mP0 - center), ToHkVector4(capsule.mP1 - center), capsule.mRadius);	\
																																\
	hkpRigidBodyCinfo ci;																										\
	ci.m_shape		= &CapsuleShape;																							\
	ci.m_motionType	= hkpMotion::MOTION_FIXED;																					\
	ci.m_rotation	= hkQuaternion(0.0f, 0.0f, 0.0f, 1.0f);																		\
	ci.m_position	= ToHkVector4(center);																						\
	hkpRigidBody rigidBody(ci);*/

#define HAVOK_COLLIDABLE_FROM_CAPSULE(capsule, center)																			\
	const hkpCapsuleShape CapsuleShape(ToHkVector4(capsule.mP0 - center), ToHkVector4(capsule.mP1 - center), capsule.mRadius);	\
	const hkTransform Pose(hkQuaternion(0.0f, 0.0f, 0.0f, 1.0f), ToHkVector4(center));											\
	const hkpCollidable Collidable(&CapsuleShape, &Pose);

	struct InternalCapsuleShape
	{
		hkpCapsuleShape*	mShape;
		float				mRadius;
		float				mHalfHeight;
	};

	hkpSphereShape*			FindSphereShape	(bool share_shapes, std::vector<hkpSphereShape*>& shapes,			const PINT_SPHERE_CREATE& sphere_create);
	hkpCapsuleShape*		FindCapsuleShape(bool share_shapes, std::vector<InternalCapsuleShape>& shapes,		const PINT_CAPSULE_CREATE& capsule_create);
	hkpBoxShape*			FindBoxShape	(bool share_shapes, std::vector<hkpBoxShape*>& shapes,				const PINT_BOX_CREATE& box_create);
	hkpConvexVerticesShape*	FindConvexShape	(bool share_shapes, std::vector<hkpConvexVerticesShape*>& shapes,	const PINT_CONVEX_CREATE& convex_create, const hkStridedVertices& strided_verts);

	enum HavokMeshFormat
	{
		HAVOK_BV_COMPRESSED_MESH_SHAPE,
		HAVOK_EXTENDED_MESH_SHAPE,
	};

//	hkpShape* CreateMeshShape(const PINT_MESH_CREATE& create, HavokMeshFormat format);

	struct HavokMeshRender
	{
		HavokMeshRender(hkpShape* shape, PintShapeRenderer* renderer) :
			mTriangleMesh	(shape),
			mRenderer		(renderer)
		{
		}
		hkpShape*			mTriangleMesh;
		PintShapeRenderer*	mRenderer;
	};
	hkpShape* CreateMeshShape(const PINT_MESH_CREATE& create, HavokMeshFormat format, std::vector<HavokMeshRender>& meshes, PintShapeRenderer* renderer);

#endif