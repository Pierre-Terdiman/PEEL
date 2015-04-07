///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// WARNING: this file is compiled by all Havok plug-ins, so put only the code here that is "the same" for all versions.

#include "stdafx.h"
#include "PINT_CommonHavok.h"

bool Havok::SetKinematicPose(PintObjectHandle handle, const Point& pos)
{
	hkpRigidBody* RB = (hkpRigidBody*)handle;

	hkpKeyFrameUtility::applyHardKeyFrame(ToHkVector4(pos), hkQuaternion::getIdentity(), 60.0f, RB);
	return true;
}

static inline_ void FillResultStruct(PintRaycastHit& hit, const hkpClosestCdPointCollector& result, float max_dist)
{
	const hkpRootCdPoint& Hit = result.getHit();
	const hkpCollidable* touched = Hit.m_rootCollidableA;
	hit.mObject			= (hkpRigidBody*)touched->getOwner();	// ###
	hit.mImpact			= ToPoint(Hit.m_contact.getPosition());
	hit.mNormal			= ToPoint(Hit.m_contact.getNormal());
	hit.mDistance		= Hit.m_contact.getDistance() * max_dist;
	hit.mTriangleIndex	= INVALID_ID;
}

udword Havok::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	// Since we're not too concerned with perfect accuracy, we can set the early out
	// distance to give the algorithm a chance to exit more quickly:
//	mPhysicsWorld->getCollisionInput()->m_config->m_iterativeLinearCastEarlyOutDistance = 0.1f;

	hkpClosestCdPointCollector castCollector;
	udword NbHits = 0;
	while(nb--)
	{
		hkpLinearCastInput input;
		input.m_to = ToHkVector4(sweeps->mBox.mCenter + sweeps->mDir*sweeps->mMaxDist);

		const hkpBoxShape BoxShape(ToHkVector4(sweeps->mBox.mExtents), 0.0f);
		const hkTransform Pose(ToHkQuaternion(Quat(sweeps->mBox.mRot)), ToHkVector4(sweeps->mBox.mCenter));
		hkpCollidable Collidable(&BoxShape, &Pose);

		castCollector.reset();
		mPhysicsWorld->linearCast(&Collidable, input, castCollector);

		if(castCollector.hasHit())
		{
			NbHits++;
			FillResultStruct(*dest, castCollector, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}
		sweeps++;
		dest++;
	}

	return NbHits;
}

udword Havok::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	hkpClosestCdPointCollector castCollector;
	udword NbHits = 0;
	while(nb--)
	{
		hkpLinearCastInput input;
		input.m_to = ToHkVector4(sweeps->mSphere.mCenter + sweeps->mDir*sweeps->mMaxDist);

		HAVOK_COLLIDABLE_FROM_SPHERE(sweeps->mSphere)

		castCollector.reset();
		mPhysicsWorld->linearCast(&Collidable, input, castCollector);

		if(castCollector.hasHit())
		{
			NbHits++;
			FillResultStruct(*dest, castCollector, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}
		sweeps++;
		dest++;
	}
	return NbHits;
}

udword Havok::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	hkpClosestCdPointCollector castCollector;
	udword NbHits = 0;
	while(nb--)
	{
		const Point Center = (sweeps->mCapsule.mP0 + sweeps->mCapsule.mP1)*0.5f;

		hkpLinearCastInput input;
		input.m_to = ToHkVector4(Center + sweeps->mDir*sweeps->mMaxDist);

		HAVOK_COLLIDABLE_FROM_CAPSULE(sweeps->mCapsule, Center)

		castCollector.reset();
		mPhysicsWorld->linearCast(&Collidable, input, castCollector);

		if(castCollector.hasHit())
		{
			NbHits++;
			FillResultStruct(*dest, castCollector, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}
		sweeps++;
		dest++;
	}
	return NbHits;
}

udword Havok::CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)
{
	hkStridedVertices StridedVerts;
	StridedVerts.m_numVertices	= desc.mNbVerts;
	StridedVerts.m_striding		= sizeof(Point);
	StridedVerts.m_vertices		= &desc.mVerts->x;

//	hkpConvexVerticesShape* shape = FindConvexShape(gShareShapes, mConvexShapes, *ConvexCreate, StridedVerts);

	hkpConvexVerticesShape* shape = new hkpConvexVerticesShape(StridedVerts);
	ASSERT(shape);

	if(desc.mRenderer)
		shape->setUserData(hkUlong(desc.mRenderer));

	const udword CurrentSize = mConvexObjects.size();
	mConvexObjects.push_back(shape);
	return CurrentSize;

/*	btConvexHullShape* shape = new btConvexHullShape(&desc.mVerts->x, desc.mNbVerts, sizeof(Point));
	ASSERT(shape);

	if(desc.mRenderer)
		shape->setUserPointer(desc.mRenderer);

	const udword CurrentSize = mConvexObjects.size();
	mConvexObjects.push_back(shape);
	return CurrentSize;*/
}

/*
			hkStridedVertices StridedVerts;
			StridedVerts.m_numVertices	= ConvexCreate->mNbVerts;
			StridedVerts.m_striding		= sizeof(Point);
			StridedVerts.m_vertices		= &ConvexCreate->mVerts->x;

			hkpConvexVerticesShape* shape = FindConvexShape(gShareShapes, mConvexShapes, *ConvexCreate, StridedVerts);
			refs.pushBack(shape);

//			sphere->addReference();
			hkpConvexTransformShape* convexTransform = new hkpConvexTransformShape(shape, LocalPose);
			ASSERT(convexTransform);
			shapeArray.pushBack(convexTransform);
*/

udword Havok::BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	hkpClosestCdPointCollector castCollector;
	udword NbHits = 0;
	while(nb--)
	{
		hkpConvexVerticesShape* ConvexShape = mConvexObjects[sweeps->mConvexObjectIndex];

		const Point& center = sweeps->mTransform.mPos;
		const Quat& q = sweeps->mTransform.mRot;

//		const Point Center = (sweeps->mCapsule.mP0 + sweeps->mCapsule.mP1)*0.5f;

		hkpLinearCastInput input;
		input.m_to = ToHkVector4(center + sweeps->mDir*sweeps->mMaxDist);

//		HAVOK_COLLIDABLE_FROM_CAPSULE(sweeps->mCapsule, Center)

//	const hkpConvexShape CapsuleShape(ToHkVector4(capsule.mP0 - center), ToHkVector4(capsule.mP1 - center), capsule.mRadius);
	const hkTransform Pose(ToHkQuaternion(q), ToHkVector4(center));
	const hkpCollidable Collidable(ConvexShape, &Pose);

		castCollector.reset();
		mPhysicsWorld->linearCast(&Collidable, input, castCollector);

		if(castCollector.hasHit())
		{
			NbHits++;
			FillResultStruct(*dest, castCollector, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}
		sweeps++;
		dest++;
	}
	return NbHits;
}

udword Havok::BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mPhysicsWorld);

//	hkpFirstCdBodyPairCollector collector;
	hkpFlagCdBodyPairCollector collector;

	udword NbHits = 0;
	while(nb--)
	{
		HAVOK_COLLIDABLE_FROM_SPHERE(overlaps->mSphere)

		collector.reset();
		mPhysicsWorld->getPenetrations(&Collidable, *mPhysicsWorld->getCollisionInput(), collector);
		if(collector.hasHit())
		{
			NbHits++;
			dest->mHit = true;
		}
		else
		{
			dest->mHit = false;
		}

		overlaps++;
		dest++;
	}
	return NbHits;
}

// "Those are the only 3 collectors we provide but you can create your own custom collector,
// it will just need to inherit from hkpCdBodyPairCollector and implement an addCdBodyPair(...) function."

class ObjectsBodyPairCollector : public hkpCdBodyPairCollector
{
	public:
//		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_AGENT, hkpAllCdBodyPairCollector);

				ObjectsBodyPairCollector() : mNbHits(0)
				{
//					reset();
				}

				virtual ~ObjectsBodyPairCollector(){}

		virtual void reset()
		{
			mNbHits = 0;
//			m_hits.clear();
			hkpCdBodyPairCollector::reset();
		}

/*		inline const hkArray<hkpRootCdBodyPair>& getHits() const
		{
			return m_hits;
		}*/

		virtual void addCdBodyPair( const hkpCdBody& bodyA, const hkpCdBody& bodyB )
		{
			mNbHits++;
		}

//		hkInplaceArray<hkpRootCdBodyPair, 16> m_hits;
		udword	mNbHits;
};

udword Havok::BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)
{
	ASSERT(mPhysicsWorld);

//	hkpFirstCdBodyPairCollector collector;
	hkpAllCdBodyPairCollector collector;
//	ObjectsBodyPairCollector collector;

	udword NbHits = 0;
	while(nb--)
	{
		HAVOK_COLLIDABLE_FROM_SPHERE(overlaps->mSphere)

		collector.reset();
		mPhysicsWorld->getPenetrations(&Collidable, *mPhysicsWorld->getCollisionInput(), collector);
		udword Nb = collector.getHits().getSize();
//		udword Nb = collector.mNbHits;
		NbHits += Nb;
		dest->mNbObjects = Nb;

		overlaps++;
		dest++;
	}
	return NbHits;
}

PR Havok::GetWorldTransform(PintObjectHandle handle)
{
	hkpRigidBody* RB = (hkpRigidBody*)handle;

	const hkTransform& BodyPose = RB->getTransform();

	return ToPR(BodyPose);
}

void Havok::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	hkpRigidBody* RB = (hkpRigidBody*)handle;
	if(action_type==PINT_ACTION_FORCE)
	{
		RB->applyForce(1.0f/60.0f, ToHkVector4(action), ToHkVector4(pos));
	}
	else if(action_type==PINT_ACTION_IMPULSE)
	{
		RB->applyPointImpulseAsCriticalOperation(ToHkVector4(action), ToHkVector4(pos));
	}
	else ASSERT(0);
}








hkpSphereShape* FindSphereShape(bool share_shapes, std::vector<hkpSphereShape*>& shapes, const PINT_SPHERE_CREATE& sphere_create)
{
	const float Radius = sphere_create.mRadius;
	if(share_shapes)
	{
		const int size = shapes.size();
		for(int i=0;i<size;i++)
		{
			hkpSphereShape* CurrentShape = shapes[i];
			if(CurrentShape->getRadius()==Radius)
			{
				CurrentShape->addReference();
				return CurrentShape;
			}
		}
	}

	hkpSphereShape* shape = new hkpSphereShape(Radius);
	ASSERT(shape);
	shapes.push_back(shape);

	if(sphere_create.mRenderer)
		shape->setUserData(hkUlong(sphere_create.mRenderer));

	return shape;
}

hkpCapsuleShape* FindCapsuleShape(bool share_shapes, std::vector<InternalCapsuleShape>& shapes, const PINT_CAPSULE_CREATE& capsule_create)
{
	const float Radius = capsule_create.mRadius;
	const float HalfHeight = capsule_create.mHalfHeight;
	if(share_shapes)
	{
		const int size = shapes.size();
		for(int i=0;i<size;i++)
		{
			const InternalCapsuleShape& CurrentShape = shapes[i];
			if(CurrentShape.mRadius==Radius && CurrentShape.mHalfHeight==HalfHeight)
			{
				CurrentShape.mShape->addReference();
				return CurrentShape.mShape;
			}
		}
	}

	const hkVector4 vertexA(0.0f, HalfHeight, 0.0f);
	const hkVector4 vertexB(0.0f, -HalfHeight, 0.0f);

	hkpCapsuleShape* shape = new hkpCapsuleShape(vertexA, vertexB, capsule_create.mRadius);
	ASSERT(shape);

	InternalCapsuleShape ICS;
	ICS.mShape		= shape;
	ICS.mRadius		= Radius;
	ICS.mHalfHeight	= HalfHeight;
	shapes.push_back(ICS);

	if(capsule_create.mRenderer)
		shape->setUserData(hkUlong(capsule_create.mRenderer));

	return shape;
}

hkpBoxShape* FindBoxShape(bool share_shapes, std::vector<hkpBoxShape*>& shapes, const PINT_BOX_CREATE& box_create)
{
	const hkVector4 halfExtents = ToHkVector4(box_create.mExtents);
	if(share_shapes)
	{
		const int size = shapes.size();
		for(int i=0;i<size;i++)
		{
			hkpBoxShape* CurrentShape = shapes[i];
			const hkVector4& CurrentExtents = CurrentShape->getHalfExtents();
			if(		CurrentExtents(0)==halfExtents(0)
				&&	CurrentExtents(1)==halfExtents(1)
				&&	CurrentExtents(2)==halfExtents(2))
			{
				CurrentShape->addReference();
				return CurrentShape;
			}
		}
	}
	hkpBoxShape* shape = new hkpBoxShape(halfExtents, 0);
	ASSERT(shape);
	shapes.push_back(shape);

	if(box_create.mRenderer)
		shape->setUserData(hkUlong(box_create.mRenderer));

	return shape;
}

hkpConvexVerticesShape* FindConvexShape(bool share_shapes, std::vector<hkpConvexVerticesShape*>& shapes, const PINT_CONVEX_CREATE& convex_create, const hkStridedVertices& strided_verts)
{
	if(share_shapes && convex_create.mRenderer)
	{
		const int size = shapes.size();
		for(int i=0;i<size;i++)
		{
			hkpConvexVerticesShape* CurrentShape = shapes[i];
			if(CurrentShape->getUserData()==hkUlong(convex_create.mRenderer))
			{
				CurrentShape->addReference();
				return CurrentShape;
			}
		}
	}

	hkpConvexVerticesShape* shape = new hkpConvexVerticesShape(strided_verts);
	ASSERT(shape);
	shapes.push_back(shape);

	if(convex_create.mRenderer)
		shape->setUserData(hkUlong(convex_create.mRenderer));

	return shape;
}




#ifndef HAVOK660
static hkpShape* CreateMeshShape(const PINT_MESH_CREATE& create, HavokMeshFormat format)
{
	// ### share meshes?
	const udword NbVerts = create.mSurface.mNbVerts;
	const Point* V = create.mSurface.mVerts;

	const udword NbTris = create.mSurface.mNbFaces;
	const udword* I = create.mSurface.mDFaces;

	hkpShape* HavokShape=null;
	if(format==HAVOK_BV_COMPRESSED_MESH_SHAPE)
	{
		// "Since this shape stores all of its primitive geometries internally in a compressed format, it cannot be used to reference external geometry."
		hkGeometry geom;
		for(udword i=0;i<NbVerts;i++)
		{
//			geom.m_vertices[i] = hkVector4(V[i].x, V[i].y, V[i].z);
			geom.m_vertices.pushBack(hkVector4(V[i].x, V[i].y, V[i].z));
		}

		for(udword i=0;i<NbTris;i++)
		{
			hkGeometry::Triangle T;
			T.set(I[0], I[1], I[2], -1);
			I+=3;
//			geom.m_triangles[i] = T;
			geom.m_triangles.pushBack(T);
		}
		{
			hkpDefaultBvCompressedMeshShapeCinfo cinfo(&geom);
//			cinfo.m_maxConvexShapeError = 0.0f;
//	hkMemoryAllocator::MemoryStatistics Stats;
//	hkMemorySystem::getInstance().getHeapStatistics(Stats);
//	hkLong before = Stats.m_inUse;
			HavokShape = new hkpBvCompressedMeshShape(cinfo);
//	hkMemorySystem::getInstance().getHeapStatistics(Stats);
//	hkLong after = Stats.m_inUse;
//	printf("Mesh size: %d\n", after-before);
//			hkpBvCompressedMeshShape* tmp = new hkpBvCompressedMeshShape(cinfo);
//			HavokShape = tmp;
//			int test = tmp->calcSizeForSpu(CalcSizeForSpuInput(), 1000000000);
//			hkBool b = tmp->castRay( const hkpShapeRayCastInput& input, hkpShapeRayCastOutput& results ) const;
		}
	}
	else if(format==HAVOK_EXTENDED_MESH_SHAPE)
	{
		hkpExtendedMeshShape* m_mesh = new hkpExtendedMeshShape;
//		hkpBvCompressedMeshShape* m_mesh = new hkpBvCompressedMeshShape;
		
//		hkpStorageExtendedMeshShape* m_mesh = new hkpStorageExtendedMeshShape;

		// it is common to have a landscape with 0 convex radius (for each triangle)
		// and all moving objects with non zero radius.
//		m_mesh->setRadius(0.0f);

		{
			hkpExtendedMeshShape::TrianglesSubpart part;

			part.m_vertexBase			= &V->x;
			part.m_vertexStriding		= sizeof(Point);
			part.m_numVertices			= NbVerts;

			part.m_indexBase			= I;
			part.m_indexStriding		= sizeof(udword)*3;
			part.m_numTriangleShapes	= NbTris;
			part.m_stridingType			= hkpExtendedMeshShape::INDICES_INT32;

			m_mesh->addTrianglesSubpart(part);
		}

		hkpMoppCompilerInput mci;
		hkpMoppCode* m_code = hkpMoppUtility::buildCode(m_mesh, mci);

		HavokShape = new hkpMoppBvTreeShape(m_mesh, m_code);

		m_code->removeReference();
		m_mesh->removeReference();

		// ### TODO
//		hkResult res = m_mesh->computeWeldingInfo(shape, hkpWeldingUtility::WELDING_TYPE_CLOCKWISE);
//		hkResult res = m_mesh->computeWeldingInfo(shape, hkpWeldingUtility::WELDING_TYPE_ANTICLOCKWISE);
	}
	return HavokShape;
}

hkpShape* CreateMeshShape(const PINT_MESH_CREATE& create, HavokMeshFormat format, std::vector<HavokMeshRender>& meshes, PintShapeRenderer* renderer)
{
	if(/*gShareMeshData &&*/renderer)
	{
		const udword Size = meshes.size();
		for(udword i=0;i<Size;i++)
		{
			const HavokMeshRender& CurrentMesh = meshes[i];
			if(CurrentMesh.mRenderer==renderer)
			{
				return CurrentMesh.mTriangleMesh;
			}
		}
	}

	hkpShape* S = CreateMeshShape(create, format);
	meshes.push_back(HavokMeshRender(S, renderer));

	if(create.mRenderer)
		S->setUserData(hkUlong(create.mRenderer));

	return S;
}
#endif
