///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_NEWTON_H
#define PINT_NEWTON_H

#include "..\Pint.h"
#include "Newton.h"
#include <vector>

class NewtonPint : public Pint
{
public:
	class PinkShapeMaterial
	{
	public:
		enum MaterialCombineModes
		{
			m_AVERAGE	= 0,		//!< Average: (a + b)/2
			m_MIN		= 1,		//!< Minimum: minimum(a,b)
			m_MULTIPLY	= 2,		//!< Multiply: a*b
		};

		PinkShapeMaterial ()
			:m_staticFriction (0.5f)
			,m_kineticFriction (0.5f)
			,m_restitution (0.0f)
			,m_frictionCombine(m_AVERAGE)
			,m_restitutionCombine(m_AVERAGE)
		{
		}

		float m_staticFriction;
		float m_kineticFriction;
		float m_restitution;
		MaterialCombineModes m_frictionCombine;
		MaterialCombineModes m_restitutionCombine;
	};

								NewtonPint();
	virtual						~NewtonPint();

	// Pint
	virtual	const char*			GetName() const;
	virtual	void				GetCaps(PintCaps& caps)	const;
	virtual	void				Init(const PINT_WORLD_CREATE& desc);
	virtual	void				SetGravity(const Point& gravity);
	virtual	void				Close();
	virtual	udword				Update(float dt);
	virtual	Point				GetMainColor();
	virtual	void				Render(PintRender& renderer);

	virtual	void				SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);

	virtual	PintObjectHandle	CreateObject(const PINT_OBJECT_CREATE& desc);
	virtual	bool				ReleaseObject(PintObjectHandle handle);
	virtual	PintJointHandle		CreateJoint(const PINT_JOINT_CREATE& desc);

	virtual	udword				BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
	virtual	udword				BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
	virtual	udword				BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
	virtual	udword				BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);

	virtual	udword				BatchSphereOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps);
	virtual	udword				BatchSphereOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps);
	virtual	udword				BatchBoxOverlapAny(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps);
	virtual	udword				BatchBoxOverlapObjects(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps);

	virtual	PR					GetWorldTransform(PintObjectHandle handle);
	virtual	void				SetWorldTransform(PintObjectHandle handle, const PR& pose);

//	virtual	void				ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);
	virtual	void				AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos);
	virtual	void				AddLocalTorque(PintObjectHandle handle, const Point& local_torque);

	virtual	Point				GetAngularVelocity(PintObjectHandle handle);
	virtual	void				SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity);

	virtual	float				GetMass(PintObjectHandle handle);
	virtual	Point				GetLocalInertia(PintObjectHandle handle);

	virtual	udword				CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc);
	virtual	udword				BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps);

	//JULIO
//	virtual	void*				BeginCreateObjectHint ();
//	virtual	void				EndCreateObjectHint (void* creationHintHandle);
//	virtual	void*				CreatePhantom(const AABB& box, void* creationHintHandle);
	virtual	PintObjectHandle	CreateObject(const PINT_OBJECT_CREATE& desc, void* creationHintHandle);
//	virtual void				SetLinearAndAngularDamp (PintJointHandle object, float linearDamp, float angularDamp);

	//~Pint
private:
	void						Render(PintRender& renderer, NewtonCollision* const collsion, const IceMaths::Matrix4x4& worldPose) const;

	NewtonBody*					CreateBody(const PINT_OBJECT_CREATE& desc, NewtonCollision* const shape, const dFloat* const worldPose, bool ccdMode);
	NewtonCollision*			CreateShape(const PINT_OBJECT_CREATE& desc) const;
	NewtonCollision*			CreateShape(NewtonWorld* const world, const PINT_SHAPE_CREATE* const shape, const PintShapeRenderer* const renderer, int groupID) const;

	virtual	bool				SetKinematicPose(PintObjectHandle handle, const Point& pos);
	virtual	bool				SetKinematicPose(PintObjectHandle handle, const PR& pr);

	static void					RayCastKernel (NewtonWorld* const world, void* const context, int threadIndex);
	static void					ConvexRayCastKernel (NewtonWorld* const world, void* const context, int threadIndex);
	static void					BodyOverlapKernel (NewtonWorld* const world, void* const context, int threadIndex);

	static int					BodyOverlapAnyCallback (const NewtonBody* const body, void* const userData);
	static int					BodyOverlapAllCallback (const NewtonBody* const body, void* const userData);
	static dFloat				GenericRayCastCallback (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam);

	static void					OnContactsProcess (const NewtonJoint* const contact, dFloat timestep, int threadIndex);
	static int					OnAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);

	static	void				ApplyForceAndTorqueCallback (const NewtonBody* const body, dFloat timestep, int threadIndex);

	static  void				DuplicateCollisionShape (const NewtonWorld* const newtonWorld, NewtonCollision* const newCollision, const NewtonCollision* const sourceCollision);
	static  void				DestroyCollisionShape (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision);

	static void					OnDestroyBody (const NewtonBody* const body);

//	virtual void				CreateFractureFromConvex (const char* const fileName, const Point* const convexCloud, int convexCloudCount, const Point* const interiorCloud, int interiorCloudCount);

	void UpdateQueiriesCache	(NewtonCollision* lastShapeCache);


	dFloat				mCurrentTimeStep;
	udword				mGroupMasks[32];
	NewtonWorld*		mWorld;
	Point				mGlobalGravity;
	PinkShapeMaterial	m_defailtMaterial;
	float				mGravityMag;
	// this is a cache for preventing the creation and destruction of shape on the test
	// this operation saddle newton with about 2500 ticks, making seen slower of the query.
	NewtonCollision*	m_lastShapeCache;

	struct BatchRaycastData
	{	
		int m_hitCount;
		PintRaycastHit* m_destArray;
		const PintRaycastData* m_raycasts;
	} ;
	BatchRaycastData	mBatchRaycastData;

	struct BatchSweepData
	{	
		enum ShapeSweepTypes
		{
			m_box,
			m_sphere,
			m_capsule,
			m_convex,
		};

		ShapeSweepTypes m_shapeType;
		int m_hitCount;
		const void* m_sweeps;
		PintRaycastHit* m_destArray;
		NewtonCollision* m_castingShape;
	};
	BatchSweepData mBatchSweepData;

	struct BatchOverlapData
	{	
		enum OverlapTypes
		{
			m_any,
			m_all,
		};

		enum ShapeOverlapTypes
		{
			m_box,
			m_sphere,
			//m_capsule,
		};


		OverlapTypes m_type;
		ShapeOverlapTypes m_shapeType;
		int m_hitCount;
		const void* m_overlaps;
		const void* m_destArray;
		NewtonCollision* m_overlapShape;
		int m_maxBodiesPerQueires;
		NewtonBody** m_hitBodiesBuffer;
	};
	BatchOverlapData mBatchOverlapData;

	struct KineData
	{
		NewtonBody*	mKine;
		Point		mKinematicTarget;
	};

//	std::vector<NewtonBody*>			mKinematics;
	std::vector<KineData>				mKinematics;

	struct KineData2
	{
		NewtonBody*	mKine;
		PR			mKinematicTarget;
	};

	std::vector<KineData2>				mKinematics2;

	void								UpdateKinematics(dFloat dt);
};


IceWindow*	Newton_InitGUI(IceWidget* parent, PintGUIHelper& helper);
void		Newton_CloseGUI();
void		Newton_Init(const PINT_WORLD_CREATE& desc);
void		Newton_Close();
NewtonPint*	GetNewton();

extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif
