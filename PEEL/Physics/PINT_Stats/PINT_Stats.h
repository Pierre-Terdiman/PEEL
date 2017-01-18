///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_STATS_H
#define PINT_STATS_H

#include "..\Pint.h"

	class Stats
	{
		public:
									Stats();

				udword				mNbStatics;
				udword				mNbDynamics;
				udword				mNbStaticCompounds;
				udword				mNbDynamicCompounds;
				udword				mNbJoints;
				udword				mNbBoxShapes;
				udword				mNbSphereShapes;
				udword				mNbCapsuleShapes;
				udword				mNbConvexShapes;
				udword				mNbMeshShapes;
				udword				mTotalNbVerts;
				udword				mTotalNbTris;
				udword				mNbAggregates;
				udword				mNbArticulations;
	};

	class StatsPint : public Pint
	{
		public:
									StatsPint();
		virtual						~StatsPint();

		// Pint
		virtual	const char*			GetName()				const	{ return "Scene stats";	}
		virtual	void				GetCaps(PintCaps& caps)	const;
		virtual	void				Init(const PINT_WORLD_CREATE& desc);
		virtual	void				SetGravity(const Point& gravity);
		virtual	void				Close();
		virtual	udword				Update(float dt);
		virtual	Point				GetMainColor();
		virtual	void				Render(PintRender& renderer);

		virtual	void				SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups){}
		virtual	PintObjectHandle	CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool				ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle		CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	void*				CreatePhantom(const AABB& box)																												{ return null;	}
		virtual	udword				BatchRaycastsPhantom(udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts, void**)												{ return 0;	}

		virtual	udword				BatchRaycasts				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)					{ return 0;	}
		virtual	udword				BatchRaycastAny				(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintRaycastData* raycasts)					{ return 0;	}
		virtual	udword				BatchRaycastAll				(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintRaycastData* raycasts)			{ return 0;	}

		virtual	udword				BatchBoxSweeps				(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)					{ return 0;	}
		virtual	udword				BatchSphereSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)				{ return 0;	}
		virtual	udword				BatchCapsuleSweeps			(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)				{ return 0;	}

		virtual	udword				BatchSphereOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintSphereOverlapData* overlaps)			{ return 0;	}
		virtual	udword				BatchSphereOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintSphereOverlapData* overlaps)		{ return 0;	}
		virtual	udword				BatchBoxOverlapAny			(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintBoxOverlapData* overlaps)				{ return 0;	}
		virtual	udword				BatchBoxOverlapObjects		(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintBoxOverlapData* overlaps)		{ return 0;	}
		virtual	udword				BatchCapsuleOverlapAny		(PintSQThreadContext context, udword nb, PintBooleanHit* dest, const PintCapsuleOverlapData* overlaps)			{ return 0;	}
		virtual	udword				BatchCapsuleOverlapObjects	(PintSQThreadContext context, udword nb, PintOverlapObjectHit* dest, const PintCapsuleOverlapData* overlaps)	{ return 0;	}

		virtual	udword				FindTriangles_MeshSphereOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintSphereOverlapData* overlaps)	{ return 0;	}
		virtual	udword				FindTriangles_MeshBoxOverlap	(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintBoxOverlapData* overlaps)		{ return 0;	}
		virtual	udword				FindTriangles_MeshCapsuleOverlap(PintSQThreadContext context, PintObjectHandle handle, udword nb, const PintCapsuleOverlapData* overlaps)	{ return 0;	}

		virtual	PR					GetWorldTransform(PintObjectHandle handle)	{ return PR();	}
		virtual	void				SetWorldTransform(PintObjectHandle handle, const PR& pose)																{}

//		virtual	void				ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)			{}
		virtual	void				AddWorldImpulseAtWorldPos(PintObjectHandle handle, const Point& world_impulse, const Point& world_pos)					{}
		virtual	void				AddLocalTorque(PintObjectHandle handle, const Point& local_torque)														{}

		virtual	Point				GetAngularVelocity(PintObjectHandle handle)																				{ return Point(0.0f, 0.0f, 0.0f);	}
		virtual	void				SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity)												{}

		virtual	float				GetMass(PintObjectHandle handle)																						{ return 0.0f;	}
		virtual	Point				GetLocalInertia(PintObjectHandle handle)																				{ return Point(0.0f, 0.0f, 0.0f);	}

		virtual	udword				GetShapes(PintObjectHandle* shapes, PintObjectHandle handle)															{ return 0;	}
		virtual	void				SetLocalRot(PintObjectHandle handle, const Quat& q)																		{}

		virtual	bool				SetKinematicPose(PintObjectHandle handle, const Point& pos)	{ return false;	}
		virtual	bool				SetKinematicPose(PintObjectHandle handle, const PR& pr)		{ return false;	}

		virtual	udword				CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)																	{ return INVALID_ID;	}
		virtual	udword				BatchConvexSweeps	(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)	{ return 0;				}

		virtual	PintObjectHandle	CreateAggregate(udword max_size, bool enable_self_collision);
		virtual	bool				AddToAggregate(PintObjectHandle object, PintObjectHandle aggregate)														{ return false;	}
		virtual	bool				AddAggregateToScene(PintObjectHandle aggregate)																			{ return false;	}

		virtual	PintObjectHandle	CreateArticulation(const PINT_ARTICULATION_CREATE&);
		virtual	PintObjectHandle	CreateArticulatedObject(const PINT_OBJECT_CREATE&, const PINT_ARTICULATED_BODY_CREATE&, PintObjectHandle articulation);
		virtual	bool				AddArticulationToScene(PintObjectHandle articulation)																	{ return false;	}
		virtual	void				SetArticulatedMotor(PintObjectHandle object, const PINT_ARTICULATED_MOTOR_CREATE& motor)								{ }

		virtual	PintObjectHandle	CreateVehicle(PintVehicleData& data, const PINT_VEHICLE_CREATE& vehicle)												{ return null;	}
		virtual	void				SetVehicleInput(PintObjectHandle vehicle, const PINT_VEHICLE_INPUT& input)												{}

		// Return 0 to disable the raytracing window, etc
		virtual	udword				GetFlags()	const	{ return 0;	}
		//~Pint

		private:
				PINT_WORLD_CREATE	mCreate;
				Stats				mStats;
				bool				mUpdateUI;
	};

	IceWindow*	Stats_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		Stats_CloseGUI();
	void		Stats_Init(const PINT_WORLD_CREATE& desc);
	void		Stats_Close();
	StatsPint*	GetStats();

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif