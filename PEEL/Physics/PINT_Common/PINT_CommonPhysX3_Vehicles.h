///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_COMMON_PHYSX3_VEHICLES_H
#define PINT_COMMON_PHYSX3_VEHICLES_H

#include "PINT_CommonPhysX3.h"

#include "foundation/PxPreprocessor.h"
#include "common/PxPhysXCommonConfig.h"
#include "vehicle/PxVehicleSDK.h"
#include "vehicle/PxVehicleUpdate.h"
#include "PxScene.h"
#include "PxBatchQueryDesc.h"

#if PX_DEBUG_VEHICLE_ON
	#include "vehicle/PxVehicleUtilTelemetry.h"
#endif

	//Data structure to store reports for each wheel. 
	class SampleVehicleWheelQueryResults
	{
	public:

		//Allocate a buffer of wheel query results for up to maxNumWheels.
		static SampleVehicleWheelQueryResults* allocate(const PxU32 maxNumWheels);

		//Free allocated buffer.
		void free();

		PxWheelQueryResult* addVehicle(const PxU32 numWheels);

	private:

		//One result for each wheel.
		PxWheelQueryResult* mWheelQueryResults;

		//Maximum number of wheels.
		PxU32 mMaxNumWheels;

		//Number of wheels 
		PxU32 mNumWheels;


		SampleVehicleWheelQueryResults()
			: mWheelQueryResults(NULL),mMaxNumWheels(0), mNumWheels(0)
		{
			init();
		}

		~SampleVehicleWheelQueryResults()
		{
		}

		void init()
		{
			mWheelQueryResults=NULL;
			mMaxNumWheels=0;
			mNumWheels=0;
		}
	};

#ifdef PHYSX_SUPPORT_VEHICLE_SUSPENSION_SWEEPS
//	#define SETUP_FILTERING
	#define USE_SWEEPS
#else
//	#define SETUP_FILTERING
//	#define USE_SWEEPS
#endif
	#define NB_SWEEP_HITS_PER_WHEEL 1

	#ifdef SETUP_FILTERING
	//Make sure that suspension raycasts only consider shapes flagged as drivable that don't belong to the owner vehicle.
	enum
	{
		SAMPLEVEHICLE_DRIVABLE_SURFACE = 0xffff0000,
		SAMPLEVEHICLE_UNDRIVABLE_SURFACE = 0x0000ffff
	};

	static PxQueryHitType::Enum SampleVehicleWheelRaycastPreFilter(	
		PxFilterData filterData0, 
		PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		PxHitFlags& queryFlags)
	{
		//filterData0 is the vehicle suspension raycast.
		//filterData1 is the shape potentially hit by the raycast.
		PX_UNUSED(queryFlags);
		PX_UNUSED(constantBlockSize);
		PX_UNUSED(constantBlock);
		PX_UNUSED(filterData0);
		return ((0 == (filterData1.word3 & SAMPLEVEHICLE_DRIVABLE_SURFACE)) ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK);
	}


	//Set up query filter data so that vehicles can drive on shapes with this filter data.
	//Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
	void SampleVehicleSetupDrivableShapeQueryFilterData(PxFilterData* qryFilterData);

	//Set up query filter data so that vehicles cannot drive on shapes with this filter data.
	//Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
	void SampleVehicleSetupNonDrivableShapeQueryFilterData(PxFilterData* qryFilterData);

	//Set up query filter data for the shapes of a vehicle to ensure that vehicles cannot drive on themselves 
	//but can drive on the shapes of other vehicles.
	//Note that we have reserved word3 of the PxFilterData for vehicle raycast query filtering.
	void SampleVehicleSetupVehicleShapeQueryFilterData(PxFilterData* qryFilterData);
	#endif

	//Data structure for quick setup of scene queries for suspension raycasts.
	class SampleVehicleSceneQueryData
	{
	public:

		//Allocate scene query data for up to maxNumWheels suspension raycasts.
		static SampleVehicleSceneQueryData* allocate(const PxU32 maxNumWheels);

		//Free allocated buffer for scene queries of suspension raycasts.
		void free();

		//Create a PxBatchQuery instance that will be used as a single batched raycast of multiple suspension lines of multiple vehicles
		PxBatchQuery* setUpBatchedSceneQuery(PxScene* scene);

	#ifdef USE_SWEEPS
		//Get the buffer of scene query results that will be used by PxVehicleNWSuspensionRaycasts
		PxSweepQueryResult* getSweepQueryResultBuffer() {return mSqResults;}

		//Get the number of scene query results that have been allocated for use by PxVehicleNWSuspensionRaycasts
		PxU32 getSweepQueryResultBufferSize() const {return mNumQueries;}
	#else
		//Get the buffer of scene query results that will be used by PxVehicleNWSuspensionRaycasts
		PxRaycastQueryResult* getRaycastQueryResultBuffer() {return mSqResults;}

		//Get the number of scene query results that have been allocated for use by PxVehicleNWSuspensionRaycasts
		PxU32 getRaycastQueryResultBufferSize() const {return mNumQueries;}
	#endif


	#ifdef SETUP_FILTERING
		//Set the pre-filter shader 
		void setPreFilterShader(PxBatchQueryPreFilterShader preFilterShader) {mPreFilterShader=preFilterShader;}
	#endif

	private:

	#ifdef USE_SWEEPS
		//One result for each wheel.
		PxSweepQueryResult* mSqResults;

		//One hit for each wheel.
		PxSweepHit* mSqHitBuffer;
	#else
		//One result for each wheel.
		PxRaycastQueryResult* mSqResults;

		//One hit for each wheel.
		PxRaycastHit* mSqHitBuffer;
	#endif
		PxU32 mNbSqResults;

	#ifdef SETUP_FILTERING
		//Filter shader used to filter drivable and non-drivable surfaces
		PxBatchQueryPreFilterShader mPreFilterShader;
	#endif

		//Maximum number of suspension raycasts that can be supported by the allocated buffers 
		//assuming a single query and hit per suspension line.
		PxU32 mNumQueries;

		void init()
		{
	#ifdef SETUP_FILTERING
			mPreFilterShader=SampleVehicleWheelRaycastPreFilter;
	#endif
		}

		SampleVehicleSceneQueryData()
		{
			init();
		}

		~SampleVehicleSceneQueryData()
		{
		}
	};

	class SampleVehicle_VehicleController
	{
	public:

		SampleVehicle_VehicleController();
		~SampleVehicle_VehicleController();

		void setCarKeyboardInputs
			(const bool accel, const bool brake, const bool handbrake, 
			 const bool steerleft, const bool steerright, 
			 const bool gearup, const bool geardown)
		{
			mKeyPressedAccel=accel;
			mKeyPressedBrake=brake;
			mKeyPressedHandbrake=handbrake;
			mKeyPressedSteerLeft=steerleft;
			mKeyPressedSteerRight=steerright;
			mKeyPressedGearUp=gearup;
			mKeyPressedGearDown=geardown;
		}

		void setCarGamepadInputs
			(const PxF32 accel, const PxF32 brake, 
			 const PxF32 steer, 
			 const bool gearup, const bool geardown, 
			 const bool handbrake)
		{
			mGamepadAccel=accel;
			mGamepadCarBrake=brake;
			mGamepadCarSteer=steer;
			mGamepadGearup=gearup;
			mGamepadGeardown=geardown;
			mGamepadCarHandbrake=handbrake;
		}

		void toggleAutoGearFlag() 
		{
			mToggleAutoGears = true;
		}

		void update(const PxF32 dtime, const PxVehicleWheelQueryResult& vehicleWheelQueryResults, PxVehicleWheels& focusVehicle);

		void clear();

	private:

		//Raw driving inputs - keys (car + tank)
		bool			mKeyPressedAccel;
		bool			mKeyPressedGearUp;
		bool			mKeyPressedGearDown;

		//Raw driving inputs - keys (car only)
		bool			mKeyPressedBrake;
		bool			mKeyPressedHandbrake;
		bool			mKeyPressedSteerLeft;
		bool			mKeyPressedSteerRight;

		//Raw driving inputs - gamepad (car + tank)
		PxF32			mGamepadAccel;
		bool			mGamepadGearup;
		bool			mGamepadGeardown;

		//Raw driving inputs - gamepad (car only)
		PxF32			mGamepadCarBrake;
		PxF32			mGamepadCarSteer;
		bool			mGamepadCarHandbrake;

		// Raw data taken from the correct stream (live input stream or replay stream)
		bool			mUseKeyInputs;

		// Toggle autogears flag on focus vehicle
		bool			mToggleAutoGears;

		//Auto-reverse mode.
		bool			mIsMovingForwardSlowly;
		bool			mInReverseMode;

		//Update 
		void processRawInputs(const PxF32 timestep, const bool useAutoGears, PxVehicleDrive4WRawInputData& rawInputData);
		void processAutoReverse(
			const PxVehicleWheels& focusVehicle, const PxVehicleDriveDynData& driveDynData, const PxVehicleWheelQueryResult& vehicleWheelQueryResults,
			const PxVehicleDrive4WRawInputData& rawInputData, 
			bool& toggleAutoReverse, bool& newIsMovingForwardSlowly) const;
	};



	namespace physx
	{
		class PxScene;
		class PxBatchQuery;
		class PxCooking;
		class PxMaterial;
		class PxConvexMesh;
		struct PxVehicleDrivableSurfaceType;
	}

	class SampleVehicleSceneQueryData;
	class SampleVehicleWheelQueryResults;

	class PxVehicle4WAlloc;

	//Collision types and flags describing collision interactions of each collision type.
	enum
	{
		COLLISION_FLAG_GROUND			=	1 << 0,
		COLLISION_FLAG_WHEEL			=	1 << 1,
		COLLISION_FLAG_CHASSIS			=	1 << 2,
		COLLISION_FLAG_OBSTACLE			=	1 << 3,
		COLLISION_FLAG_DRIVABLE_OBSTACLE=	1 << 4,

		COLLISION_FLAG_GROUND_AGAINST	=															COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
		COLLISION_FLAG_WHEEL_AGAINST	=									COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE,
		COLLISION_FLAG_CHASSIS_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
		COLLISION_FLAG_OBSTACLE_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
		COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST=	COLLISION_FLAG_GROUND 						 |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	};

	//Id of drivable surface (used by suspension raycast filtering).
	enum
	{
		DRIVABLE_SURFACE_ID=0xffffffff
	};

	//Drivable surface types.
	enum
	{
		SURFACE_TYPE_MUD=0,
		SURFACE_TYPE_TARMAC,
		SURFACE_TYPE_SNOW,
		SURFACE_TYPE_GRASS,
		MAX_NUM_SURFACE_TYPES
	};
/*	static char gSurfaceTypes[MAX_NUM_SURFACE_TYPES+1][64]=
	{
		"mud",
		"tarmac",
		"ice",
		"grass",
	};*/

	//Tire types.
	enum
	{
		TIRE_TYPE_WETS=0,
		TIRE_TYPE_SLICKS,
		TIRE_TYPE_ICE,
		TIRE_TYPE_MUD,
		MAX_NUM_TIRE_TYPES
	};

	class SampleVehicle_VehicleManager
	{
	public:

		enum
		{
			MAX_NUM_4W_VEHICLES=8,
		};

		SampleVehicle_VehicleManager();
		~SampleVehicle_VehicleManager();

		void init(PxPhysics& physics, const PxMaterial** drivableSurfaceMaterials, const PxVehicleDrivableSurfaceType* drivableSurfaceTypes, const PINT_VEHICLE_CREATE& desc);

		void shutdown();

		//Create a vehicle ready to drive.
		PxVehicleDrive4W* create4WVehicle
			(PxScene& scene, PxPhysics& physics, PxCooking& cooking, const PxMaterial& material,
			 PxConvexMesh* chassisConvexMesh, PxConvexMesh** wheelConvexMeshes4,
			 bool useAutoGearFlag, const PINT_VEHICLE_CREATE& desc);

		PX_FORCE_INLINE	PxU32						getNbVehicles()					const	{ return mNumVehicles;		}
		PX_FORCE_INLINE	PxVehicleWheels*			getVehicle(const PxU32 i)				{ return mVehicles[i];		}
		PX_FORCE_INLINE const PxVehicleWheelQueryResult& getVehicleWheelQueryResults(const PxU32 i) const { return mVehicleWheelQueryResults[i]; }
//						void						addVehicle(const PxU32 i, PxVehicleWheels* vehicle);		

		//Start the suspension raycasts (always call before calling update)
		void suspensionRaycasts(PxScene* scene);

		//Update vehicle dynamics and compute forces/torques to apply to sdk rigid bodies.
	#if PX_DEBUG_VEHICLE_ON2
		void updateAndRecordTelemetryData(const PxF32 timestep, const PxVec3& gravity, PxVehicleWheels* focusVehicleNW, PxVehicleTelemetryData* telemetryDataNW);
	#else
		void update(const PxF32 timestep, const PxVec3& gravity);
	#endif

		//Reset the car back to its rest state with a specified transform.
//		void resetNWCar(const PxTransform& transform, const PxU32 vehicleId);

	private:

		//Array of all cars and report data for each car.
		PxVehicleWheels* mVehicles[MAX_NUM_4W_VEHICLES];
		PxVehicleWheelQueryResult mVehicleWheelQueryResults[MAX_NUM_4W_VEHICLES];
		PxU32 mNumVehicles;

		//sdk raycasts (for the suspension lines).
		SampleVehicleSceneQueryData* mSqData;
		PxBatchQuery* mSqWheelRaycastBatchQuery;

		//Reports for each wheel.
		SampleVehicleWheelQueryResults* mWheelQueryResults;

		//Cached simulation data of focus vehicle in 4W mode.
		PxVehicleDriveSimData4W mDriveSimData4W;

		//Friction from combinations of tire and surface types.
		PxVehicleDrivableSurfaceToTireFrictionPairs* mSurfaceTirePairs;

		//Initialise a car back to its start transform and state.
		void resetNWCar(const PxTransform& startTransform, PxVehicleWheels* car);
	};

	class SharedPhysX_Vehicles : public SharedPhysX
	{
		public:
									SharedPhysX_Vehicles(const EditableParams& params);
		virtual						~SharedPhysX_Vehicles();

		virtual	PintObjectHandle	CreateVehicle(PintVehicleData& data, const PINT_VEHICLE_CREATE& vehicle);
		virtual	void				SetVehicleInput(PintObjectHandle vehicle, const PINT_VEHICLE_INPUT& input);

				void				CloseVehicles();
				void				UpdateVehicles();
	};

#endif