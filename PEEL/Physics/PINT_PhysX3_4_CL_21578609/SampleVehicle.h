// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2013 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef SAMPLE_VEHICLE_H
#define SAMPLE_VEHICLE_H

//#include "PhysXSample.h"
#include "SampleVehicle_ControlInputs.h"
#include "SampleVehicle_CameraController.h"
#include "SampleVehicle_VehicleController.h"
#include "SampleVehicle_VehicleManager.h"
#include "vehicle/PxVehicleTireFriction.h"


class SampleVehicle// : public PhysXSample
{

public:

											SampleVehicle(/*PhysXSampleApplication& app*/);
	virtual									~SampleVehicle();

	// Implements RAWImportCallback
//	virtual	void							newMesh(const RAWMesh&);

	// Implements SampleApplication
	virtual	void							onInit();
	virtual	void							onShutdown();

	virtual	void							onTickPreRender(PxF32 dtime);
	virtual	void							onTickPostRender(PxF32 dtime);

//	virtual void							onDigitalInputEvent(const SampleFramework::InputEvent& , bool val);
//	virtual void							onAnalogInputEvent(const SampleFramework::InputEvent& , float val);

	// Implements PhysXSampleApplication
	virtual	void							helpRender(PxU32 x, PxU32 y, PxU8 textAlpha);
	virtual	void							descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha);
//	virtual	void							customizeSample(SampleSetup&);
	virtual	void							customizeSceneDesc(PxSceneDesc&);
	virtual	void							customizeRender();
	virtual	void							onSubstep(PxF32 dtime);	
//	virtual void							collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents);

private:
			void							_onSubstep(PxF32 dtime);	

	SampleVehicle_ControlInputs		mControlInputs;
	SampleVehicle_CameraController	mCameraController;
	SampleVehicle_VehicleController	mVehicleController;

	enum
	{
		MAX_NUM_INDEX_BUFFERS = 16
	};

	//Materials
	PxVehicleDrivableSurfaceType	mVehicleDrivableSurfaceTypes[MAX_NUM_INDEX_BUFFERS];
	PxMaterial*						mStandardMaterials[MAX_NUM_INDEX_BUFFERS];
	PxMaterial*						mChassisMaterial;

//	RenderMaterial*					mTerrainMaterial;
//	RenderMaterial*					mRoadMaterial;
//	RenderMaterial*					mRoadIceMaterial;
//	RenderMaterial*					mRoadGravelMaterial;

	void							createStandardMaterials();

	// Vehicles

	PxTransform						mResetPose;

	SampleVehicle_VehicleManager	mVehicleManager;
//	std::vector<RenderMeshActor*>	mVehicleGraphics;
	PxU32							mPlayerVehicle;

	void							createVehicles();

	void							createTrack();
	void							addRenderMesh(PxF32* verts, PxU32 nVerts, PxU32* indices, PxU32 mIndices, PxU32 matID, PxU32 vertexStride);
	void							addMesh(PxRigidActor* actor, PxF32* verts, PxU32 nVerts, PxU32* indices, PxU32 mIndices, PxU32 materialIndex, const char* filename);
	void							createLandscapeMesh();

	//Debug render

	bool							mDebugRenderFlag;
#if PX_DEBUG_VEHICLE_ON
	PxU32							mDebugRenderActiveGraphChannelWheel;
	PxU32							mDebugRenderActiveGraphChannelEngine;
	PxVehicleTelemetryData*			mTelemetryData4W;
	PxVehicleTelemetryData*			mTelemetryData6W;
#endif
	void							setupTelemetryData();
	void							clearTelemetryData();

	void							drawWheels();
	void							drawVehicleDebug();
	void							drawHud();
	void							drawGraphsAndPrintTireSurfaceTypes(const PxVehicleWheels& focusVehicle, const PxVehicleWheelQueryResult& focusVehicleWheelQueryResults);
	void							drawFocusVehicleGraphsAndPrintTireSurfaces();

	bool							mFixCar;
	bool							mBackToStart;

	PxU32							mCameraMode;

	PxF32							mForwardSpeedHud;

	void							updateCameraController(const PxF32 dtime, PxScene& scene);
	void							updateVehicleController(const PxF32 dtime);
	void							updateVehicleManager(const PxF32 dtime, const PxVec3& gravity);

	void							resetFocusVehicleAtWaypoint();
	PxRigidDynamic*					getFocusVehicleRigidDynamicActor();
	bool							getFocusVehicleUsesAutoGears();
};

#endif
