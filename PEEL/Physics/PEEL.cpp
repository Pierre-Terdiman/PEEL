///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "Random.h"
#include "Render.h"
#include "GLFontRenderer.h"
#include "GLVSync.h"
#include "Pint.h"
#include "PintSQ.h"
#include "PintRender.h"
#include "PintTiming.h"
#include "PintObjectsManager.h"
#include "TestScenes.h"
#include "Camera.h"
#include "FileFinder.h"
#include "TrashCache.h"
#include "TitleWindow.h"
#include "RaytracingWindow.h"
#include "RaytracingTest.h"
#include "Script.h"
#include "RepX_Tools.h"
#include "TestSelector.h"
#include "CustomICEAllocator.h"
#include "GUI_Helpers.h"

#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

static const char* gVersion = "1.1";
enum RaytracingTestValue
{
	RAYTRACING_TEST_DISABLED,
	RAYTRACING_TEST_ST,
	RAYTRACING_TEST_MT,
};
static	RaytracingTestValue	gRaytracingTest = RAYTRACING_TEST_DISABLED;
static	int					gMainHandle = 0;
static	HWND				gWindowHandle = 0;
static	int					gMouseX = 0;
static	int					gMouseY = 0;
static	bool				gWireframe = false;
static	bool				gWireframeOverlay = true;
static	bool				gAutoCameraMove = false;
static	bool				gPaused = false;
static	bool				gOneFrame = false;
static	bool				gRender = true;
static	bool				gMenuIsVisible = true;
static	bool				gHelpIsVisible = false;
static	int					gCurrentTest = 0;
static	PhysicsTest*		gRunningTest = null;
static	PhysicsTest*		gCandidateTest = null;
static	bool				gRandomizeOrder = false;
static	bool				gTrashCache = false;
static	bool				gEnableVSync = true;
static	bool				gCommaSeparator = false;
static	bool				gDisplayMessage = false;
static	float				gDisplayMessageDelay = 0.0f;
static	udword				gDisplayMessageType = 0;
		float				gCameraSpeed = 0.2f;
//static	float				gRTDistance = MAX_FLOAT;	// Test how different engines react to "infinite" rays
static	float				gRTDistance = 5000.0f;
// TOOL_PICKING
static	float				gPickingForce = 1.0f;
// TOOL_ADD_IMPULSE
static	float				gImpulseMagnitude = 100.0f;
// TOOL_SHOOT_BOX
static	float				gBoxSize = 1.0f;
static	float				gBoxMass = 1.0f;
static	float				gBoxVelocity = 100.0f;
// TOOL_CAMERA_TRACKING
static	PintObjectHandle	gTrackedObject = null;
static	Pint*				gTrackedEngine = null;

enum ProfilingUnits
{
	PROFILING_UNITS_RDTSC,
	PROFILING_UNITS_TIME_GET_TIME,
	PROFILING_UNITS_QPC,
};
static	ProfilingUnits	gProfilingUnits = PROFILING_UNITS_RDTSC;

enum SQProfilingMode
{
	SQ_PROFILING_SIM,
	SQ_PROFILING_UPDATE,
	SQ_PROFILING_COMBINED,
};
static	SQProfilingMode	gSQProfilingMode = SQ_PROFILING_UPDATE;

enum SQRaycastMode
{
	SQ_RAYCAST_CLOSEST,
	SQ_RAYCAST_ANY,
	SQ_RAYCAST_ALL,
};
static	SQRaycastMode	gSQRaycastMode = SQ_RAYCAST_CLOSEST;
//extern bool gRaycastClosest;
extern udword gRaycastMode;

enum RaytracingResolution
{
	RT_WINDOW_64,
	RT_WINDOW_128,
	RT_WINDOW_256,
	RT_WINDOW_512,
};
static	RaytracingResolution	gRaytracingResolution = RT_WINDOW_128;

enum ToolIndex
{
	TOOL_PICKING,
	TOOL_ADD_IMPULSE,
	TOOL_SHOOT_BOX,
	TOOL_CAMERA_TRACKING,
//	TOOL_CUSTOM_CONTROL,
};
static	ToolIndex		gCurrentTool = TOOL_PICKING;

#define	INITIAL_SCREEN_WIDTH	768
#define	INITIAL_SCREEN_HEIGHT	768
#define	MAIN_GUI_POS_X			0
#define	MAIN_GUI_POS_Y			0
#define	MAIN_WINDOW_POS_X		100
#define	MAIN_WINDOW_POS_Y		100

static GLFontRenderer			gTexter;
/*static*/ udword				gScreenWidth	= INITIAL_SCREEN_WIDTH;
/*static*/ udword				gScreenHeight	= INITIAL_SCREEN_HEIGHT;
static FPS						gFPS;

#define MAX_NB_ENGINES			32
static udword					gFrameNb = 0;

class GUIHelper : public PintGUIHelper
{
	public:
	virtual	const char*		Convert(float value);

	virtual	IceWindow*		CreateMainWindow(Container*& gui, IceWidget* parent, udword id, const char* label);
	virtual	IceLabel*		CreateLabel		(IceWidget* parent,				sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner);
	virtual	IceCheckBox*	CreateCheckBox	(IceWidget* parent, udword id,	sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, bool state, CBCallback callback, const char* tooltip=null);
	virtual	IceEditBox*		CreateEditBox	(IceWidget* parent, udword id,	sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, EditBoxFilter filter, EBCallback callback, const char* tooltip=null);

}gGUIHelper;

struct CursorKeysState
{
	CursorKeysState() : mUp(false), mDown(false), mLeft(false), mRight(false)	{}
	void	Reset()
	{
		mUp = mDown = mLeft = mRight = false;
	}
	bool	mUp;
	bool	mDown;
	bool	mLeft;
	bool	mRight;
};

static CursorKeysState gState;

static void TestCSVExport();
static void PEEL_InitGUI();
static void PEEL_CloseGUI();
static void gPEEL_GetOptionsFromGUI();
static void gPEEL_PollRadioButtons();

struct EngineData
{
	EngineData() :
		mEngine					(null),
		mEnabled				(true),
		mSupportsCurrentTest	(true)
	{
		mDragPoint.Zero();
		mLocalPoint.Zero();
	}

	Pint*					mEngine;
	PintSQ					mSQHelper;
	ObjectsManager			mOMHelper;
	PintTiming				mTiming;
	PintRaycastHit			mPickingData;
	Point					mDragPoint;
	Point					mLocalPoint;
	bool					mEnabled;
	bool					mSupportsCurrentTest;
};
static EngineData			gEngines[MAX_NB_ENGINES];
static udword				gNbEngines = 0;
static udword				gNbSimulateCallsPerFrame = 1;
static float				gTimestep = 1.0f/60.0f;

class RaytracingWindow;
static PintPlugin*			gPlugIns[MAX_NB_ENGINES];
static RaytracingWindow*	gRaytracingWindows[MAX_NB_ENGINES] = {0};
static udword				gNbPlugIns = 0;

/*static*/ String*				gRoot = null;

static CameraData			gCamera;

typedef PintPlugin* (*GetPintPlugin)	();

static PintPlugin* LoadPlugIn(const char* filename)
{
/*	LIBRARY LibHandle;
	if(!IceCore::LoadLibrary(filename, LibHandle, true))
		return null;

	GetPintPlugin func = (GetPintPlugin)BindSymbol(LibHandle, "GetPintPlugin");
	if(!func)
	{
		UnloadLibrary(LibHandle);
		return null;
	}
	return (func)();*/

	udword FPUEnv[256];
	FillMemory(FPUEnv, 256*4, 0xff);
	__asm fstenv FPUEnv
		HMODULE handle = ::LoadLibraryA(filename);
	__asm fldenv FPUEnv
	if(!handle)
	{
		printf("WARNING: plugin %s failed to load.\n", filename);
		return null;
	}
	GetPintPlugin func = (GetPintPlugin)GetProcAddress(handle, "GetPintPlugin");
	if(!func)
	{
		printf("WARNING: plugin %s is invalid.\n", filename);
		FreeLibrary(handle);
		return null;
	}
	return (func)();
}

static void InitAll(PhysicsTest* test)
{
	PINT_WORLD_CREATE Desc;
	if(test)
	{
		bool MustResetCamera = false;

		static PhysicsTest* PreviousTest = null;
		if(test!=PreviousTest)
		{
			if(PreviousTest)
				PreviousTest->CloseUI();

			test->InitUI(gGUIHelper);
			PreviousTest = test;

			// We only reset the camera when changing scene. That way we let users re-start the test while
			// focusing on a specific part of the scene. They can always press 'C' to manually reset the
			// camera anyway.
			MustResetCamera = true;

			// Set focus back to render window, not to break people's habit of browsing the test scenes
			IceCore::SetFocus(gWindowHandle);
		}

		// We must get the scene params after initializing the UI
		test->GetSceneParams(Desc);

		class Access : public PINT_WORLD_CREATE
		{
			public:
			void SetName(const char* name)	{ mTestName = name; }
		};
		static_cast<Access&>(Desc).SetName(test->GetName());

		//### crude test - autodetect changes in camera data & reset camera if we found any
		if(!MustResetCamera)
		{
			CameraData Tmp;
			Tmp.Init(Desc);
			if(!(Tmp==gCamera))
				MustResetCamera = true;
		}

		if(MustResetCamera)
		{
			gCamera.Init(Desc);
			gCamera.Reset();
		}
	}
	else
	{
		Desc.mGravity	= Point(0.0f, -9.81f, 0.0f);
		//Desc.mGravity	= Point(0.0f, 0.0f, 0.0f);
	}

	for(udword i=0;i<gNbPlugIns;i++)
		gPlugIns[i]->Init(Desc);

	gNbSimulateCallsPerFrame = Desc.mNbSimulateCallsPerFrame;
	gTimestep = Desc.mTimestep;

	gNbEngines = 0;
	for(udword i=0;i<gNbPlugIns;i++)
	{
		ASSERT(gNbEngines!=MAX_NB_ENGINES);
		Pint* Engine = gPlugIns[i]->GetPint();
		ASSERT(Engine);
		gEngines[gNbEngines].mOMHelper.Init(Engine);
		gEngines[gNbEngines].mSQHelper.Init(Engine);
		gEngines[gNbEngines++].mEngine = Engine;
	}
	gPEEL_GetOptionsFromGUI();
}

static void CloseRunningTest()
{
	if(gRunningTest)
	{
		for(udword i=0;i<gNbEngines;i++)
		{
			ASSERT(gEngines[i].mEngine);
			gRunningTest->Close(*gEngines[i].mEngine);
		}
		gRunningTest->CommonRelease();
	}
	gTrackedObject = null;
	gTrackedEngine = null;
	gState.Reset();
}

static void ResetSQHelpersHitData()
{
	for(udword i=0;i<gNbEngines;i++)
		gEngines[i].mSQHelper.ResetHitData();
}

static void CloseAll()
{
	CloseRunningTest();

	for(udword i=0;i<gNbEngines;i++)
	{
		gEngines[i].mOMHelper.Reset();
		gEngines[i].mSQHelper.Reset();
	}

	for(udword i=0;i<gNbPlugIns;i++)
		gPlugIns[i]->Close();

	ReleaseAllShapeRenderers();
}

static void ResetTimers()
{
	gFrameNb = 0;
	for(udword i=0;i<gNbEngines;i++)
		gEngines[i].mTiming.ResetTimings();
}

static void ActivateTest(PhysicsTest* test=null)
{
	if(test)
		gCandidateTest = test;

	if(gCandidateTest)
	{
#ifdef PEEL_PUBLIC_BUILD
		if(gCandidateTest->IsPrivate())
		{
			gDisplayMessage = true;
			gDisplayMessageType = 1;
		}
		else
#endif
		{
			CloseAll();
			InitAll(gCandidateTest);

			gRunningTest = gCandidateTest;
			ResetTimers();
			gRunningTest->CommonSetup();
			for(udword i=0;i<gNbEngines;i++)
			{
				ASSERT(gEngines[i].mEngine);
				gEngines[i].mSupportsCurrentTest = gRunningTest->Init(*gEngines[i].mEngine);
			}

			gMenuIsVisible = false;
		}
	}
}

static Vertices* gCameraData = null;
static Point gLastCamPos(0.0f, 0.0f, 0.0f);
static Point gLastCamDir(0.0f, 0.0f, 0.0f);
static void RecordCameraPose()
{
	if(!gCameraData)
		gCameraData = ICE_NEW(Vertices);

	const Point CamPos = GetCameraPos();
	const Point CamDir = GetCameraDir();
	if(CamPos!=gLastCamPos || CamDir!=gLastCamDir)
	{
		gLastCamPos = CamPos;
		gLastCamDir = CamDir;
		gCameraData->AddVertex(CamPos);
		gCameraData->AddVertex(CamDir);
	}
}

static void SaveCameraData()
{
	if(!gCameraData)
		return;
	FILE* fp = fopen("d:\\tmp\\camera_data.bin", "wb");
//	FILE* fp = fopen("f:\\tmp\\camera_data.bin", "wb");
	if(fp)
	{
		const udword TotalNbPoses = gCameraData->GetNbVertices()/2;
		fwrite(&TotalNbPoses, sizeof(udword), 1, fp);

		const Point* V = gCameraData->GetVertices();
		fwrite(V, sizeof(Point)*TotalNbPoses*2, 1, fp);
		fclose(fp);
	}
	DELETESINGLE(gCameraData);
}

static void KeyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27:
			exit(0);
			break;

		case 13:
			if(!gMenuIsVisible)
			{
				gMenuIsVisible = true;
				Point CameraData[2];
				CameraData[0] = GetCameraPos();
				CameraData[1] = GetCameraDir();
//				Save("PEEL", "Autosaved", "CameraData", CameraData, sizeof(Point)*2);
				//CopyToClipboard();

				for(udword i=0;i<gNbEngines;i++)
				{
					gEngines[i].mPickingData.mObject = null;
				}
			}
			else
			{
				ActivateTest();
			}
			break;

		case ' ':
			{
				const Point CamPos = GetCameraPos();
				const Point CamDir = GetCameraDir();
				const char* Text = _F("desc.mCamera[0] = CameraPose(Point(%.2ff, %.2ff, %.2ff), Point(%.2ff, %.2ff, %.2ff));", CamPos.x, CamPos.y, CamPos.z, CamDir.x, CamDir.y, CamDir.z);
				CopyToClipboard(Text);
				printf("Camera pose copied to clipboard\n");
			}
			break;
		case 'c':
		case 'C':
			gCamera.Reset();
			break;
		case '+':
			gCamera.SelectNextCamera();
			break;
		case '-':
			gCamera.SelectPreviousCamera();
			break;
		case 'p':
		case 'P':
			gPaused = !gPaused;
			gOneFrame = false;
			break;
		case 'o':
		case 'O':
			gOneFrame = true;
			gPaused = false;
			break;
		case 'r':
		case 'R':
			gRender = !gRender;
			break;
		case 'h':
		case 'H':
			gHelpIsVisible = !gHelpIsVisible;
			break;
		case 's':
		case 'S':
			TestCSVExport();
			break;
		case 'w':
		case 'W':
			gWireframe = !gWireframe;
			break;
		case 'x':
		case 'X':
			gWireframeOverlay = !gWireframeOverlay;
			break;
		case 'u':
		case 'U':
			gAutoCameraMove = !gAutoCameraMove;
			if(!gAutoCameraMove)
				SaveCameraData();
			break;
/*		case 'u':
		case 'U':
			RecordCameraPose();
			break;
		case 'i':
		case 'I':
			SaveCameraData();
			break;*/
		case 127:
			{
				for(udword i=0;i<gNbEngines;i++)
				{
					if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
						continue;

					if(gEngines[i].mPickingData.mObject)
					{
						gEngines[i].mEngine->ReleaseObject(gEngines[i].mPickingData.mObject);
						gEngines[i].mPickingData.mObject = null;
					}
				}
			}
			break;
	}
}

static void SpecialKeyCallback(int key, int x, int y, bool down)
{
	if(down)
	{
		// Function keys are used to enable/disable each engine
		for(udword i=0;i<gNbEngines;i++)
		{
			if(key==i+1)
			{
				gEngines[i].mEnabled = !gEngines[i].mEnabled;
			}
		}
	}

	if(!gMenuIsVisible)
	{
		gRunningTest->SpecialKeyCallback(key, x, y, down);

		const udword Flags = gRunningTest->GetFlags();
		if(!(Flags & TEST_FLAGS_USE_CURSOR_KEYS))
		{
			switch(key)
			{
				case GLUT_KEY_UP:		gState.mUp = down; break;
				case GLUT_KEY_DOWN:		gState.mDown = down; break;
				case GLUT_KEY_LEFT:		gState.mLeft = down; break;
				case GLUT_KEY_RIGHT:	gState.mRight = down; break;
			}
		}
	}
	else
	{
		if(down)
			gCurrentTest = TestSelectionKeyboardCallback(key, gCurrentTest);
	}
}

static void SpecialKeyDownCallback(int key, int x, int y)
{
	SpecialKeyCallback(key, x, y, true);
}

static void SpecialKeyUpCallback(int key, int x, int y)
{
	SpecialKeyCallback(key, x, y, false);
}

static	bool	gIsLeftButtonDown = false;
static	bool	gIsRightButtonDown = false;

static inline_ bool Raycast(udword i, const Point& origin, const Point& dir)
{
//	return gEngines[i].mEngine->Raycast(origin, dir, 5000.0f, gEngines[i].mPickingData);
	PintRaycastData tmp;
	tmp.mOrigin = origin;
	tmp.mDir = dir;
	tmp.mMaxDist = 5000.0f;
	return gEngines[i].mEngine->BatchRaycasts(gEngines[i].mSQHelper.GetThreadContext(), 1, &gEngines[i].mPickingData, &tmp)!=0;
}

static void MouseCallback(int button, int state, int x, int y)
{
	if(button==0)
	{
		if(state==0)
			gIsLeftButtonDown = true;
		else
			gIsLeftButtonDown = false;
	}

	if(button==2)
	{
		if(state==0)
			gIsRightButtonDown = true;
		else
			gIsRightButtonDown = false;
	}

	{
		gMouseX = x;
		gMouseY = y;
	}

//	printf("button: %d; state: %d\n", button, state);
	if(button==2)
	{
		if(state==0)
		{
			if(gCurrentTool==TOOL_PICKING)
			{
				const Point Dir = ComputeWorldRay(x, y);
				const Point Origin = GetCameraPos();

				for(udword i=0;i<gNbEngines;i++)
				{
					if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
					{
						gEngines[i].mPickingData.mObject = null;
						continue;
					}

					if(Raycast(i, Origin, Dir))
					{
						printf("Picked object: 0x%x (%s)\n", size_t(gEngines[i].mPickingData.mObject), gEngines[i].mEngine->GetName());

						const float Mass = gEngines[i].mEngine->GetMass(gEngines[i].mPickingData.mObject);
						printf("  Mass: %f\n", Mass);

						const Point LocalInertia = gEngines[i].mEngine->GetLocalInertia(gEngines[i].mPickingData.mObject);
						printf("  Local inertia: %f | %f | %f\n", LocalInertia.x, LocalInertia.y, LocalInertia.z);

						gEngines[i].mDragPoint = gEngines[i].mPickingData.mImpact;

						const PR WorldTransform = gEngines[i].mEngine->GetWorldTransform(gEngines[i].mPickingData.mObject);

						// TODO: rewrite this clumsy stuff
							const Matrix4x4 M = WorldTransform;
							Matrix4x4 InvM;
							InvertPRMatrix(InvM, M);

						gEngines[i].mLocalPoint = gEngines[i].mPickingData.mImpact * InvM;
#ifdef _DEBUG
						const Point NewWorldPt = gEngines[i].mLocalPoint * M;
						int stop=1;
#endif
					}
					else
					{
						gEngines[i].mPickingData.mObject = null;
					}
				}
			}
			else if(gCurrentTool==TOOL_ADD_IMPULSE)
			{
				const Point Dir = ComputeWorldRay(x, y);
				const Point Origin = GetCameraPos();

				for(udword i=0;i<gNbEngines;i++)
				{
					if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
						continue;

					if(Raycast(i, Origin, Dir))
						gEngines[i].mEngine->AddWorldImpulseAtWorldPos(gEngines[i].mPickingData.mObject, Dir*gImpulseMagnitude, gEngines[i].mPickingData.mImpact);
					else
						gEngines[i].mPickingData.mObject = null;
				}
			}
			else if(gCurrentTool==TOOL_SHOOT_BOX)
			{
				const Point Dir = ComputeWorldRay(x, y);
				const Point Origin = GetCameraPos();

				for(udword i=0;i<gNbEngines;i++)
				{
					if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
					{
						continue;
					}

					if(1)
					{
						PINT_BOX_CREATE BoxDesc(gBoxSize, gBoxSize, gBoxSize);
						BoxDesc.mRenderer	= CreateBoxRenderer(BoxDesc.mExtents);

						PINT_OBJECT_CREATE ObjectDesc;
						ObjectDesc.mShapes		= &BoxDesc;
						ObjectDesc.mMass		= gBoxMass;
						ObjectDesc.mPosition	= Origin;
						ObjectDesc.mLinearVelocity	= Dir * gBoxVelocity;
						gEngines[i].mEngine->CreateObject(ObjectDesc);
					}
				}
			}
			else if(gCurrentTool==TOOL_CAMERA_TRACKING)
			{
				if(!gTrackedObject)
				{
					const Point Dir = ComputeWorldRay(x, y);
					const Point Origin = GetCameraPos();

					for(udword i=0;i<gNbEngines;i++)
					{
						if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
						{
							gEngines[i].mPickingData.mObject = null;
							continue;
						}

						if(Raycast(i, Origin, Dir))
						{
							printf("Picked object: %d\n", size_t(gEngines[i].mPickingData.mObject));
							gTrackedObject = gEngines[i].mPickingData.mObject;
							gTrackedEngine = gEngines[i].mEngine;
						}
						gEngines[i].mPickingData.mObject = null;
					}
				}
			}
/*			else if(gCurrentTool==TOOL_CUSTOM_CONTROL)
			{
			}*/
			else ASSERT(0);
		}
		else
		{
			for(udword i=0;i<gNbEngines;i++)
				gEngines[i].mPickingData.mObject = null;
		}
	}
}

static void MotionCallback(int x, int y)
{
	if(gIsLeftButtonDown && !gTrackedObject)
	{
		const int dx = gMouseX - x;
		const int dy = gMouseY - y;

		RotateCamera(dx, dy);

		gMouseX = x;
		gMouseY = y;
	}

	if(gIsRightButtonDown)
	{
		if(gCurrentTool==TOOL_PICKING)
		{
			const Point Origin = GetCameraPos();
			const Point Dir = ComputeWorldRay(x, y);

			for(udword i=0;i<gNbEngines;i++)
			{
				if(gEngines[i].mPickingData.mObject)
				{
					const Point NewPos = Origin + Dir * gEngines[i].mPickingData.mDistance;
					gEngines[i].mDragPoint = NewPos;
				}
			}
		}
	}
}

#include "QPCTime.h"
static QPCTime	mQPCTimer;

	static inline_ void	StartProfile_RDTSC(udword& val)
	{
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			mov		[ebx], eax
		}
//		val = __rdtsc();
	}

	static inline_ void	EndProfile_RDTSC(udword& val)
	{
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			sub		eax, [ebx]
			mov		[ebx], eax
		}
//		val = __rdtsc() - val;
	}

	static inline_ void	StartProfile_TimeGetTime(udword& val)
	{
		val = timeGetTime();
	}

	static inline_ void	EndProfile_TimeGetTime(udword& val)
	{
		val = timeGetTime() - val;
	}

	static inline_ void	StartProfile_QPC()
	{
		mQPCTimer.getElapsedSeconds();
	}

	static inline_ void	EndProfile_QPC(QPCTime::Second& val)
	{
		val = mQPCTimer.getElapsedSeconds();
	}

static udword ProfileUpdate_RDTSC(EngineData& engine, float dt)
{
	udword val, CurrentMemory;
	{
		::StartProfile_RDTSC(val);
			CurrentMemory = engine.mEngine->Update(dt);
		::EndProfile_RDTSC(val);
	}
	const udword Time = val/1024;
	engine.mTiming.RecordTimeAndMemory(Time, CurrentMemory, gFrameNb);
	return Time;
}

static udword ProfileUpdate_TimeGetTime(EngineData& engine, float dt)
{
	udword val, CurrentMemory;
	{
		::StartProfile_TimeGetTime(val);
			CurrentMemory = engine.mEngine->Update(dt);
		::EndProfile_TimeGetTime(val);
	}
	const udword Time = val;
	engine.mTiming.RecordTimeAndMemory(Time, CurrentMemory, gFrameNb);
	return Time;
}

static udword ProfileUpdate_QPC(EngineData& engine, float dt)
{
	udword val, CurrentMemory;
	{
		QPCTime::Second s;
		::StartProfile_QPC();
			CurrentMemory = engine.mEngine->Update(dt);
		::EndProfile_QPC(s);
		val = udword(s*1000000.0);
	}
	const udword Time = val;
	engine.mTiming.RecordTimeAndMemory(Time, CurrentMemory, gFrameNb);
	return Time;
}

static void NoProfileUpdate(EngineData& engine, float dt)
{
	engine.mEngine->Update(dt);
}

static udword ProfileTestUpdate_RDTSC(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		::StartProfile_RDTSC(val);
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_RDTSC(val);
	}
	const udword Time = val/1024;
	engine.mTiming.RecordTimeAndMemory(Time, 0, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static udword ProfileTestUpdate_TimeGetTime(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		::StartProfile_TimeGetTime(val);
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_TimeGetTime(val);
	}
	const udword Time = val;
	engine.mTiming.RecordTimeAndMemory(Time, 0, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static udword ProfileTestUpdate_QPC(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		QPCTime::Second s;
		::StartProfile_QPC();
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_QPC(s);
		val = udword(s*1000000.0);
	}
	const udword Time = val;
	engine.mTiming.RecordTimeAndMemory(Time, 0, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static udword ProfileTestUpdate_RDTSC_Combined(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		::StartProfile_RDTSC(val);
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_RDTSC(val);
	}
	const udword Time = val/1024;
	engine.mTiming.UpdateRecordedTime(Time, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static udword ProfileTestUpdate_TimeGetTime_Combined(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		::StartProfile_TimeGetTime(val);
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_TimeGetTime(val);
	}
	const udword Time = val;
	engine.mTiming.UpdateRecordedTime(Time, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static udword ProfileTestUpdate_QPC_Combined(PhysicsTest* test, EngineData& engine, float dt)
{
	udword val, TestResult;
	{
		QPCTime::Second s;
		::StartProfile_QPC();
			TestResult = gRunningTest->Update(*engine.mEngine, dt);
		::EndProfile_QPC(s);
		val = udword(s*1000000.0);
	}
	const udword Time = val;
	engine.mTiming.UpdateRecordedTime(Time, gFrameNb);
	engine.mTiming.mCurrentTestResult = TestResult;
	return Time;
}

static void Simulate()
{
	if(0 && gTrashCache)
	{
		gRender = true;
		static udword Count=0;
		Count++;
		if(Count!=16)
			return;
		Count=0;
	}

	if(gPaused)
		return;

	const float dt = gTimestep;

	Permutation P;
	P.Init(gNbEngines);
	if(gRandomizeOrder)
		P.Random(gNbEngines*2);
	else
		P.Identity();

	const bool MustProfileTestUpdate = gRunningTest ? gRunningTest->ProfileUpdate() : false;

	udword CurrentTime;
	for(udword ii=0;ii<gNbEngines;ii++)
	{
		const udword i = P[ii];
		if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
			continue;

		ASSERT(gEngines[i].mEngine);
		if(		MustProfileTestUpdate
			&&	gSQProfilingMode==SQ_PROFILING_UPDATE)
		{
			NoProfileUpdate(gEngines[i], dt);
		}
		else
		{
			if(gProfilingUnits==PROFILING_UNITS_RDTSC)
				CurrentTime = ProfileUpdate_RDTSC(gEngines[i], dt);
			else if(gProfilingUnits==PROFILING_UNITS_TIME_GET_TIME)
				CurrentTime = ProfileUpdate_TimeGetTime(gEngines[i], dt);
			else if(gProfilingUnits==PROFILING_UNITS_QPC)
				CurrentTime = ProfileUpdate_QPC(gEngines[i], dt);
			else ASSERT(0);
		}
//		printf(_F("%s: %d (Avg: %d)(Worst: %d)\n", gEngines[i].mEngine->GetName(), CurrentTime, gEngines[i].mTiming.GetAvgTime(), gEngines[i].mTiming.mWorstTime));

		gEngines[i].mEngine->UpdateNonProfiled(dt);

		if(gTrashCache)
			trashCache();
//			trashIcacheAndBranchPredictors();
	}

	if(gRunningTest)
	{
		gRunningTest->CommonUpdate(dt);
		for(udword ii=0;ii<gNbEngines;ii++)
		{
			const udword i = P[ii];
			if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
				continue;

			ASSERT(gEngines[i].mEngine);
			if(MustProfileTestUpdate)
			{
				if(gSQProfilingMode==SQ_PROFILING_SIM)
				{
					gRunningTest->Update(*gEngines[i].mEngine, dt);
				}
				else if(gSQProfilingMode==SQ_PROFILING_UPDATE)
				{
					if(gProfilingUnits==PROFILING_UNITS_RDTSC)
						CurrentTime = ProfileTestUpdate_RDTSC(gRunningTest, gEngines[i], dt);
					else if(gProfilingUnits==PROFILING_UNITS_TIME_GET_TIME)
						CurrentTime = ProfileTestUpdate_TimeGetTime(gRunningTest, gEngines[i], dt);
					else if(gProfilingUnits==PROFILING_UNITS_QPC)
						CurrentTime = ProfileTestUpdate_QPC(gRunningTest, gEngines[i], dt);
					else ASSERT(0);
				}
				else
				{
					if(gProfilingUnits==PROFILING_UNITS_RDTSC)
						CurrentTime = ProfileTestUpdate_RDTSC_Combined(gRunningTest, gEngines[i], dt);
					else if(gProfilingUnits==PROFILING_UNITS_TIME_GET_TIME)
						CurrentTime = ProfileTestUpdate_TimeGetTime_Combined(gRunningTest, gEngines[i], dt);
					else if(gProfilingUnits==PROFILING_UNITS_QPC)
						CurrentTime = ProfileTestUpdate_QPC_Combined(gRunningTest, gEngines[i], dt);
					else ASSERT(0);
				}
			}
			else
			{
				// Normal update without profiling
				gRunningTest->Update(*gEngines[i].mEngine, dt);
			}
		}
	}

	gFrameNb++;
}

static void PrintTimings()
{
//	const float TextScale = 0.02f * float(INITIAL_SCREEN_HEIGHT) / float(gScreenHeight);
	const float TextScale = 0.0175f * float(INITIAL_SCREEN_HEIGHT) / float(gScreenHeight);
	float y = 1.0f - TextScale;

	const bool MustProfileTestUpdate = gRunningTest ? gRunningTest->ProfileUpdate() : false;

	if(gRunningTest)
	{
		gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);
		gTexter.print(0.0f, y, TextScale, _F("Test: %s (%d camera views available)\n", gRunningTest->GetName(), gCamera.mNbSceneCameras+1));
		y -= TextScale;
	}

	for(udword i=0;i<gNbEngines;i++)
	{
		ASSERT(gEngines[i].mEngine);
		Pint* Engine = gEngines[i].mEngine;
		if(!(Engine->GetFlags() & PINT_IS_ACTIVE))
			continue;
		const Point MainColor = Engine->GetMainColor();
		gTexter.setColor(MainColor.x, MainColor.y, MainColor.z, 1.0f);
		if(gEngines[i].mEnabled)
		{
			if(gEngines[i].mSupportsCurrentTest)
			{
				const PintTiming& Timing = gEngines[i].mTiming;
				if(MustProfileTestUpdate)
//					gTexter.print(0.0f, y, TextScale, _F("%s: %d (Avg: %d)(Worst: %d)(%d Kb)(Test value: %d)\n",
					gTexter.print(0.0f, y, TextScale, _F("%s: %d (Avg: %d)(Worst: %d)(Nb hits: %d)\n",
						Engine->GetName(), Timing.mCurrentTime, Timing.GetAvgTime(), Timing.mWorstTime, Timing.mCurrentTestResult));
				else
					gTexter.print(0.0f, y, TextScale, _F("%s: %d (Avg: %d)(Worst: %d)(%d Kb)\n",
						Engine->GetName(), Timing.mCurrentTime, Timing.GetAvgTime(), Timing.mWorstTime, Timing.mCurrentMemory/1024));
			}
			else
			{
				gTexter.print(0.0f, y, TextScale, _F("%s: (unsupported/not exposed)\n", Engine->GetName()));
			}
		}
		else
			gTexter.print(0.0f, y, TextScale, _F("%s: (disabled)\n", Engine->GetName()));
		y -= TextScale;
	}
	gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);
	if(gPaused)
		gTexter.print(0.0f, y, TextScale, "(Paused)");
	y -= TextScale;
	gTexter.print(0.0f, y, TextScale, _F("Frame: %d", gFrameNb));
	y -= TextScale;
	gTexter.print(0.0f, y, TextScale, _F("FPS: %.02f", gFPS.GetFPS()));

	y -= TextScale * 2.0f;
	if(gRunningTest)
	{
		for(udword i=0;i<gNbEngines;i++)
		{
			if(gEngines[i].mEnabled && gEngines[i].mSupportsCurrentTest)
			{
				ASSERT(gEngines[i].mEngine);
				Pint* Engine = gEngines[i].mEngine;

				const Point MainColor = Engine->GetMainColor();
				gTexter.setColor(MainColor.x, MainColor.y, MainColor.z, 1.0f);

				y = gRunningTest->DrawDebugText(*Engine, gTexter, y, TextScale);
			}
		}
	}

	if(gDisplayMessage)
	{
		gDisplayMessage = false;
		gDisplayMessageDelay = 1.0f;
	}
	if(gDisplayMessageDelay>0.0f)
	{
		y -= TextScale * 2.0f;
		gTexter.setColor(1.0f, 0.0f, 0.0f, 1.0f);
		if(gDisplayMessageType==0)
			gTexter.print(0.0f, y, TextScale, "Saving results...");
		else
			gTexter.print(0.0f, y, TextScale, "This private test is disabled in public builds.");
	}
	gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);
}

static void FormatText(const char* text, udword nb_col, ParameterBlock& pb)
{
//	for(udword i=0;i<nb_col;i++)	printf("*");
//	printf("\n");

	String S(text);
	S.Replace(' ', 1);

	char* Buffer = (char*)S.Get();

	udword Total = strlen(Buffer);
	udword PreviousSpace = INVALID_ID;
	udword CurrentLength = 0;
	udword c = 0;

//	while(Buffer[c])
	while(c<Total)
	{
		// Look for space
		udword InitialPos = c;
		while(Buffer[c] && Buffer[c]!=1)	c++;
		udword WordLength = c - InitialPos;

		if(CurrentLength+WordLength<=nb_col)
		{
			// Add current word to current line
			PreviousSpace = c;
			c++;
			CurrentLength += WordLength+1;
		}
		else
		{
			ASSERT(PreviousSpace!=INVALID_ID);
			Buffer[PreviousSpace] = ' ';
			PreviousSpace = INVALID_ID;

			c = InitialPos;
			CurrentLength = 0;
		}
	}

	pb.Create(S);
	udword NbParams = pb.GetNbParams();
	for(udword i=0;i<NbParams;i++)
	{
		pb.GetParam(i).Replace(1, ' ');
//		printf("%s\n", pb.GetParam(i));
	}
}

static void DrawRectangle(float x_start, float x_end, float y_start, float y_end, const Point& color_top, const Point& color_bottom, float alpha)
{
	if(alpha!=1.0f)
	{
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	else
	{
		glDisable(GL_BLEND);
	}
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, gScreenWidth, 0, gScreenHeight, -1, 1);
	}
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const float x0 = x_start * float(gScreenWidth);
	const float x1 = x_end * float(gScreenWidth);
	const float y0 = y_start * float(gScreenHeight);
	const float y1 = y_end * float(gScreenHeight);

	const Point p0(x0,	y0, 0.0f);
	const Point p1(x0,	y1, 0.0f);
	const Point p2(x1,	y1, 0.0f);
	const Point p3(x1,	y0, 0.0f);

	glBegin(GL_TRIANGLES);
		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p0.x, p0.y, p0.z);

		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p1.x, p1.y, p1.z);

		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p2.x, p2.y, p2.z);

		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p0.x, p0.y, p0.z);

		glColor4f(color_bottom.x, color_bottom.y, color_bottom.z, alpha);
		glVertex3f(p2.x, p2.y, p2.z);

		glColor4f(color_top.x, color_top.y, color_top.z, alpha);
		glVertex3f(p3.x, p3.y, p3.z);
	glEnd();

	glColor4f(1.0f, 1.0f, 1.0f, alpha);
	glBegin(GL_LINES);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);

		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);

		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p3.x, p3.y, p3.z);

		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p0.x, p0.y, p0.z);
	glEnd();

	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glDisable(GL_BLEND);
}

static BasicRenderer gPintRender;

void SetUserDefinedPolygonMode()
{
	if(gWireframe)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static udword gLastTime = 0;
static void RenderCallback()
{
	// Automatically reset the test if its "mMustResetTest" member has been set to true.
	if(gRunningTest && gRunningTest->mMustResetTest)
	{
		gRunningTest->mMustResetTest = false;
		ActivateTest(gRunningTest);
	}

	gFPS.Update();
	gPintRender.mFrameNumber++;

	udword CurrentTime = timeGetTime();
	udword ElapsedTime = CurrentTime - gLastTime;
	gLastTime = CurrentTime;
	const float Delta = float(ElapsedTime)*0.001f;
	if(gDisplayMessageDelay>0.0f)
	{
		gDisplayMessageDelay -= Delta;
		if(gDisplayMessageDelay<0.0f)
			gDisplayMessageDelay = 0.0f;
	}

	if(!gMenuIsVisible)
	{
		for(udword i=0;i<gNbSimulateCallsPerFrame;i++)
			Simulate();
	}

	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

//	DrawRectangle(0.0f, 1.0f, 0.0f, 1.0f, Point(0.2f, 0.2f, 0.2f), Point(0.1f, 0.1f, 0.2f), 1.0f);
	DrawRectangle(0.0f, 1.0f, 0.0f, 1.0f, Point(0.35f, 0.35f, 0.35f), Point(0.1f, 0.1f, 0.2f), 1.0f);
//	DrawRectangle(0.0f, 1.0f, 0.0f, 1.0f, Point(0.85f, 0.85f, 0.85f), Point(0.1f, 0.1f, 0.5f), 1.0f);

	if(gState.mUp)
		MoveCameraForward();
	if(gState.mDown)
		MoveCameraBackward();
	if(gState.mLeft)
		MoveCameraRight();
	if(gState.mRight)
		MoveCameraLeft();

	if(gTrackedObject && gTrackedEngine)
	{
		const PR Pose = gTrackedEngine->GetWorldTransform(gTrackedObject);
		const Point CamPos = GetCameraPos();
		const Point Dir = (Pose.mPos - CamPos).Normalize();
		SetCamera(CamPos, Dir);
	}

	// Setup projection matrix
	glMatrixMode(GL_PROJECTION);
	SetupCameraMatrix();

	// Setup modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

//	glDisable(GL_CULL_FACE);
//	glEnable(GL_CULL_FACE);	glCullFace(GL_FRONT);
	glEnable(GL_CULL_FACE);	glCullFace(GL_BACK);

	if(gRender && !gMenuIsVisible)
	{
		{
			glLineWidth(0.5f);
			SetUserDefinedPolygonMode();
		}

		if(gRunningTest)
		{
			gRunningTest->CommonRender(gPintRender);

			for(udword i=0;i<gNbEngines;i++)
			{
				if(gEngines[i].mEnabled && gEngines[i].mSupportsCurrentTest)
				{
					ASSERT(gEngines[i].mEngine);
					Pint* Engine = gEngines[i].mEngine;
					const Point MainColor = Engine->GetMainColor();
					glColor3f(MainColor.x, MainColor.y, MainColor.z);
					gRunningTest->DrawDebugInfo(*Engine, gPintRender);
				}
			}
		}

		for(udword i=0;i<gNbEngines;i++)
		{
			if(gEngines[i].mEnabled && gEngines[i].mSupportsCurrentTest)
			{
				if(gWireframeOverlay)
					SetUserDefinedPolygonMode();

				ASSERT(gEngines[i].mEngine);
				Pint* Engine = gEngines[i].mEngine;
				const Point MainColor = Engine->GetMainColor();
				glColor3f(MainColor.x, MainColor.y, MainColor.z);
				Engine->Render(gPintRender);
				gEngines[i].mSQHelper.Render(gPintRender, gPaused);
				if(gWireframeOverlay)
				{
					glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
					glColor3f(0.0f, 0.0f, 0.0f);

					glMatrixMode(GL_PROJECTION);
					SetupCameraMatrix(1.005f);

					glMatrixMode(GL_MODELVIEW);
					glLoadIdentity();

					glDisable(GL_LIGHTING);
					Engine->Render(gPintRender);
					glEnable(GL_LIGHTING);
					glMatrixMode(GL_PROJECTION);
					SetupCameraMatrix();

					glMatrixMode(GL_MODELVIEW);
					glLoadIdentity();
				}

				if(0)
				{
					glEnable(GL_CULL_FACE);	glCullFace(GL_FRONT);

						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//						glColor3f(0.0f, 0.0f, 0.0f);
						glColor3f(MainColor.x*0.5f, MainColor.y*0.5f, MainColor.z*0.5f);

						glMatrixMode(GL_PROJECTION);
						SetupCameraMatrix(1.005f);

						glMatrixMode(GL_MODELVIEW);
						glLoadIdentity();

						glDisable(GL_LIGHTING);
						Engine->Render(gPintRender);
						glEnable(GL_LIGHTING);

						glMatrixMode(GL_PROJECTION);
						SetupCameraMatrix();

						glMatrixMode(GL_MODELVIEW);
						glLoadIdentity();

					glEnable(GL_CULL_FACE);	glCullFace(GL_BACK);
				}
			}
		}
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

//	DrawFrame(Point(0.0f, 0.0f, 0.0f));

	if(0)
	{
		glColor3f(1.0f, 0.5f, 0.25f);

		PR pose;
		pose.mPos	= Point(0.0f, 0.0f, 0.0f);
		pose.mRot	= Quat(1.0f, 0.0f, 0.0f, 0.0f);
		DrawCapsule(1.0f, 4.0f, pose);
	}

	if(gIsRightButtonDown)
	{
		if(gCurrentTool==TOOL_PICKING)
		{
			for(udword i=0;i<gNbEngines;i++)
			{
				if(!gEngines[i].mEnabled || !gEngines[i].mSupportsCurrentTest)
					continue;

				if(gEngines[i].mPickingData.mObject)
				{
					const Point& Pt = gEngines[i].mPickingData.mImpact;
	//				DrawFrame(Pt);

	//				DrawLine(Pt, gEngines[i].mDragPoint, gEngines[i].mEngine->GetMainColor(), 1.0f);

					{
						const PR WorldTransform = gEngines[i].mEngine->GetWorldTransform(gEngines[i].mPickingData.mObject);

						// TODO: rewrite this clumsy stuff
							const Matrix4x4 M = WorldTransform;
							const Point NewWorldPt = gEngines[i].mLocalPoint * M;

							DrawFrame(NewWorldPt);
							DrawLine(NewWorldPt, gEngines[i].mDragPoint, gEngines[i].mEngine->GetMainColor());

							const Point action = gPickingForce*(gEngines[i].mDragPoint - NewWorldPt);

							gEngines[i].mEngine->AddWorldImpulseAtWorldPos(gEngines[i].mPickingData.mObject, action, NewWorldPt);
					}
				}
			}
		}
	}

	if(0)
	{
		BasicRandom Rnd;
		for(int i=0;i<1000;i++)
		{
			Point tmp;
			UnitRandomPt(tmp, Rnd);
			DrawLine(Point(0.0f, 0.0f, 0.0f), tmp, tmp);
		}
	}

	if(gMenuIsVisible)
	{
		const float TextScale = 0.02f * float(INITIAL_SCREEN_HEIGHT) / float(gScreenHeight);
		const float x = 0.0f;
		float YLast;
		gCandidateTest = RenderTestSelector(gTexter, x, TextScale, gCurrentTest, YLast);
		gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);

		float y = 1.0f - TextScale * 11.0f;
		if(gHelpIsVisible)
		{
			gTexter.print(x, y, TextScale, "Press H to Hide help");													y -= TextScale;
			y -= TextScale;

			const float HelpTextScale = TextScale * 0.8f;

			gTexter.print(x, y, HelpTextScale, "In menu mode (this screen):");											y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press arrow up/down to select a test, left/right to skip a category");	y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press Return to run selected test (and enter 'simulation mode')");		y -= HelpTextScale;

			y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "In simulation mode:");													y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press Return to go back to the menu mode");								y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press arrow keys to move the camera");									y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press + and - (on the keypad) to switch cameras");						y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press C to reset the Camera");											y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press O to step One frame");											y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press R to disable Rendering");											y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press W to enable Wireframe mode");										y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press X to enable Wireframe Overlay mode");								y -= HelpTextScale;
//			gTexter.print(x, y, HelpTextScale, "Use the mouse and left mouse button to move the camera");				y -= HelpTextScale;
//			gTexter.print(x, y, HelpTextScale, "Use the mouse and right mouse button for picking");						y -= HelpTextScale;

			y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "At any time:");															y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press F1->Fx to enable/disable engines");								y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press P to enable/disable Pause");										y -= HelpTextScale;
			gTexter.print(x, y, HelpTextScale, "Press S to Save results as an Excel file");								y -= HelpTextScale;
		}
		else
		{
			gTexter.print(x, y, TextScale, "Press H for Help");
		}

		if(gCandidateTest)
		{
			const float PanelSize = TextScale*13.0f;
			float ystart = YLast + TextScale*10.0f + PanelSize;	// 10 here depends on number of lines in selector
			y = ystart - TextScale*2.0f;

			const float XOffset = 0.01f;
			DrawRectangle(XOffset, 1.0f-XOffset, ystart, ystart - PanelSize, Point(1.0f, 0.5f, 0.2f), Point(0.0f, 0.0f, 0.0f), 0.5f);

			const char* TestDescription = gCandidateTest->GetDescription();
			if(!TestDescription)
			{
				static const char* NoDesc = "(No description available)";
				TestDescription = NoDesc;
			}

			ParameterBlock pb;
			FormatText(TestDescription, 80, pb);
			udword NbParams = pb.GetNbParams();

			for(udword i=0;i<NbParams;i++)
			{
				gTexter.print(x+XOffset*2.0f, y, TextScale, pb[i].Get());
				y -= TextScale;
			}
		}
	}

	PrintTimings();

	if(0 && gRaytracingTest==RAYTRACING_TEST_DISABLED)
	{
		for(udword i=0;i<gNbEngines;i++)
		{
			if(gEngines[i].mEnabled)
			{
				ASSERT(gEngines[i].mEngine);
				Pint* Engine = gEngines[i].mEngine;

				if(gRaytracingWindows[i])
				{
					gRaytracingWindows[i]->CreatePic();
					gRaytracingWindows[i]->redraw();
				}
			}
		}
	}

	if(gRaytracingTest!=RAYTRACING_TEST_DISABLED && !gPaused)
	{
		Permutation P;
		P.Init(gNbEngines);
		if(gRandomizeOrder)
			P.Random(gNbEngines*2);
		else
			P.Identity();

		uword NbRays = 0;
		if(gRaytracingResolution==RT_WINDOW_64)
			NbRays = 64;
		else if(gRaytracingResolution==RT_WINDOW_128)
			NbRays = 128;
		else if(gRaytracingResolution==RT_WINDOW_256)
			NbRays = 256;
		else if(gRaytracingResolution==RT_WINDOW_512)
			NbRays = 512;

		const uword RAYTRACING_RENDER_WIDTH = NbRays;
		const uword RAYTRACING_RENDER_HEIGHT = NbRays;

		Picture Target;
		Target.Init(RAYTRACING_RENDER_WIDTH, RAYTRACING_RENDER_HEIGHT);

		for(udword ii=0;ii<gNbEngines;ii++)
		{
			const udword i = P[ii];

			if(gEngines[i].mEnabled)
			{
				ASSERT(gEngines[i].mEngine);
				Pint* Engine = gEngines[i].mEngine;

				if(gRaytracingWindows[i])
				{
					udword NbHits;
					udword TotalTime;
					if(gRaytracingTest==RAYTRACING_TEST_ST)
						NbHits = RaytracingTest(Target, *Engine, TotalTime, gScreenWidth, gScreenHeight, NbRays, gRTDistance);
					else //if(gRaytracingTest==RAYTRACING_TEST_MT)
						NbHits = RaytracingTestMT(Target, *Engine, TotalTime, gScreenWidth, gScreenHeight, NbRays, gRTDistance);
					printf("%s: %d hits, time = %d\n", Engine->GetName(), NbHits, TotalTime/1024);

					gRaytracingWindows[i]->mPic = Target;
					gRaytracingWindows[i]->mPic.Stretch(RAYTRACING_DISPLAY_WIDTH, RAYTRACING_DISPLAY_HEIGHT);
					gRaytracingWindows[i]->redraw();
				}
/*				else
				{
					udword TotalTime;
					udword NbHits = RaytracingTest(*Engine, TotalTime, gScreenWidth, gScreenHeight);
					printf("%s: %d hits, time = %d\n", Engine->GetName(), NbHits, TotalTime/1024);
				}*/
			}
		}
		printf("\n");
	}

	glutSwapBuffers();

	if(gOneFrame)
		gPaused = true;

	AutomatedTests* AutoTests = GetAutomatedTests();
	if(AutoTests)
	{
		AutomatedTest* CurrentTest = AutoTests->GetCurrentTest();
		const udword Limit = MAX(CurrentTest->mNbFrames, AutoTests->mDefaultNbFrames);
		if(gFrameNb>Limit)
		{
			TestCSVExport();

			AutomatedTest* NextTest = AutoTests->SelectNextTest();
			if(NextTest)
				ActivateTest(NextTest->mTest);
			else
				exit(0);
		}
	}
	gPEEL_PollRadioButtons();

	if(gAutoCameraMove)
	{
		MoveCameraForward();
		RecordCameraPose();
	}
}

static void ReshapeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
	gTexter.setScreenResolution(width, height);
	gScreenWidth = width;
	gScreenHeight = height;
}

static void IdleCallback()
{
	glutPostRedisplay();
}

static CustomIceAllocator* gIceAllocator = null;

static void gCleanup()
{
	{
		Save("PEEL", "Autosaved", "CurrentTest", gCurrentTest);
	}

	timeBeginPeriod(1);
	printf("Exiting...\n");
	if(gRunningTest)
		gRunningTest->CloseUI();
	CloseAll();

	DELETESINGLE(gRoot);

	for(udword i=0;i<gNbPlugIns;i++)
	{
		gPlugIns[i]->CloseGUI();
	}
	PEEL_CloseGUI();

	CloseIceGUI();
	CloseIceImageWork();
	CloseMeshmerizer();
	CloseIceMaths();
	CloseIceCore();

	DELETESINGLE(gIceAllocator);
}

static void RegisterPlugIn(const char* filename)
{
	PintPlugin* pp = LoadPlugIn(filename);
	if(pp)
	{
		printf(_F("Plugin found: %s\n", filename));
		gPlugIns[gNbPlugIns++] = pp;
	}
}




	class SelectObjects : public SelectionDialog
	{
		public:
							SelectObjects(const SelectionDialogDesc& desc) : SelectionDialog(desc)	{}
							~SelectObjects()														{}

		virtual		void	OnSelected(void* user_data)
							{
//								printf("Selected: %s\n", user_data);
								RegisterPlugIn((const char*)user_data);
							}
	};

static char gBuildFolder[MAX_PATH];
static char gCurrentFile[MAX_PATH];

const char* FindPEELFile(const char* filename)
{
	{
		const char* F0 = _F("../build/%s", filename);
		strcpy(gCurrentFile, F0);
		if(FileExists(gCurrentFile))
			return gCurrentFile;
	}

	{
		const char* F1 = _F("../build/Customers/%s", filename);
		strcpy(gCurrentFile, F1);
		if(FileExists(gCurrentFile))
			return gCurrentFile;
	}

	{
		const char* F1 = _F("./%s", filename);
		strcpy(gCurrentFile, F1);
		if(FileExists(gCurrentFile))
			return gCurrentFile;
	}

	{
		const char* F1 = _F("./Customers/%s", filename);
		strcpy(gCurrentFile, F1);
		if(FileExists(gCurrentFile))
			return gCurrentFile;
	}

	{
		const char* F2 = _F("%s%s", gBuildFolder, filename);
		strcpy(gCurrentFile, F2);
		if(FileExists(gCurrentFile))
			return gCurrentFile;
	}
	return null;
}

static udword AnalyzeCommandLine(int argc, char** argv)
{
	if(argc && argv)
	{
		strcpy(gBuildFolder, *argv);
		char* Build = strstr(gBuildFolder, "Build");
		if(Build)
		{
			Build[6]=0;
		}
		else
		{
			gBuildFolder[0] = 0;
		}
	}

	if(argc<=1)
		return 0;

	argc--;
	argv++;

	udword NbPlugins = 0;

	while(argc)
	{
		argc--;
		const char* Command = *argv++;

		if(Command[0]=='/' && Command[1]=='/')
			break;

		if(Command[0]=='-' && Command[1]=='r')
		{
			gRender = false;
		}
		else if(Command[0]=='-' && Command[1]=='p')
		{
			if(argc)
			{
				argc--;
				const char* Filename = *argv++;
				RegisterPlugIn(Filename);
				NbPlugins++;
			}
		}
		else if(Command[0]=='-' && Command[1]=='t')
		{
			if(argc)
			{
				argc--;
				const char* TestName = *argv++;

				const udword NbTests = GetNbTests();
				for(udword i=0;i<NbTests;i++)
				{
					PhysicsTest* Test = GetTest(i);
					if(strcmp(Test->GetName(), TestName)==0)
					{
						gCandidateTest = Test;
						break;
					}
				}
			}
		}
	}
	return NbPlugins;
}

void ThreadSetup()
{
	_clearfp();

   udword x86_cw;
   udword sse2_cw;
	#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_IC | _MCW_RC | _MCW_PC
	__control87_2(_CW_DEFAULT | _DN_FLUSH, _MCW_ALL, &x86_cw, &sse2_cw);

	_clearfp();
}

///////////////////////////////////////////////////////////////////////////////

//void TestSpy();

int main(int argc, char** argv)
{
	if(0)
	{
//		ProcessRepXFile_RemoveDynamics("c5m4_quarter2_Statics.repx");
//		ProcessRepXFile_RemoveDynamics("Planetside.repx");
		ProcessRepXFile_RemoveDynamics("specular_raycast_meshes.repx");
		return 0;
	}

	ThreadSetup();
	SRand(42);

	gIceAllocator = new CustomIceAllocator;
	ASSERT(gIceAllocator);
	IceCore::SetAllocator(*gIceAllocator);

	ICECORECREATE icc;
	icc.mLogFile = false;
	InitIceCore(&icc);
	InitIceMaths();
	InitMeshmerizer();
	InitIceImageWork();
	InitIceGUI();

/*	if(0)
	{
		TestSpy();
		gCleanup();
		return 0;
	}*/

	if(0)
	{
		String CurrentDir;
		GetCurrentDir(CurrentDir);
		printf("Current dir: %s\n", CurrentDir.Get());
	}

	ICEGUIAPPCREATE Create;
	IceGUIApp MyApp;
	bool Status = MyApp.Init(Create);

	InitTests();
	{
		udword Value;
		if(Load("PEEL", "Autosaved", "CurrentTest", Value))
		{
			if(Value<GetNbTests())
				gCurrentTest = Value;
		}
	}

	glutInit(&argc, argv);
	glutInitWindowSize(gScreenWidth, gScreenHeight);
	glutInitWindowPosition(MAIN_WINDOW_POS_X, MAIN_WINDOW_POS_Y);
	glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);
	gMainHandle = glutCreateWindow("PEEL (Physics Engine Evaluation Lab)");
	glutSetWindow(gMainHandle);
	glutDisplayFunc(RenderCallback);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(IdleCallback);
	glutKeyboardFunc(KeyboardCallback);
	glutSpecialFunc(SpecialKeyDownCallback);
//	glutKeyboardUpFunc(KeyboardUpCallback);
	glutSpecialUpFunc(SpecialKeyUpCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);
	MotionCallback(0,0);

	// We'll need the window handle to set the focus back to the render window
	gWindowHandle = WindowFromDC(wglGetCurrentDC());

	// Setup default render states
	glClearColor(0.2f, 0.2f, 0.2f, 1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	// Setup lighting - we need white lights not to interfere with the plugins' main colors
	glEnable(GL_LIGHTING);
//	const float ambientColor0[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
	const float diffuseColor0[]	= { 1.0f, 1.0f, 1.0f, 0.0f };
//	const float specularColor0[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
//	const float position0[]		= { 100.0f, 100.0f, 400.0f, 1.0f };
//	const float position0[]		= { 0.0f, 10.0f, 50.0f, 1.0f };
	Point Dir(1.0f, 1.0f, 0.5f);
	Dir.Normalize();
	const float position0[]		= { Dir.x, Dir.y, Dir.z, 0.0f };
//	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor0);
//	glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor0);
	glLightfv(GL_LIGHT0, GL_POSITION, position0);
	glEnable(GL_LIGHT0);

	if(0)
	{
//		const float ambientColor1[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
		const float diffuseColor1[]	= { 0.8f, 0.8f, 0.8f, 0.0f };
//		const float specularColor1[]	= { 0.0f, 0.0f, 0.0f, 0.0f };
//		const float position1[]		= { 0.0f, -10.0f, -50.0f, 1.0f };
		const float position1[]		= { -Dir.x, Dir.y, -Dir.z, 0.0f };
//		glLightfv(GL_LIGHT1, GL_AMBIENT, ambientColor1);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseColor1);
//		glLightfv(GL_LIGHT1, GL_SPECULAR, specularColor1);
		glLightfv(GL_LIGHT1, GL_POSITION, position1);
		glEnable(GL_LIGHT1);
	}
	glEnable(GL_NORMALIZE);

	EnableGLExtensions();
	GL_SelectVSYNC(gEnableVSync);

	gTexter.init();
	gTexter.setScreenResolution(gScreenWidth, gScreenHeight);
	gTexter.setColor(1.0f, 1.0f, 1.0f, 1.0f);

	ASSERT(argc && argv);
	const String ExeFilename = *argv;

	gRoot = ICE_NEW(String);
	GetPath(ExeFilename, *gRoot);

	gCandidateTest = null;
	const udword NbLoadedPlugs = AnalyzeCommandLine(argc, argv);
	if(!NbLoadedPlugs)
	{
		ICEGUIAPPCREATE Create;
		IceGUIApp MyApp;
		bool Status = MyApp.Init(Create);

		const udword DialogWidth	= 400;
		const udword DialogHeight	= 300;

		SelectionDialogDesc Desc;
		Desc.mMultiSelection		= true;
		Desc.mFilters				= "Plugins";
		Desc.mParent				= null;
		Desc.mX						= 100;
		Desc.mY						= 100;
		Desc.mWidth					= DialogWidth;
		Desc.mHeight				= DialogHeight;
		Desc.mLabel					= "PEEL - Select plugins";
		Desc.mType					= WINDOW_DIALOG;
//		Desc.mType					= WINDOW_POPUP;
	//	Desc.mStyle					= WSTYLE_DLGFRAME;
		Desc.mCloseAppWithWindow	= true;
		Desc.Center();
		SelectObjects* Main = ICE_NEW(SelectObjects)(Desc);
		Main->SetVisible(true);

		Strings ss;
		if(1)
		{
			struct Local
			{
				static bool FindFilesCB(const char* filename, const char* path, void* user_data)
				{
					ASSERT(filename);
//					if(filename[0] && filename[0]!='P')	return false;
//					if(filename[1] && filename[1]!='I')	return false;
//					if(filename[2] && filename[2]!='N')	return false;
//					if(filename[3] && filename[3]!='T')	return false;
//					if(filename[4] && filename[4]!='_')	return false;

					// ### hardcoded and lame, but oh well
					const char* Dot = strchr(filename, '.');
					const bool IsDebug = Dot ? (Dot[-1]=='D' && Dot[-2]=='_') : false;
#ifdef _DEBUG
					const bool MustAdd = IsDebug;
#else
					const bool MustAdd = !IsDebug;
#endif
					if(MustAdd)
					{
						Strings* ss = (Strings*)user_data;
		//				Main->Add(FF.mFile, 0, NULL, TRUE);
						// ### TODO: make AddString return the string directly
						ss->AddString(filename);

/*						char buffer[MAX_PATH];
						strcpy(buffer, path);
						strcat(buffer, filename);
						ASSERT(strlen(buffer)<MAX_PATH);
						ss->AddString(buffer);*/
					}

					return true;
				}
			};

			FindFilesParams FFP;
			FFP.mMask = "PINT_*.dll";
//			FFP.mMask = "*.*";

//			String Root;
//			Status = GetPath(ExeFilename, Root);
//			strcpy(FFP.mDir, Root.Get());
			strcpy(FFP.mDir, gRoot->Get());

			_FindFiles(FFP, Local::FindFilesCB, &ss);
		}

		udword NbS = ss.GetNbStrings();
		for(udword i=0;i<NbS;i++)
		{
			const String* CurrentS = ss.GetStrings()[i];
			Main->Add(CurrentS->Get(), 0, (void*)CurrentS->Get(), TRUE);
		}

		Main->Update();
		Main->Resize();

		int Result = MyApp.Run();

		DELETESINGLE(Main);
	}

	InitAll(null);

	atexit(gCleanup);

	PEEL_InitGUI();

	timeBeginPeriod(1);

	if(gCandidateTest)
		ActivateTest();

	glutMainLoop();

	return 0;
}

///////////////////////////////////////////////////////////////////////////////

const char* GUIHelper::Convert(float value)
{
	static char ConvertBuffer[256];
//	ConvertAndRemoveTrailingZeros(ConvertBuffer, value);
	sprintf(ConvertBuffer, "%g", value);
	return ConvertBuffer;
}

IceWindow* GUIHelper::CreateMainWindow(Container*& gui, IceWidget* parent, udword id, const char* label)
{
	ASSERT(!gui);
	gui = ICE_NEW(Container);
	ASSERT(gui);

	WindowDesc WD;
	WD.mID		= id;
	WD.mParent	= parent;
	WD.mX		= 100;
	WD.mY		= 100;
	WD.mWidth	= 256;
	WD.mHeight	= 300;
	WD.mLabel	= label;
//	WD.mType	= WINDOW_NORMAL;
	WD.mType	= WINDOW_DIALOG;
//	WD.mType	= WINDOW_MODALDIALOG;
//	WD.mStyle	= WSTYLE_DLGFRAME;
	IceWindow* Main = ICE_NEW(IceWindow)(WD);
	gui->Add(udword(Main));
	Main->SetVisible(true);
	return Main;
}

IceLabel* GUIHelper::CreateLabel(IceWidget* parent, sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner)
{
	LabelDesc LD;
	LD.mParent		= parent;
	LD.mX			= x;
	LD.mY			= y;
	LD.mWidth		= width;
	LD.mHeight		= height;
	LD.mLabel		= label;
	IceLabel* Label = ICE_NEW(IceLabel)(LD);
	Label->SetVisible(true);
	if(owner)
		owner->Add(udword(Label));
	return Label;
}

IceCheckBox* GUIHelper::CreateCheckBox(IceWidget* parent, udword id, sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, bool state, CBCallback callback, const char* tooltip)
{
	CheckBoxDesc CBD;
	CBD.mID			= id;
	CBD.mParent		= parent;
	CBD.mX			= x;
	CBD.mY			= y;
	CBD.mWidth		= width;
	CBD.mHeight		= height;
	CBD.mLabel		= label;
	CBD.mChecked	= state;
	CBD.mCallback	= callback;
	IceCheckBox* CB = ICE_NEW(IceCheckBox)(CBD);
	CB->SetVisible(true);
	if(owner)
		owner->Add(udword(CB));

	if(tooltip)
		CB->AddToolTip(tooltip);

	return CB;
}

IceEditBox* GUIHelper::CreateEditBox(IceWidget* parent, udword id, sdword x, sdword y, sdword width, sdword height, const char* label, Container* owner, EditBoxFilter filter, EBCallback callback, const char* tooltip)
{
	EditBoxDesc EBD;
	EBD.mID			= id;
	EBD.mParent		= parent;
	EBD.mX			= x;
	EBD.mY			= y;
	EBD.mWidth		= width;
	EBD.mHeight		= height;
	EBD.mLabel		= label;
	EBD.mFilter		= filter;
	EBD.mCallback	= callback;
	IceEditBox* EB = ICE_NEW(IceEditBox)(EBD);
	EB->SetVisible(true);
	if(owner)
		owner->Add(udword(EB));

	if(tooltip)
		EB->AddToolTip(tooltip);

	return EB;
}

static Container*	gMainGUI = null;
static IceEditBox*	gEditBox_CameraSpeed = null;
static IceEditBox*	gEditBox_RaytracingDistance = null;
static IceComboBox*	gComboBox_CurrentTool = null;
static IceComboBox*	gComboBox_ProfilingUnits = null;
static IceComboBox*	gComboBox_SQProfilingMode = null;
static IceComboBox*	gComboBox_SQRaycastMode = null;
static IceComboBox*	gComboBox_RaytracingResolution = null;
static IceRadioButton*	gRadioButton_RT_Disabled = null;
static IceRadioButton*	gRadioButton_RT_ST = null;
static IceRadioButton*	gRadioButton_RT_MT = null;

static IceEditBox*	gEditBox_PickingForce = null;
static IceLabel*	gLabel_PickingForce = null;

static IceEditBox*	gEditBox_ImpulseMagnitude = null;
static IceLabel*	gLabel_ImpulseMagnitude = null;

static IceEditBox*	gEditBox_BoxSize = null;
static IceLabel*	gLabel_BoxSize = null;
static IceEditBox*	gEditBox_BoxMass = null;
static IceLabel*	gLabel_BoxMass = null;
static IceEditBox*	gEditBox_BoxVelocity = null;
static IceLabel*	gLabel_BoxVelocity = null;

enum MainGUIElement
{
	MAIN_GUI_MAIN,
	//
	MAIN_GUI_RANDOMIZE_ORDER,
	MAIN_GUI_TRASH_CACHE,
	MAIN_GUI_ENABLE_VSYNC,
	MAIN_GUI_COMMA_SEPARATOR,
//	MAIN_GUI_PAUSED,
	//
	MAIN_GUI_CAMERA_SPEED,
	MAIN_GUI_RAYTRACING_DISTANCE,
	MAIN_GUI_PICKING_FORCE,
	MAIN_GUI_IMPULSE_MAGNITUDE,
	MAIN_GUI_BOX_SIZE,
	MAIN_GUI_BOX_MASS,
	MAIN_GUI_BOX_VELOCITY,
	//
	MAIN_GUI_CURRENT_TOOL,
	MAIN_GUI_PROFILING_UNITS,
	MAIN_GUI_SQ_PROFILING_MODE,
	MAIN_GUI_SQ_RAYCAST_MODE,
	MAIN_GUI_RAYTRACING_RESOLUTION,
	//
	MAIN_GUI_RAYTRACING_TEST_DISABLED,
	MAIN_GUI_RAYTRACING_TEST_ST,
	MAIN_GUI_RAYTRACING_TEST_MT,
	//
	MAIN_GUI_EXECUTE_SCRIPT,
	//
	MAIN_GUI_DUMMY,
};

static volatile int trashSum = 0;
static int gTrashCacheThread(void* user_data)
{
	while(1)
	{
		printf("I'm trashing your cache, hahaha!\n");

		{
			const int size = 1024*1024*64;
			void* buf_ = ICE_ALLOC(size);
			volatile int* buf = (int*)buf_;
			for (int j = 0; j < size/sizeof(buf[0]); j++)
			{
				trashSum += buf[j];
				buf[j] = 0xdeadbeef;
			}
			ICE_FREE(buf_);
		}
	}
	return 1;
}

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	switch(check_box.GetID())
	{
		case MAIN_GUI_RANDOMIZE_ORDER:
			gRandomizeOrder = checked;
			break;
		case MAIN_GUI_TRASH_CACHE:
			gTrashCache = checked;
/*			if(checked)
			{
				IceThread* Thread1 = CreateThread(gTrashCacheThread, null);
			}*/
			break;
/*		case MAIN_GUI_PAUSED:
			gPaused = checked;
			break;*/
		case MAIN_GUI_ENABLE_VSYNC:
			gEnableVSync = checked;
			GL_SelectVSYNC(gEnableVSync);
			break;
		case MAIN_GUI_COMMA_SEPARATOR:
			gCommaSeparator = checked;
			break;
	}
}

static const char* gTooltip_CurrentTool			= "Define what happens when pressing the right mouse button in simulation mode";
static const char* gTooltip_CameraSpeed			= "Camera displacement speed when using the arrow keys";
static const char* gTooltip_PickingForce		= "Coeff multiplier for picking force";
static const char* gTooltip_ImpulseMagnitude	= "Magnitude of impulse applied to object";
static const char* gTooltip_BoxSize				= "Size of shot box";
static const char* gTooltip_BoxMass				= "Mass of shot box";
static const char* gTooltip_BoxVelocity			= "Velocity of shot box";
static const char* gTooltip_RDTSC				= "Define profiling units in K-Cycles (checked) or ms (unchecked)";
static const char* gTooltip_RecordMemory		= "Profile performance (unchecked) or memory usage (checked)";
static const char* gTooltip_RaytracingTest		= "Raytrace current scene from current camera pose, using scene-level raycast calls. Results are dumped to console.";
static const char* gTooltip_RandomizeOrder		= "Randomize order in which physics engines are updated";
static const char* gTooltip_TrashCache			= "Trash cache after each simulation";
static const char* gTooltip_VSYNC				= "Enable/disable v-sync";
static const char* gTooltip_CommaSeparator		= "Use ',' or ';' as separator character in saved Excel files";
static const char* gTooltip_RaycastMode			= "Desired mode for SQ raycast tests. 'Closest' returns one closest hit, 'Any' returns the first hit and early exits, 'All' collects all hits touched by the ray.";

static void gPEEL_PollRadioButtons()
{
	if(gRadioButton_RT_Disabled->IsChecked())
		gRaytracingTest = RAYTRACING_TEST_DISABLED;
	else if(gRadioButton_RT_ST->IsChecked())
		gRaytracingTest = RAYTRACING_TEST_ST;
	else if(gRadioButton_RT_MT->IsChecked())
		gRaytracingTest = RAYTRACING_TEST_MT;
}

static void gPEEL_GetOptionsFromGUI()
{
	gImpulseMagnitude = GetFromEditBox(gImpulseMagnitude, gEditBox_ImpulseMagnitude, 0.0f, MAX_FLOAT);
	gBoxSize = GetFromEditBox(gBoxSize, gEditBox_BoxSize, 0.0f, MAX_FLOAT);
	gBoxMass = GetFromEditBox(gBoxMass, gEditBox_BoxMass, 0.0f, MAX_FLOAT);
	gBoxVelocity = GetFromEditBox(gBoxVelocity, gEditBox_BoxVelocity, 0.0f, MAX_FLOAT);
	gPickingForce = GetFromEditBox(gPickingForce, gEditBox_PickingForce, 0.0f, MAX_FLOAT);
	gCameraSpeed = GetFromEditBox(gCameraSpeed, gEditBox_CameraSpeed, 0.0f, MAX_FLOAT);
	gRTDistance = GetFromEditBox(gRTDistance, gEditBox_RaytracingDistance, 0.0f, MAX_FLOAT);

	if(gComboBox_CurrentTool)
	{
		const udword Index = gComboBox_CurrentTool->GetSelectedIndex();
		gCurrentTool = ToolIndex(Index);
	}

	if(gComboBox_ProfilingUnits)
	{
		const udword Index = gComboBox_ProfilingUnits->GetSelectedIndex();
		gProfilingUnits = ProfilingUnits(Index);
	}

	if(gComboBox_SQProfilingMode)
	{
		const udword Index = gComboBox_SQProfilingMode->GetSelectedIndex();
		gSQProfilingMode = SQProfilingMode(Index);
	}

	if(gComboBox_SQRaycastMode)
	{
		const udword Index = gComboBox_SQRaycastMode->GetSelectedIndex();
		gSQRaycastMode = SQRaycastMode(Index);
		//gRaycastClosest = gSQRaycastMode==SQ_RAYCAST_CLOSEST;
		gRaycastMode = Index;
	}

	if(gComboBox_RaytracingResolution)
	{
		const udword Index = gComboBox_RaytracingResolution->GetSelectedIndex();
		gRaytracingResolution = RaytracingResolution(Index);
	}
}

static void GetFromEditBox(float& value, const IceEditBox* edit_box, float min_value, float max_value)
{
	if(edit_box)
	{
		float Value;
		bool status = edit_box->GetTextAsFloat(Value);
		ASSERT(status);
		ASSERT(Value>=min_value && Value<=max_value);
		value = Value;
	}
}

static void gEBCallback(const IceEditBox& edit_box, udword param, void* user_data)
{
//	printf("gEBCallback\n");
	if(edit_box.GetID()==MAIN_GUI_CAMERA_SPEED)
		GetFromEditBox(gCameraSpeed, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_RAYTRACING_DISTANCE)
		GetFromEditBox(gRTDistance, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_PICKING_FORCE)
		GetFromEditBox(gPickingForce, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_IMPULSE_MAGNITUDE)
		GetFromEditBox(gImpulseMagnitude, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_BOX_SIZE)
		GetFromEditBox(gBoxSize, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_BOX_MASS)
		GetFromEditBox(gBoxMass, &edit_box, 0.0f, MAX_FLOAT);
	else if(edit_box.GetID()==MAIN_GUI_BOX_VELOCITY)
		GetFromEditBox(gBoxVelocity, &edit_box, 0.0f, MAX_FLOAT);
}

class ToolComboBox : public IceComboBox
{
	public:
					ToolComboBox(const ComboBoxDesc& desc) : IceComboBox(desc){}

	virtual	void	OnComboBoxEvent(ComboBoxEvent event)
	{
		if(event==CBE_SELECTION_CHANGED)
		{
			const ToolIndex SelectedTool = ToolIndex(GetSelectedIndex());
//			if(SelectedTool!=gCurrentTool)
			{
				gCurrentTool = SelectedTool;
//				printf("OnComboBoxEvent: %d\n", gCurrentTool);
				// Following stuff is hardcoded, could be better designed/etc later if needed. Not worth it atm.
				// We hide the UI elements that are not related to currently selected tool.
				gEditBox_PickingForce->SetVisible(false);
				gLabel_PickingForce->SetVisible(false);
				gEditBox_ImpulseMagnitude->SetVisible(false);
				gLabel_ImpulseMagnitude->SetVisible(false);
				gEditBox_BoxSize->SetVisible(false);
				gLabel_BoxSize->SetVisible(false);
				gEditBox_BoxMass->SetVisible(false);
				gLabel_BoxMass->SetVisible(false);
				gEditBox_BoxVelocity->SetVisible(false);
				gLabel_BoxVelocity->SetVisible(false);
				if(SelectedTool==TOOL_PICKING)
				{
					gEditBox_PickingForce->SetVisible(true);
					gLabel_PickingForce->SetVisible(true);
				}
				else if(SelectedTool==TOOL_ADD_IMPULSE)
				{
					gEditBox_ImpulseMagnitude->SetVisible(true);
					gLabel_ImpulseMagnitude->SetVisible(true);
				}
				else if(SelectedTool==TOOL_SHOOT_BOX)
				{
					gEditBox_BoxSize->SetVisible(true);
					gLabel_BoxSize->SetVisible(true);
					gEditBox_BoxMass->SetVisible(true);
					gLabel_BoxMass->SetVisible(true);
					gEditBox_BoxVelocity->SetVisible(true);
					gLabel_BoxVelocity->SetVisible(true);
				}
			}
		}
	}
};

class RTComboBox : public IceComboBox
{
	public:
					RTComboBox(const ComboBoxDesc& desc) : IceComboBox(desc){}

	virtual	void	OnComboBoxEvent(ComboBoxEvent event)
	{
		if(event==CBE_SELECTION_CHANGED)
		{
			gRaytracingResolution = RaytracingResolution(GetSelectedIndex());
		}
	}
};

class SQComboBox : public IceComboBox
{
	public:
					SQComboBox(const ComboBoxDesc& desc) : IceComboBox(desc){}

	virtual	void	OnComboBoxEvent(ComboBoxEvent event)
	{
		if(event==CBE_SELECTION_CHANGED)
		{
			gSQRaycastMode = SQRaycastMode(GetSelectedIndex());
			//gRaycastClosest = gSQRaycastMode==SQ_RAYCAST_CLOSEST;
			gRaycastMode = GetSelectedIndex();
			ResetSQHelpersHitData();
		}
	}
};

static void gButtonCallback(IceButton& button, void* user_data)
{
	if(1)
	{
		FILESELECTCREATE Create;
		Create.mFilter			= "Text files (*.txt)|*.txt|All Files (*.*)|*.*||";
		Create.mFileName		= "TestScript.txt";
		Create.mInitialDir		= *gRoot;
		Create.mCaptionTitle	= "Select script file";
		Create.mDefExt			= "txt";

		String ScriptFilename;
		if(!FileselectOpenSingle(Create, ScriptFilename))
			return;

		ExecuteScript(ScriptFilename);

		AutomatedTests* AutoTests = GetAutomatedTests();
		if(AutoTests && AutoTests->IsValid())
		{
			gRender = AutoTests->mRendering;
			gRandomizeOrder = AutoTests->mRandomizeOrder;
			gTrashCache = AutoTests->mTrashCache;

			AutomatedTest* Test = AutoTests->GetCurrentTest();
			ActivateTest(Test->mTest);
		}
	}
}

static void PEEL_InitGUI()
{
	IceWidget* Parent = null;
	IceTabControl* TabControl = null;
	if(1)
	{
		gMainGUI = ICE_NEW(Container);

		const udword BorderSize = 5;
		const udword MainWidth = 512;
		const udword MainHeight = 768;

		// Main window
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= MAIN_GUI_POS_X;
		WD.mY		= MAIN_GUI_POS_Y;
		WD.mWidth	= MainWidth;
		WD.mHeight	= MainHeight;
		WD.mLabel	= "Options";
		WD.mType	= WINDOW_DIALOG;
//		WD.mStyle	= WSTYLE_STATIC_EDGES;
		IceWindow* OptionsWindow = ICE_NEW(IceWindow)(WD);
		gMainGUI->Add(udword(OptionsWindow));
		OptionsWindow->SetVisible(true);

		// Create tab control
		TabControlDesc TCD;
		TCD.mParent	= OptionsWindow;
		TCD.mX		= 0;
		TCD.mY		= 0;
		TCD.mWidth	= MainWidth - WD.mX - BorderSize;
		TCD.mHeight	= MainHeight - BorderSize*2;
		TabControl = ICE_NEW(IceTabControl)(TCD);
		gMainGUI->Add(udword(TabControl));

		Parent = OptionsWindow;

		{
			WindowDesc WD;
			WD.mParent	= OptionsWindow;
			WD.mX		= 100;
			WD.mY		= 100;
			WD.mWidth	= 256;
			WD.mHeight	= 300;
			WD.mLabel	= "Main options";
		//	WD.mType	= WINDOW_NORMAL;
			WD.mType	= WINDOW_DIALOG;
		//	WD.mType	= WINDOW_MODALDIALOG;
		//	WD.mStyle	=
			IceWindow* MainOptions = ICE_NEW(IceWindow)(WD);
			gMainGUI->Add(udword(MainOptions));
			MainOptions->SetVisible(true);
			TabControl->Add(MainOptions, "Generic");

			{
				const udword TitleWindowWidth = TITLE_WIDTH + 24*2;
				const udword TitleWindowHeight = TITLE_HEIGHT + 24*2+8;
				CreateTitleWindow(*gMainGUI, MainOptions, 20, 4, TitleWindowWidth, TitleWindowHeight);
			}

			const sdword YStep = 20;
			sdword y = TITLE_HEIGHT + 60;

			{
				static const char* Date = __DATE__ " at " __TIME__;
				gGUIHelper.CreateLabel(MainOptions, 4, y, 300, 20, _F("Version %s - Build: %s", gVersion, Date), gMainGUI);
				y += YStep;
//				y += YStep;
			}

			{
				gGUIHelper.CreateCheckBox(MainOptions, MAIN_GUI_RANDOMIZE_ORDER, 4, y, 200, 20, "Randomize physics engines order", gMainGUI, gRandomizeOrder, gCheckBoxCallback, gTooltip_RandomizeOrder);
				y += YStep;

				gGUIHelper.CreateCheckBox(MainOptions, MAIN_GUI_TRASH_CACHE, 4, y, 200, 20, "Trash cache", gMainGUI, gTrashCache, gCheckBoxCallback, gTooltip_TrashCache);
				y += YStep;

				gGUIHelper.CreateCheckBox(MainOptions, MAIN_GUI_ENABLE_VSYNC, 4, y, 200, 20, "VSYNC", gMainGUI, gEnableVSync, gCheckBoxCallback, gTooltip_VSYNC);
				y += YStep;

				gGUIHelper.CreateCheckBox(MainOptions, MAIN_GUI_COMMA_SEPARATOR, 4, y, 200, 20, "Use comma separator", gMainGUI, gCommaSeparator, gCheckBoxCallback, gTooltip_CommaSeparator);
				y += YStep;
			}

			const sdword OffsetX = 90;
			const sdword EditBoxWidth = 60;
			const sdword LabelOffsetY = 2;
			{
				gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "Camera speed:", gMainGUI);
				gEditBox_CameraSpeed = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_CAMERA_SPEED, 4+OffsetX, y, EditBoxWidth, 20, gGUIHelper.Convert(gCameraSpeed), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_CameraSpeed);
				y += YStep;
			}

			{
				gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "Profiling units:", gMainGUI);
				ComboBoxDesc CBBD;
				CBBD.mID		= MAIN_GUI_PROFILING_UNITS;
				CBBD.mParent	= MainOptions;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "Profiling units";
				gComboBox_ProfilingUnits = ICE_NEW(IceComboBox)(CBBD);
				gMainGUI->Add(udword(gComboBox_ProfilingUnits));
				gComboBox_ProfilingUnits->Add("K-Cycles (RDTSC)");
				gComboBox_ProfilingUnits->Add("ms (timeGetTime)");
				gComboBox_ProfilingUnits->Add("us (QPC)");
				gComboBox_ProfilingUnits->Select(gProfilingUnits);
				gComboBox_ProfilingUnits->SetVisible(true);
				y += YStep;
			}
			{
				gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "SQ profiling mode:", gMainGUI);
				ComboBoxDesc CBBD;
				CBBD.mID		= MAIN_GUI_SQ_PROFILING_MODE;
				CBBD.mParent	= MainOptions;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "SQ profiling mode";
				gComboBox_SQProfilingMode = ICE_NEW(IceComboBox)(CBBD);
				gMainGUI->Add(udword(gComboBox_SQProfilingMode));
				gComboBox_SQProfilingMode->Add("Simulation");
				gComboBox_SQProfilingMode->Add("SQ queries");
				gComboBox_SQProfilingMode->Add("Combined");
				gComboBox_SQProfilingMode->Select(gSQProfilingMode);
				gComboBox_SQProfilingMode->SetVisible(true);
				y += YStep;
			}
			{
				gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "Raycast mode:", gMainGUI);
				ComboBoxDesc CBBD;
				CBBD.mID		= MAIN_GUI_SQ_RAYCAST_MODE;
				CBBD.mParent	= MainOptions;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "SQ raycast mode";
				gComboBox_SQRaycastMode = ICE_NEW(SQComboBox)(CBBD);
				gMainGUI->Add(udword(gComboBox_SQRaycastMode));
				gComboBox_SQRaycastMode->Add("Raycast closest");
				gComboBox_SQRaycastMode->Add("Raycast any");
				gComboBox_SQRaycastMode->Add("Raycast all");
				gComboBox_SQRaycastMode->Select(gSQRaycastMode);
				gComboBox_SQRaycastMode->SetVisible(true);
				gComboBox_SQRaycastMode->AddToolTip(gTooltip_RaycastMode);
				y += YStep;
			}

			{
				gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "Current tool:", gMainGUI);
				ComboBoxDesc CBBD;
				CBBD.mID		= MAIN_GUI_CURRENT_TOOL;
				CBBD.mParent	= MainOptions;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "Current tool";
				gComboBox_CurrentTool = ICE_NEW(ToolComboBox)(CBBD);
				gMainGUI->Add(udword(gComboBox_CurrentTool));
				gComboBox_CurrentTool->Add("Picking");
				gComboBox_CurrentTool->Add("Add impulse");
				gComboBox_CurrentTool->Add("Shoot box");
				gComboBox_CurrentTool->Add("Camera tracking");
//				gComboBox_CurrentTool->Add("Custom control");
				gComboBox_CurrentTool->Select(gCurrentTool);
				gComboBox_CurrentTool->SetVisible(true);
				gComboBox_CurrentTool->AddToolTip(gTooltip_CurrentTool);
				y += YStep;
			}
			{
				y += YStep;
				const sdword ToolSpecificAreaWidth = 300;
				const sdword ToolSpecificAreaHeight = 100;

				// Picking tool
				{
					sdword y2 = y;
					y2 += YStep;

					const sdword OffsetX = 90;
					const sdword EditBoxWidth = 60;
					const sdword LabelOffsetY = 2;
					{
						gLabel_PickingForce = gGUIHelper.CreateLabel(MainOptions, 14, y2+LabelOffsetY, 90, 20, "Picking force:", gMainGUI);
						gEditBox_PickingForce = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_PICKING_FORCE, 14+OffsetX, y2, EditBoxWidth, 20, gGUIHelper.Convert(gPickingForce), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_PickingForce);
						y2 += YStep;
					}
				}

				// Add impulse tool
				{
					sdword y2 = y;
					y2 += YStep;

					const sdword OffsetX = 100;
					const sdword EditBoxWidth = 60;
					const sdword LabelOffsetY = 2;
					{
						gLabel_ImpulseMagnitude = gGUIHelper.CreateLabel(MainOptions, 14, y2+LabelOffsetY, 100, 20, "Impulse magnitude:", gMainGUI);
						gEditBox_ImpulseMagnitude = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_IMPULSE_MAGNITUDE, 14+OffsetX, y2, EditBoxWidth, 20, gGUIHelper.Convert(gImpulseMagnitude), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_ImpulseMagnitude);
						y2 += YStep;
					}

					gLabel_ImpulseMagnitude->SetVisible(false);
					gEditBox_ImpulseMagnitude->SetVisible(false);
				}

				// Shoot box tool
				{
					sdword y2 = y;
					y2 += YStep;

					const sdword OffsetX = 100;
					const sdword EditBoxWidth = 60;
					const sdword LabelOffsetY = 2;
					{
						gLabel_BoxSize = gGUIHelper.CreateLabel(MainOptions, 14, y2+LabelOffsetY, 100, 20, "Box size:", gMainGUI);
						gEditBox_BoxSize = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_BOX_SIZE, 14+OffsetX, y2, EditBoxWidth, 20, gGUIHelper.Convert(gBoxSize), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_BoxSize);
						y2 += YStep;

						gLabel_BoxMass = gGUIHelper.CreateLabel(MainOptions, 14, y2+LabelOffsetY, 100, 20, "Box mass:", gMainGUI);
						gEditBox_BoxMass = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_BOX_MASS, 14+OffsetX, y2, EditBoxWidth, 20, gGUIHelper.Convert(gBoxMass), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_BoxMass);
						y2 += YStep;

						gLabel_BoxVelocity = gGUIHelper.CreateLabel(MainOptions, 14, y2+LabelOffsetY, 100, 20, "Box velocity:", gMainGUI);
						gEditBox_BoxVelocity = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_BOX_VELOCITY, 14+OffsetX, y2, EditBoxWidth, 20, gGUIHelper.Convert(gBoxVelocity), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback, gTooltip_BoxVelocity);
						y2 += YStep;
					}

					gLabel_BoxSize->SetVisible(false);
					gEditBox_BoxSize->SetVisible(false);
					gLabel_BoxMass->SetVisible(false);
					gEditBox_BoxMass->SetVisible(false);
					gLabel_BoxVelocity->SetVisible(false);
					gEditBox_BoxVelocity->SetVisible(false);
				}


				IceEditBox* tmp = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_DUMMY, 10, y, ToolSpecificAreaWidth, ToolSpecificAreaHeight, "=== Tool-specific settings ===", gMainGUI, EDITBOX_TEXT, null);
//				tmp->AddToolTip(gTooltip_RaytracingTest);
				tmp->SetReadOnly(true);
				y += ToolSpecificAreaHeight;
			}

			if(1)
			{
				y = TITLE_HEIGHT + 70 + 10;
//				y += YStep;
				const sdword saved = y;
				y += YStep;

				RadioButtonDesc RBD;
				RBD.mNewGroup	= true;
				RBD.mID			= MAIN_GUI_RAYTRACING_TEST_DISABLED;
				RBD.mParent		= MainOptions;
//				RBD.mX			= 10;
				RBD.mX			= 310;
				RBD.mY			= y;
				RBD.mWidth		= 100;
				RBD.mHeight		= 20;
				RBD.mLabel		= "Disabled";
				gRadioButton_RT_Disabled = ICE_NEW(IceRadioButton)(RBD);
				gMainGUI->Add(udword(gRadioButton_RT_Disabled));
				//gRadioButton_RT_Disabled->AddToolTip(gTooltip_RaytracingTest);
				y += YStep;

				RBD.mNewGroup	= false;
				RBD.mID			= MAIN_GUI_RAYTRACING_TEST_ST;
				RBD.mY			= y;
				RBD.mLabel		= "Single threaded";
				gRadioButton_RT_ST = ICE_NEW(IceRadioButton)(RBD);
				gMainGUI->Add(udword(gRadioButton_RT_ST));
				//gRadioButton_RT_ST->AddToolTip(gTooltip_RaytracingTest);
				y += YStep;

				RBD.mNewGroup	= false;
				RBD.mID			= MAIN_GUI_RAYTRACING_TEST_MT;
				RBD.mY			= y;
				RBD.mLabel		= "Multi threaded";
				gRadioButton_RT_MT = ICE_NEW(IceRadioButton)(RBD);
				gMainGUI->Add(udword(gRadioButton_RT_MT));
				//gRadioButton_RT_MT->AddToolTip(gTooltip_RaytracingTest);
				y += YStep;

				{
					gGUIHelper.CreateLabel(MainOptions, 310, y+LabelOffsetY, 50, 20, "Max dist:", gMainGUI);
					gEditBox_RaytracingDistance = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_RAYTRACING_DISTANCE, 310+50, y, EditBoxWidth, 20, gGUIHelper.Convert(gRTDistance), gMainGUI, EDITBOX_FLOAT_POSITIVE, gEBCallback/*, gTooltip_CameraSpeed*/);
				}

				{
					y += YStep;
					//gGUIHelper.CreateLabel(MainOptions, 4, y+LabelOffsetY, 90, 20, "RT resolution:", gMainGUI);
					ComboBoxDesc CBBD;
					CBBD.mID		= MAIN_GUI_RAYTRACING_RESOLUTION;
					CBBD.mParent	= MainOptions;
					CBBD.mX			= 310;
					CBBD.mY			= y;
					CBBD.mWidth		= 150;
					CBBD.mHeight	= 20;
					CBBD.mLabel		= "RT resolution";
					gComboBox_RaytracingResolution = ICE_NEW(RTComboBox)(CBBD);
					gMainGUI->Add(udword(gComboBox_RaytracingResolution));
					gComboBox_RaytracingResolution->Add("64x64 rays");
					gComboBox_RaytracingResolution->Add("128*128 rays");
					gComboBox_RaytracingResolution->Add("256*256 rays");
					gComboBox_RaytracingResolution->Add("512*512 rays");
					gComboBox_RaytracingResolution->Select(gRaytracingResolution);
					gComboBox_RaytracingResolution->SetVisible(true);
					y += YStep;
				}

//				IceEditBox* tmp = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_DUMMY, 4, saved, 120, 100, "=== Raytracing test ===", gMainGUI, EDITBOX_TEXT, null);
				IceEditBox* tmp = gGUIHelper.CreateEditBox(MainOptions, MAIN_GUI_DUMMY, 296, saved, 180, 140, "=== Raytracing test ===", gMainGUI, EDITBOX_TEXT, null);
				tmp->AddToolTip(gTooltip_RaytracingTest);
				tmp->SetReadOnly(true);
			}

			{
				y += YStep * 10;

				ButtonDesc BD;
//				BD.mStyle		= ;
				BD.mCallback	= gButtonCallback;
				BD.mID			= MAIN_GUI_EXECUTE_SCRIPT;
				BD.mParent		= MainOptions;
				BD.mX			= 20;
				BD.mY			= y;
				BD.mWidth		= 450;
				BD.mHeight		= 20;
				BD.mLabel		= "Execute script";
				IceButton* gButton_ExecuteScript = ICE_NEW(IceButton)(BD);
				gMainGUI->Add(udword(gButton_ExecuteScript));
				gButton_ExecuteScript->SetVisible(true);
				y += YStep;
			}

			if(0)
			{
				WindowDesc WD;
				WD.mParent	= null;
				WD.mX		= 0;
				WD.mY		= 0;
				WD.mWidth	= 512;
				WD.mHeight	= 80;
				WD.mLabel	= "Loading... Please wait";
				WD.mType	= WINDOW_DIALOG;
//				WD.mType	= WINDOW_POPUP;
				WD.mStyle	= WSTYLE_ALWAYS_ON_TOP|WSTYLE_CLIENT_EDGES;
				WD.Center();
				IceWindow* TestWindow = ICE_NEW(IceWindow)(WD);
				gMainGUI->Add(udword(TestWindow));
				TestWindow->SetVisible(true);

				ProgressBarDesc PBD;
//				PBD.mID			= MAIN_GUI_CURRENT_TOOL;
				PBD.mParent		= TestWindow;
				PBD.mX			= 20;
				PBD.mY			= 10;
				PBD.mWidth		= 512-20*2;
				PBD.mHeight		= 20;
//				PBD.mLabel		= "Current tool";
//				PBD.mStyle		= PROGRESSBAR_SMOOTH;
				IceProgressBar* ProgressBar = ICE_NEW(IceProgressBar)(PBD);
				gMainGUI->Add(udword(ProgressBar));
				ProgressBar->SetVisible(true);
				ProgressBar->SetTotalSteps(100);
				ProgressBar->SetValue(40);
			}
		}
	}

	for(udword i=0;i<gNbPlugIns;i++)
	{
		IceWindow* Main = gPlugIns[i]->InitGUI(Parent, gGUIHelper);
		if(Main)
		{
			Pint* Engine = gPlugIns[i]->GetPint();
			const udword Flags = Engine->GetFlags();
			if(Flags & PINT_HAS_RAYTRACING_WINDOW)
			{
				const udword RaytracingWindowWidth = RAYTRACING_DISPLAY_WIDTH + 24*2;
				const udword RaytracingWindowHeight = RAYTRACING_DISPLAY_HEIGHT + 24*2+8;
				gRaytracingWindows[i] = CreateRaytracingWindow(*gMainGUI, Main, 512 - RaytracingWindowWidth - 20, 768 - RaytracingWindowHeight - 50, RaytracingWindowWidth, RaytracingWindowHeight);
			}
			if(TabControl)
				TabControl->Add(Main, Engine->GetName());
		}
	}
}

static void PEEL_CloseGUI()
{
	if(gMainGUI)
	{
		const udword Size = gMainGUI->GetNbEntries();
		for(udword i=0;i<Size;i++)
		{
			IceWidget* W = (IceWidget*)gMainGUI->GetEntry(i);
			DELETESINGLE(W);
		}
		DELETESINGLE(gMainGUI);
	}
	gLabel_ImpulseMagnitude = null;
	gEditBox_ImpulseMagnitude = null;

	gEditBox_BoxSize = null;
	gLabel_BoxSize = null;

	gEditBox_BoxMass = null;
	gLabel_BoxMass = null;

	gEditBox_BoxVelocity = null;
	gLabel_BoxVelocity = null;

	gLabel_PickingForce = null;
	gEditBox_PickingForce = null;

	gEditBox_CameraSpeed = null;
	gEditBox_RaytracingDistance = null;
	gComboBox_CurrentTool = null;
	gComboBox_ProfilingUnits = null;
	gComboBox_SQProfilingMode = null;
	gComboBox_SQRaycastMode = null;
	gComboBox_RaytracingResolution = null;
	gRadioButton_RT_Disabled = null;
	gRadioButton_RT_ST = null;
	gRadioButton_RT_MT = null;
}

Quat ShortestRotation(const Point& v0, const Point& v1)
{
	const float d = v0|v1;
	const Point cross = v0^v1;

	Quat q = d>-1.0f ? Quat(1.0f + d, cross.x, cross.y, cross.z)
//					: fabsf(v0.x)<0.1f ? Quat(0.0f, 0.0f, v0.z, -v0.y) : Quat(0.0f, v0.y, -v0.x, 0.0f);
					: fabsf(v0.x)<0.1f ? Quat(0.0f, 0.0f, v0.z, -v0.y) : Quat(0.0f, v0.y, -v0.x, 0.0f);
//	PxQuat q = d > -1 ? PxQuat(cross.x, cross.y, cross.z, 1 + d) : PxAbs(v0.x) < 0.1f ? PxQuat(0.0f, v0.z, -v0.y, 0.0f)
//	                                                                                  : PxQuat(v0.y, -v0.x, 0.0f, 0.0f);

	q.Normalize();

	return q;
}

static void TestCSVExport()
{
	if(!gRunningTest)
		return;

	const char* Filename;
	const char* SubName = gRunningTest->GetSubName();
	if(SubName)
		Filename = _F(".\\%s_%s.csv", gRunningTest->GetName(), SubName);
	else
		Filename = _F(".\\%s.csv", gRunningTest->GetName());

	FILE* globalFile = fopen(Filename, "w");
	if(!globalFile)
		return;

	if(gProfilingUnits==PROFILING_UNITS_RDTSC)
		fprintf_s(globalFile, "%s (K-Cycles)\n\n", gRunningTest->GetName());
	else if(gProfilingUnits==PROFILING_UNITS_TIME_GET_TIME)
		fprintf_s(globalFile, "%s (ms)\n\n", gRunningTest->GetName());
	else if(gProfilingUnits==PROFILING_UNITS_QPC)
		fprintf_s(globalFile, "%s (us)\n\n", gRunningTest->GetName());
	else ASSERT(0);

	const udword NbFrames = gFrameNb<=MAX_NB_RECORDED_FRAMES ? gFrameNb : MAX_NB_RECORDED_FRAMES;

	for(udword b=0;b<gNbEngines;b++)
	{
		if(!gEngines[b].mEnabled || !gEngines[b].mSupportsCurrentTest || !(gEngines[b].mEngine->GetFlags() & PINT_IS_ACTIVE))
			continue;

		if(gCommaSeparator)
			fprintf_s(globalFile, "%s, ", gEngines[b].mEngine->GetName());
		else
			fprintf_s(globalFile, "%s; ", gEngines[b].mEngine->GetName());

		for(udword i=0;i<NbFrames;i++)
		{
			if(gCommaSeparator)
				fprintf_s(globalFile, "%d, ", gEngines[b].mTiming.mRecorded[i].mTime);
			else
				fprintf_s(globalFile, "%d; ", gEngines[b].mTiming.mRecorded[i].mTime);
		}
		fprintf_s(globalFile, "\n");
	}

	fprintf_s(globalFile, "\n\n");

	fprintf_s(globalFile, "Memory usage (Kb):\n\n");

	for(udword b=0;b<gNbEngines;b++)
	{
		if(!gEngines[b].mEnabled || !gEngines[b].mSupportsCurrentTest || !(gEngines[b].mEngine->GetFlags() & PINT_IS_ACTIVE))
			continue;

		if(gCommaSeparator)
			fprintf_s(globalFile, "%s, ", gEngines[b].mEngine->GetName());
		else
			fprintf_s(globalFile, "%s; ", gEngines[b].mEngine->GetName());

		for(udword i=0;i<NbFrames;i++)
		{
			if(gCommaSeparator)
				fprintf_s(globalFile, "%d, ", gEngines[b].mTiming.mRecorded[i].mUsedMemory/1024);
			else
				fprintf_s(globalFile, "%d; ", gEngines[b].mTiming.mRecorded[i].mUsedMemory/1024);
		}
		fprintf_s(globalFile, "\n");
	}

	fclose(globalFile);
	gDisplayMessage = true;
	gDisplayMessageType = 0;
}

