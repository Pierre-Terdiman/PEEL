///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <delayimp.h>

#define USE_DELAY_LOADING
#ifdef USE_DELAY_LOADING
#pragma comment(lib, "delayimp")

FARPROC WINAPI delayHook(unsigned dliNotify, PDelayLoadInfo pdli)
{
    switch(dliNotify)
	{
        case dliStartProcessing:
        case dliNotePreGetProcAddress:
        case dliFailLoadLib:
        case dliFailGetProc:
        case dliNoteEndProcessing:
            break;

        case dliNotePreLoadLibrary:
			{
			printf("  Remapping %s\n", pdli->szDll);

            // If you want to return control to the helper, return 0.
            // Otherwise, return your own HMODULE to be used by the 
            // helper instead of having it call LoadLibrary itself.
    		if(strcmp(pdli->szDll, "PhysX3DEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3DEBUG_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CommonDEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CommonDEBUG_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CookingDEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CookingDEBUG_x86_3_4_CL_21578609.dll");

    		if(strcmp(pdli->szDll, "PhysX3CHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CHECKED_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CommonCHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CommonCHECKED_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CookingCHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CookingCHECKED_x86_3_4_CL_21578609.dll");

    		if(strcmp(pdli->szDll, "PhysX3PROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3PROFILE_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CommonPROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CommonPROFILE_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3CookingPROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3CookingPROFILE_x86_3_4_CL_21578609.dll");

    		if(strcmp(pdli->szDll, "PhysX3_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3Common_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3Common_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PhysX3Cooking_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3Cooking_x86_3_4_CL_21578609.dll");

    		if(strcmp(pdli->szDll, "PxFoundation_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxFoundation_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxFoundationDEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxFoundationDEBUG_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxFoundationCHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxFoundationCHECKED_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxFoundationPROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxFoundationPROFILE_x86_3_4_CL_21578609.dll");

    		if(strcmp(pdli->szDll, "PxPvdSDK_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxPvdSDK_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxPvdSDKDEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxPvdSDKDEBUG_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxPvdSDKCHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxPvdSDKCHECKED_x86_3_4_CL_21578609.dll");
    		if(strcmp(pdli->szDll, "PxPvdSDKPROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PxPvdSDKPROFILE_x86_3_4_CL_21578609.dll");

#ifdef PHYSX_SUPPORT_GPU
    		if(strcmp(pdli->szDll, "PhysX3Gpu_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3Gpu_3_4_CL_21578609_x86.dll");
    		if(strcmp(pdli->szDll, "PhysX3GpuDEBUG_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3GpuDEBUG_3_4_CL_21578609_x86.dll");
    		if(strcmp(pdli->szDll, "PhysX3GpuCHECKED_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3GpuCHECKED_3_4_CL_21578609_x86.dll");
    		if(strcmp(pdli->szDll, "PhysX3GpuPROFILE_x86.dll")==0)
				return (FARPROC)::LoadLibraryA("PhysX3GpuPROFILE_3_4_CL_21578609_x86.dll");
#endif
			}
            break;

        default :
            return NULL;
    }
    return NULL;
}

PfnDliHook __pfnDliNotifyHook2 = delayHook;
#endif

#ifdef USE_DELAY_LOADING
#include "common/windows/PxWindowsDelayLoadHook.h"
#include "pvd/windows/PxWindowsPvdDelayLoadHook.h"

	class MyDelayLoadHook : public PxDelayLoadHook
#ifdef PHYSX_SUPPORT_GPU
		, public PxGpuLoadHook
#endif
	{
	public:
		MyDelayLoadHook()
		{
			PxSetPhysXDelayLoadHook(this);
			PxSetPhysXCookingDelayLoadHook(this);
			PxSetPhysXCommonDelayLoadHook(this);
			PxPvdSetFoundationDelayLoadHook(this);
#ifdef PHYSX_SUPPORT_GPU
			PxSetPhysXGpuLoadHook(this);
			PxSetPhysXGpuDelayLoadHook(this);
#endif
		}
		virtual ~MyDelayLoadHook() {}

		/////

		virtual const char* getPhysXCommonDEBUGDllName() const
		{
			printf("Remapping PhysX3CommonDEBUG_x86.dll\n");
			return "PhysX3CommonDEBUG_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPhysXCommonCHECKEDDllName() const
		{
			printf("Remapping PhysX3CommonCHECKED_x86.dll\n");
			return "PhysX3CommonCHECKED_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPhysXCommonPROFILEDllName() const
		{
			printf("Remapping PhysX3CommonPROFILE_x86.dll\n");
			return "PhysX3CommonPROFILE_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPhysXCommonDllName() const
		{
			printf("Remapping PhysX3Common_x86.dll\n");
			return "PhysX3Common_x86_3_4_CL_21578609.dll";
		}

		/////

		virtual const char* getPxFoundationDEBUGDllName() const
		{
			printf("Remapping PxFoundationDEBUG\n");
			return "PxFoundationDEBUG_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxFoundationCHECKEDDllName() const
		{
			printf("Remapping PxFoundationCHECKED\n");
			return "PxFoundationCHECKED_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxFoundationPROFILEDllName() const
		{
			printf("Remapping PxFoundationPROFILE\n");
			return "PxFoundationPROFILE_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxFoundationDllName() const
		{
			printf("Remapping PxFoundation\n");
			return "PxFoundation_x86_3_4_CL_21578609.dll";
		}

		/////

		virtual const char* getPxPvdSDKDEBUGDllName() const
		{
			printf("Remapping PxPvdSDKDEBUG_x86.dll\n");
			return "PxPvdSDKDEBUG_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxPvdSDKCHECKEDDllName() const
		{
			printf("Remapping PxPvdSDKCHECKED_x86.dll\n");
			return "PxPvdSDKCHECKED_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxPvdSDKPROFILEDllName() const
		{
			printf("Remapping PxPvdSDKPROFILE_x86.dll\n");
			return "PxPvdSDKPROFILE_x86_3_4_CL_21578609.dll";
		}

		virtual const char* getPxPvdSDKDllName() const
		{
			printf("Remapping PxPvdSDK_x86.dll\n");
			return "PxPvdSDK_x86_3_4_CL_21578609.dll";
		}

#ifdef PHYSX_SUPPORT_GPU
		virtual const char* getPhysXGpuDEBUGDllName() const
		{
			printf("Remapping PhysX3GpuDEBUG_x86.dll\n");
			return "PhysX3GpuDEBUG_3_4_CL_21578609_x86.dll";
		}

		virtual const char* getPhysXGpuCHECKEDDllName() const
		{
			printf("Remapping PhysX3GpuCHECKED_x86.dll\n");
			return "PhysX3GpuCHECKED_3_4_CL_21578609_x86.dll";
		}

		virtual const char* getPhysXGpuPROFILEDllName() const
		{
			printf("Remapping PhysX3GpuPROFILE_x86.dll\n");
			return "PhysX3GpuPROFILE_3_4_CL_21578609_x86.dll";
		}

		virtual const char* getPhysXGpuDllName() const
		{
			printf("Remapping PhysX3Gpu_x86.dll\n");
			return "PhysX3Gpu_3_4_CL_21578609_x86.dll";
		}
#endif

	}gMyDelayHook;
#endif