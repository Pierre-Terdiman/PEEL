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
    switch (dliNotify) {
        case dliStartProcessing :

            // If you want to return control to the helper, return 0.
            // Otherwise, return a pointer to a FARPROC helper function
            // that will be used instead, thereby bypassing the rest 
            // of the helper.

            break;

        case dliNotePreLoadLibrary :
			{
			printf("Remapping %s\n", pdli->szDll);

            // If you want to return control to the helper, return 0.
            // Otherwise, return your own HMODULE to be used by the 
            // helper instead of having it call LoadLibrary itself.
    		if( strcmp( pdli->szDll, "PhysX3DEBUG_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3DEBUG_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CommonDEBUG_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CommonDEBUG_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CookingDEBUG_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CookingDEBUG_x86_3_3_2.dll" );

    		if( strcmp( pdli->szDll, "PhysX3CHECKED_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CHECKED_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CommonCHECKED_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CommonCHECKED_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CookingCHECKED_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CookingCHECKED_x86_3_3_2.dll" );

    		if( strcmp( pdli->szDll, "PhysX3PROFILE_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3PROFILE_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CommonPROFILE_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CommonPROFILE_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3CookingPROFILE_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3CookingPROFILE_x86_3_3_2.dll" );

    		if( strcmp( pdli->szDll, "PhysX3_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3Common_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3Common_x86_3_3_2.dll" );
    		if( strcmp( pdli->szDll, "PhysX3Cooking_x86.dll" ) == 0 )
				return (FARPROC)::LoadLibraryA( "PhysX3Cooking_x86_3_3_2.dll" );
			}
            break;

        case dliNotePreGetProcAddress :

            // If you want to return control to the helper, return 0.
            // If you choose you may supply your own FARPROC function 
            // address and bypass the helper's call to GetProcAddress.

            break;

        case dliFailLoadLib : 

            // LoadLibrary failed.
            // If you don't want to handle this failure yourself, return 0.
            // In this case the helper will raise an exception 
            // (ERROR_MOD_NOT_FOUND) and exit.
            // If you want to handle the failure by loading an alternate 
            // DLL (for example), then return the HMODULE for 
            // the alternate DLL. The helper will continue execution with 
            // this alternate DLL and attempt to find the
            // requested entrypoint via GetProcAddress.

            break;

        case dliFailGetProc :

            // GetProcAddress failed.
            // If you don't want to handle this failure yourself, return 0.
            // In this case the helper will raise an exception 
            // (ERROR_PROC_NOT_FOUND) and exit.
            // If you choose you may handle the failure by returning 
            // an alternate FARPROC function address.


            break;

        case dliNoteEndProcessing : 

            // This notification is called after all processing is done. 
            // There is no opportunity for modifying the helper's behavior
            // at this point except by longjmp()/throw()/RaiseException. 
            // No return value is processed.

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

	class MyDelayLoadHook : public PxDelayLoadHook
	{
	public:
		MyDelayLoadHook()
		{
//			PxDelayLoadHook::setPhysXInstance(this);
//			PxDelayLoadHook::setPhysXCookingInstance(this);
			PxSetPhysXDelayLoadHook(this);
			PxSetPhysXCookingDelayLoadHook(this);
		}
		virtual ~MyDelayLoadHook() {}

		virtual const char* getPhysXCommonDEBUGDllName() const
		{
			return "PhysX3CommonDEBUG_x86_3_3_2.dll";
		}

		virtual const char* getPhysXCommonCHECKEDDllName() const
		{
			return "PhysX3CommonCHECKED_x86_3_3_2.dll";
		}

		virtual const char* getPhysXCommonPROFILEDllName() const
		{
			return "PhysX3CommonPROFILE_x86_3_3_2.dll";
		}

		virtual const char* getPhysXCommonDllName() const
		{
			return "PhysX3Common_x86_3_3_2.dll";
		}

//		PX_PHYSX_CORE_API static void SetPhysXInstance(const PxDelayLoadHook* hook);
//		PX_PHYSX_CORE_API static void SetPhysXCookingInstance(const PxDelayLoadHook* hook);
	}gMyDelayHook;
#endif