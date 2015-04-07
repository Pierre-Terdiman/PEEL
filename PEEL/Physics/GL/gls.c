/*----------------------------------------------------------------------------

gls.c - This file contains the function and variable definitions for the
    GLSetup dynamic OpenGL driver linking.  Applications can load the
    standard opengl32.dll or any other standalone driver that obeys the
    linking protocol.

    See gls.h or http://www.glsetup.com/dev/gls.htm for more information
    and to report bugs.

    Copyright 1999 GLSetup Incorporated, All Rights Reserved.

    You have permission to use this code in your applications, commercial
    or otherwise.  If you distribute source code with your application,
    this file must not be modified.  Please don't mirror these files;
    instead, link to the URL above.
 
    Version 1.0.0.2 Beta
    Created 7-24-99

*/

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef WIN32_EXTRA_LEAN
#define WIN32_EXTRA_LEAN
#endif
#include <windows.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "gls.h"
#include "glsu.h"

/*----------------------------------------------------------------------------

File-scope variables and internal function declarations.

NOTE:  The gl function pointers are defined at the bottom of the file.

*/

#ifdef NDEBUG
#define GLS_STATIC static
#else
#define GLS_STATIC
#endif

GLS_STATIC DWORD _glsBehavior = 0;

GLS_STATIC HMODULE _glsGLDriverHandle, _glsGLUDriverHandle, _glsGDI32;
#define GLS_INVALID_DRIVER_INDEX 0xFFFFFFFF
GLS_STATIC int unsigned _glsLoadedDriverIndex = GLS_INVALID_DRIVER_INDEX;
GLS_STATIC char _glsGLDriverFilePath[MAX_PATH], _glsGLUDriverFilePath[MAX_PATH];
GLS_STATIC gls_driver_info _glsDriverInfos[3];
GLS_STATIC int unsigned _glsNumberOfDriverInfos = 0;

GLS_STATIC void __stdcall _glsDebugBreakFunction( void )
{
    DebugBreak();
}

GLS_STATIC BOOL _glsRead3dfxSplashEnv = FALSE;
#define _GLS_3DFX_SPLASH_STRING "FX_GLIDE_NO_SPLASH"
#define _GLS_3DFX_SPLASH_OFF "1"
GLS_STATIC char const *_gls3dfxSplashEnv = 0;

GLS_STATIC BOOL _glsReadSGIRegistryEntry = FALSE;
#define _GLS_SGI_REGISTRY_ROOT HKEY_LOCAL_MACHINE
#define _GLS_SGI_REGISTRY_KEY "Software\\Silicon Graphics\\OpenGL"
#define _GLS_SGI_REGISTRY_VALUE "OverrideDispatch"
#define _GLS_SGI_REGISTRY_DEFAULT 0
#define _GLS_SGI_REGISTRY_OFF 1
GLS_STATIC DWORD _glsSGIRegistryEntry = 0;

/* on error utility functions clear the values they're supposed to set */

GLS_STATIC BOOL _glsGetDriverFileDate( char const *pDriverFilePath,
                                       gls_driver_file_info *pDriverFileInfo );

GLS_STATIC BOOL _glsGetDriverFileVersion( char const *pDriverFilePath,
                                          gls_driver_file_info *pDriverFileInfo,
                                          char *pDescription,
                                          int unsigned DescriptionLength );

GLS_STATIC BOOL _glsCopyPathCheckSystemDirectory(
    char const *pDriverFilePath, gls_driver_file_info *pDriverFileInfo,
        BOOL *pIsInSystemDir );

GLS_STATIC char const *_glsGetVersionString( BYTE const *pVersionInfo,
                                             char const *pVersionString,
                                             UINT *pStringSize );

GLS_STATIC BOOL _glsIsDefaultOpenGL( char const *pGLDriverFilePath );
GLS_STATIC BOOL _glsCompareDirectories( char const *pA, char const *pB );

GLS_STATIC char const *_glsGetEnv( char const *pEnv );
GLS_STATIC void _glsSetEnv( char const *pEnv, char const *pValue );
GLS_STATIC DWORD _glsGetRegistryDWORD( HKEY Root, char const *pKey,
                                       char const *pValue, DWORD Default );
GLS_STATIC void _glsSetRegistryDWORD( HKEY Root, char const *pKey,
                                      char const *pValue, DWORD Value );

/* this function will copy a string and null-terminate the dest safely */
GLS_STATIC void _safe_strncpy( char *pDest, char const *pSource,
                               int unsigned Length )
{
    while(Length && ((*pDest++ = *pSource++) != 0))
    {
        Length--;
    }
    if(!Length)
    {
        *(pDest-1) = 0;
    }
}

/*----------------------------------------------------------------------------

gls[Get|Set]Behavior - These functions change the default behavior of the
    gls library.

*/

DWORD glsGetBehavior( void )
{
    return _glsBehavior;
}

extern DWORD glsSetBehavior( DWORD NewBehavior )
{
    DWORD OldBehavior = _glsBehavior;
    _glsBehavior = NewBehavior;

    if(NewBehavior & GLS_BEHAVIOR_INVALID_FUNCTIONS_DEBUGBREAK)
    {
        glsUnloadDriver();  /* initialize the invalid functions */
    }
    return OldBehavior;
}

/*----------------------------------------------------------------------------

glsGetNumberOfDrivers - This function figures out how many installed OpenGL
    drivers are present in the system.

@todo this hoses if there was ever a 3dfxvgl.dll in the system...

*/

int unsigned glsGetNumberOfDrivers( void )
{
    char aGLPath[MAX_PATH];
    char aGLUPath[MAX_PATH];
    static struct local_driver_info
    {
        DWORD AdditionalFlags;
        char aGLDriverFileName[MAX_PATH];
        char aGLUDriverFileName[MAX_PATH];
        char aDriverDescription[sizeof(_glsDriverInfos[0].aDriverDescription)];
    } aDrivers[] =
      {
          { GLS_FLAGS_DEFAULT_OPENGL_DRIVER,    /* this is set redundantly */
            "\\opengl32.dll",
            "\\glu32.dll",
            "Default OpenGL Driver" },
          { GLS_FLAGS_FULLSCREEN_ONLY | GLS_FLAGS_STANDALONE_DRIVER,
            "\\3dfxvgl.dll",
            "\\3dfxvglu.dll",
            "3dfx Standalone OpenGL Driver" },
          { GLS_FLAGS_SOFTWARE_ONLY,
            "\\opengl.dll",
            "\\glu.dll",
            "SGI Software OpenGL Driver" }
      };
#define NUMBER_OF_DRIVERS (sizeof(aDrivers)/sizeof(aDrivers[0]))
#define NUMBER_OF_DRIVERINFOS (sizeof(_glsDriverInfos)/sizeof(_glsDriverInfos[0]))

    _glsNumberOfDriverInfos = 0;

    if(GetSystemDirectory(aGLPath,sizeof(aGLPath)))
    {
        int GLFileLength, GLUFileLength;
        int const SystemLength = strlen(aGLPath);
        int unsigned DriverIndex;
        _safe_strncpy(aGLUPath,aGLPath,sizeof(aGLUPath));

        for(DriverIndex = 0;
            (DriverIndex < NUMBER_OF_DRIVERS)
                && (_glsNumberOfDriverInfos < NUMBER_OF_DRIVERINFOS);
            DriverIndex++)
        {
            GLFileLength = strlen(aDrivers[DriverIndex].aGLDriverFileName);
            GLUFileLength = strlen(aDrivers[DriverIndex].aGLUDriverFileName);

            /* this requires GLUs to be listed in the struct above */

            if((sizeof(aGLPath) > (GLFileLength + SystemLength))
               && (sizeof(aGLUPath) > (GLUFileLength + SystemLength)))
            {
                _safe_strncpy(aGLPath+SystemLength,
                              aDrivers[DriverIndex].aGLDriverFileName,
                              sizeof(aGLPath) - SystemLength);
                _safe_strncpy(aGLUPath+SystemLength,
                              aDrivers[DriverIndex].aGLUDriverFileName,
                              sizeof(aGLUPath) - SystemLength);

                if(GLS_ERROR_OK == glsGetDriverInfoFromFilePath(
                    aGLPath,aGLUPath,&_glsDriverInfos[_glsNumberOfDriverInfos]))
                {
                    _glsDriverInfos[_glsNumberOfDriverInfos].DriverFlags &=
                        ~GLS_FLAGS_DRIVER_UNKNOWN_FEATURES;
                    _glsDriverInfos[_glsNumberOfDriverInfos].DriverFlags |=
                        aDrivers[DriverIndex].AdditionalFlags;

                    assert(sizeof(aDrivers[DriverIndex].aDriverDescription) ==
                           sizeof(_glsDriverInfos[0].aDriverDescription));

                    if(aDrivers[DriverIndex].aDriverDescription[0])
                    {
                        _safe_strncpy(
                            _glsDriverInfos[_glsNumberOfDriverInfos].
                                aDriverDescription,
                                aDrivers[DriverIndex].aDriverDescription,
                                sizeof(_glsDriverInfos[0].aDriverDescription));
                    }
                    
                    _glsNumberOfDriverInfos++;
                }
            }
        }
    }                    
#undef NUMBER_OF_DRIVERS
#undef NUMBER_OF_DRIVERINFOS

    return _glsNumberOfDriverInfos;        
}

/*----------------------------------------------------------------------------

glsIsDriverLoaded - This function returns TRUE if a driver is loaded,
    FALSE otherwise.

*/

BOOL glsIsDriverLoaded( void )
{
    return _glsGLDriverHandle != 0;
}

/*----------------------------------------------------------------------------

glsGetDriverInfo - This function returns the gls_driver_info for the indexed
    OpenGL driver.

*/

gls_error glsGetDriverInfo( int unsigned DriverInfoIndex,
                            gls_driver_info *pDriverInfo )
{
    gls_error ErrorCode = GLS_ERROR_INVALID_PARAMETERS;

    /* if we haven't been initialized, do it now */
    if(!_glsNumberOfDriverInfos)
    {
        glsGetNumberOfDrivers();
    }
    
    if(pDriverInfo && (DriverInfoIndex < _glsNumberOfDriverInfos))
    {
        ErrorCode = GLS_ERROR_OK;
        *pDriverInfo = _glsDriverInfos[DriverInfoIndex];
    }

    return ErrorCode;
}                         
    
/*----------------------------------------------------------------------------

glsGetCurrentDriverInfo - This function returns the gls_driver_info for the
    currently loaded OpenGL driver.
    
*/

gls_error glsGetCurrentDriverInfo( gls_driver_info *pDriverInfo )
{
    gls_error ErrorCode = GLS_ERROR_NO_DRIVER_LOADED;

    if(pDriverInfo && _glsGLDriverHandle)
    {
        if(_glsLoadedDriverIndex == GLS_INVALID_DRIVER_INDEX)
        {
            /* we were loaded from a file path */
            assert(_glsGLDriverFilePath[0]);
            ErrorCode = glsGetDriverInfoFromFilePath(_glsGLDriverFilePath,
                                                     _glsGLUDriverFilePath,
                                                     pDriverInfo);
        }
        else
        {
            /* we were loaded from an index */
            assert(_glsLoadedDriverIndex < _glsNumberOfDriverInfos);
            ErrorCode = glsGetDriverInfo(_glsLoadedDriverIndex,pDriverInfo);
        }
    }

    return ErrorCode;
}

/*----------------------------------------------------------------------------

glsGetCurrentDriverFlags - This function returns the driver flags from the
    currently loaded driver.

*/

DWORD glsGetCurrentDriverFlags( void )
{
    DWORD Flags = (DWORD)(GLS_FLAGS_DRIVER_UNKNOWN_FEATURES
                          | GLS_FLAGS_NO_GL_VERSION_INFO
                          | GLS_FLAGS_NO_GLU
                          | GLS_FLAGS_NO_GLU_VERSION_INFO);
    
    gls_driver_info DriverInfo;

    if(glsGetCurrentDriverInfo(&DriverInfo) == GLS_ERROR_OK)
    {
        Flags = DriverInfo.DriverFlags;
    }
    return Flags;
}

/*----------------------------------------------------------------------------

glsGetDriverInfoFromFilePath - Get the gls_driver_info using a path to the
    driver.

@todo need more error reporting here

*/

gls_error glsGetDriverInfoFromFilePath( char const *pGLDriverFilePath,
                                        char const *pGLUDriverFilePath,
                                        gls_driver_info *pDriverInfo )
{
    gls_error ErrorCode = GLS_ERROR_INVALID_PARAMETERS;

    /* check parms and get the file date */
    if(pGLDriverFilePath && pDriverInfo
       && _glsGetDriverFileDate(pGLDriverFilePath,&pDriverInfo->GLDriver))
    {
        BOOL GLUSuccessful = FALSE;
        BOOL GLUIsInSystemDir = FALSE;
        BOOL GLIsInSystemDir = FALSE;
        
        /* clear the flags out...cast to avoid warning on W4 */
        pDriverInfo->DriverFlags = (DWORD)GLS_FLAGS_DRIVER_UNKNOWN_FEATURES;

        /* take care of the GLU first and get it out of the way */

        if(pGLUDriverFilePath && pGLUDriverFilePath[0] &&
           !(_glsBehavior & GLS_BEHAVIOR_NEVER_LOAD_GLU))
        {
            if(_glsGetDriverFileDate(pGLUDriverFilePath,
                                     &pDriverInfo->GLUDriver))
            {
                if(!_glsGetDriverFileVersion(pGLUDriverFilePath,
                                             &pDriverInfo->GLUDriver,0,0))
                {
                    pDriverInfo->DriverFlags |= GLS_FLAGS_NO_GLU_VERSION_INFO;
                }

                if(_glsCopyPathCheckSystemDirectory(pGLUDriverFilePath,
                                                    &pDriverInfo->GLUDriver,
                                                    &GLUIsInSystemDir))
                {
                    GLUSuccessful = TRUE;
                }
            }
        }

        if(!GLUSuccessful)
        {
            /* either there was no GLU requested, or we couldn't find it */
            GLUSuccessful = TRUE;
            pDriverInfo->GLUDriver.aDriverFilePath[0] = 0;
            pDriverInfo->DriverFlags |=
                (GLS_FLAGS_NO_GLU | GLS_FLAGS_NO_GLU_VERSION_INFO);
        }

        /* do the GL driver now */

        /* get the version number and description */
        if(!_glsGetDriverFileVersion(pGLDriverFilePath,
                                     &pDriverInfo->GLDriver,
                                     pDriverInfo->aDriverDescription,
                                     sizeof(pDriverInfo->aDriverDescription)))
        {
            char const *pUnknownDescription = "Unknown";
            assert(sizeof(pDriverInfo->aDriverDescription) >
                   strlen(pUnknownDescription));

            _safe_strncpy(pDriverInfo->aDriverDescription,pUnknownDescription,
                          sizeof(pDriverInfo->aDriverDescription));

            pDriverInfo->DriverFlags |= GLS_FLAGS_NO_GL_VERSION_INFO;
        }

        /* get the file path and the systemdir flag */
        if(_glsCopyPathCheckSystemDirectory(pGLDriverFilePath,
                                            &pDriverInfo->GLDriver,
                                            &GLIsInSystemDir))
        {
            if(GLIsInSystemDir)
            {
                pDriverInfo->DriverFlags |= GLS_FLAGS_DRIVER_IN_SYSTEM_DIR;
            }
            if(_glsIsDefaultOpenGL(pDriverInfo->GLDriver.aDriverFilePath))
            {
                pDriverInfo->DriverFlags |= GLS_FLAGS_DEFAULT_OPENGL_DRIVER;
            }

            /* now check if GLU and GL are in the same directory and if not
               disable the GLU */

            if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_GLU_IN_DIFFERENT_DIR)
               && GLUSuccessful
               && !_glsCompareDirectories(pDriverInfo->GLDriver.aDriverFilePath,
                                          pDriverInfo->GLUDriver.aDriverFilePath))
            {
                /* okay, they're in different directories, fail GLU */
                pDriverInfo->GLUDriver.aDriverFilePath[0] = 0;
                pDriverInfo->DriverFlags |=
                    (GLS_FLAGS_NO_GLU | GLS_FLAGS_NO_GLU_VERSION_INFO);
            }
            
            ErrorCode = GLS_ERROR_OK;
        }
    }
    return ErrorCode;
}

/*----------------------------------------------------------------------------

glsLoadDriver - This function loads the indexed OpenGL driver and dynalinks
    the function pointers.

*/

gls_error glsLoadDriver( int unsigned DriverInfoIndex )
{
    gls_error ErrorCode = GLS_ERROR_INVALID_PARAMETERS;

    /* if we haven't been initialized, do it now */
    if(!_glsNumberOfDriverInfos)
    {
        glsGetNumberOfDrivers();
    }
    
    if(DriverInfoIndex < _glsNumberOfDriverInfos)
    {
        ErrorCode = glsLoadDriverFromFilePath(
            _glsDriverInfos[DriverInfoIndex].GLDriver.aDriverFilePath,
                _glsDriverInfos[DriverInfoIndex].GLUDriver.aDriverFilePath);

        if(ErrorCode == GLS_ERROR_OK)
        {
            _glsLoadedDriverIndex = DriverInfoIndex;
        }
    }

    return ErrorCode;
}                         
    

/*----------------------------------------------------------------------------

glsLoadDriverFromFilePath - This function loads the named OpenGL driver
    and dynalinks the function pointers.

*/

gls_error glsLoadDriverFromFilePath( char const *pGLDriverFilePath,
                                     char const *pGLUDriverFilePath )
{
    gls_error ErrorCode = GLS_ERROR_DRIVER_LOAD_FAILED;

    /* this has to come first because it will undo everything below */
    glsUnloadDriver();

    if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_GRATUITOUS_HYPE_SCREENS))
    {
        /* get rid of the annoying 3dfx hype splash screen */
        if(!_glsRead3dfxSplashEnv)
        {
            _gls3dfxSplashEnv = _glsGetEnv(_GLS_3DFX_SPLASH_STRING);
            _glsRead3dfxSplashEnv = TRUE;
        }
        _glsSetEnv(_GLS_3DFX_SPLASH_STRING,_GLS_3DFX_SPLASH_OFF);
    }

    if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_SOFTWARE_REVECTORING))
    {
        /* don't let the SGI opengl vector to opengl32.dll */
        if(!_glsReadSGIRegistryEntry)
        {
            _glsSGIRegistryEntry = _glsGetRegistryDWORD(_GLS_SGI_REGISTRY_ROOT,
                                                        _GLS_SGI_REGISTRY_KEY,
                                                        _GLS_SGI_REGISTRY_VALUE,
                                                        _GLS_SGI_REGISTRY_DEFAULT);
            
            _glsReadSGIRegistryEntry = TRUE;
        }
        _glsSetRegistryDWORD(_GLS_SGI_REGISTRY_ROOT,_GLS_SGI_REGISTRY_KEY,
                             _GLS_SGI_REGISTRY_VALUE,_GLS_SGI_REGISTRY_OFF);
    }

    if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_GLU_IN_DIFFERENT_DIR)
       && pGLUDriverFilePath && pGLUDriverFilePath[0]
       && !_glsCompareDirectories(pGLDriverFilePath,pGLUDriverFilePath))
    {
        /* the GL and GLU are in different directories, so fail */
        pGLDriverFilePath = pGLUDriverFilePath = 0;
    }

    if(pGLDriverFilePath
       && ((_glsGLDriverHandle = LoadLibrary(pGLDriverFilePath)) != 0))
    {
        /* the DYNALINK_*_FUNCTION macros will OR into this to signal error */
        int DynalinkFailed = 0;

        /* macros for dynalinking */
#define _ERROR_WRAP( func ) ((func) != 0) ? 0 : (DynalinkFailed |= 1)
#define _STRINGIZE_NAME(name) #name
#define _STRINGIZE_NAME0(name) _STRINGIZE_NAME(name)
#define _WGLIZE_STRINGIZE_NAME( name ) _STRINGIZE_NAME0(wgl##name)
#define DYNALINK_GL_FUNCTION( name ) _ERROR_WRAP((*(int (__stdcall **)())&name) = GetProcAddress(_glsGLDriverHandle,#name))
#define DYNALINK_GLU_FUNCTION( name ) _ERROR_WRAP((*(int (__stdcall **)())&name) = GetProcAddress(_glsGLUDriverHandle,#name))
#define DYNALINK_GLS_FUNCTION( name ) _ERROR_WRAP((*(int (__stdcall **)())&GLS_SCOPE_NAME(name)) = GetProcAddress(_glsGLDriverHandle,_WGLIZE_STRINGIZE_NAME(name)))
#define DYNALINK_GDI_FUNCTION( name ) _ERROR_WRAP((*(int (__stdcall **)())&GLS_SCOPE_NAME(name)) = GetProcAddress(_glsGDI32,#name))
            
        DYNALINK_GL_FUNCTION( glAccum );
        DYNALINK_GL_FUNCTION( glAlphaFunc );
        DYNALINK_GL_FUNCTION( glAreTexturesResident );
        DYNALINK_GL_FUNCTION( glArrayElement );
        DYNALINK_GL_FUNCTION( glBegin );
        DYNALINK_GL_FUNCTION( glBindTexture );
        DYNALINK_GL_FUNCTION( glBitmap );
        DYNALINK_GL_FUNCTION( glBlendFunc );
        DYNALINK_GL_FUNCTION( glCallList );
        DYNALINK_GL_FUNCTION( glCallLists );
        DYNALINK_GL_FUNCTION( glClear );
        DYNALINK_GL_FUNCTION( glClearAccum );
        DYNALINK_GL_FUNCTION( glClearColor );
        DYNALINK_GL_FUNCTION( glClearDepth );
        DYNALINK_GL_FUNCTION( glClearIndex );
        DYNALINK_GL_FUNCTION( glClearStencil );
        DYNALINK_GL_FUNCTION( glClipPlane );
        DYNALINK_GL_FUNCTION( glColor3b );
        DYNALINK_GL_FUNCTION( glColor3bv );
        DYNALINK_GL_FUNCTION( glColor3d );
        DYNALINK_GL_FUNCTION( glColor3dv );
        DYNALINK_GL_FUNCTION( glColor3f );
        DYNALINK_GL_FUNCTION( glColor3fv );
        DYNALINK_GL_FUNCTION( glColor3i );
        DYNALINK_GL_FUNCTION( glColor3iv );
        DYNALINK_GL_FUNCTION( glColor3s );
        DYNALINK_GL_FUNCTION( glColor3sv );
        DYNALINK_GL_FUNCTION( glColor3ub );
        DYNALINK_GL_FUNCTION( glColor3ubv );
        DYNALINK_GL_FUNCTION( glColor3ui );
        DYNALINK_GL_FUNCTION( glColor3uiv );
        DYNALINK_GL_FUNCTION( glColor3us );
        DYNALINK_GL_FUNCTION( glColor3usv );
        DYNALINK_GL_FUNCTION( glColor4b );
        DYNALINK_GL_FUNCTION( glColor4bv );
        DYNALINK_GL_FUNCTION( glColor4d );
        DYNALINK_GL_FUNCTION( glColor4dv );
        DYNALINK_GL_FUNCTION( glColor4f );
        DYNALINK_GL_FUNCTION( glColor4fv );
        DYNALINK_GL_FUNCTION( glColor4i );
        DYNALINK_GL_FUNCTION( glColor4iv );
        DYNALINK_GL_FUNCTION( glColor4s );
        DYNALINK_GL_FUNCTION( glColor4sv );
        DYNALINK_GL_FUNCTION( glColor4ub );
        DYNALINK_GL_FUNCTION( glColor4ubv );
        DYNALINK_GL_FUNCTION( glColor4ui );
        DYNALINK_GL_FUNCTION( glColor4uiv );
        DYNALINK_GL_FUNCTION( glColor4us );
        DYNALINK_GL_FUNCTION( glColor4usv );
        DYNALINK_GL_FUNCTION( glColorMask );
        DYNALINK_GL_FUNCTION( glColorMaterial );
        DYNALINK_GL_FUNCTION( glColorPointer );
        DYNALINK_GL_FUNCTION( glCopyPixels );
        DYNALINK_GL_FUNCTION( glCopyTexImage1D );
        DYNALINK_GL_FUNCTION( glCopyTexImage2D );
        DYNALINK_GL_FUNCTION( glCopyTexSubImage1D );
        DYNALINK_GL_FUNCTION( glCopyTexSubImage2D );
        DYNALINK_GL_FUNCTION( glCullFace );
        DYNALINK_GL_FUNCTION( glDeleteLists );
        DYNALINK_GL_FUNCTION( glDeleteTextures );
        DYNALINK_GL_FUNCTION( glDepthFunc );
        DYNALINK_GL_FUNCTION( glDepthMask );
        DYNALINK_GL_FUNCTION( glDepthRange );
        DYNALINK_GL_FUNCTION( glDisable );
        DYNALINK_GL_FUNCTION( glDisableClientState );
        DYNALINK_GL_FUNCTION( glDrawArrays );
        DYNALINK_GL_FUNCTION( glDrawBuffer );
        DYNALINK_GL_FUNCTION( glDrawElements );
        DYNALINK_GL_FUNCTION( glDrawPixels );
        DYNALINK_GL_FUNCTION( glEdgeFlag );
        DYNALINK_GL_FUNCTION( glEdgeFlagPointer );
        DYNALINK_GL_FUNCTION( glEdgeFlagv );
        DYNALINK_GL_FUNCTION( glEnable );
        DYNALINK_GL_FUNCTION( glEnableClientState );
        DYNALINK_GL_FUNCTION( glEnd );
        DYNALINK_GL_FUNCTION( glEndList );
        DYNALINK_GL_FUNCTION( glEvalCoord1d );
        DYNALINK_GL_FUNCTION( glEvalCoord1dv );
        DYNALINK_GL_FUNCTION( glEvalCoord1f );
        DYNALINK_GL_FUNCTION( glEvalCoord1fv );
        DYNALINK_GL_FUNCTION( glEvalCoord2d );
        DYNALINK_GL_FUNCTION( glEvalCoord2dv );
        DYNALINK_GL_FUNCTION( glEvalCoord2f );
        DYNALINK_GL_FUNCTION( glEvalCoord2fv );
        DYNALINK_GL_FUNCTION( glEvalMesh1 );
        DYNALINK_GL_FUNCTION( glEvalMesh2 );
        DYNALINK_GL_FUNCTION( glEvalPoint1 );
        DYNALINK_GL_FUNCTION( glEvalPoint2 );
        DYNALINK_GL_FUNCTION( glFeedbackBuffer );
        DYNALINK_GL_FUNCTION( glFinish );
        DYNALINK_GL_FUNCTION( glFlush );
        DYNALINK_GL_FUNCTION( glFogf );
        DYNALINK_GL_FUNCTION( glFogfv );
        DYNALINK_GL_FUNCTION( glFogi );
        DYNALINK_GL_FUNCTION( glFogiv );
        DYNALINK_GL_FUNCTION( glFrontFace );
        DYNALINK_GL_FUNCTION( glFrustum );
        DYNALINK_GL_FUNCTION( glGenLists );
        DYNALINK_GL_FUNCTION( glGenTextures );
        DYNALINK_GL_FUNCTION( glGetBooleanv );
        DYNALINK_GL_FUNCTION( glGetClipPlane );
        DYNALINK_GL_FUNCTION( glGetDoublev );
        DYNALINK_GL_FUNCTION( glGetError );
        DYNALINK_GL_FUNCTION( glGetFloatv );
        DYNALINK_GL_FUNCTION( glGetIntegerv );
        DYNALINK_GL_FUNCTION( glGetLightfv );
        DYNALINK_GL_FUNCTION( glGetLightiv );
        DYNALINK_GL_FUNCTION( glGetMapdv );
        DYNALINK_GL_FUNCTION( glGetMapfv );
        DYNALINK_GL_FUNCTION( glGetMapiv );
        DYNALINK_GL_FUNCTION( glGetMaterialfv );
        DYNALINK_GL_FUNCTION( glGetMaterialiv );
        DYNALINK_GL_FUNCTION( glGetPixelMapfv );
        DYNALINK_GL_FUNCTION( glGetPixelMapuiv );
        DYNALINK_GL_FUNCTION( glGetPixelMapusv );
        DYNALINK_GL_FUNCTION( glGetPointerv );
        DYNALINK_GL_FUNCTION( glGetPolygonStipple );
        DYNALINK_GL_FUNCTION( glGetString );
        DYNALINK_GL_FUNCTION( glGetTexEnvfv );
        DYNALINK_GL_FUNCTION( glGetTexEnviv );
        DYNALINK_GL_FUNCTION( glGetTexGendv );
        DYNALINK_GL_FUNCTION( glGetTexGenfv );
        DYNALINK_GL_FUNCTION( glGetTexGeniv );
        DYNALINK_GL_FUNCTION( glGetTexImage );
        DYNALINK_GL_FUNCTION( glGetTexLevelParameterfv );
        DYNALINK_GL_FUNCTION( glGetTexLevelParameteriv );
        DYNALINK_GL_FUNCTION( glGetTexParameterfv );
        DYNALINK_GL_FUNCTION( glGetTexParameteriv );
        DYNALINK_GL_FUNCTION( glHint );
        DYNALINK_GL_FUNCTION( glIndexMask );
        DYNALINK_GL_FUNCTION( glIndexPointer );
        DYNALINK_GL_FUNCTION( glIndexd );
        DYNALINK_GL_FUNCTION( glIndexdv );
        DYNALINK_GL_FUNCTION( glIndexf );
        DYNALINK_GL_FUNCTION( glIndexfv );
        DYNALINK_GL_FUNCTION( glIndexi );
        DYNALINK_GL_FUNCTION( glIndexiv );
        DYNALINK_GL_FUNCTION( glIndexs );
        DYNALINK_GL_FUNCTION( glIndexsv );
        DYNALINK_GL_FUNCTION( glIndexub );
        DYNALINK_GL_FUNCTION( glIndexubv );
        DYNALINK_GL_FUNCTION( glInitNames );
        DYNALINK_GL_FUNCTION( glInterleavedArrays );
        DYNALINK_GL_FUNCTION( glIsEnabled );
        DYNALINK_GL_FUNCTION( glIsList );
        DYNALINK_GL_FUNCTION( glIsTexture );
        DYNALINK_GL_FUNCTION( glLightModelf );
        DYNALINK_GL_FUNCTION( glLightModelfv );
        DYNALINK_GL_FUNCTION( glLightModeli );
        DYNALINK_GL_FUNCTION( glLightModeliv );
        DYNALINK_GL_FUNCTION( glLightf );
        DYNALINK_GL_FUNCTION( glLightfv );
        DYNALINK_GL_FUNCTION( glLighti );
        DYNALINK_GL_FUNCTION( glLightiv );
        DYNALINK_GL_FUNCTION( glLineStipple );
        DYNALINK_GL_FUNCTION( glLineWidth );
        DYNALINK_GL_FUNCTION( glListBase );
        DYNALINK_GL_FUNCTION( glLoadIdentity );
        DYNALINK_GL_FUNCTION( glLoadMatrixd );
        DYNALINK_GL_FUNCTION( glLoadMatrixf );
        DYNALINK_GL_FUNCTION( glLoadName );
        DYNALINK_GL_FUNCTION( glLogicOp );
        DYNALINK_GL_FUNCTION( glMap1d );
        DYNALINK_GL_FUNCTION( glMap1f );
        DYNALINK_GL_FUNCTION( glMap2d );
        DYNALINK_GL_FUNCTION( glMap2f );
        DYNALINK_GL_FUNCTION( glMapGrid1d );
        DYNALINK_GL_FUNCTION( glMapGrid1f );
        DYNALINK_GL_FUNCTION( glMapGrid2d );
        DYNALINK_GL_FUNCTION( glMapGrid2f );
        DYNALINK_GL_FUNCTION( glMaterialf );
        DYNALINK_GL_FUNCTION( glMaterialfv );
        DYNALINK_GL_FUNCTION( glMateriali );
        DYNALINK_GL_FUNCTION( glMaterialiv );
        DYNALINK_GL_FUNCTION( glMatrixMode );
        DYNALINK_GL_FUNCTION( glMultMatrixd );
        DYNALINK_GL_FUNCTION( glMultMatrixf );
        DYNALINK_GL_FUNCTION( glNewList );
        DYNALINK_GL_FUNCTION( glNormal3b );
        DYNALINK_GL_FUNCTION( glNormal3bv );
        DYNALINK_GL_FUNCTION( glNormal3d );
        DYNALINK_GL_FUNCTION( glNormal3dv );
        DYNALINK_GL_FUNCTION( glNormal3f );
        DYNALINK_GL_FUNCTION( glNormal3fv );
        DYNALINK_GL_FUNCTION( glNormal3i );
        DYNALINK_GL_FUNCTION( glNormal3iv );
        DYNALINK_GL_FUNCTION( glNormal3s );
        DYNALINK_GL_FUNCTION( glNormal3sv );
        DYNALINK_GL_FUNCTION( glNormalPointer );
        DYNALINK_GL_FUNCTION( glOrtho );
        DYNALINK_GL_FUNCTION( glPassThrough );
        DYNALINK_GL_FUNCTION( glPixelMapfv );
        DYNALINK_GL_FUNCTION( glPixelMapuiv );
        DYNALINK_GL_FUNCTION( glPixelMapusv );
        DYNALINK_GL_FUNCTION( glPixelStoref );
        DYNALINK_GL_FUNCTION( glPixelStorei );
        DYNALINK_GL_FUNCTION( glPixelTransferf );
        DYNALINK_GL_FUNCTION( glPixelTransferi );
        DYNALINK_GL_FUNCTION( glPixelZoom );
        DYNALINK_GL_FUNCTION( glPointSize );
        DYNALINK_GL_FUNCTION( glPolygonMode );
        DYNALINK_GL_FUNCTION( glPolygonOffset );
        DYNALINK_GL_FUNCTION( glPolygonStipple );
        DYNALINK_GL_FUNCTION( glPopAttrib );
        DYNALINK_GL_FUNCTION( glPopClientAttrib );
        DYNALINK_GL_FUNCTION( glPopMatrix );
        DYNALINK_GL_FUNCTION( glPopName );
        DYNALINK_GL_FUNCTION( glPrioritizeTextures );
        DYNALINK_GL_FUNCTION( glPushAttrib );
        DYNALINK_GL_FUNCTION( glPushClientAttrib );
        DYNALINK_GL_FUNCTION( glPushMatrix );
        DYNALINK_GL_FUNCTION( glPushName );
        DYNALINK_GL_FUNCTION( glRasterPos2d );
        DYNALINK_GL_FUNCTION( glRasterPos2dv );
        DYNALINK_GL_FUNCTION( glRasterPos2f );
        DYNALINK_GL_FUNCTION( glRasterPos2fv );
        DYNALINK_GL_FUNCTION( glRasterPos2i );
        DYNALINK_GL_FUNCTION( glRasterPos2iv );
        DYNALINK_GL_FUNCTION( glRasterPos2s );
        DYNALINK_GL_FUNCTION( glRasterPos2sv );
        DYNALINK_GL_FUNCTION( glRasterPos3d );
        DYNALINK_GL_FUNCTION( glRasterPos3dv );
        DYNALINK_GL_FUNCTION( glRasterPos3f );
        DYNALINK_GL_FUNCTION( glRasterPos3fv );
        DYNALINK_GL_FUNCTION( glRasterPos3i );
        DYNALINK_GL_FUNCTION( glRasterPos3iv );
        DYNALINK_GL_FUNCTION( glRasterPos3s );
        DYNALINK_GL_FUNCTION( glRasterPos3sv );
        DYNALINK_GL_FUNCTION( glRasterPos4d );
        DYNALINK_GL_FUNCTION( glRasterPos4dv );
        DYNALINK_GL_FUNCTION( glRasterPos4f );
        DYNALINK_GL_FUNCTION( glRasterPos4fv );
        DYNALINK_GL_FUNCTION( glRasterPos4i );
        DYNALINK_GL_FUNCTION( glRasterPos4iv );
        DYNALINK_GL_FUNCTION( glRasterPos4s );
        DYNALINK_GL_FUNCTION( glRasterPos4sv );
        DYNALINK_GL_FUNCTION( glReadBuffer );
        DYNALINK_GL_FUNCTION( glReadPixels );
        DYNALINK_GL_FUNCTION( glRectd );
        DYNALINK_GL_FUNCTION( glRectdv );
        DYNALINK_GL_FUNCTION( glRectf );
        DYNALINK_GL_FUNCTION( glRectfv );
        DYNALINK_GL_FUNCTION( glRecti );
        DYNALINK_GL_FUNCTION( glRectiv );
        DYNALINK_GL_FUNCTION( glRects );
        DYNALINK_GL_FUNCTION( glRectsv );
        DYNALINK_GL_FUNCTION( glRenderMode );
        DYNALINK_GL_FUNCTION( glRotated );
        DYNALINK_GL_FUNCTION( glRotatef );
        DYNALINK_GL_FUNCTION( glScaled );
        DYNALINK_GL_FUNCTION( glScalef );
        DYNALINK_GL_FUNCTION( glScissor );
        DYNALINK_GL_FUNCTION( glSelectBuffer );
        DYNALINK_GL_FUNCTION( glShadeModel );
        DYNALINK_GL_FUNCTION( glStencilFunc );
        DYNALINK_GL_FUNCTION( glStencilMask );
        DYNALINK_GL_FUNCTION( glStencilOp );
        DYNALINK_GL_FUNCTION( glTexCoord1d );
        DYNALINK_GL_FUNCTION( glTexCoord1dv );
        DYNALINK_GL_FUNCTION( glTexCoord1f );
        DYNALINK_GL_FUNCTION( glTexCoord1fv );
        DYNALINK_GL_FUNCTION( glTexCoord1i );
        DYNALINK_GL_FUNCTION( glTexCoord1iv );
        DYNALINK_GL_FUNCTION( glTexCoord1s );
        DYNALINK_GL_FUNCTION( glTexCoord1sv );
        DYNALINK_GL_FUNCTION( glTexCoord2d );
        DYNALINK_GL_FUNCTION( glTexCoord2dv );
        DYNALINK_GL_FUNCTION( glTexCoord2f );
        DYNALINK_GL_FUNCTION( glTexCoord2fv );
        DYNALINK_GL_FUNCTION( glTexCoord2i );
        DYNALINK_GL_FUNCTION( glTexCoord2iv );
        DYNALINK_GL_FUNCTION( glTexCoord2s );
        DYNALINK_GL_FUNCTION( glTexCoord2sv );
        DYNALINK_GL_FUNCTION( glTexCoord3d );
        DYNALINK_GL_FUNCTION( glTexCoord3dv );
        DYNALINK_GL_FUNCTION( glTexCoord3f );
        DYNALINK_GL_FUNCTION( glTexCoord3fv );
        DYNALINK_GL_FUNCTION( glTexCoord3i );
        DYNALINK_GL_FUNCTION( glTexCoord3iv );
        DYNALINK_GL_FUNCTION( glTexCoord3s );
        DYNALINK_GL_FUNCTION( glTexCoord3sv );
        DYNALINK_GL_FUNCTION( glTexCoord4d );
        DYNALINK_GL_FUNCTION( glTexCoord4dv );
        DYNALINK_GL_FUNCTION( glTexCoord4f );
        DYNALINK_GL_FUNCTION( glTexCoord4fv );
        DYNALINK_GL_FUNCTION( glTexCoord4i );
        DYNALINK_GL_FUNCTION( glTexCoord4iv );
        DYNALINK_GL_FUNCTION( glTexCoord4s );
        DYNALINK_GL_FUNCTION( glTexCoord4sv );
        DYNALINK_GL_FUNCTION( glTexCoordPointer );
        DYNALINK_GL_FUNCTION( glTexEnvf );
        DYNALINK_GL_FUNCTION( glTexEnvfv );
        DYNALINK_GL_FUNCTION( glTexEnvi );
        DYNALINK_GL_FUNCTION( glTexEnviv );
        DYNALINK_GL_FUNCTION( glTexGend );
        DYNALINK_GL_FUNCTION( glTexGendv );
        DYNALINK_GL_FUNCTION( glTexGenf );
        DYNALINK_GL_FUNCTION( glTexGenfv );
        DYNALINK_GL_FUNCTION( glTexGeni );
        DYNALINK_GL_FUNCTION( glTexGeniv );
        DYNALINK_GL_FUNCTION( glTexImage1D );
        DYNALINK_GL_FUNCTION( glTexImage2D );
        DYNALINK_GL_FUNCTION( glTexParameterf );
        DYNALINK_GL_FUNCTION( glTexParameterfv );
        DYNALINK_GL_FUNCTION( glTexParameteri );
        DYNALINK_GL_FUNCTION( glTexParameteriv );
        DYNALINK_GL_FUNCTION( glTexSubImage1D );
        DYNALINK_GL_FUNCTION( glTexSubImage2D );
        DYNALINK_GL_FUNCTION( glTranslated );
        DYNALINK_GL_FUNCTION( glTranslatef );
        DYNALINK_GL_FUNCTION( glVertex2d );
        DYNALINK_GL_FUNCTION( glVertex2dv );
        DYNALINK_GL_FUNCTION( glVertex2f );
        DYNALINK_GL_FUNCTION( glVertex2fv );
        DYNALINK_GL_FUNCTION( glVertex2i );
        DYNALINK_GL_FUNCTION( glVertex2iv );
        DYNALINK_GL_FUNCTION( glVertex2s );
        DYNALINK_GL_FUNCTION( glVertex2sv );
        DYNALINK_GL_FUNCTION( glVertex3d );
        DYNALINK_GL_FUNCTION( glVertex3dv );
        DYNALINK_GL_FUNCTION( glVertex3f );
        DYNALINK_GL_FUNCTION( glVertex3fv );
        DYNALINK_GL_FUNCTION( glVertex3i );
        DYNALINK_GL_FUNCTION( glVertex3iv );
        DYNALINK_GL_FUNCTION( glVertex3s );
        DYNALINK_GL_FUNCTION( glVertex3sv );
        DYNALINK_GL_FUNCTION( glVertex4d );
        DYNALINK_GL_FUNCTION( glVertex4dv );
        DYNALINK_GL_FUNCTION( glVertex4f );
        DYNALINK_GL_FUNCTION( glVertex4fv );
        DYNALINK_GL_FUNCTION( glVertex4i );
        DYNALINK_GL_FUNCTION( glVertex4iv );
        DYNALINK_GL_FUNCTION( glVertex4s );
        DYNALINK_GL_FUNCTION( glVertex4sv );
        DYNALINK_GL_FUNCTION( glVertexPointer );
        DYNALINK_GL_FUNCTION( glViewport );

        DYNALINK_GLS_FUNCTION( CopyContext );
        DYNALINK_GLS_FUNCTION( CreateContext );
        DYNALINK_GLS_FUNCTION( CreateLayerContext );
        DYNALINK_GLS_FUNCTION( DeleteContext );
        DYNALINK_GLS_FUNCTION( DescribeLayerPlane );
        DYNALINK_GLS_FUNCTION( GetCurrentContext );
        DYNALINK_GLS_FUNCTION( GetCurrentDC );
        DYNALINK_GLS_FUNCTION( GetLayerPaletteEntries );
        DYNALINK_GLS_FUNCTION( GetProcAddress );
        DYNALINK_GLS_FUNCTION( MakeCurrent );
        DYNALINK_GLS_FUNCTION( RealizeLayerPalette );
        DYNALINK_GLS_FUNCTION( SetLayerPaletteEntries );
        DYNALINK_GLS_FUNCTION( ShareLists );
        DYNALINK_GLS_FUNCTION( SwapLayerBuffers );
        DYNALINK_GLS_FUNCTION( UseFontBitmaps );
        DYNALINK_GLS_FUNCTION( UseFontOutlines );

        if(_glsIsDefaultOpenGL(pGLDriverFilePath))
        {
            /* if this is the default opengl32.dll, we need to
               dynalink from gdi32.dll for the following functions
               because Win2k does some magic before calling opengl32.dll */

            if((_glsGDI32 = LoadLibrary("gdi32.dll")) != 0)
            {
                DYNALINK_GDI_FUNCTION( ChoosePixelFormat );
                DYNALINK_GDI_FUNCTION( DescribePixelFormat );
                DYNALINK_GDI_FUNCTION( GetPixelFormat );
                DYNALINK_GDI_FUNCTION( SetPixelFormat );
                DYNALINK_GDI_FUNCTION( SwapBuffers );
            }
            else
            {
                DynalinkFailed = 1;
            }
        }
        else
        {
            DYNALINK_GLS_FUNCTION( ChoosePixelFormat );
            DYNALINK_GLS_FUNCTION( DescribePixelFormat );
            DYNALINK_GLS_FUNCTION( GetPixelFormat );
            DYNALINK_GLS_FUNCTION( SetPixelFormat );
            DYNALINK_GLS_FUNCTION( SwapBuffers );
        }

        /* copy the loaded file path into the global variable */
        _safe_strncpy(_glsGLDriverFilePath,pGLDriverFilePath,
                      sizeof(_glsGLDriverFilePath));

#ifndef GLS_NO_GLU

        /* now do GLU if the user requested it */

        if(!(_glsBehavior & GLS_BEHAVIOR_NEVER_LOAD_GLU)
           && pGLUDriverFilePath && pGLUDriverFilePath[0])
        {
            _glsGLUDriverHandle = LoadLibrary(pGLUDriverFilePath);
            if(_glsGLUDriverHandle)
            {
                DYNALINK_GLU_FUNCTION( gluBeginCurve );
                DYNALINK_GLU_FUNCTION( gluBeginPolygon );
                DYNALINK_GLU_FUNCTION( gluBeginSurface );
                DYNALINK_GLU_FUNCTION( gluBeginTrim );
                DYNALINK_GLU_FUNCTION( gluBuild1DMipmaps );
                DYNALINK_GLU_FUNCTION( gluBuild2DMipmaps );
                DYNALINK_GLU_FUNCTION( gluCylinder );
                DYNALINK_GLU_FUNCTION( gluDeleteNurbsRenderer );
                DYNALINK_GLU_FUNCTION( gluDeleteQuadric );
                DYNALINK_GLU_FUNCTION( gluDeleteTess );
                DYNALINK_GLU_FUNCTION( gluDisk );
                DYNALINK_GLU_FUNCTION( gluEndCurve );
                DYNALINK_GLU_FUNCTION( gluEndPolygon );
                DYNALINK_GLU_FUNCTION( gluEndSurface );
                DYNALINK_GLU_FUNCTION( gluEndTrim );
                DYNALINK_GLU_FUNCTION( gluErrorString );
                DYNALINK_GLU_FUNCTION( gluGetNurbsProperty );
                DYNALINK_GLU_FUNCTION( gluGetString );
                DYNALINK_GLU_FUNCTION( gluGetTessProperty );
                DYNALINK_GLU_FUNCTION( gluLoadSamplingMatrices );
                DYNALINK_GLU_FUNCTION( gluLookAt );
                DYNALINK_GLU_FUNCTION( gluNewNurbsRenderer );
                DYNALINK_GLU_FUNCTION( gluNewQuadric );
                DYNALINK_GLU_FUNCTION( gluNewTess );
                DYNALINK_GLU_FUNCTION( gluNextContour );
                DYNALINK_GLU_FUNCTION( gluNurbsCallback );
                DYNALINK_GLU_FUNCTION( gluNurbsCurve );
                DYNALINK_GLU_FUNCTION( gluNurbsProperty );
                DYNALINK_GLU_FUNCTION( gluNurbsSurface );
                DYNALINK_GLU_FUNCTION( gluOrtho2D );
                DYNALINK_GLU_FUNCTION( gluPartialDisk );
                DYNALINK_GLU_FUNCTION( gluPerspective );
                DYNALINK_GLU_FUNCTION( gluPickMatrix );
                DYNALINK_GLU_FUNCTION( gluProject );
                DYNALINK_GLU_FUNCTION( gluPwlCurve );
                DYNALINK_GLU_FUNCTION( gluQuadricCallback );
                DYNALINK_GLU_FUNCTION( gluQuadricDrawStyle );
                DYNALINK_GLU_FUNCTION( gluQuadricNormals );
                DYNALINK_GLU_FUNCTION( gluQuadricOrientation );
                DYNALINK_GLU_FUNCTION( gluQuadricTexture );
                DYNALINK_GLU_FUNCTION( gluScaleImage );
                DYNALINK_GLU_FUNCTION( gluSphere );
                DYNALINK_GLU_FUNCTION( gluTessBeginContour );
                DYNALINK_GLU_FUNCTION( gluTessBeginPolygon );
                DYNALINK_GLU_FUNCTION( gluTessCallback );
                DYNALINK_GLU_FUNCTION( gluTessEndContour );
                DYNALINK_GLU_FUNCTION( gluTessEndPolygon );
                DYNALINK_GLU_FUNCTION( gluTessNormal );
                DYNALINK_GLU_FUNCTION( gluTessProperty );
                DYNALINK_GLU_FUNCTION( gluTessVertex );
                DYNALINK_GLU_FUNCTION( gluUnProject );

                /* copy the loaded file path into the global variable */
                _safe_strncpy(_glsGLUDriverFilePath,pGLUDriverFilePath,
                              sizeof(_glsGLUDriverFilePath));
            }
            else
            {
                DynalinkFailed = 1;
            }
        }
        else
        {
            /* no GLU requested */
            _glsGLUDriverFilePath[0] = 0;
        }
#endif  /* GLS_NO_GLU */
        
#undef _STRING_NAME
#undef _STRING_NAME0
#undef _WGLIZE_STRINGIZE_NAME
#undef _ERROR_WRAP
#undef DYNALINK_GDI_FUNCTION
#undef DYNALINK_GL_FUNCTION
#undef DYNALINK_GLU_FUNCTION
#undef DYNALINK_GLS_FUNCTION

        if(DynalinkFailed)
        {
            ErrorCode = GLS_ERROR_DRIVER_DYNALINK_FAILED;
            glsUnloadDriver();
        }
        else
        {
            ErrorCode = GLS_ERROR_OK;
        }
    }
    return ErrorCode;
}

/*----------------------------------------------------------------------------

glsUnloadDriver - closes down the binding library and cleans up.

*/

void glsUnloadDriver(void )
{
    void (__stdcall *pInvalidFunction)( void ) = 0;

    if (_glsGLDriverHandle)
    {
        FreeLibrary(_glsGLDriverHandle);
        _glsGLDriverHandle = 0;
    }
    if (_glsGLUDriverHandle)
    {
        FreeLibrary(_glsGLUDriverHandle);
        _glsGLUDriverHandle = 0;
    }
    if(_glsGDI32)
    {
        FreeLibrary(_glsGDI32);
        _glsGDI32 = 0;
    }
    
    /* restore environment and registry */
    if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_GRATUITOUS_HYPE_SCREENS))
    {
        if(_glsRead3dfxSplashEnv)
        {
            _glsSetEnv(_GLS_3DFX_SPLASH_STRING,_gls3dfxSplashEnv);
            _glsRead3dfxSplashEnv = FALSE;
        }
    }
    if(!(_glsBehavior & GLS_BEHAVIOR_ALLOW_SOFTWARE_REVECTORING))
    {
        if(_glsReadSGIRegistryEntry)
        {
            _glsSetRegistryDWORD(_GLS_SGI_REGISTRY_ROOT,_GLS_SGI_REGISTRY_KEY,
                                 _GLS_SGI_REGISTRY_VALUE,_glsSGIRegistryEntry);
            _glsReadSGIRegistryEntry = FALSE;
        }
    }

    if(_glsBehavior & GLS_BEHAVIOR_INVALID_FUNCTIONS_DEBUGBREAK)
    {
        pInvalidFunction = _glsDebugBreakFunction;
    }

#define DISABLE_GL_FUNCTION( name ) (*(void (__stdcall **)())&name) = pInvalidFunction;
#define DISABLE_GLU_FUNCTION( name ) (*(void (__stdcall **)())&name) = pInvalidFunction;
#define DISABLE_GLS_FUNCTION( name ) (*(void (__stdcall **)())&GLS_SCOPE_NAME(name)) = pInvalidFunction;

    DISABLE_GL_FUNCTION( glAccum );
    DISABLE_GL_FUNCTION( glAlphaFunc );
    DISABLE_GL_FUNCTION( glAreTexturesResident );
    DISABLE_GL_FUNCTION( glArrayElement );
    DISABLE_GL_FUNCTION( glBegin );
    DISABLE_GL_FUNCTION( glBindTexture );
    DISABLE_GL_FUNCTION( glBitmap );
    DISABLE_GL_FUNCTION( glBlendFunc );
    DISABLE_GL_FUNCTION( glCallList );
    DISABLE_GL_FUNCTION( glCallLists );
    DISABLE_GL_FUNCTION( glClear );
    DISABLE_GL_FUNCTION( glClearAccum );
    DISABLE_GL_FUNCTION( glClearColor );
    DISABLE_GL_FUNCTION( glClearDepth );
    DISABLE_GL_FUNCTION( glClearIndex );
    DISABLE_GL_FUNCTION( glClearStencil );
    DISABLE_GL_FUNCTION( glClipPlane );
    DISABLE_GL_FUNCTION( glColor3b );
    DISABLE_GL_FUNCTION( glColor3bv );
    DISABLE_GL_FUNCTION( glColor3d );
    DISABLE_GL_FUNCTION( glColor3dv );
    DISABLE_GL_FUNCTION( glColor3f );
    DISABLE_GL_FUNCTION( glColor3fv );
    DISABLE_GL_FUNCTION( glColor3i );
    DISABLE_GL_FUNCTION( glColor3iv );
    DISABLE_GL_FUNCTION( glColor3s );
    DISABLE_GL_FUNCTION( glColor3sv );
    DISABLE_GL_FUNCTION( glColor3ub );
    DISABLE_GL_FUNCTION( glColor3ubv );
    DISABLE_GL_FUNCTION( glColor3ui );
    DISABLE_GL_FUNCTION( glColor3uiv );
    DISABLE_GL_FUNCTION( glColor3us );
    DISABLE_GL_FUNCTION( glColor3usv );
    DISABLE_GL_FUNCTION( glColor4b );
    DISABLE_GL_FUNCTION( glColor4bv );
    DISABLE_GL_FUNCTION( glColor4d );
    DISABLE_GL_FUNCTION( glColor4dv );
    DISABLE_GL_FUNCTION( glColor4f );
    DISABLE_GL_FUNCTION( glColor4fv );
    DISABLE_GL_FUNCTION( glColor4i );
    DISABLE_GL_FUNCTION( glColor4iv );
    DISABLE_GL_FUNCTION( glColor4s );
    DISABLE_GL_FUNCTION( glColor4sv );
    DISABLE_GL_FUNCTION( glColor4ub );
    DISABLE_GL_FUNCTION( glColor4ubv );
    DISABLE_GL_FUNCTION( glColor4ui );
    DISABLE_GL_FUNCTION( glColor4uiv );
    DISABLE_GL_FUNCTION( glColor4us );
    DISABLE_GL_FUNCTION( glColor4usv );
    DISABLE_GL_FUNCTION( glColorMask );
    DISABLE_GL_FUNCTION( glColorMaterial );
    DISABLE_GL_FUNCTION( glColorPointer );
    DISABLE_GL_FUNCTION( glCopyPixels );
    DISABLE_GL_FUNCTION( glCopyTexImage1D );
    DISABLE_GL_FUNCTION( glCopyTexImage2D );
    DISABLE_GL_FUNCTION( glCopyTexSubImage1D );
    DISABLE_GL_FUNCTION( glCopyTexSubImage2D );
    DISABLE_GL_FUNCTION( glCullFace );
    DISABLE_GL_FUNCTION( glDeleteLists );
    DISABLE_GL_FUNCTION( glDeleteTextures );
    DISABLE_GL_FUNCTION( glDepthFunc );
    DISABLE_GL_FUNCTION( glDepthMask );
    DISABLE_GL_FUNCTION( glDepthRange );
    DISABLE_GL_FUNCTION( glDisable );
    DISABLE_GL_FUNCTION( glDisableClientState );
    DISABLE_GL_FUNCTION( glDrawArrays );
    DISABLE_GL_FUNCTION( glDrawBuffer );
    DISABLE_GL_FUNCTION( glDrawElements );
    DISABLE_GL_FUNCTION( glDrawPixels );
    DISABLE_GL_FUNCTION( glEdgeFlag );
    DISABLE_GL_FUNCTION( glEdgeFlagPointer );
    DISABLE_GL_FUNCTION( glEdgeFlagv );
    DISABLE_GL_FUNCTION( glEnable );
    DISABLE_GL_FUNCTION( glEnableClientState );
    DISABLE_GL_FUNCTION( glEnd );
    DISABLE_GL_FUNCTION( glEndList );
    DISABLE_GL_FUNCTION( glEvalCoord1d );
    DISABLE_GL_FUNCTION( glEvalCoord1dv );
    DISABLE_GL_FUNCTION( glEvalCoord1f );
    DISABLE_GL_FUNCTION( glEvalCoord1fv );
    DISABLE_GL_FUNCTION( glEvalCoord2d );
    DISABLE_GL_FUNCTION( glEvalCoord2dv );
    DISABLE_GL_FUNCTION( glEvalCoord2f );
    DISABLE_GL_FUNCTION( glEvalCoord2fv );
    DISABLE_GL_FUNCTION( glEvalMesh1 );
    DISABLE_GL_FUNCTION( glEvalMesh2 );
    DISABLE_GL_FUNCTION( glEvalPoint1 );
    DISABLE_GL_FUNCTION( glEvalPoint2 );
    DISABLE_GL_FUNCTION( glFeedbackBuffer );
    DISABLE_GL_FUNCTION( glFinish );
    DISABLE_GL_FUNCTION( glFlush );
    DISABLE_GL_FUNCTION( glFogf );
    DISABLE_GL_FUNCTION( glFogfv );
    DISABLE_GL_FUNCTION( glFogi );
    DISABLE_GL_FUNCTION( glFogiv );
    DISABLE_GL_FUNCTION( glFrontFace );
    DISABLE_GL_FUNCTION( glFrustum );
    DISABLE_GL_FUNCTION( glGenLists );
    DISABLE_GL_FUNCTION( glGenTextures );
    DISABLE_GL_FUNCTION( glGetBooleanv );
    DISABLE_GL_FUNCTION( glGetClipPlane );
    DISABLE_GL_FUNCTION( glGetDoublev );
    DISABLE_GL_FUNCTION( glGetError );
    DISABLE_GL_FUNCTION( glGetFloatv );
    DISABLE_GL_FUNCTION( glGetIntegerv );
    DISABLE_GL_FUNCTION( glGetLightfv );
    DISABLE_GL_FUNCTION( glGetLightiv );
    DISABLE_GL_FUNCTION( glGetMapdv );
    DISABLE_GL_FUNCTION( glGetMapfv );
    DISABLE_GL_FUNCTION( glGetMapiv );
    DISABLE_GL_FUNCTION( glGetMaterialfv );
    DISABLE_GL_FUNCTION( glGetMaterialiv );
    DISABLE_GL_FUNCTION( glGetPixelMapfv );
    DISABLE_GL_FUNCTION( glGetPixelMapuiv );
    DISABLE_GL_FUNCTION( glGetPixelMapusv );
    DISABLE_GL_FUNCTION( glGetPointerv );
    DISABLE_GL_FUNCTION( glGetPolygonStipple );
    DISABLE_GL_FUNCTION( glGetString );
    DISABLE_GL_FUNCTION( glGetTexEnvfv );
    DISABLE_GL_FUNCTION( glGetTexEnviv );
    DISABLE_GL_FUNCTION( glGetTexGendv );
    DISABLE_GL_FUNCTION( glGetTexGenfv );
    DISABLE_GL_FUNCTION( glGetTexGeniv );
    DISABLE_GL_FUNCTION( glGetTexImage );
    DISABLE_GL_FUNCTION( glGetTexLevelParameterfv );
    DISABLE_GL_FUNCTION( glGetTexLevelParameteriv );
    DISABLE_GL_FUNCTION( glGetTexParameterfv );
    DISABLE_GL_FUNCTION( glGetTexParameteriv );
    DISABLE_GL_FUNCTION( glHint );
    DISABLE_GL_FUNCTION( glIndexMask );
    DISABLE_GL_FUNCTION( glIndexPointer );
    DISABLE_GL_FUNCTION( glIndexd );
    DISABLE_GL_FUNCTION( glIndexdv );
    DISABLE_GL_FUNCTION( glIndexf );
    DISABLE_GL_FUNCTION( glIndexfv );
    DISABLE_GL_FUNCTION( glIndexi );
    DISABLE_GL_FUNCTION( glIndexiv );
    DISABLE_GL_FUNCTION( glIndexs );
    DISABLE_GL_FUNCTION( glIndexsv );
    DISABLE_GL_FUNCTION( glIndexub );
    DISABLE_GL_FUNCTION( glIndexubv );
    DISABLE_GL_FUNCTION( glInitNames );
    DISABLE_GL_FUNCTION( glInterleavedArrays );
    DISABLE_GL_FUNCTION( glIsEnabled );
    DISABLE_GL_FUNCTION( glIsList );
    DISABLE_GL_FUNCTION( glIsTexture );
    DISABLE_GL_FUNCTION( glLightModelf );
    DISABLE_GL_FUNCTION( glLightModelfv );
    DISABLE_GL_FUNCTION( glLightModeli );
    DISABLE_GL_FUNCTION( glLightModeliv );
    DISABLE_GL_FUNCTION( glLightf );
    DISABLE_GL_FUNCTION( glLightfv );
    DISABLE_GL_FUNCTION( glLighti );
    DISABLE_GL_FUNCTION( glLightiv );
    DISABLE_GL_FUNCTION( glLineStipple );
    DISABLE_GL_FUNCTION( glLineWidth );
    DISABLE_GL_FUNCTION( glListBase );
    DISABLE_GL_FUNCTION( glLoadIdentity );
    DISABLE_GL_FUNCTION( glLoadMatrixd );
    DISABLE_GL_FUNCTION( glLoadMatrixf );
    DISABLE_GL_FUNCTION( glLoadName );
    DISABLE_GL_FUNCTION( glLogicOp );
    DISABLE_GL_FUNCTION( glMap1d );
    DISABLE_GL_FUNCTION( glMap1f );
    DISABLE_GL_FUNCTION( glMap2d );
    DISABLE_GL_FUNCTION( glMap2f );
    DISABLE_GL_FUNCTION( glMapGrid1d );
    DISABLE_GL_FUNCTION( glMapGrid1f );
    DISABLE_GL_FUNCTION( glMapGrid2d );
    DISABLE_GL_FUNCTION( glMapGrid2f );
    DISABLE_GL_FUNCTION( glMaterialf );
    DISABLE_GL_FUNCTION( glMaterialfv );
    DISABLE_GL_FUNCTION( glMateriali );
    DISABLE_GL_FUNCTION( glMaterialiv );
    DISABLE_GL_FUNCTION( glMatrixMode );
    DISABLE_GL_FUNCTION( glMultMatrixd );
    DISABLE_GL_FUNCTION( glMultMatrixf );
    DISABLE_GL_FUNCTION( glNewList );
    DISABLE_GL_FUNCTION( glNormal3b );
    DISABLE_GL_FUNCTION( glNormal3bv );
    DISABLE_GL_FUNCTION( glNormal3d );
    DISABLE_GL_FUNCTION( glNormal3dv );
    DISABLE_GL_FUNCTION( glNormal3f );
    DISABLE_GL_FUNCTION( glNormal3fv );
    DISABLE_GL_FUNCTION( glNormal3i );
    DISABLE_GL_FUNCTION( glNormal3iv );
    DISABLE_GL_FUNCTION( glNormal3s );
    DISABLE_GL_FUNCTION( glNormal3sv );
    DISABLE_GL_FUNCTION( glNormalPointer );
    DISABLE_GL_FUNCTION( glOrtho );
    DISABLE_GL_FUNCTION( glPassThrough );
    DISABLE_GL_FUNCTION( glPixelMapfv );
    DISABLE_GL_FUNCTION( glPixelMapuiv );
    DISABLE_GL_FUNCTION( glPixelMapusv );
    DISABLE_GL_FUNCTION( glPixelStoref );
    DISABLE_GL_FUNCTION( glPixelStorei );
    DISABLE_GL_FUNCTION( glPixelTransferf );
    DISABLE_GL_FUNCTION( glPixelTransferi );
    DISABLE_GL_FUNCTION( glPixelZoom );
    DISABLE_GL_FUNCTION( glPointSize );
    DISABLE_GL_FUNCTION( glPolygonMode );
    DISABLE_GL_FUNCTION( glPolygonOffset );
    DISABLE_GL_FUNCTION( glPolygonStipple );
    DISABLE_GL_FUNCTION( glPopAttrib );
    DISABLE_GL_FUNCTION( glPopClientAttrib );
    DISABLE_GL_FUNCTION( glPopMatrix );
    DISABLE_GL_FUNCTION( glPopName );
    DISABLE_GL_FUNCTION( glPrioritizeTextures );
    DISABLE_GL_FUNCTION( glPushAttrib );
    DISABLE_GL_FUNCTION( glPushClientAttrib );
    DISABLE_GL_FUNCTION( glPushMatrix );
    DISABLE_GL_FUNCTION( glPushName );
    DISABLE_GL_FUNCTION( glRasterPos2d );
    DISABLE_GL_FUNCTION( glRasterPos2dv );
    DISABLE_GL_FUNCTION( glRasterPos2f );
    DISABLE_GL_FUNCTION( glRasterPos2fv );
    DISABLE_GL_FUNCTION( glRasterPos2i );
    DISABLE_GL_FUNCTION( glRasterPos2iv );
    DISABLE_GL_FUNCTION( glRasterPos2s );
    DISABLE_GL_FUNCTION( glRasterPos2sv );
    DISABLE_GL_FUNCTION( glRasterPos3d );
    DISABLE_GL_FUNCTION( glRasterPos3dv );
    DISABLE_GL_FUNCTION( glRasterPos3f );
    DISABLE_GL_FUNCTION( glRasterPos3fv );
    DISABLE_GL_FUNCTION( glRasterPos3i );
    DISABLE_GL_FUNCTION( glRasterPos3iv );
    DISABLE_GL_FUNCTION( glRasterPos3s );
    DISABLE_GL_FUNCTION( glRasterPos3sv );
    DISABLE_GL_FUNCTION( glRasterPos4d );
    DISABLE_GL_FUNCTION( glRasterPos4dv );
    DISABLE_GL_FUNCTION( glRasterPos4f );
    DISABLE_GL_FUNCTION( glRasterPos4fv );
    DISABLE_GL_FUNCTION( glRasterPos4i );
    DISABLE_GL_FUNCTION( glRasterPos4iv );
    DISABLE_GL_FUNCTION( glRasterPos4s );
    DISABLE_GL_FUNCTION( glRasterPos4sv );
    DISABLE_GL_FUNCTION( glReadBuffer );
    DISABLE_GL_FUNCTION( glReadPixels );
    DISABLE_GL_FUNCTION( glRectd );
    DISABLE_GL_FUNCTION( glRectdv );
    DISABLE_GL_FUNCTION( glRectf );
    DISABLE_GL_FUNCTION( glRectfv );
    DISABLE_GL_FUNCTION( glRecti );
    DISABLE_GL_FUNCTION( glRectiv );
    DISABLE_GL_FUNCTION( glRects );
    DISABLE_GL_FUNCTION( glRectsv );
    DISABLE_GL_FUNCTION( glRenderMode );
    DISABLE_GL_FUNCTION( glRotated );
    DISABLE_GL_FUNCTION( glRotatef );
    DISABLE_GL_FUNCTION( glScaled );
    DISABLE_GL_FUNCTION( glScalef );
    DISABLE_GL_FUNCTION( glScissor );
    DISABLE_GL_FUNCTION( glSelectBuffer );
    DISABLE_GL_FUNCTION( glShadeModel );
    DISABLE_GL_FUNCTION( glStencilFunc );
    DISABLE_GL_FUNCTION( glStencilMask );
    DISABLE_GL_FUNCTION( glStencilOp );
    DISABLE_GL_FUNCTION( glTexCoord1d );
    DISABLE_GL_FUNCTION( glTexCoord1dv );
    DISABLE_GL_FUNCTION( glTexCoord1f );
    DISABLE_GL_FUNCTION( glTexCoord1fv );
    DISABLE_GL_FUNCTION( glTexCoord1i );
    DISABLE_GL_FUNCTION( glTexCoord1iv );
    DISABLE_GL_FUNCTION( glTexCoord1s );
    DISABLE_GL_FUNCTION( glTexCoord1sv );
    DISABLE_GL_FUNCTION( glTexCoord2d );
    DISABLE_GL_FUNCTION( glTexCoord2dv );
    DISABLE_GL_FUNCTION( glTexCoord2f );
    DISABLE_GL_FUNCTION( glTexCoord2fv );
    DISABLE_GL_FUNCTION( glTexCoord2i );
    DISABLE_GL_FUNCTION( glTexCoord2iv );
    DISABLE_GL_FUNCTION( glTexCoord2s );
    DISABLE_GL_FUNCTION( glTexCoord2sv );
    DISABLE_GL_FUNCTION( glTexCoord3d );
    DISABLE_GL_FUNCTION( glTexCoord3dv );
    DISABLE_GL_FUNCTION( glTexCoord3f );
    DISABLE_GL_FUNCTION( glTexCoord3fv );
    DISABLE_GL_FUNCTION( glTexCoord3i );
    DISABLE_GL_FUNCTION( glTexCoord3iv );
    DISABLE_GL_FUNCTION( glTexCoord3s );
    DISABLE_GL_FUNCTION( glTexCoord3sv );
    DISABLE_GL_FUNCTION( glTexCoord4d );
    DISABLE_GL_FUNCTION( glTexCoord4dv );
    DISABLE_GL_FUNCTION( glTexCoord4f );
    DISABLE_GL_FUNCTION( glTexCoord4fv );
    DISABLE_GL_FUNCTION( glTexCoord4i );
    DISABLE_GL_FUNCTION( glTexCoord4iv );
    DISABLE_GL_FUNCTION( glTexCoord4s );
    DISABLE_GL_FUNCTION( glTexCoord4sv );
    DISABLE_GL_FUNCTION( glTexCoordPointer );
    DISABLE_GL_FUNCTION( glTexEnvf );
    DISABLE_GL_FUNCTION( glTexEnvfv );
    DISABLE_GL_FUNCTION( glTexEnvi );
    DISABLE_GL_FUNCTION( glTexEnviv );
    DISABLE_GL_FUNCTION( glTexGend );
    DISABLE_GL_FUNCTION( glTexGendv );
    DISABLE_GL_FUNCTION( glTexGenf );
    DISABLE_GL_FUNCTION( glTexGenfv );
    DISABLE_GL_FUNCTION( glTexGeni );
    DISABLE_GL_FUNCTION( glTexGeniv );
    DISABLE_GL_FUNCTION( glTexImage1D );
    DISABLE_GL_FUNCTION( glTexImage2D );
    DISABLE_GL_FUNCTION( glTexParameterf );
    DISABLE_GL_FUNCTION( glTexParameterfv );
    DISABLE_GL_FUNCTION( glTexParameteri );
    DISABLE_GL_FUNCTION( glTexParameteriv );
    DISABLE_GL_FUNCTION( glTexSubImage1D );
    DISABLE_GL_FUNCTION( glTexSubImage2D );
    DISABLE_GL_FUNCTION( glTranslated );
    DISABLE_GL_FUNCTION( glTranslatef );
    DISABLE_GL_FUNCTION( glVertex2d );
    DISABLE_GL_FUNCTION( glVertex2dv );
    DISABLE_GL_FUNCTION( glVertex2f );
    DISABLE_GL_FUNCTION( glVertex2fv );
    DISABLE_GL_FUNCTION( glVertex2i );
    DISABLE_GL_FUNCTION( glVertex2iv );
    DISABLE_GL_FUNCTION( glVertex2s );
    DISABLE_GL_FUNCTION( glVertex2sv );
    DISABLE_GL_FUNCTION( glVertex3d );
    DISABLE_GL_FUNCTION( glVertex3dv );
    DISABLE_GL_FUNCTION( glVertex3f );
    DISABLE_GL_FUNCTION( glVertex3fv );
    DISABLE_GL_FUNCTION( glVertex3i );
    DISABLE_GL_FUNCTION( glVertex3iv );
    DISABLE_GL_FUNCTION( glVertex3s );
    DISABLE_GL_FUNCTION( glVertex3sv );
    DISABLE_GL_FUNCTION( glVertex4d );
    DISABLE_GL_FUNCTION( glVertex4dv );
    DISABLE_GL_FUNCTION( glVertex4f );
    DISABLE_GL_FUNCTION( glVertex4fv );
    DISABLE_GL_FUNCTION( glVertex4i );
    DISABLE_GL_FUNCTION( glVertex4iv );
    DISABLE_GL_FUNCTION( glVertex4s );
    DISABLE_GL_FUNCTION( glVertex4sv );
    DISABLE_GL_FUNCTION( glVertexPointer );
    DISABLE_GL_FUNCTION( glViewport );

    DISABLE_GLS_FUNCTION( CopyContext );
    DISABLE_GLS_FUNCTION( CreateContext );
    DISABLE_GLS_FUNCTION( CreateLayerContext );
    DISABLE_GLS_FUNCTION( DeleteContext );
    DISABLE_GLS_FUNCTION( DescribeLayerPlane );
    DISABLE_GLS_FUNCTION( GetCurrentContext );
    DISABLE_GLS_FUNCTION( GetCurrentDC );
    DISABLE_GLS_FUNCTION( GetLayerPaletteEntries );
    DISABLE_GLS_FUNCTION( GetProcAddress );
    DISABLE_GLS_FUNCTION( MakeCurrent );
    DISABLE_GLS_FUNCTION( RealizeLayerPalette );
    DISABLE_GLS_FUNCTION( SetLayerPaletteEntries );
    DISABLE_GLS_FUNCTION( ShareLists );
    DISABLE_GLS_FUNCTION( SwapLayerBuffers );
    DISABLE_GLS_FUNCTION( UseFontBitmaps );
    DISABLE_GLS_FUNCTION( UseFontOutlines );

    DISABLE_GLS_FUNCTION( ChoosePixelFormat );
    DISABLE_GLS_FUNCTION( DescribePixelFormat );
    DISABLE_GLS_FUNCTION( GetPixelFormat );
    DISABLE_GLS_FUNCTION( SetPixelFormat );
    DISABLE_GLS_FUNCTION( SwapBuffers );

#ifndef GLS_NO_GLU  

    DISABLE_GLU_FUNCTION( gluBeginCurve );
    DISABLE_GLU_FUNCTION( gluBeginPolygon );
    DISABLE_GLU_FUNCTION( gluBeginSurface );
    DISABLE_GLU_FUNCTION( gluBeginTrim );
    DISABLE_GLU_FUNCTION( gluBuild1DMipmaps );
    DISABLE_GLU_FUNCTION( gluBuild2DMipmaps );
    DISABLE_GLU_FUNCTION( gluCylinder );
    DISABLE_GLU_FUNCTION( gluDeleteNurbsRenderer );
    DISABLE_GLU_FUNCTION( gluDeleteQuadric );
    DISABLE_GLU_FUNCTION( gluDeleteTess );
    DISABLE_GLU_FUNCTION( gluDisk );
    DISABLE_GLU_FUNCTION( gluEndCurve );
    DISABLE_GLU_FUNCTION( gluEndPolygon );
    DISABLE_GLU_FUNCTION( gluEndSurface );
    DISABLE_GLU_FUNCTION( gluEndTrim );
    DISABLE_GLU_FUNCTION( gluErrorString );
    DISABLE_GLU_FUNCTION( gluGetNurbsProperty );
    DISABLE_GLU_FUNCTION( gluGetString );
    DISABLE_GLU_FUNCTION( gluGetTessProperty );
    DISABLE_GLU_FUNCTION( gluLoadSamplingMatrices );
    DISABLE_GLU_FUNCTION( gluLookAt );
    DISABLE_GLU_FUNCTION( gluNewNurbsRenderer );
    DISABLE_GLU_FUNCTION( gluNewQuadric );
    DISABLE_GLU_FUNCTION( gluNewTess );
    DISABLE_GLU_FUNCTION( gluNextContour );
    DISABLE_GLU_FUNCTION( gluNurbsCallback );
    DISABLE_GLU_FUNCTION( gluNurbsCurve );
    DISABLE_GLU_FUNCTION( gluNurbsProperty );
    DISABLE_GLU_FUNCTION( gluNurbsSurface );
    DISABLE_GLU_FUNCTION( gluOrtho2D );
    DISABLE_GLU_FUNCTION( gluPartialDisk );
    DISABLE_GLU_FUNCTION( gluPerspective );
    DISABLE_GLU_FUNCTION( gluPickMatrix );
    DISABLE_GLU_FUNCTION( gluProject );
    DISABLE_GLU_FUNCTION( gluPwlCurve );
    DISABLE_GLU_FUNCTION( gluQuadricCallback );
    DISABLE_GLU_FUNCTION( gluQuadricDrawStyle );
    DISABLE_GLU_FUNCTION( gluQuadricNormals );
    DISABLE_GLU_FUNCTION( gluQuadricOrientation );
    DISABLE_GLU_FUNCTION( gluQuadricTexture );
    DISABLE_GLU_FUNCTION( gluScaleImage );
    DISABLE_GLU_FUNCTION( gluSphere );
    DISABLE_GLU_FUNCTION( gluTessBeginContour );
    DISABLE_GLU_FUNCTION( gluTessBeginPolygon );
    DISABLE_GLU_FUNCTION( gluTessCallback );
    DISABLE_GLU_FUNCTION( gluTessEndContour );
    DISABLE_GLU_FUNCTION( gluTessEndPolygon );
    DISABLE_GLU_FUNCTION( gluTessNormal );
    DISABLE_GLU_FUNCTION( gluTessProperty );
    DISABLE_GLU_FUNCTION( gluTessVertex );
    DISABLE_GLU_FUNCTION( gluUnProject );

#endif  /* GLS_NO_GLU */

#undef DISABLE_GL_FUNCTION
#undef DISABLE_GLS_FUNCTION
#undef DISABLE_GLU_FUNCTION

    _glsGLDriverFilePath[0] = 0;
    _glsGLUDriverFilePath[0] = 0;
    _glsLoadedDriverIndex = GLS_INVALID_DRIVER_INDEX;
}

/*----------------------------------------------------------------------------

internal utility functions

*/

GLS_STATIC char const *_glsGetEnv( char const *pEnv )
{
    return getenv(pEnv);
}
GLS_STATIC void _glsSetEnv( char const *pEnv, char const *pValue )
{
    /* it's not terribly important that this succeed */
    
    char aBuffer[256];
    int unsigned EnvLength, ValueLength = 0;
    assert(pEnv);
    EnvLength = strlen(pEnv);
    if(pValue)
    {
        ValueLength = strlen(pValue);
    }
    if((EnvLength + 1 + ValueLength) < sizeof(aBuffer))
    {
        sprintf(aBuffer,"%s=%s",pEnv,pValue ? pValue : "");
        _putenv(aBuffer);
    }
}

GLS_STATIC DWORD _glsGetRegistryDWORD( HKEY Root, char const *pKey,
                                       char const *pValue, DWORD Default )
{
    /* it's not terribly important that this succeed */

    HKEY Key = (HKEY)INVALID_HANDLE_VALUE;
    DWORD Value = Default, SizeOfValue = sizeof(Value);
    DWORD Type = REG_DWORD;

    if(RegOpenKeyEx(Root,pKey,0,KEY_QUERY_VALUE,&Key) == ERROR_SUCCESS)
    {
        RegQueryValueEx(Key,pValue,0,&Type,(BYTE*)&Value,&SizeOfValue);
        RegCloseKey(Key);
    }

    return Value;
}
GLS_STATIC void _glsSetRegistryDWORD( HKEY Root, char const *pKey,
                                      char const *pValue, DWORD Value )
{
    /* it's not terribly important that this succeed */

    HKEY Key = (HKEY)INVALID_HANDLE_VALUE;
    DWORD Disposition;
        
    if(RegCreateKeyEx(Root,pKey,0,0,REG_OPTION_NON_VOLATILE,KEY_ALL_ACCESS,0,
                   &Key,&Disposition) == ERROR_SUCCESS)
    {
        RegSetValueEx(Key,pValue,0,REG_DWORD,(BYTE*)&Value,sizeof(Value));
        RegCloseKey(Key);
    }
}

GLS_STATIC BOOL _glsGetDriverFileDate( char const *pDriverFilePath,
                                       gls_driver_file_info *pDriverFileInfo )
{
    BOOL ReturnValue = FALSE;
    BY_HANDLE_FILE_INFORMATION FileInformation;
    HANDLE FileHandle;
    assert(pDriverFilePath && pDriverFileInfo);

    pDriverFileInfo->DriverFileDateHigh =
        pDriverFileInfo->DriverFileDateLow = 0;

    FileHandle = CreateFile(pDriverFilePath,GENERIC_READ,
                            FILE_SHARE_READ,0,OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL,0);

    if(FileHandle != INVALID_HANDLE_VALUE)
    {
        if(GetFileInformationByHandle(FileHandle,&FileInformation))
        {
            pDriverFileInfo->DriverFileDateHigh =
                FileInformation.ftLastWriteTime.dwHighDateTime;
            pDriverFileInfo->DriverFileDateLow =
                FileInformation.ftLastWriteTime.dwLowDateTime;

            ReturnValue = TRUE;
        }
        CloseHandle(FileHandle);
    }
    return ReturnValue;
}

GLS_STATIC char const *_glsGetVersionString( BYTE const *pVersionInfo,
                                             char const *pVersionString,
                                             UINT *pStringSize )
{
    /* we have to try a few different code pages because we don't
       know how to read the version strings in the correct language */
    /* @todo figure out how to read them correctly */
    /* @todo do we want to use the 0000 code page? */

    DWORD aCodePages[] = { 1200, 1252 };
#define NUMBER_OF_CODE_PAGES (sizeof(aCodePages)/sizeof(aCodePages[0]))
    int CodePageIndex;
    BOOL DebugValue;
    char *pString = 0;
    *pStringSize = 0;

    assert(pVersionInfo && pVersionString && pStringSize);

    for(CodePageIndex = 0;
        (CodePageIndex < NUMBER_OF_CODE_PAGES) && !pString;
        CodePageIndex++)
    {
        char aQueryString[1024];

        /* @todo this is not a very maintainable way of doing this */
        /* strlen("\StringFileInfo\12341234\") == 25               */
        if(sizeof(aQueryString) > (25 + strlen(pVersionString)))
        {
            sprintf(aQueryString,"\\StringFileInfo\\%04X%04X\\%s",
                    GetUserDefaultLangID(),aCodePages[CodePageIndex],
                    pVersionString);
            
            DebugValue = VerQueryValue((BYTE *)pVersionInfo,aQueryString,
                                       (void **)&pString,pStringSize);

            assert((DebugValue && pString && *pStringSize)
                   || (!DebugValue && !pString && !*pStringSize));
        }
    }
#undef NUMBER_OF_CODE_PAGES

    return pString;
}

GLS_STATIC BOOL _glsGetDriverFileVersion( char const *pDriverFilePath,
                                          gls_driver_file_info *pDriverFileInfo,
                                          char *pDescription,
                                          int unsigned DescriptionLength )
{
    BOOL ReturnValue = FALSE;
    DWORD dummy;     /* Windows rules */
    DWORD VersionSize;
    assert(pDriverFilePath && pDriverFileInfo);
    VersionSize = GetFileVersionInfoSize((char *)pDriverFilePath,&dummy);

    pDriverFileInfo->DriverFileVersionHigh =
        pDriverFileInfo->DriverFileVersionLow = 0;
    if(pDescription && DescriptionLength)
    {
        pDescription[0] = 0;
    }

    if(VersionSize)
    {
        BYTE *pTempVersionInfo = (BYTE *)malloc(VersionSize);
        if(pTempVersionInfo)
        {
            if(GetFileVersionInfo((char *)pDriverFilePath,0,VersionSize,
                                  pTempVersionInfo))
            {
                VS_FIXEDFILEINFO *pTempFixedInfo;
                int unsigned FixedInfoLength;

                /* first, get the fixed version info */

                if((VerQueryValue(pTempVersionInfo,"\\",
                                  (void **)&pTempFixedInfo,
                                  &FixedInfoLength))
                   && (FixedInfoLength == sizeof(*pTempFixedInfo))
                   && pTempFixedInfo
                   && (pTempFixedInfo->dwSignature == 0xFEEF04BD)) 
                {
                    assert(pTempFixedInfo);

                    pDriverFileInfo->DriverFileVersionHigh =
                        pTempFixedInfo->dwFileVersionMS;
                    pDriverFileInfo->DriverFileVersionLow =
                        pTempFixedInfo->dwFileVersionLS;

                    ReturnValue = TRUE;
                }

                /* now try for the string description, not caring if we fail */

                if(pDescription && DescriptionLength)
                {
                    UINT CompanyNameLength, FileDescriptionLength,
                        ProductNameLength;
                    char const *pCompanyName, *pFileDescription,
                        *pProductName;

                    pCompanyName = _glsGetVersionString(pTempVersionInfo,
                                                        "CompanyName",
                                                        &CompanyNameLength);

                    pFileDescription = _glsGetVersionString(pTempVersionInfo,
                                                            "FileDescription",
                                                            &FileDescriptionLength);

                    pProductName = _glsGetVersionString(pTempVersionInfo,
                                                        "ProductName",
                                                        &ProductNameLength);

                    /* see if the string will fit, including the " - " */
                    if(DescriptionLength >
                       (CompanyNameLength + 3 + max(ProductNameLength,
                                                    FileDescriptionLength)))
                    {
                        /* we're going to create a string something like this:
                           "CompanyName - FileDescription", but with everything
                           optional, choosing FileDescription over ProductName */
                       
                        sprintf(pDescription,"%s%s%s",
                                (pCompanyName ? pCompanyName : ""),
                                ((pCompanyName && (pFileDescription ||
                                                   pProductName)) ? " - " : ""),
                                (pFileDescription ? pFileDescription :
                                 (pProductName ? pProductName : "")));
                    }
                }
            }
            free(pTempVersionInfo);
        }
    }
    return ReturnValue;
}

GLS_STATIC BOOL _glsCopyPathCheckSystemDirectory( char const *pDriverFilePath,
                                                  gls_driver_file_info *pDriverFileInfo,
                                                  BOOL *pIsInSystemDir )
{
    BOOL ReturnValue = FALSE;
    char aSystemDir[MAX_PATH];
    char *pFilePart;
    int PathLength, SystemLength;

    assert(pDriverFilePath && pDriverFileInfo);

    pDriverFileInfo->aDriverFilePath[0] = 0;
    if(pIsInSystemDir)
    {
        *pIsInSystemDir = FALSE;
    }
    
    if(GetFullPathName(pDriverFilePath,sizeof(pDriverFileInfo->aDriverFilePath),
                       pDriverFileInfo->aDriverFilePath,&pFilePart)
       && GetSystemDirectory(aSystemDir,sizeof(aSystemDir)))
    {
        PathLength = pFilePart - pDriverFileInfo->aDriverFilePath;
        strlwr(pDriverFileInfo->aDriverFilePath);
        SystemLength = strlen(aSystemDir);        
        strlwr(aSystemDir);

        /* PathLength is going to include the \, SystemLength is not */
        if(pIsInSystemDir && ((PathLength-1) == SystemLength)
           && (strstr(pDriverFileInfo->aDriverFilePath,aSystemDir) ==
               pDriverFileInfo->aDriverFilePath))
        {
            *pIsInSystemDir = TRUE;
        }
        ReturnValue = TRUE;        
    }
    return ReturnValue;
}

GLS_STATIC BOOL _glsIsDefaultOpenGL( char const *pGLDriverFilePath )
{
    BOOL ReturnValue = FALSE;
    char aTempPath[MAX_PATH], aDriverPath[MAX_PATH], aDefaultPath[MAX_PATH];
    char *pFilePart;

    assert(pGLDriverFilePath);

    /* construct default path */

    GetSystemDirectory(aTempPath,sizeof(aTempPath));
    strcat(aTempPath,"\\opengl32.dll");
    GetFullPathName(aTempPath,sizeof(aDefaultPath),aDefaultPath,
                    &pFilePart);
    
    /* @todo what about .dll, it's optional? */

    if(GetFullPathName(pGLDriverFilePath,sizeof(aDriverPath),aDriverPath,
                       &pFilePart))
    {
        if(!stricmp(aDriverPath,aDefaultPath))
        {
            ReturnValue = TRUE;
        }
    }
    return ReturnValue;
}

GLS_STATIC BOOL _glsCompareDirectories( char const *pA, char const *pB )
{
    BOOL ReturnValue = FALSE;
    char aA[MAX_PATH], aB[MAX_PATH];
    char *pAFile = 0, *pBFile = 0;

    assert(pA && pB);

    GetFullPathName(pA,sizeof(aA),aA,&pAFile);
    GetFullPathName(pB,sizeof(aB),aB,&pBFile);

    if(pAFile)
    {
        *pAFile = 0;
    }
    if(pBFile)
    {
        *pBFile = 0;
    }

    if(!stricmp(aA,aB))
    {
        ReturnValue = TRUE;
    }
    return ReturnValue;
}


/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

Below are the definitions of the function pointers for the binding library.

*/


/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

First come the wgl function pointers.

The first three functions, ChoosePixelFormat, DescribePixelFormat, and
SetPixelFormat are defined to point to stub functions that will
initialize gls to the default driver by calling glsLoadDriver(0)
first.

*/

GLS_STATIC int WINAPI _glsChoosePixelFormat_default( HDC h, CONST PIXELFORMATDESCRIPTOR *p )
{
    if(!_glsGLDriverHandle)
    {
        glsLoadDriver(0);
    }
    return glsChoosePixelFormat(h,p);
}
int   ( WINAPI * GLS_CONST GLS_SCOPE_NAME( ChoosePixelFormat ))(HDC, CONST PIXELFORMATDESCRIPTOR *) = &_glsChoosePixelFormat_default;

GLS_STATIC int WINAPI _glsDescribePixelFormat_default(HDC h, int i, UINT u, LPPIXELFORMATDESCRIPTOR p)
{
    if(!_glsGLDriverHandle)
    {
        glsLoadDriver(0);
    }
    return glsDescribePixelFormat(h,i,u,p);
}
int   ( WINAPI * GLS_CONST GLS_SCOPE_NAME( DescribePixelFormat )) (HDC, int, UINT, LPPIXELFORMATDESCRIPTOR) = &_glsDescribePixelFormat_default;

GLS_STATIC BOOL WINAPI _glsSetPixelFormat_default(HDC h, int i, CONST PIXELFORMATDESCRIPTOR *p )
{
    if(!_glsGLDriverHandle)
    {
        glsLoadDriver(0);
    }
    return glsSetPixelFormat(h,i,p);
}
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SetPixelFormat ))(HDC, int, CONST PIXELFORMATDESCRIPTOR *) = &_glsSetPixelFormat_default;

int   ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetPixelFormat ))(HDC) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SwapBuffers ))(HDC) = 0;

HGLRC WINAPI _glsGetCurrentContext_default(VOID)
{
    if(!_glsGLDriverHandle)
    {
        glsLoadDriver(0);
    }
    return glsGetCurrentContext();
}
HGLRC ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetCurrentContext ))(VOID) = _glsGetCurrentContext_default;

BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( CopyContext ))(HGLRC, HGLRC, UINT) = 0;
HGLRC ( WINAPI * GLS_CONST GLS_SCOPE_NAME( CreateContext ))(HDC) = 0;
HGLRC ( WINAPI * GLS_CONST GLS_SCOPE_NAME( CreateLayerContext ))(HDC, int) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( DeleteContext ))(HGLRC) = 0;
HDC   ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetCurrentDC ))(VOID) = 0;
PROC  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetProcAddress ))(LPCSTR) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( MakeCurrent ))(HDC, HGLRC) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( ShareLists ))(HGLRC, HGLRC) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( UseFontBitmaps ))(HDC, DWORD, DWORD, DWORD) = 0;

BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( UseFontOutlines ))(HDC, DWORD, DWORD, DWORD, FLOAT,
                                           FLOAT, int, LPGLYPHMETRICSFLOAT) = 0;

BOOL ( WINAPI * GLS_CONST GLS_SCOPE_NAME( DescribeLayerPlane ))(HDC, int, int, UINT,
                                            LPLAYERPLANEDESCRIPTOR) = 0;
int  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SetLayerPaletteEntries ))(HDC, int, int, int,
                                                CONST COLORREF *) = 0;
int  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetLayerPaletteEntries ))(HDC, int, int, int,
                                                COLORREF *) = 0;
BOOL ( WINAPI * GLS_CONST GLS_SCOPE_NAME( RealizeLayerPalette ))(HDC, int, BOOL) = 0;
BOOL ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SwapLayerBuffers ))(HDC, UINT) = 0;

/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

wgl extension function pointers.

*/

int   ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SwapIntervalEXT ))( int interval ) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( GetDeviceGammaRamp3DFX ))( HDC, LPVOID ) = 0;
BOOL  ( WINAPI * GLS_CONST GLS_SCOPE_NAME( SetDeviceGammaRamp3DFX ))( HDC, LPVOID ) = 0;

/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

Core OpenGL function pointers.

*/

void ( APIENTRY * GLS_CONST glAccum )(GLenum op, GLfloat value) = 0;
void ( APIENTRY * GLS_CONST glAlphaFunc )(GLenum func, GLclampf ref) = 0;
GLboolean ( APIENTRY * GLS_CONST glAreTexturesResident )(GLsizei n, const GLuint *textures, GLboolean *residences) = 0;
void ( APIENTRY * GLS_CONST glArrayElement )(GLint i) = 0;
void ( APIENTRY * GLS_CONST glBegin )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glBindTexture )(GLenum target, GLuint texture) = 0;
void ( APIENTRY * GLS_CONST glBitmap )(GLsizei width, GLsizei height, GLfloat xorig, GLfloat yorig, GLfloat xmove, GLfloat ymove, const GLubyte *bitmap) = 0;
void ( APIENTRY * GLS_CONST glBlendFunc )(GLenum sfactor, GLenum dfactor) = 0;
void ( APIENTRY * GLS_CONST glCallList )(GLuint list) = 0;
void ( APIENTRY * GLS_CONST glCallLists )(GLsizei n, GLenum type, const GLvoid *lists) = 0;
void ( APIENTRY * GLS_CONST glClear )(GLbitfield mask) = 0;
void ( APIENTRY * GLS_CONST glClearAccum )(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = 0;
void ( APIENTRY * GLS_CONST glClearColor )(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha) = 0;
void ( APIENTRY * GLS_CONST glClearDepth )(GLclampd depth) = 0;
void ( APIENTRY * GLS_CONST glClearIndex )(GLfloat c) = 0;
void ( APIENTRY * GLS_CONST glClearStencil )(GLint s) = 0;
void ( APIENTRY * GLS_CONST glClipPlane )(GLenum plane, const GLdouble *equation) = 0;
void ( APIENTRY * GLS_CONST glColor3b )(GLbyte red, GLbyte green, GLbyte blue) = 0;
void ( APIENTRY * GLS_CONST glColor3bv )(const GLbyte *v) = 0;
void ( APIENTRY * GLS_CONST glColor3d )(GLdouble red, GLdouble green, GLdouble blue) = 0;
void ( APIENTRY * GLS_CONST glColor3dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glColor3f )(GLfloat red, GLfloat green, GLfloat blue) = 0;
void ( APIENTRY * GLS_CONST glColor3fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glColor3i )(GLint red, GLint green, GLint blue) = 0;
void ( APIENTRY * GLS_CONST glColor3iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glColor3s )(GLshort red, GLshort green, GLshort blue) = 0;
void ( APIENTRY * GLS_CONST glColor3sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glColor3ub )(GLubyte red, GLubyte green, GLubyte blue) = 0;
void ( APIENTRY * GLS_CONST glColor3ubv )(const GLubyte *v) = 0;
void ( APIENTRY * GLS_CONST glColor3ui )(GLuint red, GLuint green, GLuint blue) = 0;
void ( APIENTRY * GLS_CONST glColor3uiv )(const GLuint *v) = 0;
void ( APIENTRY * GLS_CONST glColor3us )(GLushort red, GLushort green, GLushort blue) = 0;
void ( APIENTRY * GLS_CONST glColor3usv )(const GLushort *v) = 0;
void ( APIENTRY * GLS_CONST glColor4b )(GLbyte red, GLbyte green, GLbyte blue, GLbyte alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4bv )(const GLbyte *v) = 0;
void ( APIENTRY * GLS_CONST glColor4d )(GLdouble red, GLdouble green, GLdouble blue, GLdouble alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glColor4f )(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glColor4i )(GLint red, GLint green, GLint blue, GLint alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glColor4s )(GLshort red, GLshort green, GLshort blue, GLshort alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glColor4ub )(GLubyte red, GLubyte green, GLubyte blue, GLubyte alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4ubv )(const GLubyte *v) = 0;
void ( APIENTRY * GLS_CONST glColor4ui )(GLuint red, GLuint green, GLuint blue, GLuint alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4uiv )(const GLuint *v) = 0;
void ( APIENTRY * GLS_CONST glColor4us )(GLushort red, GLushort green, GLushort blue, GLushort alpha) = 0;
void ( APIENTRY * GLS_CONST glColor4usv )(const GLushort *v) = 0;
void ( APIENTRY * GLS_CONST glColorMask )(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha) = 0;
void ( APIENTRY * GLS_CONST glColorMaterial )(GLenum face, GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glColorPointer )(GLint size, GLenum type, GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glCopyPixels )(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type) = 0;
void ( APIENTRY * GLS_CONST glCopyTexImage1D )(GLenum target, GLint level, GLenum internalFormat, GLint x, GLint y, GLsizei width, GLint border) = 0;
void ( APIENTRY * GLS_CONST glCopyTexImage2D )(GLenum target, GLint level, GLenum internalFormat, GLint x, GLint y, GLsizei width, GLsizei height, GLint border) = 0;
void ( APIENTRY * GLS_CONST glCopyTexSubImage1D )(GLenum target, GLint level, GLint xoffset, GLint x, GLint y, GLsizei width) = 0;
void ( APIENTRY * GLS_CONST glCopyTexSubImage2D )(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint x, GLint y, GLsizei width, GLsizei height) = 0;
void ( APIENTRY * GLS_CONST glCullFace )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glDeleteLists )(GLuint list, GLsizei range) = 0;
void ( APIENTRY * GLS_CONST glDeleteTextures )(GLsizei n, const GLuint *textures) = 0;
void ( APIENTRY * GLS_CONST glDepthFunc )(GLenum func) = 0;
void ( APIENTRY * GLS_CONST glDepthMask )(GLboolean flag) = 0;
void ( APIENTRY * GLS_CONST glDepthRange )(GLclampd zNear, GLclampd zFar) = 0;
void ( APIENTRY * GLS_CONST glDisable )(GLenum cap) = 0;
void ( APIENTRY * GLS_CONST glDisableClientState )(GLenum array) = 0;
void ( APIENTRY * GLS_CONST glDrawArrays )(GLenum mode, GLint first, GLsizei count) = 0;
void ( APIENTRY * GLS_CONST glDrawBuffer )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glDrawElements )(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices) = 0;
void ( APIENTRY * GLS_CONST glDrawPixels )(GLsizei width, GLsizei height, GLenum format, GLenum type, const GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glEdgeFlag )(GLboolean flag) = 0;
void ( APIENTRY * GLS_CONST glEdgeFlagPointer )(GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glEdgeFlagv )(const GLboolean *flag) = 0;
void ( APIENTRY * GLS_CONST glEnable )(GLenum cap) = 0;
void ( APIENTRY * GLS_CONST glEnableClientState )(GLenum array) = 0;
void ( APIENTRY * GLS_CONST glEnd )(void) = 0;
void ( APIENTRY * GLS_CONST glEndList )(void) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord1d )(GLdouble u) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord1dv )(const GLdouble *u) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord1f )(GLfloat u) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord1fv )(const GLfloat *u) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord2d )(GLdouble u, GLdouble v) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord2dv )(const GLdouble *u) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord2f )(GLfloat u, GLfloat v) = 0;
void ( APIENTRY * GLS_CONST glEvalCoord2fv )(const GLfloat *u) = 0;
void ( APIENTRY * GLS_CONST glEvalMesh1 )(GLenum mode, GLint i1, GLint i2) = 0;
void ( APIENTRY * GLS_CONST glEvalMesh2 )(GLenum mode, GLint i1, GLint i2, GLint j1, GLint j2) = 0;
void ( APIENTRY * GLS_CONST glEvalPoint1 )(GLint i) = 0;
void ( APIENTRY * GLS_CONST glEvalPoint2 )(GLint i, GLint j) = 0;
void ( APIENTRY * GLS_CONST glFeedbackBuffer )(GLsizei size, GLenum type, GLfloat *buffer) = 0;
void ( APIENTRY * GLS_CONST glFinish )(void) = 0;
void ( APIENTRY * GLS_CONST glFlush )(void) = 0;
void ( APIENTRY * GLS_CONST glFogf )(GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glFogfv )(GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glFogi )(GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glFogiv )(GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glFrontFace )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glFrustum )(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar) = 0;
GLuint ( APIENTRY * GLS_CONST glGenLists )(GLsizei range) = 0;
void ( APIENTRY * GLS_CONST glGenTextures )(GLsizei n, GLuint *textures) = 0;
void ( APIENTRY * GLS_CONST glGetBooleanv )(GLenum pname, GLboolean *params) = 0;
void ( APIENTRY * GLS_CONST glGetClipPlane )(GLenum plane, GLdouble *equation) = 0;
void ( APIENTRY * GLS_CONST glGetDoublev )(GLenum pname, GLdouble *params) = 0;
GLenum ( APIENTRY * GLS_CONST glGetError )(void) = 0;
void ( APIENTRY * GLS_CONST glGetFloatv )(GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetIntegerv )(GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetLightfv )(GLenum light, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetLightiv )(GLenum light, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetMapdv )(GLenum target, GLenum query, GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glGetMapfv )(GLenum target, GLenum query, GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glGetMapiv )(GLenum target, GLenum query, GLint *v) = 0;
void ( APIENTRY * GLS_CONST glGetMaterialfv )(GLenum face, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetMaterialiv )(GLenum face, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetPixelMapfv )(GLenum map, GLfloat *values) = 0;
void ( APIENTRY * GLS_CONST glGetPixelMapuiv )(GLenum map, GLuint *values) = 0;
void ( APIENTRY * GLS_CONST glGetPixelMapusv )(GLenum map, GLushort *values) = 0;
void ( APIENTRY * GLS_CONST glGetPointerv )(GLenum pname, GLvoid* *params) = 0;
void ( APIENTRY * GLS_CONST glGetPolygonStipple )(GLubyte *mask) = 0;
const GLubyte * const ( APIENTRY * GLS_CONST glGetString )(GLenum name) = 0;
void ( APIENTRY * GLS_CONST glGetTexEnvfv )(GLenum target, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexEnviv )(GLenum target, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexGendv )(GLenum coord, GLenum pname, GLdouble *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexGenfv )(GLenum coord, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexGeniv )(GLenum coord, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexImage )(GLenum target, GLint level, GLenum format, GLenum type, GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glGetTexLevelParameterfv )(GLenum target, GLint level, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexLevelParameteriv )(GLenum target, GLint level, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexParameterfv )(GLenum target, GLenum pname, GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glGetTexParameteriv )(GLenum target, GLenum pname, GLint *params) = 0;
void ( APIENTRY * GLS_CONST glHint )(GLenum target, GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glIndexMask )(GLuint mask) = 0;
void ( APIENTRY * GLS_CONST glIndexPointer )(GLenum type, GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glIndexd )(GLdouble c) = 0;
void ( APIENTRY * GLS_CONST glIndexdv )(const GLdouble *c) = 0;
void ( APIENTRY * GLS_CONST glIndexf )(GLfloat c) = 0;
void ( APIENTRY * GLS_CONST glIndexfv )(const GLfloat *c) = 0;
void ( APIENTRY * GLS_CONST glIndexi )(GLint c) = 0;
void ( APIENTRY * GLS_CONST glIndexiv )(const GLint *c) = 0;
void ( APIENTRY * GLS_CONST glIndexs )(GLshort c) = 0;
void ( APIENTRY * GLS_CONST glIndexsv )(const GLshort *c) = 0;
void ( APIENTRY * GLS_CONST glIndexub )(GLubyte c) = 0;
void ( APIENTRY * GLS_CONST glIndexubv )(const GLubyte *c) = 0;
void ( APIENTRY * GLS_CONST glInitNames )(void) = 0;
void ( APIENTRY * GLS_CONST glInterleavedArrays )(GLenum format, GLsizei stride, const GLvoid *pointer) = 0;
GLboolean ( APIENTRY * GLS_CONST glIsEnabled )(GLenum cap) = 0;
GLboolean ( APIENTRY * GLS_CONST glIsList )(GLuint list) = 0;
GLboolean ( APIENTRY * GLS_CONST glIsTexture )(GLuint texture) = 0;
void ( APIENTRY * GLS_CONST glLightModelf )(GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glLightModelfv )(GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glLightModeli )(GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glLightModeliv )(GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glLightf )(GLenum light, GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glLightfv )(GLenum light, GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glLighti )(GLenum light, GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glLightiv )(GLenum light, GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glLineStipple )(GLint factor, GLushort pattern) = 0;
void ( APIENTRY * GLS_CONST glLineWidth )(GLfloat width) = 0;
void ( APIENTRY * GLS_CONST glListBase )(GLuint base) = 0;
void ( APIENTRY * GLS_CONST glLoadIdentity )(void) = 0;
void ( APIENTRY * GLS_CONST glLoadMatrixd )(const GLdouble *m) = 0;
void ( APIENTRY * GLS_CONST glLoadMatrixf )(const GLfloat *m) = 0;
void ( APIENTRY * GLS_CONST glLoadName )(GLuint name) = 0;
void ( APIENTRY * GLS_CONST glLogicOp )(GLenum opcode) = 0;
void ( APIENTRY * GLS_CONST glMap1d )(GLenum target, GLdouble u1, GLdouble u2, GLint stride, GLint order, const GLdouble *points) = 0;
void ( APIENTRY * GLS_CONST glMap1f )(GLenum target, GLfloat u1, GLfloat u2, GLint stride, GLint order, const GLfloat *points) = 0;
void ( APIENTRY * GLS_CONST glMap2d )(GLenum target, GLdouble u1, GLdouble u2, GLint ustride, GLint uorder, GLdouble v1, GLdouble v2, GLint vstride, GLint vorder, const GLdouble *points) = 0;
void ( APIENTRY * GLS_CONST glMap2f )(GLenum target, GLfloat u1, GLfloat u2, GLint ustride, GLint uorder, GLfloat v1, GLfloat v2, GLint vstride, GLint vorder, const GLfloat *points) = 0;
void ( APIENTRY * GLS_CONST glMapGrid1d )(GLint un, GLdouble u1, GLdouble u2) = 0;
void ( APIENTRY * GLS_CONST glMapGrid1f )(GLint un, GLfloat u1, GLfloat u2) = 0;
void ( APIENTRY * GLS_CONST glMapGrid2d )(GLint un, GLdouble u1, GLdouble u2, GLint vn, GLdouble v1, GLdouble v2) = 0;
void ( APIENTRY * GLS_CONST glMapGrid2f )(GLint un, GLfloat u1, GLfloat u2, GLint vn, GLfloat v1, GLfloat v2) = 0;
void ( APIENTRY * GLS_CONST glMaterialf )(GLenum face, GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glMaterialfv )(GLenum face, GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glMateriali )(GLenum face, GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glMaterialiv )(GLenum face, GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glMatrixMode )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glMultMatrixd )(const GLdouble *m) = 0;
void ( APIENTRY * GLS_CONST glMultMatrixf )(const GLfloat *m) = 0;
void ( APIENTRY * GLS_CONST glNewList )(GLuint list, GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glNormal3b )(GLbyte nx, GLbyte ny, GLbyte nz) = 0;
void ( APIENTRY * GLS_CONST glNormal3bv )(const GLbyte *v) = 0;
void ( APIENTRY * GLS_CONST glNormal3d )(GLdouble nx, GLdouble ny, GLdouble nz) = 0;
void ( APIENTRY * GLS_CONST glNormal3dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glNormal3f )(GLfloat nx, GLfloat ny, GLfloat nz) = 0;
void ( APIENTRY * GLS_CONST glNormal3fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glNormal3i )(GLint nx, GLint ny, GLint nz) = 0;
void ( APIENTRY * GLS_CONST glNormal3iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glNormal3s )(GLshort nx, GLshort ny, GLshort nz) = 0;
void ( APIENTRY * GLS_CONST glNormal3sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glNormalPointer )(GLenum type, GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glOrtho )(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar) = 0;
void ( APIENTRY * GLS_CONST glPassThrough )(GLfloat token) = 0;
void ( APIENTRY * GLS_CONST glPixelMapfv )(GLenum map, GLsizei mapsize, const GLfloat *values) = 0;
void ( APIENTRY * GLS_CONST glPixelMapuiv )(GLenum map, GLsizei mapsize, const GLuint *values) = 0;
void ( APIENTRY * GLS_CONST glPixelMapusv )(GLenum map, GLsizei mapsize, const GLushort *values) = 0;
void ( APIENTRY * GLS_CONST glPixelStoref )(GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glPixelStorei )(GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glPixelTransferf )(GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glPixelTransferi )(GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glPixelZoom )(GLfloat xfactor, GLfloat yfactor) = 0;
void ( APIENTRY * GLS_CONST glPointSize )(GLfloat size) = 0;
void ( APIENTRY * GLS_CONST glPolygonMode )(GLenum face, GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glPolygonOffset )(GLfloat factor, GLfloat units) = 0;
void ( APIENTRY * GLS_CONST glPolygonStipple )(const GLubyte *mask) = 0;
void ( APIENTRY * GLS_CONST glPopAttrib )(void) = 0;
void ( APIENTRY * GLS_CONST glPopClientAttrib )(void) = 0;
void ( APIENTRY * GLS_CONST glPopMatrix )(void) = 0;
void ( APIENTRY * GLS_CONST glPopName )(void) = 0;
void ( APIENTRY * GLS_CONST glPrioritizeTextures )(GLsizei n, const GLuint *textures, const GLclampf *priorities) = 0;
void ( APIENTRY * GLS_CONST glPushAttrib )(GLbitfield mask) = 0;
void ( APIENTRY * GLS_CONST glPushClientAttrib )(GLbitfield mask) = 0;
void ( APIENTRY * GLS_CONST glPushMatrix )(void) = 0;
void ( APIENTRY * GLS_CONST glPushName )(GLuint name) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2d )(GLdouble x, GLdouble y) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2f )(GLfloat x, GLfloat y) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2i )(GLint x, GLint y) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2s )(GLshort x, GLshort y) = 0;
void ( APIENTRY * GLS_CONST glRasterPos2sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3d )(GLdouble x, GLdouble y, GLdouble z) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3f )(GLfloat x, GLfloat y, GLfloat z) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3i )(GLint x, GLint y, GLint z) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3s )(GLshort x, GLshort y, GLshort z) = 0;
void ( APIENTRY * GLS_CONST glRasterPos3sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4d )(GLdouble x, GLdouble y, GLdouble z, GLdouble w) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4f )(GLfloat x, GLfloat y, GLfloat z, GLfloat w) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4i )(GLint x, GLint y, GLint z, GLint w) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4s )(GLshort x, GLshort y, GLshort z, GLshort w) = 0;
void ( APIENTRY * GLS_CONST glRasterPos4sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glReadBuffer )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glReadPixels )(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glRectd )(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2) = 0;
void ( APIENTRY * GLS_CONST glRectdv )(const GLdouble *v1, const GLdouble *v2) = 0;
void ( APIENTRY * GLS_CONST glRectf )(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2) = 0;
void ( APIENTRY * GLS_CONST glRectfv )(const GLfloat *v1, const GLfloat *v2) = 0;
void ( APIENTRY * GLS_CONST glRecti )(GLint x1, GLint y1, GLint x2, GLint y2) = 0;
void ( APIENTRY * GLS_CONST glRectiv )(const GLint *v1, const GLint *v2) = 0;
void ( APIENTRY * GLS_CONST glRects )(GLshort x1, GLshort y1, GLshort x2, GLshort y2) = 0;
void ( APIENTRY * GLS_CONST glRectsv )(const GLshort *v1, const GLshort *v2) = 0;
GLint ( APIENTRY * GLS_CONST glRenderMode )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glRotated )(GLdouble angle, GLdouble x, GLdouble y, GLdouble z) = 0;
void ( APIENTRY * GLS_CONST glRotatef )(GLfloat angle, GLfloat x, GLfloat y, GLfloat z) = 0;
void ( APIENTRY * GLS_CONST glScaled )(GLdouble x, GLdouble y, GLdouble z) = 0;
void ( APIENTRY * GLS_CONST glScalef )(GLfloat x, GLfloat y, GLfloat z) = 0;
void ( APIENTRY * GLS_CONST glScissor )(GLint x, GLint y, GLsizei width, GLsizei height) = 0;
void ( APIENTRY * GLS_CONST glSelectBuffer )(GLsizei size, GLuint *buffer) = 0;
void ( APIENTRY * GLS_CONST glShadeModel )(GLenum mode) = 0;
void ( APIENTRY * GLS_CONST glStencilFunc )(GLenum func, GLint ref, GLuint mask) = 0;
void ( APIENTRY * GLS_CONST glStencilMask )(GLuint mask) = 0;
void ( APIENTRY * GLS_CONST glStencilOp )(GLenum fail, GLenum zfail, GLenum zpass) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1d )(GLdouble s) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1f )(GLfloat s) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1i )(GLint s) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1s )(GLshort s) = 0;
void ( APIENTRY * GLS_CONST glTexCoord1sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2d )(GLdouble s, GLdouble t) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2f )(GLfloat s, GLfloat t) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2i )(GLint s, GLint t) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2s )(GLshort s, GLshort t) = 0;
void ( APIENTRY * GLS_CONST glTexCoord2sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3d )(GLdouble s, GLdouble t, GLdouble r) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3f )(GLfloat s, GLfloat t, GLfloat r) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3i )(GLint s, GLint t, GLint r) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3s )(GLshort s, GLshort t, GLshort r) = 0;
void ( APIENTRY * GLS_CONST glTexCoord3sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4d )(GLdouble s, GLdouble t, GLdouble r, GLdouble q) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4f )(GLfloat s, GLfloat t, GLfloat r, GLfloat q) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4i )(GLint s, GLint t, GLint r, GLint q) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4s )(GLshort s, GLshort t, GLshort r, GLshort q) = 0;
void ( APIENTRY * GLS_CONST glTexCoord4sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glTexCoordPointer )(GLint size, GLenum type, GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glTexEnvf )(GLenum target, GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glTexEnvfv )(GLenum target, GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glTexEnvi )(GLenum target, GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glTexEnviv )(GLenum target, GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glTexGend )(GLenum coord, GLenum pname, GLdouble param) = 0;
void ( APIENTRY * GLS_CONST glTexGendv )(GLenum coord, GLenum pname, const GLdouble *params) = 0;
void ( APIENTRY * GLS_CONST glTexGenf )(GLenum coord, GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glTexGenfv )(GLenum coord, GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glTexGeni )(GLenum coord, GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glTexGeniv )(GLenum coord, GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glTexImage1D )(GLenum target, GLint level, GLint internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glTexImage2D )(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glTexParameterf )(GLenum target, GLenum pname, GLfloat param) = 0;
void ( APIENTRY * GLS_CONST glTexParameterfv )(GLenum target, GLenum pname, const GLfloat *params) = 0;
void ( APIENTRY * GLS_CONST glTexParameteri )(GLenum target, GLenum pname, GLint param) = 0;
void ( APIENTRY * GLS_CONST glTexParameteriv )(GLenum target, GLenum pname, const GLint *params) = 0;
void ( APIENTRY * GLS_CONST glTexSubImage1D )(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLenum type, const GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glTexSubImage2D )(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLenum type, const GLvoid *pixels) = 0;
void ( APIENTRY * GLS_CONST glTranslated )(GLdouble x, GLdouble y, GLdouble z) = 0;
void ( APIENTRY * GLS_CONST glTranslatef )(GLfloat x, GLfloat y, GLfloat z) = 0;
void ( APIENTRY * GLS_CONST glVertex2d )(GLdouble x, GLdouble y) = 0;
void ( APIENTRY * GLS_CONST glVertex2dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glVertex2f )(GLfloat x, GLfloat y) = 0;
void ( APIENTRY * GLS_CONST glVertex2fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glVertex2i )(GLint x, GLint y) = 0;
void ( APIENTRY * GLS_CONST glVertex2iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glVertex2s )(GLshort x, GLshort y) = 0;
void ( APIENTRY * GLS_CONST glVertex2sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glVertex3d )(GLdouble x, GLdouble y, GLdouble z) = 0;
void ( APIENTRY * GLS_CONST glVertex3dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glVertex3f )(GLfloat x, GLfloat y, GLfloat z) = 0;
void ( APIENTRY * GLS_CONST glVertex3fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glVertex3i )(GLint x, GLint y, GLint z) = 0;
void ( APIENTRY * GLS_CONST glVertex3iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glVertex3s )(GLshort x, GLshort y, GLshort z) = 0;
void ( APIENTRY * GLS_CONST glVertex3sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glVertex4d )(GLdouble x, GLdouble y, GLdouble z, GLdouble w) = 0;
void ( APIENTRY * GLS_CONST glVertex4dv )(const GLdouble *v) = 0;
void ( APIENTRY * GLS_CONST glVertex4f )(GLfloat x, GLfloat y, GLfloat z, GLfloat w) = 0;
void ( APIENTRY * GLS_CONST glVertex4fv )(const GLfloat *v) = 0;
void ( APIENTRY * GLS_CONST glVertex4i )(GLint x, GLint y, GLint z, GLint w) = 0;
void ( APIENTRY * GLS_CONST glVertex4iv )(const GLint *v) = 0;
void ( APIENTRY * GLS_CONST glVertex4s )(GLshort x, GLshort y, GLshort z, GLshort w) = 0;
void ( APIENTRY * GLS_CONST glVertex4sv )(const GLshort *v) = 0;
void ( APIENTRY * GLS_CONST glVertexPointer )(GLint size, GLenum type, GLsizei stride, const GLvoid *pointer) = 0;
void ( APIENTRY * GLS_CONST glViewport )(GLint x, GLint y, GLsizei width, GLsizei height) = 0;

/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

GLU function pointers.

*/

#ifndef GLS_NO_GLU

void ( APIENTRY * GLS_CONST gluBeginCurve ) (GLUnurbs* nurb) = 0;
void ( APIENTRY * GLS_CONST gluBeginPolygon ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluBeginSurface ) (GLUnurbs* nurb) = 0;
void ( APIENTRY * GLS_CONST gluBeginTrim ) (GLUnurbs* nurb) = 0;
GLint ( APIENTRY * GLS_CONST gluBuild1DMipmaps ) (GLenum target, GLint component, GLsizei width, GLenum format, GLenum type, const void *data) = 0;
GLint ( APIENTRY * GLS_CONST gluBuild2DMipmaps ) (GLenum target, GLint component, GLsizei width, GLsizei height, GLenum format, GLenum type, const void *data) = 0;
void ( APIENTRY * GLS_CONST gluCylinder ) (GLUquadric* quad, GLdouble base, GLdouble top, GLdouble height, GLint slices, GLint stacks) = 0;
void ( APIENTRY * GLS_CONST gluDeleteNurbsRenderer ) (GLUnurbs* nurb) = 0;
void ( APIENTRY * GLS_CONST gluDeleteQuadric ) (GLUquadric* quad) = 0;
void ( APIENTRY * GLS_CONST gluDeleteTess ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluDisk ) (GLUquadric* quad, GLdouble inner, GLdouble outer, GLint slices, GLint loops) = 0;
void ( APIENTRY * GLS_CONST gluEndCurve ) (GLUnurbs* nurb) = 0;
void ( APIENTRY * GLS_CONST gluEndPolygon ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluEndSurface ) (GLUnurbs* nurb) = 0;
void ( APIENTRY * GLS_CONST gluEndTrim ) (GLUnurbs* nurb) = 0;
const GLubyte * ( APIENTRY * GLS_CONST gluErrorString ) (GLenum error) = 0;
void ( APIENTRY * GLS_CONST gluGetNurbsProperty ) (GLUnurbs* nurb, GLenum property, GLfloat* data) = 0;
const GLubyte * ( APIENTRY * GLS_CONST gluGetString ) (GLenum name) = 0;
void ( APIENTRY * GLS_CONST gluGetTessProperty ) (GLUtesselator* tess, GLenum which, GLdouble* data) = 0;
void ( APIENTRY * GLS_CONST gluLoadSamplingMatrices ) (GLUnurbs* nurb, const GLfloat *model, const GLfloat *perspective, const GLint *view) = 0;
void ( APIENTRY * GLS_CONST gluLookAt ) (GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ, GLdouble centerX, GLdouble centerY, GLdouble centerZ, GLdouble upX, GLdouble upY, GLdouble upZ) = 0;
GLUnurbs* ( APIENTRY * GLS_CONST gluNewNurbsRenderer ) (void) = 0;
GLUquadric* ( APIENTRY * GLS_CONST gluNewQuadric ) (void) = 0;
GLUtesselator* ( APIENTRY * GLS_CONST gluNewTess ) (void) = 0;
void ( APIENTRY * GLS_CONST gluNextContour ) (GLUtesselator* tess, GLenum type) = 0;
void ( APIENTRY * GLS_CONST gluNurbsCallback ) (GLUnurbs* nurb, GLenum which, GLvoid (CALLBACK *CallBackFunc)()) = 0;
void ( APIENTRY * GLS_CONST gluNurbsCurve ) (GLUnurbs* nurb, GLint knotCount, GLfloat *knots, GLint stride, GLfloat *control, GLint order, GLenum type) = 0;
void ( APIENTRY * GLS_CONST gluNurbsProperty ) (GLUnurbs* nurb, GLenum property, GLfloat value) = 0;
void ( APIENTRY * GLS_CONST gluNurbsSurface ) (GLUnurbs* nurb, GLint sKnotCount, GLfloat* sKnots, GLint tKnotCount, GLfloat* tKnots, GLint sStride, GLint tStride, GLfloat* control, GLint sOrder, GLint tOrder, GLenum type) = 0;
void ( APIENTRY * GLS_CONST gluOrtho2D ) (GLdouble left, GLdouble right, GLdouble bottom, GLdouble top) = 0;
void ( APIENTRY * GLS_CONST gluPartialDisk ) (GLUquadric* quad, GLdouble inner, GLdouble outer, GLint slices, GLint loops, GLdouble start, GLdouble sweep) = 0;
void ( APIENTRY * GLS_CONST gluPerspective ) (GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar) = 0;
void ( APIENTRY * GLS_CONST gluPickMatrix ) (GLdouble x, GLdouble y, GLdouble delX, GLdouble delY, GLint *viewport) = 0;
GLint ( APIENTRY * GLS_CONST gluProject ) (GLdouble objX, GLdouble objY, GLdouble objZ, const GLdouble *model, const GLdouble *proj, const GLint *view, GLdouble* winX, GLdouble* winY, GLdouble* winZ) = 0;
void ( APIENTRY * GLS_CONST gluPwlCurve ) (GLUnurbs* nurb, GLint count, GLfloat* data, GLint stride, GLenum type) = 0;
void ( APIENTRY * GLS_CONST gluQuadricCallback ) (GLUquadric* quad, GLenum which, GLvoid (CALLBACK *CallBackFunc)()) = 0;
void ( APIENTRY * GLS_CONST gluQuadricDrawStyle ) (GLUquadric* quad, GLenum draw) = 0;
void ( APIENTRY * GLS_CONST gluQuadricNormals ) (GLUquadric* quad, GLenum normal) = 0;
void ( APIENTRY * GLS_CONST gluQuadricOrientation ) (GLUquadric* quad, GLenum orientation) = 0;
void ( APIENTRY * GLS_CONST gluQuadricTexture ) (GLUquadric* quad, GLboolean texture) = 0;
GLint ( APIENTRY * GLS_CONST gluScaleImage ) (GLenum format, GLsizei wIn, GLsizei hIn, GLenum typeIn, const void *dataIn, GLsizei wOut, GLsizei hOut, GLenum typeOut, GLvoid* dataOut) = 0;
void ( APIENTRY * GLS_CONST gluSphere ) (GLUquadric* quad, GLdouble radius, GLint slices, GLint stacks) = 0;
void ( APIENTRY * GLS_CONST gluTessBeginContour ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluTessBeginPolygon ) (GLUtesselator* tess, GLvoid* data) = 0;
void ( APIENTRY * GLS_CONST gluTessCallback ) (GLUtesselator* tess, GLenum which, GLvoid (CALLBACK *CallBackFunc)()) = 0;
void ( APIENTRY * GLS_CONST gluTessEndContour ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluTessEndPolygon ) (GLUtesselator* tess) = 0;
void ( APIENTRY * GLS_CONST gluTessNormal ) (GLUtesselator* tess, GLdouble valueX, GLdouble valueY, GLdouble valueZ) = 0;
void ( APIENTRY * GLS_CONST gluTessProperty ) (GLUtesselator* tess, GLenum which, GLdouble data) = 0;
void ( APIENTRY * GLS_CONST gluTessVertex ) (GLUtesselator* tess, GLdouble *location, GLvoid* data) = 0;
GLint ( APIENTRY * GLS_CONST gluUnProject ) (GLdouble winX, GLdouble winY, GLdouble winZ, const GLdouble *model, const GLdouble *proj, const GLint *view, GLdouble* objX, GLdouble* objY, GLdouble* objZ) = 0;

#endif   /* GLS_NO_GLU */

/*----------------------------------------------------------------------------
  ----------------------------------------------------------------------------
  ----------------------------------------------------------------------------

OpenGL extension pointers.

*/


