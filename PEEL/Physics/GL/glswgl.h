/*----------------------------------------------------------------------------

glswgl.h - This header file defines macros to translate a source
    file using the Win32 OpenGL API into one that uses the gls API.  It
    can be used to construct a stub gl.h file to automatically port a
    Win32 OpenGL application to gls.

    See gls.h or http://www.glsetup.com/dev/gls.htm for more information.
    
    Copyright 1999 GLSetup Incorporated, All Rights Reserved.

    You have permission to use this code in your applications, commercial
    or otherwise.  If you distribute source code with your application,
    this file must not be modified.  Please don't mirror these files;
    instead, link to the URL above.
    
    Version 1.0.0.2 Beta
    Created 9-5-99

*/

#ifndef GLSWGL_H

/*-------------- non-wgl* PixelFormat functions from gdi.h --------------*/

/* ChoosePixelFormat  */
#ifdef ChoosePixelFormat 
#undef ChoosePixelFormat 
#endif
#define ChoosePixelFormat  glsChoosePixelFormat 
/* DescribePixelFormat */
#ifdef DescribePixelFormat
#undef DescribePixelFormat
#endif
#define DescribePixelFormat glsDescribePixelFormat
/* SetPixelFormat */
#ifdef SetPixelFormat
#undef SetPixelFormat
#endif
#define SetPixelFormat glsSetPixelFormat
/* GetPixelFormat */
#ifdef GetPixelFormat
#undef GetPixelFormat
#endif
#define GetPixelFormat glsGetPixelFormat
/* SwapBuffers */
#ifdef SwapBuffers
#undef SwapBuffers
#endif
#define SwapBuffers glsSwapBuffers

/*----------------- wgl* functions from opengl32.dll ------------------*/

/* wglChoosePixelFormat  */
#ifdef wglChoosePixelFormat 
#undef wglChoosePixelFormat 
#endif
#define wglChoosePixelFormat  glsChoosePixelFormat 
/* wglDescribePixelFormat */
#ifdef wglDescribePixelFormat
#undef wglDescribePixelFormat
#endif
#define wglDescribePixelFormat glsDescribePixelFormat
/* wglSetPixelFormat */
#ifdef wglSetPixelFormat
#undef wglSetPixelFormat
#endif
#define wglSetPixelFormat glsSetPixelFormat
/* wglGetPixelFormat */
#ifdef wglGetPixelFormat
#undef wglGetPixelFormat
#endif
#define wglGetPixelFormat glsGetPixelFormat
/* wglSwapBuffers */
#ifdef wglSwapBuffers
#undef wglSwapBuffers
#endif
#define wglSwapBuffers glsSwapBuffers

/*--------------------- wgl* functions from gdi.h ---------------------*/

/* wglCopyContext */
#ifdef wglCopyContext
#undef wglCopyContext
#endif
#define wglCopyContext glsCopyContext
/* wglCreateContext */
#ifdef wglCreateContext
#undef wglCreateContext
#endif
#define wglCreateContext glsCreateContext
/* wglCreateLayerContext */
#ifdef wglCreateLayerContext
#undef wglCreateLayerContext
#endif
#define wglCreateLayerContext glsCreateLayerContext
/* wglDeleteContext */
#ifdef wglDeleteContext
#undef wglDeleteContext
#endif
#define wglDeleteContext glsDeleteContext
/* wglGetCurrentContext */
#ifdef wglGetCurrentContext
#undef wglGetCurrentContext
#endif
#define wglGetCurrentContext glsGetCurrentContext
/* wglGetCurrentDC */
#ifdef wglGetCurrentDC
#undef wglGetCurrentDC
#endif
#define wglGetCurrentDC glsGetCurrentDC
/* wglGetProcAddress */
#ifdef wglGetProcAddress
#undef wglGetProcAddress
#endif
#define wglGetProcAddress glsGetProcAddress
/* wglMakeCurrent */
#ifdef wglMakeCurrent
#undef wglMakeCurrent
#endif
#define wglMakeCurrent glsMakeCurrent
/* wglShareLists */
#ifdef wglShareLists
#undef wglShareLists
#endif
#define wglShareLists glsShareLists
/* wglUseFontBitmaps */
#ifdef wglUseFontBitmaps
#undef wglUseFontBitmaps
#endif
#define wglUseFontBitmaps glsUseFontBitmaps
/* wglUseFontOutlines */
#ifdef wglUseFontOutlines
#undef wglUseFontOutlines
#endif
#define wglUseFontOutlines glsUseFontOutlines
/* wglDescribeLayerPlane */
#ifdef wglDescribeLayerPlane
#undef wglDescribeLayerPlane
#endif
#define wglDescribeLayerPlane glsDescribeLayerPlane
/* wglSetLayerPaletteEntries */
#ifdef wglSetLayerPaletteEntries
#undef wglSetLayerPaletteEntries
#endif
#define wglSetLayerPaletteEntries glsSetLayerPaletteEntries
/* wglGetLayerPaletteEntries */
#ifdef wglGetLayerPaletteEntries
#undef wglGetLayerPaletteEntries
#endif
#define wglGetLayerPaletteEntries glsGetLayerPaletteEntries
/* wglRealizeLayerPalette */
#ifdef wglRealizeLayerPalette
#undef wglRealizeLayerPalette
#endif
#define wglRealizeLayerPalette glsRealizeLayerPalette
/* wglSwapLayerBuffers */
#ifdef wglSwapLayerBuffers
#undef wglSwapLayerBuffers
#endif
#define wglSwapLayerBuffers glsSwapLayerBuffers

#define GLSWGL_H
#endif
