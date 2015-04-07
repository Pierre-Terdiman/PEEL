///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#pragma once

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#include ".\PINT_Common\PINT_Ice.h"

// Windows includes
#ifdef WIN32
	#include <windows.h>		// General purpose Win32 functions
	#include <windowsx.h>
	#include <vfw.h>			// Header File For Video For Windows
	#include <commctrl.h>		// For the toolbar
	#include <commdlg.h>		// For open/save file structures
	#include <shellapi.h>		// For drag & drop
	#include <shlobj.h>			// For browse info
#endif // WIN32

// Standard includes
#ifdef _DEBUG
	#include <crtdbg.h>			// C runtime debug functions
#endif
	#include <stdio.h>			// Standard Input/Output functions
	#include <stdlib.h>
	#include <float.h>
	#include <malloc.h>			// For _alloca
	#include <math.h>

#include <IceMaths/IceMathsAFX.h>
using namespace IceMaths;

#define Interval	IceMaths::Interval

#include "Meshmerizer\MeshmerizerAFX.h"
#include "IceImageWork\IceImageWorkAFX.h"
#include "IceGUI\IceGUIAFX.h"
using namespace Meshmerizer;
using namespace IceImageWork;
using namespace IceGUI;

#include "GL/glut.h"
#include <vector>
