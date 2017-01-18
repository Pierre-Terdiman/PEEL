///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef LOADER_BIN_H
#define LOADER_BIN_H

	enum TessellationScheme
	{
		TESS_BUTTERFLY,
		TESS_POLYHEDRAL,
	};

	class SurfaceManager;
	void LoadMeshesFromFile_(SurfaceManager& test, const char* filename, const float* scale=null, bool mergeMeshes=false, udword tessellation=0, TessellationScheme ts = TESS_BUTTERFLY);

#endif
