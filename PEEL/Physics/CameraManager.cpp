///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CameraManager.h"
#include "Camera.h"
#include "Pint.h"

CameraManager::CameraManager() : mNbFrames(0), mNbCameraPoses(0), mCameraPoses(null)
{
}

CameraManager::~CameraManager()
{
	Release();
}

void CameraManager::Release()
{
	DELETEARRAY(mCameraPoses);
}

bool CameraManager::TryLoadCameraData(const char* filename)
{
	IceFile BinFile(filename);
	if(!BinFile.IsValid())
		return false;

	const udword NbCameraPoses = BinFile.LoadDword();
	printf("Loading %d camera poses...\n", NbCameraPoses);
	mCameraPoses = ICE_NEW(Ray)[NbCameraPoses];
	mNbCameraPoses = NbCameraPoses;
	for(udword i=0;i<NbCameraPoses;i++)
	{
		BinFile.LoadBuffer(&mCameraPoses[i].mOrig, sizeof(Point));
		BinFile.LoadBuffer(&mCameraPoses[i].mDir, sizeof(Point));
	}
	return true;
}

bool CameraManager::LoadCameraData(const char* filename)
{
	ASSERT(filename);

	const char* File = FindPEELFile(filename);
	if(!File || !TryLoadCameraData(File))
	{
		printf(_F("Failed to load '%s'\n", filename));
		return false;
	}

/*	if(!TryLoadCameraData(_F("../build/%s", filename)))
	{
		if(!TryLoadCameraData(_F("./%s", filename)))
		{
			printf(_F("Failed to load '%s'\n", filename));
			return false;
		}
	}*/
	return true;
}

void CameraManager::UpdateCameraPose()
{
	SetCamera(mCameraPoses[mNbFrames].mOrig, mCameraPoses[mNbFrames].mDir);
	glMatrixMode(GL_PROJECTION);
	SetupCameraMatrix();

	mNbFrames++;
	if(mNbFrames==mNbCameraPoses)
		mNbFrames=0;
}

extern udword gScreenWidth;
extern udword gScreenHeight;
Point ComputeWorldRay(const GLint viewPort[4], const GLdouble modelMatrix[16], const GLdouble projMatrix[16], int xs, int ys);

void CameraManager::GenerateRays(PintRaycastData* rays, udword nb_rays, float max_dist)
{
	GLint viewPort[4];
	GLdouble modelMatrix[16];
	GLdouble projMatrix[16];
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);

	const float fScreenWidth = float(gScreenWidth)/float(nb_rays);
	const float fScreenHeight = float(gScreenHeight)/float(nb_rays);
	const Point Origin = GetCameraPos();

	udword Offset=0;
	for(udword j=0;j<nb_rays;j++)
	{
		const udword yi = udword(fScreenHeight*float(j));
		for(udword i=0;i<nb_rays;i++)
		{
			const udword xi = udword(fScreenWidth*float(i));
			rays[Offset].mOrigin = Origin;
			rays[Offset].mDir = ComputeWorldRay(viewPort, modelMatrix, projMatrix, xi, yi);
			rays[Offset].mMaxDist = max_dist;
			Offset++;
		}
	}
}