///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef CAMERA_H
#define CAMERA_H

	void	SetCamera(const Point& pos, const Point& dir);
	Point	GetCameraPos();
	Point	GetCameraDir();
	void	ResetCamera();
	void	MoveCameraForward();
	void	MoveCameraBackward();
	void	MoveCameraRight();
	void	MoveCameraLeft();
	void	RotateCamera(int dx, int dy);
	void	SetupCameraMatrix(float z_near=1.0f, float z_far=10000.0f);
	Point	ComputeWorldRay(int xs, int ys);

#endif
