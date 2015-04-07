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
	void	SetupCameraMatrix();
	Point	ComputeWorldRay(int xs, int ys);

#endif
