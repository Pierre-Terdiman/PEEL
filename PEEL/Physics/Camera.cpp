///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Common.h"
#include "Camera.h"

static	Point		gEye(50.0f, 50.0f, 50.0f);
static	Point		gDir(-0.6f, -0.2f, -0.7f);
static	Point		gViewY(0.0f, 0.0f, 0.0f);
static	const float	gFOV = 60.0f;
extern	float		gCameraSpeed;

void SetCamera(const Point& pos, const Point& dir)
{
	gEye = pos;
	gDir = dir;
}

Point GetCameraPos()
{
	return gEye;
}

Point GetCameraDir()
{
	return gDir;
}

void ResetCamera()
{
	gEye = Point(50.0f, 50.0f, 50.0f);
	gDir = Point(-0.6f, -0.2f, -0.7f);
}

void MoveCameraForward()
{
	gEye += gDir * gCameraSpeed;
}

void MoveCameraBackward()
{
	gEye -= gDir * gCameraSpeed;
}

void MoveCameraRight()
{
	gEye -= gViewY * gCameraSpeed;
}

void MoveCameraLeft()
{
	gEye += gViewY * gCameraSpeed;
}

class MyQuat
{
	public:

	inline_	MyQuat()													{}
	inline_	MyQuat(const MyQuat& q) : x(q.x), y(q.y), z(q.z), w(q.w)	{}
	inline_	MyQuat(const Point& v, float s)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		w = s;
	}

	inline_ float magnitudeSquared() const
	{
		return x*x + y*y + z*z + w*w;
	}

	inline_ float dot(const MyQuat& v) const
	{
		return x * v.x + y * v.y + z * v.z  + w * v.w;
	}

	inline_ float magnitude() const
	{
		return sqrtf(magnitudeSquared());
	}

	inline_ float normalize()
	{
		const float mag = magnitude();
		if(mag)
		{
			const float imag = float(1) / mag;

			x *= imag;
			y *= imag;
			z *= imag;
			w *= imag;
		}
		return mag;
	}

	inline_	void	fromAngleAxis(float Angle, const Point & axis)			// set the Quat by Angle-axis (see AA constructor)
	{
		x = axis.x;
		y = axis.y;
		z = axis.z;

		// required: Normalize the axis

		const float i_length =  float(1.0) / sqrtf( x*x + y*y + z*z );
		
		x = x * i_length;
		y = y * i_length;
		z = z * i_length;

		// now make a clQuaternionernion out of it
		float Half = degToRad(Angle * float(0.5));

		w = cosf(Half);//this used to be w/o deg to rad.
		const float sin_theta_over_two = sinf(Half );
		x = x * sin_theta_over_two;
		y = y * sin_theta_over_two;
		z = z * sin_theta_over_two;
	}

	inline_	MyQuat(const float angle, const Point& axis)				// creates a Quat from an Angle axis -- note that if Angle > 360 the resulting rotation is Angle mod 360
	{
		fromAngleAxis(angle,axis);
	}

	inline_ void	multiply(const MyQuat& left, const Point& right)		// this = a * b
	{
		float a,b,c,d;

		a = - left.x*right.x - left.y*right.y - left.z *right.z;
		b =   left.w*right.x + left.y*right.z - right.y*left.z;
		c =   left.w*right.y + left.z*right.x - right.z*left.x;
		d =   left.w*right.z + left.x*right.y - right.x*left.y;

		w = a;
		x = b;
		y = c;
		z = d;
	}

	inline_	void	rotate(Point & v) const						//rotates passed vec by rot expressed by quaternion.  overwrites arg ith the result.
	{
		//float msq = float(1.0)/magnitudeSquared();	//assume unit quat!
		MyQuat myInverse;
		myInverse.x = -x;//*msq;
		myInverse.y = -y;//*msq;
		myInverse.z = -z;//*msq;
		myInverse.w =  w;//*msq;

		//v = ((*this) * v) ^ myInverse;

		MyQuat left;
		left.multiply(*this,v);
		v.x =left.w*myInverse.x + myInverse.w*left.x + left.y*myInverse.z - myInverse.y*left.z;
		v.y =left.w*myInverse.y + myInverse.w*left.y + left.z*myInverse.x - myInverse.z*left.x;
		v.z =left.w*myInverse.z + myInverse.w*left.z + left.x*myInverse.y - myInverse.x*left.y;
	}

    float x,y,z,w;
};

static const float NxPiF32 = 3.141592653589793f;
void RotateCamera(int dx, int dy)
{
	const Point Up(0.0f, 1.0f, 0.0f);
	gDir.Normalize();
	gViewY = gDir^Up;

	// #### TODO: replicate this using Ice quats
	MyQuat qx(NxPiF32 * dx * 20.0f/ 180.0f, Up);
	qx.rotate(gDir);
	MyQuat qy(NxPiF32 * dy * 20.0f/ 180.0f, gViewY);
	qy.rotate(gDir);
}

extern udword gScreenWidth;
extern udword gScreenHeight;

void SetupCameraMatrix(float z_near, float z_far)
{
	const Point Up(0.0f, 1.0f, 0.0f);
	gDir.Normalize();
	gViewY = gDir^Up;

	const float Width	= float(gScreenWidth);
	const float Height	= float(gScreenHeight);

	glLoadIdentity();
	gluPerspective(gFOV, Width/Height, z_near, z_far);
//	gluPerspective(gFOV, 1.0f, z_near, z_far);
//	gluPerspective(gFOV, ((float)glutGet(GLUT_WINDOW_WIDTH))/((float)glutGet(GLUT_WINDOW_HEIGHT)), z_near, z_far);
	gluLookAt(gEye.x, gEye.y, gEye.z, gEye.x + gDir.x, gEye.y + gDir.y, gEye.z + gDir.z, 0.0f, 1.0f, 0.0f);
}


#define NEW_VERSION
#ifdef NEW_VERSION
// Fetched from the old ICE renderer. More accurate and MUCH faster than the previous crazy gluUnProject-based version...
Point ComputeWorldRay(int xs, int ys)
{
	// Catch width & height
	const float Width	= float(gScreenWidth);
	const float Height	= float(gScreenHeight);

	// Recenter coordinates in camera space ([-1, 1])
	const float u = ((xs - Width*0.5f)/Width)*2.0f;
	const float v = -((ys - Height*0.5f)/Height)*2.0f;

	// Adjust coordinates according to camera aspect ratio
	const float HTan = tanf(0.25f * fabsf(DEGTORAD * gFOV * 2.0f));
	const float VTan = HTan*(Width/Height);

	// Ray in camera space
	const Point CamRay(VTan*u, HTan*v, 1.0f);

	// Compute ray in world space
	Matrix3x3 InvView;
	Point Right, Up;
	ComputeBasis(gDir, Right, Up);
	InvView.SetCol(0, -Right);
	InvView.SetCol(1, Up);
	InvView.SetCol(2, gDir);

	return (InvView * CamRay).Normalize();
}
#else
Point ComputeWorldRay(int xs, int ys)
{
	GLint viewPort[4];
	GLdouble modelMatrix[16];
	GLdouble projMatrix[16];
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);

	ys = viewPort[3] - ys - 1;
	GLdouble wx0, wy0, wz0;
	gluUnProject((GLdouble) xs, (GLdouble) ys, 0.0, modelMatrix, projMatrix, viewPort, &wx0, &wy0, &wz0);
	GLdouble wx1, wy1, wz1;
	int ret = gluUnProject((GLdouble) xs, (GLdouble) ys, 1.0, modelMatrix, projMatrix, viewPort, &wx1, &wy1, &wz1);
	if(!ret)
		printf("gluUnProject failed\n");
	Point tmp(float(wx1-wx0), float(wy1-wy0), float(wz1-wz0));
	tmp.Normalize();

	if(tmp.Dot(gDir)<0.0f)
		tmp = -tmp;

	return tmp;
}
#endif