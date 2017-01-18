///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "GLFontRenderer.h"

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CollisionGroups = "Simple filtering test, replicating the old collision groups from the NovodeX SDK. If the test works, \
the falling spheres should not collide with the spheres below them. The groups will mainly be used to disable collision detection between jointed \
objects, for engines that do not have a built-in feature for doing so.";

START_TEST(CollisionGroups, CATEGORY_API, gDesc_CollisionGroups)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(11.05f, 6.34f, 10.78f), Point(-0.67f, -0.32f, -0.67f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCollisionGroups || !caps.mSupportRigidBodySimulation)
			return false;

		// Group 0 is already used by the default static environment so we start with group 1
		const PintCollisionGroup Layer0_Group = 1;
		const PintCollisionGroup Layer1_Group = 2;

		const PintDisabledGroups DG(Layer0_Group, Layer1_Group);
		pint.SetDisabledGroups(1, &DG);

		const float Radius = 1.0f;
		const udword NbX = 4;
		const udword NbY = 4;
		GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Radius, 4.0f, 4.0f, 1.0f, Layer0_Group);
		GenerateArrayOfSpheres(pint, Radius, NbX, NbY, Radius*4.0f, 4.0f, 4.0f, 1.0f, Layer1_Group);
		return true;
	}

END_TEST(CollisionGroups)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_COMLocalOffset = "Tests that the center-of-mass (COM) local offset is properly taken into account. The box should rotate on its own if \
the test succeeds.";

START_TEST(COMLocalOffset, CATEGORY_API, gDesc_COMLocalOffset)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(4.40f, 2.92f, 5.12f), Point(-0.69f, -0.31f, -0.65f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		const Point Extents(1.5f, 0.5f, 1.0f);
		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer = CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes			= &BoxDesc;
		ObjectDesc.mMass			= 1.0f;
		ObjectDesc.mPosition		= Point(0.0f, Extents.y, 0.0f);
		ObjectDesc.mCOMLocalOffset	= Point(0.0f, 0.0f, 1.5f);
		const PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
		pint.mUserData = Handle;

		return true;
	}

	virtual	void	DrawDebugInfo(Pint& pint, PintRender& render)
	{
		const PintObjectHandle Handle = pint.mUserData;
		const Matrix4x4 GlobalPose = pint.GetWorldTransform(Handle);

		const Point COMLocalOffset(0.0f, 0.0f, 1.5f);

		const Point p = COMLocalOffset * GlobalPose;

		render.DrawLine(p, p+GlobalPose[0], Point(1.0f, 0.0f, 0.0f));
		render.DrawLine(p, p+GlobalPose[1], Point(0.0f, 1.0f, 0.0f));
		render.DrawLine(p, p+GlobalPose[2], Point(0.0f, 0.0f, 1.0f));
	}

END_TEST(COMLocalOffset)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AngularVelocity = "Tests that the SetAngularVelocity and GetAngularVelocity functions work as expected. Set the angular damping to zero \
in each plugin's UI if you want the velocity to remain constant. Otherwise it will decrease at a slightly different rate for each engine, depending on how damping \
is implemented. You can also use this test to check that the 'max angular velocity' setting works.";

START_TEST(AngularVelocity, CATEGORY_API, gDesc_AngularVelocity)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(0.87f, 23.38f, 8.87f), Point(-0.49f, -0.47f, -0.74f));
		desc.mGravity = Point(0.0f, 0.0f, 0.0f);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const Point Extents(1.5f, 0.5f, 1.0f);
		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer = CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= Point(-2.264f, 18.78940f, 3.333f);
		const PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
		pint.mUserData = Handle;

		pint.SetAngularVelocity(Handle, Point(0.0f, 0.0f, 10.0f));

		return true;
	}

	virtual	float		DrawDebugText(Pint& pint, GLFontRenderer& renderer, float y, float text_scale)
	{
		const PintObjectHandle Handle = pint.mUserData;
		const Point AngVel = pint.GetAngularVelocity(Handle);
		renderer.print(0.0f, y, text_scale, _F("Angular velocity: %.3f | %.3f | %.3f\n", AngVel.x, AngVel.y, AngVel.z));
		return y - text_scale;
	}

END_TEST(AngularVelocity)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_AddLocalTorque = "Tests that the AddLocalTorque function works as expected. Set the angular damping to zero \
in each plugin's UI if you want the angular velocity to remain constant. That way you can easily see which velocity has been reached \
in each engine, for the same torque.";

START_TEST(AddLocalTorque, CATEGORY_API, gDesc_AddLocalTorque)

	virtual	void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(0.87f, 23.38f, 8.87f), Point(-0.49f, -0.47f, -0.74f));
		desc.mGravity = Point(0.0f, 0.0f, 0.0f);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRigidBodySimulation)
			return false;

		mCreateDefaultEnvironment = false;

		const Point Extents(1.5f, 0.5f, 1.0f);
		PINT_BOX_CREATE BoxDesc(Extents);
		BoxDesc.mRenderer = CreateBoxRenderer(Extents);

		PINT_OBJECT_CREATE ObjectDesc;
		ObjectDesc.mShapes		= &BoxDesc;
		ObjectDesc.mMass		= 1.0f;
		ObjectDesc.mPosition	= Point(-2.264f, 18.78940f, 3.333f);
		const PintObjectHandle Handle = CreatePintObject(pint, ObjectDesc);
		pint.mUserData = Handle;

//		pint.SetAngularVelocity(Handle, Point(0.0f, 0.0f, 10.0f));
		pint.AddLocalTorque(Handle, Point(0.0f, 0.0f, 10.0f));

		return true;
	}

	virtual	float		DrawDebugText(Pint& pint, GLFontRenderer& renderer, float y, float text_scale)
	{
		const PintObjectHandle Handle = pint.mUserData;
		const Point AngVel = pint.GetAngularVelocity(Handle);
		renderer.print(0.0f, y, text_scale, _F("Angular velocity: %.5f | %.5f | %.5f\n", AngVel.x, AngVel.y, AngVel.z));
		return y - text_scale;
	}

END_TEST(AddLocalTorque)

///////////////////////////////////////////////////////////////////////////////
