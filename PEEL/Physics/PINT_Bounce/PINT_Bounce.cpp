///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PINT_Bounce.h"
#include "..\PINT_Common\PINT_Common.h"

#define nullptr	0
#include <bounce\bounce.h>

	inline_ const Point&	ToPoint(const b3Vec3& p)	{ return (const Point&)p;					}
	inline_ const b3Vec3&	ToB3Vec3(const Point& p)	{ return (const b3Vec3&)p;					}

	ICE_COMPILE_TIME_ASSERT(OFFSET_OF(Quat, p.x)==OFFSET_OF(b3Quat, v.x));
	ICE_COMPILE_TIME_ASSERT(OFFSET_OF(Quat, p.y)==OFFSET_OF(b3Quat, v.y));
	ICE_COMPILE_TIME_ASSERT(OFFSET_OF(Quat, p.z)==OFFSET_OF(b3Quat, v.z));
	ICE_COMPILE_TIME_ASSERT(OFFSET_OF(Quat, w)==OFFSET_OF(b3Quat, s));
	inline_ const Quat&		ToQuat(const b3Quat& q)		{ return (const Quat&)q;					}
	inline_ const b3Quat&	ToB3Quat(const Quat& q)		{ return (const b3Quat&)q;					}


void b3BeginProfileScope(const char* name)
{
}

void b3EndProfileScope()
{
}

#ifdef _DEBUG
	#pragma comment(lib, "./bounce_d.lib")
#else
	#pragma comment(lib, "./bounce.lib")
#endif

///////////////////////////////////////////////////////////////////////////////

static b3World* gWorld = null;

BouncePint::BouncePint()
{
}

BouncePint::~BouncePint()
{
}

void BouncePint::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation = true;
}

void BouncePint::Init(const PINT_WORLD_CREATE& desc)
{
	gWorld = new b3World;

	gWorld->SetGravity(ToB3Vec3(desc.mGravity));
}

void BouncePint::Close()
{
	DELETESINGLE(gWorld);
}

void BouncePint::SetGravity(const Point& gravity)
{
	ASSERT(gWorld);
	gWorld->SetGravity(ToB3Vec3(gravity));
}

udword BouncePint::Update(float dt)
{
	ASSERT(gWorld);

	const u32 velocityIterations = 8;
	const u32 positionIterations = 2;
	gWorld->Step(dt, velocityIterations, positionIterations);

	return 0;
}

Point BouncePint::GetMainColor()
{
	return Point(0.1f, 0.80f, 0.20f);
}

void BouncePint::Render(PintRender& renderer)
{
	if(!gWorld)
		return;

	b3Body* body = gWorld->GetBodyList().m_head;
	while(body)
	{
		const b3Vec3 position = body->GetPosition();
		const b3Quat orientation = body->GetOrientation();

		const PR IcePose(ToPoint(position), ToQuat(orientation));

		b3Shape* shape = body->GetShapeList().m_head;
		while(shape)
		{
			PintShapeRenderer* shapeRenderer = (PintShapeRenderer*)shape->GetUserData();
			if(shapeRenderer)
				shapeRenderer->Render(IcePose);

			shape = shape->GetNext();
		}

		body = body->GetNext();
	}
}

PintObjectHandle BouncePint::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	const udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(gWorld);

	b3Body* body;
	{
		const bool isDyna = desc.mMass!=0.0f;

		b3BodyDef bodyDef;
		bodyDef.type = isDyna ? e_dynamicBody : e_staticBody;
		
		bodyDef.position = ToB3Vec3(desc.mPosition);
		bodyDef.orientation = ToB3Quat(desc.mRotation);
		bodyDef.linearVelocity = ToB3Vec3(desc.mLinearVelocity);
		bodyDef.angularVelocity = ToB3Vec3(desc.mAngularVelocity);
		
		body = gWorld->CreateBody(bodyDef);
	}

	if(!body)
		return null;

//	CreateShapes(desc, actor);

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;
	while(CurrentShape)
	{
//		PxTransform LocalPose(ToPxVec3(CurrentShape->mLocalPos), ToPxQuat(CurrentShape->mLocalRot));

		float restitution = 0.0f;
		float friction = 1.0f;
		if(CurrentShape->mMaterial)
		{
			restitution = CurrentShape->mMaterial->mRestitution;
			friction = CurrentShape->mMaterial->mDynamicFriction;
		}

		b3Shape* shape = null;

		if(CurrentShape->mType==PINT_SHAPE_SPHERE)
		{
			const PINT_SPHERE_CREATE* SphereCreate = static_cast<const PINT_SPHERE_CREATE*>(CurrentShape);

			b3SphereShape bodyShape;
			bodyShape.m_center.SetZero();
			bodyShape.m_radius = SphereCreate->mRadius;

			b3ShapeDef bodyBoxDef;
			bodyBoxDef.shape = &bodyShape;
			bodyBoxDef.density = desc.mMass;
			bodyBoxDef.restitution = restitution;
			bodyBoxDef.friction = friction;
			bodyBoxDef.userData = CurrentShape->mRenderer;
			shape = body->CreateShape(bodyBoxDef);
		}
		else if(CurrentShape->mType==PINT_SHAPE_BOX)
		{
			const PINT_BOX_CREATE* BoxCreate = static_cast<const PINT_BOX_CREATE*>(CurrentShape);

			// TODO: fix leak
//			b3BoxHull bodyBox(BoxCreate->mExtents.x, BoxCreate->mExtents.y, BoxCreate->mExtents.z);
			b3BoxHull* bodyBox = new b3BoxHull(BoxCreate->mExtents.x, BoxCreate->mExtents.y, BoxCreate->mExtents.z);

			b3HullShape bodyShape;
//			bodyShape.m_hull = &bodyBox;
			bodyShape.m_hull = bodyBox;

			b3ShapeDef bodyBoxDef;
			bodyBoxDef.shape = &bodyShape;
			bodyBoxDef.density = desc.mMass;
			bodyBoxDef.restitution = restitution;
			bodyBoxDef.friction = friction;
			bodyBoxDef.userData = CurrentShape->mRenderer;
			shape = body->CreateShape(bodyBoxDef);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CAPSULE)
		{
			const PINT_CAPSULE_CREATE* CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE*>(CurrentShape);
		}
		else if(CurrentShape->mType==PINT_SHAPE_CONVEX)
		{
			const PINT_CONVEX_CREATE* ConvexCreate = static_cast<const PINT_CONVEX_CREATE*>(CurrentShape);
		}
		else if(CurrentShape->mType==PINT_SHAPE_MESH)
		{
		}
		else ASSERT(0);

		ASSERT(shape);
		if(shape)
		{
		}

		CurrentShape = CurrentShape->mNext;
	}

//	if(rigidDynamic)
//		SetupDynamic(*rigidDynamic, desc);

	if(desc.mAddToWorld)
	{
//		AddActor(actor);

//		if(rigidDynamic && !desc.mKinematic)
//			SetupSleeping(rigidDynamic, mParams.mEnableSleeping);
	}
//	return CreateHandle(actor);
	return body;
}

bool BouncePint::ReleaseObject(PintObjectHandle handle)
{
	return false;
}

PintJointHandle BouncePint::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	return null;
}

void BouncePint::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
}

namespace
{
class MyRaycastFilter : public b3RayCastFilter
{
public:
	virtual bool ShouldRayCast(b3Shape* shape)
	{
		return true;
	}

}gRaycastFilter;
}

udword BouncePint::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(gWorld);

	udword NbHits = 0;
	while(nb--)
	{
		const b3Vec3 p1 = ToB3Vec3(raycasts->mOrigin);
		const b3Vec3 p2 = ToB3Vec3(raycasts->mOrigin + raycasts->mDir * raycasts->mMaxDist);

		b3RayCastSingleOutput Hit;
		if(gWorld->RayCastSingle(&Hit, &gRaycastFilter, p1, p2))
		{
			NbHits++;
			dest->mObject = Hit.shape;
			dest->mImpact = ToPoint(Hit.point);
			dest->mNormal = ToPoint(Hit.normal);
			dest->mDistance = Hit.fraction;
		}
		else
		{
			dest->mObject = null;
		}

		raycasts++;
		dest++;
	}
	return NbHits;
}

static BouncePint* gBounce = null;

void Bounce_Init(const PINT_WORLD_CREATE& desc)
{
	ASSERT(!gBounce);
	gBounce = ICE_NEW(BouncePint);
	gBounce->Init(desc);
}

void Bounce_Close()
{
	if(gBounce)
	{
		gBounce->Close();
		delete gBounce;
		gBounce = null;
	}
}

BouncePint* GetBounce()
{
	return gBounce;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gBounceGUI = null;

enum BounceGUIElement
{
	BOUNCE_GUI_MAIN,
	//
};

IceWindow* Bounce_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gBounceGUI, parent, BOUNCE_GUI_MAIN, "Bounce options");

	return Main;
}

void Bounce_CloseGUI()
{
	Common_CloseGUI(gBounceGUI);
}

///////////////////////////////////////////////////////////////////////////////

class BouncePlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return Bounce_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ Bounce_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ Bounce_Init(desc);						}
	virtual	void		Close()												{ Bounce_Close();							}
	virtual	Pint*		GetPint()											{ return GetBounce();						}
};
static BouncePlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}
