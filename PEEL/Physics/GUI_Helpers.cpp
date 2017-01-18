///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "GUI_Helpers.h"
#include "MyConvex.h"
#include "TestScenes.h"

float GetFromEditBox(float value, IceEditBox* edit_box, float min_value, float max_value)
{
	if(edit_box)
	{
		float Value;
		bool status = edit_box->GetTextAsFloat(Value);
		ASSERT(status);
		ASSERT(Value>=min_value && Value<=max_value);
		value = Value;
	}
	return value;
}

udword GetFromEditBox(udword value, IceEditBox* edit_box)
{
	if(edit_box)
	{
		sdword Value;
		bool status = edit_box->GetTextAsInt(Value);
		ASSERT(status);
		ASSERT(Value>=0);
		value = udword(Value);
	}
	return value;
}

IceComboBox* CreateConvexObjectComboBox(IceWidget* parent, sdword x, sdword y, bool enabled)
{
	ComboBoxDesc CBBD;
	CBBD.mID		= 0;
	CBBD.mParent	= parent;
	CBBD.mX			= x;
	CBBD.mY			= y;
	CBBD.mWidth		= 150;
	CBBD.mHeight	= 20;
	CBBD.mLabel		= "Convex index";
	IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);
	CB->Add("Convex 0");	// Cylinder
	CB->Add("Convex 1");	// "Big convex" from MAX
	CB->Add("Convex 2");	// Dodecahedron?
	CB->Add("Convex 3");	// Pyramid
	CB->Add("Convex 4");	// Truncated cone
	CB->Add("Convex 5");	// "Sphere"
	CB->Add("Convex 6");	// Cone
	CB->Add("Convex 7");	// Pentagon
	CB->Add("Convex 8");	// Thin pyramid
	CB->Add("Convex 9");	// Cylinder
	CB->Add("Convex 10");	// Debris / random
	CB->Add("Convex 11");	// Debris / random
	CB->Add("Convex 12");	// Debris / random
	CB->Add("Convex 13");	// Debris / random
	CB->Select(CONVEX_INDEX_0);
	CB->SetVisible(true);
	CB->SetEnabled(enabled);
	return CB;
}

IceComboBox* CreateShapeTypeComboBox(IceWidget* parent, sdword x, sdword y, bool enabled, udword mask)
{
	ComboBoxDesc CBBD;
	CBBD.mID		= 0;
	CBBD.mParent	= parent;
	CBBD.mX			= x;
	CBBD.mY			= y;
	CBBD.mWidth		= 150;
	CBBD.mHeight	= 20;
	CBBD.mLabel		= "Shape type";
	IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);

	// See enum PintShape
	CB->Add(mask & SSM_UNDEFINED ? "Undefined" : "Undefined");
	CB->Add(mask & SSM_SPHERE ? "Sphere" : "Sphere (not supported)");
	CB->Add(mask & SSM_CAPSULE ? "Capsule" : "Capsule (not supported)");
	CB->Add(mask & SSM_CYLINDER ? "Cylinder" : "Cylinder (not supported)");
	CB->Add(mask & SSM_BOX ? "Box" : "Box (not supported)");
	CB->Add(mask & SSM_CONVEX ? "Convex" : "Convex (not supported)");
	CB->Add(mask & SSM_MESH ? "Mesh" : "Mesh (not supported)");

	CB->Select(0);
	CB->SetVisible(true);
	CB->SetEnabled(enabled);
	return CB;
}

ResetButton::ResetButton(PhysicsTest& test, const ButtonDesc& desc) : mTest(test), IceButton(desc)
{
}

void ResetButton::OnClick()
{
	mTest.mMustResetTest = true;
}
