///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "TitleWindow.h"

#define	BORDER_SIZE				8
#define	GROUP_BOX_BORDER_SIZE	16

TitleWindow::TitleWindow(const WindowDesc& desc) : IceWindow(desc)
{
	mRGBWindow = null;
//	mRGBGroup = null;
	mFlags |= WF_MOVABLE_CLIENT_AREA;

	mPic.Init(TITLE_WIDTH, TITLE_HEIGHT);
	mPic.Clear();

	CreatePic();
}

TitleWindow::~TitleWindow()
{
}

extern udword gPictureData_Data[];

void TitleWindow::CreatePic()
{
	mPic.Init(TITLE_WIDTH, TITLE_HEIGHT);
	RGBAPixel* Pixels = mPic.GetPixels();
	CopyMemory(Pixels, gPictureData_Data, mPic.GetDataSize());
}

void TitleWindow::redraw()
{
//	CreatePic();
	mRGBWindow->redraw();
}

TitleBitmapWindow::TitleBitmapWindow(const WindowDesc& desc) : IceWindow(desc)
{
	mMainW = null;
}

TitleBitmapWindow::~TitleBitmapWindow()
{
}

void TitleBitmapWindow::redraw()
{
	CopyPictureToWindow(GetHandle(), mMainW->mPic, false);
}

static void CreateBitmapWindow(Container& owner, TitleBitmapWindow*& bitmap, /*IceGroupBox*& group,*/ TitleWindow* parent, sdword x, sdword y, udword width, udword height, const char* label)
{
	const udword OffsetY = 4;	// ### not sure why this is needed

	// Create bitmap window
	WindowDesc WD;
	WD.mParent	= parent;
	WD.mX		= x + GROUP_BOX_BORDER_SIZE;
	WD.mY		= y + GROUP_BOX_BORDER_SIZE + OffsetY;
	WD.mWidth	= width;
	WD.mHeight	= height;
	WD.mLabel	= "BitmapWindow";
	WD.mType	= WINDOW_POPUP;
	WD.mStyle	= WSTYLE_NORMAL;
// ### the better style works but not all pixels get displayed
//	WD.mStyle	= WSTYLE_CLIENT_EDGES|WSTYLE_STATIC_EDGES;
	bitmap = ICE_NEW(TitleBitmapWindow)(WD);
	bitmap->SetVisible(true);
	bitmap->mMainW = parent;
	owner.Add(udword(bitmap));

/*	if(0)
	{
		GroupBoxDesc GBD;
		GBD.mParent	= parent;
		GBD.mX		= x;
		GBD.mY		= y;
		GBD.mWidth	= width + GROUP_BOX_BORDER_SIZE*2;
		GBD.mHeight	= height + GROUP_BOX_BORDER_SIZE*2 + OffsetY;
		GBD.mLabel	= label;
		group = ICE_NEW(IceGroupBox)(GBD);
		owner.Add(udword(group));
	}
	else*/
	{
		EditBoxDesc EBD;
		EBD.mParent		= parent;
		EBD.mX			= x;
		EBD.mY			= y;
		EBD.mWidth		= width + GROUP_BOX_BORDER_SIZE*2;
		EBD.mHeight		= height + GROUP_BOX_BORDER_SIZE*2 + OffsetY;
		EBD.mLabel		= label;
		EBD.mFilter		= EDITBOX_TEXT;
		EBD.mType		= EDITBOX_READ_ONLY;
		IceEditBox* EB = ICE_NEW(IceEditBox)(EBD);
		EB->SetVisible(true);
		owner.Add(udword(EB));
	}
}

TitleWindow* CreateTitleWindow(Container& owner, IceWidget* parent, sdword x, sdword y, sdword width, sdword height)
{
	// Texture size
	const udword TextureWidth = TITLE_WIDTH;
	const udword TextureHeight = TITLE_HEIGHT;

	// Create main window
	TitleWindow* TTWindow = null;
	if(1)
	{
		WindowDesc WD;
		WD.mParent	= parent;
		WD.mX		= x;
		WD.mY		= y;
		WD.mWidth	= width;
		WD.mHeight	= height;
		WD.mLabel	= "Raytracing window";
		WD.mType	= WINDOW_NORMAL;
//		WD.mType	= WINDOW_POPUP;
//		WD.mStyle	= WSTYLE_CLIENT_EDGES;
		WD.mStyle	= WSTYLE_NORMAL;
		TTWindow = ICE_NEW(TitleWindow)(WD);
		TTWindow->SetVisible(true);
		owner.Add(udword(TTWindow));
	}

	CreateBitmapWindow(owner, TTWindow->mRGBWindow, /*TTWindow->mRGBGroup,*/ TTWindow, BORDER_SIZE, BORDER_SIZE, TextureWidth, TextureHeight, "");

	return TTWindow;
}
