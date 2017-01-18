#include "stdafx.h"
// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2017 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxIO.h"
#include "PxExtensionsAPI.h"
#include "PsFoundation.h"
#include "PxMetaData.h"
#include "ExtDistanceJoint.h"
#include "ExtD6Joint.h"
#include "ExtFixedJoint.h"
#include "ExtPrismaticJoint.h"
#include "ExtRevoluteJoint.h"
#include "ExtSphericalJoint.h"
#include "PxRepXSerializer.h"
//#include "SnRepXCoreSerializer.h"
//#include "SnRepXSerializerImpl.h"
//#include "PxExtensionMetaDataObjects.h"
#include "PxJointRepXSerializer.h"
#include "PxSerializer.h"
#include "ExtSerialization.h"

#if PX_SUPPORT_PVD
#include "ExtPvd.h"
#include "PxPvdDataStream.h"
#include "PxPvdClient.h"
#include "PsPvd.h"
#endif

using namespace physx;
using namespace physx::pvdsdk;

#if PX_SUPPORT_PVD
struct JointConnectionHandler : public PvdClient
{
	JointConnectionHandler() : mPvd(NULL),mConnected(false){}

	PvdDataStream*		getDataStream()
	{
		return NULL;
	}	
	PvdUserRenderer*    getUserRender()
	{
		return NULL;
	}

	void onPvdConnected()
	{
		PvdDataStream* stream = PvdDataStream::create(mPvd);
		if(stream)
		{
			mConnected = true;
			Ext::Pvd::sendClassDescriptions(*stream);	
			stream->release();
		}		
	}

	bool isConnected() const
	{
		return mConnected;
	}

	void onPvdDisconnected()
	{
		mConnected = false;
	}

	void flush()
	{
	}

	PsPvd* mPvd;
	bool mConnected;
};

static JointConnectionHandler gPvdHandler;
#endif

bool PxInitExtensions(PxPhysics& physics, PxPvd* pvd)
{
	PX_ASSERT(static_cast<Ps::Foundation*>(&physics.getFoundation()) == &Ps::Foundation::getInstance());
	PX_UNUSED(physics);
	PX_UNUSED(pvd);
	Ps::Foundation::incRefCount();

#if PX_SUPPORT_PVD
	if(pvd)
	{
		gPvdHandler.mPvd = static_cast<PsPvd*>(pvd);
		gPvdHandler.mPvd->addClient(&gPvdHandler);
	}
#endif

	return true;
}

void PxCloseExtensions(void)
{	
	Ps::Foundation::decRefCount();

#if PX_SUPPORT_PVD
	if(gPvdHandler.mConnected)
	{	
		PX_ASSERT(gPvdHandler.mPvd);
		gPvdHandler.mPvd->removeClient(&gPvdHandler);
		gPvdHandler.mPvd = NULL;
	}
#endif
}

void Ext::RegisterExtensionsSerializers(PxSerializationRegistry& sr)
{
}

void Ext::UnregisterExtensionsSerializers(PxSerializationRegistry& sr)
{
}
