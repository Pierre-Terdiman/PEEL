/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_SOFTBODY_SHAPE_PROXY_H
#define B3_SOFTBODY_SHAPE_PROXY_H

#include <bounce/common/settings.h>

// Shape base type
enum b3SoftBodyShapeBaseType
{
	e_softBodyShapeBase,
	e_softBodyWorldShapeBase,
	e_maxClothShapeBases
};

class b3SoftBodyShapeBase;

// This structure is used to connect the base shape 
// to the broad-phase.
struct b3SoftBodyShapeBaseProxy
{
	b3SoftBodyShapeBase* shape;
	u32 proxyId;
};

// Base shape. 
// Anything inheriting from this class goes inside the broadphase.
class b3SoftBodyShapeBase
{
public:
	// Default ctor does nothing.
	b3SoftBodyShapeBase() { }
	
	// Default dtor does nothing.
	~b3SoftBodyShapeBase() { }

	// Get the proxy type.
	b3SoftBodyShapeBaseType GetBaseType() const { return m_baseType; }
protected:
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;

	// Type
	b3SoftBodyShapeBaseType m_baseType;

	// Proxy 
	b3SoftBodyShapeBaseProxy m_proxy;
};

#endif