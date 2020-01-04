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

#ifndef B3_CONVEX_CONTACT_H
#define B3_CONVEX_CONTACT_H

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/manifold.h>
#include <bounce/dynamics/contacts/collide/collide.h>

class b3ConvexContact : public b3Contact 
{
public:
	static b3Contact* Create(b3Shape* shapeA, b3Shape* shapeB, b3BlockPool* allocator);
	static void Destroy(b3Contact* contact, b3BlockPool* allocator);

	b3ConvexContact(b3Shape* shapeA, b3Shape* shapeB);
	~b3ConvexContact() { }

	bool TestOverlap();

	void Collide();
	
	b3Manifold m_stackManifold;
	b3ConvexCache m_cache;
};

#endif
