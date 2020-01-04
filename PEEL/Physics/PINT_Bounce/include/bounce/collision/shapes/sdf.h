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

#ifndef B3_SDF_H
#define B3_SDF_H

#include <bounce/collision/shapes/aabb.h>

// This class represents a signed distance function (SDF) interface.
// You can implement this interface using your own generated SDF.
class b3SDF
{
public: 
	// Default ctor does nothing.
	b3SDF() { }

	// Default virtual dtor does nothing.
	virtual ~b3SDF() { }

	// Return the AABB ideally convering the geometry represented by this SDF.
	// This is usually the domain of the SDF.
	virtual b3AABB GetAABB() const = 0;

	// Evaluate the signed distance function.
	// Optionally output the normal at the SDF boundary if the given pointer is not a null pointer.
	// Return true if the output values are valid and false otherwise.
	virtual bool Evaluate(const b3Vec3& point, scalar& distance, b3Vec3* normal = nullptr) const = 0;
};

#endif