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

#ifndef B3_SOFTBODY_FORCE_SOLVER_H
#define B3_SOFTBODY_FORCE_SOLVER_H

#include <bounce/softbody/softbody_time_step.h>

struct b3Vec3;

class b3StackAllocator;

class b3SoftBodyParticle;
class b3SoftBodyForce;

struct b3SoftBodyForceSolverDef
{
	b3SoftBodyTimeStep step;
	b3StackAllocator* stack;
	u32 particleCount;
	b3SoftBodyParticle** particles;
	u32 forceCount;
	b3SoftBodyForce** forces;
};

class b3SoftBodyForceSolver
{
public:
	b3SoftBodyForceSolver(const b3SoftBodyForceSolverDef& def);
	~b3SoftBodyForceSolver();

	void Solve(const b3Vec3& gravity);
private:
	b3SoftBodyTimeStep m_step;

	b3StackAllocator* m_stack;

	u32 m_particleCount;
	b3SoftBodyParticle** m_particles;

	u32 m_forceCount;
	b3SoftBodyForce** m_forces;
};

#endif