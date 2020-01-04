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

#ifndef B3_SPARSE_FORCE_SOLVER_H
#define B3_SPARSE_FORCE_SOLVER_H

#include <bounce/common/settings.h>

struct b3Vec3;
struct b3Mat33;
struct b3DenseVec3;
struct b3DiagMat33;
struct b3SparseMat33;

// Output of force model.
struct b3SparseForceSolverData
{
	scalar h, inv_h;
	b3DenseVec3* x;
	b3DenseVec3* v;
	b3DenseVec3* f;
	b3SparseMat33* dfdx;
	b3SparseMat33* dfdv;
};

// An implementation for this class must provide forces and derivatives to the integrator.
// Forces without accompanying Jacobians will be integrated explicitly and may 
// make the simulation unstable.
class b3SparseForceModel
{
public:
	// Compute forces and Jacobians.
	virtual void ComputeForces(const b3SparseForceSolverData* data) = 0;
};

// Input for Backward Euler integrator.
struct b3SolveBEInput
{
	// Default constructor sets some parameters for convenience.
	b3SolveBEInput()
	{
		fixedDofCount = 0;
		maxIterations = 1;
		tolerance = B3_EPSILON;
		maxSubIterations = 20;
		subTolerance = B3_EPSILON;
	}

	scalar h; // time-step
	scalar inv_h; // inverse time-step (frequency)

	b3SparseForceModel* forceModel; // force callback

	u32 dofCount; // number of degrees of freedom 

	const b3DenseVec3* x0; // initial position x(t)
	const b3DenseVec3* v0; // initial velocity v(t)
	const b3DenseVec3* fe; // external forces constant over time-step [t, t + h]
	const b3DiagMat33* M; // mass matrix for inertial effects
	const b3DenseVec3* y; // translation such that x(t + h) = x(t + h) + y

	u32 fixedDofCount; // number of constrained degrees of freedom
	u32* fixedDofs; // indices to constrained degrees of freedom

	u32 maxIterations; // max of outer iterations
	scalar tolerance; // outer tolerance. units: m^2/s^2 
	
	u32 maxSubIterations; // max of inner iterations
	scalar subTolerance; // inner tolerance. units: m^2/s^2
};

// Output of Backward Euler integrator.
struct b3SolveBEOutput
{
	b3DenseVec3* x; // x(t + h)
	b3DenseVec3* v; // v(t + h)
	u32 iterations; // number of non-linear solver iterations
	scalar error; // error
	u32 minSubIterations; // min of inner iterations
	u32 maxSubIterations; // max of inner iterations
};

// Integrate F = ma over [t, t + h] using Backward Euler.
void b3SparseSolveBE(b3SolveBEOutput* output, const b3SolveBEInput* input);

#endif