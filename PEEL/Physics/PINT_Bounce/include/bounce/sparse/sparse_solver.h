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

#ifndef B3_SPARSE_SOLVER_H
#define B3_SPARSE_SOLVER_H

#include <bounce/common/settings.h>

struct b3DenseVec3;
struct b3SparseMat33;

// Input for CG solver.
struct b3SolveCGInput
{
	const b3SparseMat33* A; // A in Ax = b
	const b3DenseVec3* b; // b in Ax = b
	u32 maxIterations; // maximum CG iterations
	scalar tolerance; // allowed error
};

// Output of CG solver.
struct b3SolveCGOutput
{
	b3DenseVec3* x; // solution. it must be initialized with an initial guess.
	u32 iterations; // number of CG iterations
	scalar error; // error
};

// Solve Ax = b using a preconditioned Conjugate Gradient method.
// The system matrix A must be a positive-definite matrix.
// This solver uses Jacobi preconditioner.
bool b3SparseSolveCG(b3SolveCGOutput* output, const b3SolveCGInput* input);

#endif