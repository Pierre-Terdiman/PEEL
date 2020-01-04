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

#ifndef B3_FRAME_ALLOCATOR_H
#define B3_FRAME_ALLOCATOR_H

#include <bounce/common/settings.h>

// Allocate 1 MiB from the stack. 
// Increase as you want.
const u32 b3_maxFrameSize = B3_MiB(1);

// A small frame allocator.
class b3FrameAllocator
{
public:
	b3FrameAllocator();
	~b3FrameAllocator();

	// Allocate a block of memory.
	// Allocate using b3Alloc if the memory is full.
	void* Allocate(u32 size);
	
	// Free the memory if it was allocated by b3Alloc.
	void Free(void* q);
	
	// Reset the memory pointer. 
	// This function should be called at the beginning of a frame.
	void Reset();
private:
	// Block header
	struct b3Block
	{
		u32 size;
		void* p;
		bool parent;
	};

	u8 m_memory[b3_maxFrameSize];
	u8* m_p;
	u32 m_allocatedSize;
};

#endif