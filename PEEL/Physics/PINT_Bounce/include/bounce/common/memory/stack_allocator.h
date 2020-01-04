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

#ifndef B3_STACK_ALLOCATOR_H
#define B3_STACK_ALLOCATOR_H

#include <bounce/common/settings.h>

// Allocate 1 MiB from the stack. 
// Increase as you want.
const u32 b3_maxStackSize = B3_MiB(1);

// A stack allocator.
class b3StackAllocator 
{
public :
	b3StackAllocator();
	~b3StackAllocator();

	void* Allocate(u32 size);
	void Free(void* p);
private :
	struct b3Block 
	{
		u32 size;
		u8* data;
		bool parent;
	};
	
	u32 m_blockCapacity;
	b3Block* m_blocks;
	u32 m_blockCount;

	u32 m_allocatedSize; // marker
	u8 m_memory[b3_maxStackSize];
};

#endif
