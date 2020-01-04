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

#ifndef B3_BLOCK_POOL_H
#define B3_BLOCK_POOL_H

#include <bounce/common/settings.h>

// Number of blocks per chunk.
const u32 b3_blockCount = 32;

// A pool of memory blocks.
class b3BlockPool
{
public:
	b3BlockPool(u32 blockSize);
	~b3BlockPool();

	void* Allocate();
	void Free(void* p);
private:
	struct b3Block
	{
		b3Block* next;
	};

	struct b3Chunk
	{
		b3Block* freeBlocks;
		b3Chunk* next;
	};

	u32 m_blockSize;
	u32 m_chunkSize;

	b3Chunk* m_chunks;
	u32 m_chunkCount;
};

#endif
