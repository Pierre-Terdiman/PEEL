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

#ifndef B3_STACK_H
#define B3_STACK_H

#include <bounce/common/settings.h>

// A growable LIFO stack with an initial capacity of N.
// If the stack capacity exceeds the initial capacity, the heap 
// is used to increase the capacity of the stack.
template <typename T, u32 N>
class b3Stack
{
public:
	b3Stack()
	{
		m_capacity = N;
		m_elements = m_stackElements;
		m_count = 0;
	}

	~b3Stack()
	{
		if (m_elements != m_stackElements)
		{
			b3Free(m_elements);
		}
		m_elements = nullptr;
	}

	const T& Top() const
	{
		B3_ASSERT(m_count);
		return m_elements[m_count - 1];
	}

	T& Top()
	{
		B3_ASSERT(m_count);
		return m_elements[m_count - 1];
	}

	void Push(const T& ele)
	{
		if (m_count == m_capacity)
		{
			T* oldElements = m_elements;
			m_capacity *= 2;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_elements, oldElements, m_count * sizeof(T));
			if (oldElements != m_stackElements)
			{
				b3Free(oldElements);
			}
		}
		B3_ASSERT(m_count < m_capacity);
		m_elements[m_count] = ele;
		++m_count;
	}

	void Pop()
	{
		B3_ASSERT(m_count);
		--m_count;
	}

	u32 Count() const
	{
		return m_count;
	}

	bool IsEmpty() const
	{
		return m_count == 0;
	}
private:
	u32 m_capacity;
	T* m_elements;
	u32 m_count;
	T m_stackElements[N];
};

#endif
