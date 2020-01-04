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

#ifndef B3_QUEUE_H
#define B3_QUEUE_H

#include <bounce/common/settings.h>

// A bounded FIFO queue with a fixed capacity of N.
// The capacity must be greater than one.
// The elements are allocated on the stack.
template <typename T, u32 N>
class b3BoundedQueue
{
public:
	b3BoundedQueue()
	{
		B3_ASSERT(N > 1);
		m_count = 0;
		m_back = 0;
		m_front = 0;
	}

	~b3BoundedQueue()
	{
	}

	T* Push(const T& ele)
	{
		if ((m_count + 1) == N)
		{
			return nullptr;
		}

		B3_ASSERT(m_back < N);
		T* e = m_elements + m_back;
		*e = ele;

		m_back = (m_back + 1) % N;
		B3_ASSERT(m_back != m_front);
		++m_count;
		return e;
	}

	bool Pop()
	{
		if (m_front == m_back)
		{
			return false;
		}

		B3_ASSERT(m_front != m_back);
		m_front = (m_front + 1) % N;
		B3_ASSERT(m_count > 0);
		--m_count;
		return true;
	}

	const T& Front() const
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_front];
	}

	T& Front()
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_front];
	}

	const T& Back() const
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_back > 0 ? m_back - 1 : N - 1];
	}

	T& Back()
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_back > 0 ? m_back - 1 : N - 1];
	}

	u32 Count() const
	{
		return m_count;
	}

	bool IsEmpty() const
	{
		return m_back == m_front;
	}
private:
	T m_elements[N];
	u32 m_count;
	u32 m_back;
	u32 m_front;
};

#endif