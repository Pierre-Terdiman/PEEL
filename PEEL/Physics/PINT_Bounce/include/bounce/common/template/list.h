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

#ifndef B3_LIST_H
#define B3_LIST_H

#include <bounce/common/settings.h>

// A doubly-linked list.
template<class T>
class b3List
{
public:
	b3List()
	{
		m_head = nullptr;
		m_count = 0;
	}

	~b3List() { }
	
	void PushFront(T* link)
	{
		link->m_prev = nullptr;
		link->m_next = m_head;
		if (m_head)
		{
			m_head->m_prev = link;
		}
		m_head = link;
		++m_count;
	}

	void PushAfter(T* prev, T* link)
	{
		link->m_prev = prev;
		
		if (prev->m_next == nullptr)
		{
			link->m_next = nullptr;
		}
		else
		{
			link->m_next = prev->m_next;
			prev->m_next->m_prev = link;
		}

		prev->m_next = link;
		
		++m_count;
	}
	
	void Remove(T* link)
	{
		if (link->m_prev)
		{
			link->m_prev->m_next = link->m_next;
		}
		if (link->m_next)
		{
			link->m_next->m_prev = link->m_prev;
		}
		if (link == m_head)
		{
			m_head = link->m_next;
		}
		--m_count;
	}

	T* m_head;
	u32 m_count;
};

#endif
