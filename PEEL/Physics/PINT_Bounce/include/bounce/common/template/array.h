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

#ifndef B3_ARRAY_POD_H
#define B3_ARRAY_POD_H

#include <bounce/common/settings.h>

// An array for bytes (POD).
template <typename T>
class b3Array
{
public:
	const T& operator[](u32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_elements[i];
	}

	T& operator[](u32 i)
	{
		B3_ASSERT(i < m_count);
		return m_elements[i];
	}

	const T* Get(u32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_elements + i;
	}

	T* Get(u32 i)
	{
		B3_ASSERT(i < m_count);
		return m_elements + i;
	}

	const T* Begin() const
	{
		return m_elements;
	}

	T* Begin()
	{
		return m_elements;
	}

	void PushBack(const T& ele)
	{
		if (m_count == m_capacity)
		{
			T* oldElements = m_elements;
			m_capacity *= 2;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_elements, oldElements, m_count * sizeof(T));
			if (oldElements != m_localElements)
			{
				b3Free(oldElements);
			}
		}
		B3_ASSERT(m_count < m_capacity);
		m_elements[m_count] = ele;
		++m_count;
	}

	void PopBack()
	{
		B3_ASSERT(m_count > 0);
		--m_count;
	}

	const T& Back() const
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_count - 1];
	}

	T& Back()
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_count - 1];
	}

	u32 Capacity() const
	{
		return m_capacity;
	}

	u32 Count() const
	{
		return m_count;
	}

	bool IsEmpty() const
	{
		return m_count == 0;
	}

	void Remove(u32 index)
	{
		B3_ASSERT(m_count > 0);
		B3_ASSERT(index < m_count);
		--m_count;
		// Swap current element with its next.
		for (u32 i = index; i < m_count; ++i)
		{
			m_elements[i] = m_elements[i + 1];
		}
	}
	
	void Reserve(u32 size)
	{
		if (m_capacity < size)
		{
			T* oldElements = m_elements;
			m_capacity = 2 * size;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_elements, oldElements, m_count * sizeof(T));
			if (oldElements != m_localElements)
			{
				b3Free(oldElements);
			}
		}

		B3_ASSERT(m_capacity >= size);
	}

	void Resize(u32 size)
	{
		if (m_capacity < size)
		{
			T* oldElements = m_elements;
			m_capacity = 2 * size;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
			memcpy(m_elements, oldElements, m_count * sizeof(T));
			if (oldElements != m_localElements)
			{
				b3Free(oldElements);
			}
		}
		B3_ASSERT(m_capacity >= size);
		m_count = size;
	}

	void Swap(const b3Array<T>& other)
	{
		if (m_elements == other.m_elements)
		{
			return;
		}

		// Ensure sufficient capacity for copy.
		if (m_capacity < other.m_count)
		{
			if (m_elements != m_localElements)
			{
				b3Free(m_elements);
			}
			m_capacity = other.m_capacity;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
		}
		
		// Copy.
		B3_ASSERT(m_capacity >= other.m_count);
		m_count = other.m_count;
		memcpy(m_elements, other.m_elements, other.m_count * sizeof(T));
	}

	void operator=(const b3Array<T>& other)
	{
		Swap(other);
	}
protected:
	b3Array(T* elements, u32 N)
	{
		B3_ASSERT(N > 0);
		m_localElements = elements;
		m_capacity = N;
		m_elements = m_localElements;
		m_count = 0;
	}

	b3Array(const b3Array<T>& other)
	{
		m_localElements = nullptr;
		m_capacity = 0;
		m_elements = nullptr;
		m_count = 0;

		Swap(other);
	}

	~b3Array()
	{
		if (m_elements != m_localElements)
		{
			b3Free(m_elements);
		}
	}

	u32 m_capacity;
	T* m_elements;
	u32 m_count;
	
	T* m_localElements;
};

template <typename T, u32 N>
class b3StackArray : public b3Array<T>
{
public :
	b3StackArray<T, N>() : b3Array<T>(m_stackElements, N)
	{
	}

	b3StackArray<T, N>(const b3StackArray<T, N>& other) : b3Array<T>(other)
	{
	}
	
	b3StackArray<T, N>(const b3Array<T>& other) : b3Array<T>(other)
	{
	}

	void operator=(const b3StackArray<T, N>& other)
	{
		b3Array<T>::Swap((const b3Array<T>& )other);
	}
	
	void operator=(const b3Array<T>& other)
	{
		Swap(other);
	}

protected:
	T m_stackElements[N];
};

#endif
