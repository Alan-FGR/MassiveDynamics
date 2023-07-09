#pragma once

#include <vector>

// TODO profile this vs custom impl
template <class T>
struct AvxAllocator
{
    //using value_type = T;
    typedef T value_type;

    AvxAllocator() = default;
    template <class U>
    constexpr AvxAllocator(const AvxAllocator<U>&) noexcept {}

    [[nodiscard]] T* allocate(size_t n)
    {
        T* ptr = static_cast<T*>(_mm_malloc(n * sizeof(T) + 32, 32));

        if (ptr == nullptr)
            throw std::bad_alloc();

        return ptr;
    }

    void deallocate(T* p, size_t) noexcept
    {
        _mm_free(p);
    }
};

template<typename T>
using AvxVector = std::vector<T, AvxAllocator<T>>;