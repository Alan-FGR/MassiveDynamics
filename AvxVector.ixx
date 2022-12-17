export module AvxVector;

import <vector>;

// TODO profile this vs custom impl
export template <class T>
struct avx_allocator
{
    //using value_type = T;
    typedef T value_type;

    avx_allocator() = default;
    template <class U>
    constexpr avx_allocator(const avx_allocator<U>&) noexcept {}

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

export template<typename T>
using avx_vector = std::vector<T, avx_allocator<T>>;