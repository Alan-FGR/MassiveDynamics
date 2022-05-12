export module MxBuffer;

import <vector>;

template <class T>
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

export
template<typename T>
struct MxBuffer
{
	unsigned& size() { return _size; }

private:
    unsigned _size;

	// TODO don't use this nonsense
	std::vector<T, avx_allocator<T>> _backingBuffer;
};