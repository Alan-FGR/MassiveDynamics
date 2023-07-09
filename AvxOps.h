#pragma once

#include "immintrin.h"

inline __m256 operator+(__m256 v)
{
	return v;
}

inline __m256 operator-(__m256 v)
{
	return _mm256_xor_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)), v);
}

inline __m256 operator+(__m256 l, __m256 r)
{
	return _mm256_add_ps(l, r);
}

inline __m256 operator-(__m256 l, __m256 r)
{
	return _mm256_sub_ps(l, r);
}

inline __m256 operator*(__m256 l, __m256 r)
{
	return _mm256_mul_ps(l, r);
}

inline __m256 operator/(__m256 l, __m256 r)
{
	return _mm256_div_ps(l, r);
}

inline void operator+=(__m256& l, __m256 r)
{
	l = _mm256_add_ps(l, r);
}

inline void operator-=(__m256& l, __m256 r)
{
	l = _mm256_sub_ps(l, r);
}

inline void operator*=(__m256& l, __m256 r)
{
	l = _mm256_mul_ps(l, r);
}

inline void operator/=(__m256& l, __m256 r)
{
	l = _mm256_div_ps(l, r);
}

inline __m256 operator==(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_EQ_UQ);
}

inline __m256 operator==(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_cmpeq_epi32(l, r));
}

inline __m256 operator!=(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_NEQ_UQ);
}

inline __m256 operator!=(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_xor_si256(_mm256_setzero_si256(), _mm256_cmpeq_epi32(l, r)));
}

inline __m256 operator<(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_LT_OQ);
}

inline __m256 operator<(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_cmpgt_epi32(r, l));
}

inline __m256 operator<=(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_LE_OQ);
}

inline __m256 operator<=(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_xor_si256(_mm256_setzero_si256(), _mm256_cmpgt_epi32(l, r)));
}

inline __m256 operator>(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_GT_OQ);
}

inline __m256 operator>(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_cmpgt_epi32(l, r));
}

inline __m256 operator>=(__m256 l, __m256 r)
{
	return _mm256_cmp_ps(l, r, _CMP_GE_OQ);
}

inline __m256 operator>=(__m256i l, __m256i r)
{
	return _mm256_castsi256_ps(_mm256_xor_si256(_mm256_setzero_si256(), _mm256_cmpgt_epi32(r, l)));
}

inline __m256 operator!(__m256 v)
{
	return _mm256_xor_ps(_mm256_setzero_ps(), v);
}

inline __m256 operator&(__m256 l, __m256 r)
{
	return _mm256_and_ps(l, r);
}

inline __m256 operator|(__m256 l, __m256 r)
{
	return _mm256_or_ps(l, r);
}

inline __m256 operator^(__m256 l, __m256 r)
{
	return _mm256_xor_ps(l, r);
}

inline void operator&=(__m256& l, __m256 r)
{
	l = _mm256_and_ps(l, r);
}

inline void operator|=(__m256& l, __m256 r)
{
	l = _mm256_or_ps(l, r);
}

inline void operator^=(__m256& l, __m256 r)
{
	l = _mm256_xor_ps(l, r);
}

inline void _mm256_load4_ps(__m256& v0, __m256& v1, __m256& v2, __m256& v3, const void* srcPtr, const int indices[8], size_t stride)
{
	const char* ptr = static_cast<const char*>(srcPtr);

	__m128 m128s[8];
	__m256 m256s[4];

	for (size_t i = 0; i < 8; ++i)
		m128s[i] = _mm_load_ps(reinterpret_cast<const float*>(ptr + indices[i] * stride));

	for (size_t i = 0; i < 4; ++i)
		m256s[i] = _mm256_insertf128_ps(_mm256_castps128_ps256(m128s[i]), m128s[i + 4], 1);

	const __m256 temp0 = _mm256_unpacklo_ps(m256s[0], m256s[1]);
	const __m256 temp1 = _mm256_unpackhi_ps(m256s[0], m256s[1]);
	const __m256 temp2 = _mm256_unpacklo_ps(m256s[2], m256s[3]);
	const __m256 temp3 = _mm256_unpackhi_ps(m256s[2], m256s[3]);

	m256s[0] = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(1, 0, 1, 0));
	m256s[1] = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(3, 2, 3, 2));
	m256s[2] = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(1, 0, 1, 0));
	m256s[3] = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(3, 2, 3, 2));

	v0 = m256s[0];
	v1 = m256s[1];
	v2 = m256s[2];
	v3 = m256s[3];
}

inline void _mm256_store8_ps(const __m256& v0, const __m256& v1, const __m256& v2, const __m256& v3, void* srcPtr, const int indices[8], size_t stride)
{
	char* ptr = static_cast<char*>(srcPtr);

	__m256 r0 = v0;
	__m256 r1 = v1;
	__m256 r2 = v2;
	__m256 r3 = v3;

	const __m256 temp0 = _mm256_unpacklo_ps(r0, r1);
	const __m256 temp1 = _mm256_unpackhi_ps(r0, r1);
	const __m256 temp2 = _mm256_unpacklo_ps(r2, r3);
	const __m256 temp3 = _mm256_unpackhi_ps(r2, r3);

	r0 = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(1, 0, 1, 0));
	r1 = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(3, 2, 3, 2));
	r2 = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(1, 0, 1, 0));
	r3 = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(3, 2, 3, 2));

	__m128 hr0 = _mm256_castps256_ps128(r0);
	__m128 hr1 = _mm256_castps256_ps128(r1);
	__m128 hr2 = _mm256_castps256_ps128(r2);
	__m128 hr3 = _mm256_castps256_ps128(r3);
	__m128 hr4 = _mm256_extractf128_ps(r0, 1);
	__m128 hr5 = _mm256_extractf128_ps(r1, 1);
	__m128 hr6 = _mm256_extractf128_ps(r2, 1);
	__m128 hr7 = _mm256_extractf128_ps(r3, 1);

	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[0] * stride), hr0);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[1] * stride), hr1);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[2] * stride), hr2);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[3] * stride), hr3);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[4] * stride), hr4);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[5] * stride), hr5);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[6] * stride), hr6);
	_mm_store_ps(reinterpret_cast<float*>(ptr + indices[7] * stride), hr7);
}

inline void _mm256_load8_permute2f128_ps(__m256& v0, __m256& v1, __m256& v2, __m256& v3, __m256& v4, __m256& v5, __m256& v6, __m256& v7, const void* srcPtr, const int indices[8], unsigned int stride)
{
	const char* ptr = static_cast<const char*>(srcPtr);

	const __m256 temp0 = _mm256_unpacklo_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[0] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[1] * stride)));
	const __m256 temp1 = _mm256_unpackhi_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[0] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[1] * stride)));
	const __m256 temp2 = _mm256_unpacklo_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[2] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[3] * stride)));
	const __m256 temp3 = _mm256_unpackhi_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[2] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[3] * stride)));
	const __m256 temp4 = _mm256_unpacklo_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[4] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[5] * stride)));
	const __m256 temp5 = _mm256_unpackhi_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[4] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[5] * stride)));
	const __m256 temp6 = _mm256_unpacklo_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[6] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[7] * stride)));
	const __m256 temp7 = _mm256_unpackhi_ps(_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[6] * stride)),
		_mm256_load_ps(reinterpret_cast<const float*>(ptr + indices[7] * stride)));

	const __m256 shuffle0 = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(1, 0, 1, 0));
	const __m256 shuffle1 = _mm256_shuffle_ps(temp0, temp2, _MM_SHUFFLE(3, 2, 3, 2));
	const __m256 shuffle2 = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(1, 0, 1, 0));
	const __m256 shuffle3 = _mm256_shuffle_ps(temp1, temp3, _MM_SHUFFLE(3, 2, 3, 2));
	const __m256 shuffle4 = _mm256_shuffle_ps(temp4, temp6, _MM_SHUFFLE(1, 0, 1, 0));
	const __m256 shuffle5 = _mm256_shuffle_ps(temp4, temp6, _MM_SHUFFLE(3, 2, 3, 2));
	const __m256 shuffle6 = _mm256_shuffle_ps(temp5, temp7, _MM_SHUFFLE(1, 0, 1, 0));
	const __m256 shuffle7 = _mm256_shuffle_ps(temp5, temp7, _MM_SHUFFLE(3, 2, 3, 2));

	v0 = _mm256_permute2f128_ps(shuffle0, shuffle4, 0x20);
	v1 = _mm256_permute2f128_ps(shuffle1, shuffle5, 0x20);
	v2 = _mm256_permute2f128_ps(shuffle2, shuffle6, 0x20);
	v3 = _mm256_permute2f128_ps(shuffle3, shuffle7, 0x20);
	v4 = _mm256_permute2f128_ps(shuffle0, shuffle4, 0x31);
	v5 = _mm256_permute2f128_ps(shuffle1, shuffle5, 0x31);
	v6 = _mm256_permute2f128_ps(shuffle2, shuffle6, 0x31);
	v7 = _mm256_permute2f128_ps(shuffle3, shuffle7, 0x31);
}

inline void scalar_load4(float& v0, float& v1, float& v2, float& v3, const void* srcPtr, const int* indices, unsigned int stride)
{
	const float* ptr = reinterpret_cast<const float*>(static_cast<const char*>(srcPtr) + *indices * stride);
	v0 = ptr[0];
	v1 = ptr[1];
	v2 = ptr[2];
	v3 = ptr[3];
}

inline void scalar_store4(const float& v0, const float& v1, const float& v2, const float& v3, void* srcPtr, const int* indices, unsigned int stride)
{
	float* ptr = reinterpret_cast<float*>(static_cast<char*>(srcPtr) + *indices * stride);
	ptr[0] = v0;
	ptr[1] = v1;
	ptr[2] = v2;
	ptr[3] = v3;
}

inline void scalar_load8(
	float& v0, float& v1, float& v2, float& v3, float& v4, float& v5, float& v6, float& v7,
	const void* srcPtr, const int* indices, unsigned int stride)
{
	const float* ptr = reinterpret_cast<const float*>(static_cast<const char*>(srcPtr) + *indices * stride);
	v0 = ptr[0];
	v1 = ptr[1];
	v2 = ptr[2];
	v3 = ptr[3];
	v4 = ptr[4];
	v5 = ptr[5];
	v6 = ptr[6];
	v7 = ptr[7];
}
