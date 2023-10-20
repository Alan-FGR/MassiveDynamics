#pragma once
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>

//TODO namespace

#ifdef DOUBLE_PRECISION

#include "glm/ext/vector_double2.hpp"
#include <glm/ext/matrix_double2x2.hpp>
using real = double;
using vec2 = glm::dvec2;
using mat2x2 = glm::dmat2x2;

// TODO soft float
// TODO fixed 1sign 48dec 14frac

#else

#include "glm/ext/vector_float2_precision.hpp"
#include <glm/ext/matrix_float2x2_precision.hpp>
using real = float;
using vec2 = glm::highp_vec2;
using mat2x2 = glm::highp_mat2x2;

#endif

constexpr real const_pi = glm::pi<real>();
constexpr real const_tau = const_pi * 2;


//TODO start move ------------------
//TODO concepts and restrictions
template <typename T>
constexpr T hashIt(const T& c)
{
	return (c & 0xffffff) * 0x6b43a9b5 >> 19; // https://github.com/plan9foundation/plan9/blob/main/sys/src/libflate/deflate.c
}

template <typename TResult, typename T0, typename T1>
constexpr TResult orderIndependentHash(const T0& a, const T1& b)
{
	return hashIt((TResult)a) ^ hashIt((TResult)b);
}

//TODO end move ------------------


struct Transform
{
	mat2x2 rotation;
	vec2 translation;

	[[nodiscard]] vec2 getRotationCosSin() const { return {rotation[0][0], rotation[0][1]}; }
	[[nodiscard]] vec2 transformPoint(const vec2& p) const { return rotation * p + translation; }
	[[nodiscard]] vec2 transformPointInverse(const vec2& p) const { return inverse(rotation) * (p - translation); }
};

struct Aabb
{
	vec2 min;
	vec2 max;

	Aabb() = default;

	Aabb(const real& minX, const real& minY, const real& maxX, const real& maxY) :
		min({minX, minY}),
		max({maxX, maxY})
	{
	}

	[[nodiscard]] bool containsPoint(const vec2& point) const
	{
		return point.x > min.x
			&& point.y > min.y
			&& point.x < max.x
			&& point.y < max.y;
	}
};

template <typename T> constexpr auto zero = glm::zero<T>();
template <typename T> constexpr auto identity = glm::identity<T>();
