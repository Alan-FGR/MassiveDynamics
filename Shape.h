#pragma once

#include "glm/glm.hpp"
#include "glm/ext.hpp"

using namespace glm;

struct Aabb
{
	static Aabb CalculateFrom(const vec2& position, const mat2x2& orientation, const vec2& size);

	Aabb() = default;
	Aabb(const Aabb& other) = default;
	Aabb(Aabb&& other) noexcept = default;
	Aabb& operator=(const Aabb& other) = default;
	Aabb& operator=(Aabb&& other) noexcept = default;

	Aabb(const vec2& min_, const vec2& max_) :
		max(max_),
		min(min_)
	{}

	vec2 max{0};
	vec2 min{0};
};

struct Shape
{
	vec2 size; // TODO can we go without this?
	Aabb aabb;
};
