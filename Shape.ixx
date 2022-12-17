export module Geometry;

import "glm/glm.hpp";
import "glm/ext.hpp";

using namespace glm;

export struct Aabb
{
	Aabb(const vec2& max_, const vec2& min_) :
		max(max_),
		min(min_)
	{}

	bool IntersectsWith(const Aabb& other) const
	{
		return
			((max.x > other.min.x) || (other.max.x > min.x)) ||
			((max.y > other.min.y) || (other.max.y > min.y));
	}

	vec2 max{ 0 };
	vec2 min{ 0 };
};

export struct Shape
{
	vec2 size;
	Aabb aabb;
};