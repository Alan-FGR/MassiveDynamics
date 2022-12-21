export module Shape;

import "glm/glm.hpp";
import "glm/ext.hpp";

using namespace glm;

export struct Aabb
{
	static Aabb& CalculateFrom(const vec2& position, const mat2x2& orientation, const vec2& size)
	{
		vec2 sizeDifference = vec2{
			abs(orientation[0].x) * size.x + abs(orientation[1].x) * size.y,
			abs(orientation[0].y) * size.x + abs(orientation[1].y) * size.y
		};
		auto aabb = Aabb{ position - sizeDifference, position + sizeDifference };
		return aabb;
	}

	Aabb() = default;

	Aabb(const vec2& min_, const vec2& max_) :
		min(min_),
		max(max_)
	{}

	bool IntersectsWith(const Aabb& other) const // TODO rem?
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
	vec2 size; // TODO can we go without this?
	Aabb aabb;
};