#include "Shape.h"

Aabb Aabb::CalculateFrom(const vec2& position, const mat2x2& orientation, const vec2& size)
{
	vec2 sizeDifference = vec2{
		abs(orientation[0].x) * size.x + abs(orientation[1].x) * size.y,
		abs(orientation[0].y) * size.x + abs(orientation[1].y) * size.y
	};
	auto aabb = Aabb{ position - sizeDifference, position + sizeDifference };
	return aabb;
}
