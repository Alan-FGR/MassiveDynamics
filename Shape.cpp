#include "Shape.h"

Aabb Aabb::CalculateFrom(const vec2& position, const mat2x2& orientation, const vec2& size)
{
	vec2 sizeDifference = vec2{
		fabsf(orientation[0].x) * size.x + fabsf(orientation[1].x) * size.y,
		fabsf(orientation[0].y) * size.x + fabsf(orientation[1].y) * size.y
	};
	auto aabb = Aabb{ position - sizeDifference, position + sizeDifference };
	return aabb;
}
