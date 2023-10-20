#pragma once
#include "Common/MathExtras.h"
#include "PhysicalShape.h"

struct CircleShape final : PhysicalShape
{
	CircleShape(const SurfaceMaterial& material, real radius, const vec2& offset)
		: PhysicalShape(material),
		  radius(radius),
		  offset(offset)
	{
	}

	real radius;
	vec2 offset;
	vec2 transformedOffset{};

	[[nodiscard]] ShapeType shapeType() const override { return Circle; }
};
