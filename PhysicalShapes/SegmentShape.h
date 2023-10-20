#pragma once
#include "Common/MathExtras.h"
#include "PhysicalShape.h"

struct SegmentShape : PhysicalShape
{
	SegmentShape(const SurfaceMaterial& material, real radius, const vec2& pointA, const vec2& pointB)
		: PhysicalShape(material),
		  radius(radius),
		  pointA(pointA),
		  pointB(pointB)
	{
		normal = reversePerp(normalize(sub(pointA, pointB)));
	}

	real radius;

	vec2 pointA;
	vec2 pointB;
	vec2 normal;

	vec2 transformedA{};
	vec2 transformedB{};
	vec2 transformedNormal{};

	vec2 tangentA{};
	vec2 tangentB{};

	[[nodiscard]] ShapeType shapeType() const override { return Segment; }
};
