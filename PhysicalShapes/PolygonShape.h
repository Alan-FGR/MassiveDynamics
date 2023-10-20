#pragma once
#include <vector>

#include "PhysicalShape.h"

struct SplittingLine {
	vec2 point;
	vec2 normal;
};

struct PolygonShape : PhysicalShape {

	PolygonShape(const SurfaceMaterial& material, real boxWidth, real boxHeight, real radius);

	real radius;
	vec2 transformedCenter; // TODO this should be removed

	std::vector<SplittingLine> localLines;
	std::vector<SplittingLine> transformedLines;

	[[nodiscard]] ShapeType shapeType() const override { return Polygon; }
};
