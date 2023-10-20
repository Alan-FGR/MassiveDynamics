#pragma once
#include "Common/MathTypes.h"
#include "PhysicalShapes/CircleShape.h"
#include "PhysicalShapes/PolygonShape.h"
#include "PhysicalShapes/SegmentShape.h"

namespace GeometryUtils
{
	// Moments
	real MomentForCircle(real m, real r1, real r2, vec2 offset);
	real MomentForSegment(real m, vec2 a, vec2 b, real r);
	real MomentForPoly(real m, int count, const vec2* verts, vec2 offset, real r);
	real MomentForBox(real m, real width, real height);

	// AABBs
	Aabb CalculateSegmentAABB(SegmentShape* seg);// TODO ref
	Aabb CalculatePolyAABB(PolygonShape* poly);
	Aabb CalculateCircleAABB(CircleShape& circle);
}
