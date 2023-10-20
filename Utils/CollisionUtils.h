#pragma once

#include "Collision/Separation.h"
// TODO shouldn't need these
#include "PhysicalShapes/CircleShape.h"
#include "PhysicalShapes/PolygonShape.h"
#include "PhysicalShapes/SegmentShape.h"

struct PolygonShape;
struct SegmentShape;
struct Manifold;
struct CircleShape;

namespace CollisionUtils
{
	// Collision tests
	void CircleToCircle(const CircleShape* c1, const CircleShape* c2, Manifold& contacts);
	void CircleToSegment(const CircleShape* circle, const SegmentShape* segment, Manifold& contacts);
	void CircleToPoly(const CircleShape* circle, const PolygonShape* poly, Manifold& contacts);
	void SegmentToSegment(const SegmentShape* seg1, const SegmentShape* seg2, Manifold& contacts);
	void SegmentToPoly(const SegmentShape* seg, const PolygonShape* poly, Manifold& contacts);
	void PolyToPoly(const PolygonShape* poly1, const PolygonShape* poly2, Manifold& contacts);

	// Collision resolution
	void ArbiterPreStep(Positional* positionalsPool, InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool, Arbiter
	                      & arb, real dt, real slop, real
	                      bias);
	void ArbiterApplyImpulse(InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool, OverlapCorrectors* correctorsPool, Arbiter& arb);
	void ArbiterApplyCachedImpulse(InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool, Arbiter& arb, real dtCoef);
}
