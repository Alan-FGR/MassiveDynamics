/* MIT-LICENSED CODE ADAPTED FROM THE ORIGINAL WORK BY SCOTT LEMBCKE AND HOWLING MOON SOFTWARE: Copyright (c) 2013 Scott Lembcke and Howling Moon Software
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */

#include "CollisionUtils.h"

#include "Common/MathTypes.h"
#include "Utils/SolverUtils.h"
#include "Collision/Separation.h"
#include "PhysicalShapes/PhysicalShape.h"
#include "PhysicalShapes/PolygonShape.h"
#include "PhysicalShapes/CircleShape.h"
#include "PhysicalShapes/SegmentShape.h"

constexpr int MAX_GJK_ITERATIONS = 16;
constexpr real CPFLOAT_MIN = FLT_MIN;

static bool cpCheckPointGreater(const vec2 a, const vec2 b, const vec2 c)
{
	return (b.y - a.y) * (a.x + b.x - 2 * c.x) > (b.x - a.x) * (a.y + b.y - 2 * c.y);
}

static bool cpCheckAxis(vec2 v0, vec2 v1, vec2 p, vec2 n)
{
	return dot(p, n) <= glm::max(dot(v0, n), dot(v1, n));
}

//MARK: Support Points and Edges:

// Support points are the maximal points on a shape's perimeter along a certain axis.
// The GJK and EPA algorithms use support points to iteratively sample the surface of the two shapes' minkowski difference.

static int PolySupportPointIndex(const int count, const SplittingLine *planes, const vec2 n)
{
	real max = -INFINITY;
	int index = 0;
	
	for(int i=0; i<count; i++){
		vec2 v = planes[i].point;
		const real d = dot(v, n);
		if(d > max){
			max = d;
			index = i;
		}
	}
	
	return index;
}

typedef vec2 (*SupportPointFunc)(const PhysicalShape* shape, const vec2 n);

static vec2 CircleSupportPoint(const CircleShape* circle, const vec2 n)
{
	return circle->transformedOffset;
}

static vec2 SegmentSupportPoint(const SegmentShape* seg, const vec2 n)
{
	if(dot(seg->transformedA, n) > dot(seg->transformedB, n)){
		return (seg->transformedA);
	}
	return (seg->transformedB);
}

static vec2 PolySupportPoint(const PolygonShape* poly, const vec2 n)
{
	const int count = poly->localLines.size();

	auto& planes = poly->transformedLines;
	auto& dst = poly->transformedLines;
	auto& src = poly->localLines;
	const int i = PolySupportPointIndex(count, planes.data(), n);
	return (planes[i].point);
}

// A point on the surface of two shape's minkowski difference.
struct MinkowskiPoint {
	// Cache the two original support points.
	vec2 a, b;
	// b - a
	vec2 ab;
};

static MinkowskiPoint MinkowskiPointNew(const vec2 a, const vec2 b)
{
	const MinkowskiPoint point = {a, b, sub(b, a)};
	return point;
}

struct SupportContext {
	const PhysicalShape *shape1, *shape2;
	SupportPointFunc func1, func2;
};

// Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
static MinkowskiPoint Support(const SupportContext *ctx, const vec2 n)
{
	const vec2 a = ctx->func1(ctx->shape1, neg(n));
	const vec2 b = ctx->func2(ctx->shape2, n);
	return MinkowskiPointNew(a, b);
}

struct EdgePoint {
	vec2 p;
	// Keep a hash value for Chipmunk's collision hashing mechanism.
	int hash;
};

// Support edges are the edges of a polygon or segment shape that are in contact.
struct Edge {
	EdgePoint a, b;
	real r;
	vec2 n;
};

static Edge SupportEdgeForPoly(const PolygonShape *poly, const vec2 n)
{
	const int count = poly->localLines.size();

	auto& planes = poly->transformedLines;
	auto& dst = poly->transformedLines;
	auto& src = poly->localLines;
	const int i1 = PolySupportPointIndex(count, planes.data(), n);
	
	// TODO: get rid of mod eventually, very expensive on ARM
	const int i0 = (i1 - 1 + count)%count;
	const int i2 = (i1 + 1)%count;

	const uintptr_t hashid = poly->hashId;
	if(dot(n, planes[i1].normal) > dot(n, planes[i2].normal)){
		const Edge edge = {{planes[i0].point, orderIndependentHash<int>(hashid, i0)}, {planes[i1].point, orderIndependentHash<int>(hashid, i1)}, poly->radius, planes[i1].normal };
		return edge;
	}
	else
	{
		const Edge edge = { {planes[i1].point, orderIndependentHash<int>(hashid, i1)}, {planes[i2].point, orderIndependentHash<int>(hashid, i2)}, poly->radius, planes[i2].normal };
		return edge;
	}
}

static Edge SupportEdgeForSegment(const SegmentShape* seg, const vec2 n)
{
	const uintptr_t hashid = seg->hashId;
	if (dot(seg->transformedNormal, n) > 0.0)
	{
		const Edge edge = { {seg->transformedA, orderIndependentHash<int>(hashid, 0)}, {seg->transformedB, orderIndependentHash<int>(hashid, 1)}, seg->radius, seg->transformedNormal };
		return edge;
	}
	else
	{
		const Edge edge = { {seg->transformedB, orderIndependentHash<int>(hashid, 1)}, {seg->transformedA, orderIndependentHash<int>(hashid, 0)}, seg->radius, neg(seg->transformedNormal) };
		return edge;
	}
}

// Find the closest p(t) to (0, 0) where p(t) = a*(1-t)/2 + b*(1+t)/2
// The range for t is [-1, 1] to avoid cpCollision::floating point issues if the parameters are swapped.
static real ClosestT(const vec2 a, const vec2 b)
{
	const vec2 delta = sub(b, a);
	return -glm::clamp(dot(delta, add(a, b)) / (length2(delta) + CPFLOAT_MIN), -1.0f, 1.0f);
}

// Basically the same as cpvlerp(), except t = [-1, 1]
static vec2 LerpT(const vec2 a, const vec2 b, const real t)
{
	const real ht = 0.5f * t;
	return add(mul(a, 0.5f - ht), mul(b, 0.5f + ht));
}

// Closest points on the surface of two shapes.
struct ClosestPoints
{
	// Surface points in absolute coordinates.
	vec2 a, b;
	// Minimum separating axis of the two shapes.
	vec2 n;
	// Signed distance between the points.
	real d;
};

// Calculate the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
static ClosestPoints ClosestPointsNew(const MinkowskiPoint v0, const MinkowskiPoint v1)
{
	// Find the closest p(t) on the minkowski difference to (0, 0)
	real t = ClosestT(v0.ab, v1.ab);
	vec2 p = LerpT(v0.ab, v1.ab, t);

	// Interpolate the original support points using the same 't' value as above.
	// This gives you the closest surface points in absolute coordinates. NEAT!
	vec2 pa = LerpT(v0.a, v1.a, t);
	vec2 pb = LerpT(v0.b, v1.b, t);

	// First try calculating the MSA from the minkowski difference edge.
	// This gives us a nice, accurate MSA when the surfaces are close together.
	vec2 delta = sub(v1.ab, v0.ab);
	vec2 n = normalize(reversePerp(delta));
	real d = dot(n, p);

	if (d <= 0.0f || (-1.0f < t && t < 1.0f))
	{
		// If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
		ClosestPoints points = { pa, pb, n, d };
		return points;
	}
	else
	{
		// Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
		real d2 = length(p);
		vec2 n2 = mul(p, 1.0f / (d2 + CPFLOAT_MIN));

		ClosestPoints points = { pa, pb, n2, d2 };
		return points;
	}
}

static real ClosestDist(const vec2 v0, const vec2 v1)
{
	return length2(LerpT(v0, v1, ClosestT(v0, v1)));
}

//MARK: GJK Functions.

// Recursive implementation of the GJK loop.
static ClosestPoints GJKRecurse(const SupportContext* ctx, const MinkowskiPoint v0, const MinkowskiPoint v1, const int iteration)
{
	if (iteration > MAX_GJK_ITERATIONS)
		return ClosestPointsNew(v0, v1);

	if (cpCheckPointGreater(v1.ab, v0.ab, zero<vec2>))
		// Origin is behind axis. Flip and try again.
		return GJKRecurse(ctx, v1, v0, iteration);

	real t = ClosestT(v0.ab, v1.ab);
	vec2 n = (-1.0f < t && t < 1.0f ? perp(sub(v1.ab, v0.ab)) : neg(LerpT(v0.ab, v1.ab, t)));
	MinkowskiPoint p = Support(ctx, n);

	if (cpCheckPointGreater(p.ab, v0.ab, zero<vec2>) && cpCheckPointGreater(v1.ab, p.ab, zero<vec2>))
		// The triangle v0, p, v1 contains the origin. Use EPA to find the MSA. TODO EPA
		return { v0.a, v0.b, normalize(v0.ab), 0.f };

	if (cpCheckAxis(v0.ab, v1.ab, p.ab, n))
		// The edge v0, v1 that we already have is the closest to (0, 0) since p was not closer.
		return ClosestPointsNew(v0, v1);

	// p was closer to the origin than our existing edge.
	// Need to figure out which existing point to drop.
	if (ClosestDist(v0.ab, p.ab) < ClosestDist(p.ab, v1.ab))
		return GJKRecurse(ctx, v0, p, iteration + 1);

	return GJKRecurse(ctx, p, v1, iteration + 1);
}

// Find the closest points between two shapes using the GJK algorithm.
static ClosestPoints GJK(const SupportContext* ctx, const vec2& initialAxis)
{
	MinkowskiPoint v0, v1;

	v0 = Support(ctx, initialAxis);
	v1 = Support(ctx, neg(initialAxis));

	const ClosestPoints points = GJKRecurse(ctx, v0, v1, 1);
	return points;
}

//MARK: Contact Clipping

// Given two support edges, find contact point pairs on their surfaces.
static void ContactPoints(const Edge e1, const Edge e2, const ClosestPoints points, Manifold& contacts)
{
	real mindist = e1.r + e2.r;
	if (points.d <= mindist)
	{
		vec2 n = contacts.normal = points.n;

		// Distances along the axis parallel to n
		real d_e1_a = perpDot(e1.a.p, n);
		real d_e1_b = perpDot(e1.b.p, n);
		real d_e2_a = perpDot(e2.a.p, n);
		real d_e2_b = perpDot(e2.b.p, n);

		// TODO + min isn't a complete fix.
		real e1_denom = 1.0f / (d_e1_b - d_e1_a + CPFLOAT_MIN);
		real e2_denom = 1.0f / (d_e2_b - d_e2_a + CPFLOAT_MIN);

		// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
		// Compare the projected points to the collision normal to see if the shapes overlap there.
		{
			vec2 p1 = add(mul(n, e1.r), mix(e1.a.p, e1.b.p, clamp01((d_e2_b - d_e1_a) * e1_denom)));
			vec2 p2 = add(mul(n, -e2.r), mix(e2.a.p, e2.b.p, clamp01((d_e1_a - d_e2_a) * e2_denom)));
			real dist = dot(sub(p2, p1), n);
			if (dist <= 0.0f)
			{
				uintptr_t hash_1a2b = orderIndependentHash<int>(e1.a.hash, e2.b.hash);
				contacts.addWorldSpaceContact(hash_1a2b, p1, p2);
			}
		}
		{
			vec2 p1 = add(mul(n, e1.r), mix(e1.a.p, e1.b.p, clamp01((d_e2_a - d_e1_a) * e1_denom)));
			vec2 p2 = add(mul(n, -e2.r), mix(e2.a.p, e2.b.p, clamp01((d_e1_b - d_e2_a) * e2_denom)));
			real dist = dot(sub(p2, p1), n);
			if (dist <= 0.0f)
			{
				uintptr_t hash_1b2a = orderIndependentHash<int>(e1.b.hash, e2.a.hash);
				contacts.addWorldSpaceContact(hash_1b2a, p1, p2);
			}
		}
	}
}

//MARK: Collision Functions

// Collide circle shapes.
void CollisionUtils::CircleToCircle(const CircleShape* c1, const CircleShape* c2, Manifold& contacts)
{
	const real mindist = c1->radius + c2->radius;
	const vec2 delta = sub(c2->transformedOffset, c1->transformedOffset);
	const real distsq = length2(delta);

	if (distsq < mindist * mindist) {
		const real dist = sqrt(distsq);
		const vec2 n = contacts.normal = (dist ? mul(delta, 1.0f / dist) : vec2(1.0f, 0.0f));
		contacts.addWorldSpaceContact(0, add(c1->transformedOffset, mul(n, c1->radius)), add(c2->transformedOffset, mul(n, -c2->radius)));
	}
}

void CollisionUtils::CircleToSegment(const CircleShape* circle, const SegmentShape* segment, Manifold& contacts)
{
	const vec2 seg_a = segment->transformedA;
	const vec2 seg_b = segment->transformedB;
	const vec2 center = circle->transformedOffset;

	// Find the closest point on the segment to the circle.
	const vec2 seg_delta = sub(seg_b, seg_a);
	const real closest_t = clamp01(dot(seg_delta, sub(center, seg_a)) / length2(seg_delta));
	const vec2 closest = add(seg_a, mul(seg_delta, closest_t));

	// Compare the radii of the two shapes to see if they are colliding.
	const real mindist = circle->radius + segment->radius;
	const vec2 delta = sub(closest, center);
	const real distsq = length2(delta);
	if (distsq < mindist * mindist) {
		const real dist = sqrt(distsq);
		// Handle coincident shapes as gracefully as possible.
		vec2 n = contacts.normal = (dist ? mul(delta, 1.0f / dist) : segment->transformedNormal);

		// Reject endcap collisions if tangents are provided.
		/*vec2 rot = (segment->body->transform.getRotationCosSin());
		if ((closest_t != 0.0f || dot(n, rotateBy(segment->a_tangent, rot)) >= 0.0) &&
			(closest_t != 1.0f || dot(n, rotateBy(segment->b_tangent, rot)) >= 0.0)
			)
			contacts.addContact(0, add(center, mul(n, circle->radius)), add(closest, mul(n, -segment->radius)));*/
	}
}

void CollisionUtils::SegmentToSegment(const SegmentShape* seg1, const SegmentShape* seg2, Manifold& contacts)
{
	const SupportContext context = { (PhysicalShape*)seg1, (PhysicalShape*)seg2, (SupportPointFunc)SegmentSupportPoint, (SupportPointFunc)SegmentSupportPoint };

	const auto dir = perp(sub(mix(seg1->transformedA, seg1->transformedB, 0.5), mix(seg2->transformedA, seg2->transformedB, 0.5)));

	const ClosestPoints points = GJK(&context, dir);

	vec2 n = points.n;
	//vec2 rot1 = (seg1->body->transform.getRotationCosSin());
	//vec2 rot2 = (seg2->body->transform.getRotationCosSin());

	//// If the closest points are nearer than the sum of the radii...
	//if (points.d <= (seg1->radius + seg2->radius) && (
	//		// Reject endcap collisions if tangents are provided.
	//		(!equal(points.a, seg1->transformedA) || dot(n, rotateBy(seg1->a_tangent, rot1)) <= 0.0) &&
	//		(!equal(points.a, seg1->transformedB) || dot(n, rotateBy(seg1->b_tangent, rot1)) <= 0.0) &&
	//		(!equal(points.b, seg2->transformedA) || dot(n, rotateBy(seg2->a_tangent, rot2)) >= 0.0) &&
	//		(!equal(points.b, seg2->transformedB) || dot(n, rotateBy(seg2->b_tangent, rot2)) >= 0.0)))
	//	ContactPoints(SupportEdgeForSegment(seg1, n), SupportEdgeForSegment(seg2, neg(n)), points, contacts);
}

void CollisionUtils::PolyToPoly(const PolygonShape* poly1, const PolygonShape* poly2, Manifold& contacts)
{
	const SupportContext context = { (PhysicalShape*)poly1, (PhysicalShape*)poly2, (SupportPointFunc)PolySupportPoint, (SupportPointFunc)PolySupportPoint };

	const auto dir = perp(sub(poly1->transformedCenter, poly2->transformedCenter));

	const ClosestPoints points = GJK(&context, dir);

	// If the closest points are nearer than the sum of the radii...
	if (points.d - poly1->radius - poly2->radius <= 0.0) {
		ContactPoints(SupportEdgeForPoly(poly1, points.n), SupportEdgeForPoly(poly2, neg(points.n)), points, contacts);
	}
}

void CollisionUtils::ArbiterPreStep(
	Positional* positionalsPool, InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool,
	Arbiter& arb, real dt, real slop, real bias)
{
	auto& pa = physicalsPool[arb.bodyA];
	auto& pb = physicalsPool[arb.bodyB];

	auto& da = dynamicalsPool[arb.bodyA];
	auto& db = dynamicalsPool[arb.bodyB];

	auto& ta = positionalsPool[arb.bodyA];
	auto& tb = positionalsPool[arb.bodyB];

	const vec2 n = arb.manifold.normal;
	const vec2 body_delta = sub(tb.position, ta.position);

	for (size_t i = 0; i < arb.manifold.size(); i++)
	{
		Contact& con = arb.manifold[i];

		// Calculate the mass normal and mass tangent.
		con.normalMass = 1.0f / SolverUtils::k_scalar(pa, pb, con.offsetA, con.offsetB, n);
		con.tangentMass = 1.0f / SolverUtils::k_scalar(pa, pb, con.offsetA, con.offsetB, perp(n));

		// Calculate the target bias velocity.
		const real dist = dot(add(sub(con.offsetB, con.offsetA), body_delta), n);
		con.bias = -bias * glm::min(0.0f, dist + slop) / dt;
		con.cachedBias = 0.0f;

		// Calculate the target bounce velocity.
		con.bounce = SolverUtils::normal_relative_velocity(da, db, con.offsetA, con.offsetB, n) * arb.surfaceProperties.material.elasticity;
	}
}

void CollisionUtils::ArbiterApplyImpulse(InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool, OverlapCorrectors* correctorsPool, Arbiter& arb)
{
	auto& pa = physicalsPool[arb.bodyA];
	auto& pb = physicalsPool[arb.bodyB];

	auto& da = dynamicalsPool[arb.bodyA];
	auto& db = dynamicalsPool[arb.bodyB];

	auto& ca = correctorsPool[arb.bodyA];
	auto& cb = correctorsPool[arb.bodyB];

	const vec2 n = arb.manifold.normal;
	const vec2 surface_vr = arb.surfaceProperties.velocity;
	const real friction = arb.surfaceProperties.material.friction;

	for (size_t i = 0; i < arb.manifold.size(); i++)
	{
		auto& con = arb.manifold[i];
		const real nMass = con.normalMass;
		const vec2 r1 = con.offsetA;
		const vec2 r2 = con.offsetB;

		const vec2 vb1 = add(ca.linearCorrection, mul(perp(r1), ca.angularCorrection));
		const vec2 vb2 = add(cb.linearCorrection, mul(perp(r2), cb.angularCorrection));
		vec2 vr = add(SolverUtils::relative_velocity(da, db, r1, r2), surface_vr);

		const real vbn = dot(sub(vb2, vb1), n);
		const real vrn = dot(vr, n);
		const real vrt = dot(vr, perp(n));

		const real jbn = (con.bias - vbn) * nMass;
		const real jbnOld = con.cachedBias;
		con.cachedBias = glm::max(jbnOld + jbn, 0.0f);

		const real jn = -(con.bounce + vrn) * nMass;
		const real jnOld = con.normalAcceleration;
		con.normalAcceleration = glm::max(jnOld + jn, 0.0f);

		const real jtMax = friction * con.normalAcceleration;
		const real jt = -vrt * con.tangentMass;
		const real jtOld = con.tangentAcceleration;
		con.tangentAcceleration = glm::clamp(jtOld + jt, -jtMax, jtMax);

		// these are penetration correction - maybe could correct position then?
		SolverUtils::apply_bias_impulses(pa, pb, r1, r2, mul(n, con.cachedBias - jbnOld), ca, cb);
		SolverUtils::apply_impulses(pa, pb, r1, r2, rotateBy(n, vec2(con.normalAcceleration - jnOld, con.tangentAcceleration - jtOld)), da, db);
	}
}

void CollisionUtils::ArbiterApplyCachedImpulse(InversePhysicalProps* physicalsPool, Dynamical* dynamicalsPool, Arbiter& arb, real dtCoef)
{
	if (arb.state == ArbiterState::Initial) return;

	auto& pa = physicalsPool[arb.bodyA];
	auto& pb = physicalsPool[arb.bodyB];

	auto& da = dynamicalsPool[arb.bodyA];
	auto& db = dynamicalsPool[arb.bodyB];

	const vec2 n = arb.manifold.normal;

	for (size_t i = 0; i < arb.manifold.size(); i++)
	{
		const auto& con = arb.manifold[i];
		const vec2 j = rotateBy(n, vec2(con.normalAcceleration, con.tangentAcceleration));
		SolverUtils::apply_impulses(pa, pb, con.offsetA, con.offsetB, mul(j, dtCoef), da, db);
	}
}

void CollisionUtils::SegmentToPoly(const SegmentShape* seg, const PolygonShape* poly, Manifold& contacts)
{
	const SupportContext context = { (PhysicalShape*)seg, (PhysicalShape*)poly, (SupportPointFunc)SegmentSupportPoint, (SupportPointFunc)PolySupportPoint };

	const auto dir = perp(sub(mix(seg->transformedA, seg->transformedB, 0.5), poly->transformedCenter));

	const ClosestPoints points = GJK(&context, dir);

	vec2 n = points.n;
	//vec2 rot = (seg->body->transform.getRotationCosSin());

	//if (// If the closest points are nearer than the sum of the radii...
	//	points.d - seg->radius - poly->radius <= 0.0 && (
	//		// Reject endcap collisions if tangents are provided.
	//		(!equal(points.a, seg->transformedA) || dot(n, rotateBy(seg->a_tangent, rot)) <= 0.0) &&
	//		(!equal(points.a, seg->transformedB) || dot(n, rotateBy(seg->b_tangent, rot)) <= 0.0)))
	//	ContactPoints(SupportEdgeForSegment(seg, n), SupportEdgeForPoly(poly, neg(n)), points, contacts);
}

void CollisionUtils::CircleToPoly(const CircleShape* circle, const PolygonShape* poly, Manifold& contacts)
{
	const SupportContext context = { (PhysicalShape*)circle, (PhysicalShape*)poly, (SupportPointFunc)CircleSupportPoint, (SupportPointFunc)PolySupportPoint };

	const auto dir = perp(sub(circle->transformedOffset, poly->transformedCenter));

	const ClosestPoints points = GJK(&context, dir);

	// If the closest points are nearer than the sum of the radii...
	if (points.d <= circle->radius + poly->radius) {
		const vec2 n = contacts.normal = points.n;
		contacts.addWorldSpaceContact(0, add(points.a, mul(n, circle->radius)), add(points.b, mul(n, -poly->radius)));
	}
}
