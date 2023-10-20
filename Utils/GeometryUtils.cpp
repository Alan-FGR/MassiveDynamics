/* MIT-LICENSED CODE ADAPTED FROM THE ORIGINAL WORK BY SCOTT LEMBCKE AND HOWLING MOON SOFTWARE: Copyright (c) 2013 Scott Lembcke and Howling Moon Software
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */
#include "GeometryUtils.h"

#include "Common/MathExtras.h"

real GeometryUtils::MomentForCircle(real m, real r1, real r2, vec2 offset)
{
	return m * (0.5f * (r1 * r1 + r2 * r2) + length2(offset));
}

real GeometryUtils::MomentForSegment(real m, vec2 a, vec2 b, real r)
{
	vec2 offset = mix(a, b, 0.5f);

	// This approximates the shape as a box for rounded segments, but it's quite close.
	real length = distance(b, a) + 2.0f * r;
	return m * ((length * length + 4.0f * r * r) / 12.0f + length2(offset));
}

real GeometryUtils::MomentForBox(real m, real width, real height)
{
	return m * (width * width + height * height) / 12.0f;
}

real GeometryUtils::MomentForPoly(real m, int count, const vec2* verts, vec2 offset, real r)
{
	// TODO account for radius.
	if (count == 2) return MomentForSegment(m, verts[0], verts[1], 0.0f);

	real sum1 = 0.0f;
	real sum2 = 0.0f;
	for (int i = 0; i < count; i++)
	{
		vec2 v1 = add(verts[i], offset);
		vec2 v2 = add(verts[(i + 1) % count], offset);

		real a = perpDot(v2, v1);
		real b = dot(v1, v1) + dot(v1, v2) + dot(v2, v2);

		sum1 += a * b;
		sum2 += a;
	}

	return (m * sum1) / (6.0f * sum2);
}

Aabb GeometryUtils::CalculateSegmentAABB(SegmentShape* seg)
{
	real l, r, b, t;

	if (seg->transformedA.x < seg->transformedB.x)
	{
		l = seg->transformedA.x;
		r = seg->transformedB.x;
	}
	else
	{
		l = seg->transformedB.x;
		r = seg->transformedA.x;
	}

	if (seg->transformedA.y < seg->transformedB.y)
	{
		b = seg->transformedA.y;
		t = seg->transformedB.y;
	}
	else
	{
		b = seg->transformedB.y;
		t = seg->transformedA.y;
	}

	real rad = seg->radius;
	return Aabb(l - rad, b - rad, r + rad, t + rad);
}

Aabb GeometryUtils::CalculateCircleAABB(CircleShape& circle)
{
	return Aabb(
		circle.transformedOffset.x - circle.radius, 
		circle.transformedOffset.y - circle.radius, 
		circle.transformedOffset.x + circle.radius, 
		circle.transformedOffset.y + circle.radius);
}

Aabb GeometryUtils::CalculatePolyAABB(PolygonShape* poly)
{
	int count = poly->transformedLines.size();

	auto& dst = poly->transformedLines;

	real l = INFINITY, r = -(real)INFINITY;
	real b = INFINITY, t = -(real)INFINITY;

	for (size_t i = 0; i < count; i++)
	{
		vec2 v = dst[i].point;
		l = glm::min(l, v.x);
		r = glm::max(r, v.x);
		b = glm::min(b, v.y);
		t = glm::max(t, v.y);
	}

	poly->transformedCenter = mix(vec2(l, b), vec2(r, t), 0.5); // TODO and update TC - but we don't want to do this here

	real radius = poly->radius;
	return Aabb(l - radius, b - radius, r + radius, t + radius);
}