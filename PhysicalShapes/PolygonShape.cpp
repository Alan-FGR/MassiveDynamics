#include "PolygonShape.h"

#include "Common/MathExtras.h"
#include "Common/PhysicsData.h"

std::vector<vec2> BoxDimensionsToVerts(const real width, const real height)
{
	const real halfWidth = width / 2.f;
	const real halfHeight = height / 2.f;

	return {
		vec2(halfWidth, -halfHeight),
		vec2(halfWidth, halfHeight),
		vec2(-halfWidth, halfHeight),
		vec2(-halfWidth, -halfHeight),
	};
}

PolygonShape::PolygonShape(const SurfaceMaterial& material, const real boxWidth, const real boxHeight, const real radius)
	: PhysicalShape(material),
	radius(radius),
	transformedCenter(zero<vec2>)
{
	auto verts = BoxDimensionsToVerts(boxWidth, boxHeight);

	auto count = verts.size();
	for (size_t i = 0; i < count; i++)
	{
		vec2 a = verts[(i - 1 + count) % count];
		vec2 b = verts[i];
		vec2 n = glm::normalize(reversePerp(sub(b, a)));

		SplittingLine nb{};
		nb.point = b;
		nb.normal = n;

		localLines.emplace_back(nb);
		transformedLines.push_back({});
	}
}