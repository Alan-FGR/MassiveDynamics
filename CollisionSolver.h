#pragma once

#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/component_wise.hpp"
#include "glm/gtx/norm.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_set>

#include "AvxVector.h"
#include "Data.h"
#include "Shape.h"
#include "Transform.h"

#include "Contacts.h"

using namespace glm;

struct BroadphaseEntry
{
	Aabb originalAabb;
	int originalIndex;
};

struct _intPairHasher {
	int operator()(const std::pair<int, int>& pair) const {
		return std::hash<int>()(pair.first) * 31 + std::hash<int>()(pair.second);
	}
};

namespace CollisionSolver {
	std::vector<BroadphaseEntry> SortAabbs(const std::vector<Aabb>& aabbs);
	void AddContactPointToBuffer(std::vector<ContactPoint>& newPointsBuffer, int& pointCountToIncrement, ContactPoint& newPoint);
	int CalculateContactPoints(const vec2& size, const vec2& axis, vec2* supportPoints, const vec2& pos, const mat2x2& rot);
	void GenerateContacts(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2, vec2& separatingAxis, std::vector<ContactPoint>& newPointsBuffer, int& pointCountToIncrement);
	bool CalculateSatSimd(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2, vec2& axis);
	std::vector<ContactPoint> CreateOverlaps(std::vector<Overlap>& overlaps, const AvxVector<DynamicProperties>& dynamicProperties, const AvxVector<Shape>&shapeData);
	std::vector<Overlap> CreatePairs(std::vector<BroadphaseEntry>& sortedAabbs);

	inline vec2 GetPerpendicular(const vec2& value)
	{
		return vec2{ -value.y, value.x };
	}

	// TODO ret vec2
	inline void IntersectRayPlane(const vec2& point, const vec2& planePoint, const vec2& planeNormal, const vec2& projectionDirection, vec2& output)
	{
		const auto multiplier = 1.f / compAdd(projectionDirection * planeNormal);
		output = point + projectionDirection * (planePoint * planeNormal - point * planeNormal) * multiplier;
	}
}
