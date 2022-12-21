export module PhysicsWorld;

#include "glm/glm.hpp";
#include "glm/ext.hpp";
#include "glm/gtx/component_wise.hpp";
#include "glm/gtx/norm.hpp";
//import "glm/gtx/intersect.hpp";

import <algorithm>;
import <memory>;
import <vector>;
import <unordered_set>;

import TypeAliases;
import AvxVector;
import Transform;
import Dynamics;
import Shape;

using namespace glm;

export struct BroadphaseEntry
{
	Aabb originalAabb;
	int originalIndex;
};

export class PhysicsWorld
{

public:
	DataPools pools;
	PhysicsWorld()
	{

	}

	PhysicsWorld(const PhysicsWorld& other) noexcept = delete;
	PhysicsWorld(PhysicsWorld&& other) noexcept = delete;
	PhysicsWorld& operator=(const PhysicsWorld& other) noexcept = delete;
	PhysicsWorld& operator=(PhysicsWorld&& other) noexcept = delete;

	~PhysicsWorld()
	{

	}

	int EntityCount()
	{
		return pools.dynamicProperties.size();
	}

	void ForEntity(auto func)
	{
		for (int i = 0; i < pools.dynamicProperties.size(); ++i)
		{
			func(pools.entities[i],
				pools.dynamicProperties[i],
				pools.shapeSize[i],
				pools.shapeData[i]
				//TODO pass ptrs to other stuff that makes sense (e.g. shape)
			);
		}
	}

	EntityId AddEntity(Transform& transform, ShapeType shapeType, bool isKinematic = false)
	{
		auto density = 1e-5f;

		//TODO LP memcpy
		DynamicProperties properties;
		properties.position = transform.position;
		properties.orientation = transform.orientation;

		//TODO calc mass from transform and shape
		auto calculatedMassInverse = 0.5f;
		auto calculatedInertiaInverse = 0.5f;

		properties.massInverse = isKinematic ? 0 : calculatedMassInverse;
		properties.inertiaInverse = isKinematic ? 0 : calculatedInertiaInverse;

		return pools.AddSimulationEntry(properties, transform.scale, shapeType);
	}
	
	void Update(float iterationTime)
	{

		// COLLISION

		// Update AABBs

		// Sort AABBs

		// Calculate manifolds (contact points)

		// Create joints for contact points

		// IMPULSE SOLVING

		// Solve joints

		// INTEGRATION

		// Apply velocities

		// Apply positions

	}

	std::vector<Aabb> UpdateAABBs()
	{
		auto poolSize = pools.dynamicProperties.size();

		std::vector<Aabb> data;

		for (int i = 0; i < poolSize; ++i)
		{
			data.emplace_back(Aabb::CalculateFrom(
				pools.dynamicProperties[i].position,
				pools.dynamicProperties[i].orientation,
				pools.shapeSize[i]
			));
		}
		
		return data; // RVO
	}

	std::vector<BroadphaseEntry> SortAabbs(const std::vector<Aabb>& aabbs)
	{
		auto poolSize = aabbs.size(); // TODO

		// Calculate broad phase pairs
		std::vector<BroadphaseEntry> broadphaseData;
		{
			// TODO SoA, prealloc and cache these
			avx_vector<std::pair<float, int>> broadphaseDataPairs;
			avx_vector<std::pair<float, int>> broadphaseDataPairsSorted;

			// Populate pairs to be sorted by aabb max X
			for (int i = 0; i < poolSize; ++i)
			{
				const auto& aabb = aabbs[i];
				std::pair<float, int> pair{ aabb.max.x, i }; // Pairs hold their index
				broadphaseDataPairs.emplace_back(pair);
			}

			broadphaseDataPairsSorted.resize(broadphaseDataPairs.size());

			std::partial_sort_copy(
				broadphaseDataPairs.data(), broadphaseDataPairs.data() + poolSize,
				broadphaseDataPairsSorted.data(), broadphaseDataPairsSorted.data() + poolSize
			);

			for (int i = 0; i < poolSize; ++i)
			{
				auto originalIndex = broadphaseDataPairsSorted[i].second;
				const auto& aabb = aabbs[originalIndex];

				BroadphaseEntry e = { aabb, originalIndex };
				broadphaseData.emplace_back(e);
			}
		}
		return broadphaseData; // RVO
	}

	// TODO move all this stuff to proper places
	static constexpr int MAX_POINTS_PER_OVERLAP = 2;

	struct _intPairHasher {
		int operator()(const std::pair<int, int>& pair) const {
			return std::hash<int>()(pair.first) * 31 + std::hash<int>()(pair.second);
		}
	};

	// An overlap represented by a number of contact points
	class Overlap
	{
		int pointsCount;

	public:

		const int firstEntityIndex, secondEntityIndex, pointsStartIndex;
		
		Overlap(int firstEntityIndex, int secondEntityIndex, int pointsRangeIndex) :
			firstEntityIndex(firstEntityIndex),
			secondEntityIndex(secondEntityIndex),
			pointsStartIndex(pointsRangeIndex * MAX_POINTS_PER_OVERLAP),
			pointsCount(0)
		{ }
		
		bool TryAddPoint() // maybe this could be clearer by being explicit? e.g.: if HasSpace AddPoint
		{
			//if (pointsCount < MAX_POINTS_PER_OVERLAP) // TODO
			{
				pointsCount++;
				return true;
			}
			return false;
		}

		int GetPointCount() const { return pointsCount; }
		void ResetPointCount() { pointsCount = 0; }
	};

	struct ContactPoint
	{
		// TODO this is not really "local" as it's just transposed, not rotated or scaled... maybe rename?
		vec2 localPointA, localPointB, normal;
		bool isMerged, isNew;

		ContactPoint() = default;
		ContactPoint(const ContactPoint& other) = default;
		ContactPoint(ContactPoint&& other) noexcept = default;
		ContactPoint& operator=(const ContactPoint& other) = default;
		ContactPoint& operator=(ContactPoint&& other) noexcept = default;

		ContactPoint(const vec2& bodyPositionA, const vec2& worldPointA, const vec2& bodyPositionB, const vec2& worldPointB, const vec2& normal) :
			localPointA(worldPointA - bodyPositionA),
			localPointB(worldPointB - bodyPositionB),
			normal(normal),
			isMerged(false),
			isNew(true)
		{}
	};

	std::vector<Overlap> CreatePairs(std::vector<BroadphaseEntry> sortedAabbs)
	{
		auto poolSize = sortedAabbs.size(); // TODO

		// TODO we could also build all intersections on one axis, then solve the other in multiple threads

		// TODO one collection per thread then join all
		std::unordered_set<std::pair<int, int>, _intPairHasher> pairs;

		for (int i = 0; i < poolSize; ++i)
		{
			const BroadphaseEntry& currentEntry = sortedAabbs[i];
			
			for (int nextEntryIndex = i + 1; nextEntryIndex < poolSize; nextEntryIndex++)
			{
				const BroadphaseEntry& nextEntry = sortedAabbs[nextEntryIndex];

				if (currentEntry.originalAabb.max.x < nextEntry.originalAabb.min.x) continue;

				// TODO we could do a clever trick by subtracting min y and comparing to height difference
				// TODO it might be faster to remove duplicates at a later pass, use a different set here (robin probably)
				// TODO if MT, make this thread safe then
				// In compiler we trust... to make this branchless
				if (nextEntry.originalAabb.max.y > currentEntry.originalAabb.min.y &&
					nextEntry.originalAabb.min.y < currentEntry.originalAabb.max.y)
					pairs.emplace(std::make_pair(
						std::min(currentEntry.originalIndex, nextEntry.originalIndex),
						std::max(currentEntry.originalIndex, nextEntry.originalIndex)
					));
				
			}
		}

		std::vector<Overlap> overlaps;

		// TODO when MT, join all threads results
		int c = 0;
		for (const auto& pair : pairs)
		{
			overlaps.emplace_back(Overlap(pair.first, pair.second, c++));
		}

		return overlaps; // RVO
	}

	// TODO decide if we're going with A/B or 1/2 :P
	static bool CalculateSatSimd(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2, vec2& axis)
	{
		//TODO further parallelize this to AVX2 - i.e. compute multiple
		//TODO FMA could be used here in some operations

		const float* body1Orientation = &transform1.orientation[0].x;
		const float* body2Orientation = &transform2.orientation[0].x;
		const float* shape1Size = &shape1.size.x;
		const float* shape2Size = &shape2.size.x;

		vec2 distance = transform1.position - transform2.position;

		// TODO apparently shuffle is faster than permute? :(((((((((( if confirmed refactor!

		auto colAx = _mm_permute_ps(_mm_load_ps(body1Orientation), _MM_PERM_CCAA);
		auto colBx = _mm_permute_ps(_mm_load_ps(body2Orientation), _MM_PERM_CACA);

		auto colAy = _mm_permute_ps(_mm_load_ps(body1Orientation), _MM_PERM_DDBB);
		auto colBy = _mm_permute_ps(_mm_load_ps(body2Orientation), _MM_PERM_DBDB);

		auto innerProds = _mm_add_ps(_mm_mul_ps(colAx, colBx), _mm_mul_ps(colAy, colBy));

		auto absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
		auto absProds = _mm_and_ps(absMask, innerProds);

		auto colS1 = _mm_permute_ps(_mm_load_ps(shape1Size), _MM_PERM_AABA);
		auto colS2 = _mm_permute_ps(_mm_load_ps(shape2Size), _MM_PERM_BAAA);
		auto colD1 = _mm_permute_ps(absProds, _MM_PERM_BACA);
		auto colD2 = _mm_permute_ps(absProds, _MM_PERM_DCDB);

		auto colS3 = _mm_shuffle_ps(colS2, colS1, _MM_PERM_BBDD);

		auto rSums =
			_mm_add_ps(
				_mm_add_ps(
					_mm_mul_ps(colS2, colD1),
					_mm_mul_ps(colS3, colD2)),
				colS1);

		auto orsA = _mm_shuffle_ps(colAx, colBx, _MM_PERM_DCCA);
		auto orsB = _mm_shuffle_ps(colAy, colBy, _MM_PERM_BACA);

		auto dists0 = _mm_broadcast_ss(&distance[0]);
		auto dists1 = _mm_broadcast_ss(&distance[1]);

		auto rDists =
			_mm_sub_ps(
				_mm_and_ps(absMask,
					_mm_add_ps(
						_mm_mul_ps(orsA, dists0),
						_mm_mul_ps(orsB, dists1))),
				rSums);

		// faster way to check whether at least one float in rDists is >= 0
		//auto signMask = _mm_srli_epi32(_mm_set1_epi32(1), 31); // not sure what could go wrong passing 0x8000000 here?
		//auto allNegative = _mm_test_all_zeros(signMask, _mm_castps_si128(rDists));

		auto greaterThanZero = _mm_cmpgt_ps(rDists, _mm_setzero_ps());
		auto anyGreaterThanZero = _mm_movemask_ps(greaterThanZero);

		if (anyGreaterThanZero) return false;

		// at this point we need to get the horizontal max and index
		// but we're about to leave the routine so we just dump the vector to proceed

		float dists[4];
		_mm_storeu_ps(&dists[0], rDists);

		auto greatest = std::distance(&dists[0], std::max_element(&dists[0], &dists[4])); // TODO C++23 ranges

		switch (greatest)
		{
		case 0:
			axis = vec2{
				body1Orientation[0],
				body1Orientation[1] };
			break;
		case 1:
			axis = vec2{
				body1Orientation[2],
				body1Orientation[3] };
			break;
		case 2:
			axis = vec2{
				body2Orientation[0],
				body2Orientation[1] };
			break;
		default:
			axis = vec2{
				body2Orientation[2],
				body2Orientation[3] };
			break;
		}

		return true;
	}

	static int CalculateContactPoints(const vec2& size, const vec2& axis, vec2* supportPoints, const vec2& pos, const mat2x2& rot)
	{
		auto distToEdgeMidPoint = min(abs(compAdd(axis * rot[0])), abs(compAdd(axis * rot[1])));

		if (distToEdgeMidPoint < 0.1f) // TODO magic number tweak
		{
			vec2& edgePoint1 = supportPoints[0];
			vec2& edgePoint2 = supportPoints[1];
			edgePoint1 = pos;
			edgePoint2 = pos;
			vec2 xSize = rot[0] * size.x;
			vec2 ySize = rot[1] * size.y;
			vec2 offset = vec2{ 0 };
			float xDiff = compAdd(axis * rot[0]);
			float yDiff = compAdd(axis * rot[1]);
			if (abs(xDiff) < abs(yDiff))
			{
				if (compAdd(axis * ySize) > 0.0f)
				{
					offset += ySize;
					edgePoint1 += xSize;
					edgePoint2 -= xSize;
				}
				else
				{
					offset -= ySize;
					edgePoint1 -= xSize;
					edgePoint2 += xSize;
				}
			}
			else
			{
				if (compAdd(axis * xSize) > 0.0f)
				{
					offset += xSize;
					edgePoint1 -= ySize;
					edgePoint2 += ySize;
				}
				else
				{
					offset -= xSize;
					edgePoint1 += ySize;
					edgePoint2 -= ySize;
				}
			}
			edgePoint1 += offset;
			edgePoint2 += offset;
			return 2;
		}
		else
		{
			vec2& edgepoint1 = supportPoints[0];
			vec2 xdim = rot[0] * size.x;
			vec2 ydim = rot[1] * size.y;
			float xsgn = compAdd(rot[0] * axis) < 0.0f ? -1.0f : 1.0f;
			float ysgn = compAdd(rot[1] * axis) < 0.0f ? -1.0f : 1.0f;
			edgepoint1 = pos + xsgn * xdim + ysgn * ydim;
			return 1;
		}
	}

	static void AddContactPointToBuffer(ContactPoint* pointsBuffer, int& pointCountToIncrement, ContactPoint& newPoint)
	{
		ContactPoint* closest = nullptr;
		auto smallestDepth = std::numeric_limits<float>::max();

		for (int pointIndex = 0; pointIndex < pointCountToIncrement; pointIndex++)
		{
			ContactPoint& existingPoint = pointsBuffer[pointIndex];
			
			if (length2(existingPoint.localPointA - newPoint.localPointA) < 4.f || // TODO tweak magic number
				length2(existingPoint.localPointB - newPoint.localPointB) < 4.f)
			{
				const auto depth = length2(newPoint.localPointA - existingPoint.localPointA) + length2(newPoint.localPointB - existingPoint.localPointB);
				if (depth < smallestDepth)
				{
					smallestDepth = depth;
					closest = &existingPoint;
				}
			}
		}

		if (closest != nullptr)
		{
			closest->isMerged = true;
			closest->isNew = false;
			closest->normal = newPoint.normal;
			closest->localPointA = newPoint.localPointA;
			closest->localPointB = newPoint.localPointB;
		}
		else
		{
			newPoint.isMerged = true;
			newPoint.isNew = true;
			pointsBuffer[pointCountToIncrement++] = newPoint;
		}
	}

	static vec2 GetPerpendicular(const vec2& value)
	{
		return vec2{ -value.y, value.x };
	}

	static void IntersectRayPlane(const vec2& point, const vec2& planePoint, const vec2& planeNormal, const vec2& projectionDirection,	vec2& output)
	{
		const auto multiplier = 1.f / compAdd(projectionDirection * planeNormal);
		output = point + projectionDirection * (planePoint * planeNormal - point * planeNormal) * multiplier;
	}

	static void GenerateContacts(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2,
		vec2& separatingAxis, ContactPoint* pointsBuffer, int& pointCountToIncrement)
	{
		if (compAdd(separatingAxis * (transform1.position - transform2.position)) < 0.0f)
			separatingAxis *= -1.f;
		
		vec2 contactPoints1[MAX_POINTS_PER_OVERLAP];
		vec2 contactPoints2[MAX_POINTS_PER_OVERLAP];

		float linearTolerance = 2.0f;

		// TODO shape size should come from transform?
		int pointsCount1 = CalculateContactPoints(shape1.size, -separatingAxis, contactPoints1, transform1.position, transform1.orientation);
		int pointsCount2 = CalculateContactPoints(shape2.size, separatingAxis, contactPoints2, transform2.position, transform2.orientation);

		if (pointsCount1 == 2 && length2(contactPoints1[0] - contactPoints1[1]) < linearTolerance * linearTolerance)
		{
			contactPoints1[0] = (contactPoints1[0] + contactPoints1[1]) * 0.5f;
			pointsCount1 = 1;
		}
		if (pointsCount2 == 2 && length2(contactPoints2[0] - contactPoints2[1]) < linearTolerance * linearTolerance)
		{
			contactPoints2[0] = (contactPoints2[0] + contactPoints2[1]) * 0.5f;
			pointsCount2 = 1;
		}
		
		if (pointsCount1 == 1 && pointsCount2 == 1)
		{
			vec2 delta = contactPoints2[0] - contactPoints1[0];
			if (compAdd(delta * separatingAxis) >= 0.0f)
			{
				ContactPoint newPoint(transform1.position, contactPoints1[0], transform2.position, contactPoints2[0], separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint);
				//DebugRendererTempData::Instance().DrawPoint(newPoint + transform1->position);
			}
		}
		else if (pointsCount1 == 1 && pointsCount2 == 2)
		{
			vec2 n = GetPerpendicular(contactPoints2[1] - contactPoints2[0]);
			vec2 outPoint;
			// TODO intersectRayPlane(contactPoints1[0], contactPoints2[0], n, separatingAxis, outPoint);
			IntersectRayPlane(contactPoints1[0], contactPoints2[0], n, separatingAxis, outPoint);

			if (compAdd((outPoint - contactPoints2[0]) * (contactPoints2[1] - contactPoints2[0])) >= 0.0f &&
				compAdd((outPoint - contactPoints2[1]) * (contactPoints2[0] - contactPoints2[1])) >= 0.0f)
			{
				ContactPoint newPoint(transform1.position, contactPoints1[0], transform2.position, contactPoints2[0], separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint);
				//DebugRendererTempData::Instance().DrawPoint(newPoint + transform1->position);
			}
		}
		else if (pointsCount1 == 2 && pointsCount2 == 1)
		{
			vec2 normal = GetPerpendicular(contactPoints1[1] - contactPoints1[0]);
			vec2 outPoint;
			IntersectRayPlane(contactPoints2[0], contactPoints1[0], normal, separatingAxis, outPoint);

			if (compAdd((outPoint - contactPoints1[0]) * (contactPoints1[1] - contactPoints1[0])) >= 0.0f &&
				compAdd((outPoint - contactPoints1[1]) * (contactPoints1[0] - contactPoints1[1])) >= 0.0f)
			{
				ContactPoint newPoint(transform1.position, outPoint, transform2.position, contactPoints2[0], separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint);
				//DebugRendererTempData::Instance().DrawPoint(newPoint + transform1->position);
			}
		}
		else if (pointsCount2 == 2 && pointsCount1 == 2)
		{
			struct TempColInfo
			{
				vec2 point1, point2;
			};
			TempColInfo tempCol[4];
			int tempCols = 0;
			for (int i = 0; i < 2; i++)
			{
				vec2 normal = GetPerpendicular(contactPoints2[1] - contactPoints2[0]);
				if (compAdd((contactPoints1[i] - contactPoints2[0]) * normal) >= 0.0)
				{
					vec2 outPoint;
					IntersectRayPlane(contactPoints1[i], contactPoints2[0], normal, separatingAxis, outPoint);

					if (compAdd((outPoint - contactPoints2[0]) * (contactPoints2[1] - contactPoints2[0])) >= 0.0f &&
						compAdd((outPoint - contactPoints2[1]) * (contactPoints2[0] - contactPoints2[1])) >= 0.0f)
					{
						tempCol[tempCols].point1 = contactPoints1[i];
						tempCol[tempCols].point2 = outPoint;
						tempCols++;
					}
				}
			}
			for (int i = 0; i < 2; i++)
			{
				vec2 normal = GetPerpendicular(contactPoints1[1] - contactPoints1[0]);
				if (compAdd((contactPoints2[i] - contactPoints1[0]) * normal) >= 0.0)
				{
					vec2 outPoint;
					IntersectRayPlane(contactPoints2[i], contactPoints1[0], normal, separatingAxis, outPoint);

					if (compAdd((outPoint - contactPoints1[0]) * (contactPoints1[1] - contactPoints1[0])) >= 0.0f &&
						compAdd((outPoint - contactPoints1[1]) * (contactPoints1[0] - contactPoints1[1])) >= 0.0f)
					{
						tempCol[tempCols].point1 = outPoint;
						tempCol[tempCols].point2 = contactPoints2[i];
						tempCols++;
					}
				}
			}

			if (tempCols == 1) // TODO review this, it's a kinda hack
			{
				ContactPoint newPoint(transform1.position, tempCol[0].point1, transform2.position, tempCol[0].point2, separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint);
				//DebugRendererTempData::Instance().DrawPoint(newPoint + transform1->position);
			}
			if (tempCols >= 2) // TODO review this
			{
				ContactPoint newPoint1(transform1.position, tempCol[0].point1, transform2.position, tempCol[0].point2, separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint1);
				//DebugRendererTempData::Instance().DrawPoint(newPoint1 + transform1->position);
				ContactPoint newPoint2(transform1.position, tempCol[1].point1, transform2.position, tempCol[1].point2, separatingAxis);
				AddContactPointToBuffer(pointsBuffer, pointCountToIncrement, newPoint2);
				//DebugRendererTempData::Instance().DrawPoint(newPoint2 + transform1->position);
			}
		}
	}

	void CreateOverlaps(std::vector<Overlap>& overlaps)
	{
		auto poolSize = overlaps.size(); // TODO

		// Each "overlap" has a 2-point range in the contact points collection
		// TODO SoA
		std::vector<ContactPoint> contactPoints(overlaps.size() * MAX_POINTS_PER_OVERLAP);

		//TODO this is perfectly parallelizable
		for (int i = 0; i < poolSize; ++i)
		{
			auto contactPointsRangeStart = i * MAX_POINTS_PER_OVERLAP;
			auto& overlap = overlaps[i];

			auto* overlapPointsBuffer = &contactPoints[contactPointsRangeStart]; // TODO go modern?

			std::vector<ContactPoint> newPointsBuffer(MAX_POINTS_PER_OVERLAP * 2); // TODO why *2???

			int pointCount = overlap.GetPointCount();

			for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
			{
				newPointsBuffer[pointIndex] = overlapPointsBuffer[pointIndex];
				newPointsBuffer[pointIndex].isMerged = false;
				newPointsBuffer[pointIndex].isNew = false;
			}
			
			const auto& transform1 = pools.dynamicProperties[overlap.firstEntityIndex];
			const auto& transform2 = pools.dynamicProperties[overlap.secondEntityIndex];
			const auto& shapeData1 = pools.shapeData[overlap.firstEntityIndex];
			const auto& shapeData2 = pools.shapeData[overlap.secondEntityIndex];

			vec2 separatingAxis;

			Transform t1{ transform1.position, vec2{}, transform1.orientation };
			Transform t2{ transform2.position, vec2{}, transform2.orientation };

			// TODO this takes position and orientation
			if (CalculateSatSimd(t1, t2, shapeData1, shapeData2, separatingAxis))
				GenerateContacts(t1, t2, shapeData1, shapeData2, separatingAxis, newPointsBuffer.data(), pointCount);

			overlap.ResetPointCount();

			for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
				if (overlapPointsBuffer[pointIndex].isMerged)
					overlapPointsBuffer[overlap.TryAddPoint()] = newPointsBuffer[pointIndex];
		}

		// RVO
	}
};
