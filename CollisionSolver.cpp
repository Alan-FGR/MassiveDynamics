#include "CollisionSolver.h"

std::vector<BroadphaseEntry> CollisionSolver::SortAabbs(const std::vector<Aabb>& aabbs)
{
	const auto poolSize = aabbs.size(); // TODO

	// Calculate broad phase pairs
	std::vector<BroadphaseEntry> broadphaseData;
	{
		// TODO SoA, prealloc and cache these
		AvxVector<std::pair<float, int>> broadphaseDataPairs;
		AvxVector<std::pair<float, int>> broadphaseDataPairsSorted;

		// Populate pairs to be sorted by aabb min X
		for (size_t i = 0; i < poolSize; ++i)
		{
			const auto& aabb = aabbs[i];
			std::pair<float, int> pair{ aabb.min.x, i }; // Pairs hold their index
			broadphaseDataPairs.emplace_back(pair);
		}

		broadphaseDataPairsSorted.resize(broadphaseDataPairs.size());

		std::partial_sort_copy(
			broadphaseDataPairs.data(), broadphaseDataPairs.data() + poolSize,
			broadphaseDataPairsSorted.data(), broadphaseDataPairsSorted.data() + poolSize
		);

		for (size_t i = 0; i < poolSize; ++i)
		{
			const auto originalIndex = broadphaseDataPairsSorted[i].second;
			const auto& aabb = aabbs[originalIndex];

			BroadphaseEntry e = { aabb, originalIndex };
			broadphaseData.emplace_back(e);
		}
	}
	return broadphaseData; // RVO
}

std::vector<Overlap> CollisionSolver::CreatePairs(std::vector<BroadphaseEntry>& sortedAabbs)
{
	const auto poolSize = sortedAabbs.size(); // TODO

	// TODO we could also build all intersections on one axis, then solve the other in multiple threads

	// TODO one collection per thread then join all
	std::unordered_set<std::pair<int, int>, _intPairHasher> pairs;

	for (size_t i = 0; i < poolSize; ++i)
	{
		const BroadphaseEntry& currentEntry = sortedAabbs[i];

		for (size_t nextEntryIndex = i + 1; nextEntryIndex < poolSize; nextEntryIndex++)
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

	std::vector<Overlap> overlaps{};

	// TODO when MT, join all threads results
	int c = 0;
	for (const auto& pair : pairs)
	{
		overlaps.emplace_back(pair.first, pair.second, c++);
	}

	return overlaps; // RVO
}

std::vector<ContactPoint> CollisionSolver::CreateOverlaps(
	std::vector<Overlap>& overlaps,
	const AvxVector<DynamicProperties>& dynamicProperties,
	const AvxVector<Shape>& shapeData)
{
	//debugLines.clear();

	const auto poolSize = overlaps.size(); // TODO

	// Each "overlap" has a 2-point range in the contact points collection
	// TODO SoA
	std::vector<ContactPoint> contactPoints(overlaps.size() * MAX_POINTS_PER_OVERLAP);

	//TODO this is perfectly parallelizable
	for (size_t i = 0; i < poolSize; ++i)
	{
		auto contactPointsRangeStart = i * MAX_POINTS_PER_OVERLAP;
		auto& overlap = overlaps[i];

		//auto* overlapPointsBuffer = &contactPoints[contactPointsRangeStart]; // TODO go modern?

		std::vector<ContactPoint> newPointsBuffer(MAX_POINTS_PER_OVERLAP * 2); // TODO explain *2

		int pointCount = overlap.GetPointCount();

		for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
		{
			newPointsBuffer[pointIndex] = contactPoints[pointIndex];
			newPointsBuffer[pointIndex].isMerged = false;
			newPointsBuffer[pointIndex].isNew = false;
		}

		const auto& transform1 = dynamicProperties[overlap.firstEntityIndex];
		const auto& transform2 = dynamicProperties[overlap.secondEntityIndex];
		const auto& shapeData1 = shapeData[overlap.firstEntityIndex];
		const auto& shapeData2 = shapeData[overlap.secondEntityIndex];

		vec2 separatingAxis;

		Transform t1{ transform1.position, vec2{}, transform1.orientation };
		Transform t2{ transform2.position, vec2{}, transform2.orientation };

		// TODO this takes position and orientation
		if (CalculateSatSimd(t1, t2, shapeData1, shapeData2, separatingAxis))
			GenerateContacts(t1, t2, shapeData1, shapeData2, separatingAxis, newPointsBuffer, pointCount);

		overlap.ResetPointCount();

		for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
			if (newPointsBuffer[pointIndex].isMerged)
				contactPoints[overlap.AddPointAndReturnIndex()] = newPointsBuffer[pointIndex];
	}

	return contactPoints; // RVO
}

bool CollisionSolver::CalculateSatSimd(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2, vec2& axis)
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

void CollisionSolver::GenerateContacts(const Transform& transform1, const Transform& transform2, const Shape& shape1, const Shape& shape2,
	vec2& separatingAxis, std::vector<ContactPoint>& newPointsBuffer, int& pointCountToIncrement)
{
	if (compAdd(separatingAxis * (transform1.position - transform2.position)) < 0.0f)
		separatingAxis *= -1.f;

	vec2 contactPoints1[MAX_POINTS_PER_OVERLAP];
	vec2 contactPoints2[MAX_POINTS_PER_OVERLAP];

	float linearTolerance = 2.0f;

	//std::array<uint8, 4> lineColor{ 255,0,255,255 };

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
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint.localPointA, lineColor });
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
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint.localPointA, lineColor });
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
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint.localPointA, lineColor });
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
		for (auto& i : contactPoints1)
		{
			vec2 normal = GetPerpendicular(contactPoints2[1] - contactPoints2[0]);
			if (compAdd((i - contactPoints2[0]) * normal) >= 0.f)
			{
				vec2 outPoint;
				IntersectRayPlane(i, contactPoints2[0], normal, separatingAxis, outPoint);

				if (compAdd((outPoint - contactPoints2[0]) * (contactPoints2[1] - contactPoints2[0])) >= 0.0f &&
					compAdd((outPoint - contactPoints2[1]) * (contactPoints2[0] - contactPoints2[1])) >= 0.0f)
				{
					tempCol[tempCols].point1 = i;
					tempCol[tempCols].point2 = outPoint;
					tempCols++;
				}
			}
		}
		for (auto& i : contactPoints2)
		{
			vec2 normal = GetPerpendicular(contactPoints1[1] - contactPoints1[0]);
			if (compAdd((i - contactPoints1[0]) * normal) >= 0.f)
			{
				vec2 outPoint;
				IntersectRayPlane(i, contactPoints1[0], normal, separatingAxis, outPoint);

				if (compAdd((outPoint - contactPoints1[0]) * (contactPoints1[1] - contactPoints1[0])) >= 0.0f &&
					compAdd((outPoint - contactPoints1[1]) * (contactPoints1[0] - contactPoints1[1])) >= 0.0f)
				{
					tempCol[tempCols].point1 = outPoint;
					tempCol[tempCols].point2 = i;
					tempCols++;
				}
			}
		}

		if (tempCols == 1) // TODO review this, it's a kinda hack
		{
			ContactPoint newPoint(transform1.position, tempCol[0].point1, transform2.position, tempCol[0].point2, separatingAxis);
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint.localPointA, lineColor });
		}
		if (tempCols >= 2) // TODO review this
		{
			ContactPoint newPoint1(transform1.position, tempCol[0].point1, transform2.position, tempCol[0].point2, separatingAxis);
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint1);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint1.localPointA, lineColor });

			ContactPoint newPoint2(transform1.position, tempCol[1].point1, transform2.position, tempCol[1].point2, separatingAxis);
			AddContactPointToBuffer(newPointsBuffer, pointCountToIncrement, newPoint2);
			//debugLines.emplace_back(DebugLine{ transform1.position, transform1.position + newPoint2.localPointA, lineColor });
		}
	}
}

int CollisionSolver::CalculateContactPoints(const vec2& size, const vec2& axis, vec2* supportPoints, const vec2& pos, const mat2x2& rot)
{
	const auto distToEdgeMidPoint = std::min(fabsf(compAdd(axis * rot[0])), fabsf(compAdd(axis * rot[1])));

	if (distToEdgeMidPoint < 0.1f) // TODO magic number tweak
	{
		vec2& edgePoint1 = supportPoints[0];
		vec2& edgePoint2 = supportPoints[1];
		edgePoint1 = pos;
		edgePoint2 = pos;
		const vec2 xSize = rot[0] * size.x;
		const vec2 ySize = rot[1] * size.y;
		vec2 offset = vec2{ 0 };
		const float xDiff = compAdd(axis * rot[0]);
		const float yDiff = compAdd(axis * rot[1]);
		if (fabsf(xDiff) < fabsf(yDiff))
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
		const vec2 xdim = rot[0] * size.x;
		const vec2 ydim = rot[1] * size.y;
		const float xsgn = compAdd(rot[0] * axis) < 0.0f ? -1.0f : 1.0f;
		const float ysgn = compAdd(rot[1] * axis) < 0.0f ? -1.0f : 1.0f;
		edgepoint1 = pos + xsgn * xdim + ysgn * ydim;
		return 1;
	}
}

void CollisionSolver::AddContactPointToBuffer(std::vector<ContactPoint>& newPointsBuffer, int& pointCountToIncrement, ContactPoint& newPoint)
{
	ContactPoint* closest = nullptr;
	auto smallestDepth = std::numeric_limits<float>::max();

	for (int pointIndex = 0; pointIndex < pointCountToIncrement; pointIndex++)
	{
		ContactPoint& existingPoint = newPointsBuffer[pointIndex];

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
		newPointsBuffer[pointCountToIncrement] = newPoint;
		pointCountToIncrement++;
	}
}

