export module PhysicsWorld;

import <algorithm>;
import <memory>;
import <vector>;
import <unordered_set>;

import TypeAliases;
import AvxVector;
import Transform;
import Dynamics;
import Shape;

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
	
	struct _intPairHasher {
		int operator()(const std::pair<int, int>& pair) const {
			return std::hash<int>()(pair.first) * 31 + std::hash<int>()(pair.second);
		}
	};

	class OverlapPoints
	{
		int pointsCount;

	public:
		const int firstEntityIndex, secondEntityIndex, pointsStartIndex;

		const int MAX_POINTS_PER_OVERLAP = 2;

		OverlapPoints(int firstEntityIndex, int secondEntityIndex, int pointsRangeIndex) :
			firstEntityIndex(firstEntityIndex),
			secondEntityIndex(secondEntityIndex),
			pointsStartIndex(pointsRangeIndex * MAX_POINTS_PER_OVERLAP)
		{ }
	};

	std::vector<OverlapPoints> CreatePairs(std::vector<BroadphaseEntry> sortedAabbs)
	{
		auto poolSize = sortedAabbs.size(); // TODO
		
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
				if (nextEntry.originalAabb.max.y > currentEntry.originalAabb.min.y &&
					nextEntry.originalAabb.min.y < currentEntry.originalAabb.max.y)
					pairs.emplace(std::make_pair(
						std::min(currentEntry.originalIndex, nextEntry.originalIndex),
						std::max(currentEntry.originalIndex, nextEntry.originalIndex)
					));
				
			}
		}

		std::vector<OverlapPoints> overlaps;

		// TODO when MT, join all threads results
		int c = 0;
		for (const auto& pair : pairs)
		{
			overlaps.emplace_back(OverlapPoints(pair.first, pair.second, c++));
		}

		return overlaps; // RVO
	}

	void CreateOverlaps(std::vector<OverlapPoints>& overlapPoints)
	{
		
	}

	

};
