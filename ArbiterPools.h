#pragma once
#include <vector>
#include <unordered_map>

#include "Common/MathTypes.h"
#include "Collision/Separation.h"
#include "Utils/CollisionUtils.h"

// TODO befriend arbiter creation

struct ArbiterPools
{
	// TODO separate manifolds and other arb data, since these can be bound as output pools
	std::vector<Arbiter> storage;
	std::unordered_map<uintptr_t, size_t> cached;

private:
	std::vector<bool> freeSlots; // TODO bit fields, do the SIMD-width popcnt check trick
	// TODO res-68 shows a tipping point in which perf start to degrade quadratically
	std::vector<bool> activeSlots;

	size_t createArbiter()
	{
		for (size_t i = 0; i < storage.size(); ++i)
			if (freeSlots[i])
			{
				storage[i] = Arbiter{};
				freeSlots[i] = false;
				return i;
			}

		storage.emplace_back(Arbiter{});
		freeSlots.emplace_back(false);
		activeSlots.emplace_back(false);
		return storage.size() - 1;
	}

public:
	void reset()
	{
		for (size_t i = 0; i < storage.size(); ++i)
			if (activeSlots[i])
			{
				// TODO memzero all range? Can we store arbiter state fully outta the band?
				activeSlots[i] = false;
				storage[i].state = ArbiterState::Default;
			}
	}

	void filter(uint32_t currentIteration, uint32_t collisionPersistence)
	{
		erase_if(cached, [&](std::pair<uintptr_t, size_t> pair)
		{
			auto& arb = storage[pair.second];

			const uint32_t elapsedIterations = currentIteration - arb.lastIteration;

			if (elapsedIterations >= 1 && arb.state != ArbiterState::Cached)
			{
				arb.state = ArbiterState::Cached; // TODO do this elsewhere?
			}

			if (elapsedIterations >= collisionPersistence)
			{
				freeSlots[pair.second] = true;
				return true;
			}

			return false;
		});
	}

	template <typename TFunc>
	void foreachActive(TFunc&& func)
	{
		for (size_t i = 0; i < storage.size(); ++i)
		{
			if (activeSlots[i])
				func(storage[i]);
		}
	}

	Arbiter& getOrCreateFor(PhysicalShape* shapeA, PhysicalShape* shapeB)
	{
		auto arbHashID = orderIndependentHash<uintptr_t>(shapeA, shapeB);

		size_t arb;

		if (const auto search = cached.find(arbHashID); search != cached.end())
		{
			arb = search->second;
		}
		else
		{
			arb = createArbiter();
			cached.insert({arbHashID, arb});
		}

		activeSlots[arb] = true;

		return storage[arb];
	}
};
