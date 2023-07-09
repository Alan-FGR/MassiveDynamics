#pragma once

#include "glm/glm.hpp"

#include <vector>

#include "DebugTypes.h"
#include "ImpulseSolver.h"
#include "Transform.h"

using namespace glm;

class PhysicsWorld
{

public:
	DataPools pools; // TODO

	inline static std::vector<DebugLine> debugLines;
	inline static std::vector<DebugText> debugText;

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
		for (size_t i = 0; i < pools.dynamicProperties.size(); ++i)
		{
			func(pools.entities[i],
				pools.dynamicProperties[i],
				pools.shapeSize[i],
				pools.shapeData[i]
				//TODO pass ptrs to other stuff that makes sense (e.g. shape)
			);
		}
	}

	EntityId AddEntity(const Transform& transform, const ShapeType shapeType, bool isKinematic = false)
	{
		const auto density = 1e-5f;

		//TODO LP memcpy
		DynamicProperties properties;
		properties.position = transform.position;
		properties.orientation = transform.orientation;

		//TODO calc mass from transform and shape
		const auto area = transform.scale.x * transform.scale.y;
		const auto calculatedMass = density * area;
		const auto calculatedMassInverse = 1.f / calculatedMass;
		const auto calculatedInertiaInverse = 1.f / (calculatedMass *
			(glm::pow(transform.scale.x, 2.f) + glm::pow(transform.scale.y, 2.f)));

		properties.massInverse = isKinematic ? 0 : calculatedMassInverse;
		properties.inertiaInverse = isKinematic ? 0 : calculatedInertiaInverse;

		return pools.AddSimulationEntry(properties, transform.scale, shapeType);
	}

	void Update(float iterationTime)
	{
		// TODO this is currently done in the debug program

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

	// TODO idk, move? :(
	SolverData PrepareSolverData(int bodiesCount)
	{
		SolverData results{};

		results.Impulse.reserve(bodiesCount);
		results.Displacement.reserve(bodiesCount);

		// TODO parallel
		for (size_t i = 0; i < bodiesCount; ++i)
		{
			results.Impulse.data()[i].velocity = pools.velocity[i];
			results.Impulse.data()[i].angularVelocity = pools.angularVelocity[i];
			results.Impulse.data()[i].lastIteration = -1;

			results.Displacement.data()[i].velocity = pools.displacingVelocity[i];
			results.Displacement.data()[i].angularVelocity = pools.displacingAngularVelocity[i];
			results.Displacement.data()[i].lastIteration = -1;
		}

		return results;
	}

	//TODO move
	void ApplySolverData(int bodiesCount, const SolverData& solverData)
	{

		// TODO parallel
		for (size_t i = 0; i < bodiesCount; ++i)
		{
			pools.velocity[i] = solverData.Impulse.data()[i].velocity;
			pools.angularVelocity[i] = solverData.Impulse.data()[i].angularVelocity;

			pools.displacingVelocity[i] = solverData.Displacement.data()[i].velocity;
			pools.displacingAngularVelocity[i] = solverData.Displacement.data()[i].angularVelocity;
		}
	}

	//TODO move
	static std::vector<Aabb> UpdateAABBs(const AvxVector<DynamicProperties>& dynamicProperties, const AvxVector<vec2>& shapeSizes)
	{
		const auto poolSize = dynamicProperties.size();

		std::vector<Aabb> data;

		for (size_t i = 0; i < poolSize; ++i)
		{
			data.emplace_back(Aabb::CalculateFrom(
				dynamicProperties[i].position,
				dynamicProperties[i].orientation,
				shapeSizes[i]
			));
		}

		return data; // RVO
	}
	
	// TODO move all this stuff to proper places
	
	static constexpr int GRAVITY = -10; // u/s^2
				
	void IntegrateVelocity(float dt) // copies accelerations to velocities
	{
		//TODO parallelize
		for (size_t i = 0; i < pools.entities.size(); i++)
		{
			EntityId& body = pools.entities[i];
			const DynamicProperties& param = pools.dynamicProperties[i];

			if (param.massInverse > 0.0f)
			{
				pools.acceleration[i].y += GRAVITY;
			}

			pools.velocity[i] += pools.acceleration[i] * dt;
			pools.acceleration[i] = vec2(0.0f, 0.0f);

			pools.angularVelocity[i] += pools.angularAcceleration[i] * dt;
			pools.angularAcceleration[i] = 0.0f;
		}
	}

	void IntegratePosition(float dt)
	{
		//TODO parallelize
		for (size_t i = 0; i < pools.entities.size(); i++)
		{
			const auto penetrationCorrectionMult = 5.f;

			pools.dynamicProperties[i].position += pools.displacingVelocity[i] * penetrationCorrectionMult + pools.velocity[i] * dt;

			const auto angle = -(pools.displacingAngularVelocity[i] * 1 + pools.angularVelocity[i] * dt);

			RotateVector(pools.dynamicProperties[i].orientation[0], angle);
			RotateVector(pools.dynamicProperties[i].orientation[1], angle);

			pools.displacingVelocity[i] = vec2(0.0f, 0.0f);
			pools.displacingAngularVelocity[i] = 0.0f;
		}
	}

	void RotateVector(vec2& self, const float angle)
	{
		const vec2 x = self;
		const vec2 y = vec2(-x.y, x.x);
		const vec2 delta = x * cos(angle) + y * sin(angle) - x;
		self += delta;
	}
};
