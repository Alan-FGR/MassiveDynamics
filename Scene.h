#pragma once
#include <algorithm>
#include <functional>
#include <random>
#include <vector>

// TODO review all of these
#include "ArbiterPools.h"
#include "Constraint.h"
#include "Integration.h"
#include "Utils/CollisionUtils.h"
#include "Utils/GeometryUtils.h"
#include "PhysicalShapes/PhysicalShape.h"
#include "PinJoint.h"
#include "SoA.h"

// TODO api could be:
// add bodies to space - dynamic bodies
//			add shapes to dynamic bodies
// add shapes to space - static shapes

// TODO body-shape mapping
// sparse map for bodies to shapes
// ---------------------------------
// bodies ##########################
// tforms ########################## (body physical->transform output pool)
// shape0 ############ ############# (maybe a body won't have a shape here and there, e.g. a bullet in which collision is raycasted manually)
// shape1 ## # ##   #   # ## ## # #  
// shape2 #     #   #      #  #   #
// shape3 #                #
// shape4+ could be mapped elsewhere. Every 10k shape pool takes 1MB.
// ---------------------------------

// TODO optional faster collision detection library when you only need performance
// TODO alternative stateless XPBD solver - if I can ever make separation quality acceptable

enum struct CollisionType : unsigned
{
	CircleToCircle   = 0b001, //1
	SegmentToSegment = 0b010, //2
	CircleToSegment  = 0b011, //3
	PolyToPoly       = 0b100, //4
	CircleToPoly     = 0b101, //5
	SegmentToPoly    = 0b110, //6
};

struct NarrowPhasePools
{
	using CollisionFunc = void(*)(const PhysicalShape* a, const PhysicalShape* b, Manifold& contacts);

	std::vector<std::pair<PhysicalShape*, PhysicalShape*>> collisionGroups[6];

	inline static CollisionFunc collisionFuncs[] = {
		(CollisionFunc)CollisionUtils::CircleToCircle,
		(CollisionFunc)CollisionUtils::SegmentToSegment,
		(CollisionFunc)CollisionUtils::CircleToSegment,
		(CollisionFunc)CollisionUtils::PolyToPoly,
		(CollisionFunc)CollisionUtils::CircleToPoly,
		(CollisionFunc)CollisionUtils::SegmentToPoly,
	};

public: // TODO
	std::vector<std::pair<PhysicalShape*, PhysicalShape*>>& GetPoolForCollision(CollisionType type)
	{
		return collisionGroups[((unsigned)type)-1];
	}

	void calculateCollisions(const Positional* positionalsPool, const uint32_t& currentIteration, ArbiterPools& arbiterPools) const
	{
		const int sc = sizeof CircleShape;
		const int ss = sizeof SegmentShape;
		const int sp = sizeof PolygonShape;

		/*constexpr [C++26]*/
		for (size_t i = 0; i < 6; ++i) // TODO we're doing this 6 times for all the combinations. Kinda sux
		{
			const auto& pool = collisionGroups[i];
			const auto& func = collisionFuncs[i];

			for (const auto& [shapeA, shapeB] : pool)
			{
				if (shapeA->body == shapeB->body) continue;
				
				Manifold contacts{};

				// TODO this could return the obj containing normal+contacts
				// Run the actual contact calculation routine for the shape type
				func(static_cast<PhysicalShape*>(shapeA), static_cast<PhysicalShape*>(shapeB), contacts);

				if (contacts.isEmpty()) continue;
				
				auto& arb = arbiterPools.getOrCreateFor(shapeA, shapeB);

				// TODO are we really gonna go with this for mapping shapes<-bodies?
				auto bodyA = shapeA->body;
				auto bodyB = shapeB->body;

				arb.init(positionalsPool, shapeA->sp, shapeB->sp, bodyA, bodyB, currentIteration, std::move(contacts));
			}
		}
	}
};

struct BroadPhasePools
{
	std::vector<std::pair<Aabb, PhysicalShape*>> boundingBoxes; // of all shapes
	std::vector<std::pair<Aabb, PhysicalShape*>> sortedAABBs;

	void prepareForQuerying()
	{
		sortedAABBs.resize(boundingBoxes.size());

		std::partial_sort_copy(
			boundingBoxes.begin(), boundingBoxes.end(),
			sortedAABBs.begin(), sortedAABBs.end(),
			[](std::pair<Aabb, PhysicalShape*>& a, std::pair<Aabb, PhysicalShape*>& b)
			{
				return a.first.min.x < b.first.min.x;
			}
		);
	}

	void createNarrowPhasePairs(NarrowPhasePools& narrowPhasePools) const
	{
		for (std::size_t i = 0; i < sortedAABBs.size() - 1; i++)
		{
			auto current = sortedAABBs[i];
			for (std::size_t j = i + 1; j < sortedAABBs.size(); ++j)
			{
				auto next = sortedAABBs[j];

				if (current.first.max.x < next.first.min.x)
					break;

				//TODO calculate min+max then here calc the other dimension
				if (current.first.max.y > next.first.min.y && current.first.min.y < next.first.max.y)
				{
					auto shapeTypeA = current.second->shapeType();
					auto shapeTypeB = next.second->shapeType();
					const auto collisionType = (CollisionType)(shapeTypeA | shapeTypeB); // TODO bring to lower cache
					if (shapeTypeA < shapeTypeB)
						narrowPhasePools.GetPoolForCollision(collisionType).emplace_back(current.second, next.second);
					else
						narrowPhasePools.GetPoolForCollision(collisionType).emplace_back(next.second, current.second);
				}
			}
		}
	}

	void updateSpatialStructure(
		std::unordered_map<uintptr_t, CircleShape*>& circleShapes,
		std::unordered_map<uintptr_t, SegmentShape*>& segmentShapes,
		std::unordered_map<uintptr_t, PolygonShape*>& polyShapes)
	{
		boundingBoxes.clear();

		// TODO template this
		// TODO update (at least poly) transforms on demand only (when broadphase passes)
		for (auto& shapePair : circleShapes)
		{
			auto* shape = shapePair.second;
			auto aabb = GeometryUtils::CalculateCircleAABB(*shape);
			boundingBoxes.emplace_back(aabb, shape);
		}
		for (auto& shapePair : segmentShapes)
		{
			auto* shape = shapePair.second;
			auto aabb = GeometryUtils::CalculateSegmentAABB(shape);
			boundingBoxes.emplace_back(aabb, shape);
		}
		for (auto& shapePair : polyShapes)
		{
			auto* shape = shapePair.second;
			auto aabb = GeometryUtils::CalculatePolyAABB(shape);
			boundingBoxes.emplace_back(aabb, shape);
		}
	}
};

struct Scene
{
	BodiesSoA bodiesSoA;

	// TODO separate transforms 
	std::unordered_map<uintptr_t, CircleShape*> circleShapes;
	std::unordered_map<uintptr_t, SegmentShape*> segmentShapes;
	std::unordered_map<uintptr_t, PolygonShape*> polyShapes;

	BroadPhasePools broadPhasePools;
	ArbiterPools arbiterPools;

	std::vector<Constraint*> constraints;

	uintptr_t shapeIDCounter;

	uint32_t iteration;
	real currentDeltaTime;

	uint32_t collisionPersistence{100};
	real collisionSlop{ 0.1f };
	real collisionBias{ pow(1.0f - 0.1f, 60.0f) };
	real damping{ 1 };

	size_t AddBody(const RigidBody& body)
	{
		return bodiesSoA.AddBody(std::move(body)); // TODO
	}

	void SetBodyPosition(size_t body, const vec2& position)
	{
		auto& positional = bodiesSoA.getPoolForPositional()[body];
		auto& transform = bodiesSoA.getPoolForTransform()[body];

		// TODO very confused now :S
		positional.position = transform.rotation * positional.centerOfMass + position;

		Integration::UpdateTransform(positional, transform);
	}

	PhysicalShape* AddShape(size_t bodyIndex, PhysicalShape* shape)
	{
		Transform& transform = bodiesSoA.getPoolForTransform()[bodyIndex];
		std::vector<PhysicalShape*>& shapeList = bodiesSoA.getPoolForShapeList()[bodyIndex];

		shapeList.emplace_back(shape);

		shape->body = bodyIndex;
		shape->hashId = shapeIDCounter++;

		Aabb bb;

		// TODO normalize all these called funcs (take const ref etc)
		// TODO add to correct collection, then call routine to update single instead
		if (shape->shapeType() == Circle)
		{
			Integration::TransformCircleToWorld(transform, (CircleShape&)*shape);
			bb = GeometryUtils::CalculateCircleAABB((CircleShape&)*shape);
			circleShapes.insert({ shape->hashId, (CircleShape*)shape });
		}
		else if (shape->shapeType() == Segment)
		{
			Integration::TransformSegmentToWorld(transform, (SegmentShape&)*shape);
			bb = GeometryUtils::CalculateSegmentAABB((SegmentShape*)shape);
			segmentShapes.insert({ shape->hashId, (SegmentShape*)shape });
		}
		else
		{
			Integration::TransformPolyToWorld(transform, (PolygonShape*)shape);
			bb = GeometryUtils::CalculatePolyAABB((PolygonShape*)shape);
			polyShapes.insert({ shape->hashId, (PolygonShape*)shape });
		}

		broadPhasePools.boundingBoxes.emplace_back(bb, shape);

		return shape;
	}

	void AddConstraint(PinJoint* constraint)
	{
		constraints.push_back(constraint);
	}

	void SimpleStep(float dt)
	{
		assert(dt > std::numeric_limits<real>::epsilon()); // TODO just skip

		real lastDeltaTime = currentDeltaTime;
		currentDeltaTime = dt;

		iteration++;

		broadPhasePools.prepareForQuerying(); // TODO do this last, and update on insertions/deletions to the scene
												// so we can accelerate user queries properly

		//TODO reuse and whatnot
		NarrowPhasePools narrowPhasePools;
		broadPhasePools.createNarrowPhasePairs(narrowPhasePools);

		arbiterPools.reset();

		narrowPhasePools.calculateCollisions(bodiesSoA.getPoolForPositional(), iteration, arbiterPools); // produces arbiters TODO createCollisionArbiters

		arbiterPools.filter(iteration, collisionPersistence);

		// Prestep the arbiters and constraints.
		real slop = collisionSlop;
		real biasCoef = 1.0f - pow(collisionBias, dt);
		arbiterPools.foreachActive([&](Arbiter& arb)
			{
				CollisionUtils::ArbiterPreStep(
					bodiesSoA.getPoolForPositional(),
					bodiesSoA.getPoolForInversePhysicalProperties(),
					bodiesSoA.getPoolForDynamical(),
					arb, dt, slop, biasCoef);
			});

		for (size_t i = 0; i < constraints.size(); i++) {
			Constraint* constraint = constraints[i];
			constraint->preStep(bodiesSoA, dt);
		}

		// Integrate velocities.
		real tickDamping = pow(damping, dt);
		for (size_t i = 0; i < bodiesSoA.size(); i++) 
		{
			auto& physics = bodiesSoA.getPoolForInversePhysicalProperties()[i];
			auto& forces = bodiesSoA.getPoolForForces()[i];
			auto& dynamics = bodiesSoA.getPoolForDynamical()[i];

			Integration::IntegrateVelocities(physics, forces, tickDamping, dt, dynamics);

			//// Apply gravity, this should be user-provided
			if (physics.massInverse != 0.f) // equivalent to mass == infinite
				dynamics.linearVelocity.y += -100 * dt;

			Integration::ZeroForces(forces);
		}

		// TODO contact and constraint graph, sort for solving
		// TODO sleeping and whatnot

		// Apply cached impulses
		real dtCoef = (lastDeltaTime == 0.0f ? 0.0f : dt / lastDeltaTime);
		arbiterPools.foreachActive([&](Arbiter& arb)
			{
				CollisionUtils::ArbiterApplyCachedImpulse(
					bodiesSoA.getPoolForInversePhysicalProperties(),
					bodiesSoA.getPoolForDynamical(),
					arb, dtCoef); // TODO can't we loop just cacheds??
			});

		for (size_t i = 0; i < constraints.size(); i++) {
			Constraint* constraint = constraints[i];
			constraint->applyCachedImpulse(bodiesSoA, dtCoef);
		}

		// Run the impulse solver.
		for (size_t i = 0; i </*iterations*/10; i++) {
			arbiterPools.foreachActive([&](Arbiter& arb)
				{
					CollisionUtils::ArbiterApplyImpulse(
						bodiesSoA.getPoolForInversePhysicalProperties(),
						bodiesSoA.getPoolForDynamical(), 
						bodiesSoA.getPoolForCorrectors(),
						arb);
				});

			for (size_t j = 0; j < constraints.size(); j++) {
				Constraint* constraint = constraints[j];
				constraint->applyImpulse(bodiesSoA, dt);
			}
		}

		auto* transforms = bodiesSoA.getPoolForTransform();

		for (int i = 0; i < bodiesSoA.size(); i++)
		{
			auto& positional = bodiesSoA.getPoolForPositional()[i];
			auto& dynamics = bodiesSoA.getPoolForDynamical()[i];
			auto& transform = transforms[i];

			Integration::IntegrateCorrections(dt, bodiesSoA.getPoolForCorrectors()[i], positional, 4);
			Integration::IntegratePositionals(dt, dynamics, positional);
			Integration::UpdateTransform(positional, transform);
			Integration::ZeroOverlapCorrectors(bodiesSoA.getPoolForCorrectors()[i]);
		}

		// TODO templates
		for (auto& shapePair : circleShapes)
		{
			auto* shape = shapePair.second;
			auto transform = transforms[shape->body];
			Integration::TransformCircleToWorld(transform, *shape);
		}
		for (auto& shapePair : segmentShapes)
		{
			auto* shape = shapePair.second;
			auto transform = transforms[shape->body];
			Integration::TransformSegmentToWorld(transform, *shape);
		}
		for (auto& shapePair : polyShapes)
		{
			auto* shape = shapePair.second;
			auto transform = transforms[shape->body];
			Integration::TransformPolyToWorld(transform, shape);
		}

		// TODO there's probably a smart way to spare some cache misses from what we do above and below here

		broadPhasePools.updateSpatialStructure(circleShapes, segmentShapes, polyShapes);
		// ^ We naturally (TODO SHOULD) leave the spatial structure primed for calls in between simulation steps
	}

	PhysicalShape* QueryPoint(const vec2& vec) const
	{
		for (const auto& pair : broadPhasePools.boundingBoxes)
		{
			if (pair.first.containsPoint(vec))
				return pair.second;
		}
		return nullptr;
	}

	// TODO very slow, map these somehow
	size_t GetShapeBody(const PhysicalShape* shape)
	{
		std::vector<PhysicalShape*>* shapeListsPool = bodiesSoA.getPoolForShapeList();
		for (size_t i = 0; i < bodiesSoA.size(); ++i)
		{
			auto& shapesList = shapeListsPool[i];
			for (const auto& bodyShape : shapesList)
			{
				if (bodyShape == shape)
					return i;
			}
		}
		return std::numeric_limits<size_t>::max();
	}

	void RemoveConstraint(PinJoint* constraint)
	{
		std::erase(constraints, constraint);
	}
};
