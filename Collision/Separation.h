#pragma once
#include "CollisionData.h"
#include "RigidBody.h"
#include "Common/MathExtras.h"

enum class ArbiterState
{
	Default,
	Initial, ///< Just collided, there's probably a smart way to avoid having this state, then we could go binary perhaps
	Cached,	///< We cache contacts for reusing in case a collision between the same shapes happens in short sequence
};

struct Arbiter // TODO make this a template for concrete shapes
{
	size_t bodyA;
	size_t bodyB;

	SurfaceProperties surfaceProperties;
	Manifold manifold{};

	uint32_t lastIteration;
	ArbiterState state{ArbiterState::Default}; // TODO pack with iterstamp?

	void init(
		const Positional* positionalsPool,
		const SurfaceProperties& surfA, const SurfaceProperties& surfB,
		const size_t& bodyA, const size_t& bodyB,
		const uint32_t& currentIteration, Manifold&& newManifold)
	{
		// We're resurrecting a cached contact. TODO revamp this
		if (state == ArbiterState::Cached) state = ArbiterState::Initial;

		this->bodyA = bodyA;
		this->bodyB = bodyB;

		// newManifold is the new contacts calculated in the current (new) frame for the same pair.
		// They have to be transformed to local since they're created in world space,
		// and this also checks whether new contacts match the existing ones (persisting contacts)
		newManifold.transformAndSyncContacts(positionalsPool[bodyA], positionalsPool[bodyB], manifold);

		// TODO maybe overload *? idk
		surfaceProperties.material.elasticity = surfA.material.elasticity * surfB.material.elasticity;
		surfaceProperties.material.friction = surfA.material.friction * surfB.material.friction;

		const vec2 relativeVelocity = surfB.velocity - surfA.velocity;
		surfaceProperties.velocity = relativeVelocity - newManifold.normal * dot(relativeVelocity, newManifold.normal);

		lastIteration = currentIteration;

		manifold = std::move(newManifold);
	}
};
