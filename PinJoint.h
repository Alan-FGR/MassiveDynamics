#pragma once
#include "Constraint.h"
#include "Utils/SolverUtils.h"
#include "SoA.h"

struct PinJoint : Constraint
{
	// Absolute anchors A and B
	vec2 anchorA;
	vec2 anchorB;

	// Anchors A and B relative to bodies CoMs
	vec2 relAnchorA;
	vec2 relAnchorB;

	mat2x2 massTensor;

	vec2 bias;

	vec2 cachedAcceleration;

	PinJoint(BodiesSoA& bodiesSoA, size_t bodyA, size_t bodyB, vec2 pivot) : Constraint(bodyA, bodyB),
             relAnchorA{0},
             relAnchorB{0},
             massTensor{0},
             bias{0},
             cachedAcceleration{0}
	{
		anchorA = bodiesSoA.getPoolForTransform()[bodyA].transformPointInverse(pivot);
		anchorB = bodiesSoA.getPoolForTransform()[bodyB].transformPointInverse(pivot);
	}

	void preStep(BodiesSoA& bodiesSoA, const real dt) override
	{
		const auto transforms = bodiesSoA.getPoolForTransform();
		const auto positionals = bodiesSoA.getPoolForPositional();
		const auto physics = bodiesSoA.getPoolForInversePhysicalProperties();

		relAnchorA = transforms[a].rotation * (anchorA - positionals[a].centerOfMass);
		relAnchorB = transforms[b].rotation * (anchorB - positionals[b].centerOfMass);

		massTensor = SolverUtils::k_tensor(physics[a], physics[b], relAnchorA, relAnchorB);

		const vec2 delta = (positionals[b].position + relAnchorB) - (positionals[a].position + relAnchorA);
		bias = mul(delta, -1.f / dt);
	}

	void applyCachedImpulse(BodiesSoA& bodiesSoA, real dtCoef) override
	{
		const auto physics = bodiesSoA.getPoolForInversePhysicalProperties();
		const auto dynamics = bodiesSoA.getPoolForDynamical();
		SolverUtils::apply_impulses(physics[a], physics[b], relAnchorA, relAnchorB, mul(cachedAcceleration, dtCoef), dynamics[a], dynamics[b]);
	}

	void applyImpulse(BodiesSoA& bodiesSoA, const real dt) override
	{
		const auto physics = bodiesSoA.getPoolForInversePhysicalProperties();
		const auto dynamics = bodiesSoA.getPoolForDynamical();

		const vec2 relVelocity = SolverUtils::relative_velocity(dynamics[a], dynamics[b], relAnchorA, relAnchorB);

		const vec2 currentAcceleration = cachedAcceleration;
		vec2 newAcceleration = massTensor * (bias - relVelocity);
		cachedAcceleration = cachedAcceleration + newAcceleration;
		newAcceleration = cachedAcceleration - currentAcceleration;

		SolverUtils::apply_impulses(physics[a], physics[b], relAnchorA, relAnchorB, newAcceleration, dynamics[a], dynamics[b]);
	}
};
