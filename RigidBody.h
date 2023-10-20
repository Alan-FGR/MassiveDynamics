#pragma once
#include <vector>

#include "Common/MathTypes.h"
#include "PhysicalShapes/PhysicalShape.h"

struct PhysicalProps
{
	real mass;
	real moment;
};

struct InversePhysicalProps
{
	real massInverse;
	real momentInverse;
};

struct Positional
{
	vec2 position;
	real angle;
	vec2 centerOfMass;
};

struct Dynamical
{
	vec2 linearVelocity;
	real angularVelocity;
};

struct OverlapCorrectors
{
	vec2 linearCorrection;
	real angularCorrection;
};

struct Forces
{
	vec2 linearForce;
	real angularForce; ///< AKA Torque
};

// TODO lib that will create this facade for us. This is just for convenience.
struct RigidBody
{
	PhysicalProps physicalProps;
	InversePhysicalProps inversePhysicalProps;
	Positional positional;
	Dynamical dynamics;
	OverlapCorrectors overlapCorrectors;
	Forces forces;
	Transform transform;
	std::vector<PhysicalShape*> shapeList;

	RigidBody(real mass, real moment)
		: positional{},
		  dynamics{},
		  overlapCorrectors{},
		  forces{},
		  transform{{},{}}
	{
		shapeList = std::vector<PhysicalShape*>{};
		physicalProps = { mass, moment };

		// TODO these shall be updated when the ones above are
		inversePhysicalProps.massInverse = mass == 0.f ? INFINITY : 1.f / mass;
		inversePhysicalProps.momentInverse = moment == 0.f ? INFINITY : 1.f / moment;
	}
};
