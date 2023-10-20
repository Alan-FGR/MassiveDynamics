#pragma once

#include "soagen/soagen.hpp"
#include "RigidBody.h"

class BodiesSoA
{

	soagen::table<soagen::table_traits<
		PhysicalProps,
		InversePhysicalProps,
		Positional, 
		Dynamical,
		Forces, 
		Transform, 
		std::vector<PhysicalShape*>,
		OverlapCorrectors
		//soagen::column_traits<vec3, 32>, // over-aligned TODO align stuff like this and whatnot
		>> pools;

public:

	// TODO revamp big time! we have inputs, outputs and whatnot, all shoehorned in here.
	// bodies shouldn't contain dynamics / solver output stuff, much less caches and whatnot

	PhysicalProps* getPoolForPhysicalProperties() { return pools.column<0>(); }
	InversePhysicalProps* getPoolForInversePhysicalProperties() { return pools.column<1>(); }
	Positional* getPoolForPositional() { return pools.column<2>(); }
	Dynamical* getPoolForDynamical() { return pools.column<3>(); }
	Forces* getPoolForForces() { return pools.column<4>(); }
	Transform* getPoolForTransform() { return pools.column<5>(); }
	std::vector<PhysicalShape*>* getPoolForShapeList() { return pools.column<6>(); }
	OverlapCorrectors* getPoolForCorrectors() { return pools.column<7>(); }

	size_t size() const { return pools.size(); }

	size_t AddBody(const RigidBody&& body)
	{
		pools.emplace_back(
			body.physicalProps,
			body.inversePhysicalProps,
			body.positional,
			body.dynamics,
			body.forces,
			body.transform,
			body.shapeList,
			body.overlapCorrectors
		);
		return pools.size() - 1;
	}
};