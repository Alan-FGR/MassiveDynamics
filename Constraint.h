#pragma once
#include "Common/MathTypes.h"

struct BodiesSoA;

struct Constraint
{
	size_t a, b;

	real errorBias;

	virtual ~Constraint() = default;

	Constraint(size_t _a, size_t _b)
	{
		a = _a;
		b = _b;

		errorBias = pow(1.0f - 0.1f, 60.0f);
	}

	virtual void  preStep(BodiesSoA&, real dt) = 0;
	virtual void  applyCachedImpulse(BodiesSoA&, real dtCoef) = 0;
	virtual void  applyImpulse(BodiesSoA&, real dt) = 0;
};
