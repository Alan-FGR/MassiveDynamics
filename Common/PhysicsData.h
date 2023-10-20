#pragma once
#include "Common/MathTypes.h"

struct SurfaceMaterial
{
	real elasticity;
	real friction;
};

struct SurfaceProperties
{
	SurfaceMaterial material;
	vec2 velocity;
};