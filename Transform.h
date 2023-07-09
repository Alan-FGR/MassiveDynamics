#pragma once

#include "glm/glm.hpp"
#include "glm/gtx/euler_angles.hpp"

using namespace glm;

struct Transform
{
	vec2 position;
	vec2 scale;
	mat2x2 orientation{vec2{1,0},vec2{0,1}};

	void SetOrientationAngle(float angle)
	{
		orientation = orientate2(angle);
	}
};