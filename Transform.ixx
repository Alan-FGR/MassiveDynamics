export module Transform;

import "glm/glm.hpp";
import "glm/ext.hpp";
import "glm/gtx/euler_angles.hpp";

using namespace glm;

export struct Transform
{
	vec2 position;
	vec2 scale;
	mat2x2 orientation{vec2{1,0},vec2{0,1}};

	void SetOrientationAngle(float angle)
	{
		orientation = orientate2(angle);
	}
};