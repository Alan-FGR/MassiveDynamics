export module Transform;

import "glm/glm.hpp";
import "glm/ext.hpp";

using namespace glm;

export struct Transform
{
	vec2 position;
	vec2 scale;
	vec2 matX;
	vec2 matY;

	vec2 test()
	{
		auto dsd = position * matX.x;
		return dsd;
	}
};