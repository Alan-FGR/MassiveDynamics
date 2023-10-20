#pragma once

#include <array>
#include <string>

#include "glm/glm.hpp"

struct DebugLine
{
	vec2 start, end;
	std::array<uint8, 4> color;
};

struct DebugText
{
	vec2 position;
	std::string text;
	std::array<uint8, 4> color;
};