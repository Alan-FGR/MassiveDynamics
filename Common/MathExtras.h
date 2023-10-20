#pragma once
#include "Common/MathTypes.h"
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtx/norm.hpp"

inline real clamp01(real f) { return glm::max(0.f, glm::min(f, 1.f)); }

// TODO Adapt and use these scalar math utils for easier SIMD conversions
inline vec2 add(const vec2 v1, const vec2 v2) { return v1 + v2; }
inline vec2 sub(const vec2 v1, const vec2 v2) { return v1 - v2; }
inline vec2 mul(const vec2 v, const real s) { return v * s; }
inline vec2 neg(const vec2 v) { return -v; }
inline bool equal(const vec2 v1, const vec2 v2) { return v1 == v2; }

inline vec2 rotateBy(const vec2 v1, const vec2 v2) { return { v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x }; }
inline vec2 perp(const vec2 v) { return { -v.y, v.x }; }
inline vec2 reversePerp(const vec2 v) { return { v.y, -v.x }; }
inline real perpDot(const vec2 v1, const vec2 v2) { return v1.x * v2.y - v1.y * v2.x; }
inline vec2 clampMagnitude(const vec2 v, const real len) { return length(v) > len ? normalize(v) : v; }