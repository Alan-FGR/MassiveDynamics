#pragma once
#include "Common/MathTypes.h"
#include "Common/PhysicsData.h"

struct RigidBody;
struct PhysicalShape;

enum ShapeType : unsigned
{
	Circle = 0b001,
	Segment = 0b010,
	Polygon = 0b100,
};

// TODO rid our poor lives of this OOP nonsense! :((((((
struct PhysicalShape
{
	virtual ~PhysicalShape() = default;
	PhysicalShape(const PhysicalShape& other) = default;
	PhysicalShape(PhysicalShape&& other) noexcept = default;
	PhysicalShape& operator=(const PhysicalShape& other) = default;
	PhysicalShape& operator=(PhysicalShape&& other) noexcept = default;

	explicit PhysicalShape(const SurfaceMaterial& material) :
		body{std::numeric_limits<size_t>::max()},
		sp{{material.elasticity, material.friction}, zero<vec2>}
	{
	}

	uintptr_t hashId{};
	size_t body; //TODO

	// TODO use same data as arbiter, only diff is that arbiter is always relative
	// TODO "material" type as sugar for these
	SurfaceProperties sp;

	[[nodiscard]] virtual ShapeType shapeType() const = 0;
};
