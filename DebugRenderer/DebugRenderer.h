#pragma once
#include <vector>
#include <sstream>
#include <array>
#include <iostream>

#include "Common/MathTypes.h"
#include "Scene.h"

#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "SDL3/SDL.h"

// TODO some parts of this were kinda frankensteined together. Revamp.

template <typename A> constexpr auto println(A&& a) { std::cout << std::forward<A>(a) << "\n"; }

using uint8 = glm::uint8;
using debugColor = std::array<uint8, 4>;

struct DebugCamera // TODO use a proper matrix
{
	vec2 position;
	vec2 scale;
	mat2x2 orientation{ vec2{1,0},vec2{0,1} };

	void SetOrientationAngle(float angle)
	{
		orientation = glm::orientate2(angle);
	}
};

inline debugColor toDebugColor(const SDL_Color& color)
{
	return { color.r, color.g, color.b, color.a };
}

inline debugColor ptrToFillColor(uintptr_t ptr)
{
	return {
		(uint8)((std::hash<uintptr_t>()(ptr + 0) % 150) + 50),
		(uint8)((std::hash<uintptr_t>()(ptr + 1) % 150) + 50),
		(uint8)((std::hash<uintptr_t>()(ptr + 2) % 150) + 50),
		255
	};
}

struct Vertex
{
	vec2 position;
	unsigned char r, g, b, a;
};

static void SetVertex(SDL_Vertex& vert, const vec2& position, const debugColor& color)
{
	vert.position.x = position.x;
	vert.position.y = position.y;
	std::memcpy(&vert.color, &color, sizeof Uint8 * 4);
}

static SDL_Vertex CreateVertex(const vec2& position, const debugColor& color)
{
	SDL_Vertex v{};
	SetVertex(v, position, color);
	return v;
}

class DebugProgram
{
	Scene physicsScene;

	SDL_Window* window;
	SDL_Renderer* renderer;

	SurfaceMaterial mat{ 0.5, 0.7 };
	size_t ground;
	PinJoint* mouseJoint = nullptr;

	DebugCamera camera{ vec2{-500,-500}, vec2{1.5, -1.5}, mat2x2{vec2{1,0},vec2{0,1}} };
	std::vector<Vertex> vertices;

	std::unordered_map<uintptr_t, std::pair<vec2, real>> contactsToRender;

	bool shouldQuit = false;

	static double getElapsedTime()
	{
		return SDL_GetPerformanceCounter() / (double)SDL_GetPerformanceFrequency();
	}

	const static struct DebugColors
	{
		constexpr static SDL_Color Outline{ 255,255,255,64 };

		constexpr static SDL_Color White{ 255,255,255,255 };
		constexpr static SDL_Color Red{ 255,0,0,255 };
		constexpr static SDL_Color Green{ 0,255,0,255 };
		constexpr static SDL_Color Blue{ 0,0,255,255 };

		constexpr static SDL_Color Joint{ 255,200,0,255 };
		constexpr static SDL_Color JointDarker{ 200,150,0,255 };
		constexpr static SDL_Color JointLink{ 255,200,0,127 };
		constexpr static SDL_Color BroadphaseAabb{ 255,0,0,255 };
		constexpr static SDL_Color BroadphasePair{ 255,255,0,255 };
		constexpr static SDL_Color SortingPointer{ 255,0,0,127 };
		constexpr static SDL_Color ActiveArbiter{ 255,0,255,200 };
		constexpr static SDL_Color CachedContact{ 255,0,255,127 };
	};

public:
	explicit DebugProgram() : physicsScene({})
	{
		// C++23 println
		println("Starting debug renderer...");

		println("Initializing SDL3...");

		if (SDL_Init(SDL_InitFlags::SDL_INIT_VIDEO)) println(SDL_GetError());

		println("Creating Window...");

		window = SDL_CreateWindow("SDL3 Debug Renderer", 1024, 768, SDL_WindowFlags::SDL_WINDOW_MOUSE_FOCUS);
		if (!window) println(SDL_GetError());

		println("Creating Renderer...");

		SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
		SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengl");
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 6);

		renderer = SDL_CreateRenderer(window, nullptr, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
		if (!renderer) println(SDL_GetError());

		SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

		println("Debug Renderer Created!");

		populatePhysicsScene();
	}

	DebugProgram(const DebugProgram& other) = delete;
	DebugProgram& operator=(const DebugProgram& other) = delete;

	~DebugProgram()
	{
		println("Destroying debug renderer...");
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
	}

	void populatePhysicsScene()
	{
		real mass = 1;
		real width = 30;
		real height = width * 2;

		//mat = {1,1};
		//ground = AddCircle(vec2(0, 0), INFINITY, 100);
		//auto qbodyj2 = AddCircle(vec2(5, 150), mass, 10);
		//auto qbodyj3 = AddCircle(vec2(50, 150), mass, 10);
		//return;

		for (int y = 0; y < 25; ++y)
		{
			AddBox(add(vec2(240, y * 26 - 80), vec2(0.0, (height - width) / 2.0)), mass, 15, 15);
		}

		int stressAdditionalMult = 1;
		real stressSpacing = stressAdditionalMult > 1 ? 1.2f : 1;

		for (int x = 0; x < 5 * stressAdditionalMult; ++x)
		{
			for (int y = 0; y < 5 * stressAdditionalMult; ++y)
			{
				if ((x + y) % 2 == 0)
				{
					AddBox(add(vec2(x * (15 * stressSpacing) - 50, y * (15 * stressSpacing)), vec2(0.0, (height - width) / 2.0)), mass, 15, 15);
				}
				else
				{
					AddCircle(add(vec2(x * (15 * stressSpacing) - 50, y * (15 * stressSpacing)), vec2(0.0, (height - width) / 2.0)), mass, 10);
				}
			}
		}

		AddCircle(add(vec2(-75, -150), vec2(0.0, (height - width) / 2.0)), mass, 10);

		auto bodyj1 = AddCircle(vec2(-155, 250), mass, 10);
		auto bodyj2 = AddCircle(vec2(-125, 250), mass, 10);
		physicsScene.AddConstraint(new PinJoint(physicsScene.bodiesSoA, bodyj1, bodyj2, vec2(-140, 250)));

		// add multi shape body
		RigidBody multiShapeBody(6, GeometryUtils::MomentForBox(6, 50, 50));
		multiShapeBody.positional.centerOfMass = vec2(10, 10);
		auto msb = physicsScene.AddBody(multiShapeBody);
		auto msbPos = vec2(-50, 200);
		physicsScene.SetBodyPosition(msb, msbPos);
		auto msbA = physicsScene.AddShape(msb, new PolygonShape(mat, 50, 10, 0));
		auto msbB = physicsScene.AddShape(msb, new PolygonShape(mat, 10, 50, 0));
		auto msbC = physicsScene.AddShape(msb, new CircleShape(mat, 20, vec2(20, 20)));
		msbA->sp.material.friction = 0.7f;
		msbB->sp.material.friction = 0.7f;
		msbC->sp.material.friction = 0.7f;

		// add ground
		vec2 pos = vec2(0, -150);
		auto body = physicsScene.AddBody(RigidBody(INFINITY, INFINITY));
		physicsScene.SetBodyPosition(body, pos);
		PhysicalShape* inlined_shape = physicsScene.AddShape(body, new PolygonShape(mat, 500, 10, 10));
		inlined_shape->sp.material.elasticity = 1.0f;
		inlined_shape->sp.material.friction = 0.7f;
		ground = body;

		// add 
		//auto bodyjj1 = AddSegment(vec2(260, 100), 1, 10, 100);
		//auto bodyjj2 = AddSegment(vec2(-160, 100), 1, 10, 100);
		auto bodyjj1 = AddBox(vec2(260, 100), 1, 10, 100);
		auto bodyjj2 = AddBox(vec2(-160, 100), 1, 10, 100);
		physicsScene.AddConstraint(new PinJoint(physicsScene.bodiesSoA, bodyjj1, body, vec2(260, 100)));
		physicsScene.AddConstraint(new PinJoint(physicsScene.bodiesSoA, bodyjj2, body, vec2(-160, 100)));
	}

	auto AddBox(vec2 pos, real mass, real width, real height)
	{
		auto body = physicsScene.AddBody(RigidBody(mass, GeometryUtils::MomentForBox(mass, width, height)));
		physicsScene.SetBodyPosition(body, pos);

		PhysicalShape* shape = physicsScene.AddShape(body, new PolygonShape(mat, width, height, 2));
		shape->sp.material.elasticity = 0.5f;
		shape->sp.material.friction = 0.7f;

		return body;
	}

	// TODO just like box creation, add capsule creation for segments
	auto AddSegment(vec2 pos, real mass, real width, real height)
	{
		auto body = physicsScene.AddBody(RigidBody(mass, GeometryUtils::MomentForBox(mass, width, height)));
		physicsScene.SetBodyPosition(body, pos);

		PhysicalShape* shape = physicsScene.AddShape(body, new SegmentShape(mat, width / 2.0, vec2(0.0, (height - width) / 2.0), vec2(0.0, (width - height) / 2.0)));
		shape->sp.material.elasticity = 0.5f;
		shape->sp.material.friction = 0.7f;

		return body;
	}

	auto AddCircle(vec2 pos, real mass, real radius)
	{
		auto body = physicsScene.AddBody(RigidBody(mass, GeometryUtils::MomentForCircle(mass, 0.0, radius, ::zero<vec2>)));
		physicsScene.SetBodyPosition(body, pos);

		PhysicalShape* shape = physicsScene.AddShape(body, new CircleShape(mat, radius, ::zero<vec2>));
		shape->sp.material.elasticity = 0.5f;
		shape->sp.material.friction = 0.7f;

		return body;
	}

	void RunLoop()
	{
		double prevUpdateTime = 0.0f;

		while (!shouldQuit)
		{
			const float iterationTime = 1 / 60.f;
			float deltaTime = iterationTime;//TODO calc from elapsed

			const float cameraSpeed = 250;

			// Handle input
			vec2 mousePos;
			SDL_GetMouseState(&mousePos.x, &mousePos.y);
			mousePos = vec2{
			 (mousePos.x + camera.position.x) / camera.scale.x,
			 (mousePos.y + camera.position.y) / camera.scale.y,
			};

			if (mouseJoint)
				mouseJoint->anchorA = physicsScene.bodiesSoA.getPoolForTransform()[ground].transformPointInverse(mousePos);

			SDL_Event ev;
			while (SDL_PollEvent(&ev) != 0)
			{
				if (ev.type == SDL_EVENT_QUIT) return;
				if (ev.type == SDL_EVENT_KEY_DOWN)
				{
					if (ev.key.keysym.sym == SDLK_ESCAPE) return;

					if (ev.key.keysym.sym == SDLK_r)
					{
						physicsScene = {};
						populatePhysicsScene();
					}

					/*else if (ev.key.keysym.sym == SDLK_SPACE)
						physicsWorld.AddEntity(Transform{ glm::vec2{0, 10}, glm::vec2{10, 10} }, 0);*/

					// Camera controls
					else if (ev.key.keysym.sym == SDLK_w) camera.position += vec2(0, 1) * deltaTime * -cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_s) camera.position -= vec2(0, 1) * deltaTime * -cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_d) camera.position += vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_a) camera.position -= vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_e) camera.scale += vec2(1, -1) * deltaTime * cameraSpeed * 0.01f;
					else if (ev.key.keysym.sym == SDLK_q) camera.scale -= vec2(1, -1) * deltaTime * cameraSpeed * 0.01f;
				}
				else if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
				{
					if (ev.button.button == SDL_BUTTON_RIGHT)
						AddBox(mousePos, 1, 30, 10);
					else if (ev.button.button == SDL_BUTTON_LEFT && !mouseJoint)
					{
						if (const auto* shape = physicsScene.QueryPoint(mousePos))
						{
							if (const auto body = physicsScene.GetShapeBody(shape); body != ground)
							{
								mouseJoint = new PinJoint(physicsScene.bodiesSoA, ground, body, mousePos);
								physicsScene.AddConstraint(mouseJoint);
							}
						}
					}
				}
				else if (ev.type == SDL_EVENT_MOUSE_BUTTON_UP)
				{
					if (ev.button.button == SDL_BUTTON_LEFT && mouseJoint)
					{
						physicsScene.RemoveConstraint(mouseJoint);
						delete mouseJoint;
						mouseJoint = nullptr;
					}
				}
			}

			// Render
			SDL_SetRenderDrawColor(renderer, 15, 31, 31, 255);
			SDL_RenderClear(renderer);

			vertices.clear();
						
			RenderLine(mousePos + vec2(6, 6), mousePos, DebugColors::White);

			if (getElapsedTime() > prevUpdateTime + iterationTime)
			{
				prevUpdateTime += iterationTime;

				physicsScene.SimpleStep(1 / 120.f);

				for (const auto& aabb : physicsScene.broadPhasePools.boundingBoxes)
				{
					auto sp = WorldToScreen(aabb.first.min);
					if (sp.x > 0 && sp.x < 1024 && sp.y > 0 && sp.y < 768)
						RenderAabb(aabb.first);
				}

				for (const auto& circleShape : physicsScene.circleShapes)
				{
					auto sp = WorldToScreen(circleShape.second->transformedOffset);
					if (sp.x > 0 && sp.x < 1024 && sp.y > 0 && sp.y < 768)
					{
						// render high resolution circles if scene is simple
						if (physicsScene.bodiesSoA.size() < 100)
							RenderCircle<32>(circleShape.second->transformedOffset, circleShape.second->radius, ptrToFillColor((uintptr_t)circleShape.second));
						else
							RenderCircle<16>(circleShape.second->transformedOffset, circleShape.second->radius, ptrToFillColor((uintptr_t)circleShape.second));

						const auto& transform = physicsScene.bodiesSoA.getPoolForTransform()[circleShape.second->body];
						RenderLine(
							circleShape.second->transformedOffset,
							transform.transformPoint(vec2(circleShape.second->radius, 0)),
							DebugColors::Outline);
					}
				}

				for (const auto& polyShape : physicsScene.polyShapes)
				{
					auto sp = WorldToScreen(polyShape.second->transformedCenter);
					if (sp.x > 0 && sp.x < 1024 && sp.y > 0 && sp.y < 768)
						RenderPolygon(polyShape.second->transformedLines, polyShape.second->radius, ptrToFillColor((uintptr_t)polyShape.second),
							physicsScene.bodiesSoA.size() < 100); // only render pretty corners if scene is simple
				}

				// Draw center of mass
				auto* transforms = physicsScene.bodiesSoA.getPoolForTransform();
				auto* cogs = physicsScene.bodiesSoA.getPoolForPositional();
				for (size_t i = 0; i < physicsScene.bodiesSoA.size(); ++i)
				{
					auto& transform = transforms[i];
					auto& cog = cogs[i];
					auto transformedCog = transform.transformPoint(cog.centerOfMass);
					RenderLine(transformedCog, transformedCog + vec2(0, 3), DebugColors::Green);
					RenderLine(transformedCog, transformedCog + vec2(3, 0), DebugColors::Red);
				}

				// Draw contacts
				physicsScene.arbiterPools.foreachActive([&](const Arbiter& arb)
					{
						auto& transformA = physicsScene.bodiesSoA.getPoolForTransform()[arb.bodyA];
						for (int i = 0; i < arb.manifold.count; ++i)
						{
							auto hash = orderIndependentHash<uintptr_t>(orderIndependentHash<uintptr_t>(arb.bodyA, arb.bodyB), i);
							contactsToRender.insert_or_assign(hash, std::pair{ transformA.translation + arb.manifold.contacts[i].offsetA , 3 });
						}
					});

				std::erase_if(contactsToRender, [&](std::pair<uintptr_t, std::pair<vec2, real>> pair)
					{
						if (pair.second.second <= 0)
							return true;

						RenderCircle<4>(pair.second.first, pair.second.second, toDebugColor(DebugColors::ActiveArbiter));
						return false;
					});


				for (auto& pair : contactsToRender)
				{
					pair.second.second -= 0.1f;
				}

				for (const auto& pair : physicsScene.arbiterPools.cached)
				{
					auto& arb = physicsScene.arbiterPools.storage[pair.second];

					for (int i = 0; i < arb.manifold.count; ++i)
					{
						auto& transformA = physicsScene.bodiesSoA.getPoolForTransform()[arb.bodyA];
						auto& transformB = physicsScene.bodiesSoA.getPoolForTransform()[arb.bodyB];

						RenderLine(
							transformA.translation + arb.manifold.contacts[i].offsetA,
							transformB.translation + arb.manifold.contacts[i].offsetB,
							DebugColors::CachedContact
						);
					}
				}

				for (const auto& constraint : physicsScene.constraints)
				{
					auto* pin = (PinJoint*)constraint;

					auto& bodyA = physicsScene.bodiesSoA.getPoolForTransform()[constraint->a];
					auto& bodyB = physicsScene.bodiesSoA.getPoolForTransform()[constraint->b];

					auto anchorA = bodyA.transformPoint(pin->anchorA);
					auto anchorB = bodyB.transformPoint(pin->anchorB);

					RenderLine(anchorA, anchorB, DebugColors::JointLink);
					RenderCircle<4>(anchorA, 3, toDebugColor(DebugColors::Joint));
					RenderCircle<4>(anchorB, 3, toDebugColor(DebugColors::JointDarker));
				}
			}

			SDL_RenderPresent(renderer);
		}
	}

	vec2 WorldToScreen(const vec2& input) const
	{
		return vec2{
			 input.x * camera.scale.x - camera.position.x,
			 input.y * camera.scale.y - camera.position.y,
		};
	}

	template<int sides>
	void RenderCircle(const vec2& center, float radius, const debugColor& color)
	{
		SDL_Vertex vert[sides * 3];

		for (int i = 0; i < sides; ++i)
		{
			const auto circlePoint = center + glm::rotate(vec2{ 0, radius }, const_tau * i / sides);
			const auto nextCirclePoint = center + glm::rotate(vec2{ 0, radius }, const_tau * (i + 1) / sides);

			SetVertex(vert[i * 3 + 0], WorldToScreen(nextCirclePoint), color);
			SetVertex(vert[i * 3 + 1], WorldToScreen(circlePoint), color);
			SetVertex(vert[i * 3 + 2], WorldToScreen(center), color);
		}

		SDL_RenderGeometry(renderer, nullptr, vert, sides * 3, nullptr, 0);
	}

	void RenderPolygon(const std::vector<SplittingLine>& planes, real radius, const debugColor& color, bool roundCorners = false)
	{
		std::vector<SDL_Vertex> verts{};
		verts.reserve(planes.size() * 6);

		auto count = planes.size();
		for (int i = 0; i < count; i++)
		{
			auto& plane = planes[i];
			auto& nextPlane = planes[(i + 1) % count];
			auto& prevPlane = planes[(i - 1) % count];

			auto p = plane.point + plane.normal * radius;
			auto pp = prevPlane.point + plane.normal * radius;
			auto np = prevPlane.point + prevPlane.normal * radius;
			auto bp = nextPlane.point;

			if (roundCorners)
				RenderCircle<16>(plane.point, radius, color);

			verts.emplace_back(CreateVertex(WorldToScreen(p), color));
			verts.emplace_back(CreateVertex(WorldToScreen(pp), color));
			verts.emplace_back(CreateVertex(WorldToScreen(np), color));

			verts.emplace_back(CreateVertex(WorldToScreen(p), color));
			verts.emplace_back(CreateVertex(WorldToScreen(np), color));
			verts.emplace_back(CreateVertex(WorldToScreen(bp), color));
		}

		SDL_RenderGeometry(renderer, nullptr, verts.data(), (int)verts.size(), nullptr, 0);
	}

	void RenderAabb(const Aabb& aabb)
	{
		auto bl = WorldToScreen(aabb.min);
		auto tr = WorldToScreen(aabb.max);

		SDL_FPoint points[5];

		points[0].x = points[4].x = bl.x;
		points[0].y = points[4].y = bl.y;

		points[1].x = bl.x;
		points[1].y = tr.y;

		points[2].x = tr.x;
		points[2].y = tr.y;

		points[3].x = tr.x;
		points[3].y = bl.y;

		auto& color = DebugColors::BroadphaseAabb;
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderLines(renderer, points, 5);
	}

	void RenderPoint(const vec2& point)
	{
		auto p = WorldToScreen(point);
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
		SDL_RenderPoint(renderer, p.x, p.y);
	}

	void RenderLine(const vec2& start, const vec2& end, const SDL_Color color)
	{
		auto convStart = WorldToScreen(start);
		auto convEnd = WorldToScreen(end);
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderLine(renderer, convStart.x, convStart.y, convEnd.x, convEnd.y);
	}
};
