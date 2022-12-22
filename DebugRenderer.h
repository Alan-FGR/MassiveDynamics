#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>

#include "glm/glm.hpp"
#include "glm/gtx/compatibility.hpp"
#include "glm/ext.hpp"

#define SDL_MAIN_HANDLED
#include "sdl2/SDL.h"
#include "SDL_ttf.h"

#include "PhysicsWorld.h"

using namespace glm;

struct Vertex
{
	vec2 position;
	unsigned char r, g, b, a;
};

static void SetVertex(SDL_Vertex& vert, const vec2& position, const Uint8& color)
{
	vert.position.x = position.x;
	vert.position.y = position.y;
	std::memcpy(&vert.color, &color, sizeof Uint8 * 3);
};

class DebugRenderer
{
	PhysicsWorld& physicsWorld;

	SDL_Window* window;
	SDL_Renderer* renderer;

	TTF_Font* Font;

	Transform camera{ vec2{-400,-300}, vec2{1, -1}, mat2x2{vec2{1,0},vec2{0,1}} };
	std::vector<Vertex> vertices;

	bool shouldQuit = false;

	static double getElapsedTime() {
		return SDL_GetPerformanceCounter() / (double)SDL_GetPerformanceFrequency();
	}

	const static struct DebugColors
	{
		constexpr static SDL_Color BroadphaseAabb{ 255,0,0,255 };
		constexpr static SDL_Color BroadphasePair{ 255,255,0,255 };
		constexpr static SDL_Color SortingPointer{ 255,0,0,127 };
	};

public:
	explicit DebugRenderer(PhysicsWorld& physics_world)
		: physicsWorld(physics_world)
	{
		std::cout << "Starting debug renderer...\n";

		window = SDL_CreateWindow(
			"Massive Dynamics Debug Renderer",
			SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			800, 600, 0
		);

		SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
		renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
		SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

		TTF_Init();
		Font = TTF_OpenFont("CascadiaMono.ttf", 8);
		if (Font == nullptr)
			std::cout << TTF_GetError() << "\n";
	}

	DebugRenderer(const DebugRenderer& other) = delete;
	DebugRenderer& operator=(const DebugRenderer& other) = delete;

	~DebugRenderer()
	{
		std::cout << "Destroying debug renderer...\n";
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
	}
	
	void RunLoop()
	{
		double prevUpdateTime = 0.0f;

		while (!shouldQuit)
		{
			SDL_SetRenderDrawColor(renderer, 31, 31, 0, 255);
			SDL_RenderClear(renderer);

			const float iterationTime = 1 / 30.f;
			float deltaTime = iterationTime;//TODO calc from elapsed

			const float cameraSpeed = 250;

			vertices.clear();

			/*physicsWorld.ForEntity([&](
				EntityId id,
				const DynamicProperties& dynamicProperties,
				const vec2& shapeSize,
				const Shape& shapeData
				)
				{
					Transform t;

					t.position = dynamicProperties.position;
					t.orientation = dynamicProperties.orientation;
					t.scale = shapeSize;

					Uint8 col[3];
					col[0] = ((id + 0) * 40) % 128;
					col[1] = ((id + 1) * 40) % 128;
					col[2] = ((id + 2) * 40) % 128;

					RenderBox(t, 0, col[0]);
				});

			physicsWorld.ForEntity([&](
				EntityId id,
				const DynamicProperties& dynamicProperties,
				const vec2& shapeSize,
				const Shape& shapeData
				)
				{
					RenderAabb(shapeData.aabb);
				});*/

			if (getElapsedTime() > prevUpdateTime + iterationTime)
			{
				prevUpdateTime += iterationTime;

				// COLLISION

				// Update AABBs
				auto aabbs = physicsWorld.UpdateAABBs();
				for (const auto& aabb : aabbs)
				{
					RenderAabb(aabb);
				}

				// Sort AABBs
				auto sortedAabbs = physicsWorld.SortAabbs(aabbs);
				for (int i = 0; i < sortedAabbs.size() - 1; ++i)
				{
					const auto& current = sortedAabbs[i].originalAabb;
					const auto& next = sortedAabbs[i+1].originalAabb;
					
					RenderLine(
						lerp(current.min, current.max, .5f),
						lerp(next.min, next.max, .5f),
						DebugColors::SortingPointer
					);
				}

				// Create AABBs pairs
				auto pairs = physicsWorld.CreatePairs(sortedAabbs);
				for (const auto& overlap : pairs)
				{
					const auto& bodyA = physicsWorld.pools.dynamicProperties[overlap.firstEntityIndex];
					const auto& bodyB = physicsWorld.pools.dynamicProperties[overlap.secondEntityIndex];

					const auto& aabbA = aabbs[overlap.firstEntityIndex];
					const auto& aabbB = aabbs[overlap.secondEntityIndex];

					RenderLine(aabbA.min, aabbB.min, DebugColors::BroadphasePair);
				}

				// Calculate manifolds (contact points)
				/*auto overlaps = */physicsWorld.CreateOverlaps(pairs);

				// Create joints for contact points

				// IMPULSE SOLVING

				// Solve joints

				// INTEGRATION

				// Apply velocities

				// Apply positions

			}
			
			SDL_RenderPresent(renderer);

			// Handle input
			SDL_Event ev;
			while (SDL_PollEvent(&ev) != 0)
			{
				switch (ev.type)
				{
				case SDL_QUIT:
					shouldQuit = true;
					break;
				case SDL_KEYDOWN:

					if (ev.key.keysym.sym == SDLK_ESCAPE)
						shouldQuit = true;

					// Camera controls
					else if (ev.key.keysym.sym == SDLK_w) camera.position += vec2(0, 1) * deltaTime * -cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_s) camera.position -= vec2(0, 1) * deltaTime * -cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_d) camera.position += vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_a) camera.position -= vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_e) camera.scale += vec2(1, -1) * deltaTime * cameraSpeed * 0.01f;
					else if (ev.key.keysym.sym == SDLK_q) camera.scale -= vec2(1, -1) * deltaTime * cameraSpeed * 0.01f;

					break;
				}
			}
		}
	}

	vec2 WorldToScreen(const vec2& input) const
	{
		return vec2{
			 input.x * camera.scale.x - camera.position.x,
			 input.y * camera.scale.y - camera.position.y,
		};
	}

	void RenderBox(const Transform& transform, ShapeType shapeId, const Uint8& color)
	{
		vec2 axisX = transform.orientation[0] * transform.scale.x;
		vec2 axisY = transform.orientation[1] * transform.scale.y;

		auto bl = WorldToScreen(transform.position - axisX - axisY);
		auto br = WorldToScreen(transform.position + axisX - axisY);
		auto tr = WorldToScreen(transform.position + axisX + axisY);
		auto tl = WorldToScreen(transform.position - axisX + axisY);
		
		SDL_Vertex vert[6];

		SetVertex(vert[0], bl, color);
		SetVertex(vert[1], br, color);
		SetVertex(vert[2], tr, color);
		SetVertex(vert[3], bl, color);
		SetVertex(vert[4], tl, color);
		SetVertex(vert[5], tr, color);
		SDL_RenderGeometry(renderer, nullptr, vert, 6, nullptr, 0);
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
		SDL_RenderDrawLinesF(renderer, points, 5);

		RenderText(aabb.min, aabb.min.x, aabb.max.x, "|", aabb.min.y, aabb.max.y);
	}

	void RenderPoint(const vec2& point)
	{
		auto p = WorldToScreen(point);
		// p.y *= -1;
		SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
		SDL_RenderDrawPointF(renderer, p.x, p.y);
	}

	void RenderLine(const vec2& start, const vec2& end, const SDL_Color color)
	{
		auto convStart = WorldToScreen(start);
		auto convEnd = WorldToScreen(end);
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderDrawLineF(renderer, convStart.x, convStart.y, convEnd.x, convEnd.y);
	}
	
	void print(std::ostringstream& stream) {}
	template <typename T> void print(std::ostringstream& stream, const T& t) {
		stream << t;
	}
	template <typename First, typename... Rest> void print(std::ostringstream& stream, const First& first, const Rest&... rest) {
		stream << first << ",";
		print(stream, rest...); // recursive call using pack expansion syntax
	}

	template<typename... Ts>
	void RenderText(const vec2& position, Ts const&... ts)
	{
		auto screenPos = WorldToScreen(position);
		std::ostringstream stringStream;
		stringStream << std::fixed << std::setprecision(0);
		print(stringStream, ts...);
		auto string = stringStream.str();
		if (string.size() <= 0) return;
		SDL_Surface* surface = TTF_RenderText_Solid(Font, string.c_str(), SDL_Color{ 255, 255, 255, 191 });
		SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
		const SDL_Rect rect = { (int)screenPos.x, (int)screenPos.y, surface->w, surface->h };
		SDL_RenderCopy(renderer, texture, nullptr, &rect);
		SDL_FreeSurface(surface);
		SDL_DestroyTexture(texture);
	}

};
