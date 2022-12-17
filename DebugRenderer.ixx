export module DebugRenderer;

import <iostream>;
import <vector>;

import TypeAliases;
import Transform;
import PhysicsWorld;
import Dynamics;

import "glm/glm.hpp";
import "glm/ext.hpp";

#define SDL_MAIN_HANDLED

#include "sdl2/SDL.h"

using namespace glm;

struct Vertex
{
	vec2 position;
	unsigned char r, g, b, a;
};

void SetVertex(SDL_Vertex& vert, const vec2& position, const Uint8& color)
{
	vert.position.x = position.x;
	vert.position.y = position.y;
	std::memcpy(&vert.color, &color, sizeof Uint8 * 3);
}

export class DebugRenderer
{
	PhysicsWorld& physicsWorld;

	SDL_Window* window;
	SDL_Renderer* renderer;

	Transform camera{ vec2{-400,-500}, vec2{-4}, mat2x2{vec2{1,0},vec2{0,1}} };
	std::vector<Vertex> vertices;

	bool shouldQuit = false;

	static double getElapsedTime() {
		return SDL_GetPerformanceCounter() / (double)SDL_GetPerformanceFrequency();
	}

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
		while (!shouldQuit)
		{
			SDL_SetRenderDrawColor(renderer, 31, 31, 0, 255);
			SDL_RenderClear(renderer);

			const float iterationTime = 1 / 30.f;
			float deltaTime = iterationTime;//TODO calc from elapsed

			const float cameraSpeed = 250;

			vertices.clear();

			physicsWorld.ForEntity([&](
				EntityId id,
				const DynamicProperties& dynamicProperties,
				const vec2& shapeSize)
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

			SDL_RenderPresent(renderer); // Debug draw

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
					else if (ev.key.keysym.sym == SDLK_w) camera.position += vec2(0, 1) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_s) camera.position -= vec2(0, 1) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_d) camera.position += vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_a) camera.position -= vec2(1, 0) * deltaTime * cameraSpeed;
					else if (ev.key.keysym.sym == SDLK_e) camera.scale += vec2(1, 1) * deltaTime * cameraSpeed * 0.01f;
					else if (ev.key.keysym.sym == SDLK_q) camera.scale -= vec2(1, 1) * deltaTime * cameraSpeed * 0.01f;

					break;
				}
			}
		}
	}

	void RenderBox(const Transform& transform, ShapeType shapeId, const Uint8& color)
	{
		std::cout << camera.position.x << "\n";

		vec2 axisX = transform.orientation[0] * transform.scale.x;
		vec2 axisY = transform.orientation[1] * transform.scale.y;

		auto bl = (transform.position - axisX - axisY) * camera.scale.x - camera.position;
		auto br = (transform.position + axisX - axisY) * camera.scale.x - camera.position;
		auto tr = (transform.position + axisX + axisY) * camera.scale.x - camera.position;
		auto tl = (transform.position - axisX + axisY) * camera.scale.x - camera.position;
		//bl.y *= -1;
		//br.y *= -1;
		//tr.y *= -1;
		//tl.y *= -1;

		SDL_Vertex vert[6];

		SetVertex(vert[0], bl, color);
		SetVertex(vert[1], br, color);
		SetVertex(vert[2], tr, color);
		SetVertex(vert[3], bl, color);
		SetVertex(vert[4], tl, color);
		SetVertex(vert[5], tr, color);
		SDL_RenderGeometry(renderer, nullptr, vert, 6, nullptr, 0);

		// SDL_FRect r;
		// r.w = 2;
		// r.h = 2;
		// r.x = tl.x;
		// r.y = tl.y;
		// SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
		// SDL_RenderFillRectF(renderer, &r);

	}
};
