export module DebugRenderer;

import <iostream>;

import Transform;

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

void SetVertex(SDL_Vertex& vert, const vec2 position, const Uint8* color)
{
	vert.position.x = position.x;
	vert.position.y = position.y;
	std::memcpy(&vert.color, color, sizeof Uint8 * 3);
}

export class DebugRenderer
{

public:

	DebugRenderer()
	{
		std::cout << "Starting debug renderer...\n";
		// INIT SDL
		SDL_bool quit = SDL_FALSE;
		SDL_Window* window = SDL_CreateWindow("Massive Dynamics Debug Renderer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, 0);
		SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
		SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	}

	DebugRenderer(const DebugRenderer& other) = delete;
	DebugRenderer& operator=(const DebugRenderer& other) = delete;

	~DebugRenderer()
	{
		std::cout << "Destroying debug renderer...\n";
		//TODO
	}

	void RenderBox(SDL_Renderer* renderer, const Transform& camera, const Transform& transform, const Uint8* color)
	{
		vec2 fd = vec2(3, 4) * 3.f;

		auto tf = Transform();
		auto x = tf.position * 2.0f;

		vec2 axisX = transform.matX * transform.scale.x;
		vec2 axisY = transform.matY * transform.scale.y;

		auto bl = (transform.position - axisX - axisY) * camera.scale.x - camera.position;
		bl.y *= -1;
		auto br = (transform.position + axisX - axisY) * camera.scale.x - camera.position;
		br.y *= -1;
		auto tr = (transform.position + axisX + axisY) * camera.scale.x - camera.position;
		tr.y *= -1;
		auto tl = (transform.position - axisX + axisY) * camera.scale.x - camera.position;
		tl.y *= -1;

		SDL_Vertex vert[6];

		// center
		SetVertex(vert[0], bl, color);
		SetVertex(vert[1], br, color);
		SetVertex(vert[2], tr, color);
		SetVertex(vert[3], bl, color);
		SetVertex(vert[4], tl, color);
		SetVertex(vert[5], tr, color);

		SDL_RenderGeometry(renderer, NULL, vert, 6, NULL, 0);
	}
};
