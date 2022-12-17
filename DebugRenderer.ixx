export module DebugRenderer;

import <iostream>;

#define SDL_MAIN_HANDLED
#include "SDL.h"

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
	}	
};
