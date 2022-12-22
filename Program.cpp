#pragma once

#include <iostream>

#include "glm/glm.hpp"
#include "glm/ext.hpp"

#include "DebugRenderer.h"
#include "Transform.h"
#include "PhysicsWorld.h"

float random(float min, float max)
{
    return min + (max - min) * (float(rand()) / float(RAND_MAX));
}

int main()
{
	std::cout << "Starting Demo...\n";
    
    PhysicsWorld physicsWorld;

    // auto floor = Transform{ glm::vec2{0, -10}, glm::vec2{400.f, 4.f} };
    // physicsWorld.AddEntity(floor, 0);
    //
    // for (int bodyIndex = 0; bodyIndex < 30; bodyIndex++)
    // {
    //     Transform t;
    //
    //     t.position = glm::vec2(random(-140.0f, 140.0f), random(10.f, 90.0f));
    //     const auto length = random(2.0f, 6.0f);
    //     t.scale = glm::vec2(length, 8 - length);
    //     t.SetOrientationAngle(random(0, 2));
    //     
    //     physicsWorld.AddEntity(t, 0);
    // }

    physicsWorld.AddEntity(Transform{ glm::vec2{-55, -45}, glm::vec2{100, 100} }, 0);
    physicsWorld.AddEntity(Transform{ glm::vec2{70, 60}, glm::vec2{100, 100} }, 0);

	DebugRenderer debugRenderer(physicsWorld);
    debugRenderer.RunLoop();

	return 0;
}
