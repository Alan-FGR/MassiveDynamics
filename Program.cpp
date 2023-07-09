#include <iostream>

#include "glm/glm.hpp"
#include "glm/ext.hpp"

#include "DebugRenderer.h"
#include "Transform.h"

float random(float min, float max)
{
    return min + (max - min) * (float(rand()) / float(RAND_MAX));
}

int main()
{
	std::cout << "Starting Demo...\n";
    
    PhysicsWorld physicsWorld;

    auto floor = Transform{ glm::vec2{0, -200}, glm::vec2{400.f, 4.f} };
    physicsWorld.AddEntity(floor, 0, true);
    
	 //for (int bodyIndex = 0; bodyIndex < 30; bodyIndex++)
	 //{
	 //    Transform t;

	 //    t.position = glm::vec2(random(-40.0f, 40.0f), random(10.f, 90.0f));
	 //    const auto length = random(2.0f, 6.0f);
	 //    t.scale = glm::vec2(length, 8 - length);
	 //    t.SetOrientationAngle(random(0, 0));
	 //    
	 //    physicsWorld.AddEntity(t, 0);
	 //}

    physicsWorld.AddEntity(Transform{ glm::vec2{-5, -14}, glm::vec2{10, 10} }, 0);
    physicsWorld.AddEntity(Transform{ glm::vec2{7, 10}, glm::vec2{10, 10} }, 0);

	DebugProgram debugRenderer(physicsWorld);
    debugRenderer.RunLoop();

	return 0;
}
