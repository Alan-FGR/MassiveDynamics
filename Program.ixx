import <iostream>;
import <glm/glm.hpp>;

using EntityId = unsigned int;

import "glm/glm.hpp";
import "glm/ext.hpp";

import DebugRenderer;
import Transform;
import PhysicsWorld;

float random(float min, float max)
{
    return min + (max - min) * (float(rand()) / float(RAND_MAX));
}

int main()
{
	std::cout << "Starting Demo...\n";
    
    PhysicsWorld physicsWorld;

    for (int bodyIndex = 0; bodyIndex < 30; bodyIndex++)
    {
        Transform t;

        t.position = glm::vec2(random(-40.0f, 40.0f), random(10.f, 90.0f));
        const auto length = random(2.0f, 6.0f);
        t.scale = glm::vec2(length, 8 - length);
        
        physicsWorld.AddEntity(t, 0);
    }

	DebugRenderer debugRenderer(physicsWorld);
    debugRenderer.RunLoop();

	return 0;
}
