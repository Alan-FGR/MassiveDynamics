export module PhysicsWorld;

import <algorithm>;
import <memory>;

import TypeAliases;
import AvxVector;
import Transform;
import Dynamics;
import Shape;

export class PhysicsWorld
{
    DataPools pools;

public:
    PhysicsWorld()
    {

    }

    PhysicsWorld(const PhysicsWorld& other) = delete;
    PhysicsWorld& operator=(const PhysicsWorld& other) = delete;
    PhysicsWorld(PhysicsWorld&& other) noexcept = default;
    PhysicsWorld& operator=(PhysicsWorld&& other) noexcept = default;

    ~PhysicsWorld()
    {
	    
    }

    int EntityCount()
    {
	    return pools.dynamicProperties.size();
    }

    void ForEntity(auto func)
    {
	    for (int i = 0; i < pools.dynamicProperties.size(); ++i)
	    {
            func(pools.entities[i],
				pools.dynamicProperties[i],
                pools.shapeSize[i],
                pools.shapeData[i]
                //TODO pass ptrs to other stuff that makes sense (e.g. shape)
            );
	    }
    }

    EntityId AddEntity(Transform& transform, ShapeType shapeType, bool isKinematic = false)
    {
        auto density = 1e-5f;

        //TODO LP memcpy
        DynamicProperties properties;
        properties.position = transform.position;
        properties.orientation = transform.orientation;
        
        //TODO calc mass from transform and shape
        auto calculatedMassInverse = 0.5f;
        auto calculatedInertiaInverse = 0.5f;
        
        properties.massInverse = isKinematic ? 0 : calculatedMassInverse;
        properties.inertiaInverse = isKinematic ? 0 : calculatedInertiaInverse;

        return pools.AddSimulationEntry(properties, transform.scale, shapeType);
    }

    void Update(float iterationTime)
    {

        // COLLISION

        // Update AABBs
        for (int i = 0; i < pools.dynamicProperties.size(); ++i)
        {
            pools.shapeData[i].aabb = Aabb::CalculateFrom(
                pools.dynamicProperties[i].position,
                pools.dynamicProperties[i].orientation,
                pools.shapeSize[i]
            );
        }

        // Calculate broad phase pairs

        // Calculate manifolds (contact points)

        // Create joints for contact points

        // IMPULSE SOLVING

        // Solve joints

        // INTEGRATION

        // Apply velocities

        // Apply positions
        
    }

};
