export module Dynamics;

using EntityId = unsigned int;

import AvxVector;

import "glm/glm.hpp";
import "glm/ext.hpp";

using namespace glm;

export struct DynamicProperties
{
    float massInverse;
    float inertiaInverse;
    vec2 position;
    mat2x2 orientation;
};

export struct DataPools
{
	DataPools() = default;
	DataPools(const DataPools& other) = delete;
	DataPools& operator=(const DataPools& other) = delete;
	DataPools(DataPools&& other) noexcept = default;
	DataPools& operator=(DataPools&& other) noexcept = default;

    avx_vector<DynamicProperties> dynamicProperties;

    avx_vector<vec2> acceleration;
    avx_vector<float> angularAcceleration;

    avx_vector<vec2> velocity;
    avx_vector<vec2> displacingVelocity;
    avx_vector<float> angularVelocity;
    avx_vector<float> displacingAngularVelocity;

    //TODO reuse these in dynamicProperties
    avx_vector<float> massInverse;
    avx_vector<float> inertiaInverse;

    avx_vector<EntityId> entities;
    
    EntityId AddSimulationEntry(DynamicProperties& dynamicProperties_)
    {
        auto newId = dynamicProperties.size();

        dynamicProperties.emplace_back(dynamicProperties_);

        acceleration.emplace_back(vec2(0.f, 0.f));
        angularAcceleration.push_back(0.f);

        velocity.emplace_back(vec2(0.f, 0.f));
        displacingVelocity.emplace_back(vec2(0.f, 0.f));
        angularVelocity.push_back(0.f);
        displacingAngularVelocity.push_back(0.f);

        massInverse.push_back(dynamicProperties_.massInverse);
        inertiaInverse.push_back(dynamicProperties_.inertiaInverse);

        entities.push_back(newId);

        return newId;
    }
};