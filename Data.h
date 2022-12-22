#pragma once

#include "glm/glm.hpp"
#include "glm/ext.hpp"

#include "AvxVector.h"
#include "Shape.h"
#include "TypeAliases.h"

struct DynamicProperties
{
    float massInverse;
    float inertiaInverse;
    vec2 position;
    mat2x2 orientation;
};

struct DataPools
{
	DataPools() = default;
	DataPools(const DataPools& other) = delete;
	DataPools& operator=(const DataPools& other) = delete;
	DataPools(DataPools&& other) noexcept = default;
	DataPools& operator=(DataPools&& other) noexcept = default;

    avx_vector<DynamicProperties> dynamicProperties;

    avx_vector<vec2> shapeSize;
    avx_vector<ShapeType> shapeType;
    avx_vector<Shape> shapeData;

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
    
    EntityId AddSimulationEntry(const DynamicProperties& dynamicProperties_, const vec2& scale_, ShapeType shapeType_)
    {
        auto newId = dynamicProperties.size();
        
        dynamicProperties.emplace_back(dynamicProperties_);

        shapeSize.emplace_back(scale_);
        shapeType.emplace_back(shapeType_);
        shapeData.emplace_back(Shape{ scale_, Aabb{} });

        acceleration.emplace_back(vec2(0.f, 0.f));
        angularAcceleration.emplace_back(0.f);

        velocity.emplace_back(vec2(0.f, 0.f));
        displacingVelocity.emplace_back(vec2(0.f, 0.f));
        angularVelocity.emplace_back(0.f);
        displacingAngularVelocity.emplace_back(0.f);

        massInverse.emplace_back(dynamicProperties_.massInverse);
        inertiaInverse.emplace_back(dynamicProperties_.inertiaInverse);

        entities.emplace_back(newId);

        return newId;
    }
};