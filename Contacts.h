#pragma once

#include "glm/glm.hpp"

using namespace glm;

static constexpr int MAX_POINTS_PER_OVERLAP = 2;

struct ContactPoint
{
	// TODO this is not really "local" as it's just transposed, not rotated or scaled... maybe rename?
	vec2 localPointA, localPointB, normal;
	bool isMerged, isNew;

	int solverIndex;

	ContactPoint() = default;
	ContactPoint(const ContactPoint& other) = default;
	ContactPoint(ContactPoint&& other) noexcept = default;
	ContactPoint& operator=(const ContactPoint& other) = default;
	ContactPoint& operator=(ContactPoint&& other) noexcept = default;

	ContactPoint(const vec2& bodyPositionA, const vec2& worldPointA, const vec2& bodyPositionB, const vec2& worldPointB, const vec2& normal) :
		localPointA(worldPointA - bodyPositionA),
		localPointB(worldPointB - bodyPositionB),
		normal(normal),
		isMerged(false),
		isNew(true),
		solverIndex(-1) // TODO
	{}
};

struct Contact
{
	Contact(int body1Index, int body2Index, int collisionIndex)
	{
		this->contactPointIndex = collisionIndex;
		this->body1Index = body1Index;
		this->body2Index = body2Index;

		normalLimiterTotalImpulse = 0.f;
		frictionLimiterTotalImpulse = 0.f;
	}

	int contactPointIndex;
	int body1Index;
	int body2Index;
	float normalLimiterTotalImpulse;
	float frictionLimiterTotalImpulse;
};


struct ContactLimiterWide
{
	float normalProjector1X[8];
	float normalProjector1Y[8];
	float normalProjector2X[8];
	float normalProjector2Y[8];
	float angularProjector1[8];
	float angularProjector2[8];
	float linearMass1X[8];
	float linearMass1Y[8];
	float linearMass2X[8];
	float linearMass2Y[8];
	float angularMass1[8];
	float angularMass2[8];
	float invMass[8];
};

struct ContactJointWide
{
	int body1Index[8];
	int body2Index[8];
	int contactPointIndex[8];

	ContactLimiterWide normalLimiter;
	float normalLimiterInvMass[8];
	float normalLimiterTotalImpulse[8];
	float normalLimiterTotalDisplacingImpulse[8];
	float normalLimiterTargetVelocity[8];
	float normalLimiterTargetDisplacingVelocity[8];

	ContactLimiterWide frictionLimiter;
	float frictionLimiterTotalImpulse[8];
};

// An overlap represented by a number of contact points
class Overlap
{

public:

	int pointsCount;

	const int firstEntityIndex, secondEntityIndex, pointsStartIndex;

	Overlap(int firstEntityIndex, int secondEntityIndex, int pointsRangeIndex) :
		pointsCount(0),
		firstEntityIndex(firstEntityIndex),
		secondEntityIndex(secondEntityIndex),
		pointsStartIndex(pointsRangeIndex* MAX_POINTS_PER_OVERLAP)
	{ }

	int AddPointAndReturnIndex() // maybe this could be clearer by being explicit? e.g.: if HasSpace AddPoint
	{
		pointsCount++;
		//if (pointsCount < MAX_POINTS_PER_OVERLAP) // TODO
		/*{
			return true;
		}
		return false;*/
		return pointsStartIndex + pointsCount - 1;
	}

	int GetPointCount() const { return pointsCount; }
	void ResetPointCount() { pointsCount = 0; }
};