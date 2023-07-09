#pragma once

#include "glm/glm.hpp"
#include "glm/gtx/component_wise.hpp"

#include <vector>

#include "AvxOps.h"
#include "AvxVector.h"
#include "Data.h"

#include "Contacts.h"

using namespace glm;

struct SolverBody
{
	vec2 velocity;
	float angularVelocity;
	int lastIteration;
};

struct SolverData
{
	AvxVector<SolverBody> Impulse;
	AvxVector<SolverBody> Displacement;
};

static constexpr float IMPULSE_DISCARD_THRESHOLD = 0.005f;
static constexpr float FRICTION_COEFFICIENT = 0.3f;

namespace ImpulseSolver {
	AvxVector<Contact> CreateJoints(const std::vector<Overlap>& overlaps, std::vector<ContactPoint>& contacts);
	void ProcessJoints(int bodiesCount, ContactPoint* contactPoints, const AvxVector<DynamicProperties>& dynamicProperties, AvxVector<Contact>& contactJoints, SolverData& solverData);
	void UpdateJoints(const ContactJointWide* jointPacked, int jointBegin, int jointEnd, AvxVector<SolverBody>& solveBodiesImpulse);
	char ProcessJointsImpulses(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex, AvxVector<SolverBody>& solveBodiesImpulse);
	void UpdateJointScalarContactWide(const ContactJointWide* jointPacked, int jointBegin, int jointEnd, AvxVector<SolverBody>& solveBodiesImpulse);
	bool ProcessJointImpulseScalarContactWide(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex, AvxVector<SolverBody>& solveBodiesImpulse);
	bool ProcessJointsDisplacement(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex, AvxVector<SolverBody>& solveBodiesDisplacement);
	bool ProcessJointDisplacementScalarContactWide(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex, AvxVector<SolverBody>& solveBodiesDisplacement);
	void FinishJoints(const AvxVector<ContactJointWide>& jointPacked, int jointBegin, int jointEnd, const AvxVector<int>& jointIndex, AvxVector<Contact>& contactJoints);
	void ProcessJointIsland(AvxVector<ContactJointWide>& jointPacked, int jointBegin, int jointEnd, ContactPoint* contactPoints, AvxVector<int>& jointGroupJoints, AvxVector<int>& jointIndex, AvxVector<int>& jointGroupBodies, AvxVector<Contact>& contactJoints, SolverData& solverData, const AvxVector	<DynamicProperties>& dynamicProperties);
	int PrepareJoints(AvxVector<ContactJointWide>& jointPacked, int jointBegin, int jointEnd, int groupSizeTarget, AvxVector<int>& jointGroupJoints, AvxVector<int>& jointIndex, AvxVector<int>& jointGroupBodies, const AvxVector<Contact>& contactJoints);
	int PrepareIndices(int jointBegin, int jointEnd, int groupSizeTarget, AvxVector<int>& jointGroupJoints, AvxVector<int>& jointIndex, AvxVector<int>& jointGroupBodies, const AvxVector<Contact>& contactJoints);
	void RefreshJoints(ContactJointWide* jointPacked, int jointBegin, int jointEnd, ContactPoint* contactPoints, AvxVector<SolverBody>& solvedBodies, const	AvxVector<DynamicProperties>& dynamicProperties);
	void RefreshJointScalarContactWide(ContactJointWide* jointPacked, int jointBegin, int jointEnd, ContactPoint* contactPoints, const AvxVector<SolverBody>& solvedBodies, const AvxVector<DynamicProperties>& dynamicProperties);
	void LimitNormalWide(ContactLimiterWide& limiter, int start, const __m256& n1X, const __m256& n1Y, const __m256& n2X, const __m256& n2Y, const __m256& w1X, const __m256& w1Y, const __m256& w2X, const __m256& w2Y, const __m256& body1InvMass, const __m256& body1InvInertia, const __m256& body2InvMass, const __m256& body2InvInertia);
	void LimitNormal(ContactLimiterWide& limiter, int start, const vec2& normal, const vec2& delta, const vec2& w, const float& body1InvMass, const float& body1InvInertia, const float& body2InvMass, const float& body2InvInertia);
}
