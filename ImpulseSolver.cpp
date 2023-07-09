#include "ImpulseSolver.h"

AvxVector<Contact> ImpulseSolver::CreateJoints(const std::vector<Overlap>& overlaps, std::vector<ContactPoint>& contacts)
{
	AvxVector<Contact> joints;

	for (auto& joint : joints)
	{
		joint.contactPointIndex = -1;
	}

	for (auto overlap : overlaps)
	{
		for (int collisionIndex = 0; collisionIndex < overlap.GetPointCount(); collisionIndex++)
		{
			const int contactPointIndex = overlap.pointsStartIndex + collisionIndex;
			ContactPoint& col = contacts[contactPointIndex];

			if (col.solverIndex < 0)
			{
				col.solverIndex = joints.size();
				joints.push_back(Contact(overlap.firstEntityIndex, overlap.secondEntityIndex, contactPointIndex));

			}
			else
			{
				Contact& joint = joints[col.solverIndex];
				joint.contactPointIndex = contactPointIndex;
			}
		}
	}

	for (int jointIndex = 0; jointIndex < joints.size();)
	{
		Contact& joint = joints[jointIndex];

		if (joint.contactPointIndex < 0)
		{
			joint = joints[joints.size() - 1];
			joints.pop_back();

		}
		else
		{
			//TODO solver index is joint index/ID or something
			contacts[joint.contactPointIndex].solverIndex = jointIndex;
			jointIndex++;
		}
	}

	return joints;
}

void ImpulseSolver::ProcessJoints(const int bodiesCount, ContactPoint* contactPoints,
                                  const AvxVector<DynamicProperties>& dynamicProperties,
                                  AvxVector<Contact>& contactJoints, SolverData& solverData)
{
	const int jointCount = contactJoints.size();

	AvxVector<int> joint_index(jointCount);
	AvxVector<ContactJointWide> joint_packed(jointCount);
	AvxVector<int> jointGroup_joints(jointCount);
	AvxVector<int> jointGroup_bodies(jointCount);

	joint_index.reserve(jointCount);
	joint_packed.reserve(jointCount);
	jointGroup_joints.reserve(jointCount);
	jointGroup_bodies.reserve(bodiesCount);

	for (size_t i = 0; i < jointCount; ++i)
		joint_index.data()[i] = i; // TODO foreach

	for (size_t i = 0; i < bodiesCount; ++i)
		jointGroup_bodies.data()[i] = 0; // TODO foreach

	// TODO split into islands

	ProcessJointIsland(
		joint_packed, 0, jointCount,
		contactPoints, jointGroup_joints, joint_index, jointGroup_bodies, contactJoints,
		solverData, dynamicProperties
	);
}

void ImpulseSolver::UpdateJoints(const ContactJointWide* jointPacked, const int jointBegin, const int jointEnd, AvxVector<SolverBody>& solveBodiesImpulse)
{
	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex += 8)
	{
		const int i = jointIndex;
		const ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];

		__m256 body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		__m256 body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		_mm256_load4_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_load4_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index, sizeof(SolverBody));

		const __m256 normalLimiterMass1LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass1X);
		const __m256 normalLimiterMass1LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass1Y);
		const __m256 normalLimiterMass2LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass2X);
		const __m256 normalLimiterMass2LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass2Y);
		const __m256 normalLimiterMass1Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass1);
		const __m256 normalLimiterMass2Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass2);
		const __m256 normalLimiterAccumulatedImpulse = _mm256_load_ps(jointWide.normalLimiterTotalImpulse);

		const __m256 frictionLimiterMass1LinearX = _mm256_load_ps(jointWide.frictionLimiter.linearMass1X);
		const __m256 frictionLimiterMass1LinearY = _mm256_load_ps(jointWide.frictionLimiter.linearMass1Y);
		const __m256 frictionLimiterMass2LinearX = _mm256_load_ps(jointWide.frictionLimiter.linearMass2X);
		const __m256 frictionLimiterMass2LinearY = _mm256_load_ps(jointWide.frictionLimiter.linearMass2Y);
		const __m256 frictionLimiterMass1Angular = _mm256_load_ps(jointWide.frictionLimiter.angularMass1);
		const __m256 frictionLimiterMass2Angular = _mm256_load_ps(jointWide.frictionLimiter.angularMass2);
		const __m256 frictionLimiterAccumulatedImpulse = _mm256_load_ps(jointWide.frictionLimiterTotalImpulse);

		body1VelocityX += normalLimiterMass1LinearX * normalLimiterAccumulatedImpulse;
		body1VelocityY += normalLimiterMass1LinearY * normalLimiterAccumulatedImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * normalLimiterAccumulatedImpulse;

		body2VelocityX += normalLimiterMass2LinearX * normalLimiterAccumulatedImpulse;
		body2VelocityY += normalLimiterMass2LinearY * normalLimiterAccumulatedImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * normalLimiterAccumulatedImpulse;

		body1VelocityX += frictionLimiterMass1LinearX * frictionLimiterAccumulatedImpulse;
		body1VelocityY += frictionLimiterMass1LinearY * frictionLimiterAccumulatedImpulse;
		body1AngularVelocity += frictionLimiterMass1Angular * frictionLimiterAccumulatedImpulse;

		body2VelocityX += frictionLimiterMass2LinearX * frictionLimiterAccumulatedImpulse;
		body2VelocityY += frictionLimiterMass2LinearY * frictionLimiterAccumulatedImpulse;
		body2AngularVelocity += frictionLimiterMass2Angular * frictionLimiterAccumulatedImpulse;

		_mm256_store8_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_store8_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index, sizeof(SolverBody));
	}
}

char ImpulseSolver::ProcessJointsImpulses(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex, AvxVector<SolverBody>& solveBodiesImpulse)
{
	__m256i iterationIndex0 = _mm256_set1_epi32(iterationIndex);
	__m256i iterationIndex2 = _mm256_set1_epi32(iterationIndex - 2);

	__m256 hasAnyResult = _mm256_setzero_ps();

	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex += 8)
	{
		int i = jointIndex;

		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];

		__m256 body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		__m256 body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		_mm256_load4_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_load4_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index, sizeof(SolverBody));

		__m256i body1LastIteration = _mm256_castps_si256(body1EncodedLastIter);
		__m256i body2LastIteration = _mm256_castps_si256(body2EncodedLastIter);

		__m256 body1Productive = body1LastIteration > iterationIndex2;
		__m256 body2Productive = body2LastIteration > iterationIndex2;
		__m256 bodyHasResult = body1Productive | body2Productive;

		if (_mm256_movemask_ps(bodyHasResult) == 0)
			continue;

		__m256 normalLimiterNormalProjector1X = _mm256_load_ps(jointWide.normalLimiter.normalProjector1X);
		__m256 normalLimiterNormalProjector1Y = _mm256_load_ps(jointWide.normalLimiter.normalProjector1Y);
		__m256 normalLimiterNormalProjector2X = _mm256_load_ps(jointWide.normalLimiter.normalProjector2X);
		__m256 normalLimiterNormalProjector2Y = _mm256_load_ps(jointWide.normalLimiter.normalProjector2Y);
		__m256 normalLimiterAngularProjector1 = _mm256_load_ps(jointWide.normalLimiter.angularProjector1);
		__m256 normalLimiterAngularProjector2 = _mm256_load_ps(jointWide.normalLimiter.angularProjector2);

		__m256 normalLimiterMass1LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass1X);
		__m256 normalLimiterMass1LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass1Y);
		__m256 normalLimiterMass2LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass2X);
		__m256 normalLimiterMass2LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass2Y);
		__m256 normalLimiterMass1Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass1);
		__m256 normalLimiterMass2Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass2);
		__m256 normalLimiterCompInvMass = _mm256_load_ps(jointWide.normalLimiter.invMass);
		__m256 normalLimiterAccumulatedImpulse = _mm256_load_ps(jointWide.normalLimiterTotalImpulse);
		__m256 normalLimiterDstVelocity = _mm256_load_ps(jointWide.normalLimiterTargetVelocity);

		__m256 frictionLimiterNormalProjector1X = _mm256_load_ps(jointWide.frictionLimiter.normalProjector1X);
		__m256 frictionLimiterNormalProjector1Y = _mm256_load_ps(jointWide.frictionLimiter.normalProjector1Y);
		__m256 frictionLimiterNormalProjector2X = _mm256_load_ps(jointWide.frictionLimiter.normalProjector2X);
		__m256 frictionLimiterNormalProjector2Y = _mm256_load_ps(jointWide.frictionLimiter.normalProjector2Y);
		__m256 frictionLimiterAngularProjector1 = _mm256_load_ps(jointWide.frictionLimiter.angularProjector1);
		__m256 frictionLimiterAngularProjector2 = _mm256_load_ps(jointWide.frictionLimiter.angularProjector2);

		__m256 frictionLimiterMass1LinearX = _mm256_load_ps(jointWide.frictionLimiter.linearMass1X);
		__m256 frictionLimiterMass1LinearY = _mm256_load_ps(jointWide.frictionLimiter.linearMass1Y);
		__m256 frictionLimiterMass2LinearX = _mm256_load_ps(jointWide.frictionLimiter.linearMass2X);
		__m256 frictionLimiterMass2LinearY = _mm256_load_ps(jointWide.frictionLimiter.linearMass2Y);
		__m256 frictionLimiterMass1Angular = _mm256_load_ps(jointWide.frictionLimiter.angularMass1);
		__m256 frictionLimiterMass2Angular = _mm256_load_ps(jointWide.frictionLimiter.angularMass2);
		__m256 frictionLimiterCompInvMass = _mm256_load_ps(jointWide.frictionLimiter.invMass);
		__m256 frictionLimiterAccumulatedImpulse = _mm256_load_ps(jointWide.frictionLimiterTotalImpulse);

		__m256 normalDeltaVel = normalLimiterDstVelocity;

		normalDeltaVel -= normalLimiterNormalProjector1X * body1VelocityX;
		normalDeltaVel -= normalLimiterNormalProjector1Y * body1VelocityY;
		normalDeltaVel -= normalLimiterAngularProjector1 * body1AngularVelocity;

		normalDeltaVel -= normalLimiterNormalProjector2X * body2VelocityX;
		normalDeltaVel -= normalLimiterNormalProjector2Y * body2VelocityY;
		normalDeltaVel -= normalLimiterAngularProjector2 * body2AngularVelocity;

		__m256 normalDeltaImpulse = normalDeltaVel * normalLimiterCompInvMass;

		normalDeltaImpulse = _mm256_max_ps(normalDeltaImpulse, -normalLimiterAccumulatedImpulse);

		body1VelocityX += normalLimiterMass1LinearX * normalDeltaImpulse;
		body1VelocityY += normalLimiterMass1LinearY * normalDeltaImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * normalDeltaImpulse;

		body2VelocityX += normalLimiterMass2LinearX * normalDeltaImpulse;
		body2VelocityY += normalLimiterMass2LinearY * normalDeltaImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * normalDeltaImpulse;

		normalLimiterAccumulatedImpulse += normalDeltaImpulse;

		__m256 frictionDeltaVel = _mm256_setzero_ps();

		frictionDeltaVel -= frictionLimiterNormalProjector1X * body1VelocityX;
		frictionDeltaVel -= frictionLimiterNormalProjector1Y * body1VelocityY;
		frictionDeltaVel -= frictionLimiterAngularProjector1 * body1AngularVelocity;

		frictionDeltaVel -= frictionLimiterNormalProjector2X * body2VelocityX;
		frictionDeltaVel -= frictionLimiterNormalProjector2Y * body2VelocityY;
		frictionDeltaVel -= frictionLimiterAngularProjector2 * body2AngularVelocity;

		__m256 frictionDeltaImpulse = frictionDeltaVel * frictionLimiterCompInvMass;

		__m256 reactionForce = normalLimiterAccumulatedImpulse;
		__m256 accumulatedImpulse = frictionLimiterAccumulatedImpulse;

		__m256 frictionForce = accumulatedImpulse + frictionDeltaImpulse;
		__m256 reactionForceScaled = reactionForce * _mm256_set1_ps(FRICTION_COEFFICIENT);

		__m256 frictionForceAbs = _mm256_andnot_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)), frictionForce);
		__m256 reactionForceScaledSigned = _mm256_xor_ps(reactionForceScaled,
		                                                 _mm256_and_ps(frictionForce,
		                                                               _mm256_castsi256_ps(
			                                                               _mm256_set1_epi32(0x80000000))));
		__m256 frictionDeltaImpulseAdjusted = reactionForceScaledSigned - accumulatedImpulse;

		frictionDeltaImpulse = _mm256_blendv_ps(frictionDeltaImpulse, frictionDeltaImpulseAdjusted, frictionForceAbs > reactionForceScaled);

		frictionLimiterAccumulatedImpulse += frictionDeltaImpulse;

		body1VelocityX += frictionLimiterMass1LinearX * frictionDeltaImpulse;
		body1VelocityY += frictionLimiterMass1LinearY * frictionDeltaImpulse;
		body1AngularVelocity += frictionLimiterMass1Angular * frictionDeltaImpulse;

		body2VelocityX += frictionLimiterMass2LinearX * frictionDeltaImpulse;
		body2VelocityY += frictionLimiterMass2LinearY * frictionDeltaImpulse;
		body2AngularVelocity += frictionLimiterMass2Angular * frictionDeltaImpulse;

		_mm256_store_ps(jointWide.normalLimiterTotalImpulse, normalLimiterAccumulatedImpulse);
		_mm256_store_ps(jointWide.frictionLimiterTotalImpulse, frictionLimiterAccumulatedImpulse);

		__m256 cumulativeImpulse = _mm256_max_ps(
			__m256(_mm256_andnot_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)), normalDeltaImpulse)),
			__m256(_mm256_andnot_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)), frictionDeltaImpulse)));

		__m256 hasResults = cumulativeImpulse > _mm256_set1_ps(IMPULSE_DISCARD_THRESHOLD);

		hasAnyResult |= hasResults;

		body1LastIteration = _mm256_blendv_epi8(body1LastIteration, iterationIndex0, _mm256_castps_si256(hasResults));
		body2LastIteration = _mm256_blendv_epi8(body2LastIteration, iterationIndex0, _mm256_castps_si256(hasResults));

		body1EncodedLastIter = _mm256_castsi256_ps(body1LastIteration);
		body2EncodedLastIter = _mm256_castsi256_ps(body2LastIteration);

		_mm256_store8_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_store8_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index, sizeof(SolverBody));
	}

	return _mm256_movemask_ps(hasAnyResult) != 0;
}

void ImpulseSolver::UpdateJointScalarContactWide(const ContactJointWide* jointPacked, const int jointBegin, const int jointEnd, AvxVector<SolverBody>& solveBodiesImpulse)
{
	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex++)
	{
		const int i = jointIndex;
		const ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];
		const int start = jointIndex % 8;

		float body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		float body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		scalar_load4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_load4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index + start, sizeof(SolverBody));

		const float normalLimiterMass1LinearX = jointWide.normalLimiter.linearMass1X[start];
		const float normalLimiterMass1LinearY = jointWide.normalLimiter.linearMass1Y[start];
		const float normalLimiterMass2LinearX = jointWide.normalLimiter.linearMass2X[start];
		const float normalLimiterMass2LinearY = jointWide.normalLimiter.linearMass2Y[start];
		const float normalLimiterMass1Angular = jointWide.normalLimiter.angularMass1[start];
		const float normalLimiterMass2Angular = jointWide.normalLimiter.angularMass2[start];
		const float normalLimiterAccumulatedImpulse = jointWide.normalLimiterTotalImpulse[start];

		const float frictionLimiterMass1LinearX = jointWide.frictionLimiter.linearMass1X[start];
		const float frictionLimiterMass1LinearY = jointWide.frictionLimiter.linearMass1Y[start];
		const float frictionLimiterMass2LinearX = jointWide.frictionLimiter.linearMass2X[start];
		const float frictionLimiterMass2LinearY = jointWide.frictionLimiter.linearMass2Y[start];
		const float frictionLimiterMass1Angular = jointWide.frictionLimiter.angularMass1[start];
		const float frictionLimiterMass2Angular = jointWide.frictionLimiter.angularMass2[start];
		const float frictionLimiterAccumulatedImpulse = jointWide.frictionLimiterTotalImpulse[start];

		body1VelocityX += normalLimiterMass1LinearX * normalLimiterAccumulatedImpulse;
		body1VelocityY += normalLimiterMass1LinearY * normalLimiterAccumulatedImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * normalLimiterAccumulatedImpulse;

		body2VelocityX += normalLimiterMass2LinearX * normalLimiterAccumulatedImpulse;
		body2VelocityY += normalLimiterMass2LinearY * normalLimiterAccumulatedImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * normalLimiterAccumulatedImpulse;

		body1VelocityX += frictionLimiterMass1LinearX * frictionLimiterAccumulatedImpulse;
		body1VelocityY += frictionLimiterMass1LinearY * frictionLimiterAccumulatedImpulse;
		body1AngularVelocity += frictionLimiterMass1Angular * frictionLimiterAccumulatedImpulse;

		body2VelocityX += frictionLimiterMass2LinearX * frictionLimiterAccumulatedImpulse;
		body2VelocityY += frictionLimiterMass2LinearY * frictionLimiterAccumulatedImpulse;
		body2AngularVelocity += frictionLimiterMass2Angular * frictionLimiterAccumulatedImpulse;

		scalar_store4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_store4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index + start, sizeof(SolverBody));
	}
}

bool ImpulseSolver::ProcessJointImpulseScalarContactWide(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex,
	AvxVector<SolverBody>& solveBodiesImpulse)
{
	int iterationIndex0 = iterationIndex;
	int iterationIndex2 = iterationIndex - 2;

	bool hasAnyResult = false;

	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex++)
	{
		int i = jointIndex;
		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];
		const int start = jointIndex % 8;

		float body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		float body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		scalar_load4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_load4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index + start, sizeof(SolverBody));

		int body1LastIteration = std::bit_cast<int>(body1EncodedLastIter);
		int body2LastIteration = std::bit_cast<int>(body2EncodedLastIter);

		bool body1Productive = body1LastIteration > iterationIndex2;
		bool body2Productive = body2LastIteration > iterationIndex2;
		bool bodyHasResult = body1Productive | body2Productive;

		if (!bodyHasResult)
			continue;

		float normalLimiterNormalProjector1X = jointWide.normalLimiter.normalProjector1X[start];
		float normalLimiterNormalProjector1Y = jointWide.normalLimiter.normalProjector1Y[start];
		float normalLimiterNormalProjector2X = jointWide.normalLimiter.normalProjector2X[start];
		float normalLimiterNormalProjector2Y = jointWide.normalLimiter.normalProjector2Y[start];
		float normalLimiterAngularProjector1 = jointWide.normalLimiter.angularProjector1[start];
		float normalLimiterAngularProjector2 = jointWide.normalLimiter.angularProjector2[start];

		float normalLimiterMass1LinearX = jointWide.normalLimiter.linearMass1X[start];
		float normalLimiterMass1LinearY = jointWide.normalLimiter.linearMass1Y[start];
		float normalLimiterMass2LinearX = jointWide.normalLimiter.linearMass2X[start];
		float normalLimiterMass2LinearY = jointWide.normalLimiter.linearMass2Y[start];
		float normalLimiterMass1Angular = jointWide.normalLimiter.angularMass1[start];
		float normalLimiterMass2Angular = jointWide.normalLimiter.angularMass2[start];
		float normalLimiterCompInvMass = jointWide.normalLimiter.invMass[start];
		float normalLimiterAccumulatedImpulse = jointWide.normalLimiterTotalImpulse[start];
		float normalLimiterDstVelocity = jointWide.normalLimiterTargetVelocity[start];

		float frictionLimiterNormalProjector1X = jointWide.frictionLimiter.normalProjector1X[start];
		float frictionLimiterNormalProjector1Y = jointWide.frictionLimiter.normalProjector1Y[start];
		float frictionLimiterNormalProjector2X = jointWide.frictionLimiter.normalProjector2X[start];
		float frictionLimiterNormalProjector2Y = jointWide.frictionLimiter.normalProjector2Y[start];
		float frictionLimiterAngularProjector1 = jointWide.frictionLimiter.angularProjector1[start];
		float frictionLimiterAngularProjector2 = jointWide.frictionLimiter.angularProjector2[start];

		float frictionLimiterMass1LinearX = jointWide.frictionLimiter.linearMass1X[start];
		float frictionLimiterMass1LinearY = jointWide.frictionLimiter.linearMass1Y[start];
		float frictionLimiterMass2LinearX = jointWide.frictionLimiter.linearMass2X[start];
		float frictionLimiterMass2LinearY = jointWide.frictionLimiter.linearMass2Y[start];
		float frictionLimiterMass1Angular = jointWide.frictionLimiter.angularMass1[start];
		float frictionLimiterMass2Angular = jointWide.frictionLimiter.angularMass2[start];
		float frictionLimiterCompInvMass = jointWide.frictionLimiter.invMass[start];
		float frictionLimiterAccumulatedImpulse = jointWide.frictionLimiterTotalImpulse[start];

		float normalDeltaVel = normalLimiterDstVelocity;

		normalDeltaVel -= normalLimiterNormalProjector1X * body1VelocityX;
		normalDeltaVel -= normalLimiterNormalProjector1Y * body1VelocityY;
		normalDeltaVel -= normalLimiterAngularProjector1 * body1AngularVelocity;

		normalDeltaVel -= normalLimiterNormalProjector2X * body2VelocityX;
		normalDeltaVel -= normalLimiterNormalProjector2Y * body2VelocityY;
		normalDeltaVel -= normalLimiterAngularProjector2 * body2AngularVelocity;

		float normalDeltaImpulse = normalDeltaVel * normalLimiterCompInvMass;

		normalDeltaImpulse = std::max(normalDeltaImpulse, -normalLimiterAccumulatedImpulse);

		body1VelocityX += normalLimiterMass1LinearX * normalDeltaImpulse;
		body1VelocityY += normalLimiterMass1LinearY * normalDeltaImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * normalDeltaImpulse;

		body2VelocityX += normalLimiterMass2LinearX * normalDeltaImpulse;
		body2VelocityY += normalLimiterMass2LinearY * normalDeltaImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * normalDeltaImpulse;

		normalLimiterAccumulatedImpulse += normalDeltaImpulse;

		float frictionDeltaVel = 0.f;

		frictionDeltaVel -= frictionLimiterNormalProjector1X * body1VelocityX;
		frictionDeltaVel -= frictionLimiterNormalProjector1Y * body1VelocityY;
		frictionDeltaVel -= frictionLimiterAngularProjector1 * body1AngularVelocity;

		frictionDeltaVel -= frictionLimiterNormalProjector2X * body2VelocityX;
		frictionDeltaVel -= frictionLimiterNormalProjector2Y * body2VelocityY;
		frictionDeltaVel -= frictionLimiterAngularProjector2 * body2AngularVelocity;

		float frictionDeltaImpulse = frictionDeltaVel * frictionLimiterCompInvMass;

		float reactionForce = normalLimiterAccumulatedImpulse;
		float accumulatedImpulse = frictionLimiterAccumulatedImpulse;

		float frictionForce = accumulatedImpulse + frictionDeltaImpulse;
		float reactionForceScaled = reactionForce * FRICTION_COEFFICIENT;

		float frictionForceAbs = fabsf(frictionForce);
		float reactionForceScaledSigned = frictionForce < 0.f ? -reactionForceScaled : reactionForceScaled;
		float frictionDeltaImpulseAdjusted = reactionForceScaledSigned - accumulatedImpulse;

		frictionDeltaImpulse = frictionForceAbs > reactionForceScaled ? frictionDeltaImpulseAdjusted : frictionDeltaImpulse;

		frictionLimiterAccumulatedImpulse += frictionDeltaImpulse;

		body1VelocityX += frictionLimiterMass1LinearX * frictionDeltaImpulse;
		body1VelocityY += frictionLimiterMass1LinearY * frictionDeltaImpulse;
		body1AngularVelocity += frictionLimiterMass1Angular * frictionDeltaImpulse;

		body2VelocityX += frictionLimiterMass2LinearX * frictionDeltaImpulse;
		body2VelocityY += frictionLimiterMass2LinearY * frictionDeltaImpulse;
		body2AngularVelocity += frictionLimiterMass2Angular * frictionDeltaImpulse;

		jointWide.normalLimiterTotalImpulse[start] = normalLimiterAccumulatedImpulse;
		jointWide.frictionLimiterTotalImpulse[start] = frictionLimiterAccumulatedImpulse;

		float cumulativeImpulse = std::max(fabsf(normalDeltaImpulse), fabsf(frictionDeltaImpulse));

		bool hasResults = cumulativeImpulse > IMPULSE_DISCARD_THRESHOLD;

		hasAnyResult |= hasResults;

		body1LastIteration = hasResults ? iterationIndex0 : body1LastIteration;
		body2LastIteration = hasResults ? iterationIndex0 : body2LastIteration;

		body1EncodedLastIter = std::bit_cast<float>(body1LastIteration);
		body2EncodedLastIter = std::bit_cast<float>(body2LastIteration);

		scalar_store4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_store4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesImpulse.data(), jointWide.body2Index + start, sizeof(SolverBody));
	}

	return hasAnyResult;
}

bool ImpulseSolver::ProcessJointsDisplacement(ContactJointWide* jointPacked, int jointBegin, int jointEnd, int iterationIndex,
	AvxVector<SolverBody>& solveBodiesDisplacement)
{
	__m256i iterationIndex0 = _mm256_set1_epi32(iterationIndex);
	__m256i iterationIndex2 = _mm256_set1_epi32(iterationIndex - 2);

	__m256 hasAnyResult = _mm256_setzero_ps();
	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex += 8)
	{
		int i = jointIndex;

		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];

		__m256 body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		__m256 body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		_mm256_load4_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_load4_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body2Index, sizeof(SolverBody));

		__m256i body1LastIteration = _mm256_castps_si256(body1EncodedLastIter);
		__m256i body2LastIteration = _mm256_castps_si256(body2EncodedLastIter);

		__m256 body1Productive = body1LastIteration > iterationIndex2;
		__m256 body2Productive = body2LastIteration > iterationIndex2;
		__m256 bodyHasResult = body1Productive | body2Productive;

		if (_mm256_movemask_ps(bodyHasResult) == 0)
			continue;

		__m256 normalLimiterNormalProjector1X = _mm256_load_ps(jointWide.normalLimiter.normalProjector1X);
		__m256 normalLimiterNormalProjector1Y = _mm256_load_ps(jointWide.normalLimiter.normalProjector1Y);
		__m256 normalLimiterNormalProjector2X = _mm256_load_ps(jointWide.normalLimiter.normalProjector2X);
		__m256 normalLimiterNormalProjector2Y = _mm256_load_ps(jointWide.normalLimiter.normalProjector2Y);
		__m256 normalLimiterAngularProjector1 = _mm256_load_ps(jointWide.normalLimiter.angularProjector1);
		__m256 normalLimiterAngularProjector2 = _mm256_load_ps(jointWide.normalLimiter.angularProjector2);

		__m256 normalLimiterMass1LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass1X);
		__m256 normalLimiterMass1LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass1Y);
		__m256 normalLimiterMass2LinearX = _mm256_load_ps(jointWide.normalLimiter.linearMass2X);
		__m256 normalLimiterMass2LinearY = _mm256_load_ps(jointWide.normalLimiter.linearMass2Y);
		__m256 normalLimiterMass1Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass1);
		__m256 normalLimiterMass2Angular = _mm256_load_ps(jointWide.normalLimiter.angularMass2);
		__m256 normalLimiterCompInvMass = _mm256_load_ps(jointWide.normalLimiter.invMass);
		__m256 normalLimiterDstDisplacingVelocity = _mm256_load_ps(jointWide.normalLimiterTargetDisplacingVelocity);
		__m256 normalLimiterAccumulatedDisplacingImpulse = _mm256_load_ps(jointWide.normalLimiterTotalDisplacingImpulse);

		__m256 dV = normalLimiterDstDisplacingVelocity;

		dV -= normalLimiterNormalProjector1X * body1VelocityX;
		dV -= normalLimiterNormalProjector1Y * body1VelocityY;
		dV -= normalLimiterAngularProjector1 * body1AngularVelocity;

		dV -= normalLimiterNormalProjector2X * body2VelocityX;
		dV -= normalLimiterNormalProjector2Y * body2VelocityY;
		dV -= normalLimiterAngularProjector2 * body2AngularVelocity;

		__m256 displacingDeltaImpulse = dV * normalLimiterCompInvMass;

		displacingDeltaImpulse = _mm256_max_ps(displacingDeltaImpulse, -normalLimiterAccumulatedDisplacingImpulse);

		body1VelocityX += normalLimiterMass1LinearX * displacingDeltaImpulse;
		body1VelocityY += normalLimiterMass1LinearY * displacingDeltaImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * displacingDeltaImpulse;

		body2VelocityX += normalLimiterMass2LinearX * displacingDeltaImpulse;
		body2VelocityY += normalLimiterMass2LinearY * displacingDeltaImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * displacingDeltaImpulse;

		normalLimiterAccumulatedDisplacingImpulse += displacingDeltaImpulse;

		_mm256_store_ps(jointWide.normalLimiterTotalDisplacingImpulse, normalLimiterAccumulatedDisplacingImpulse);

		__m256 hasResults = __m256(_mm256_andnot_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)),
		                                            displacingDeltaImpulse)) >
			_mm256_set1_ps(IMPULSE_DISCARD_THRESHOLD);

		hasAnyResult |= hasResults;

		body1LastIteration = _mm256_blendv_epi8(body1LastIteration, iterationIndex0, _mm256_castps_si256(hasResults));
		body2LastIteration = _mm256_blendv_epi8(body2LastIteration, iterationIndex0, _mm256_castps_si256(hasResults));

		body1EncodedLastIter = _mm256_castsi256_ps(body1LastIteration);
		body2EncodedLastIter = _mm256_castsi256_ps(body2LastIteration);

		_mm256_store8_ps(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_store8_ps(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body2Index, sizeof(SolverBody));
	}

	return _mm256_movemask_ps(hasAnyResult) != 0;
}

bool ImpulseSolver::ProcessJointDisplacementScalarContactWide(ContactJointWide* jointPacked, const int jointBegin, const int jointEnd, const int iterationIndex,
	AvxVector<SolverBody>& solveBodiesDisplacement)
{
	const int iterationIndex0 = iterationIndex;
	const int iterationIndex2 = iterationIndex - 2;

	bool hasAnyResult = false;

	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex++)
	{
		const int i = jointIndex;

		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];
		const int start = jointIndex % 8;

		float body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		float body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		scalar_load4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_load4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body2Index + start, sizeof(SolverBody));

		int body1LastIteration = std::bit_cast<int>(body1EncodedLastIter);
		int body2LastIteration = std::bit_cast<int>(body2EncodedLastIter);

		const bool body1Productive = body1LastIteration > iterationIndex2;
		const bool body2Productive = body2LastIteration > iterationIndex2;
		const bool bodyHasResult = body1Productive | body2Productive;

		if (!bodyHasResult)
			continue;

		const float normalLimiterNormalProjector1X = jointWide.normalLimiter.normalProjector1X[start];
		const float normalLimiterNormalProjector1Y = jointWide.normalLimiter.normalProjector1Y[start];
		const float normalLimiterNormalProjector2X = jointWide.normalLimiter.normalProjector2X[start];
		const float normalLimiterNormalProjector2Y = jointWide.normalLimiter.normalProjector2Y[start];
		const float normalLimiterAngularProjector1 = jointWide.normalLimiter.angularProjector1[start];
		const float normalLimiterAngularProjector2 = jointWide.normalLimiter.angularProjector2[start];

		const float normalLimiterMass1LinearX = jointWide.normalLimiter.linearMass1X[start];
		const float normalLimiterMass1LinearY = jointWide.normalLimiter.linearMass1Y[start];
		const float normalLimiterMass2LinearX = jointWide.normalLimiter.linearMass2X[start];
		const float normalLimiterMass2LinearY = jointWide.normalLimiter.linearMass2Y[start];
		const float normalLimiterMass1Angular = jointWide.normalLimiter.angularMass1[start];
		const float normalLimiterMass2Angular = jointWide.normalLimiter.angularMass2[start];
		const float normalLimiterCompInvMass = jointWide.normalLimiter.invMass[start];
		const float normalLimiterDstDisplacingVelocity = jointWide.normalLimiterTargetDisplacingVelocity[start];
		float normalLimiterAccumulatedDisplacingImpulse = jointWide.normalLimiterTotalDisplacingImpulse[start];

		float deltaV = normalLimiterDstDisplacingVelocity;

		deltaV -= normalLimiterNormalProjector1X * body1VelocityX;
		deltaV -= normalLimiterNormalProjector1Y * body1VelocityY;
		deltaV -= normalLimiterAngularProjector1 * body1AngularVelocity;

		deltaV -= normalLimiterNormalProjector2X * body2VelocityX;
		deltaV -= normalLimiterNormalProjector2Y * body2VelocityY;
		deltaV -= normalLimiterAngularProjector2 * body2AngularVelocity;

		float displacingDeltaImpulse = deltaV * normalLimiterCompInvMass;

		displacingDeltaImpulse = std::max(displacingDeltaImpulse, -normalLimiterAccumulatedDisplacingImpulse);

		body1VelocityX += normalLimiterMass1LinearX * displacingDeltaImpulse;
		body1VelocityY += normalLimiterMass1LinearY * displacingDeltaImpulse;
		body1AngularVelocity += normalLimiterMass1Angular * displacingDeltaImpulse;

		body2VelocityX += normalLimiterMass2LinearX * displacingDeltaImpulse;
		body2VelocityY += normalLimiterMass2LinearY * displacingDeltaImpulse;
		body2AngularVelocity += normalLimiterMass2Angular * displacingDeltaImpulse;

		normalLimiterAccumulatedDisplacingImpulse += displacingDeltaImpulse;

		jointWide.normalLimiterTotalDisplacingImpulse[start] = normalLimiterAccumulatedDisplacingImpulse;

		const bool hasResults = fabsf(displacingDeltaImpulse) > IMPULSE_DISCARD_THRESHOLD;

		hasAnyResult |= hasResults;

		body1LastIteration = hasResults ? iterationIndex0 : body1LastIteration;
		body2LastIteration = hasResults ? iterationIndex0 : body2LastIteration;
		
		body1EncodedLastIter = std::bit_cast<float>(body1LastIteration);
		body2EncodedLastIter = std::bit_cast<float>(body2LastIteration);

		scalar_store4(body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body1Index + start, sizeof(SolverBody));

		scalar_store4(body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter,
			solveBodiesDisplacement.data(), jointWide.body2Index + start, sizeof(SolverBody));
	}

	return hasAnyResult;
}

void ImpulseSolver::FinishJoints(const AvxVector<ContactJointWide>& jointPacked, const int jointBegin, const int jointEnd,
                                 const AvxVector<int>& jointIndex,
                                 AvxVector<Contact>& contactJoints)
{
	// queue.parallelize_loop(jointEnd - jointBegin,
	// 	[&](const int start, const int end)
	// 	{
	for (size_t i = 0; i < jointEnd - jointBegin; ++i)
	{
		Contact& joint = contactJoints.data()[jointIndex.data()[jointBegin + i]];

		const ContactJointWide& jointWide = jointPacked.data()[static_cast<unsigned>(jointBegin + i) / 8];
		const int start = i % 8;

		joint.normalLimiterTotalImpulse = jointWide.normalLimiterTotalImpulse[start];
		joint.frictionLimiterTotalImpulse = jointWide.frictionLimiterTotalImpulse[start];
	}
	// })
	// .wait();
}

void ImpulseSolver::ProcessJointIsland(AvxVector<ContactJointWide>& jointPacked, const int jointBegin, const int jointEnd, ContactPoint* contactPoints,
	AvxVector<int>& jointGroupJoints,
	AvxVector<int>& jointIndex,
	AvxVector<int>& jointGroupBodies,
	AvxVector<Contact>& contactJoints,
	SolverData& solverData,
	const AvxVector<DynamicProperties>& dynamicProperties)
{
	const int groupOffset = PrepareJoints(jointPacked, jointBegin, jointEnd, 8, jointGroupJoints, jointIndex, jointGroupBodies, contactJoints);

	const int batchSize = 512;
	const int batchCount = (jointEnd - jointBegin + batchSize - 1) / batchSize;

	//TODO parallel
	for (size_t i = 0; i < batchCount; ++i)
	{
		int batchBegin = jointBegin + i * batchSize;
		int batchEnd = std::min(batchBegin + batchSize, jointEnd);

		RefreshJoints(jointPacked.data(), batchBegin, std::min(groupOffset, batchEnd), contactPoints, solverData.Impulse, dynamicProperties);
		RefreshJointScalarContactWide(jointPacked.data(), std::max(groupOffset, batchBegin), batchEnd, contactPoints, solverData.Impulse, dynamicProperties);
	}

	// queue.parallelize_loop(batchCount,
	// 	[&](const int start, const int end)
	// 	{
	for (size_t i = 0; i < batchCount; ++i)
	{
		int batchBegin = jointBegin + i * batchSize;
		int batchEnd = std::min(batchBegin + batchSize, jointEnd);

		UpdateJoints(jointPacked.data(), batchBegin, std::min(groupOffset, batchEnd), solverData.Impulse);
		UpdateJointScalarContactWide(jointPacked.data(), std::max(groupOffset, batchBegin), batchEnd, solverData.Impulse);
	}
	// })
	// .wait();

	//for (int iterationIndex = 0; iterationIndex < ITERS_PER_CONTACT; iterationIndex++)
	// queue.parallelize_loop(batchCount,
	// 	[&](const int start, const int end)
	// 	{
	for (size_t i = 0; i < batchCount; ++i)
	{
		int batchBegin = jointBegin + i * batchSize;
		int batchEnd = std::min(batchBegin + batchSize, jointEnd);

		ProcessJointsImpulses(jointPacked.data(), batchBegin, std::min(groupOffset, batchEnd), 0, solverData.Impulse);
		ProcessJointImpulseScalarContactWide(jointPacked.data(), std::max(groupOffset, batchBegin), batchEnd, 0, solverData.Impulse);
	}
	// })
	// .wait();

	// TODO we could early out here when no results are produced

	//for (int iterationIndex = 0; iterationIndex < ITERS_PER_CONTACT; iterationIndex++)
	// queue.parallelize_loop(batchCount,
	// 	[&](const int start, const int end)
	// 	{
	for (size_t i = 0; i < batchCount; ++i)
	{
		int batchBegin = jointBegin + i * batchSize;
		int batchEnd = std::min(batchBegin + batchSize, jointEnd);

		ProcessJointsDisplacement(jointPacked.data(), batchBegin, std::min(groupOffset, batchEnd), 0, solverData.Displacement);
		ProcessJointDisplacementScalarContactWide(jointPacked.data(), std::max(groupOffset, batchBegin), batchEnd, 0, solverData.Displacement);
	}
	// })
	// .wait();

	// TODO we could early out here when no results are produced

	FinishJoints(jointPacked, jointBegin, jointEnd, jointIndex, contactJoints);
}

int ImpulseSolver::PrepareJoints(AvxVector<ContactJointWide>& jointPacked, const int jointBegin, const int jointEnd, const int groupSizeTarget,
	AvxVector<int>& jointGroupJoints,
	AvxVector<int>& jointIndex,
	AvxVector<int>& jointGroupBodies,
	const AvxVector<Contact>& contactJoints)
{
	const int groupOffset = PrepareIndices(jointBegin, jointEnd, groupSizeTarget, jointGroupJoints, jointIndex, jointGroupBodies, contactJoints);

	//TODO parallel
	for (size_t i = 0; i < jointEnd - jointBegin; ++i)
	{
		const auto& joint = contactJoints.data()[jointIndex.data()[jointBegin + i]];

		ContactJointWide& jointWide = jointPacked.data()[static_cast<unsigned>(jointBegin + i) / 8];
		const int start = i % 8;

		jointWide.body1Index[start] = joint.body1Index;
		jointWide.body2Index[start] = joint.body2Index;
		jointWide.contactPointIndex[start] = joint.contactPointIndex;

		jointWide.normalLimiterTotalImpulse[start] = joint.normalLimiterTotalImpulse;
		jointWide.frictionLimiterTotalImpulse[start] = joint.frictionLimiterTotalImpulse;
	}

	return groupOffset;
}

int ImpulseSolver::PrepareIndices(const int jointBegin, const int jointEnd, const int groupSizeTarget,
                                  AvxVector<int>& jointGroupJoints,
                                  AvxVector<int>& jointIndex,
                                  AvxVector<int>& jointGroupBodies,
                                  const AvxVector<Contact>& contactJoints
)
{
	if (groupSizeTarget == 1)
		return jointEnd;

	for (int i = jointBegin; i < jointEnd; ++i)
		jointGroupJoints.data()[i] = jointIndex.data()[i];

	int tag = 0;

	int remainingJoints = jointEnd - jointBegin;
	int groupOffset = jointBegin;

	while (remainingJoints >= groupSizeTarget)
	{
		int groupSize = 0;

		tag++;

		for (size_t i = 0; i < remainingJoints && groupSize < groupSizeTarget;)
		{
			const int curIndex = jointGroupJoints.data()[jointBegin + i];
			const Contact& joint = contactJoints.data()[curIndex];

			// TODO: race between static bodies from different islands
			if (jointGroupBodies.data()[joint.body1Index] < tag && jointGroupBodies.data()[joint.body2Index] < tag)
			{
				jointGroupBodies.data()[joint.body1Index] = tag;
				jointGroupBodies.data()[joint.body2Index] = tag;

				jointIndex.data()[groupOffset + groupSize] = curIndex;
				groupSize++;

				jointGroupJoints.data()[jointBegin + i] = jointGroupJoints.data()[jointBegin + remainingJoints - 1];
				remainingJoints--;
			}
			else
			{
				i++;
			}
		}

		groupOffset += groupSize;

		if (groupSize < groupSizeTarget)
			break;
	}

	// fill in the rest of the joints sequentially - they don't form a group so we'll have to solve them 1 by 1
	for (size_t i = 0; i < remainingJoints; ++i)
		jointIndex.data()[groupOffset + i] = jointGroupJoints.data()[jointBegin + i];

	return groupOffset & ~(groupSizeTarget - 1);
}

void ImpulseSolver::RefreshJoints(ContactJointWide* jointPacked, int jointBegin, int jointEnd, ContactPoint* contactPoints,
	AvxVector<SolverBody>& solvedBodies, const AvxVector<DynamicProperties>& dynamicProperties)
{
	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex += 8)
	{
		int i = jointIndex;

		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(i) / 8];

		__m256 body1VelocityX, body1VelocityY, body1AngularVelocity, body1EncodedLastIter;
		__m256 body2VelocityX, body2VelocityY, body2AngularVelocity, body2EncodedLastIter;

		__m256 body1InvMass, body1InvInertia, body1Coords_posX, body1Coords_posY;
		__m256 body1Coords_xVectorX, body1Coords_xVectorY, body1Coords_yVectorX, body1Coords_yVectorY;

		__m256 body2InvMass, body2InvInertia, body2Coords_posX, body2Coords_posY;
		__m256 body2Coords_xVectorX, body2Coords_xVectorY, body2Coords_yVectorX, body2Coords_yVectorY;

		__m256 collision_delta1X, collision_delta1Y, collision_delta2X, collision_delta2Y;
		__m256 collision_normalX, collision_normalY;
		__m256 dummy;

		_mm256_load4_ps(
			body1VelocityX, body1VelocityY, body1AngularVelocity, dummy,
			solvedBodies.data(), jointWide.body1Index, sizeof(SolverBody));

		_mm256_load4_ps(
			body2VelocityX, body2VelocityY, body2AngularVelocity, dummy,
			solvedBodies.data(), jointWide.body2Index, sizeof(SolverBody));

		_mm256_load8_permute2f128_ps(
			body1InvMass, body1InvInertia, body1Coords_posX, body1Coords_posY,
			body1Coords_xVectorX, body1Coords_xVectorY, body1Coords_yVectorX, body1Coords_yVectorY,
			dynamicProperties.data(), jointWide.body1Index, sizeof(DynamicProperties));

		_mm256_load8_permute2f128_ps(
			body2InvMass, body2InvInertia, body2Coords_posX, body2Coords_posY,
			body2Coords_xVectorX, body2Coords_xVectorY, body2Coords_yVectorX, body2Coords_yVectorY,
			dynamicProperties.data(), jointWide.body2Index, sizeof(DynamicProperties));

		_mm256_load8_permute2f128_ps(
			collision_delta1X, collision_delta1Y, collision_delta2X, collision_delta2Y,
			collision_normalX, collision_normalY, dummy, dummy,
			contactPoints, jointWide.contactPointIndex, sizeof(ContactPoint));

		__m256 point1X = collision_delta1X + body1Coords_posX;
		__m256 point1Y = collision_delta1Y + body1Coords_posY;
		__m256 point2X = collision_delta2X + body2Coords_posX;
		__m256 point2Y = collision_delta2Y + body2Coords_posY;

		__m256 w1X = collision_delta1X;
		__m256 w1Y = collision_delta1Y;
		__m256 w2X = point1X - body2Coords_posX;
		__m256 w2Y = point1Y - body2Coords_posY;

		LimitNormalWide(jointWide.normalLimiter, 0,
			collision_normalX, collision_normalY, -collision_normalX, -collision_normalY,
			w1X, w1Y, w2X, w2Y,
			body1InvMass, body1InvInertia, body2InvMass, body2InvInertia);

		__m256 bounce = _mm256_setzero_ps();
		__m256 deltaVelocity = _mm256_set1_ps(1.f);
		__m256 maxPenetrationVelocity = _mm256_set1_ps(0.1f);
		__m256 deltaDepth = _mm256_set1_ps(0.1f);
		__m256 errorReduction = _mm256_set1_ps(0.1f);

		__m256 pointVelocity_body1X = (body1Coords_posY - point1Y) * body1AngularVelocity + body1VelocityX;
		__m256 pointVelocity_body1Y = (point1X - body1Coords_posX) * body1AngularVelocity + body1VelocityY;

		__m256 pointVelocity_body2X = (body2Coords_posY - point2Y) * body2AngularVelocity + body2VelocityX;
		__m256 pointVelocity_body2Y = (point2X - body2Coords_posX) * body2AngularVelocity + body2VelocityY;

		__m256 relativeVelocityX = pointVelocity_body1X - pointVelocity_body2X;
		__m256 relativeVelocityY = pointVelocity_body1Y - pointVelocity_body2Y;

		__m256 dv = -bounce * (relativeVelocityX * collision_normalX + relativeVelocityY * collision_normalY);
		__m256 depth = (point2X - point1X) * collision_normalX + (point2Y - point1Y) * collision_normalY;

		__m256 dstVelocity = _mm256_max_ps(dv - deltaVelocity, _mm256_setzero_ps());

		__m256 normalLimiterDstVelocity = _mm256_blendv_ps(dstVelocity, dstVelocity - maxPenetrationVelocity, depth < deltaDepth);
		__m256 normalLimiterDstDisplacingVelocity = errorReduction * _mm256_max_ps(_mm256_setzero_ps(), depth - _mm256_set1_ps(2.0f) * deltaDepth);
		__m256 normalLimiterAccumulatedDisplacingImpulse = _mm256_setzero_ps();

		__m256 tangentX = -collision_normalY;
		__m256 tangentY = collision_normalX;

		LimitNormalWide(jointWide.frictionLimiter, 0,
			tangentX, tangentY, -tangentX, -tangentY,
			w1X, w1Y, w2X, w2Y,
			body1InvMass, body1InvInertia, body2InvMass, body2InvInertia);

		_mm256_store_ps(jointWide.normalLimiterTargetVelocity, normalLimiterDstVelocity);
		_mm256_store_ps(jointWide.normalLimiterTargetDisplacingVelocity, normalLimiterDstDisplacingVelocity);
		_mm256_store_ps(jointWide.normalLimiterTotalDisplacingImpulse, normalLimiterAccumulatedDisplacingImpulse);

	}
}

//TODO merge these ^v

void ImpulseSolver::RefreshJointScalarContactWide(ContactJointWide* jointPacked, int jointBegin, int jointEnd, ContactPoint* contactPoints,
	const AvxVector<SolverBody>& solvedBodies, const AvxVector<DynamicProperties>& dynamicProperties)
{
	for (int jointIndex = jointBegin; jointIndex < jointEnd; jointIndex++)
	{
		ContactJointWide& jointWide = jointPacked[static_cast<unsigned>(jointIndex) / 8];

		const int start = jointIndex % 8;

		vec2 body1Velocity;
		vec2 body2Velocity;

		float body1Coords_xVectorX, body1Coords_xVectorY, body1Coords_yVectorX, body1Coords_yVectorY;
		float body2Coords_xVectorX, body2Coords_xVectorY, body2Coords_yVectorX, body2Coords_yVectorY;

		vec2 body1Coords_pos;
		vec2 body2Coords_pos;

		float body1AngularVelocity;
		float body2AngularVelocity;

		float body1InvMass, body1InvInertia;
		float body2InvMass, body2InvInertia;

		vec2 collision_delta1, collision_delta2;
		vec2 collision_normal;
		float dummy;

		const float* sourceAvx = reinterpret_cast<const float*>(solvedBodies.data() + (*jointWide.body1Index + start) * sizeof(SolverBody));
		body1Velocity = static_cast<vec2>(sourceAvx[0]);
		body1AngularVelocity = sourceAvx[2];
		body2Velocity = static_cast<vec2>(sourceAvx[4]);
		body2AngularVelocity = sourceAvx[6];

		scalar_load8(
			body1InvMass, body1InvInertia, body1Coords_pos.x, body1Coords_pos.y,
			body1Coords_xVectorX, body1Coords_xVectorY, body1Coords_yVectorX, body1Coords_yVectorY,
			dynamicProperties.data(), jointWide.body1Index + start, sizeof(DynamicProperties));

		scalar_load8(
			body2InvMass, body2InvInertia, body2Coords_pos.x, body2Coords_pos.y,
			body2Coords_xVectorX, body2Coords_xVectorY, body2Coords_yVectorX, body2Coords_yVectorY,
			dynamicProperties.data(), jointWide.body2Index + start, sizeof(DynamicProperties));

		scalar_load8(
			collision_delta1.x, collision_delta1.y, collision_delta2.x, collision_delta2.y,
			collision_normal.x, collision_normal.y, dummy, dummy,
			contactPoints, jointWide.contactPointIndex + start, sizeof(ContactPoint));

		float point1X = collision_delta1.x + body1Coords_pos.x;
		float point1Y = collision_delta1.y + body1Coords_pos.y;
		float point2X = collision_delta2.x + body2Coords_pos.x;
		float point2Y = collision_delta2.y + body2Coords_pos.y;

		float w2X = point1X - body2Coords_pos.x;
		float w2Y = point1Y - body2Coords_pos.y;

		LimitNormal(jointWide.normalLimiter, start,
			vec2(collision_normal.x, collision_normal.y),
			vec2(collision_delta1.x, collision_delta1.y),
			vec2(w2X, w2Y),
			body1InvMass, body1InvInertia, body2InvMass, body2InvInertia);

		float bounce = 0.f;
		float deltaVelocity = 1.f;
		float maxPenetrationVelocity = 0.1f;
		float deltaDepth = 0.1f;
		float errorReduction = 0.1f;

		float pointVelocity_body1X = (body1Coords_pos.y - point1Y) * body1AngularVelocity + body1Velocity.x;
		float pointVelocity_body1Y = (point1X - body1Coords_pos.x) * body1AngularVelocity + body1Velocity.y;

		float pointVelocity_body2X = (body2Coords_pos.y - point2Y) * body2AngularVelocity + body2Velocity.x;
		float pointVelocity_body2Y = (point2X - body2Coords_pos.x) * body2AngularVelocity + body2Velocity.y;

		float relativeVelocityX = pointVelocity_body1X - pointVelocity_body2X;
		float relativeVelocityY = pointVelocity_body1Y - pointVelocity_body2Y;

		float dv = -bounce * (relativeVelocityX * collision_normal.x + relativeVelocityY * collision_normal.y);
		float depth = (point2X - point1X) * collision_normal.x + (point2Y - point1Y) * collision_normal.y;

		float dstVelocity = std::max(dv - deltaVelocity, 0.f);

		float normalLimiterDstVelocity = depth < deltaDepth ? dstVelocity - maxPenetrationVelocity : dstVelocity;
		float normalLimiterDstDisplacingVelocity = errorReduction * std::max(0.f, depth - 2.0f * deltaDepth);
		float normalLimiterAccumulatedDisplacingImpulse = 0.f;

		LimitNormal(jointWide.frictionLimiter, start,
			vec2(-collision_normal.y, collision_normal.x),
			vec2(collision_delta1.x, collision_delta1.y),
			vec2(w2X, w2Y),
			body1InvMass, body1InvInertia, body2InvMass, body2InvInertia);

		jointWide.normalLimiterTargetVelocity[start] = normalLimiterDstVelocity;
		jointWide.normalLimiterTargetDisplacingVelocity[start] = normalLimiterDstDisplacingVelocity;
		jointWide.normalLimiterTotalDisplacingImpulse[start] = normalLimiterAccumulatedDisplacingImpulse;

	}
}

void ImpulseSolver::LimitNormal(
	ContactLimiterWide& limiter, const int start,
	const vec2& normal, const vec2& delta, const vec2& w,
	const float& body1InvMass, const float& body1InvInertia, const float& body2InvMass, const float& body2InvInertia)
{
	const vec2 normalProjector1 = normal;
	const vec2 normalProjector2 = -normal;

	const float angularProjector1 = normal.x * delta.y - normal.y * delta.x;
	const float angularProjector2 = normalProjector2.x * w.y - normalProjector2.y * w.x;

	const vec2 compMass1Linear = normalProjector1 * body1InvMass;
	const vec2 compMass2Linear = normalProjector2 * body2InvMass;

	const float compMass1Angular = angularProjector1 * body1InvInertia;
	const float compMass2Angular = angularProjector2 * body2InvInertia;

	const float compMass1 = glm::compAdd(normalProjector1 * compMass1Linear) + angularProjector1 * compMass1Angular;
	const float compMass2 = glm::compAdd(normalProjector2 * compMass2Linear) + angularProjector2 * compMass2Angular;

	const float compMass = compMass1 + compMass2;

	const float compInvMass = fabsf(compMass) > 0.f ? 1 / compMass : 0;

	limiter.normalProjector1X[start] = normalProjector1.x;
	limiter.normalProjector1Y[start] = normalProjector1.y;
	limiter.normalProjector2X[start] = normalProjector2.x;
	limiter.normalProjector2Y[start] = normalProjector2.y;
	limiter.angularProjector1[start] = angularProjector1;
	limiter.angularProjector2[start] = angularProjector2;
	limiter.linearMass1X[start] = compMass1Linear.x;
	limiter.linearMass1Y[start] = compMass1Linear.y;
	limiter.linearMass2X[start] = compMass2Linear.x;
	limiter.linearMass2Y[start] = compMass2Linear.y;
	limiter.angularMass1[start] = compMass1Angular;
	limiter.angularMass2[start] = compMass2Angular;

	limiter.invMass[start] = compInvMass;
}

void ImpulseSolver::LimitNormalWide(
	ContactLimiterWide& limiter, const int start,
	const __m256& n1X, const __m256& n1Y, const __m256& n2X, const __m256& n2Y, const __m256& w1X, const __m256& w1Y, const __m256& w2X, const __m256& w2Y,
	const __m256& body1InvMass, const __m256& body1InvInertia, const __m256& body2InvMass, const __m256& body2InvInertia)
{
	const __m256 normalProjector1X = n1X;
	const __m256 normalProjector1Y = n1Y;
	const __m256 normalProjector2X = n2X;
	const __m256 normalProjector2Y = n2Y;
	const __m256 angularProjector1 = n1X * w1Y - n1Y * w1X;
	const __m256 angularProjector2 = n2X * w2Y - n2Y * w2X;

	const __m256 compMass1LinearX = normalProjector1X * body1InvMass;
	const __m256 compMass1LinearY = normalProjector1Y * body1InvMass;
	const __m256 compMass1Angular = angularProjector1 * body1InvInertia;
	const __m256 compMass2LinearX = normalProjector2X * body2InvMass;
	const __m256 compMass2LinearY = normalProjector2Y * body2InvMass;
	const __m256 compMass2Angular = angularProjector2 * body2InvInertia;

	const __m256 compMass1 = normalProjector1X * compMass1LinearX + normalProjector1Y * compMass1LinearY + angularProjector1 * compMass1Angular;
	const __m256 compMass2 = normalProjector2X * compMass2LinearX + normalProjector2Y * compMass2LinearY + angularProjector2 * compMass2Angular;

	const __m256 compMass = compMass1 + compMass2;

	const __m256 compInvMass = _mm256_blendv_ps(
		_mm256_setzero_ps(), _mm256_set1_ps(1) / compMass, _mm256_andnot_ps(_mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)), compMass) > _mm256_setzero_ps());

	_mm256_store_ps(&limiter.normalProjector1X[start], normalProjector1X);
	_mm256_store_ps(&limiter.normalProjector1Y[start], normalProjector1Y);
	_mm256_store_ps(&limiter.normalProjector2X[start], normalProjector2X);
	_mm256_store_ps(&limiter.normalProjector2Y[start], normalProjector2Y);
	_mm256_store_ps(&limiter.angularProjector1[start], angularProjector1);
	_mm256_store_ps(&limiter.angularProjector2[start], angularProjector2);

	_mm256_store_ps(&limiter.linearMass1X[start], compMass1LinearX);
	_mm256_store_ps(&limiter.linearMass1Y[start], compMass1LinearY);
	_mm256_store_ps(&limiter.linearMass2X[start], compMass2LinearX);
	_mm256_store_ps(&limiter.linearMass2Y[start], compMass2LinearY);
	_mm256_store_ps(&limiter.angularMass1[start], compMass1Angular);
	_mm256_store_ps(&limiter.angularMass2[start], compMass2Angular);
	_mm256_store_ps(&limiter.invMass[start], compInvMass);
}