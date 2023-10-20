#pragma once

// TODO shouldn't need these
#include "PhysicalShapes/CircleShape.h"
#include "PhysicalShapes/PolygonShape.h"
#include "PhysicalShapes/SegmentShape.h"

namespace Integration
{
	inline void IntegratePositionals(const real dt, const Dynamical& dynamicsComp, Positional& outputPositional)
	{
		outputPositional.position = add(outputPositional.position, mul(dynamicsComp.linearVelocity, dt));
		outputPositional.angle = outputPositional.angle + dynamicsComp.angularVelocity * dt;
	}

	inline void IntegrateCorrections(const real dt, const OverlapCorrectors& dynamicsComp, Positional& outputPositional, const real scale)
	{
		outputPositional.position = add(outputPositional.position, mul(dynamicsComp.linearCorrection, dt * scale));
		outputPositional.angle = outputPositional.angle + dynamicsComp.angularCorrection * dt * scale;
	}

	inline void UpdateTransform(const Positional& integratedPos, Transform& outputTransform)
	{
		const real sinA = sin(integratedPos.angle);
		const real cosA = cos(integratedPos.angle);

		outputTransform = {
			{ cosA, sinA, -sinA, cosA },
			{
				integratedPos.position.x - (integratedPos.centerOfMass.x * cosA - integratedPos.centerOfMass.y * sinA),
				integratedPos.position.y - (integratedPos.centerOfMass.x * sinA + integratedPos.centerOfMass.y * cosA)
			}
		};
	}

	inline void ZeroOverlapCorrectors(OverlapCorrectors& dynamics)
	{
		dynamics.linearCorrection = ::zero<vec2>;
		dynamics.angularCorrection = 0.0f;
	}

	inline void TransformPolyToWorld(const Transform& transform, PolygonShape* poly)
	{
		auto count = poly->localLines.size();

		auto& dst = poly->transformedLines;
		auto& src = poly->localLines;

		for (size_t i = 0; i < count; i++) {
			vec2 v = transform.transformPoint(src[i].point);
			vec2 n = transform.rotation * src[i].normal;

			dst[i].point = v;
			dst[i].normal = n;
		}
	}

	inline void TransformSegmentToWorld(const Transform& transform, SegmentShape& segment)
	{
		segment.transformedA = transform.transformPoint(segment.pointA);
		segment.transformedB = transform.transformPoint(segment.pointB);
		segment.transformedNormal = transform.rotation * segment.normal;
	}

	inline void TransformCircleToWorld(const Transform& transform, CircleShape& circle)
	{
		circle.transformedOffset = transform.transformPoint(circle.offset);
	}

	inline void IntegrateVelocities(
		const InversePhysicalProps& physicalProps, const Forces forces, const real damping, const real dt,
		Dynamical& dynamicsOutput)
	{
		dynamicsOutput.linearVelocity = add(mul(dynamicsOutput.linearVelocity, damping), mul(mul(forces.linearForce, physicalProps.massInverse), dt));
		dynamicsOutput.angularVelocity = dynamicsOutput.angularVelocity * damping + forces.angularForce * physicalProps.momentInverse * dt;
	}

	inline void ZeroForces(Forces& forces)
	{
		forces.linearForce = ::zero<vec2>;
		forces.angularForce = 0.0f;
	}
};

