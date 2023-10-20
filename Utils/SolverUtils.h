/* MIT-LICENSED CODE ADAPTED FROM THE ORIGINAL WORK BY SCOTT LEMBCKE AND HOWLING MOON SOFTWARE: Copyright (c) 2013 Scott Lembcke and Howling Moon Software
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */
#pragma once

#include "Common/MathExtras.h"
#include "RigidBody.h"

namespace SolverUtils
{
	static real bias_coef(const real errorBias, const real dt)
	{
		return 1.0f - pow(errorBias, dt);
	}

	static void apply_impulse(const InversePhysicalProps& props, const vec2 j, const vec2 r, Dynamical& dynamics)
	{
		dynamics.linearVelocity = add(dynamics.linearVelocity, mul(j, props.massInverse));
		dynamics.angularVelocity += props.momentInverse * perpDot(r, j);
	}

	static void apply_impulses(InversePhysicalProps& a, InversePhysicalProps& b, const vec2 r1, const vec2 r2, const vec2 j, Dynamical& dynA, Dynamical& dynB)
	{
		apply_impulse(a, neg(j), r1, dynA);
		apply_impulse(b, j, r2, dynB);
	}

	static void apply_bias_impulse(const InversePhysicalProps& props, const vec2 j, const vec2 r, OverlapCorrectors& correctors)
	{
		correctors.linearCorrection = add(correctors.linearCorrection, mul(j, props.massInverse));
		correctors.angularCorrection += props.momentInverse * perpDot(r, j);
	}

	static void apply_bias_impulses(
		InversePhysicalProps& a, InversePhysicalProps& b, const vec2 r1, const vec2 r2, const vec2 j, OverlapCorrectors& dynA, OverlapCorrectors& dynB)
	{
		apply_bias_impulse(a, neg(j), r1, dynA);
		apply_bias_impulse(b, j, r2, dynB);
	}

	static vec2 relative_velocity(const Dynamical& dynA, const Dynamical& dynB, const vec2 r1, const vec2 r2)
	{
		const vec2 v1_sum = add(dynA.linearVelocity, mul(perp(r1), dynA.angularVelocity));
		const vec2 v2_sum = add(dynB.linearVelocity, mul(perp(r2), dynB.angularVelocity));

		return sub(v2_sum, v1_sum);
	}

	static real normal_relative_velocity(const Dynamical& a, const Dynamical& b, const vec2 r1, const vec2 r2, const vec2 n)
	{
		return dot(relative_velocity(a, b, r1, r2), n);
	}

	static real k_scalar_single(const InversePhysicalProps& props, const vec2 r, const vec2 n)
	{
		const real rcn = perpDot(r, n);
		return props.massInverse + props.momentInverse * rcn * rcn;
	}

	static real k_scalar(const InversePhysicalProps& a, const InversePhysicalProps& b, const vec2 r1, const vec2 r2, const vec2 n)
	{
		const real value = k_scalar_single(a, r1, n) + k_scalar_single(b, r2, n);

		return value;
	}

	// TODO SIMD
	static mat2x2 k_tensor(const InversePhysicalProps& a_physics, const InversePhysicalProps& b_physics, const vec2 r1, const vec2 r2)
	{
		const real m_sum = a_physics.massInverse + b_physics.massInverse;

		// start with Identity*m_sum
		real k11 = m_sum, k12 = 0.0f;
		real k21 = 0.0f, k22 = m_sum;

		// add the influence from r1
		real a_i_inv = a_physics.momentInverse;
		real r1xsq = r1.x * r1.x * a_i_inv;
		real r1ysq = r1.y * r1.y * a_i_inv;
		real r1nxy = -r1.x * r1.y * a_i_inv;
		k11 += r1ysq;
		k12 += r1nxy;
		k21 += r1nxy;
		k22 += r1xsq;

		// add the influnce from r2
		real b_i_inv = b_physics.momentInverse;
		real r2xsq = r2.x * r2.x * b_i_inv;
		real r2ysq = r2.y * r2.y * b_i_inv;
		real r2nxy = -r2.x * r2.y * b_i_inv;
		k11 += r2ysq;
		k12 += r2nxy;
		k21 += r2nxy;
		k22 += r2xsq;

		// invert
		real det = k11 * k22 - k12 * k21;

		real det_inv = 1.0f / det;
		return {
			k22 * det_inv, -k12 * det_inv,
			-k21 * det_inv, k11 * det_inv
		};
	}
};
