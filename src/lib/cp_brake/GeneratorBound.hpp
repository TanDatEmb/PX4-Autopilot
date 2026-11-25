/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file GeneratorBound.hpp
 *
 * The cp_brake rule as a bound for a trajectory generator (ADR 0026): the generator keeps its own velocity within
 * the caps, so the hook of ADR 0014, which stays the backstop, does not have to cut in normal flight. Pure logic, no
 * PX4 type, no clock.
 */

#pragma once

#include "SetpointLimiter.hpp"
#include "StoppingModel.hpp"

namespace cp_brake
{

struct GeneratorBound {
	bool limited;      ///< false: nothing bounds the direction, speed_m_s is the generator's own fastest speed
	float speed_m_s;   ///< highest speed the generator may have along the direction [m/s], >= 0
	float brake_m_s2;  ///< braking acceleration of the class that binds [m/s^2], valid when limited
	float jerk_m_s3;   ///< jerk to follow the bound with [m/s^3]: the class jerk or a lower generator limit, valid when limited
	float fall_per_m;  ///< [(m/s)/m] how much the bound falls per metre closed on its obstacle, 0 for the lock or none
};

/**
 * Speed bound along the unit direction dir (local NED) for a generator whose fastest speed is speed_max_m_s.
 *
 * It is the speed the hook would let through in that direction (limitHorizontalSetpoint, same context, constraint
 * and current velocity, probed above every cap so that a bound above the top speed or saturated is reported too),
 * lowered by a lag margin when an obstacle sector binds. A generator that brakes with a limited jerk lags a falling
 * cap by rate^2 / (2 J), rate being how fast the cap falls while the vehicle moves at the bound; the margin is twice
 * that, so the generator settles that far under the cap instead of on it, where the hook would cut. The speed lock
 * does not fall and has no margin. A constraint the hook turns into a safe stop gives 0. Braking uses the stopping
 * model of the class of the sector that binds (forward or side, ADR 0026). follower_jerk_m_s3 is the generator's own
 * jerk limit: when it is below the class jerk the generator follows with it, and the margin is sized with it.
 */
GeneratorBound generatorSpeedBound(const HookContext &context, const HookConstraint &constraint,
				   const float vel_ned_m_s[2], const float dir_ned[2], float speed_max_m_s,
				   const StoppingParams &forward, const StoppingParams &side, float follower_jerk_m_s3);

/**
 * Highest acceleration along the bound direction for a generator at speed_m_s and acceleration accel_m_s2 that
 * integrates over dt_s, toward a bound that falls by fall_per_m per metre closed (GeneratorBound::fall_per_m), so at
 * speed_m_s * fall_per_m. Below the bound: what a ramp at jerk_m_s3 can still bring to the bound's own rate exactly
 * when the speed meets it (the fall grows with the generator's own acceleration, which uses fall_per_m * accel of the
 * jerk), and never more than one step can take; above it, a braking acceleration of the same form. Never a
 * deceleration harder than brake_m_s2.
 */
float boundAcceleration(float speed_m_s, float accel_m_s2, float bound_m_s, float fall_per_m, float jerk_m_s3,
			float brake_m_s2, float dt_s);

} // namespace cp_brake
