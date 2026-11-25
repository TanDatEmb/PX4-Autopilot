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

#include "GeneratorBound.hpp"

#include <cmath>
#include <math.h>

#include "CapLimiter.hpp"

namespace cp_brake
{

namespace
{

// Speed step of the finite difference that gives the slope of the stopping distance. A numerical step, not a
// safety value: the stopping distance is smooth, and 0.01 m/s changes the slope by far less than 1 %.
constexpr float kSlopeStepM_s{0.01f};

} // namespace

GeneratorBounds generatorSpeedBounds(const HookContext &context, const HookConstraint &constraint,
				     const float vel_ned_m_s[2], const float dir_ned[2], const float speed_max_m_s,
				     const StoppingParams &forward, const StoppingParams &side, const float follower_jerk_m_s3)
{
	// the jerk the generator follows the bound with: the class jerk, or a lower jerk limit of the generator
	const auto follow = [follower_jerk_m_s3](const float class_jerk_m_s3) {
		return (std::isfinite(follower_jerk_m_s3) && follower_jerk_m_s3 > 0.f && follower_jerk_m_s3 < class_jerk_m_s3)
		       ? follower_jerk_m_s3 : class_jerk_m_s3;
	};

	const GeneratorBound none{false, speed_max_m_s, 0.f, 0.f, 0.f};

	// Probe above the highest speed any cap can have (CPB_MAX_V), not at the generator's own top speed: a cap still
	// above the top speed, or saturated at CPB_MAX_V, can fall and must be seen with its margin before it does, or the
	// margin appears as a step. Twice the larger speed is a numerical choice, anything above every cap would do.
	const float probe_m_s = 2.f * fmaxf(speed_max_m_s, fmaxf(forward.max_speed_m_s, side.max_speed_m_s));

	// the rule as the hook applies it, speed lock included
	float vel_sp[2];
	vel_sp[0] = probe_m_s * dir_ned[0];
	vel_sp[1] = probe_m_s * dir_ned[1];
	float acc_sp[2] = {NAN, NAN};
	const HookResult result = limitHorizontalSetpoint(context, constraint, vel_ned_m_s, vel_sp, acc_sp);

	if (result.action == HookAction::kSafeStop) {
		return GeneratorBounds{GeneratorBound{true, 0.f, forward.a_brake_m_s2, follow(forward.jerk_m_s3), 0.f}, none};
	}

	if (result.action != HookAction::kLimited) {
		return GeneratorBounds{none, none};
	}

	GeneratorBounds bounds{none, none};

	if (result.active_sector >= kSectorCount) {
		// the speed lock (SYS-CP-031) binds: a class limit outside the forward class that does not fall
		bounds.lock = GeneratorBound{true, sqrtf(vel_sp[0] * vel_sp[0] + vel_sp[1] * vel_sp[1]), side.a_brake_m_s2, follow(side.jerk_m_s3), 0.f};
	}

	// The obstacles alone, with the lock out of reach: the hook reports only the tighter of the two, so without this
	// the obstacle bound and its margin would appear only when the obstacle overtakes the lock, a step of the margin,
	// and its falling law would be lost while the lock is the lower speed.
	HookContext obstacles_only = context;
	obstacles_only.side_speed_m_s = probe_m_s;
	vel_sp[0] = probe_m_s * dir_ned[0];
	vel_sp[1] = probe_m_s * dir_ned[1];
	acc_sp[0] = NAN;
	acc_sp[1] = NAN;
	const HookResult obstacle = limitHorizontalSetpoint(obstacles_only, constraint, vel_ned_m_s, vel_sp, acc_sp);

	if (obstacle.action != HookAction::kLimited || obstacle.active_sector >= kSectorCount) {
		return bounds;
	}

	const std::size_t sector = obstacle.active_sector;
	const StoppingParams &model = (sectorBearingDeg(sector) <= kForwardClassDeg) ? forward : side;
	const float raw_cap = constraint.caps_m_s[sector];
	const float cap = (std::isfinite(raw_cap) && raw_cap >= 0.f) ? raw_cap : 0.f;
	const float slope = (stoppingDistance(cap + kSlopeStepM_s, model) - stoppingDistance(cap, model)) / kSlopeStepM_s;
	const float jerk = follow(model.jerk_m_s3);
	float speed = 0.f; // no usable model: fail closed
	float fall_per_m = 0.f;

	if (std::isfinite(slope) && (slope > 0.f) && (jerk > 0.f)) {
		// the cap of the sector falls by dc/dD = 1 / slope per metre, and the generator closes on it at allowed m/s
		const float allowed = sqrtf(vel_sp[0] * vel_sp[0] + vel_sp[1] * vel_sp[1]);
		const float rate = allowed / slope;
		const float margin = rate * rate / jerk; // twice the lag rate^2 / (2 J) of the generator
		speed = (allowed - margin > 0.f) ? allowed - margin : 0.f;
		fall_per_m = 1.f / slope;
	}

	bounds.obstacle = GeneratorBound{true, speed, model.a_brake_m_s2, jerk, fall_per_m};
	return bounds;
}

float boundAcceleration(const float speed_m_s, const float accel_m_s2, const float bound_m_s, const float fall_per_m,
			const float jerk_m_s3, const float brake_m_s2, const float dt_s)
{
	// The headroom closes at the acceleration plus the rate the bound falls (speed times fall_per_m). A ramp at the
	// jerk brings that closing acceleration to zero over (a + fall)^2 / (2 J) of headroom, so
	// a <= -fall + sqrt(2 J headroom). While the generator accelerates, the fall itself grows at fall_per_m * a,
	// which uses that much of the jerk: the ramp has J - fall_per_m * a left.
	const float per_m = (std::isfinite(fall_per_m) && fall_per_m > 0.f) ? fall_per_m : 0.f;
	const float fall = (speed_m_s > 0.f) ? speed_m_s * per_m : 0.f;
	const float headroom = bound_m_s - speed_m_s;
	const float jerk = jerk_m_s3 > 0.f ? jerk_m_s3 : 0.f;
	float acceleration;

	if (!(headroom < 0.f)) {
		const float growth = (accel_m_s2 > 0.f) ? per_m * accel_m_s2 : 0.f;
		const float jerk_left = (jerk > growth) ? jerk - growth : 0.f;
		const float ramp = sqrtf(2.f * jerk_left * headroom) - fall;
		// the continuous law crosses the bound within one step when the headroom is below about 2 J dt^2
		const float step = (dt_s > 0.f) ? headroom / dt_s - fall : ramp;
		acceleration = ramp < step ? ramp : step;

	} else {
		acceleration = -fall - sqrtf(2.f * jerk * -headroom);
	}

	return (acceleration > -brake_m_s2) ? acceleration : -brake_m_s2;
}

} // namespace cp_brake
