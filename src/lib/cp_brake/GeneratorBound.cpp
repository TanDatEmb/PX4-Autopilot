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

GeneratorBound generatorSpeedBound(const HookContext &context, const HookConstraint &constraint,
				   const float vel_ned_m_s[2], const float dir_ned[2], const float speed_max_m_s,
				   const StoppingParams &forward, const StoppingParams &side)
{
	// Probe at the highest speed any cap can have (CPB_MAX_V), not only at the generator's own top speed: a cap
	// that is still above the top speed must be seen while the generator accelerates toward it, or it is found
	// only when it drops below, with the full acceleration still to ramp down.
	const float probe_m_s = fmaxf(speed_max_m_s, fmaxf(forward.max_speed_m_s, side.max_speed_m_s));
	float vel_sp[2];
	vel_sp[0] = probe_m_s * dir_ned[0];
	vel_sp[1] = probe_m_s * dir_ned[1];
	float acc_sp[2] = {NAN, NAN};
	const HookResult result = limitHorizontalSetpoint(context, constraint, vel_ned_m_s, vel_sp, acc_sp);
	const float allowed = sqrtf(vel_sp[0] * vel_sp[0] + vel_sp[1] * vel_sp[1]);

	if (result.action == HookAction::kSafeStop) {
		return GeneratorBound{true, 0.f, forward.a_brake_m_s2, forward.jerk_m_s3};
	}

	if (result.action != HookAction::kLimited) {
		return GeneratorBound{false, speed_max_m_s, 0.f, 0.f};
	}

	if (result.active_sector >= kSectorCount) {
		// the speed lock (SYS-CP-031) binds: it is a class limit outside the forward class and does not fall
		return GeneratorBound{true, allowed, side.a_brake_m_s2, side.jerk_m_s3};
	}

	const std::size_t sector = result.active_sector;
	const StoppingParams &model = (sectorBearingDeg(sector) <= kForwardClassDeg) ? forward : side;
	const float raw_cap = constraint.caps_m_s[sector];
	const float cap = (std::isfinite(raw_cap) && raw_cap >= 0.f) ? raw_cap : 0.f;
	const float slope = (stoppingDistance(cap + kSlopeStepM_s, model) - stoppingDistance(cap, model)) / kSlopeStepM_s;

	if (!std::isfinite(slope) || !(slope > 0.f) || !(model.jerk_m_s3 > 0.f)) {
		return GeneratorBound{true, 0.f, model.a_brake_m_s2, model.jerk_m_s3}; // no usable model: fail closed
	}

	// the cap of the sector falls by dc/dD = 1 / slope per metre, and the generator closes on it at allowed m/s
	const float rate = allowed / slope;
	const float margin = rate * rate / model.jerk_m_s3; // twice the lag rate^2 / (2 J)
	const float speed = allowed - margin;
	return GeneratorBound{true, speed > 0.f ? speed : 0.f, model.a_brake_m_s2, model.jerk_m_s3};
}

float boundAcceleration(const float speed_m_s, const float bound_m_s, const float jerk_m_s3, const float brake_m_s2,
			const float dt_s)
{
	const float headroom = bound_m_s - speed_m_s;
	const float jerk = jerk_m_s3 > 0.f ? jerk_m_s3 : 0.f;

	if (!(headroom < 0.f)) {
		const float ramp = sqrtf(2.f * jerk * headroom);
		// the continuous law crosses the bound within one step when the headroom is below 2 J dt^2
		const float step = (dt_s > 0.f) ? headroom / dt_s : ramp;
		return ramp < step ? ramp : step;
	}

	const float brake = sqrtf(2.f * jerk * -headroom);
	return -(brake < brake_m_s2 ? brake : brake_m_s2);
}

} // namespace cp_brake
