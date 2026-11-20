/****************************************************************************
 *
 *   Copyright (c) 2026 collision-prevention contributors. All rights reserved.
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
 * @file SetpointLimiter.cpp
 *
 * The limiter is a sequence of projections of the velocity setpoint onto the half-plane of the sector
 * that it violates most, ending when no sector is violated by more than a numerical tolerance. The
 * origin is inside every half-plane (caps are >= 0), so a projection never makes the vector longer and
 * the empty-handed answer, zero, is always feasible: if the loop does not end within the pass limit,
 * the output is zero, which is what ADR 0008 says about an empty feasible set.
 *
 * The unit vectors of the sectors come from a table of the 72 bearings and one rotation by the
 * heading, so there is no trigonometry per sector and no array on the stack: this runs on the work
 * queue of the position controller (2240 byte stack on the boards), up to three times per cycle.
 * No <array>, <limits> or <algorithm> and no std::fmod: the NuttX builds do not have them.
 */

#include "SetpointLimiter.hpp"

#include <cmath>
#include <math.h>

#include "ReasonFlags.hpp"
#include "StateMachine.hpp"
#include "TimeUnits.hpp"

namespace cp_brake
{

namespace
{

// A numerical tolerance, not a safety value: float rounding of a projection is about 1e-6 m/s at
// 12 m/s, and a cap that is met within this is met. It also keeps a sector at exactly 90 degrees from
// the motion (cap 0, product of the order of 1e-16) from counting as violated.
constexpr float kToleranceM_s{1e-4f};

// numerical tolerance of the same kind for the acceleration [m/s^2]
constexpr float kToleranceM_s2{1e-4f};

// Each projection removes the largest violation. Two sectors with nearly opposite directions can make
// the sequence converge slowly; then the output is zero.
constexpr int kMaxPasses{8};

// cos and sin of the bearing of sector i (i * 5 degrees, clockwise from forward), generated, and
// checked against cosf and sinf by the unit tests.
constexpr float kSectorCos[kSectorCount] = {
	1.000000000e+00f, 9.961946981e-01f, 9.848077530e-01f, 9.659258263e-01f,
	9.396926208e-01f, 9.063077870e-01f, 8.660254038e-01f, 8.191520443e-01f,
	7.660444431e-01f, 7.071067812e-01f, 6.427876097e-01f, 5.735764364e-01f,
	5.000000000e-01f, 4.226182617e-01f, 3.420201433e-01f, 2.588190451e-01f,
	1.736481777e-01f, 8.715574275e-02f, 6.123233996e-17f, -8.715574275e-02f,
	-1.736481777e-01f, -2.588190451e-01f, -3.420201433e-01f, -4.226182617e-01f,
	-5.000000000e-01f, -5.735764364e-01f, -6.427876097e-01f, -7.071067812e-01f,
	-7.660444431e-01f, -8.191520443e-01f, -8.660254038e-01f, -9.063077870e-01f,
	-9.396926208e-01f, -9.659258263e-01f, -9.848077530e-01f, -9.961946981e-01f,
	-1.000000000e+00f, -9.961946981e-01f, -9.848077530e-01f, -9.659258263e-01f,
	-9.396926208e-01f, -9.063077870e-01f, -8.660254038e-01f, -8.191520443e-01f,
	-7.660444431e-01f, -7.071067812e-01f, -6.427876097e-01f, -5.735764364e-01f,
	-5.000000000e-01f, -4.226182617e-01f, -3.420201433e-01f, -2.588190451e-01f,
	-1.736481777e-01f, -8.715574275e-02f, -1.836970199e-16f, 8.715574275e-02f,
	1.736481777e-01f, 2.588190451e-01f, 3.420201433e-01f, 4.226182617e-01f,
	5.000000000e-01f, 5.735764364e-01f, 6.427876097e-01f, 7.071067812e-01f,
	7.660444431e-01f, 8.191520443e-01f, 8.660254038e-01f, 9.063077870e-01f,
	9.396926208e-01f, 9.659258263e-01f, 9.848077530e-01f, 9.961946981e-01f
};

constexpr float kSectorSin[kSectorCount] = {
	0.000000000e+00f, 8.715574275e-02f, 1.736481777e-01f, 2.588190451e-01f,
	3.420201433e-01f, 4.226182617e-01f, 5.000000000e-01f, 5.735764364e-01f,
	6.427876097e-01f, 7.071067812e-01f, 7.660444431e-01f, 8.191520443e-01f,
	8.660254038e-01f, 9.063077870e-01f, 9.396926208e-01f, 9.659258263e-01f,
	9.848077530e-01f, 9.961946981e-01f, 1.000000000e+00f, 9.961946981e-01f,
	9.848077530e-01f, 9.659258263e-01f, 9.396926208e-01f, 9.063077870e-01f,
	8.660254038e-01f, 8.191520443e-01f, 7.660444431e-01f, 7.071067812e-01f,
	6.427876097e-01f, 5.735764364e-01f, 5.000000000e-01f, 4.226182617e-01f,
	3.420201433e-01f, 2.588190451e-01f, 1.736481777e-01f, 8.715574275e-02f,
	1.224646799e-16f, -8.715574275e-02f, -1.736481777e-01f, -2.588190451e-01f,
	-3.420201433e-01f, -4.226182617e-01f, -5.000000000e-01f, -5.735764364e-01f,
	-6.427876097e-01f, -7.071067812e-01f, -7.660444431e-01f, -8.191520443e-01f,
	-8.660254038e-01f, -9.063077870e-01f, -9.396926208e-01f, -9.659258263e-01f,
	-9.848077530e-01f, -9.961946981e-01f, -1.000000000e+00f, -9.961946981e-01f,
	-9.848077530e-01f, -9.659258263e-01f, -9.396926208e-01f, -9.063077870e-01f,
	-8.660254038e-01f, -8.191520443e-01f, -7.660444431e-01f, -7.071067812e-01f,
	-6.427876097e-01f, -5.735764364e-01f, -5.000000000e-01f, -4.226182617e-01f,
	-3.420201433e-01f, -2.588190451e-01f, -1.736481777e-01f, -8.715574275e-02f
};

// the direction of a sector in local NED for a heading with cos ch and sin sh
inline void sectorDirection(const std::size_t sector, const float ch, const float sh, float &north, float &east)
{
	north = ch * kSectorCos[sector] - sh * kSectorSin[sector];
	east = sh * kSectorCos[sector] + ch * kSectorSin[sector];
}

// a cap that is not a finite number >= 0 is no evidence of free space: 0
inline float usableCap(const float cap)
{
	return (std::isfinite(cap) && cap >= 0.f) ? cap : 0.f;
}

// Fail closed, like the sample age of the state machine: no constraint, a limit that is not a positive
// number, a constraint stamped after now, or an age at or above the limit is stale.
bool constraintIsStale(const HookContext &context, const HookConstraint &constraint)
{
	if (!constraint.present || !std::isfinite(context.constraint_age_limit_s) || context.constraint_age_limit_s <= 0.f
	    || constraint.timestamp_us > context.now_us) {
		return true;
	}

	return (context.now_us - constraint.timestamp_us) >= secondsToMicroseconds(context.constraint_age_limit_s);
}

HookResult safeStop(const uint32_t reason_flags, float vel_sp[2], float acc_sp[2])
{
	vel_sp[0] = 0.f;
	vel_sp[1] = 0.f;
	acc_sp[0] = 0.f;
	acc_sp[1] = 0.f;
	return HookResult{HookAction::kSafeStop, reason_flags, kNoSector};
}

} // namespace

HookResult limitHorizontalSetpoint(const HookContext &context, const HookConstraint &constraint,
				   const float vel_state_ned_m_s[2], float vel_sp_ned_m_s[2], float acc_sp_ned_m_s2[2])
{
	if (!context.enabled || !context.in_flight) {
		return HookResult{HookAction::kTransparent, 0U, kNoSector};
	}

	if (!context.nav_state_enforced) {
		return HookResult{HookAction::kTransparent, kReasonNavState, kNoSector};
	}

	// from here on the hook is enforcing
	if (constraintIsStale(context, constraint)) {
		return safeStop(kReasonConstraintStale, vel_sp_ned_m_s, acc_sp_ned_m_s2);
	}

	if (!constraint.enforcement_active) {
		// observer-only, or the PX4 estimator or heading is invalid: CP adds no action (ADR 0007)
		return HookResult{HookAction::kNoAction, constraint.reason_flags, kNoSector};
	}

	if (!isEnforcingState(constraint.state)) {
		// the constraint allows enforcement in a state that does not enforce: it cannot be trusted
		return safeStop(constraint.reason_flags | kReasonConstraintMalformed, vel_sp_ned_m_s, acc_sp_ned_m_s2);
	}

	if (constraint.state == kStateSafeStop || !constraint.valid) {
		return safeStop(constraint.reason_flags, vel_sp_ned_m_s, acc_sp_ned_m_s2);
	}

	if (!constraint.heading_valid || !std::isfinite(constraint.heading_rad)) {
		// no transform from body to NED for the sample time: no valid direction for any cap
		const uint32_t malformed = (constraint.heading_valid ? kReasonConstraintMalformed : 0U);
		return safeStop(constraint.reason_flags | kReasonNoHeading | malformed, vel_sp_ned_m_s, acc_sp_ned_m_s2);
	}

	// the limiter: a setpoint that is not a velocity command has nothing to limit
	const float vel_in[2] = {vel_sp_ned_m_s[0], vel_sp_ned_m_s[1]};
	HookResult result{HookAction::kClear, constraint.reason_flags, kNoSector};

	if (!std::isfinite(vel_in[0]) || !std::isfinite(vel_in[1])) {
		return result;
	}

	// a sector with a cap at or above the speed cannot be violated: its direction is not computed
	const float speed_in = sqrtf(vel_in[0] * vel_in[0] + vel_in[1] * vel_in[1]);
	const float ch = cosf(constraint.heading_rad);
	const float sh = sinf(constraint.heading_rad);

	float vel[2] = {vel_in[0], vel_in[1]};
	float most_excess = 0.f;
	bool limited = false;
	bool converged = false;

	for (int pass = 0; pass <= kMaxPasses; ++pass) {
		std::size_t worst = kSectorCount;
		float worst_excess = kToleranceM_s;
		float worst_north = 0.f;
		float worst_east = 0.f;

		for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
			const float cap = usableCap(constraint.caps_m_s[sector]);

			if (cap >= speed_in) {
				continue;
			}

			float north;
			float east;
			sectorDirection(sector, ch, sh, north, east);
			const float excess = vel[0] * north + vel[1] * east - cap;

			if (excess > worst_excess) {
				worst = sector;
				worst_excess = excess;
				worst_north = north;
				worst_east = east;
			}
		}

		if (worst == kSectorCount) {
			converged = true;
			break;
		}

		if (pass == kMaxPasses) {
			break;
		}

		// only the component entering the sector is removed
		vel[0] -= worst_excess * worst_north;
		vel[1] -= worst_excess * worst_east;
		limited = true;

		if (worst_excess > most_excess) {
			most_excess = worst_excess;
			result.active_sector = static_cast<uint8_t>(worst);
		}
	}

	if (!converged) {
		// no vector within the pass limit satisfies every sector: nothing is the only safe answer
		vel[0] = 0.f;
		vel[1] = 0.f;
	}

	if (!limited && converged) {
		return result;
	}

	result.action = HookAction::kLimited;
	vel_sp_ned_m_s[0] = vel[0];
	vel_sp_ned_m_s[1] = vel[1];

	// no feed-forward means nothing to remove, and NaN stays NaN
	if (!std::isfinite(acc_sp_ned_m_s2[0]) || !std::isfinite(acc_sp_ned_m_s2[1])) {
		return result;
	}

	if (!converged) {
		acc_sp_ned_m_s2[0] = 0.f;
		acc_sp_ned_m_s2[1] = 0.f;
		return result;
	}

	// The feed-forward toward a sector whose cap cut the velocity setpoint is limited to what is left after
	// removing it in proportion to how close the vehicle already is to that cap: all of it at the cap or
	// above it (or when the cap is 0 or the vehicle velocity is not a number), none at rest or moving away,
	// so the trajectory cannot push the vehicle through a cap and the removal has no step. Each such
	// sector is a half-plane on the acceleration, a . u_i <= allowed_i with allowed_i >= 0, and they are
	// projected the same way as the velocity until every one holds, because removing the acceleration
	// toward one sector can bring it back toward another. Zero satisfies all of them, and is the answer
	// if the passes run out.
	const float acc_in[2] = {acc_sp_ned_m_s2[0], acc_sp_ned_m_s2[1]};
	const bool state_known = std::isfinite(vel_state_ned_m_s[0]) && std::isfinite(vel_state_ned_m_s[1]);
	float acc[2] = {acc_in[0], acc_in[1]};
	bool acc_converged = false;

	for (int pass = 0; pass <= kMaxPasses; ++pass) {
		std::size_t worst = kSectorCount;
		float worst_excess = kToleranceM_s2;
		float worst_north = 0.f;
		float worst_east = 0.f;

		for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
			const float cap = usableCap(constraint.caps_m_s[sector]);

			if (cap >= speed_in) {
				continue;
			}

			float north;
			float east;
			sectorDirection(sector, ch, sh, north, east);

			if (vel_in[0] * north + vel_in[1] * east - cap <= kToleranceM_s) {
				continue; // this cap did not limit the setpoint
			}

			float allowed = 0.f;
			const float toward_in = acc_in[0] * north + acc_in[1] * east;

			if (toward_in > 0.f) {
				float closeness = 1.f;

				if (state_known && cap > 0.f) {
					const float speed_toward = vel_state_ned_m_s[0] * north + vel_state_ned_m_s[1] * east;
					closeness = (speed_toward <= 0.f) ? 0.f : ((speed_toward >= cap) ? 1.f : speed_toward / cap);
				}

				allowed = (1.f - closeness) * toward_in;
			}

			const float excess = acc[0] * north + acc[1] * east - allowed;

			if (excess > worst_excess) {
				worst = sector;
				worst_excess = excess;
				worst_north = north;
				worst_east = east;
			}
		}

		if (worst == kSectorCount) {
			acc_converged = true;
			break;
		}

		if (pass == kMaxPasses) {
			break;
		}

		acc[0] -= worst_excess * worst_north;
		acc[1] -= worst_excess * worst_east;
	}

	if (!acc_converged) {
		acc[0] = 0.f;
		acc[1] = 0.f;
	}

	acc_sp_ned_m_s2[0] = acc[0];
	acc_sp_ned_m_s2[1] = acc[1];
	return result;
}

} // namespace cp_brake
