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
 * The limiter (ADR 0025) is one pass over the sectors: the velocity setpoint is scaled by the smallest
 * cap_i / (v_sp . u_i) among the sectors it exceeds that may hold an obstacle within the clearance of
 * its path. The direction never changes; the origin is inside every half-plane (caps are >= 0), so the
 * scaled vector meets every sector that counts and is never longer than the input; a cap of 0 on the
 * path gives zero, which is what ADR 0008 says about a blocked direction.
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

// A numerical tolerance, not a safety value: float rounding of a product is about 1e-6 m/s at 12 m/s,
// and a cap that is met within this is met. It also keeps a sector at exactly 90 degrees from the
// motion (cap 0, product of the order of 1e-16) from counting as exceeded.
constexpr float kToleranceM_s{1e-4f};

constexpr float kDegToRad{3.14159265358979323846f / 180.f};

// cos and sin of half a sector (2.5 degrees): sectors are centred on their bearing (ICD-CP-03)
constexpr float kHalfSectorCos{9.990482216e-01f};
constexpr float kHalfSectorSin{4.361938737e-02f};

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

// ADR 0025 decision 1: true when the sector with direction (u_n, u_e) may hold an obstacle within the
// clearance of the ray along the unit direction (dir_n, dir_e). Every point of the sector at a range of
// at least distance_m lies at least distance_m * sin(angle - half sector) from the ray. A distance or a
// clearance that is not a number makes the comparison false, so the sector counts (fail closed).
inline bool inCorridor(const float dir_n, const float dir_e, const float u_n, const float u_e,
		       const float distance_m, const float clearance_m)
{
	const float cos_angle = dir_n * u_n + dir_e * u_e;

	if (!(cos_angle > 0.f)) {
		return false; // the ray does not enter the sector
	}

	const float sin_angle = fabsf(dir_n * u_e - dir_e * u_n);
	const float sin_edge = sin_angle * kHalfSectorCos - cos_angle * kHalfSectorSin; // the nearest edge
	const float offset_m = distance_m * (sin_edge > 0.f ? sin_edge : 0.f);
	return !(offset_m > clearance_m);
}

// True when the unit direction (dir_n, dir_e) is more than the angle whose cosine is cos_limit from the
// nose (nose_n, nose_e). A nose or a limit that is not a number gives true (fail closed). The tolerance is
// numerical only: a direction exactly at the limit stays inside, as the class definition says.
inline bool beyondAngle(const float dir_n, const float dir_e, const float nose_n, const float nose_e,
			const float cos_limit)
{
	return !(dir_n * nose_n + dir_e * nose_e >= cos_limit - 1e-6f);
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
				   const float vel_ned_m_s[2], float vel_sp_ned_m_s[2], float acc_sp_ned_m_s2[2])
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

	// a sector with a cap at or above the speed cannot be exceeded: its direction is not computed
	const float speed_in = sqrtf(vel_in[0] * vel_in[0] + vel_in[1] * vel_in[1]);

	if (!(speed_in > 0.f)) {
		return result; // no motion asked: nothing to limit
	}

	const float ch = cosf(constraint.heading_rad);
	const float sh = sinf(constraint.heading_rad);

	// the two rays of the corridor (ADR 0025 decision 2): the setpoint and, when it has one, the current
	// velocity, which covers the turn while the vehicle follows a changed command
	const float sp_dir[2] = {vel_in[0] / speed_in, vel_in[1] / speed_in};
	const float speed_now = (vel_ned_m_s != nullptr) ? sqrtf(vel_ned_m_s[0] * vel_ned_m_s[0] + vel_ned_m_s[1] * vel_ned_m_s[1])
				: 0.f;
	const bool has_vel_dir = std::isfinite(speed_now) && speed_now > 0.f;
	const float vel_dir[2] = {has_vel_dir ? vel_ned_m_s[0] / speed_now : 0.f, has_vel_dir ? vel_ned_m_s[1] / speed_now : 0.f};

	// a clearance that is not a finite number >= 0 is no evidence that a sector is off the path: every
	// sector the setpoint enters counts
	const float clearance_m = (std::isfinite(constraint.clearance_m) && constraint.clearance_m >= 0.f)
				  ? constraint.clearance_m : INFINITY;

	float scale = 1.f;

	for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
		const float cap = usableCap(constraint.caps_m_s[sector]);

		if (cap >= speed_in) {
			continue;
		}

		float north;
		float east;
		sectorDirection(sector, ch, sh, north, east);
		const float along = vel_in[0] * north + vel_in[1] * east;

		if (along - cap <= kToleranceM_s) {
			continue; // the setpoint does not exceed this cap
		}

		const float distance_m = constraint.distances_m[sector];
		const bool counts = inCorridor(sp_dir[0], sp_dir[1], north, east, distance_m, clearance_m)
				    || (has_vel_dir && inCorridor(vel_dir[0], vel_dir[1], north, east, distance_m, clearance_m));

		if (counts && cap / along < scale) {
			scale = cap / along;
			result.active_sector = static_cast<uint8_t>(sector);
		}
	}

	// SYS-CP-031: the speed lock and the unclaimed directions, judged against the current heading. The
	// current velocity counts only while it is faster than the lock: slower, it is within the lock already,
	// and at hover its direction is noise.
	const float lock_m_s = (std::isfinite(context.side_speed_m_s) && context.side_speed_m_s > 0.f)
			       ? context.side_speed_m_s : 0.f;
	const float nose_n = cosf(context.heading_rad);
	const float nose_e = sinf(context.heading_rad);
	const bool vel_counts = has_vel_dir && speed_now > lock_m_s;
	const float forward_cos = cosf(kForwardClassDeg * kDegToRad);
	const bool unclaimed_usable = std::isfinite(context.unclaimed_start_deg)
				      && context.unclaimed_start_deg > kForwardClassDeg && context.unclaimed_start_deg <= 180.f;
	const float unclaimed_cos = unclaimed_usable ? cosf(context.unclaimed_start_deg * kDegToRad) : forward_cos;

	const bool locked = beyondAngle(sp_dir[0], sp_dir[1], nose_n, nose_e, forward_cos)
			    || (vel_counts && beyondAngle(vel_dir[0], vel_dir[1], nose_n, nose_e, forward_cos));

	if (beyondAngle(sp_dir[0], sp_dir[1], nose_n, nose_e, unclaimed_cos)
	    || (vel_counts && beyondAngle(vel_dir[0], vel_dir[1], nose_n, nose_e, unclaimed_cos))) {
		result.reason_flags |= kReasonUnclaimedDirection;
	}

	// the lock scales the vector like a cap would, so the direction is kept; it then sets the scale
	const bool lock_sets_scale = locked && (speed_in * scale - lock_m_s > kToleranceM_s);

	if (lock_sets_scale) {
		scale = lock_m_s / speed_in;
		result.active_sector = kNoSector;

	} else if (result.active_sector == kNoSector) {
		return result; // nothing limits the setpoint: it and the feed-forward are unchanged
	}

	// only the speed changes, never the direction (no sliding, ADR 0025)
	vel_sp_ned_m_s[0] = vel_in[0] * scale;
	vel_sp_ned_m_s[1] = vel_in[1] * scale;

	// ADR 0020: when a sector limits the velocity setpoint the horizontal feed-forward is cleared, so the
	// trajectory cannot push the vehicle through the cap (the cost is a small acceleration step when
	// limiting starts). No feed-forward, or one that is not finite, is left as it is: NaN stays NaN.
	if (std::isfinite(acc_sp_ned_m_s2[0]) && std::isfinite(acc_sp_ned_m_s2[1])) {
		acc_sp_ned_m_s2[0] = 0.f;
		acc_sp_ned_m_s2[1] = 0.f;
	}

	result.action = HookAction::kLimited;
	return result;
}

} // namespace cp_brake
