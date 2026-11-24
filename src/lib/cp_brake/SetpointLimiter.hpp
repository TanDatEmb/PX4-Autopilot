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
 * @file SetpointLimiter.hpp
 *
 * The limiter hook of cp_brake (collision-prevention ICD-CP-04, ADR 0008, ADR 0014, OPEN-035): a pure
 * function that applies the per-sector caps of cp_brake_constraints to the horizontal velocity
 * setpoint and to the horizontal feed-forward acceleration, in local NED. No PX4 type, no clock, no
 * state: everything it needs is given, so calling it again with the same inputs gives the same
 * result (PX4 calls the position controller up to three times per cycle). It never touches the
 * vertical axis, the yaw or the attitude.
 *
 * Paths, in order:
 * - Transparent, setpoints untouched: CPB_EN is off, the vehicle has not reached
 *   TAKEOFF_STATE_FLIGHT (ADR 0009), or the navigation state is neither Position nor Mission (the
 *   result carries a reason for that last case).
 * - Safe stop (zero horizontal velocity, feed-forward cleared, plus a reason): no constraint was ever
 *   received, or it is older than CPB_CSTR_AGE, or that limit is not a positive number (a sentinel),
 *   or the constraint says safe stop or is not valid, or its heading is missing (the caps cannot be
 *   rotated) or not a number.
 * - No action: the constraint says the hook may not act (enforcement_active false), which is how
 *   cp_brake reports an observer-only state or an invalid PX4 estimator or heading (ADR 0007).
 * - Limiting (ADR 0025): the direction of the velocity setpoint is kept and only its speed is scaled,
 *   v_out = s v_sp with s = min(1, cap_i / (v_sp . u_i)) over the sectors that count, where u_i is the
 *   direction of the sector rotated to local NED with the heading of the constraint (the heading of
 *   the sample time, not the current one). A sector counts when it may hold an obstacle within the
 *   clearance of the ray along the setpoint or along the current velocity: distance_i *
 *   sin(max(0, angle - 2.5 deg)) <= clearance. There is no global minimum range and no sliding. A cap
 *   that is not a finite number >= 0 counts as 0, and a distance or clearance that is not a usable
 *   number makes the sector count (fail closed). When the speed is scaled the horizontal feed-forward
 *   acceleration is cleared (ADR 0020), otherwise it is unchanged; a feed-forward that is not finite
 *   stays as it is.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "CapLimiter.hpp"

namespace cp_brake
{

/** Where and when the hook runs. Every field is given; the all-zero value is a disabled hook. */
struct HookContext {
	bool enabled;                  ///< CPB_EN is on
	bool in_flight;                ///< the takeoff state is TAKEOFF_STATE_FLIGHT
	bool nav_state_enforced;       ///< the navigation state is Position or Mission
	uint64_t now_us;               ///< the time at which the hook applies the constraint [us]
	float constraint_age_limit_s;  ///< CPB_CSTR_AGE [s]; not a positive number means every constraint is stale
};

/** The latest cp_brake_constraints, as the hook needs it. The all-zero value is "none received". */
struct HookConstraint {
	bool present;                  ///< a constraint has been received at all
	uint64_t timestamp_us;         ///< publication time of the constraint [us]
	float heading_rad;             ///< heading at the sample time, rotates the body FRD caps to local NED [rad]
	bool heading_valid;            ///< a heading exists for the sample time
	float caps_m_s[kSectorCount];  ///< speed cap per sector [m/s], body FRD, sector 0 forward, clockwise
	float distances_m[kSectorCount]; ///< distance whose stopping speed is the cap, per sector [m]
	float clearance_m;             ///< CPB_MARGIN: the half-width of the corridor of ADR 0025 [m]
	bool enforcement_active;      ///< the hook may act on the caps
	bool valid;                    ///< the caps may be used
	uint8_t state;                 ///< kState* of the state machine
	uint32_t reason_flags;         ///< kReason* bits of the state machine
};

/** What the hook did. The values are the ACTION_* constants of the cp_brake_hook message. */
enum class HookAction : uint8_t {
	kTransparent = 0,  ///< disabled, before flight, or another navigation state: nothing was read or changed
	kNoAction = 1,     ///< the constraint says no action (observer-only, or an unqualified PX4 state)
	kClear = 2,        ///< the limiter ran and no setpoint exceeded a cap
	kLimited = 3,      ///< the limiter reduced the velocity setpoint and cleared the horizontal feed-forward
	kSafeStop = 4      ///< zero horizontal velocity and feed-forward cleared
};

/** Sector index meaning "none". */
constexpr uint8_t kNoSector{255};

struct HookResult {
	HookAction action;      ///< what the hook did
	uint32_t reason_flags;  ///< kReason* bits: those of the constraint when it was used, plus the hook's own
	uint8_t active_sector;  ///< the sector that sets the scale of the velocity setpoint, or kNoSector
};

/**
 * Applies the hook.
 *
 * @param context the situation of this call
 * @param constraint the latest cp_brake_constraints
 * @param vel_ned_m_s [north, east] current horizontal velocity [m/s]; not finite or zero: no direction
 * @param vel_sp_ned_m_s [north, east] horizontal velocity setpoint [m/s], changed in place only when
 *        the result is kLimited or kSafeStop; a setpoint that is not finite is not a velocity command
 *        and is left alone, except by a safe stop
 * @param acc_sp_ned_m_s2 [north, east] horizontal feed-forward acceleration [m/s^2], changed in place
 *        under the same rule; NaN means no feed-forward and stays NaN, except in a safe stop
 */
HookResult limitHorizontalSetpoint(const HookContext &context, const HookConstraint &constraint,
				   const float vel_ned_m_s[2], float vel_sp_ned_m_s[2], float acc_sp_ned_m_s2[2]);

} // namespace cp_brake
