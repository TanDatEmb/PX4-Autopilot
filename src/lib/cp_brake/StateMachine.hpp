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
 * @file StateMachine.hpp
 *
 * Operational state of cp_brake (architecture section 4, ICD-CP-04, ADR 0007, ADR 0009): a pure
 * function from the inputs of one evaluation to the state and the reason flags. No PX4 type, no
 * clock, no default value: the sample age, its limit and every other input are given.
 *
 * States: DISABLED, OBSERVER_ONLY, NOT_READY, ACTIVE_CLEAR, BRAKING, SAFE_STOP. UNQUALIFIED is a
 * reason flag, not a state.
 *
 * - DISABLED: the module is off (CPB_EN). This is also the logged disable that clears a safe stop.
 * - Before TAKEOFF_STATE_FLIGHT the constraint is never enforced (ADR 0009): the state is
 *   OBSERVER_ONLY when everything is valid and NOT_READY otherwise (an indication only; it does not
 *   block arming). SAFE_STOP and BRAKING are not entered on the ground.
 * - From TAKEOFF_STATE_FLIGHT, unknown, invalid, stale or unconfigured data is no evidence of free
 *   space (SYS-CP-004, SYS-CP-007): SAFE_STOP, zero horizontal velocity. If the PX4 estimator or
 *   heading is itself invalid the claim is UNQUALIFIED: NOT_READY with that reason, and CP adds no
 *   action of its own, so existing PX4 failsafes act (ADR 0007). Valid data gives ACTIVE_CLEAR, or
 *   BRAKING while a cap limits the speed. Persistent blockage stays BRAKING: there is no timer and
 *   no escalation. The module never requests a mode (ADR 0007).
 * - SAFE_STOP is a latch. It leaves only through the pilot acknowledgement together with a
 *   revalidation in the same evaluation, or through a pilot override. The acknowledgement is an
 *   event of this evaluation and is not remembered: an acknowledgement given while the data is
 *   still invalid is lost, and persistent invalid data keeps SAFE_STOP. The latch is carried
 *   across phases, so a vehicle that landed in SAFE_STOP still needs the acknowledgement.
 * - A pilot override (logged by the caller) releases the latch and gives OBSERVER_ONLY with the
 *   override reason, whatever the data says. How an override is requested is OPEN-020.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

/** The values equal the STATE_* constants of the CpBrakeConstraints message. */
constexpr uint8_t kStateDisabled{0};
constexpr uint8_t kStateObserverOnly{1};
constexpr uint8_t kStateNotReady{2};
constexpr uint8_t kStateActiveClear{3};
constexpr uint8_t kStateBraking{4};
constexpr uint8_t kStateSafeStop{5};

/** TAKEOFF_STATE_FLIGHT of the takeoff_status message. Any other value is observer-only. */
constexpr uint8_t kTakeoffStateFlight{5};

struct StateInputs {
	bool enabled;                ///< CPB_EN is on
	uint32_t config_faults;      ///< ConfigFaults bit set from checkConfig; 0 means a valid configuration
	uint32_t input_flags;        ///< ReasonFlags of the latest input check (frame and data bits are used)
	bool has_sample;             ///< a map has been accepted
	float sample_age_s;          ///< age of the latest accepted map [s]; negative or non-finite is stale
	float sample_age_stop_s;     ///< CPB_AGE_STOP [s]; a limit that is not a positive number is stale
	bool heading_valid;          ///< a heading exists for the sample time (false until issue #94)
	bool px4_state_valid;        ///< the PX4 estimator and heading are valid; false gives UNQUALIFIED
	uint8_t takeoff_state;       ///< takeoff_status.takeoff_state
	bool limiting;               ///< some cap is below the maximum speed
	bool blocked;                ///< the caps leave no horizontal motion
	bool pilot_ack;              ///< the pilot acknowledged in this evaluation (event, not a level)
	bool pilot_override;         ///< a pilot override is active
	bool safe_stop_latched;      ///< StateResult::safe_stop_latched of the previous evaluation
};

struct StateResult {
	uint8_t state;               ///< kState*
	uint32_t reason_flags;       ///< kReason* bits that say why the state is what it is
	bool constraint_valid;       ///< the caps may be used (no kReasonConstraintFaults bit)
	bool safe_stop_latched;      ///< give this back as StateInputs::safe_stop_latched
};

/** Pure and deterministic: the same inputs give the same result. */
StateResult evaluateState(const StateInputs &inputs);

} // namespace cp_brake
