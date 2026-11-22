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

#include "StateMachine.hpp"

#include <cmath>

#include "ReasonFlags.hpp"

namespace cp_brake
{

namespace
{

// Fail closed: no sample, an age that is not a non-negative number, or a limit that is not a
// positive number all count as stale. A sample at exactly the limit is stale (PERF-04: safe stop
// at 0.5 s).
bool sampleIsStale(const StateInputs &in)
{
	if (!in.has_sample || !std::isfinite(in.sample_age_s) || in.sample_age_s < 0.f
	    || !std::isfinite(in.sample_age_stop_s) || in.sample_age_stop_s <= 0.f) {
		return true;
	}

	return in.sample_age_s >= in.sample_age_stop_s;
}

} // namespace

StateResult evaluateState(const StateInputs &in)
{
	StateResult out{kStateDisabled, 0U, false, false};

	if (!in.enabled) {
		return out; // the logged disable: nothing is reported and a latched safe stop is cleared
	}

	uint32_t reasons = in.input_flags & (kReasonFrameInvalid | kReasonDataInvalid);

	// Issue #112: a heading that became good only after the map was sampled leaves the PX4 state not qualified yet,
	// the existing UNQUALIFIED path (NOT_READY, nothing latched, PX4 failsafes act, ADR 0007). Never past the stale
	// limit: a map stream that stopped must still reach the stale safe stop.
	const bool qualified = in.px4_state_valid && !(in.heading_not_yet_good && !sampleIsStale(in));

	if (in.config_faults != 0U) {
		reasons |= kReasonInvalidConfig;
	}

	if (sampleIsStale(in)) {
		reasons |= kReasonSampleStale;
	}

	out.constraint_valid = (reasons & kReasonConstraintFaults) == 0U;

	if (!in.heading_valid) {
		reasons |= kReasonNoHeading;
	}

	// a map whose time alignment is not good is not used and gives caps of 0: a reason and a
	// readiness fault, not a safe stop on its own (the unused map does not refresh the sample, so
	// the stale rule above takes over after the age limit)
	if (!in.time_sync_good) {
		reasons |= kReasonTimeSync;
	}

	if (!qualified) {
		reasons |= kReasonUnqualified;
	}

	if (in.blocked && out.constraint_valid) {
		reasons |= kReasonBlocked; // the caps mean nothing when they are not usable
	}

	if (in.pilot_override) {
		out.state = kStateObserverOnly; // released and not enforced, whatever the data says
		out.reason_flags = reasons | kReasonOverride;
		return out;
	}

	const bool faulted = (reasons & (kReasonConstraintFaults | kReasonNoHeading)) != 0U;
	bool latched = in.safe_stop_latched;

	// the only exit: the acknowledgement of this evaluation together with a revalidation of it
	if (latched && in.pilot_ack && !faulted && qualified && in.time_sync_good) {
		latched = false;
	}

	if (in.takeoff_state != kTakeoffStateFlight) {
		// ADR 0009: nothing is enforced before TAKEOFF_STATE_FLIGHT, so no safe stop and no braking
		out.state = (latched || faulted || !qualified || !in.time_sync_good) ? kStateNotReady
			    : kStateObserverOnly;

	} else if (latched) {
		out.state = kStateSafeStop;

	} else if (!qualified) {
		out.state = kStateNotReady; // unqualified: CP adds no action, PX4 failsafes act (ADR 0007)

	} else if (faulted) {
		out.state = kStateSafeStop; // no evidence of free space
		latched = true;

	} else {
		out.state = in.limiting ? kStateBraking : kStateActiveClear;
	}

	if (latched) {
		reasons |= kReasonAckRequired;
	}

	out.reason_flags = reasons;
	out.safe_stop_latched = latched;
	return out;
}

bool isEnforcingState(uint8_t state)
{
	return state == kStateActiveClear || state == kStateBraking || state == kStateSafeStop;
}

} // namespace cp_brake
