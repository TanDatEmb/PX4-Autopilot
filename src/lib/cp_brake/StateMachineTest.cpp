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
 * @file StateMachineTest.cpp
 *
 * Unit tests of the cp_brake state machine (architecture section 4, ADR 0007, ADR 0009). The 0.5 s
 * sample age limit is the approved baseline CPB_AGE_STOP (PERF-04); every other number here is a
 * test fixture.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "ReasonFlags.hpp"
#include "StateMachine.hpp"

using namespace cp_brake;

namespace
{
constexpr float kAgeStop_s{0.5f};
constexpr uint8_t kTakeoffReady{3};

// every value of the takeoff_status message below TAKEOFF_STATE_FLIGHT, plus values it does not define
constexpr uint8_t kNotFlightStates[] = {0, 1, 2, 3, 4, 6, 7, 255};

#define EXPECT_STATE(result, expected) \
	EXPECT_EQ(static_cast<int>((result).state), static_cast<int>(expected))

// everything valid, in flight, no cap limiting
StateInputs healthy()
{
	StateInputs in{};
	in.enabled = true;
	in.config_faults = 0U;
	in.input_flags = 0U;
	in.has_sample = true;
	in.sample_age_s = 0.1f;
	in.sample_age_stop_s = kAgeStop_s;
	in.heading_valid = true;
	in.time_sync_good = true;
	in.px4_state_valid = true;
	in.takeoff_state = kTakeoffStateFlight;
	in.limiting = false;
	in.blocked = false;
	in.pilot_ack = false;
	in.pilot_override = false;
	in.safe_stop_latched = false;
	return in;
}

StateInputs healthyOnGround()
{
	StateInputs in = healthy();
	in.takeoff_state = kTakeoffReady;
	return in;
}

// the module keeps only the latch between two evaluations
StateResult step(StateInputs in, const StateResult &previous)
{
	in.safe_stop_latched = previous.safe_stop_latched;
	return evaluateState(in);
}

struct Fault {
	const char *name;
	void (*apply)(StateInputs &);
	uint32_t reason;            ///< the reason bit the fault must raise
	bool constraint_valid;      ///< whether the caps are still usable with this fault
};

// every way the data can be unusable; heading is the only one that leaves the caps usable
const Fault kFaults[] = {
	{"invalid config", [](StateInputs & in) { in.config_faults = 1U << 0; }, kReasonInvalidConfig, false},
	{"invalid config, high bit", [](StateInputs & in) { in.config_faults = 1U << 6; }, kReasonInvalidConfig, false},
	{"frame invalid", [](StateInputs & in) { in.input_flags = kReasonFrameInvalid; }, kReasonFrameInvalid, false},
	{"data invalid", [](StateInputs & in) { in.input_flags = kReasonDataInvalid; }, kReasonDataInvalid, false},
	{"no sample", [](StateInputs & in) { in.has_sample = false; }, kReasonSampleStale, false},
	{"age at the limit", [](StateInputs & in) { in.sample_age_s = kAgeStop_s; }, kReasonSampleStale, false},
	{"age above the limit", [](StateInputs & in) { in.sample_age_s = 5.f; }, kReasonSampleStale, false},
	{
		"age not finite", [](StateInputs & in) { in.sample_age_s = std::numeric_limits<float>::quiet_NaN(); },
		kReasonSampleStale, false
	},
	{
		"age infinite", [](StateInputs & in) { in.sample_age_s = std::numeric_limits<float>::infinity(); },
		kReasonSampleStale, false
	},
	{"age negative", [](StateInputs & in) { in.sample_age_s = -0.001f; }, kReasonSampleStale, false},
	{"age limit zero", [](StateInputs & in) { in.sample_age_stop_s = 0.f; }, kReasonSampleStale, false},
	{"age limit negative", [](StateInputs & in) { in.sample_age_stop_s = -1.f; }, kReasonSampleStale, false},
	{
		"age limit not finite", [](StateInputs & in) { in.sample_age_stop_s = std::numeric_limits<float>::quiet_NaN(); },
		kReasonSampleStale, false
	},
	{"no heading", [](StateInputs & in) { in.heading_valid = false; }, kReasonNoHeading, true},
};
} // namespace

// ---------------------------------------------------------------------------------------------
// DISABLED

TEST(StateMachineTest, DisabledWhateverTheOtherInputsSay)
{
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs in = healthy();
		fault.apply(in);
		in.enabled = false;
		in.pilot_override = true;
		in.blocked = true;
		in.limiting = true;
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateDisabled);
		EXPECT_EQ(out.reason_flags, 0U);
		EXPECT_FALSE(out.constraint_valid);
		EXPECT_FALSE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, DisablingIsTheLoggedDisableThatClearsASafeStop)
{
	StateInputs in = healthy();
	in.safe_stop_latched = true;
	in.enabled = false;
	const StateResult disabled = evaluateState(in);
	EXPECT_STATE(disabled, kStateDisabled);
	EXPECT_FALSE(disabled.safe_stop_latched);

	// enabled again with healthy data: no acknowledgement is waiting
	in.enabled = true;
	const StateResult enabled = step(in, disabled);
	EXPECT_STATE(enabled, kStateActiveClear);
}

// ---------------------------------------------------------------------------------------------
// Before TAKEOFF_STATE_FLIGHT: observer-only (ADR 0009, SYS-CP-014)

TEST(StateMachineTest, BeforeFlightAHealthyConstraintIsObserverOnly)
{
	for (const uint8_t takeoff_state : kNotFlightStates) {
		SCOPED_TRACE(static_cast<int>(takeoff_state));
		StateInputs in = healthy();
		in.takeoff_state = takeoff_state;
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateObserverOnly);
		EXPECT_EQ(out.reason_flags, 0U);
		EXPECT_TRUE(out.constraint_valid);
		EXPECT_FALSE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, OnlyTakeoffStateFlightIsEnforced)
{
	EXPECT_EQ(kTakeoffStateFlight, 5);
	EXPECT_STATE(evaluateState(healthy()), kStateActiveClear);
}

TEST(StateMachineTest, BeforeFlightLimitingAndBlockageDoNotLeaveObserverOnly)
{
	for (const uint8_t takeoff_state : kNotFlightStates) {
		SCOPED_TRACE(static_cast<int>(takeoff_state));
		StateInputs in = healthy();
		in.takeoff_state = takeoff_state;
		in.limiting = true;
		in.blocked = true;
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateObserverOnly);
		EXPECT_EQ(out.reason_flags, kReasonBlocked);  // reported, but nothing is enforced
	}
}

TEST(StateMachineTest, BeforeFlightAFaultIsNotReadyWithItsReasonAndNeverASafeStop)
{
	for (const Fault &fault : kFaults) {
		for (const uint8_t takeoff_state : kNotFlightStates) {
			SCOPED_TRACE(fault.name);
			SCOPED_TRACE(static_cast<int>(takeoff_state));
			StateInputs in = healthy();
			in.takeoff_state = takeoff_state;
			fault.apply(in);
			const StateResult out = evaluateState(in);
			EXPECT_STATE(out, kStateNotReady);
			EXPECT_EQ(out.reason_flags, fault.reason);
			EXPECT_EQ(out.constraint_valid, fault.constraint_valid);
			EXPECT_FALSE(out.safe_stop_latched);
		}
	}
}

TEST(StateMachineTest, BeforeFlightAnInvalidPx4StateIsNotReadyAndUnqualified)
{
	StateInputs in = healthyOnGround();
	in.px4_state_valid = false;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateNotReady);
	EXPECT_EQ(out.reason_flags, kReasonUnqualified);
	EXPECT_TRUE(out.constraint_valid);
}

// ---------------------------------------------------------------------------------------------
// In flight: ACTIVE_CLEAR and BRAKING

TEST(StateMachineTest, InFlightValidDataWithNoLimitingCapIsActiveClear)
{
	const StateResult out = evaluateState(healthy());
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_EQ(out.reason_flags, 0U);
	EXPECT_TRUE(out.constraint_valid);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, InFlightALimitingCapIsBraking)
{
	StateInputs in = healthy();
	in.limiting = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateBraking);
	EXPECT_EQ(out.reason_flags, 0U);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, BrakingReleasesToActiveClearWhenNothingLimitsAnyMore)
{
	StateInputs in = healthy();
	in.limiting = true;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateBraking);
	in.limiting = false;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
}

TEST(StateMachineTest, ActiveClearToBrakingAsSoonAsACapLimits)
{
	StateInputs in = healthy();
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateActiveClear);
	in.limiting = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateBraking);
}

TEST(StateMachineTest, InFlightBlockageIsBrakingWithABlockedReason)
{
	StateInputs in = healthy();
	in.limiting = true;
	in.blocked = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateBraking);
	EXPECT_EQ(out.reason_flags, kReasonBlocked);
}

TEST(StateMachineTest, PersistentBlockageStaysBrakingAndNeverEscalates)
{
	StateInputs in = healthy();
	in.limiting = true;
	in.blocked = true;
	StateResult out = evaluateState(in);

	// 100000 evaluations at 20 Hz is about 83 minutes: no timer, no Hold, no safe stop
	for (int i = 0; i < 100000; ++i) {
		out = step(in, out);
		ASSERT_EQ(static_cast<int>(out.state), static_cast<int>(kStateBraking)) << "evaluation " << i;
		ASSERT_FALSE(out.safe_stop_latched);
	}

	EXPECT_EQ(out.reason_flags, kReasonBlocked);
}

TEST(StateMachineTest, BlockageIsOnlyReportedWhileTheCapsAreUsable)
{
	StateInputs in = healthyOnGround();
	in.blocked = true;
	in.has_sample = false;
	EXPECT_EQ(evaluateState(in).reason_flags, kReasonSampleStale);  // no blocked reason on stale caps
}

// ---------------------------------------------------------------------------------------------
// In flight: SAFE_STOP entry (SYS-CP-004, SYS-CP-007, PERF-04)

TEST(StateMachineTest, InFlightEveryFaultEntersSafeStopAndRaisesItsReason)
{
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs in = healthy();
		fault.apply(in);
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateSafeStop);
		EXPECT_EQ(out.reason_flags, fault.reason | kReasonAckRequired);
		EXPECT_EQ(out.constraint_valid, fault.constraint_valid);
		EXPECT_TRUE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, InFlightSeveralFaultsReportEveryReason)
{
	StateInputs in = healthy();
	in.config_faults = 1U << 2;
	in.input_flags = kReasonFrameInvalid | kReasonDataInvalid;
	in.has_sample = false;
	in.heading_valid = false;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_EQ(out.reason_flags, kReasonInvalidConfig | kReasonFrameInvalid | kReasonDataInvalid
		  | kReasonSampleStale | kReasonNoHeading | kReasonAckRequired);
	EXPECT_FALSE(out.constraint_valid);
}

TEST(StateMachineTest, InputFlagsOtherThanFrameAndDataAreNotCopied)
{
	StateInputs in = healthy();
	in.input_flags = kReasonUnqualified | kReasonOverride | kReasonAckRequired;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_EQ(out.reason_flags, 0U);
}

TEST(StateMachineTest, TheSampleAgeLimitIsTheParameterNotAConstant)
{
	StateInputs in = healthy();
	in.sample_age_stop_s = 1.5f;
	in.sample_age_s = 1.0f;
	EXPECT_STATE(evaluateState(in), kStateActiveClear);
	in.sample_age_s = 1.5f;
	EXPECT_STATE(evaluateState(in), kStateSafeStop);
}

TEST(StateMachineTest, JustBelowTheAgeLimitIsStillFresh)
{
	StateInputs in = healthy();
	in.sample_age_s = std::nextafter(kAgeStop_s, 0.f);
	EXPECT_STATE(evaluateState(in), kStateActiveClear);
	in.sample_age_s = 0.f;
	EXPECT_STATE(evaluateState(in), kStateActiveClear);
}

TEST(StateMachineTest, AStoppedInputIsNoticedWhenItsAgeReachesTheLimit)
{
	// the input stops: the age grows 50 ms per periodic evaluation with no new sample
	StateInputs in = healthy();
	StateResult out = evaluateState(in);
	int first_safe_stop = -1;

	for (int i = 1; i <= 20; ++i) {
		in.sample_age_s = 0.05f * static_cast<float>(i);
		out = step(in, out);

		if (first_safe_stop < 0 && out.state == kStateSafeStop) {
			first_safe_stop = i;
		}
	}

	EXPECT_EQ(first_safe_stop, 10);  // 0.5 s
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_NE(out.reason_flags & kReasonSampleStale, 0U);
}

// ---------------------------------------------------------------------------------------------
// UNQUALIFIED is a reason flag, not a state (ADR 0007)

TEST(StateMachineTest, InFlightAnInvalidPx4StateIsNotReadyAndUnqualifiedAndNeverASafeStop)
{
	StateInputs in = healthy();
	in.px4_state_valid = false;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateNotReady);
	EXPECT_EQ(out.reason_flags, kReasonUnqualified);
	EXPECT_FALSE(out.safe_stop_latched);

	// the other faults are still reported, but CP adds no action of its own
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs faulty = in;
		fault.apply(faulty);
		out = evaluateState(faulty);
		EXPECT_STATE(out, kStateNotReady);
		EXPECT_EQ(out.reason_flags, fault.reason | kReasonUnqualified);
		EXPECT_FALSE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, RegainingAValidPx4StateReturnsToActiveClearWithoutAnAcknowledgement)
{
	StateInputs in = healthy();
	in.px4_state_valid = false;
	StateResult out = evaluateState(in);
	in.px4_state_valid = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
}

TEST(StateMachineTest, NoStateOutsideTheSixStatesIsEverProduced)
{
	for (const Fault &fault : kFaults) {
		for (const uint8_t takeoff_state : {kTakeoffReady, kTakeoffStateFlight}) {
			for (const bool px4 : {false, true}) {
				StateInputs in = healthy();
				fault.apply(in);
				in.takeoff_state = takeoff_state;
				in.px4_state_valid = px4;
				EXPECT_LE(evaluateState(in).state, kStateSafeStop);
			}
		}
	}
}

// ---------------------------------------------------------------------------------------------
// The time alignment of the map (issue #94, ADR 0019): a map whose alignment is not good is not
// used, so its sectors are unknown and the caps are 0. That is a reason and a readiness fault, not
// an immediate safe stop: the unused map does not refresh the sample, so after the sample age
// limit the ordinary stale rule enters SAFE_STOP.

TEST(StateMachineTest, ABadTimeAlignmentInFlightIsReportedAndIsNotASafeStopOnItsOwn)
{
	StateInputs in = healthy();
	in.time_sync_good = false;
	in.limiting = true;     // what the module gives for an unused map: every sector unknown, every cap 0
	in.blocked = true;

	const StateResult out = evaluateState(in);

	EXPECT_STATE(out, kStateBraking);
	EXPECT_EQ(out.reason_flags, kReasonTimeSync | kReasonBlocked);
	EXPECT_TRUE(out.constraint_valid);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, ABadTimeAlignmentBeforeFlightIsNotReady)
{
	for (const uint8_t takeoff_state : kNotFlightStates) {
		SCOPED_TRACE(static_cast<int>(takeoff_state));
		StateInputs in = healthy();
		in.takeoff_state = takeoff_state;
		in.time_sync_good = false;
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateNotReady);
		EXPECT_EQ(out.reason_flags, kReasonTimeSync);
		EXPECT_FALSE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, ABadTimeAlignmentAddsItsReasonToEveryOtherFault)
{
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs in = healthy();
		fault.apply(in);
		in.time_sync_good = false;
		const StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateSafeStop);
		EXPECT_EQ(out.reason_flags, fault.reason | kReasonTimeSync | kReasonAckRequired);
	}
}

TEST(StateMachineTest, ABadTimeAlignmentWithAnInvalidPx4StateIsNotReadyAndUnqualified)
{
	StateInputs in = healthy();
	in.px4_state_valid = false;
	in.time_sync_good = false;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateNotReady);
	EXPECT_EQ(out.reason_flags, kReasonTimeSync | kReasonUnqualified);
}

TEST(StateMachineTest, RegainingAGoodTimeAlignmentReturnsToActiveClearWithoutAnAcknowledgement)
{
	StateInputs in = healthy();
	in.time_sync_good = false;
	StateResult out = evaluateState(in);
	in.time_sync_good = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_EQ(out.reason_flags, 0U);
}

TEST(StateMachineTest, AnAcknowledgementDoesNotReleaseASafeStopWhileTheTimeAlignmentIsBad)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateSafeStop);

	in.has_sample = true;
	in.time_sync_good = false;
	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_TRUE(out.safe_stop_latched);
	EXPECT_NE(out.reason_flags & kReasonTimeSync, 0U);

	in.time_sync_good = true;
	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, TheTimeAlignmentIsStillReportedUnderAnOverride)
{
	StateInputs in = healthy();
	in.time_sync_good = false;
	in.pilot_override = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateObserverOnly);
	EXPECT_EQ(out.reason_flags, kReasonTimeSync | kReasonOverride);
}

TEST(StateMachineTest, DisabledReportsNoTimeAlignmentReason)
{
	StateInputs in = healthy();
	in.enabled = false;
	in.time_sync_good = false;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateDisabled);
	EXPECT_EQ(out.reason_flags, 0U);
}

TEST(StateMachineTest, TheTimeAlignmentComesOnlyFromItsOwnInput)
{
	StateInputs in = healthy();
	in.input_flags = kReasonTimeSync;   // not an input check result: it is not copied
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_EQ(out.reason_flags, 0U);
}

// ---------------------------------------------------------------------------------------------
// SAFE_STOP exits: acknowledgement and revalidation, or a logged override (architecture section 4)

TEST(StateMachineTest, SafeStopStaysAfterTheDataRecoversUntilTheAcknowledgement)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateSafeStop);

	in.has_sample = true;  // the data is valid again
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_EQ(out.reason_flags, kReasonAckRequired);
	EXPECT_TRUE(out.constraint_valid);
	EXPECT_TRUE(out.safe_stop_latched);
}

TEST(StateMachineTest, AcknowledgementWithARevalidationLeavesSafeStopForActiveClear)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);

	in.has_sample = true;
	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
	EXPECT_EQ(out.reason_flags, 0U);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, AcknowledgementWithARevalidationLeavesSafeStopForBrakingWhenACapLimits)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);

	in.has_sample = true;
	in.pilot_ack = true;
	in.limiting = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateBraking);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, AcknowledgementWhileAFaultPersistsKeepsSafeStop)
{
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs in = healthy();
		fault.apply(in);
		StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateSafeStop);

		in.pilot_ack = true;
		out = step(in, out);
		EXPECT_STATE(out, kStateSafeStop);
		EXPECT_EQ(out.reason_flags, fault.reason | kReasonAckRequired);
		EXPECT_TRUE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, AcknowledgementNeedsAValidPx4StateToo)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);

	in.has_sample = true;
	in.px4_state_valid = false;
	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);  // latched: CP keeps the safe stop, and reports it unqualified
	EXPECT_EQ(out.reason_flags, kReasonUnqualified | kReasonAckRequired);
	EXPECT_TRUE(out.safe_stop_latched);
}

TEST(StateMachineTest, AnAcknowledgementGivenWhileInvalidIsNotRemembered)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);

	in.pilot_ack = true;  // data still invalid: the acknowledgement is lost
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);

	in.pilot_ack = false;
	in.has_sample = true;  // data valid, but nobody acknowledged after the revalidation
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_TRUE(out.safe_stop_latched);
}

TEST(StateMachineTest, PersistentInvalidDataStaysSafeStopWhateverTheAcknowledgements)
{
	StateInputs in = healthy();
	in.input_flags = kReasonDataInvalid;
	StateResult out = evaluateState(in);

	for (int i = 0; i < 100000; ++i) {
		in.pilot_ack = (i % 2) == 0;
		out = step(in, out);
		ASSERT_EQ(static_cast<int>(out.state), static_cast<int>(kStateSafeStop)) << "evaluation " << i;
		ASSERT_TRUE(out.safe_stop_latched);
	}

	EXPECT_EQ(out.reason_flags, kReasonDataInvalid | kReasonAckRequired);
}

TEST(StateMachineTest, AcknowledgementWithoutALatchChangesNothing)
{
	StateInputs in = healthy();
	in.pilot_ack = true;
	EXPECT_STATE(evaluateState(in), kStateActiveClear);
	in.limiting = true;
	EXPECT_STATE(evaluateState(in), kStateBraking);
}

TEST(StateMachineTest, AfterTheAcknowledgementANewFaultEntersSafeStopAgain)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);
	in.has_sample = true;
	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);

	in.pilot_ack = false;
	in.input_flags = kReasonFrameInvalid;
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_TRUE(out.safe_stop_latched);
}

TEST(StateMachineTest, OverrideLeavesSafeStopForObserverOnlyWhateverTheData)
{
	for (const Fault &fault : kFaults) {
		SCOPED_TRACE(fault.name);
		StateInputs in = healthy();
		fault.apply(in);
		StateResult out = evaluateState(in);
		EXPECT_STATE(out, kStateSafeStop);

		in.pilot_override = true;
		out = step(in, out);
		EXPECT_STATE(out, kStateObserverOnly);
		EXPECT_EQ(out.reason_flags, fault.reason | kReasonOverride);
		EXPECT_FALSE(out.safe_stop_latched);
	}
}

TEST(StateMachineTest, OverrideWithHealthyDataIsObserverOnlyInFlightToo)
{
	StateInputs in = healthy();
	in.limiting = true;
	in.pilot_override = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateObserverOnly);
	EXPECT_EQ(out.reason_flags, kReasonOverride);
}

TEST(StateMachineTest, OverrideOverAnInvalidPx4StateIsStillObserverOnly)
{
	StateInputs in = healthy();
	in.px4_state_valid = false;
	in.pilot_override = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateObserverOnly);
	EXPECT_EQ(out.reason_flags, kReasonUnqualified | kReasonOverride);
}

TEST(StateMachineTest, WhenTheOverrideEndsAPersistentFaultEntersSafeStopAgain)
{
	StateInputs in = healthy();
	in.has_sample = false;
	in.pilot_override = true;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateObserverOnly);

	in.pilot_override = false;
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_TRUE(out.safe_stop_latched);
}

TEST(StateMachineTest, WhenTheOverrideEndsWithHealthyDataTheStateIsActiveClear)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);
	in.has_sample = true;
	in.pilot_override = true;
	out = step(in, out);
	in.pilot_override = false;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);  // the override released the latch, no acknowledgement is due
}

// ---------------------------------------------------------------------------------------------
// The latch across phases

TEST(StateMachineTest, TheLatchSurvivesLeavingFlightAndSafeStopIsNeverShownOnTheGround)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateSafeStop);

	in.has_sample = true;
	in.takeoff_state = kTakeoffReady;  // landed
	out = step(in, out);
	EXPECT_STATE(out, kStateNotReady);  // ADR 0009: nothing is enforced before flight
	EXPECT_EQ(out.reason_flags, kReasonAckRequired);
	EXPECT_TRUE(out.safe_stop_latched);

	in.takeoff_state = kTakeoffStateFlight;  // flying again with no acknowledgement
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
}

TEST(StateMachineTest, AnAcknowledgementOnTheGroundWithARevalidationClearsTheLatch)
{
	StateInputs in = healthyOnGround();
	in.safe_stop_latched = true;
	StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateNotReady);

	in.pilot_ack = true;
	out = step(in, out);
	EXPECT_STATE(out, kStateObserverOnly);
	EXPECT_FALSE(out.safe_stop_latched);

	in.pilot_ack = false;
	in.takeoff_state = kTakeoffStateFlight;
	out = step(in, out);
	EXPECT_STATE(out, kStateActiveClear);
}

TEST(StateMachineTest, AnOverrideOnTheGroundClearsTheLatch)
{
	StateInputs in = healthyOnGround();
	in.safe_stop_latched = true;
	in.pilot_override = true;
	const StateResult out = evaluateState(in);
	EXPECT_STATE(out, kStateObserverOnly);
	EXPECT_FALSE(out.safe_stop_latched);
}

TEST(StateMachineTest, ALatchedSafeStopWithAnInvalidPx4StateStaysLatchedInFlight)
{
	StateInputs in = healthy();
	in.has_sample = false;
	StateResult out = evaluateState(in);

	in.px4_state_valid = false;
	out = step(in, out);
	EXPECT_STATE(out, kStateSafeStop);
	EXPECT_NE(out.reason_flags & kReasonUnqualified, 0U);
	EXPECT_TRUE(out.safe_stop_latched);
}

// ---------------------------------------------------------------------------------------------
// Determinism (NFR-CP-001)

TEST(StateMachineTest, TheSameInputsGiveTheSameResult)
{
	for (const Fault &fault : kFaults) {
		StateInputs in = healthy();
		fault.apply(in);
		in.limiting = true;
		const StateResult a = evaluateState(in);
		const StateResult b = evaluateState(in);
		EXPECT_EQ(a.state, b.state);
		EXPECT_EQ(a.reason_flags, b.reason_flags);
		EXPECT_EQ(a.constraint_valid, b.constraint_valid);
		EXPECT_EQ(a.safe_stop_latched, b.safe_stop_latched);
	}
}
