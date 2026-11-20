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
 * @file TimeSyncTest.cpp
 *
 * Unit tests of the cp_brake time-alignment gate (ICD-CP-04, ADR 0019, issue #94). The limits are
 * test fixtures: the approved limits are OPEN-033, measured at G5.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "TimeSync.hpp"

using namespace cp_brake;

namespace
{
constexpr uint64_t kNow_us{50000000U};

// 0.25 s, 0.125 s and 0.5 s are exact in a float, so the boundary tests are exact
TimeSyncLimits fixtureLimits()
{
	return TimeSyncLimits{0.25f, 0.125f, 0.5f};
}

TimeSyncStatus goodStatus()
{
	TimeSyncStatus s{};
	s.timestamp_us = kNow_us - 100000U;       // 0.1 s old
	s.observed_offset_us = 1700000000000000LL;
	s.estimated_offset_us = 1700000000000000LL + 1000;
	s.round_trip_time_us = 2000U;
	return s;
}

#define EXPECT_VERDICT(actual, expected) \
	EXPECT_EQ(static_cast<int>(actual), static_cast<int>(expected))
} // namespace

TEST(TimeSyncGateTest, AGoodStatusPasses)
{
	EXPECT_VERDICT(judgeTimeSync(goodStatus(), kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);
}

TEST(TimeSyncGateTest, ATbdSentinelOrUnusableLimitMakesEveryStatusFail)
{
	const TimeSyncLimits bad[] = {
		{-1.f, 0.125f, 0.5f},     // the round trip sentinel
		{0.25f, -1.f, 0.5f},      // the offset sentinel
		{0.25f, 0.125f, -1.f},    // the status age sentinel
		{0.f, 0.125f, 0.5f},
		{0.25f, 0.f, 0.5f},
		{0.25f, 0.125f, 0.f},
		{std::numeric_limits<float>::quiet_NaN(), 0.125f, 0.5f},
		{0.25f, std::numeric_limits<float>::infinity(), 0.5f},
		{0.25f, 0.125f, std::numeric_limits<float>::quiet_NaN()},
	};

	for (const TimeSyncLimits &limits : bad) {
		EXPECT_FALSE(isValid(limits));
		EXPECT_VERDICT(judgeTimeSync(goodStatus(), kNow_us, limits), TimeSyncVerdict::kInvalidLimits);
	}

	EXPECT_TRUE(isValid(fixtureLimits()));
}

TEST(TimeSyncGateTest, NoStatusYetFails)
{
	TimeSyncStatus none{};   // timestamp 0: nothing was received

	EXPECT_VERDICT(judgeTimeSync(none, kNow_us, fixtureLimits()), TimeSyncVerdict::kNoStatus);
}

TEST(TimeSyncGateTest, AStatusThatStoppedArrivingIsStale)
{
	TimeSyncStatus s = goodStatus();

	s.timestamp_us = kNow_us - 500000U;   // exactly the limit: still good
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);

	s.timestamp_us = kNow_us - 500001U;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kStale);

	s.timestamp_us = 1U;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kStale);
}

TEST(TimeSyncGateTest, AStatusFromTheFutureIsStale)
{
	TimeSyncStatus s = goodStatus();
	s.timestamp_us = kNow_us + 1U;

	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kStale);
}

TEST(TimeSyncGateTest, AStatusFromTheFutureIsStaleEvenWithAHugeAgeLimit)
{
	// an age computed as now - timestamp would wrap to a huge number and pass a huge limit
	const TimeSyncLimits huge{0.25f, 0.125f, 1e30f};
	TimeSyncStatus s = goodStatus();
	s.timestamp_us = kNow_us + 1U;

	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, huge), TimeSyncVerdict::kStale);

	s.timestamp_us = std::numeric_limits<uint64_t>::max();
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, huge), TimeSyncVerdict::kStale);
}

TEST(TimeSyncGateTest, ALongRoundTripFails)
{
	TimeSyncStatus s = goodStatus();

	s.round_trip_time_us = 250000U;   // exactly the limit: still good
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);

	s.round_trip_time_us = 250001U;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kRoundTripTooLong);

	s.round_trip_time_us = std::numeric_limits<uint32_t>::max();
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kRoundTripTooLong);
}

TEST(TimeSyncGateTest, ASampleFarFromTheSmoothedOffsetFailsInEitherDirection)
{
	TimeSyncStatus s = goodStatus();

	s.observed_offset_us = s.estimated_offset_us + 125000;   // exactly the limit: still good
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);

	s.observed_offset_us = s.estimated_offset_us + 125001;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kOffsetDeviation);

	s.observed_offset_us = s.estimated_offset_us - 125000;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);

	s.observed_offset_us = s.estimated_offset_us - 125001;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kOffsetDeviation);
}

TEST(TimeSyncGateTest, ALargeOffsetDoesNotOverflowTheComparison)
{
	TimeSyncStatus s = goodStatus();

	s.observed_offset_us = std::numeric_limits<int64_t>::max();
	s.estimated_offset_us = std::numeric_limits<int64_t>::min();
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kOffsetDeviation);

	s.observed_offset_us = std::numeric_limits<int64_t>::min();
	s.estimated_offset_us = std::numeric_limits<int64_t>::max();
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kOffsetDeviation);

	// the same huge offset on both sides is a perfectly steady clock: the offset itself is not judged
	s.observed_offset_us = std::numeric_limits<int64_t>::max();
	s.estimated_offset_us = std::numeric_limits<int64_t>::max();
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kGood);
}

TEST(TimeSyncGateTest, AHugeLimitDoesNotWrapIntoASmallOne)
{
	TimeSyncLimits huge{1e30f, 1e30f, 1e30f};
	TimeSyncStatus s = goodStatus();
	s.round_trip_time_us = std::numeric_limits<uint32_t>::max();
	s.observed_offset_us = s.estimated_offset_us + 1000000000000LL;
	s.timestamp_us = 1U;

	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, huge), TimeSyncVerdict::kGood);
}

TEST(TimeSyncGateTest, WhenSeveralThingsAreWrongTheVerdictNamesTheFirstInAFixedOrder)
{
	TimeSyncStatus s = goodStatus();
	s.round_trip_time_us = 400000U;
	s.observed_offset_us = s.estimated_offset_us + 400000;
	s.timestamp_us = kNow_us - 900000U;

	// order: limits, no status, stale, round trip, offset
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kStale);

	s.timestamp_us = kNow_us - 100000U;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kRoundTripTooLong);

	s.round_trip_time_us = 2000U;
	EXPECT_VERDICT(judgeTimeSync(s, kNow_us, fixtureLimits()), TimeSyncVerdict::kOffsetDeviation);
}
