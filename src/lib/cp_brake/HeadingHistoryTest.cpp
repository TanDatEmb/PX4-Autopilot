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
 * @file HeadingHistoryTest.cpp
 *
 * Unit tests of the cp_brake heading history (ICD-CP-04, ADR 0019, issue #94). The depth, the gap
 * limit and every time and heading here are test fixtures, not approved values (OPEN-032).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "HeadingHistory.hpp"

using namespace cp_brake;

namespace
{
constexpr double kPi{3.14159265358979323846};
constexpr uint64_t kStep_us{8000U};     // the 125 Hz of vehicle_local_position in Gazebo SITL
constexpr uint64_t kT0_us{1000000U};

HeadingHistoryParams fixtureParams()
{
	return HeadingHistoryParams{16, 0.1f};
}

// the shortest signed angle from b to a, in [-pi, pi]
double angleDiff(double a, double b)
{
	double d = std::fmod(a - b, 2.0 * kPi);

	if (d > kPi) { d -= 2.0 * kPi; }

	if (d < -kPi) { d += 2.0 * kPi; }

	return d;
}

// n samples, kStep_us apart from kT0_us, constant heading, reset counter 0, good
HeadingHistory filled(std::size_t n, float heading_rad = 0.f)
{
	HeadingHistory history(fixtureParams());

	for (std::size_t i = 0; i < n; ++i) {
		EXPECT_TRUE(history.add(kT0_us + i * kStep_us, heading_rad, 0U, true));
	}

	return history;
}

#define EXPECT_LOOKUP(result, expected) \
	EXPECT_EQ(static_cast<int>((result).lookup), static_cast<int>(expected))
} // namespace

TEST(HeadingHistoryTest, AnEmptyHistoryHasNoHeading)
{
	const HeadingHistory history(fixtureParams());
	const HeadingResult r = history.at(kT0_us);

	EXPECT_FALSE(r.valid());
	EXPECT_LOOKUP(r, HeadingLookup::kEmpty);
	EXPECT_EQ(history.size(), 0U);
}

TEST(HeadingHistoryTest, ATimeBeforeTheFirstSampleHasNoHeading)
{
	const HeadingHistory history = filled(4);

	EXPECT_LOOKUP(history.at(kT0_us - 1U), HeadingLookup::kBeforeFirst);
	EXPECT_LOOKUP(history.at(0U), HeadingLookup::kBeforeFirst);
	EXPECT_FALSE(history.at(kT0_us - 1U).valid());
}

TEST(HeadingHistoryTest, ATimeAfterTheLastSampleHasNoHeadingAndIsNotExtrapolated)
{
	const HeadingHistory history = filled(4);
	const uint64_t last_us = kT0_us + 3U * kStep_us;

	EXPECT_LOOKUP(history.at(last_us + 1U), HeadingLookup::kAfterLast);
	EXPECT_FALSE(history.at(last_us + 1U).valid());
}

TEST(HeadingHistoryTest, TheExactTimeOfTheFirstAndTheLastSampleGivesTheirHeading)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, 0.25f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 0.75f, 0U, true));

	const HeadingResult first = history.at(kT0_us);
	const HeadingResult last = history.at(kT0_us + kStep_us);

	ASSERT_TRUE(first.valid());
	ASSERT_TRUE(last.valid());
	EXPECT_FLOAT_EQ(first.heading_rad, 0.25f);
	EXPECT_FLOAT_EQ(last.heading_rad, 0.75f);
}

TEST(HeadingHistoryTest, TheMidpointIsTheMeanOfTheTwoSamples)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, 0.2f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 0.6f, 0U, true));

	const HeadingResult r = history.at(kT0_us + kStep_us / 2U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(r.heading_rad, 0.4f, 1e-6f);
}

TEST(HeadingHistoryTest, AQuarterOfTheWayWeighsTheEarlierSampleThreeToOne)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, -0.4f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 0.4f, 0U, true));

	const HeadingResult r = history.at(kT0_us + kStep_us / 4U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(r.heading_rad, -0.2f, 1e-6f);
}

TEST(HeadingHistoryTest, TheTwoSamplesAroundTheTimeAreUsedWhateverTheDepth)
{
	// a ramp of 0.01 rad per sample: the value at any time is the ramp at that time
	HeadingHistory history(fixtureParams());

	for (std::size_t i = 0; i < 10; ++i) {
		ASSERT_TRUE(history.add(kT0_us + i * kStep_us, 0.01f * static_cast<float>(i), 0U, true));
	}

	const HeadingResult r = history.at(kT0_us + 6U * kStep_us + kStep_us / 2U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(r.heading_rad, 0.065f, 1e-6f);
}

TEST(HeadingHistoryTest, WrapAroundAtPlusMinus180DegreesTakesTheShortArc)
{
	// 3.10 rad and -3.10 rad are 0.0832 rad apart across the +-pi cut, not 6.2 rad apart
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, 3.10f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, -3.10f, 0U, true));

	const HeadingResult quarter = history.at(kT0_us + kStep_us / 4U);
	const HeadingResult mid = history.at(kT0_us + kStep_us / 2U);

	ASSERT_TRUE(quarter.valid());
	ASSERT_TRUE(mid.valid());

	// the mean taken without the wrap would be 0: it must be near +-pi
	EXPECT_GT(std::fabs(mid.heading_rad), 3.0f);
	EXPECT_NEAR(std::fabs(angleDiff(mid.heading_rad, kPi)), 0.0, 1e-5);

	const double expected_quarter = 3.10 + 0.25 * (2.0 * kPi - 6.20);
	EXPECT_NEAR(angleDiff(quarter.heading_rad, expected_quarter), 0.0, 1e-5);
	EXPECT_LE(quarter.heading_rad, static_cast<float>(kPi));
	EXPECT_GT(quarter.heading_rad, -static_cast<float>(kPi));
}

TEST(HeadingHistoryTest, WrapAroundTakesTheShortArcInTheOtherDirectionToo)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, -3.10f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 3.10f, 0U, true));

	const HeadingResult r = history.at(kT0_us + kStep_us / 4U);

	ASSERT_TRUE(r.valid());
	// the short arc from -3.10 to 3.10 is -0.0832 rad (through -pi), a quarter of it gives -3.1208
	const double expected = -3.10 - 0.25 * (2.0 * kPi - 6.20);
	EXPECT_NEAR(angleDiff(r.heading_rad, expected), 0.0, 1e-5);
	EXPECT_GT(std::fabs(r.heading_rad), 3.0f);
	EXPECT_LE(r.heading_rad, static_cast<float>(kPi));
	EXPECT_GT(r.heading_rad, -static_cast<float>(kPi));
}

TEST(HeadingHistoryTest, AnInterpolatedHeadingThatPassesTheCutIsWrappedBackIntoRange)
{
	// 3.13 rad to -3.13 rad is a short arc of +0.0232 rad: three quarters of the way is 3.1474 rad,
	// which is past pi and has to come back as -3.1358 rad
	HeadingHistory up(fixtureParams());
	ASSERT_TRUE(up.add(kT0_us, 3.13f, 0U, true));
	ASSERT_TRUE(up.add(kT0_us + kStep_us, -3.13f, 0U, true));

	const HeadingResult r_up = up.at(kT0_us + 3U * kStep_us / 4U);
	ASSERT_TRUE(r_up.valid());
	EXPECT_NEAR(r_up.heading_rad, -3.1358f, 1e-4f);
	EXPECT_GT(r_up.heading_rad, -static_cast<float>(kPi));
	EXPECT_LE(r_up.heading_rad, static_cast<float>(kPi));

	// and the same turning the other way, past -pi
	HeadingHistory down(fixtureParams());
	ASSERT_TRUE(down.add(kT0_us, -3.13f, 0U, true));
	ASSERT_TRUE(down.add(kT0_us + kStep_us, 3.13f, 0U, true));

	const HeadingResult r_down = down.at(kT0_us + 3U * kStep_us / 4U);
	ASSERT_TRUE(r_down.valid());
	EXPECT_NEAR(r_down.heading_rad, 3.1358f, 1e-4f);
	EXPECT_GT(r_down.heading_rad, -static_cast<float>(kPi));
	EXPECT_LE(r_down.heading_rad, static_cast<float>(kPi));
}

TEST(HeadingHistoryTest, ASampleExactlyOnThePlusMinus180CutIsReturnedAsIs)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, static_cast<float>(kPi), 0U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, -static_cast<float>(kPi), 0U, true));

	const HeadingResult r = history.at(kT0_us + kStep_us / 2U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(std::fabs(r.heading_rad), static_cast<float>(kPi), 1e-5f);
}

TEST(HeadingHistoryTest, AResetBlocksInterpolationAcrossItAndOnlyAcrossIt)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us + 0U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 1U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 2U * kStep_us, 1.5f, 1U, true));   // the heading jumped: reset 0 -> 1
	ASSERT_TRUE(history.add(kT0_us + 3U * kStep_us, 1.5f, 1U, true));

	EXPECT_LOOKUP(history.at(kT0_us + 1U * kStep_us + kStep_us / 2U), HeadingLookup::kReset);
	EXPECT_FALSE(history.at(kT0_us + 1U * kStep_us + 1U).valid());

	// before the reset and after it the samples agree on the counter
	EXPECT_TRUE(history.at(kT0_us + kStep_us / 2U).valid());
	EXPECT_TRUE(history.at(kT0_us + 2U * kStep_us + kStep_us / 2U).valid());

	// the sample time of the first sample after the reset is known exactly
	const HeadingResult at_reset = history.at(kT0_us + 2U * kStep_us);
	ASSERT_TRUE(at_reset.valid());
	EXPECT_FLOAT_EQ(at_reset.heading_rad, 1.5f);
}

TEST(HeadingHistoryTest, AResetCounterThatWrapsAroundIsStillAChange)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, 0.f, 255U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 0.f, 0U, true));

	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kReset);
}

TEST(HeadingHistoryTest, AnyChangeOfTheResetCounterBlocksNotOnlyAnIncrease)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us, 0.f, 5U, true));
	ASSERT_TRUE(history.add(kT0_us + kStep_us, 0.f, 3U, true));

	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kReset);
}

TEST(HeadingHistoryTest, ASampleNotGoodForControlBlocksTheInterpolationAroundIt)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us + 0U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 1U * kStep_us, 0.1f, 0U, false));
	ASSERT_TRUE(history.add(kT0_us + 2U * kStep_us, 0.1f, 0U, true));

	// the dropout has recovered: the times around it are kNotYetGood (issue #112), never a heading
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us), HeadingLookup::kNotYetGood);
	EXPECT_TRUE(history.at(kT0_us).valid());
	EXPECT_TRUE(history.at(kT0_us + 2U * kStep_us).valid());
}

// Issue #112: the heading is not good for control until the estimator aligns it, some seconds after take-off,
// and a map is sampled about 0.1 s before it is evaluated.
TEST(HeadingHistoryTest, BeforeTheCurrentRunOfGoodSamplesTheLookupIsNotYetGoodNotNotGood)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us + 0U * kStep_us, 0.1f, 0U, false));
	ASSERT_TRUE(history.add(kT0_us + 1U * kStep_us, 0.1f, 0U, false));
	ASSERT_TRUE(history.add(kT0_us + 2U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 3U * kStep_us, 0.1f, 0U, true));

	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us + kStep_us / 2U), HeadingLookup::kNotYetGood);  // between not good and good
	EXPECT_TRUE(history.at(kT0_us + 2U * kStep_us).valid());
	EXPECT_TRUE(history.at(kT0_us + 2U * kStep_us + kStep_us / 2U).valid());
}

TEST(HeadingHistoryTest, ADropoutThatHasRecoveredIsNotYetGoodAndOneThatIsStillOnStaysNotGood)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us + 0U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 1U * kStep_us, 0.1f, 0U, false));
	ASSERT_TRUE(history.add(kT0_us + 2U * kStep_us, 0.1f, 0U, false));
	// the newest sample is not good: the heading is invalid now
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kNotGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us + kStep_us / 2U), HeadingLookup::kNotGood);

	ASSERT_TRUE(history.add(kT0_us + 3U * kStep_us, 0.1f, 0U, true));
	// it recovered: the maps sampled during the dropout are kNotYetGood now
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + kStep_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_LOOKUP(history.at(kT0_us + 2U * kStep_us + kStep_us / 2U), HeadingLookup::kNotYetGood);
	EXPECT_TRUE(history.at(kT0_us).valid());
}

TEST(HeadingHistoryTest, AGapAndAResetAreStillFailuresInsideTheRunOfGoodSamples)
{
	HeadingHistory gap(fixtureParams());
	ASSERT_TRUE(gap.add(kT0_us, 0.f, 0U, true));
	ASSERT_TRUE(gap.add(kT0_us + 100U * kStep_us, 0.f, 0U, true));
	EXPECT_LOOKUP(gap.at(kT0_us + 50U * kStep_us), HeadingLookup::kGap);

	HeadingHistory reset(fixtureParams());
	ASSERT_TRUE(reset.add(kT0_us, 0.f, 5U, true));
	ASSERT_TRUE(reset.add(kT0_us + kStep_us, 0.f, 6U, true));
	EXPECT_LOOKUP(reset.at(kT0_us + kStep_us / 2U), HeadingLookup::kReset);
}

// Review of the fork PR #10: with a small depth the sample of a delayed map is evicted for good. That is a
// failure and must stay a safe stop: it is kBeforeFirst, never kNotYetGood.
TEST(HeadingHistoryTest, AnEvictedSampleStaysBeforeFirstAndIsNeverNotYetGood)
{
	HeadingHistory history(HeadingHistoryParams{2, 0.1f});
	// heading samples 8 ms apart, all good; a map sampled 100 ms before the newest sample
	for (uint64_t i = 0U; i < 40U; ++i) {
		ASSERT_TRUE(history.add(kT0_us + i * 8'000U, 0.1f, 0U, true));
	}

	const HeadingResult late = history.at(kT0_us + 39U * 8'000U - 100'000U);
	EXPECT_EQ(late.lookup, HeadingLookup::kBeforeFirst);
}

TEST(HeadingHistoryTest, ANonFiniteHeadingIsNeverUsed)
{
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(kT0_us + 0U * kStep_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 1U * kStep_us, std::numeric_limits<float>::quiet_NaN(), 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 2U * kStep_us, std::numeric_limits<float>::infinity(), 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 3U * kStep_us, 0.1f, 0U, true));

	EXPECT_FALSE(history.at(kT0_us + kStep_us / 2U).valid());
	EXPECT_FALSE(history.at(kT0_us + kStep_us).valid());
	EXPECT_FALSE(history.at(kT0_us + 2U * kStep_us).valid());
	EXPECT_FALSE(history.at(kT0_us + 2U * kStep_us + kStep_us / 2U).valid());
}

TEST(HeadingHistoryTest, AGapLongerThanTheLimitBlocksTheInterpolationAcrossIt)
{
	HeadingHistory history(fixtureParams());   // gap limit 0.1 s = 100000 us
	ASSERT_TRUE(history.add(kT0_us, 0.1f, 0U, true));
	ASSERT_TRUE(history.add(kT0_us + 100000U, 0.2f, 0U, true));   // exactly the limit
	ASSERT_TRUE(history.add(kT0_us + 200001U, 0.3f, 0U, true));   // one us over it

	EXPECT_TRUE(history.at(kT0_us + 50000U).valid());
	EXPECT_LOOKUP(history.at(kT0_us + 150000U), HeadingLookup::kGap);

	// the samples themselves are still known
	EXPECT_TRUE(history.at(kT0_us + 200001U).valid());
}

TEST(HeadingHistoryTest, TheDepthLimitsHowFarBackTheHistoryGoes)
{
	HeadingHistory history(HeadingHistoryParams{4, 0.1f});

	for (std::size_t i = 0; i < 10; ++i) {
		ASSERT_TRUE(history.add(kT0_us + i * kStep_us, 0.f, 0U, true));
	}

	EXPECT_EQ(history.size(), 4U);
	EXPECT_LOOKUP(history.at(kT0_us + 5U * kStep_us + kStep_us / 2U), HeadingLookup::kBeforeFirst);
	EXPECT_LOOKUP(history.at(kT0_us + 6U * kStep_us - 1U), HeadingLookup::kBeforeFirst);
	EXPECT_TRUE(history.at(kT0_us + 6U * kStep_us).valid());
	EXPECT_TRUE(history.at(kT0_us + 9U * kStep_us).valid());
	EXPECT_LOOKUP(history.at(kT0_us + 9U * kStep_us + 1U), HeadingLookup::kAfterLast);
}

TEST(HeadingHistoryTest, TheRingKeepsTheOrderAfterManyMoreSamplesThanItsDepth)
{
	const int32_t depth = static_cast<int32_t>(kHeadingHistoryCapacity);
	HeadingHistory history(HeadingHistoryParams{depth, 0.1f});
	const std::size_t total = 3U * kHeadingHistoryCapacity + 5U;

	for (std::size_t i = 0; i < total; ++i) {
		ASSERT_TRUE(history.add(kT0_us + i * kStep_us, 0.001f * static_cast<float>(i % 1000U), 0U, true));
	}

	EXPECT_EQ(history.size(), kHeadingHistoryCapacity);

	const std::size_t newest = total - 1U;
	const std::size_t oldest = total - kHeadingHistoryCapacity;
	const HeadingResult mid = history.at(kT0_us + (newest - 1U) * kStep_us + kStep_us / 2U);

	ASSERT_TRUE(mid.valid());
	EXPECT_NEAR(mid.heading_rad, 0.001f * (static_cast<float>((newest - 1U) % 1000U) + 0.5f), 1e-5f);
	EXPECT_TRUE(history.at(kT0_us + oldest * kStep_us).valid());
	EXPECT_LOOKUP(history.at(kT0_us + oldest * kStep_us - 1U), HeadingLookup::kBeforeFirst);
}

TEST(HeadingHistoryTest, ALargeOffsetFromTheHistoryHasNoHeading)
{
	const HeadingHistory history = filled(8);
	const uint64_t last_us = kT0_us + 7U * kStep_us;

	EXPECT_LOOKUP(history.at(last_us + 3600ULL * 1000000ULL), HeadingLookup::kAfterLast);
	EXPECT_LOOKUP(history.at(std::numeric_limits<uint64_t>::max()), HeadingLookup::kAfterLast);
	EXPECT_LOOKUP(history.at(1U), HeadingLookup::kBeforeFirst);
}

TEST(HeadingHistoryTest, LargeAbsoluteTimesDoNotCostPrecision)
{
	// a clock set to UTC counts about 1.7e15 us: a float cannot tell two times 4 ms apart there
	constexpr uint64_t base_us = 1763600000000000ULL;
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(base_us, 0.2f, 0U, true));
	ASSERT_TRUE(history.add(base_us + kStep_us, 0.6f, 0U, true));

	const HeadingResult r = history.at(base_us + kStep_us / 2U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(r.heading_rad, 0.4f, 1e-6f);
}

TEST(HeadingHistoryTest, TimesNearTheTopOfTheRangeDoNotOverflow)
{
	constexpr uint64_t top_us = std::numeric_limits<uint64_t>::max();
	HeadingHistory history(fixtureParams());
	ASSERT_TRUE(history.add(top_us - 2U * kStep_us, 0.2f, 0U, true));
	ASSERT_TRUE(history.add(top_us - kStep_us, 0.6f, 0U, true));

	const HeadingResult r = history.at(top_us - kStep_us - kStep_us / 2U);

	ASSERT_TRUE(r.valid());
	EXPECT_NEAR(r.heading_rad, 0.4f, 1e-6f);
	EXPECT_LOOKUP(history.at(top_us), HeadingLookup::kAfterLast);
}

TEST(HeadingHistoryTest, ATimeThatDoesNotAdvanceIsRejected)
{
	HeadingHistory history(fixtureParams());

	EXPECT_FALSE(history.add(0U, 0.f, 0U, true));          // 0 means no time
	EXPECT_TRUE(history.add(kT0_us, 0.3f, 0U, true));
	EXPECT_FALSE(history.add(kT0_us, 0.9f, 0U, true));     // the same time again
	EXPECT_FALSE(history.add(kT0_us - 1U, 0.9f, 0U, true)); // an earlier time
	EXPECT_EQ(history.size(), 1U);

	const HeadingResult r = history.at(kT0_us);
	ASSERT_TRUE(r.valid());
	EXPECT_FLOAT_EQ(r.heading_rad, 0.3f);
}

TEST(HeadingHistoryTest, ATbdSentinelOrOutOfRangeParameterGivesNoHeadingAtAll)
{
	const HeadingHistoryParams bad[] = {
		{-1, 0.1f},                                                    // the depth sentinel
		{0, 0.1f},
		{1, 0.1f},                                                     // interpolation needs two samples
		{static_cast<int32_t>(kHeadingHistoryCapacity) + 1, 0.1f},     // more than the storage holds
		{8, -1.f},                                                     // the gap sentinel
		{8, 0.f},
		{8, std::numeric_limits<float>::quiet_NaN()},
		{8, std::numeric_limits<float>::infinity()},
	};

	for (const HeadingHistoryParams &params : bad) {
		EXPECT_FALSE(isValid(params)) << params.depth << " " << params.max_gap_s;

		HeadingHistory history(params);
		EXPECT_FALSE(history.add(kT0_us, 0.f, 0U, true));
		EXPECT_FALSE(history.add(kT0_us + kStep_us, 0.f, 0U, true));
		EXPECT_LOOKUP(history.at(kT0_us), HeadingLookup::kInvalidParams);
		EXPECT_EQ(history.size(), 0U);
	}

	EXPECT_TRUE(isValid(HeadingHistoryParams{2, 0.1f}));
	EXPECT_TRUE(isValid(HeadingHistoryParams{static_cast<int32_t>(kHeadingHistoryCapacity), 0.1f}));
}

TEST(HeadingHistoryTest, ResetEmptiesTheHistoryAndAppliesTheNewParameters)
{
	HeadingHistory history = filled(10);
	ASSERT_EQ(history.size(), 10U);

	history.reset(HeadingHistoryParams{4, 0.05f});

	EXPECT_EQ(history.size(), 0U);
	EXPECT_EQ(history.params().depth, 4);
	EXPECT_FLOAT_EQ(history.params().max_gap_s, 0.05f);
	EXPECT_LOOKUP(history.at(kT0_us), HeadingLookup::kEmpty);

	// the new depth is in force, and an old sample is not remembered
	for (std::size_t i = 0; i < 6; ++i) {
		ASSERT_TRUE(history.add(kT0_us + i * kStep_us, 0.f, 0U, true));
	}

	EXPECT_EQ(history.size(), 4U);
	EXPECT_LOOKUP(history.at(kT0_us), HeadingLookup::kBeforeFirst);
}

TEST(HeadingHistoryTest, ResetToInvalidParametersGivesNoHeading)
{
	HeadingHistory history = filled(10);

	history.reset(HeadingHistoryParams{-1, -1.f});

	EXPECT_EQ(history.size(), 0U);
	EXPECT_FALSE(history.add(kT0_us + 20U * kStep_us, 0.f, 0U, true));
	EXPECT_LOOKUP(history.at(kT0_us), HeadingLookup::kInvalidParams);
}

TEST(HeadingHistoryTest, ParametersAreTheSameOnlyWhenBothAreUsableAndEqual)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();

	EXPECT_TRUE(sameParams(HeadingHistoryParams{64, 0.1f}, HeadingHistoryParams{64, 0.1f}));
	EXPECT_TRUE(sameParams(HeadingHistoryParams{-1, -1.f}, HeadingHistoryParams{-1, -1.f}));   // the sentinels

	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, 0.1f}, HeadingHistoryParams{32, 0.1f}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, 0.1f}, HeadingHistoryParams{64, 0.2f}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, 0.2f}, HeadingHistoryParams{64, 0.1f}));

	// the review: a gap that was NaN and is corrected must count as a change, in either direction
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, nan}, HeadingHistoryParams{64, 0.1f}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, 0.1f}, HeadingHistoryParams{64, nan}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, nan}, HeadingHistoryParams{64, nan}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, inf}, HeadingHistoryParams{64, 0.1f}));
	EXPECT_FALSE(sameParams(HeadingHistoryParams{64, 0.1f}, HeadingHistoryParams{64, inf}));
}

TEST(HeadingHistoryTest, AParameterThatWasInvalidAndIsCorrectedStartsTheHistoryAgain)
{
	// what the module does on a parameter update: reset unless the parameters are the same
	const float nan = std::numeric_limits<float>::quiet_NaN();
	HeadingHistory history(HeadingHistoryParams{16, nan});
	const HeadingHistoryParams corrected{16, 0.1f};

	EXPECT_FALSE(history.add(kT0_us, 0.f, 0U, true));
	EXPECT_LOOKUP(history.at(kT0_us), HeadingLookup::kInvalidParams);

	if (!sameParams(history.params(), corrected)) {
		history.reset(corrected);
	}

	EXPECT_TRUE(history.add(kT0_us, 0.f, 0U, true));
	EXPECT_TRUE(history.add(kT0_us + kStep_us, 0.f, 0U, true));
	EXPECT_TRUE(history.at(kT0_us + kStep_us / 2U).valid());
}

TEST(HeadingHistoryTest, AnInvalidResultCarriesNoHeadingValue)
{
	const HeadingHistory history = filled(4, 1.f);

	EXPECT_FLOAT_EQ(history.at(kT0_us - 1U).heading_rad, 0.f);
	EXPECT_FLOAT_EQ(history.at(kT0_us + 10U * kStep_us).heading_rad, 0.f);
}
