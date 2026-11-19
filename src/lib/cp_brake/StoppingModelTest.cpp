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
 * @file StoppingModelTest.cpp
 *
 * Unit tests of the cp_brake stopping model. Parameter sets here are test fixtures, not approved
 * platform values.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "StoppingModel.hpp"

using namespace cp_brake;

namespace
{
constexpr uint16_t kUnknown{std::numeric_limits<uint16_t>::max()};

// Test fixture values: 3 m/s^2 and 4 m/s^3 are the design baseline references of the architecture
// (PERF-02), t_lat and margin are arbitrary for the test.
StoppingParams fixture()
{
	return StoppingParams{3.f, 4.f, 0.1f, 2.f, 12.f};
}
} // namespace

TEST(StoppingModelTest, BrakingDistanceFromTwelveMetresPerSecondIsTwentyEightPointFive)
{
	EXPECT_NEAR(brakingDistance(12.f, 3.f, 4.f), 28.5f, 1e-3f);
}

TEST(StoppingModelTest, BrakingDistanceHasNoJumpWhereTheAccelerationLimitIsReached)
{
	const float a = 3.f;
	const float j = 4.f;
	const float v_edge = a * a / j; // 2.25 m/s
	EXPECT_NEAR(brakingDistance(v_edge - 1e-4f, a, j), brakingDistance(v_edge + 1e-4f, a, j), 1e-3f);
	EXPECT_NEAR(brakingDistance(1.f, a, j), 0.5f, 1e-5f); // v * sqrt(v / j) below the edge
}

TEST(StoppingModelTest, InvalidBrakingInputsGiveZero)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(brakingDistance(0.f, 3.f, 4.f), 0.f);
	EXPECT_EQ(brakingDistance(-1.f, 3.f, 4.f), 0.f);
	EXPECT_EQ(brakingDistance(nan, 3.f, 4.f), 0.f);
	EXPECT_EQ(brakingDistance(5.f, 0.f, 4.f), 0.f);
	EXPECT_EQ(brakingDistance(5.f, 3.f, nan), 0.f);
}

TEST(StoppingModelTest, StoppingDistanceIsStrictlyIncreasingInSpeed)
{
	const StoppingParams p = fixture();
	float previous = stoppingDistance(0.f, p);
	EXPECT_FLOAT_EQ(previous, p.margin_m);

	for (int i = 1; i <= 240; ++i) {
		const float distance = stoppingDistance(0.05f * static_cast<float>(i), p);
		EXPECT_GT(distance, previous);
		previous = distance;
	}
}

TEST(StoppingModelTest, CapIsNonDecreasingInRangeAndBounded)
{
	const StoppingParams p = fixture();
	float previous = 0.f;

	for (int i = 0; i <= 600; ++i) {
		const float cap = speedCap(0.1f * static_cast<float>(i), p);
		EXPECT_GE(cap, previous);
		EXPECT_GE(cap, 0.f);
		EXPECT_LE(cap, p.max_speed_m_s);
		previous = cap;
	}
}

TEST(StoppingModelTest, CapStopsWithinTheUsableRange)
{
	const StoppingParams p = fixture();

	for (float range = 2.1f; range < 60.f; range += 0.7f) {
		const float cap = speedCap(range, p);
		EXPECT_LE(stoppingDistance(cap, p), range + 1e-3f);

		if (cap < p.max_speed_m_s) {
			// the cap is the largest such speed: a little more speed no longer fits
			EXPECT_GT(stoppingDistance(cap + 0.01f, p), range - 1e-3f);
		}
	}
}

TEST(StoppingModelTest, NoCapUnderTheMarginAndFullCapWhenFar)
{
	const StoppingParams p = fixture();
	EXPECT_EQ(speedCap(p.margin_m, p), 0.f);
	EXPECT_EQ(speedCap(1.f, p), 0.f);
	EXPECT_EQ(speedCap(1000.f, p), p.max_speed_m_s);
}

TEST(StoppingModelTest, InvalidParametersOrRangeGiveZeroCap)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	StoppingParams p = fixture();
	EXPECT_TRUE(isValid(p));
	EXPECT_EQ(speedCap(nan, p), 0.f);

	p = fixture(); p.a_brake_m_s2 = 0.f;
	EXPECT_FALSE(isValid(p)); EXPECT_EQ(speedCap(50.f, p), 0.f);
	p = fixture(); p.jerk_m_s3 = -1.f;
	EXPECT_FALSE(isValid(p)); EXPECT_EQ(speedCap(50.f, p), 0.f);
	p = fixture(); p.t_lat_s = nan;
	EXPECT_FALSE(isValid(p)); EXPECT_EQ(speedCap(50.f, p), 0.f);
	p = fixture(); p.margin_m = -0.1f;
	EXPECT_FALSE(isValid(p)); EXPECT_EQ(speedCap(50.f, p), 0.f);
	p = fixture(); p.max_speed_m_s = 0.f;
	EXPECT_FALSE(isValid(p)); EXPECT_EQ(speedCap(50.f, p), 0.f);
}

TEST(StoppingModelTest, SectorCapFollowsTheWireValue)
{
	const StoppingParams p = fixture();
	const uint16_t min_cm = 10;
	const uint16_t max_cm = 4000;

	EXPECT_EQ(sectorSpeedCap(0, min_cm, max_cm, 0.f, p), 0.f);        // too close
	EXPECT_EQ(sectorSpeedCap(kUnknown, min_cm, max_cm, 0.f, p), 0.f); // unknown
	EXPECT_EQ(sectorSpeedCap(5, min_cm, max_cm, 0.f, p), 0.f);        // not a value of the interface
	EXPECT_EQ(sectorSpeedCap(4002, min_cm, max_cm, 0.f, p), 0.f);     // not a value of the interface

	EXPECT_FLOAT_EQ(sectorSpeedCap(1234, min_cm, max_cm, 0.f, p), speedCap(12.34f, p));
	EXPECT_FLOAT_EQ(sectorSpeedCap(4001, min_cm, max_cm, 0.f, p), speedCap(40.f, p)); // clear
}

TEST(StoppingModelTest, ASensorMinimumOfZeroStillGivesCapsForValidReadings)
{
	const StoppingParams p = fixture();

	EXPECT_EQ(sectorSpeedCap(0, 0, 4000, 0.f, p), 0.f);        // too close keeps its meaning
	EXPECT_EQ(sectorSpeedCap(kUnknown, 0, 4000, 0.f, p), 0.f); // unknown keeps its meaning
	EXPECT_FLOAT_EQ(sectorSpeedCap(1234, 0, 4000, 0.f, p), speedCap(12.34f, p));
	EXPECT_FLOAT_EQ(sectorSpeedCap(4001, 0, 4000, 0.f, p), speedCap(40.f, p));
	EXPECT_EQ(sectorSpeedCap(5, 4000, 4000, 0.f, p), 0.f);     // min not below max stays invalid
}

TEST(StoppingModelTest, TravelledDistanceShortensTheRangeButMovingAwayDoesNot)
{
	const StoppingParams p = fixture();

	EXPECT_LT(sectorSpeedCap(1500, 10, 4000, 5.f, p), sectorSpeedCap(1500, 10, 4000, 0.f, p));
	EXPECT_FLOAT_EQ(sectorSpeedCap(1500, 10, 4000, -3.f, p), sectorSpeedCap(1500, 10, 4000, 0.f, p));
	EXPECT_FLOAT_EQ(sectorSpeedCap(1500, 10, 4000, std::numeric_limits<float>::quiet_NaN(), p),
			sectorSpeedCap(1500, 10, 4000, 0.f, p));
}
