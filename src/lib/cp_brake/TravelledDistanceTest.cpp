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
 * @file TravelledDistanceTest.cpp
 *
 * Unit tests of the distance moved since the sample and of the raw caps of a map (ADR 0020 D4,
 * issue #115). Velocities, ages, bounds and stopping parameters are test fixtures, not approved
 * values.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "HeadingHistory.hpp"
#include "StoppingModel.hpp"
#include "TravelledDistance.hpp"

using namespace cp_brake;

namespace
{
constexpr double kPi{3.14159265358979323846};
constexpr float kNaN{std::numeric_limits<float>::quiet_NaN()};
constexpr float kInf{std::numeric_limits<float>::infinity()};
constexpr uint16_t kUnknown{UINT16_MAX};
constexpr uint16_t kMinCm{10};
constexpr uint16_t kMaxCm{4000};
constexpr uint16_t kClearCm{4001};
constexpr float kBound{3.f};

StoppingParams fixtureStopping()
{
	return StoppingParams{3.f, 4.f, 0.1f, 2.f, 12.f};
}

TravelInputs inputs(float north, float east, float age_s, float heading_rad = 0.f)
{
	return TravelInputs{true, north, east, true, heading_rad, age_s, kBound};
}

float allowance(float age_s)
{
	return 0.5f * kBound * age_s * age_s;
}

struct Map {
	uint16_t wire[kSectorCount];

	explicit Map(uint16_t value)
	{
		for (std::size_t i = 0; i < kSectorCount; ++i) {
			wire[i] = value;
		}
	}
};
} // namespace

TEST(TravelledDistanceTest, MovingTowardTheSectorAddsTheSpeedTimesTheAgeAndTheAllowance)
{
	const float age = 0.4f;
	EXPECT_NEAR(travelledTowardSector(inputs(5.f, 0.f, age), 0), 5.f * age + allowance(age), 1e-5f);
	EXPECT_NEAR(travelledTowardSector(inputs(5.f, 0.f, age), 0), 2.24f, 1e-5f);
}

TEST(TravelledDistanceTest, OnlyTheComponentTowardTheSectorCounts)
{
	// v = (3, 4): toward north 3 m/s, toward east 4 m/s, toward the bearing of 90 degrees (sector 18)
	const float age = 0.5f;
	EXPECT_NEAR(travelledTowardSector(inputs(3.f, 4.f, age), 0), 3.f * age + allowance(age), 1e-5f);
	EXPECT_NEAR(travelledTowardSector(inputs(3.f, 4.f, age), 18), 4.f * age + allowance(age), 1e-5f);
	// sector 9 is at 45 degrees: (3 + 4) / sqrt(2)
	EXPECT_NEAR(travelledTowardSector(inputs(3.f, 4.f, age), 9), 7.f / std::sqrt(2.f) * age + allowance(age), 1e-4f);
}

TEST(TravelledDistanceTest, MovingAwayGivesOnlyTheAllowance)
{
	// the sector behind the velocity: max(0, negative) is 0, never a negative distance
	const float age = 0.4f;
	EXPECT_NEAR(travelledTowardSector(inputs(-5.f, 0.f, age), 0), allowance(age), 1e-6f);
	EXPECT_NEAR(travelledTowardSector(inputs(5.f, 0.f, age), 36), allowance(age), 1e-6f);
	EXPECT_NEAR(travelledTowardSector(inputs(5.f, 0.f, age), 18), allowance(age), 1e-5f); // perpendicular
}

TEST(TravelledDistanceTest, AStationaryVehicleGivesOnlyTheAllowance)
{
	const float age = 0.5f;

	for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
		EXPECT_NEAR(travelledTowardSector(inputs(0.f, 0.f, age), sector), allowance(age), 1e-6f);
	}
}

TEST(TravelledDistanceTest, ZeroAgeGivesZeroWhateverTheVelocity)
{
	EXPECT_NEAR(travelledTowardSector(inputs(12.f, 0.f, 0.f), 0), 0.f, 1e-9f);
	EXPECT_NEAR(travelledTowardSector(inputs(-12.f, 3.f, 0.f), 5), 0.f, 1e-9f);
	EXPECT_TRUE(isUsable(inputs(12.f, 0.f, 0.f)));
}

TEST(TravelledDistanceTest, ALargeAgeGivesALargeDistance)
{
	const float age = 20.f;
	EXPECT_NEAR(travelledTowardSector(inputs(4.f, 0.f, age), 0), 4.f * age + allowance(age), 1e-3f);
	EXPECT_TRUE(isUsable(inputs(4.f, 0.f, age))); // the stale rule is the state machine's, not this function's
}

TEST(TravelledDistanceTest, TheSectorsAreRotatedByTheHeadingOfTheSample)
{
	// heading 90 degrees: the vehicle flies east, which is forward (sector 0) at that heading
	const float east = static_cast<float>(kPi / 2.0);
	const float age = 0.4f;
	EXPECT_NEAR(travelledTowardSector(inputs(0.f, 5.f, age, east), 0), 5.f * age + allowance(age), 1e-5f);
	// the same velocity at heading 0 is toward sector 18 (right), not toward forward
	EXPECT_NEAR(travelledTowardSector(inputs(0.f, 5.f, age, 0.f), 0), allowance(age), 1e-5f);
	EXPECT_NEAR(travelledTowardSector(inputs(0.f, 5.f, age, 0.f), 18), 5.f * age + allowance(age), 1e-5f);
	// heading -90 degrees: north is to the right of the vehicle, sector 18
	EXPECT_NEAR(travelledTowardSector(inputs(5.f, 0.f, age, -east), 18), 5.f * age + allowance(age), 1e-5f);
}

TEST(TravelledDistanceTest, ASectorThatDoesNotExistIsNotAnAnswer)
{
	EXPECT_TRUE(std::isnan(travelledTowardSector(inputs(1.f, 0.f, 0.1f), kSectorCount)));
}

TEST(TravelledDistanceTest, AnInvalidOrNonFiniteVelocityIsNotUsable)
{
	TravelInputs in = inputs(5.f, 0.f, 0.4f);
	in.velocity_valid = false;
	EXPECT_FALSE(isUsable(in));
	EXPECT_TRUE(std::isnan(travelledTowardSector(in, 0)));

	EXPECT_FALSE(isUsable(inputs(kNaN, 0.f, 0.4f)));
	EXPECT_FALSE(isUsable(inputs(0.f, kNaN, 0.4f)));
	EXPECT_FALSE(isUsable(inputs(kInf, 0.f, 0.4f)));
	EXPECT_FALSE(isUsable(inputs(0.f, -kInf, 0.4f)));
}

TEST(TravelledDistanceTest, ANoHeadingOrABadHeadingIsNotUsable)
{
	TravelInputs in = inputs(5.f, 0.f, 0.4f);
	in.heading_valid = false;
	EXPECT_FALSE(isUsable(in));

	EXPECT_FALSE(isUsable(inputs(5.f, 0.f, 0.4f, kNaN)));
	EXPECT_FALSE(isUsable(inputs(5.f, 0.f, 0.4f, kInf)));
}

TEST(TravelledDistanceTest, AnAgeThatIsNotANonNegativeNumberIsNotUsable)
{
	EXPECT_FALSE(isUsable(inputs(5.f, 0.f, -0.001f)));  // a map stamped after now
	EXPECT_FALSE(isUsable(inputs(5.f, 0.f, kNaN)));
	EXPECT_FALSE(isUsable(inputs(5.f, 0.f, kInf)));
}

TEST(TravelledDistanceTest, ABoundThatIsNotAPositiveNumberIsNotUsable)
{
	// the sentinel of CPB_DECEL_MAX, a zero and non-finite values (ConfigCheck also refuses them)
	for (const float bound : {-1.f, 0.f, kNaN, kInf}) {
		TravelInputs in = inputs(5.f, 0.f, 0.4f);
		in.decel_bound_m_s2 = bound;
		EXPECT_FALSE(isUsable(in)) << bound;
	}
}

TEST(RawSectorCapsTest, AMovingVehicleHasLessUsableRangeThanAStationaryOneAtAgeZero)
{
	Map map(1000U); // 10 m in every sector
	SectorCaps moving{};
	SectorCaps at_rest{};
	SectorFlags valid{};

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(8.f, 0.f, 0.5f), fixtureStopping(), moving, valid));
	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(0.f, 0.f, 0.f), fixtureStopping(), at_rest, valid));

	// ahead the range is 10 - 8 * 0.5 - 0.375 = 5.625 m; at rest and age 0 it is the full 10 m
	EXPECT_LT(moving[0], at_rest[0]);
	EXPECT_NEAR(moving[0], sectorSpeedCap(1000U, kMinCm, kMaxCm, 8.f * 0.5f + allowance(0.5f), fixtureStopping()), 1e-5f);
	EXPECT_NEAR(at_rest[0], sectorSpeedCap(1000U, kMinCm, kMaxCm, 0.f, fixtureStopping()), 1e-6f);
	// behind: only the allowance is taken off
	EXPECT_NEAR(moving[36], sectorSpeedCap(1000U, kMinCm, kMaxCm, allowance(0.5f), fixtureStopping()), 1e-5f);
	EXPECT_LT(moving[0], moving[36]);
	EXPECT_TRUE(valid[0]);
}

TEST(RawSectorCapsTest, TheCapsFollowTheRotationByTheHeading)
{
	Map map(1000U);
	SectorCaps caps{};
	SectorFlags valid{};
	const float east = static_cast<float>(kPi / 2.0);

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(0.f, 8.f, 0.5f, east), fixtureStopping(), caps, valid));

	for (std::size_t i = 1; i < kSectorCount; ++i) {
		EXPECT_LE(caps[0], caps[i]) << i; // forward at this heading is the direction of travel
	}
}

TEST(RawSectorCapsTest, ALargeAgeTakesTheWholeRangeEvenOfAClearSector)
{
	Map map(kClearCm);
	SectorCaps caps{};
	SectorFlags valid{};

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(4.f, 0.f, 20.f), fixtureStopping(), caps, valid));
	EXPECT_NEAR(caps[0], 0.f, 1e-6f);   // 40 m of travel is more than the 40 m of range plus the margin
	EXPECT_NEAR(caps[36], 0.f, 1e-6f);  // the allowance alone is 600 m
	EXPECT_TRUE(valid[0]);              // known, but with no usable range
}

TEST(RawSectorCapsTest, AnAgeSoLargeThatTheDistanceOverflowsStillTakesTheWholeRange)
{
	// the stopping model reads a non-finite distance as 0, so the distance must stay a finite number
	Map map(kClearCm);
	SectorCaps caps{};
	SectorFlags valid{};

	const float huge = 1e30f;
	EXPECT_TRUE(std::isfinite(travelledTowardSector(inputs(4.f, 0.f, huge), 0)));
	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(4.f, 0.f, huge), fixtureStopping(), caps, valid));

	for (std::size_t i = 0; i < kSectorCount; ++i) {
		EXPECT_NEAR(caps[i], 0.f, 1e-9f) << i;
	}
}

TEST(RawSectorCapsTest, AZeroAgeAndAStationaryVehicleChangeNothing)
{
	Map map(1000U);
	SectorCaps caps{};
	SectorFlags valid{};

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(12.f, 0.f, 0.f), fixtureStopping(), caps, valid));

	for (std::size_t i = 0; i < kSectorCount; ++i) {
		EXPECT_NEAR(caps[i], sectorSpeedCap(1000U, kMinCm, kMaxCm, 0.f, fixtureStopping()), 1e-6f) << i;
	}
}

TEST(RawSectorCapsTest, TheWireValuesKeepTheirMeaning)
{
	Map map(1000U);
	map.wire[1] = kUnknown;
	map.wire[2] = 0U;
	map.wire[3] = kClearCm;
	SectorCaps caps{};
	SectorFlags valid{};

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(1.f, 0.f, 0.1f), fixtureStopping(), caps, valid));
	EXPECT_NEAR(caps[1], 0.f, 1e-9f);
	EXPECT_FALSE(valid[1]);
	EXPECT_NEAR(caps[2], 0.f, 1e-9f);
	EXPECT_TRUE(valid[2]);
	EXPECT_GT(caps[3], caps[0]); // clear is evaluated at the maximum range
	EXPECT_TRUE(valid[3]);
}

TEST(RawSectorCapsTest, AnUnusableInputMakesTheWholeMapUnknown)
{
	Map map(kClearCm); // every sector observed clear: the worst thing to believe
	TravelInputs bad_velocity = inputs(5.f, 0.f, 0.4f);
	bad_velocity.velocity_valid = false;
	TravelInputs no_heading = inputs(5.f, 0.f, 0.4f);
	no_heading.heading_valid = false;
	TravelInputs sentinel_bound = inputs(5.f, 0.f, 0.4f);
	sentinel_bound.decel_bound_m_s2 = -1.f;
	const TravelInputs cases[] = {bad_velocity, no_heading, sentinel_bound, inputs(kNaN, 0.f, 0.4f),
				      inputs(5.f, 0.f, -0.1f), inputs(5.f, 0.f, kNaN)};

	for (const TravelInputs &in : cases) {
		SectorCaps caps{};
		SectorFlags valid{};
		caps.fill(12.f);
		valid.fill(true);

		EXPECT_FALSE(rawSectorCaps(map.wire, kMinCm, kMaxCm, in, fixtureStopping(), caps, valid));

		for (std::size_t i = 0; i < kSectorCount; ++i) {
			EXPECT_NEAR(caps[i], 0.f, 1e-9f) << i;
			EXPECT_FALSE(valid[i]) << i;
		}
	}
}

TEST(RawSectorCapsTest, AHeadingResetInTheHistoryMakesTheMapUnknown)
{
	// the heading of the sample time comes from the history; a reset between the samples around it
	// gives no heading, and with it no map (ADR 0019 stays as it is)
	HeadingHistory history(HeadingHistoryParams{16, 0.1f});
	ASSERT_TRUE(history.add(1000000U, 0.f, 0U, true));
	ASSERT_TRUE(history.add(1008000U, 0.f, 1U, true)); // the reset counter changed here
	ASSERT_TRUE(history.add(1016000U, 0.f, 1U, true));

	Map map(kClearCm);
	SectorCaps caps{};
	SectorFlags valid{};

	const HeadingResult across = history.at(1004000U);
	ASSERT_FALSE(across.valid());
	TravelInputs in = inputs(5.f, 0.f, 0.4f);
	in.heading_valid = across.valid();
	in.heading_rad = across.heading_rad;
	EXPECT_FALSE(rawSectorCaps(map.wire, kMinCm, kMaxCm, in, fixtureStopping(), caps, valid));
	EXPECT_NEAR(caps[0], 0.f, 1e-9f);
	EXPECT_FALSE(valid[0]);

	// a time with a heading, after the reset, is used again
	const HeadingResult after = history.at(1012000U);
	ASSERT_TRUE(after.valid());
	in.heading_valid = after.valid();
	in.heading_rad = after.heading_rad;
	EXPECT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, in, fixtureStopping(), caps, valid));
	EXPECT_GT(caps[0], 0.f);
}

TEST(RawSectorCapsTest, AMapOutsideTheHistoryIsUnknownAsBefore)
{
	HeadingHistory history(HeadingHistoryParams{16, 0.1f});
	ASSERT_TRUE(history.add(1000000U, 0.f, 0U, true));
	ASSERT_TRUE(history.add(1008000U, 0.f, 0U, true));

	Map map(kClearCm);
	SectorCaps caps{};
	SectorFlags valid{};

	for (const uint64_t sample_us : {900000ULL, 1100000ULL}) { // before the first, after the last
		const HeadingResult heading = history.at(sample_us);
		ASSERT_FALSE(heading.valid());
		TravelInputs in = inputs(0.f, 0.f, 0.4f);
		in.heading_valid = heading.valid();
		in.heading_rad = heading.heading_rad;
		EXPECT_FALSE(rawSectorCaps(map.wire, kMinCm, kMaxCm, in, fixtureStopping(), caps, valid));
		EXPECT_NEAR(caps[0], 0.f, 1e-9f);
		EXPECT_FALSE(valid[0]);
	}
}

TEST(RawSectorCapsTest, AnInvalidInterfaceValueStaysZero)
{
	Map map(1000U);
	map.wire[0] = 5U; // below the minimum range, not a value of the interface
	map.wire[1] = 4500U;
	SectorCaps caps{};
	SectorFlags valid{};

	ASSERT_TRUE(rawSectorCaps(map.wire, kMinCm, kMaxCm, inputs(1.f, 0.f, 0.1f), fixtureStopping(), caps, valid));
	EXPECT_NEAR(caps[0], 0.f, 1e-9f);
	EXPECT_NEAR(caps[1], 0.f, 1e-9f);
}
