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
 * @file ChecksTest.cpp
 *
 * Unit tests of the cp_brake configuration and input checks. Parameter sets are test fixtures,
 * not approved platform values.
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "ConfigCheck.hpp"
#include "InputCheck.hpp"
#include "ReasonFlags.hpp"

using namespace cp_brake;

namespace
{
constexpr uint16_t kUnknown{std::numeric_limits<uint16_t>::max()};

ConfigInputs validConfig()
{
	ConfigInputs c{};
	c.stopping = StoppingParams{3.f, 4.f, 0.1f, 2.f, 12.f};
	c.limiter = LimiterParams{0.3f, 2.f};
	c.sample_age_stop_s = 0.5f;
	c.constraint_age_limit_s = 0.2f;
	c.stock_cp_dist_m = -1.f;
	c.mpc_xy_vel_max_m_s = 12.f;
	c.second_obstacle_source = false;
	c.heading_history = HeadingHistoryParams{16, 0.1f};
	c.time_sync_limits = TimeSyncLimits{0.25f, 0.125f, 0.5f};
	c.time_sync_enabled = true;
	c.time_sync_required = true;
	c.decel_bound_m_s2 = 3.f;
	return c;
}

struct Input {
	std::array<uint16_t, kInputSectorCount> distances{};
	ObstacleInput view{};

	Input()
	{
		distances.fill(kUnknown);
		view = ObstacleInput{kFrameBodyFrd, 5.f, 0.f, 10, 4000, distances.data(), distances.size()};
	}

	void refresh() { view.distances = distances.data(); }
};
} // namespace

TEST(ConfigCheckTest, AValidConfigurationHasNoFault)
{
	EXPECT_EQ(checkConfig(validConfig()), 0U);
}

TEST(ConfigCheckTest, ATbdSentinelIsAFault)
{
	ConfigInputs c = validConfig();
	c.stopping.t_lat_s = -1.f; // T_LAT has no approved value yet
	EXPECT_EQ(checkConfig(c) & ConfigFaults::kStoppingParams, ConfigFaults::kStoppingParams);

	c = validConfig();
	c.constraint_age_limit_s = -1.f; // OPEN-032
	EXPECT_EQ(checkConfig(c), ConfigFaults::kConstraintAge);
}

TEST(ConfigCheckTest, EachFaultHasItsOwnBit)
{
	ConfigInputs c = validConfig();
	c.limiter.rise_m_s2 = 0.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kLimiterParams);

	c = validConfig();
	c.sample_age_stop_s = 0.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kSampleAge);

	c = validConfig();
	c.stock_cp_dist_m = 1.f; // the stock collision prevention would act too
	EXPECT_EQ(checkConfig(c), ConfigFaults::kStockCollisionPreventionActive);

	c = validConfig();
	c.mpc_xy_vel_max_m_s = 12.5f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kSpeedLimit);

	c = validConfig();
	c.second_obstacle_source = true;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kSecondObstacleSource);
}

TEST(ConfigCheckTest, NonFiniteValuesAreFaults)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	ConfigInputs c = validConfig();
	c.stock_cp_dist_m = nan;
	EXPECT_NE(checkConfig(c) & ConfigFaults::kStockCollisionPreventionActive, 0U);

	c = validConfig();
	c.mpc_xy_vel_max_m_s = nan;
	EXPECT_NE(checkConfig(c) & ConfigFaults::kSpeedLimit, 0U);

	c = validConfig();
	c.sample_age_stop_s = nan;
	EXPECT_NE(checkConfig(c) & ConfigFaults::kSampleAge, 0U);
}

TEST(ConfigCheckTest, InvalidStoppingParametersAlsoFailTheSpeedLimitCheck)
{
	ConfigInputs c = validConfig();
	c.stopping.max_speed_m_s = 0.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kStoppingParams | ConfigFaults::kSpeedLimit);
}

TEST(ConfigCheckTest, TheHeadingHistorySentinelsAreAFault)
{
	// OPEN-032: the depth and the gap limit have no approved value, so they hold a sentinel
	ConfigInputs c = validConfig();
	c.heading_history.depth = -1;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kHeadingHistory);

	c = validConfig();
	c.heading_history.max_gap_s = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kHeadingHistory);

	c = validConfig();
	c.heading_history.depth = static_cast<int32_t>(kHeadingHistoryCapacity) + 1;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kHeadingHistory);
}

TEST(ConfigCheckTest, TheTimeSyncLimitSentinelsAreAFault)
{
	// OPEN-033: the three limits of the time-alignment gate have no approved value
	ConfigInputs c = validConfig();
	c.time_sync_limits.max_rtt_s = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kTimeSyncLimits);

	c = validConfig();
	c.time_sync_limits.max_offset_deviation_s = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kTimeSyncLimits);

	c = validConfig();
	c.time_sync_limits.max_status_age_s = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kTimeSyncLimits);
}

TEST(ConfigCheckTest, TheLimitsAreRequiredEvenWhereTheGateIsNotUsed)
{
	// the simulation profile has no timesync_status, but no CPB_ parameter may keep its sentinel
	ConfigInputs c = validConfig();
	c.time_sync_enabled = false;
	c.time_sync_required = false;
	c.time_sync_limits.max_rtt_s = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kTimeSyncLimits);
}

TEST(ConfigCheckTest, HardwareWithoutTheUxrceTimeSyncIsAFaultAndSimulationIsNot)
{
	// ADR 0003: hardware runs UXRCE_DDS_SYNCT=1, only simulation and replay run it at 0
	ConfigInputs c = validConfig();
	c.time_sync_enabled = false;
	c.time_sync_required = true;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kTimeSyncDisabled);

	c.time_sync_required = false;
	EXPECT_EQ(checkConfig(c), 0U);

	c.time_sync_enabled = true;
	c.time_sync_required = true;
	EXPECT_EQ(checkConfig(c), 0U);
}

TEST(ConfigCheckTest, TheDecelerationBoundSentinelIsAFault)
{
	// ADR 0020 D4: the upper bound of the horizontal deceleration has no approved value, so it holds a sentinel
	ConfigInputs c = validConfig();
	c.decel_bound_m_s2 = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);

	c = validConfig();
	c.decel_bound_m_s2 = 0.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);

	c = validConfig();
	c.decel_bound_m_s2 = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);

	c = validConfig();
	c.decel_bound_m_s2 = std::numeric_limits<float>::infinity();
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);
}

TEST(ConfigCheckTest, TheDecelerationBoundIsNotBelowTheBrakingAcceleration)
{
	// ADR 0020 D4: no smaller than the braking acceleration of the model; equal is allowed
	ConfigInputs c = validConfig();
	c.decel_bound_m_s2 = c.stopping.a_brake_m_s2;
	EXPECT_EQ(checkConfig(c), 0U);

	c.decel_bound_m_s2 = std::nextafter(c.stopping.a_brake_m_s2, 0.f);
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);
}

TEST(ConfigCheckTest, TheDecelerationBoundIsRequiredEvenWhereTheGateIsNotUsed)
{
	ConfigInputs c = validConfig();
	c.time_sync_enabled = false;
	c.time_sync_required = false;
	c.decel_bound_m_s2 = -1.f;
	EXPECT_EQ(checkConfig(c), ConfigFaults::kDecelBound);
}

TEST(InputCheckTest, AnInterfaceConformingInputIsValid)
{
	Input in;
	in.distances[0] = 0;
	in.distances[1] = 10;
	in.distances[2] = 1234;
	in.distances[3] = 4000;
	in.distances[4] = 4001;
	EXPECT_EQ(checkObstacleInput(in.view), 0U);
}

TEST(InputCheckTest, WrongFrameIncrementOffsetOrCountIsAFrameFault)
{
	Input in;
	in.view.frame = 0;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);

	in = Input();
	in.view.increment_deg = 10.f;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);

	in = Input();
	in.view.angle_offset_deg = 2.5f;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);

	in = Input();
	in.view.count = 71;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);

	in = Input();
	in.view.increment_deg = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);

	in = Input();
	in.view.distances = nullptr;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid);
}

TEST(InputCheckTest, WrongRangeMetadataOrSectorValueIsADataFault)
{
	Input in;
	in.view.min_cm = 0;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonDataInvalid);

	in = Input();
	in.view.max_cm = 5000;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonDataInvalid);

	for (const uint16_t bad : {uint16_t{5}, uint16_t{9}, uint16_t{4002}, uint16_t{60000}}) {
		in = Input();
		in.distances[71] = bad;
		in.refresh();
		EXPECT_EQ(checkObstacleInput(in.view), kReasonDataInvalid) << bad;
	}
}

TEST(InputCheckTest, FrameAndDataFaultsCanBeReportedTogether)
{
	Input in;
	in.view.frame = 1;
	in.view.min_cm = 0;
	EXPECT_EQ(checkObstacleInput(in.view), kReasonFrameInvalid | kReasonDataInvalid);
}
