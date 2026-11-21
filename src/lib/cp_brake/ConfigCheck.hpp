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
 * @file ConfigCheck.hpp
 *
 * Start-up configuration check of cp_brake (ICD-CP-04). A parameter without an approved value
 * holds a sentinel outside its valid range, so it fails here and cp_brake reports an invalid
 * configuration instead of running on an unapproved number.
 */

#pragma once

#include <cstdint>

#include "CapLimiter.hpp"
#include "HeadingHistory.hpp"
#include "StoppingModel.hpp"
#include "TimeSync.hpp"

namespace cp_brake
{

struct ConfigInputs {
	StoppingParams stopping;          ///< CPB_MAX_V, CPB_A_BRK, CPB_JERK, CPB_T_LAT, CPB_MARGIN
	LimiterParams limiter;            ///< CPB_CLR_T, CPB_RISE
	float sample_age_stop_s;          ///< CPB_AGE_STOP [s], > 0
	float constraint_age_limit_s;     ///< staleness limit of the constraint [s], > 0 (OPEN-032)
	float stock_cp_dist_m;            ///< CP_DIST: the stock collision prevention is active when > 0
	float mpc_xy_vel_max_m_s;         ///< MPC_XY_VEL_MAX: must not exceed the model maximum speed
	bool second_obstacle_source;      ///< another obstacle_distance publisher is enabled (SIM_GZ_EN_LIDAR)
	HeadingHistoryParams heading_history;  ///< depth and gap limit of the heading history (OPEN-032)
	TimeSyncLimits time_sync_limits;  ///< limits of the time-alignment gate (OPEN-033)
	bool time_sync_enabled;           ///< UXRCE_DDS_SYNCT is on, so timesync_status exists
	bool time_sync_required;          ///< this build is a hardware one: ADR 0003 requires UXRCE_DDS_SYNCT=1
	float decel_bound_m_s2;           ///< CPB_DECEL_MAX [m/s^2]: bound of the horizontal deceleration (ADR 0020 D4), > 0, not below the braking acceleration
};

namespace ConfigFaults
{
constexpr uint32_t kStoppingParams{1U << 0};
constexpr uint32_t kLimiterParams{1U << 1};
constexpr uint32_t kSampleAge{1U << 2};
constexpr uint32_t kConstraintAge{1U << 3};
constexpr uint32_t kStockCollisionPreventionActive{1U << 4};
constexpr uint32_t kSpeedLimit{1U << 5};
constexpr uint32_t kSecondObstacleSource{1U << 6};
constexpr uint32_t kHeadingHistory{1U << 7};
constexpr uint32_t kTimeSyncLimits{1U << 8};
constexpr uint32_t kTimeSyncDisabled{1U << 9};
constexpr uint32_t kDecelBound{1U << 10};
} // namespace ConfigFaults

/** Bit set of ConfigFaults; 0 means the configuration is valid. */
uint32_t checkConfig(const ConfigInputs &inputs);

} // namespace cp_brake
