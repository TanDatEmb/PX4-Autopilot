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

#include "ConfigCheck.hpp"

#include <cmath>

namespace cp_brake
{

uint32_t checkConfig(const ConfigInputs &inputs)
{
	uint32_t faults = 0U;

	if (!isValid(inputs.stopping)) {
		faults |= ConfigFaults::kStoppingParams;
	}

	if (!isValid(inputs.limiter)) {
		faults |= ConfigFaults::kLimiterParams;
	}

	if (!std::isfinite(inputs.sample_age_stop_s) || inputs.sample_age_stop_s <= 0.f) {
		faults |= ConfigFaults::kSampleAge;
	}

	if (!std::isfinite(inputs.constraint_age_limit_s) || inputs.constraint_age_limit_s <= 0.f) {
		faults |= ConfigFaults::kConstraintAge;
	}

	// the stock collision prevention acts when CP_DIST > 0; a non-finite value is not trusted
	if (!std::isfinite(inputs.stock_cp_dist_m) || inputs.stock_cp_dist_m > 0.f) {
		faults |= ConfigFaults::kStockCollisionPreventionActive;
	}

	if (!std::isfinite(inputs.mpc_xy_vel_max_m_s) || !isValid(inputs.stopping)
	    || inputs.mpc_xy_vel_max_m_s > inputs.stopping.max_speed_m_s) {
		faults |= ConfigFaults::kSpeedLimit;
	}

	if (inputs.second_obstacle_source) {
		faults |= ConfigFaults::kSecondObstacleSource;
	}

	return faults;
}

} // namespace cp_brake
