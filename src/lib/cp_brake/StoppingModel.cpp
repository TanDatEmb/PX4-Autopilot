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

#include "StoppingModel.hpp"

#include <cmath>
#include <cstdint>

namespace cp_brake
{

namespace
{
constexpr int kBisectionSteps{64};
}

bool isValid(const StoppingParams &params)
{
	return std::isfinite(params.a_brake_m_s2) && std::isfinite(params.jerk_m_s3)
	       && std::isfinite(params.t_lat_s) && std::isfinite(params.margin_m)
	       && std::isfinite(params.max_speed_m_s)
	       && params.a_brake_m_s2 > 0.f && params.jerk_m_s3 > 0.f && params.t_lat_s >= 0.f
	       && params.margin_m >= 0.f && params.max_speed_m_s > 0.f;
}

float brakingDistance(float speed_m_s, float a_brake_m_s2, float jerk_m_s3)
{
	if (!std::isfinite(speed_m_s) || !std::isfinite(a_brake_m_s2) || !std::isfinite(jerk_m_s3)
	    || speed_m_s <= 0.f || a_brake_m_s2 <= 0.f || jerk_m_s3 <= 0.f) {
		return 0.f;
	}

	const double v = speed_m_s;
	const double a = a_brake_m_s2;
	const double j = jerk_m_s3;

	if (v < a * a / j) {
		// the acceleration never reaches a: two jerk ramps of sqrt(v / j) each
		return static_cast<float>(v * std::sqrt(v / j));
	}

	// two jerk ramps and a constant-acceleration phase
	return static_cast<float>(v * v / (2.0 * a) + v * a / (2.0 * j));
}

float stoppingDistance(float speed_m_s, const StoppingParams &params)
{
	const float v = speed_m_s > 0.f ? speed_m_s : 0.f;
	return v * params.t_lat_s + brakingDistance(v, params.a_brake_m_s2, params.jerk_m_s3)
	       + params.margin_m;
}

float speedCap(float usable_range_m, const StoppingParams &params)
{
	if (!isValid(params) || !std::isfinite(usable_range_m) || usable_range_m <= params.margin_m) {
		return 0.f;
	}

	if (stoppingDistance(params.max_speed_m_s, params) <= usable_range_m) {
		return params.max_speed_m_s;
	}

	float low = 0.f;
	float high = params.max_speed_m_s;

	for (int i = 0; i < kBisectionSteps; ++i) {
		const float mid = 0.5f * (low + high);

		if (stoppingDistance(mid, params) <= usable_range_m) {
			low = mid;

		} else {
			high = mid;
		}
	}

	return low;
}

float sectorSpeedCap(uint16_t wire_cm, uint16_t min_cm, uint16_t max_cm, float travelled_toward_m,
		     const StoppingParams &params)
{
	constexpr uint16_t kUnknown{UINT16_MAX};
	constexpr float kCentimetresPerMetre{100.f};

	// the clear marker max_cm + 1 must stay distinct from 0 (too close) and UINT16_MAX (unknown)
	if (wire_cm == 0U || wire_cm == kUnknown || min_cm >= max_cm || max_cm > kUnknown - 2U) {
		return 0.f;
	}

	uint16_t range_cm{0U};

	if (wire_cm == static_cast<uint16_t>(max_cm + 1U)) {
		range_cm = max_cm; // observed clear: evaluated at the maximum range

	} else if (wire_cm >= min_cm && wire_cm <= max_cm) {
		range_cm = wire_cm;

	} else {
		return 0.f; // not a value of the interface
	}

	const float travelled = (std::isfinite(travelled_toward_m) && travelled_toward_m > 0.f) ? travelled_toward_m : 0.f;
	return speedCap(static_cast<float>(range_cm) / kCentimetresPerMetre - travelled, params);
}

} // namespace cp_brake
