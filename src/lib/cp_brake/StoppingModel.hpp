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
 * @file StoppingModel.hpp
 *
 * Pure stopping model of cp_brake (ICD-CP-04, cap law): the speed cap of a sector is the largest
 * speed whose stopping distance fits the usable range. Stopping distance = v * t_lat + jerk-limited
 * braking distance from v with a_brake and jerk + margin. No PX4 dependency, no default value:
 * every parameter is an input.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

struct StoppingParams {
	float a_brake_m_s2;    ///< braking acceleration used by the model [m/s^2], > 0
	float jerk_m_s3;       ///< braking jerk used by the model [m/s^3], > 0
	float t_lat_s;         ///< lag from the hook output to the vehicle response [s], >= 0
	float margin_m;        ///< clearance margin [m], >= 0
	float max_speed_m_s;   ///< upper bound of any cap [m/s], > 0
};

/** True when every parameter is finite and in its valid range. */
bool isValid(const StoppingParams &params);

/** Jerk-limited braking distance from speed to rest [m]; 0 for a non-positive or invalid input. */
float brakingDistance(float speed_m_s, float a_brake_m_s2, float jerk_m_s3);

/** Stopping distance at speed [m]; strictly increasing in speed for valid parameters. */
float stoppingDistance(float speed_m_s, const StoppingParams &params);

/** Largest speed in [0, max_speed] whose stopping distance fits usable_range_m; 0 when the
 *  parameters or the range are invalid (fail closed). */
float speedCap(float usable_range_m, const StoppingParams &params);

/**
 * Cap of one sector from its ObstacleDistance wire value (ICD-CP-03).
 * 0 (too close) and UINT16_MAX (unknown) give 0, whatever min_cm is (a sensor may report 0); max_cm + 1 (clear) is evaluated at max_cm;
 * a value in [min_cm, max_cm] is an occupied range in centimetres; anything else is invalid and
 * gives 0. travelled_toward_m is the distance moved toward the sector since the sample time; a
 * negative or non-finite value is treated as 0, so moving away never extends the range.
 */
float sectorSpeedCap(uint16_t wire_cm, uint16_t min_cm, uint16_t max_cm, float travelled_toward_m,
		     const StoppingParams &params);

} // namespace cp_brake
