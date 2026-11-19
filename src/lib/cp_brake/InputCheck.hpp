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
 * @file InputCheck.hpp
 *
 * Validation of an ObstacleDistance input against ICD-CP-03 (frame BODY_FRD, 72 sectors of
 * 5 degrees, angle offset 0, minimum 10 cm, maximum 4000 cm, and only the values 0, 10 to 4000,
 * 4001 and UINT16_MAX in the sectors). Pure logic on plain values.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace cp_brake
{

constexpr uint8_t kFrameBodyFrd{12};
constexpr std::size_t kInputSectorCount{72};
constexpr float kInputIncrementDeg{5.f};
constexpr uint16_t kInputMinDistanceCm{10};
constexpr uint16_t kInputMaxDistanceCm{4000};

struct ObstacleInput {
	uint8_t frame;
	float increment_deg;
	float angle_offset_deg;
	uint16_t min_cm;
	uint16_t max_cm;
	const uint16_t *distances;
	std::size_t count;
};

/** ReasonFlags: kReasonFrameInvalid for the frame, increment, offset or sector count,
 *  kReasonDataInvalid for the range metadata or a sector value outside the interface; 0 when valid. */
uint32_t checkObstacleInput(const ObstacleInput &input);

} // namespace cp_brake
