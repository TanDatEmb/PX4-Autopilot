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

#include "InputCheck.hpp"

#include <cmath>
#include <cstdint>

#include "ReasonFlags.hpp"

namespace cp_brake
{

namespace
{
// the angles come from a float message field: compare with a tolerance, not exactly
constexpr float kAngleToleranceDeg{1e-3f};
}

uint32_t checkObstacleInput(const ObstacleInput &input)
{
	uint32_t flags = 0U;

	if (input.frame != kFrameBodyFrd || input.count != kInputSectorCount || input.distances == nullptr
	    || !std::isfinite(input.increment_deg) || std::fabs(input.increment_deg - kInputIncrementDeg) > kAngleToleranceDeg
	    || !std::isfinite(input.angle_offset_deg) || std::fabs(input.angle_offset_deg) > kAngleToleranceDeg) {
		flags |= kReasonFrameInvalid;
	}

	if (input.min_cm != kInputMinDistanceCm || input.max_cm != kInputMaxDistanceCm) {
		flags |= kReasonDataInvalid;
	}

	if ((flags & kReasonFrameInvalid) != 0U) {
		return flags; // the sectors cannot be read
	}

	constexpr uint16_t kUnknown{UINT16_MAX};

	for (std::size_t i = 0; i < input.count; ++i) {
		const uint16_t d = input.distances[i];
		const bool valid = d == 0U || d == kUnknown || d == static_cast<uint16_t>(kInputMaxDistanceCm + 1U)
				   || (d >= kInputMinDistanceCm && d <= kInputMaxDistanceCm);

		if (!valid) {
			flags |= kReasonDataInvalid;
			break;
		}
	}

	return flags;
}

} // namespace cp_brake
