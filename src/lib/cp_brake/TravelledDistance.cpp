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

#include "TravelledDistance.hpp"

#include <float.h>
#include <cmath>
#include <math.h>

namespace cp_brake
{

namespace
{
constexpr double kSectorRad{3.14159265358979323846 / 36.0};   // 5 degrees, clockwise from forward
}

bool isUsable(const TravelInputs &in)
{
	return in.velocity_valid && std::isfinite(in.v_north_m_s) && std::isfinite(in.v_east_m_s)
	       && in.heading_valid && std::isfinite(in.heading_rad)
	       && std::isfinite(in.age_s) && in.age_s >= 0.f
	       && std::isfinite(in.decel_bound_m_s2) && in.decel_bound_m_s2 > 0.f;
}

float travelledTowardSector(const TravelInputs &in, std::size_t sector)
{
	if (!isUsable(in) || sector >= kSectorCount) {
		return NAN;
	}

	// u_i in local NED: the bearing of the sector at the heading of the sample, clockwise from north
	const double bearing = static_cast<double>(in.heading_rad) + static_cast<double>(sector) * kSectorRad;
	const double toward = static_cast<double>(in.v_north_m_s) * cos(bearing)
			      + static_cast<double>(in.v_east_m_s) * sin(bearing);
	const double age = static_cast<double>(in.age_s);
	const double moved = (toward > 0.0 ? toward : 0.0) * age
			     + 0.5 * static_cast<double>(in.decel_bound_m_s2) * age * age;

	// a finite float that the model reads as "more than the whole range", never a value it would
	// read as 0 (sectorSpeedCap ignores a non-finite distance)
	return (moved > static_cast<double>(FLT_MAX)) ? FLT_MAX : static_cast<float>(moved);
}

bool rawSectorCaps(const uint16_t *wire_cm, uint16_t min_cm, uint16_t max_cm, const TravelInputs &inputs,
		   const StoppingParams &stopping, SectorCaps &caps, SectorFlags &valid)
{
	if (!isUsable(inputs)) {
		// no usable travelled distance: the map is not used, never read as "did not move"
		caps.fill(0.f);
		valid.fill(false);
		return false;
	}

	for (std::size_t i = 0; i < kSectorCount; ++i) {
		caps[i] = sectorSpeedCap(wire_cm[i], min_cm, max_cm, travelledTowardSector(inputs, i), stopping);
		valid[i] = (wire_cm[i] != UINT16_MAX);
	}

	return true;
}

} // namespace cp_brake
