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
 * @file TravelledDistance.hpp
 *
 * Distance moved toward a sector since the sample time of the map (ICD-CP-04, ADR 0020 D4, issue
 * #115), and the raw caps of one map with that distance taken off the usable range. Pure logic on
 * plain values: no PX4 type, no clock, no default value.
 *
 * The distance toward sector i is the present velocity toward it times the age of the map, plus an
 * allowance for a vehicle that slowed down during that age:
 *
 *     max(0, v_now . u_i) * age + 0.5 * a_bound * age^2
 *
 * v_now is the horizontal velocity of the estimator (north, east), u_i the unit vector of sector i
 * at the heading of the sample time, age = now - sample time, and a_bound an upper bound of the
 * horizontal deceleration. The allowance keeps the estimate on the safe side, so the clearance
 * margin of the stopping model is never spent on this error. No position history is kept.
 *
 * An input that is not usable (an invalid or non-finite velocity, no heading for the sample time,
 * an age that is not a non-negative number, a bound that is not a positive number) makes the whole
 * map unknown: every sector gets cap 0 and is not valid. It is never read as "did not move".
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "CapLimiter.hpp"
#include "StoppingModel.hpp"

namespace cp_brake
{

struct TravelInputs {
	bool velocity_valid;       ///< the horizontal velocity of the estimator is valid (v_xy_valid)
	float v_north_m_s;         ///< horizontal velocity, local NED north [m/s]
	float v_east_m_s;          ///< horizontal velocity, local NED east [m/s]
	bool heading_valid;        ///< the heading history gives a heading for the sample time
	float heading_rad;         ///< heading of the sample time, FRD to NED yaw [rad]
	float age_s;               ///< now minus the sample time [s]
	float decel_bound_m_s2;    ///< upper bound of the horizontal deceleration (CPB_DECEL_MAX) [m/s^2], > 0
};

/** True when every input is usable (see the file comment); false means the map is unknown. */
bool isUsable(const TravelInputs &inputs);

/**
 * Distance moved toward sector (0 forward, clockwise, 5 degrees each) since the sample [m], >= 0.
 * NaN when the inputs are not usable or the sector does not exist: the caller must test isUsable
 * first, because sectorSpeedCap reads a non-finite distance as 0.
 */
float travelledTowardSector(const TravelInputs &inputs, std::size_t sector);

/**
 * Raw caps and validity of the 72 sectors of one map. wire_cm points at kSectorCount values of the
 * ObstacleDistance of ICD-CP-03. Returns true when the inputs were usable; false when the whole map
 * is unknown (caps 0, no sector valid). A sector is valid when its wire value is not UINT16_MAX.
 *
 * Direction classes (SYS-CP-028, SYS-CP-031 b): a sector whose centre is within kForwardClassDeg of
 * the nose uses `forward`, every other sector uses `side`. A sector whose centre is more than
 * unclaimed_start_deg from the nose is outside the claim: when it has no evidence (UINT16_MAX) its cap
 * is unclaimed_cap_m_s and it counts as valid, so the limiter may raise it; an observed range there is
 * capped by the side model as everywhere else. An unclaimed_start_deg of 180 or more leaves no sector
 * outside the claim.
 */
bool rawSectorCaps(const uint16_t *wire_cm, uint16_t min_cm, uint16_t max_cm, const TravelInputs &inputs,
		   const StoppingParams &forward, const StoppingParams &side, float unclaimed_start_deg,
		   float unclaimed_cap_m_s, SectorCaps &caps, SectorFlags &valid);

} // namespace cp_brake
