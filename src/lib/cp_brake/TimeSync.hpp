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
 * @file TimeSync.hpp
 *
 * Quality gate of the time alignment of a map (ICD-CP-04, ADR 0019, issue #94). The map timestamp
 * reaches PX4 converted from the companion clock by the uXRCE-DDS time offset. The message
 * timesync_status has no convergence flag, so the gate judges what it carries: how old the status
 * is, the round-trip time of the exchange (which bounds the error of one offset sample), and how
 * far the offset observed in that exchange is from the smoothed estimate (a jitter or a time
 * jump). A map whose alignment is not good is not used: its sectors are unknown, never free.
 *
 * Pure logic on plain values, no PX4 type and no default: the three limits have no approved value
 * (OPEN-033, measured at G5), so a sentinel outside the valid range makes every status fail.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

struct TimeSyncLimits {
	float max_rtt_s;               ///< longest round-trip time of a good exchange [s], > 0 (TBD, OPEN-033)
	float max_offset_deviation_s;  ///< largest |observed - estimated offset| of a good exchange [s], > 0 (TBD, OPEN-033)
	float max_status_age_s;        ///< oldest status still trusted [s], > 0 (TBD, OPEN-033)
};

/** True when the three limits are finite positive numbers. */
bool isValid(const TimeSyncLimits &limits);

/** The fields of timesync_status that the gate reads. */
struct TimeSyncStatus {
	uint64_t timestamp_us;         ///< PX4 time the status was published; 0 when none was received
	int64_t observed_offset_us;    ///< offset seen in this exchange
	int64_t estimated_offset_us;   ///< smoothed offset used for the conversion
	uint32_t round_trip_time_us;   ///< round-trip time of this exchange
};

/** The values equal the TIME_SYNC_* constants of CpBrakeStatus. */
enum class TimeSyncVerdict : uint8_t {
	kGood = 0,
	kInvalidLimits = 1,       ///< a limit is outside its range (a TBD sentinel)
	kNoStatus = 2,            ///< no timesync_status was received
	kStale = 3,               ///< older than the status age limit, or stamped in the future
	kRoundTripTooLong = 4,
	kOffsetDeviation = 5
};

/**
 * Judges the status at now_us (PX4 time). Each limit is inclusive: a value equal to its limit is
 * good. The checks run in the order of the enumeration, so the verdict names the first failure.
 * Only the deviation of the offset is judged, never the offset itself, which is any large number.
 */
TimeSyncVerdict judgeTimeSync(const TimeSyncStatus &status, uint64_t now_us, const TimeSyncLimits &limits);

} // namespace cp_brake
