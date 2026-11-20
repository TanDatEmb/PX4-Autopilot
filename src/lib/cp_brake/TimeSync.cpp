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

#include "TimeSync.hpp"

#include <cmath>

#include "TimeUnits.hpp"

namespace cp_brake
{

namespace
{

bool isLimit(float limit_s)
{
	return std::isfinite(limit_s) && limit_s > 0.f;
}

// |a - b| for any two int64 values: the unsigned difference is exact because it always fits 64 bits
uint64_t distance(int64_t a, int64_t b)
{
	return (a >= b) ? static_cast<uint64_t>(a) - static_cast<uint64_t>(b)
	       : static_cast<uint64_t>(b) - static_cast<uint64_t>(a);
}

} // namespace

bool isValid(const TimeSyncLimits &limits)
{
	return isLimit(limits.max_rtt_s) && isLimit(limits.max_offset_deviation_s) && isLimit(limits.max_status_age_s);
}

TimeSyncVerdict judgeTimeSync(const TimeSyncStatus &status, uint64_t now_us, const TimeSyncLimits &limits)
{
	if (!isValid(limits)) {
		return TimeSyncVerdict::kInvalidLimits;
	}

	if (status.timestamp_us == 0U) {
		return TimeSyncVerdict::kNoStatus;
	}

	// a status stamped after now cannot be trusted: the clock went back, or the status is not ours
	if (status.timestamp_us > now_us
	    || now_us - status.timestamp_us > secondsToMicroseconds(limits.max_status_age_s)) {
		return TimeSyncVerdict::kStale;
	}

	if (status.round_trip_time_us > secondsToMicroseconds(limits.max_rtt_s)) {
		return TimeSyncVerdict::kRoundTripTooLong;
	}

	if (distance(status.observed_offset_us, status.estimated_offset_us)
	    > secondsToMicroseconds(limits.max_offset_deviation_s)) {
		return TimeSyncVerdict::kOffsetDeviation;
	}

	return TimeSyncVerdict::kGood;
}

} // namespace cp_brake
