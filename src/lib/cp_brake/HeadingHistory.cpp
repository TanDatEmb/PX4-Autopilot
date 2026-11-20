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

#include "HeadingHistory.hpp"

#include <cmath>
#include <math.h> // fmodf: the NuttX <cmath> has no std::fmod

#include "TimeUnits.hpp"

namespace cp_brake
{

namespace
{

constexpr float kPi{3.14159265358979323846f};
constexpr float kTwoPi{6.28318530717958647692f};

// the angle in (-pi, pi]; a value already in range is returned untouched
float wrapPi(float angle_rad)
{
	if (angle_rad > -kPi && angle_rad <= kPi) {
		return angle_rad;
	}

	const float turns = fmodf(angle_rad + kPi, kTwoPi);
	return (turns <= 0.f ? turns + kTwoPi : turns) - kPi;
}

} // namespace

bool isValid(const HeadingHistoryParams &params)
{
	return params.depth >= 2 && static_cast<std::size_t>(params.depth) <= kHeadingHistoryCapacity
	       && std::isfinite(params.max_gap_s) && params.max_gap_s > 0.f;
}

bool sameParams(const HeadingHistoryParams &a, const HeadingHistoryParams &b)
{
	// no == on floats (-Wfloat-equal), and a non-finite gap limit is never the same as anything
	return a.depth == b.depth && std::isfinite(a.max_gap_s) && std::isfinite(b.max_gap_s)
	       && !(a.max_gap_s < b.max_gap_s) && !(a.max_gap_s > b.max_gap_s);
}

HeadingHistory::HeadingHistory(const HeadingHistoryParams &params) : _params(params)
{
}

const HeadingHistory::Sample &HeadingHistory::newest(std::size_t i) const
{
	return _samples[(_next + kHeadingHistoryCapacity - 1U - i) % kHeadingHistoryCapacity];
}

void HeadingHistory::reset(const HeadingHistoryParams &params)
{
	_params = params;
	_next = 0U;
	_count = 0U;
}

bool HeadingHistory::add(uint64_t time_us, float heading_rad, uint8_t reset_counter, bool good_for_control)
{
	if (!isValid(_params) || time_us == 0U) {
		return false;
	}

	if (_count > 0U && time_us <= newest(0).time_us) {
		return false;
	}

	const bool finite = std::isfinite(heading_rad);

	_samples[_next] = Sample{time_us, finite ? wrapPi(heading_rad) : 0.f, reset_counter, good_for_control && finite};
	_next = (_next + 1U) % kHeadingHistoryCapacity;

	const std::size_t depth = static_cast<std::size_t>(_params.depth);

	if (_count < depth) {
		++_count;
	}

	return true;
}

HeadingResult HeadingHistory::at(uint64_t time_us) const
{
	if (!isValid(_params)) {
		return HeadingResult{HeadingLookup::kInvalidParams, 0.f};
	}

	if (_count == 0U) {
		return HeadingResult{HeadingLookup::kEmpty, 0.f};
	}

	if (time_us > newest(0).time_us) {
		return HeadingResult{HeadingLookup::kAfterLast, 0.f};
	}

	if (time_us < newest(_count - 1U).time_us) {
		return HeadingResult{HeadingLookup::kBeforeFirst, 0.f};
	}

	// the newest sample at or before the time: it exists, because the time is not before the oldest
	std::size_t i = 0U;

	while (newest(i).time_us > time_us) {
		++i;
	}

	const Sample &earlier = newest(i);

	if (earlier.time_us == time_us) {
		// the exact time of a sample needs no neighbour
		return earlier.good ? HeadingResult{HeadingLookup::kOk, earlier.heading_rad}
		       : HeadingResult{HeadingLookup::kNotGood, 0.f};
	}

	// the time is strictly inside: i > 0, because the newest sample is not before the time
	const Sample &later = newest(i - 1U);

	if (!earlier.good || !later.good) {
		return HeadingResult{HeadingLookup::kNotGood, 0.f};
	}

	if (earlier.reset_counter != later.reset_counter) {
		return HeadingResult{HeadingLookup::kReset, 0.f};
	}

	const uint64_t span_us = later.time_us - earlier.time_us;

	if (span_us > secondsToMicroseconds(_params.max_gap_s)) {
		return HeadingResult{HeadingLookup::kGap, 0.f};
	}

	// both differences are small integers, so the fraction keeps its precision at any absolute time
	const float fraction = static_cast<float>(time_us - earlier.time_us) / static_cast<float>(span_us);
	const float arc_rad = wrapPi(later.heading_rad - earlier.heading_rad);

	return HeadingResult{HeadingLookup::kOk, wrapPi(earlier.heading_rad + fraction * arc_rad)};
}

} // namespace cp_brake
