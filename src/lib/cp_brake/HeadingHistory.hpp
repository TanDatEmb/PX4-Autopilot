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
 * @file HeadingHistory.hpp
 *
 * Timestamped history of the PX4 heading (ICD-CP-04, ADR 0019, issue #94). The caps of cp_brake are
 * body FRD at the heading of the sample time, not at the current heading, so the module keeps the
 * heading of the estimator with the time of each sample and interpolates it at the map timestamp
 * (PX4 time). Pure logic on plain values: no PX4 type, no clock, no default value.
 *
 * - Samples are keyed by their sample time (timestamp_sample). A time that does not advance is
 *   rejected, and 0 means no time.
 * - The value at a time between two samples is the linear interpolation along the short arc, so
 *   the +-180 degree cut is crossed correctly. A time outside the kept samples is never
 *   extrapolated: there is no heading for it.
 * - A change of the reset counter between the two samples around the time invalidates the
 *   interpolation (the heading jumped or was redefined). The exact time of a sample is always
 *   known, so it does not need its neighbour.
 * - A sample that was not good for control, or whose heading is not finite, gives no heading
 *   at its own time and in the two intervals next to it.
 * - Two samples further apart than the gap limit are not interpolated across.
 * - The depth and the gap limit have no approved value (OPEN-032): the caller gives them, and a
 *   sentinel outside the valid range makes every lookup fail instead of running on a guess.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace cp_brake
{

/**
 * Storage bound of the history, not a limit of the design: the number of samples the module can
 * hold. It is sized from the rate measured in Gazebo SITL for vehicle_local_position (125 Hz, so
 * 128 samples reach back about 1 s). The depth actually used is a parameter (OPEN-032).
 */
constexpr std::size_t kHeadingHistoryCapacity{128};

struct HeadingHistoryParams {
	int32_t depth;      ///< newest samples kept, 2 to kHeadingHistoryCapacity (TBD, OPEN-032)
	float max_gap_s;    ///< longest span between the two samples of an interpolation [s], > 0 (TBD, OPEN-032)
};

/** True when the depth is in its range and the gap limit is a finite positive number. */
bool isValid(const HeadingHistoryParams &params);

/**
 * True when both sets are the same usable values: equal depth and equal finite gap limits. A
 * non-finite gap limit is never the same as anything, itself included, so a parameter that was
 * invalid and is corrected always counts as a change (and an invalid one resets the history again,
 * which costs nothing: it holds no sample).
 */
bool sameParams(const HeadingHistoryParams &a, const HeadingHistoryParams &b);

/** Why a lookup gave a heading or not. The values equal the HEADING_LOOKUP_* constants of CpBrakeStatus. */
enum class HeadingLookup : uint8_t {
	kOk = 0,
	kInvalidParams = 1,  ///< the depth or the gap limit is outside its range (a TBD sentinel)
	kEmpty = 2,          ///< no sample yet
	kBeforeFirst = 3,    ///< the time is older than the oldest kept sample
	kAfterLast = 4,      ///< the time is newer than the newest sample: no extrapolation
	kReset = 5,          ///< the reset counter changed between the samples around the time
	kGap = 6,            ///< the samples around the time are further apart than the gap limit
	kNotGood = 7,        ///< a sample at or around the time was not good for control, or not finite
	kNotYetGood = 8      ///< as kNotGood, but the heading has been good for control since after the time: a warm-up
};

struct HeadingResult {
	HeadingLookup lookup;
	float heading_rad;   ///< (-pi, pi], FRD to NED yaw; 0 unless lookup is kOk

	bool valid() const { return lookup == HeadingLookup::kOk; }
};

class HeadingHistory
{
public:
	explicit HeadingHistory(const HeadingHistoryParams &params);

	/**
	 * Adds one sample: time_us is its sample time, heading_rad the heading at that time,
	 * reset_counter the heading reset counter of the estimator, good_for_control its validity.
	 * Returns false, and changes nothing, when the parameters are invalid, the time is 0, or it
	 * does not advance beyond the newest sample. A non-finite heading is kept as not good.
	 */
	bool add(uint64_t time_us, float heading_rad, uint8_t reset_counter, bool good_for_control);

	/** The heading at time_us (the map timestamp, PX4 time), or the reason there is none. */
	HeadingResult at(uint64_t time_us) const;

	/**
	 * Empties the history and takes new parameters. It does not build a second history, so it fits
	 * the small stack of a NuttX task (a HeadingHistory is about 2 KB).
	 */
	void reset(const HeadingHistoryParams &params);

	std::size_t size() const { return _count; }
	const HeadingHistoryParams &params() const { return _params; }

private:
	struct Sample {
		uint64_t time_us;
		float heading_rad;
		uint8_t reset_counter;
		bool good;
	};

	/** The i-th newest sample, 0 being the newest. */
	const Sample &newest(std::size_t i) const;

	/** kNotYetGood when the newest samples are good and the time precedes their run (the heading became good
	 *  after the time: a warm-up, ADR 0020 D5), else kNotGood. */
	HeadingResult notGood(uint64_t time_us) const;

	HeadingHistoryParams _params;
	Sample _samples[kHeadingHistoryCapacity] {};
	std::size_t _next{0};     ///< index the next sample goes to
	std::size_t _count{0};
};

} // namespace cp_brake
