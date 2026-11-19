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
 * @file CapLimiter.hpp
 *
 * Dwell and rise limiter of the cp_brake caps (ADR 0008, SYS-CP-013): a cap tightens at once, and
 * rises only after the sector has shown valid data supporting a higher cap for a continuous dwell
 * time, then at a bounded rate. Unknown, stale or invalid sector data resets the dwell. Pure logic,
 * no PX4 dependency; the dwell and the rise rate are inputs, not defaults.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace cp_brake
{

constexpr std::size_t kSectorCount{72};

/**
 * One value per sector. Not std::array: the NuttX builds of PX4 have no <array>, and this library
 * is built for them.
 */
template <typename T>
struct SectorArray {
	T values[kSectorCount];

	T &operator[](std::size_t i) { return values[i]; }
	const T &operator[](std::size_t i) const { return values[i]; }

	void fill(const T &value)
	{
		for (std::size_t i = 0; i < kSectorCount; ++i) {
			values[i] = value;
		}
	}
};

using SectorCaps = SectorArray<float>;
using SectorFlags = SectorArray<bool>;

struct LimiterParams {
	float dwell_s;      ///< continuous time of supporting data before a cap may rise [s], >= 0
	float rise_m_s2;    ///< highest rate of rise of a cap after the dwell [m/s^2], > 0
};

bool isValid(const LimiterParams &params);

/**
 * Time step [s] to give CapLimiter::update from the times of two accepted samples [us], or -1 when
 * there is no usable step: no previous sample, a time that did not advance, an age limit that is
 * not a positive number, or a gap longer than age_limit_s. A gap that long gave no supporting
 * data, so it must not count as dwell or as time to rise: the limiter then only tightens and its
 * dwell starts again.
 */
float sampleStep(uint64_t previous_us, uint64_t current_us, float age_limit_s);

class CapLimiter
{
public:
	/** Every cap starts at 0 (nothing is allowed until the data supports it). */
	explicit CapLimiter(const LimiterParams &params);

	/**
	 * raw: the cap each sector would have without any limiting, valid: whether each sector holds
	 * qualified (not unknown, stale or invalid) data, dt_s: time since the previous call.
	 * Returns the limited caps: never above raw, never below 0. With invalid parameters or an
	 * invalid raw cap or dt_s, a cap can only tighten (fail closed).
	 */
	const SectorCaps &update(const SectorCaps &raw, const SectorFlags &valid, float dt_s);

	const SectorCaps &caps() const { return _caps; }

private:
	LimiterParams _params;
	SectorCaps _caps{};
	float _dwell_s[kSectorCount] {};
};

} // namespace cp_brake
