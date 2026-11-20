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
 * @file MapTimeline.hpp
 *
 * Times of the maps of cp_brake (ICD-CP-04, ADR 0019): which map is the current sample, and which
 * time step the dwell and rise limiter may credit. Pure logic, no PX4 type, no clock.
 *
 * Two times are kept apart. The sample is the latest map that was used: it gives the age for the
 * stale rule and the time of the heading lookup. A map that failed the time-alignment gate is not
 * used (every sector unknown, caps 0), so it does not refresh the sample. The limiter step is
 * different: the limiter was fed that unused map, so the time before the first used map after it
 * is not supporting data. Crediting it would let a cap rise at once after an outage that was shorter
 * than the sample age limit.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

class MapTimeline
{
public:
	/**
	 * A map that passed the input check, stamped timestamp_us (PX4 time). used: it also passed the
	 * time-alignment gate, so it becomes the current sample. Returns the step [s] to give
	 * CapLimiter::update, or -1 for none (the limiter then only tightens and its dwell starts
	 * again): the first map, a gap longer than age_limit_s, an unusable age limit, or the first
	 * used map after one that was not used.
	 */
	float onMap(uint64_t timestamp_us, bool used, float age_limit_s);

	/** Time of the latest used map, 0 before the first. */
	uint64_t sampleTimestamp() const { return _sample_us; }

private:
	uint64_t _previous_us{0U};      ///< latest map that passed the input check, used or not
	bool _previous_used{false};     ///< whether that map was used
	uint64_t _sample_us{0U};        ///< latest used map
};

} // namespace cp_brake
