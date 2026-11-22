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
 * @file WarmUpBound.hpp
 *
 * Bounds the heading warm-up of ADR 0020 D5 (issue #112). The warm-up exempts a map from the NO_HEADING safe
 * stop while the heading became good for control only after the map was sampled. Incoming maps keep refreshing
 * the sample timestamp, so the stale rule does not end a warm-up that never completes: this bound does, with
 * the stop age CPB_AGE_STOP counted from the start of the warm-up episode.
 */

#pragma once

#include <cstdint>

namespace cp_brake
{

class WarmUpBound
{
public:
	/**
	 * Call once per evaluation. warming: the heading lookup is a warm-up now. Returns whether the exemption
	 * still holds: true from the first warming evaluation until limit_s has passed. It is false when not
	 * warming, when the limit is not a finite positive number, when the time went backwards, and after the
	 * limit; a not-warming evaluation ends the episode, so the next warm-up starts a new one.
	 */
	bool update(bool warming, uint64_t now_us, float limit_s);

	void reset();

private:
	bool _active{false};
	uint64_t _since_us{0};
};

} // namespace cp_brake
