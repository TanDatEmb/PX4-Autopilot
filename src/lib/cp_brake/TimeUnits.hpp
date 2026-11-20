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
 * @file TimeUnits.hpp
 *
 * Seconds to microseconds for the time limits of cp_brake. Integer microseconds are used to
 * compare times, because a float cannot tell two PX4 times a few ms apart once a clock counts
 * 1e15 us (a clock set to UTC).
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace cp_brake
{

/**
 * Truncates to a whole microsecond and saturates at UINT64_MAX. The caller has checked that the
 * limit is finite and positive; anything else gives 0, so a comparison against it fails closed.
 */
inline uint64_t secondsToMicroseconds(float seconds)
{
	if (!std::isfinite(seconds) || seconds <= 0.f) {
		return 0U;
	}

	const float us = seconds * 1e6f;

	// 2^64 as a float is exactly 18446744073709551616; the largest float below it fits a uint64
	return (us >= 18446744073709551616.f) ? UINT64_MAX : static_cast<uint64_t>(us);
}

} // namespace cp_brake
