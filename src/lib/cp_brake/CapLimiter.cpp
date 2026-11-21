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

#include "CapLimiter.hpp"

#include <cmath>

namespace cp_brake
{

bool isValid(const LimiterParams &params)
{
	return std::isfinite(params.dwell_s) && std::isfinite(params.rise_m_s2)
	       && params.dwell_s >= 0.f && params.rise_m_s2 > 0.f;
}

float sampleStep(uint64_t previous_us, uint64_t current_us, float age_limit_s)
{
	if (previous_us == 0U || current_us <= previous_us || !std::isfinite(age_limit_s) || age_limit_s <= 0.f) {
		return -1.f;
	}

	const float step_s = static_cast<float>(current_us - previous_us) * 1e-6f;

	return (step_s > age_limit_s) ? -1.f : step_s;
}

CapLimiter::CapLimiter(const LimiterParams &params) : _params(params)
{
}

const SectorCaps &CapLimiter::update(const SectorCaps &raw, const SectorFlags &valid, float dt_s)
{
	const bool may_rise = isValid(_params) && std::isfinite(dt_s) && dt_s > 0.f;

	for (std::size_t i = 0; i < kSectorCount; ++i) {
		// an invalid raw cap counts as 0: the sector is not allowed anything
		const float target = (std::isfinite(raw[i]) && raw[i] > 0.f) ? raw[i] : 0.f;

		if (target <= _caps[i]) {
			// tighten at once; supporting data for a higher cap is over, so the dwell starts again
			_caps[i] = target;
			_dwell_s[i] = 0.f;
			continue;
		}

		if (!valid[i] || !may_rise) {
			_dwell_s[i] = 0.f;
			continue;
		}

		_dwell_s[i] += dt_s;

		if (_dwell_s[i] >= _params.dwell_s) {
			const float raised = _caps[i] + _params.rise_m_s2 * dt_s;
			_caps[i] = raised < target ? raised : target;
		}
	}

	return _caps;
}

const SectorCaps &CapLimiter::tighten(const SectorCaps &raw)
{
	for (std::size_t i = 0; i < kSectorCount; ++i) {
		const float target = (std::isfinite(raw[i]) && raw[i] > 0.f) ? raw[i] : 0.f;

		if (target < _caps[i]) {
			_caps[i] = target;
			_dwell_s[i] = 0.f;
		}
	}

	return _caps;
}

} // namespace cp_brake
