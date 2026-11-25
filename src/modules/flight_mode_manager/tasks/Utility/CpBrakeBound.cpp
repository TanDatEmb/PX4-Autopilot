/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include "CpBrakeBound.hpp"

#include <drivers/drv_hrt.h>

CpBrakeBound::CpBrakeBound(ModuleParams *parent) :
	ModuleParams(parent)
{
}

cp_brake::GeneratorBound CpBrakeBound::bound(const matrix::Vector2f &dir, const matrix::Vector2f &vel_m_s,
		const float speed_max_m_s, const float yaw_rad)
{
	// the copy is field by field, as MulticopterPositionControl::updateCollisionLimiter does for the hook
	if (_constraints_sub.update(&_constraints)) {
		_constraint.present = true;
		_constraint.timestamp_us = _constraints.timestamp;
		_constraint.heading_rad = _constraints.heading_rad;
		_constraint.heading_valid = _constraints.heading_valid;

		for (std::size_t sector = 0; sector < cp_brake::kSectorCount; ++sector) {
			_constraint.caps_m_s[sector] = _constraints.caps_m_s[sector];
			_constraint.distances_m[sector] = _constraints.distances_m[sector];
		}

		_constraint.clearance_m = _constraints.clearance_m;
		_constraint.enforcement_active = _constraints.enforcement_active;
		_constraint.valid = _constraints.valid;
		_constraint.state = _constraints.state;
		_constraint.reason_flags = _constraints.reason_flags;
	}

	takeoff_status_s takeoff_status;

	if (_takeoff_status_sub.update(&takeoff_status)) {
		_in_flight = (takeoff_status.takeoff_state == takeoff_status_s::TAKEOFF_STATE_FLIGHT);
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_nav_state = vehicle_status.nav_state;
	}

	if (!_constraint.present) {
		return cp_brake::GeneratorBound{false, speed_max_m_s, 0.f, 0.f};
	}

	// the enforcement scope of the hook: Position and Mission (owner, 2026-11-19)
	const bool nav_state_enforced = (_nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL)
					|| (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	const cp_brake::HookContext context{_param_cpb_en.get() == 1, _in_flight, nav_state_enforced, hrt_absolute_time(),
					    _param_cpb_cstr_age.get(), yaw_rad, _param_cpb_v_side.get(), _param_cpb_rear_ang.get()};
	const cp_brake::StoppingParams forward{_param_cpb_a_brk.get(), _param_cpb_jerk.get(), _param_cpb_t_lat.get(),
					       _param_cpb_margin.get(), _param_cpb_max_v.get()};
	cp_brake::StoppingParams side = forward;
	side.a_brake_m_s2 = _param_cpb_a_side.get(); // the side class is the same model with its own braking (SYS-CP-028)

	const float vel[2] = {vel_m_s(0), vel_m_s(1)};
	const float direction[2] = {dir(0), dir(1)};
	return cp_brake::generatorSpeedBound(context, _constraint, vel, direction, speed_max_m_s, forward, side);
}
