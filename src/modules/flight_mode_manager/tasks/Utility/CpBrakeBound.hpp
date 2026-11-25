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

/**
 * @file CpBrakeBound.hpp
 *
 * Feeds the cp_brake speed bound (lib/cp_brake GeneratorBound, ADR 0026) to a trajectory generator with the same
 * constraint message and parameters the hook in mc_pos_control uses, so the generator and the hook apply one rule.
 */

#pragma once

#include <GeneratorBound.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/cp_brake_constraints.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

class CpBrakeBound : public ModuleParams
{
public:
	explicit CpBrakeBound(ModuleParams *parent);
	~CpBrakeBound() = default;

	/**
	 * Speed bound along the unit direction dir (local NED) for a generator whose fastest speed is speed_max_m_s, heading
	 * yaw_rad and jerk limit jerk_max_m_s3. The current velocity the rule needs is the measured one, as in the hook.
	 */
	cp_brake::GeneratorBound bound(const matrix::Vector2f &dir, float speed_max_m_s, float yaw_rad, float jerk_max_m_s3);

private:
	// members, not locals: this runs deep in the flight-task call chain, on a small work-queue stack
	uORB::Subscription _constraints_sub{ORB_ID(cp_brake_constraints)};
	uORB::SubscriptionData<takeoff_status_s> _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_local_position_s> _local_position_sub{ORB_ID(vehicle_local_position)};

	cp_brake_constraints_s _constraints{};
	cp_brake::HookConstraint _constraint{}; ///< present stays false until a message arrives: the limiter then safe-stops

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CPB_EN>) _param_cpb_en,
		(ParamFloat<px4::params::CPB_CSTR_AGE>) _param_cpb_cstr_age,
		(ParamFloat<px4::params::CPB_V_SIDE>) _param_cpb_v_side,
		(ParamFloat<px4::params::CPB_REAR_ANG>) _param_cpb_rear_ang,
		(ParamFloat<px4::params::CPB_A_BRK>) _param_cpb_a_brk,
		(ParamFloat<px4::params::CPB_A_SIDE>) _param_cpb_a_side,
		(ParamFloat<px4::params::CPB_JERK>) _param_cpb_jerk,
		(ParamFloat<px4::params::CPB_T_LAT>) _param_cpb_t_lat,
		(ParamFloat<px4::params::CPB_MARGIN>) _param_cpb_margin,
		(ParamFloat<px4::params::CPB_MAX_V>) _param_cpb_max_v
	)
};
